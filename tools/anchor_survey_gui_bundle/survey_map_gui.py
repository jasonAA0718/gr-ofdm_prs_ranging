#!/usr/bin/env python3
"""GNU Radio survey GUI: LLH/ENU, map tiles and live position CSV debug tracks.

Requires Python 3 + tkinter. Place beside survey_llh_to_enu.py and
anchor_csv_loader.py. No third-party pip packages required.
"""
from __future__ import annotations

import csv
from collections import deque
import io
import math
import os
from pathlib import Path
import queue
import sqlite3
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import urllib.error
import urllib.request

from survey_llh_to_enu import llh_to_ecef, ecef_to_enu
from position_csv_reader import PositionCsvReader, MAX_TRAIL_POINTS, enu_to_latlon

APP_TITLE = "GNSS / SDR 站點 Survey · LLH → ENU"
INPUT_HEADERS = ["anchor_id", "latitude_deg", "longitude_deg", "ground_elevation_m", "antenna_agl_m"]
OUTPUT_HEADERS = ["anchor_id", "east_m", "north_m", "up_m", "latitude_deg", "longitude_deg",
                  "ground_elevation_m", "antenna_agl_m"]
CACHE_DIR = Path.home() / ".cache" / "prs_anchor_survey" / "osm"
OSM_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
USER_AGENT = "PRSAnchorSurvey/1.0 (educational SDR field survey; normal viewport-only requests)"
TILE_PX = 256
MAX_LAT = 85.05112878


def latlon_to_world(lat, lon, zoom):
    lat = max(-MAX_LAT, min(MAX_LAT, lat))
    n = 2 ** zoom * TILE_PX
    x = (lon + 180.0) / 360.0 * n
    r = math.radians(lat)
    y = (1.0 - math.asinh(math.tan(r)) / math.pi) / 2.0 * n
    return x, y


def world_to_latlon(x, y, zoom):
    n = 2 ** zoom * TILE_PX
    lon = x / n * 360.0 - 180.0
    lat = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / n))))
    return lat, lon


def decimal_to_dms(value, latitude):
    direction = ("N" if value >= 0 else "S") if latitude else ("E" if value >= 0 else "W")
    val = abs(value)
    degrees = int(val)
    minutes = int((val - degrees) * 60)
    seconds = (val - degrees - minutes / 60) * 3600
    return str(degrees), str(minutes), f"{seconds:.5f}", direction


def dms_to_decimal(degrees, minutes, seconds, hemisphere, latitude):
    deg, mins, secs = float(degrees), float(minutes), float(seconds)
    if not all(math.isfinite(n) for n in (deg, mins, secs)) or deg < 0 or not (0 <= mins < 60) or not (0 <= secs < 60):
        raise ValueError("DMS 需使用非負的度數，以及 0–59 的分與秒")
    bound = 90 if latitude else 180
    val = deg + mins / 60 + secs / 3600
    if val > bound:
        raise ValueError(f"座標超過 {bound}°")
    return val * (-1 if hemisphere in ("S", "W") else 1)


def parse_finite(text, label):
    try:
        val = float(text)
    except ValueError as exc:
        raise ValueError(f"{label} 必須是數字") from exc
    if not math.isfinite(val):
        raise ValueError(f"{label} 不可為 NaN 或 Inf")
    return val


def sorted_sites(sites):
    anchors = sorted((s for s in sites if s["id"] != "initiator"), key=lambda s: int(s["id"]))
    initiators = [s for s in sites if s["id"] == "initiator"]
    return anchors + initiators


def validate_anchor_ids(sites):
    labels = [s["id"] for s in sites]
    if len(labels) != len(set(labels)):
        raise ValueError("站點 ID 重複")
    anchors = sorted(int(x) for x in labels if x != "initiator")
    if not anchors:
        raise ValueError("至少需要一個 Anchor")
    if anchors != list(range(len(anchors))):
        raise ValueError("Anchor ID 必須由 0 開始且連續 (0, 1, 2, ...)")
    if len(anchors) > 8:
        raise ValueError("目前 Position Solver 最多支援 8 個 Anchors")
    if "initiator" not in labels:
        raise ValueError("請新增一筆 initiator 才能輸出 vehicle_height_m")


def convert_sites_to_enu(sites, geoid=0.0):
    validate_anchor_ids(sites)
    sites = sorted_sites(sites)
    origin = next(s for s in sites if s["id"] == 0)
    origin_h = origin["ground"] + origin["agl"] + geoid
    xyz0 = llh_to_ecef(origin["lat"], origin["lon"], origin_h)
    rows = []
    for s in sites:
        h = s["ground"] + s["agl"] + geoid
        xyz = llh_to_ecef(s["lat"], s["lon"], h)
        e, n, u = ecef_to_enu(xyz, xyz0, origin["lat"], origin["lon"])
        rows.append({"anchor_id": s["id"], "east_m": e, "north_m": n, "up_m": u,
                     "latitude_deg": s["lat"], "longitude_deg": s["lon"],
                     "ground_elevation_m": s["ground"], "antenna_agl_m": s["agl"]})
    return rows


class TileProvider:
    def __init__(self):
        self.mbtiles = None
        self.conn = None
        self.scheme = "tms"
        self.attribution = ""
        self.lock = threading.Lock()

    def set_mbtiles(self, filename):
        conn = sqlite3.connect(f"file:{Path(filename).resolve()}?mode=ro", uri=True, check_same_thread=False)
        tables = {row[0] for row in conn.execute("SELECT name FROM sqlite_master WHERE type='table'")}
        if "tiles" not in tables:
            conn.close()
            raise ValueError("不是標準 raster MBTiles：找不到 tiles 表")
        meta = dict(conn.execute("SELECT name, value FROM metadata")) if "metadata" in tables else {}
        if meta.get("format", "png").lower() != "png":
            conn.close()
            raise ValueError("目前純標準函式庫版本僅支援 PNG raster MBTiles，不支援 JPEG/pbf/vector")
        # MBTiles spec uses TMS tile_row; some nonstandard archives use XYZ.
        self.scheme = meta.get("scheme", "tms").lower()
        with self.lock:
            old = self.conn
            self.conn = conn
            self.mbtiles = str(filename)
            self.attribution = meta.get("attribution", "離線圖資：請依來源授權標示")
            if old is not None:
                old.close()
        return meta

    def clear_mbtiles(self):
        with self.lock:
            if self.conn:
                self.conn.close()
            self.conn = None
            self.mbtiles = None
            self.attribution = ""

    def read_mbtiles(self, z, x, y):
        with self.lock:
            if self.conn is None:
                return None
            yy = 2 ** z - 1 - y if self.scheme != "xyz" else y
            row = self.conn.execute(
                "SELECT tile_data FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?",
                (z, x, yy)).fetchone()
            return row[0] if row else None


class SurveyApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(APP_TITLE)
        self.geometry("1330x805")
        self.minsize(1000, 660)
        self.configure(bg="#f3f6fa")
        self.sites = []
        self.selected_id = None
        self.map_lat, self.map_lon, self.zoom = 24.0, 120.0, 15
        self.tiles = TileProvider()
        self.offline = tk.BooleanVar(value=False)
        self.coord_mode = tk.StringVar(value="Decimal")
        self.site_id = tk.StringVar(value="0")
        self.lat = tk.StringVar(value="24.000000")
        self.lon = tk.StringVar(value="120.000000")
        self.ground = tk.StringVar(value="30.0")
        self.agl = tk.StringVar(value="2.0")
        self.geoid = tk.StringVar(value="0.0")
        self.status = tk.StringVar(value="請新增站點。地圖高度不影響底圖，Up 將輸出至 ENU CSV。")
        self.dms_vars = [[tk.StringVar(value="0") for _ in range(3)] for _ in range(2)]
        self.hemis = [tk.StringVar(value="N"), tk.StringVar(value="E")]
        self.tile_queue = queue.Queue()
        self.tile_pending = set()
        self.tile_images = {}
        self.drawn_images = []
        self.current_tiles = []
        self.drag_at = None
        self.render_after = None
        self.position_path = tk.StringVar()
        self.position_hz = tk.StringVar(value="0.2")
        self.position_follow = tk.BooleanVar(value=True)
        self.position_status = tk.StringVar(value="尚未載入 Position CSV")
        self.position_queue = queue.Queue()
        self.position_track = deque(maxlen=MAX_TRAIL_POINTS)
        self.position_latest = None
        self.position_fix = None
        self.position_origin = None
        self.position_running = False
        self.position_generation = 0
        self.position_cancel = threading.Event()
        self.position_after = None
        self.position_last_record_at = None
        self.position_error = ""
        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self._close)
        self._previous_mode = self.coord_mode.get()
        self.coord_mode.trace_add("write", lambda *_: self._switch_mode())
        self.after(100, self._poll_tiles)
        self.after(250, self.render_map)

    def _build_ui(self):
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("TButton", padding=(10, 6))
        style.configure("Title.TLabel", font=("DejaVu Sans", 16, "bold"))
        main = ttk.Panedwindow(self, orient="horizontal")
        main.pack(fill="both", expand=True, padx=12, pady=12)
        map_frame = ttk.Frame(main)
        side_frame = ttk.Frame(main, width=405)
        main.add(map_frame, weight=4)
        main.add(side_frame, weight=2)

        header = ttk.Frame(map_frame)
        header.pack(fill="x", pady=(0, 7))
        ttk.Label(header, text="站點與即時定位", style="Title.TLabel").pack(side="left")
        ttk.Button(header, text="所有站點", command=self.fit_sites).pack(side="right", padx=3)
        ttk.Button(header, text="−", width=3, command=lambda: self.change_zoom(-1)).pack(side="right", padx=3)
        ttk.Button(header, text="+", width=3, command=lambda: self.change_zoom(1)).pack(side="right", padx=3)
        navigation = ttk.LabelFrame(map_frame, text="Position CSV 即時 Debug", padding=6)
        navigation.pack(fill="x", pady=(0, 7))
        path_row = ttk.Frame(navigation)
        path_row.pack(fill="x")
        ttk.Entry(path_row, textvariable=self.position_path).pack(side="left", fill="x", expand=True)
        ttk.Button(path_row, text="選取 CSV", command=self.open_position_csv).pack(side="left", padx=3)
        controls = ttk.Frame(navigation)
        controls.pack(fill="x", pady=(4, 0))
        ttk.Label(controls, text="更新 Hz").pack(side="left")
        ttk.Combobox(controls, textvariable=self.position_hz, values=("0.1", "0.2", "0.5", "1"),
                     width=5, state="readonly").pack(side="left", padx=3)
        self.position_button = ttk.Button(controls, text="開始 / 重新載入", command=self.start_positions)
        self.position_button.pack(side="left", padx=2)
        ttk.Button(controls, text="暫停", command=self.stop_positions).pack(side="left", padx=2)
        track_controls = ttk.Frame(navigation)
        track_controls.pack(fill="x", pady=(3, 0))
        ttk.Button(track_controls, text="定位置中", command=self.center_position).pack(side="left", padx=2)
        ttk.Button(track_controls, text="清除軌跡", command=self.clear_position_track).pack(side="left", padx=2)
        ttk.Checkbutton(track_controls, text="跟隨定位", variable=self.position_follow).pack(side="left")
        origin_hint = ttk.Label(navigation,
                                text="ENU 原點使用目前 Survey 的 A0 天線位置；須與 solver 設定一致。",
                                foreground="#657789", wraplength=480)
        origin_hint.pack(anchor="w", pady=(3, 0))
        position_label = ttk.Label(navigation, textvariable=self.position_status,
                                   wraplength=480, foreground="#2e5070")
        position_label.pack(anchor="w")
        navigation.bind("<Configure>", lambda event: (
            origin_hint.configure(wraplength=max(180, event.width - 20)),
            position_label.configure(wraplength=max(180, event.width - 20))))
        self.canvas = tk.Canvas(map_frame, bg="#e6ecf2", highlightthickness=0, cursor="fleur")
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda e: self.queue_redraw())
        self.canvas.bind("<ButtonPress-1>", self._press_map)
        self.canvas.bind("<B1-Motion>", self._move_map)
        self.canvas.bind("<MouseWheel>", self._wheel_map)
        self.canvas.bind("<Button-4>", lambda e: self.change_zoom(1))
        self.canvas.bind("<Button-5>", lambda e: self.change_zoom(-1))
        map_bottom = ttk.Frame(map_frame)
        map_bottom.pack(fill="x", pady=(6, 0))
        ttk.Checkbutton(map_bottom, text="離線模式（僅讀本機地圖）", variable=self.offline,
                        command=self.queue_redraw).pack(side="left")
        ttk.Button(map_bottom, text="開啟離線 MBTiles", command=self.open_mbtiles).pack(side="right", padx=3)
        ttk.Button(map_bottom, text="離線地圖說明", command=self.offline_help).pack(side="right", padx=3)
        ttk.Button(map_bottom, text="使用線上地圖", command=self.use_online).pack(side="right", padx=3)
        self.map_hint = ttk.Label(map_frame, text="在線：© OpenStreetMap contributors ｜ 左鍵拖曳地圖，滾輪縮放",
                                  foreground="#67788b")
        self.map_hint.pack(anchor="w", pady=(3, 0))

        side = ttk.Frame(side_frame, padding=(14, 2, 4, 4))
        side.pack(fill="both", expand=True)
        ttk.Label(side, text="Survey 站點管理", style="Title.TLabel").pack(anchor="w", pady=(0, 10))
        f = ttk.LabelFrame(side, text="站點資料", padding=10)
        f.pack(fill="x")
        idrow = ttk.Frame(f)
        idrow.pack(fill="x", pady=3)
        ttk.Label(idrow, text="站點 ID", width=15).pack(side="left")
        self.id_combo = ttk.Combobox(idrow, textvariable=self.site_id,
                                      values=[str(i) for i in range(8)] + ["initiator"], width=18)
        self.id_combo.pack(side="left", fill="x", expand=True)
        r = ttk.Frame(f)
        r.pack(fill="x", pady=(7, 3))
        ttk.Label(r, text="經緯度格式", width=15).pack(side="left")
        ttk.Radiobutton(r, text="小數度", variable=self.coord_mode, value="Decimal").pack(side="left")
        ttk.Radiobutton(r, text="度分秒 DMS", variable=self.coord_mode, value="DMS").pack(side="left")
        self.dec_frame = ttk.Frame(f)
        self.dec_frame.pack(fill="x")
        self._form_entry(self.dec_frame, "緯度 Latitude", self.lat)
        self._form_entry(self.dec_frame, "經度 Longitude", self.lon)
        self.dms_frame = ttk.Frame(f)
        for idx, label in enumerate(("緯度", "經度")):
            r = ttk.Frame(self.dms_frame)
            r.pack(fill="x", pady=3)
            ttk.Label(r, text=label, width=7).pack(side="left")
            for j, suffix in enumerate(("°", "′", "″")):
                ttk.Entry(r, textvariable=self.dms_vars[idx][j], width=6).pack(side="left")
                ttk.Label(r, text=suffix).pack(side="left")
            ttk.Combobox(r, textvariable=self.hemis[idx], width=3, state="readonly",
                         values=("N", "S") if idx == 0 else ("E", "W")).pack(side="left", padx=5)
        self._ground_row = self._form_entry(f, "地面高程 (m)", self.ground)
        self._form_entry(f, "天線離地 (m)", self.agl)
        ttk.Label(f, text="天線高度 = 地面高程 + 天線離地高度", foreground="#63748a").pack(anchor="w", pady=(2, 4))
        buttons = ttk.Frame(f)
        buttons.pack(fill="x", pady=(9, 0))
        ttk.Button(buttons, text="新增 / 更新", command=self.add_or_update).pack(side="left", fill="x", expand=True, padx=2)
        ttk.Button(buttons, text="清空表單", command=self.clear_form).pack(side="left", padx=2)
        ttk.Button(buttons, text="刪除選取", command=self.delete_site).pack(side="left", padx=2)

        sites_frame = ttk.LabelFrame(side, text="目前站點（點選可編輯）", padding=8)
        sites_frame.pack(fill="both", expand=True, pady=(11, 0))
        self.tree = ttk.Treeview(sites_frame, columns=("id", "lat", "lon", "height"), show="headings", height=7)
        for col, title, w in (("id", "ID", 72), ("lat", "緯度", 105), ("lon", "經度", 105), ("height", "天線高程", 91)):
            self.tree.heading(col, text=title)
            self.tree.column(col, width=w, anchor="center", stretch=col != "id")
        self.tree.pack(fill="both", expand=True)
        self.tree.bind("<<TreeviewSelect>>", self.select_site)

        export = ttk.LabelFrame(side, text="Survey CSV / ENU", padding=10)
        export.pack(fill="x", pady=(10, 0))
        self._form_entry(export, "共同 Geoid N (m)", self.geoid)
        ttk.Label(export, text="Google Earth 高程通常近似正高；N=0 為局部近似，精密測量請確認高程基準。",
                  foreground="#657789", wraplength=355, justify="left").pack(anchor="w", pady=3)
        exp_row = ttk.Frame(export)
        exp_row.pack(fill="x", pady=4)
        ttk.Button(exp_row, text="載入 Survey CSV", command=self.load_survey).pack(side="left", fill="x", expand=True, padx=2)
        ttk.Button(exp_row, text="儲存 LLH ＋ 產生 ENU", command=self.save_survey).pack(side="left", fill="x", expand=True, padx=2)
        ttk.Button(export, text="匯出 ENU CSV（供 GNU Radio 使用）", command=self.export_enu).pack(fill="x", pady=(4, 0))

        ttk.Label(self, textvariable=self.status, anchor="w", padding=(12, 5),
                  foreground="#2e5070").pack(fill="x", side="bottom")
        self._switch_mode()

    @staticmethod
    def _form_entry(parent, label, variable):
        r = ttk.Frame(parent)
        r.pack(fill="x", pady=3)
        ttk.Label(r, text=label, width=19).pack(side="left")
        ttk.Entry(r, textvariable=variable).pack(side="left", fill="x", expand=True)
        return r

    def _switch_mode(self):
        # Preserve edits when switching from DMS back to decimal degrees.
        if getattr(self, "_previous_mode", "Decimal") == "DMS" and self.coord_mode.get() == "Decimal":
            try:
                lat = dms_to_decimal(*(v.get() for v in self.dms_vars[0]), self.hemis[0].get(), True)
                lon = dms_to_decimal(*(v.get() for v in self.dms_vars[1]), self.hemis[1].get(), False)
                self.lat.set(f"{lat:.10f}")
                self.lon.set(f"{lon:.10f}")
            except ValueError:
                pass
        self._previous_mode = self.coord_mode.get()
        if self.coord_mode.get() == "DMS":
            try:
                for idx, val in enumerate((float(self.lat.get()), float(self.lon.get()))):
                    a, b, c, direction = decimal_to_dms(val, idx == 0)
                    for j, txt in enumerate((a, b, c)):
                        self.dms_vars[idx][j].set(txt)
                    self.hemis[idx].set(direction)
            except ValueError:
                pass
            self.dec_frame.pack_forget()
            self.dms_frame.pack(fill="x", before=self._ground_row)
        else:
            # Reorder after mode switching so elevation stays underneath lat/lon
            self.dms_frame.pack_forget()
            self.dec_frame.pack(fill="x", before=self._ground_row)

    def get_coords(self):
        if self.coord_mode.get() == "DMS":
            lat = dms_to_decimal(*(v.get() for v in self.dms_vars[0]), self.hemis[0].get(), True)
            lon = dms_to_decimal(*(v.get() for v in self.dms_vars[1]), self.hemis[1].get(), False)
        else:
            lat = parse_finite(self.lat.get(), "緯度")
            lon = parse_finite(self.lon.get(), "經度")
        if not -90 <= lat <= 90 or not -180 <= lon <= 180:
            raise ValueError("經緯度超出有效範圍")
        self.lat.set(f"{lat:.10f}")
        self.lon.set(f"{lon:.10f}")
        return lat, lon

    def add_or_update(self):
        try:
            label = self.site_id.get().strip().lower()
            if label != "initiator":
                ident = int(label)
                if not 0 <= ident <= 7:
                    raise ValueError("Anchor ID 必須在 0 至 7 之間")
            else:
                ident = "initiator"
            lat, lon = self.get_coords()
            ground = parse_finite(self.ground.get(), "地面高程")
            agl = parse_finite(self.agl.get(), "天線離地高度")
            if agl < 0:
                raise ValueError("天線離地高度不可為負")
            entry = {"id": ident, "lat": lat, "lon": lon, "ground": ground, "agl": agl}
            old = next((idx for idx, s in enumerate(self.sites) if s["id"] == ident), None)
            if old is None:
                self.sites.append(entry)
            else:
                self.sites[old] = entry
            self.selected_id = ident
            self.refresh_sites()
            self.center_at(lat, lon)
            self.status.set(f"已{'新增' if old is None else '更新'} {ident}，座標已顯示於地圖")
        except (ValueError, TypeError) as exc:
            messagebox.showerror("站點資料錯誤", str(exc))

    def clear_form(self):
        self.selected_id = None
        ids = {s["id"] for s in self.sites}
        new_id = next((n for n in range(8) if n not in ids), 0)
        self.site_id.set(str(new_id))
        self.tree.selection_remove(self.tree.selection())

    def delete_site(self):
        if self.selected_id is None:
            return
        self.sites = [s for s in self.sites if s["id"] != self.selected_id]
        self.status.set(f"已刪除 {self.selected_id}")
        self.selected_id = None
        self.refresh_sites()

    def refresh_sites(self):
        for item in self.tree.get_children():
            self.tree.delete(item)
        for s in sorted_sites(self.sites):
            idstr = str(s["id"])
            self.tree.insert("", "end", iid=idstr,
                             values=(idstr, f"{s['lat']:.7f}", f"{s['lon']:.7f}", f"{s['ground']+s['agl']:.2f}"))
        if self.selected_id is not None and self.tree.exists(str(self.selected_id)):
            self.tree.selection_set(str(self.selected_id))
        self.queue_redraw()

    def select_site(self, _event=None):
        selected = self.tree.selection()
        if not selected:
            return
        label = selected[0]
        site = next((s for s in self.sites if str(s["id"]) == label), None)
        if site is None:
            return
        self.selected_id = site["id"]
        self.site_id.set(label)
        self.lat.set(str(site["lat"]))
        self.lon.set(str(site["lon"]))
        self.ground.set(str(site["ground"]))
        self.agl.set(str(site["agl"]))
        if self.coord_mode.get() == "DMS":
            for idx, val in enumerate((site["lat"], site["lon"])):
                a, b, c, direction = decimal_to_dms(val, idx == 0)
                for j, txt in enumerate((a, b, c)):
                    self.dms_vars[idx][j].set(txt)
                self.hemis[idx].set(direction)
        self.center_at(site["lat"], site["lon"])

    def center_at(self, lat, lon):
        self.map_lat, self.map_lon = lat, lon
        self.queue_redraw()

    def fit_sites(self):
        if not self.sites:
            return
        lats = [s["lat"] for s in self.sites]
        lons = [s["lon"] for s in self.sites]
        self.map_lat = (max(lats) + min(lats)) / 2
        self.map_lon = (max(lons) + min(lons)) / 2
        w = max(self.canvas.winfo_width() - 130, 150)
        h = max(self.canvas.winfo_height() - 130, 150)
        for z in range(19, 2, -1):
            pts = [latlon_to_world(s["lat"], s["lon"], z) for s in self.sites]
            if max(x for x, _ in pts) - min(x for x, _ in pts) <= w and max(y for _, y in pts) - min(y for _, y in pts) <= h:
                self.zoom = z
                break
        self.queue_redraw()

    def _press_map(self, event):
        self.drag_at = (event.x, event.y)

    def _move_map(self, event):
        if self.drag_at is None:
            return
        dx = event.x - self.drag_at[0]
        dy = event.y - self.drag_at[1]
        wx, wy = latlon_to_world(self.map_lat, self.map_lon, self.zoom)
        self.map_lat, self.map_lon = world_to_latlon(wx - dx, wy - dy, self.zoom)
        self.map_lon = max(-179.999, min(179.999, self.map_lon))
        self.drag_at = (event.x, event.y)
        self.queue_redraw()

    def _wheel_map(self, event):
        self.change_zoom(1 if event.delta > 0 else -1)

    def change_zoom(self, delta):
        self.zoom = max(2, min(19, self.zoom + delta))
        self.queue_redraw()

    def queue_redraw(self):
        if self.render_after:
            self.after_cancel(self.render_after)
        self.render_after = self.after(80, self.render_map)

    def render_map(self):
        self.render_after = None
        cv = self.canvas
        w, h = cv.winfo_width(), cv.winfo_height()
        if w < 10 or h < 10:
            return
        cv.delete("all")
        self.drawn_images = []
        cx, cy = latlon_to_world(self.map_lat, self.map_lon, self.zoom)
        x0, y0 = cx - w / 2, cy - h / 2
        z = self.zoom
        limit = 2 ** z
        xs = range(math.floor(x0 / TILE_PX), math.ceil((x0 + w) / TILE_PX))
        ys = range(math.floor(y0 / TILE_PX), math.ceil((y0 + h) / TILE_PX))
        for tx in xs:
            for ty in ys:
                left, top = tx * TILE_PX - x0, ty * TILE_PX - y0
                if ty < 0 or ty >= limit:
                    continue
                xx = tx % limit
                tile = self._tile_image(z, xx, ty)
                if tile:
                    cv.create_image(left, top, image=tile, anchor="nw")
                    self.drawn_images.append(tile)
                else:
                    cv.create_rectangle(left, top, left + TILE_PX, top + TILE_PX,
                                        fill="#e8edf2", outline="#d4dde7")
                    cv.create_text(left + 128, top + 128, text=f"{z}/{xx}/{ty}", fill="#a6b2bf", font=("Arial", 10))
        for s in sorted_sites(self.sites):
            px, py = latlon_to_world(s["lat"], s["lon"], z)
            # make sure the marker stays aligned when wrapping the longitude
            px += round((cx - px) / (limit * TILE_PX)) * limit * TILE_PX
            sx, sy = px - x0, py - y0
            if sx < -80 or sy < -80 or sx > w + 100 or sy > h + 80:
                continue
            chosen = self.selected_id == s["id"]
            initiator = s["id"] == "initiator"
            color = "#e68b25" if initiator else "#267acb"
            rad = 9 if chosen else 7
            cv.create_oval(sx-rad-3, sy-rad-3, sx+rad+3, sy+rad+3, fill="white", outline="")
            cv.create_oval(sx-rad, sy-rad, sx+rad, sy+rad, fill=color, outline="#183b5d" if chosen else "white", width=2)
            label = "Initiator" if initiator else f"A{s['id']}"
            cv.create_text(sx + 13, sy - 14, text=label, anchor="w", font=("Arial", 11, "bold"), fill="#142e43")
            cv.create_text(sx + 14, sy + 4, text=f"H {s['ground']+s['agl']:.2f} m", anchor="w", font=("Arial", 9), fill="#304455")
        self._draw_positions()
        cv.create_rectangle(9, 9, 272, 52, fill="white", outline="#cbd7e2")
        cv.create_text(17, 18, text=f"Lat {self.map_lat:.7f}   Lon {self.map_lon:.7f}", anchor="nw", fill="#23415e")
        cv.create_text(17, 36, text=f"Zoom {z}  |  {'離線 MBTiles' if self.offline.get() else '線上 OSM'}", anchor="nw", fill="#60758a")
        self.map_hint.configure(text=(f"離線圖資：{Path(self.tiles.mbtiles).name} ｜ {self.tiles.attribution} ｜缺少圖磚會顯示空白網格" if self.offline.get() and self.tiles.mbtiles
                                      else "離線模式：尚未開啟 MBTiles" if self.offline.get()
                                      else "© OpenStreetMap contributors ｜ 地圖操作：拖曳 / 滾輪縮放"))

    def _tile_image(self, z, x, y):
        key = ("offline" if self.offline.get() else "online", z, x, y)
        if key in self.tile_images:
            return self.tile_images[key]
        if self.offline.get():
            data = self.tiles.read_mbtiles(z, x, y)
            if data:
                try:
                    import base64
                    photo = tk.PhotoImage(data=base64.b64encode(data).decode("ascii"))
                    self.tile_images[key] = photo
                    return photo
                except tk.TclError:
                    # JPEG imagery requires Pillow; provide an actionable warning once.
                    self.status.set("此 MBTiles 包含 Tk 不支援的影像格式；請換 PNG MBTiles，或安裝 Pillow 轉圖。")
            return None
        file = CACHE_DIR / str(z) / str(x) / f"{y}.png"
        if file.exists():
            try:
                photo = tk.PhotoImage(file=str(file))
                self.tile_images[key] = photo
                return photo
            except tk.TclError:
                pass
        if key not in self.tile_pending:
            self.tile_pending.add(key)
            threading.Thread(target=self._fetch_tile, args=(key, file), daemon=True).start()
        return None

    def _fetch_tile(self, key, file):
        _, z, x, y = key
        # Only tiles in the *current viewport* are fetched: no prefetch / bulk offline.
        try:
            req = urllib.request.Request(OSM_URL.format(z=z, x=x, y=y), headers={"User-Agent": USER_AGENT})
            with urllib.request.urlopen(req, timeout=6) as resp:
                data = resp.read(2_000_000)
            if not data.startswith(b"\x89PNG\r\n\x1a\n"):
                return
            file.parent.mkdir(parents=True, exist_ok=True)
            file.write_bytes(data)
        except (urllib.error.URLError, OSError):
            pass
        finally:
            self.tile_queue.put(key)

    def _poll_tiles(self):
        updated = False
        try:
            while True:
                key = self.tile_queue.get_nowait()
                self.tile_pending.discard(key)
                updated = True
        except queue.Empty:
            pass
        if updated and not self.offline.get():
            self.queue_redraw()
        self._poll_position_results()
        self.after(150, self._poll_tiles)

    def _get_position_origin(self):
        anchor = next((site for site in self.sites if site["id"] == 0), None)
        if anchor is None:
            raise ValueError("請先載入 Survey CSV 或設定 A0，才能將 ENU 定位轉為地圖座標")
        geoid = parse_finite(self.geoid.get(), "Geoid N")
        return (anchor["lat"], anchor["lon"], anchor["ground"] + anchor["agl"] + geoid)

    def open_position_csv(self):
        filename = filedialog.askopenfilename(title="開啟持續寫入的 Position CSV",
                                             filetypes=[("CSV", "*.csv"), ("All files", "*")])
        if filename:
            self.stop_positions()
            self.position_path.set(filename)
            self.position_status.set("已選取檔案，按「開始 / 重新載入」讀取")

    def start_positions(self):
        try:
            origin = self._get_position_origin()
            filename = self.position_path.get().strip()
            if not filename:
                raise ValueError("請先選取 Position CSV")
            hz = parse_finite(self.position_hz.get(), "更新 Hz")
            if not 0.01 <= hz <= 2:
                raise ValueError("Debug 更新率請使用 0.01–2 Hz")
        except ValueError as exc:
            messagebox.showerror("無法開始定位顯示", str(exc))
            return
        self.stop_positions()
        self.position_origin = origin
        self.position_track.clear()
        self.position_fix = self.position_latest = None
        self.position_last_record_at = None
        self.position_error = ""
        self.position_reader = PositionCsvReader(filename)
        self.position_cancel = threading.Event()
        self.position_running = True
        self.position_status.set("正在讀取 Position CSV…")
        self._launch_position_read()
        self.queue_redraw()

    def stop_positions(self):
        self.position_running = False
        self.position_generation += 1
        self.position_cancel.set()
        if self.position_after is not None:
            self.after_cancel(self.position_after)
            self.position_after = None
        if self.position_fix is not None:
            self.position_status.set("已暫停；地圖保留上次有效定位")
        self._draw_positions()

    def _launch_position_read(self):
        self.position_after = None
        if not self.position_running:
            return
        try:
            if self._get_position_origin() != self.position_origin:
                raise ValueError("Survey A0 或 Geoid 已變更，請重新載入 Position CSV")
        except ValueError as exc:
            self.stop_positions()
            self.position_track.clear()
            self.position_fix = None
            self.position_status.set(str(exc))
            self._draw_positions()
            return
        generation = self.position_generation
        reader, cancelled = self.position_reader, self.position_cancel

        def read():
            try:
                result = reader.poll(cancelled)
                error = ""
            except (OSError, ValueError, UnicodeError, csv.Error) as exc:
                result, error = None, str(exc)
            if not cancelled.is_set():
                self.position_queue.put((generation, result, error))
        threading.Thread(target=read, daemon=True).start()

    def _poll_position_results(self):
        changed = False
        try:
            while True:
                generation, result, error = self.position_queue.get_nowait()
                if generation != self.position_generation or not self.position_running:
                    continue
                # Editing the survey while a read was in flight must not plot
                # an old frame against the newly edited anchor markers.
                try:
                    same_origin = self._get_position_origin() == self.position_origin
                except ValueError:
                    same_origin = False
                if not same_origin:
                    self.stop_positions()
                    self.position_track.clear()
                    self.position_fix = None
                    self.position_status.set("Survey 原點已變更，請重新載入 Position CSV")
                    self._draw_positions()
                    continue
                self.position_error = error
                if result is not None:
                    if result["reset"]:
                        self.position_track.clear()
                        self.position_fix = self.position_latest = None
                        self.position_last_record_at = None
                    for row in result["fixes"]:
                        latitude, longitude = enu_to_latlon(*row["xyz"], self.position_origin)
                        point = (latitude, longitude, row["break_before"])
                        self.position_track.append(point)
                        self.position_fix = (latitude, longitude, row)
                    self.position_latest = result["latest"]
                    self.position_invalid = result["invalid"]
                    if result["new_records"]:
                        # Use file mtime only to seed local freshness; never
                        # compare UHD reference_timestamp with Unix time.
                        file_age = max(0., time.time() - result.get("mtime", time.time()))
                        self.position_last_record_at = time.monotonic() - file_age
                    if result["fixes"] and self.position_follow.get():
                        self.center_position()
                self.position_after = self.after(int(1000 / float(self.position_hz.get())),
                                                 self._launch_position_read)
                changed = True
        except queue.Empty:
            pass
        if self.position_running:
            self._update_position_status()
        if changed:
            self._draw_positions()

    def _position_state(self):
        if not self.position_running:
            return "已暫停", "#8793a0"
        if self.position_error:
            return "CSV 讀取失敗", "#e68b25"
        if self.position_last_record_at is None:
            return "等待完整紀錄", "#8793a0"
        age = time.monotonic() - self.position_last_record_at
        if age >= max(15., 3 / float(self.position_hz.get())):
            return "資料未更新", "#8793a0"
        if self.position_latest and self.position_latest["success"].strip() == "0":
            return "定位失敗", "#e68b25"
        return "定位成功", "#159a71"

    def _update_position_status(self):
        state, _ = self._position_state()
        if self.position_error:
            self.position_status.set(f"{state}：{self.position_error[:180]}")
            return
        row = self.position_latest
        if row is None:
            self.position_status.set(f"{state}（只接受 success=1 的有限 XYZ）")
            return
        reason = " ".join(row["reason"].split())[:100]
        age = max(0., time.monotonic() - self.position_last_record_at)
        text = (f"{state} | cycle {row.get('cycle_sequence', '')} / poll {row.get('poll_id', '')}"
                f" | {reason} | CSV 最近新增紀錄 {age:.0f} s 前")
        if self.position_fix:
            x, y, z = self.position_fix[2]["xyz"]
            text += f"\n上次有效 ENU：E {x:.2f} / N {y:.2f} / U {z:.2f} m"
        if getattr(self, "position_invalid", 0):
            text += f" | 略過無效列 {self.position_invalid}"
        self.position_status.set(text)

    def center_position(self):
        if self.position_fix:
            self.center_at(self.position_fix[0], self.position_fix[1])

    def clear_position_track(self):
        self.position_track.clear()
        self._draw_positions()

    def _draw_positions(self):
        if not hasattr(self, "canvas"):
            return
        cv = self.canvas
        cv.delete("position_overlay")
        w, h = cv.winfo_width(), cv.winfo_height()
        cx, cy = latlon_to_world(self.map_lat, self.map_lon, self.zoom)
        world_width = 2 ** self.zoom * TILE_PX

        def screen(lat, lon):
            px, py = latlon_to_world(lat, lon, self.zoom)
            px += round((cx - px) / world_width) * world_width
            return px - cx + w / 2, py - cy + h / 2

        segment = []
        for lat, lon, break_before in self.position_track:
            if break_before and len(segment) >= 4:
                cv.create_line(*segment, fill="#17a2b8", width=3, tags="position_overlay")
            if break_before:
                segment = []
            segment.extend(screen(lat, lon))
        if len(segment) >= 4:
            cv.create_line(*segment, fill="#17a2b8", width=3, tags="position_overlay")
        if self.position_fix:
            sx, sy = screen(self.position_fix[0], self.position_fix[1])
            state, color = self._position_state()
            cv.create_oval(sx-10, sy-10, sx+10, sy+10, fill=color, outline="white",
                           width=3, tags="position_overlay")
            cv.create_text(sx+14, sy-12, text=f"SDR {state}", anchor="w",
                           fill=color, font=("Arial", 11, "bold"), tags="position_overlay")

    def _close(self):
        self.stop_positions()
        self.tiles.clear_mbtiles()
        self.destroy()

    def offline_help(self):
        messagebox.showinfo(
            "預先準備離線地圖",
            "本程式可以直接載入 PNG raster MBTiles。\n\n"
            "若要預先下載，可使用隨附 prepare_survey_mbtiles.py，"
            "但必須使用明確允許離線下載的圖磚供應商或自架圖磚伺服器。\n\n"
            "OpenStreetMap 官方標準圖磚服務禁止 bulk prefetch / 離線下載，"
            "本程式的 OSM 線上模式僅載入目前視窗所需圖磚。\n\n"
            "製作完成後按『開啟離線 MBTiles』。"
        )

    def open_mbtiles(self):
        filename = filedialog.askopenfilename(title="選取預先下載的 raster MBTiles 地圖", filetypes=[("MBTiles", "*.mbtiles"), ("All files", "*")])
        if not filename:
            return
        try:
            meta = self.tiles.set_mbtiles(filename)
            self.tile_images.clear()
            self.offline.set(True)
            self.status.set(f"已載入離線地圖 {Path(filename).name}。圖資來源：{meta.get('attribution', '請依圖資授權標示')}。")
            self.queue_redraw()
        except (OSError, sqlite3.Error, ValueError) as exc:
            messagebox.showerror("無法開啟 MBTiles", str(exc))

    def use_online(self):
        self.offline.set(False)
        self.queue_redraw()

    def load_survey(self):
        filename = filedialog.askopenfilename(title="開啟 LLH Survey CSV", filetypes=[("CSV", "*.csv")])
        if not filename:
            return
        try:
            sites = []
            with open(filename, newline="", encoding="utf-8-sig") as fp:
                reader = csv.DictReader(fp)
                if not reader.fieldnames or not set(INPUT_HEADERS).issubset(reader.fieldnames):
                    raise ValueError(f"缺少欄位：{', '.join(INPUT_HEADERS)}")
                for line, r in enumerate(reader, start=2):
                    ident = r["anchor_id"].strip().lower()
                    ident = "initiator" if ident == "initiator" else int(ident)
                    if ident != "initiator" and not 0 <= ident <= 7:
                        raise ValueError(f"第 {line} 行 Anchor ID 超出 0~7")
                    lat, lon = parse_finite(r["latitude_deg"], "latitude"), parse_finite(r["longitude_deg"], "longitude")
                    ground, agl = parse_finite(r["ground_elevation_m"], "ground"), parse_finite(r["antenna_agl_m"], "agl")
                    if not -90 <= lat <= 90 or not -180 <= lon <= 180 or agl < 0:
                        raise ValueError(f"第 {line} 行資料超出範圍")
                    sites.append({"id": ident, "lat": lat, "lon": lon, "ground": ground, "agl": agl})
            if len({s["id"] for s in sites}) != len(sites):
                raise ValueError("CSV 中有重複的站點 ID")
            self.sites = sites
            self.selected_id = None
            self.refresh_sites()
            self.fit_sites()
            self.status.set(f"已載入 {len(sites)} 筆站點。可繼續修改並匯出。")
        except (ValueError, OSError, TypeError) as exc:
            messagebox.showerror("CSV 載入失敗", str(exc))

    def _write_enu_csv(self, filename, rows):
        """Write the same CSV schema consumed by anchor_csv_loader.py."""
        with open(filename, "w", newline="", encoding="utf-8") as fp:
            writer = csv.DictWriter(fp, fieldnames=OUTPUT_HEADERS)
            writer.writeheader()
            writer.writerows(rows)

    def save_survey(self):
        """Save editable LLH and automatically generate a matching ENU CSV.

        Draft LLH surveys can still be saved without a complete initiator/
        consecutive anchor IDs; ENU export is deferred until valid.
        """
        if not self.sites:
            messagebox.showwarning("無站點資料", "請先新增站點")
            return
        filename = filedialog.asksaveasfilename(
            title="儲存 LLH Survey，並自動產生 anchors_enu.csv",
            defaultextension=".csv", initialfile="survey_llh.csv",
            filetypes=[("CSV", "*.csv")])
        if not filename:
            return

        # Validate before writing, but allow incomplete draft LLH surveys.
        rows = None
        conversion_error = None
        try:
            geoid = parse_finite(self.geoid.get(), "Geoid N")
            rows = convert_sites_to_enu(self.sites, geoid)
        except ValueError as exc:
            conversion_error = str(exc)

        llh_path = Path(filename)
        enu_path = llh_path.with_name("anchors_enu.csv")
        if rows is not None and enu_path.exists():
            if not messagebox.askyesno(
                    "確認覆寫 ENU CSV",
                    f"已存在 {enu_path}。\n是否以目前站點重新產生並覆寫？"):
                rows = None
                conversion_error = "使用者取消覆寫既有 ENU CSV"

        try:
            with llh_path.open("w", newline="", encoding="utf-8") as fp:
                writer = csv.DictWriter(fp, fieldnames=INPUT_HEADERS)
                writer.writeheader()
                for site in sorted_sites(self.sites):
                    writer.writerow({
                        "anchor_id": site["id"],
                        "latitude_deg": f"{site['lat']:.10f}",
                        "longitude_deg": f"{site['lon']:.10f}",
                        "ground_elevation_m": site["ground"],
                        "antenna_agl_m": site["agl"],
                    })
            if rows is None:
                self.status.set(f"LLH 已儲存：{llh_path}；ENU 未更新")
                messagebox.showwarning(
                    "LLH 已儲存，ENU 未輸出",
                    f"LLH 資料已儲存至：\n{llh_path}\n\n"
                    f"尚未產生 ENU：{conversion_error}\n"
                    "請補齊站點後再次儲存，或按「匯出 ENU CSV」。")
                return

            self._write_enu_csv(enu_path, rows)
            initiator = next(r for r in rows if r["anchor_id"] == "initiator")
            self.status.set(
                f"已儲存 LLH 並自動產生 {enu_path} | "
                f"vehicle_height_m = {initiator['up_m']:.4f} m")
            messagebox.showinfo(
                "Survey 儲存完成",
                f"LLH：{llh_path}\nENU：{enu_path}\n\n"
                f"Anchors：{len(rows)-1}\n"
                f"vehicle_height_m：{initiator['up_m']:.4f} m\n\n"
                "GRC 請讓 anchor_config 指向上述 ENU CSV。")
        except OSError as exc:
            messagebox.showerror("儲存或轉換失敗", str(exc))

    def export_enu(self):
        """Export ENU directly from GUI site entries, without a CLI step."""
        try:
            geoid = parse_finite(self.geoid.get(), "Geoid N")
            rows = convert_sites_to_enu(self.sites, geoid)
        except ValueError as exc:
            messagebox.showerror("無法轉換 ENU", str(exc))
            return
        filename = filedialog.asksaveasfilename(
            title="輸出供 GNU Radio 使用的 ENU CSV",
            defaultextension=".csv", initialfile="anchors_enu.csv",
            filetypes=[("CSV", "*.csv")])
        if not filename:
            return
        try:
            self._write_enu_csv(filename, rows)
            initiator = next(r for r in rows if r["anchor_id"] == "initiator")
            self.status.set(
                f"已輸出 {filename} | vehicle_height_m = {initiator['up_m']:.4f} m")
            messagebox.showinfo(
                "ENU CSV 已輸出",
                f"已輸出 {len(rows)-1} 個 Anchors 與 1 個 Initiator。\n"
                f"vehicle_height_m = {initiator['up_m']:.4f} m\n\n"
                "GRC: anchor_config[0] = IDs\n"
                "anchor_config[1] = XYZ\n"
                "anchor_config[2] = Vehicle Up")
        except OSError as exc:
            messagebox.showerror("寫入失敗", str(exc))


if __name__ == "__main__":
    app = SurveyApp()
    app.mainloop()
