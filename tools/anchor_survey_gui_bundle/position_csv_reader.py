#!/usr/bin/env python3
"""Incremental, read-only position CSV tailer and WGS84 ENU map conversion.

No GNU Radio, Tk or third-party dependencies. CSV timestamps remain UHD time;
wall-clock freshness is measured by the GUI when new records are observed.
"""
from __future__ import annotations

import codecs
import csv
from collections import deque
import io
import math
import os
from pathlib import Path

from survey_llh_to_enu import A, F, E2, llh_to_ecef

REQUIRED_FIELDS = {"success", "reason", "x_m", "y_m", "z_m"}
MAX_TRAIL_POINTS = 1000


def enu_to_latlon(east, north, up, origin):
    """Invert the survey tool's ECEF->ENU rotation, then WGS84 ECEF->LLH.

    origin is (latitude_deg, longitude_deg, antenna_ellipsoidal_height_m).
    Solver XYZ must use the matching survey ENU frame, normally Anchor 0.
    """
    if not all(math.isfinite(v) for v in (east, north, up, *origin)):
        raise ValueError("ENU 與原點必須是有限數值")
    lat0, lon0, height0 = origin
    if not -90 <= lat0 <= 90 or not -180 <= lon0 <= 180:
        raise ValueError("ENU 原點經緯度超出範圍")
    lat, lon = math.radians(lat0), math.radians(lon0)
    slat, clat, slon, clon = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
    x0, y0, z0 = llh_to_ecef(lat0, lon0, height0)
    x = x0 - slon * east - slat * clon * north + clat * clon * up
    y = y0 + clon * east - slat * slon * north + clat * slon * up
    z = z0 + clat * north + slat * up
    p = math.hypot(x, y)
    b = A * (1 - F)
    theta = math.atan2(z * A, p * b)
    latitude = math.atan2(z + E2 / (1 - E2) * b * math.sin(theta)**3,
                          p - E2 * A * math.cos(theta)**3)
    return math.degrees(latitude), math.degrees(math.atan2(y, x))


class PositionCsvReader:
    """One worker owns an instance; each poll reads only appended bytes.

    Every poll consumes at most the file size observed when it opened, so a
    continuously appending writer cannot keep the worker busy indefinitely.
    Complete quoted CSV records (including multiline reasons) are accepted;
    unterminated records/UTF-8 codepoints are held for the next poll. Rotation,
    truncation and rewrites at the consumed boundary reset the track.
    """
    def __init__(self, filename):
        self.path = Path(filename)
        self.identity = None
        self._reset()

    def _reset(self):
        self.offset = 0
        self.decoder = codecs.getincrementaldecoder("utf-8-sig")()
        self.pending = ""
        self.fields = None
        self.boundary = b""
        self.latest = None
        self.records = 0
        self.invalid = 0
        self.break_pending = True

    def _parse(self, fixes):
        stream = io.StringIO(self.pending, newline="")
        reader = csv.reader(stream, strict=True)
        consumed = 0
        while True:
            start = stream.tell()
            try:
                cells = next(reader)
            except StopIteration:
                break
            except csv.Error as exc:
                if "unexpected end of data" in str(exc):
                    consumed = start
                    break
                # A malformed complete row does not poison later records.
                self.invalid += 1
                self.break_pending = True
                consumed = stream.tell()
                continue
            end = stream.tell()
            # Writer may be between write() calls. Do not accept a CSV record
            # until the terminating LF has actually arrived.
            if not self.pending[start:end].endswith("\n"):
                consumed = start
                break
            consumed = end
            if not cells:
                continue
            if self.fields is None:
                if not REQUIRED_FIELDS.issubset(cells) or len(cells) != len(set(cells)):
                    raise ValueError("Position CSV 缺少 success/reason/x_m/y_m/z_m 欄位，或欄位重複")
                self.fields = cells
                continue
            if len(cells) != len(self.fields):
                self.invalid += 1
                self.break_pending = True
                continue
            row = dict(zip(self.fields, cells))
            flag = row["success"].strip()
            if flag not in ("0", "1"):
                self.invalid += 1
                self.break_pending = True
                continue
            if flag == "1":
                try:
                    xyz = tuple(float(row[key]) for key in ("x_m", "y_m", "z_m"))
                    if not all(math.isfinite(v) for v in xyz):
                        raise ValueError("Nonfinite position")
                except (ValueError, TypeError):
                    self.invalid += 1
                    self.break_pending = True
                    continue
                row["xyz"] = xyz
                row["break_before"] = self.break_pending
                fixes.append(row)
                self.break_pending = False
            else:
                self.break_pending = True
            self.latest = row
            self.records += 1
        self.pending = self.pending[consumed:]
        if len(self.pending) > 1_000_000:
            raise ValueError("CSV 未完成的紀錄超過 1 MB，請確認輸入檔案")

    def poll(self, cancelled=None):
        try:
            return self._poll(cancelled)
        except (ValueError, UnicodeError, csv.Error):
            # Do not retain a broken decoder/header after the writer replaces
            # an invalid or temporarily incomplete input with a valid file.
            self.identity = None
            self._reset()
            raise

    def _poll(self, cancelled=None):
        fixes = deque(maxlen=MAX_TRAIL_POINTS)
        before = self.records
        reset = self.identity is None
        with self.path.open("rb") as source:
            stat = os.fstat(source.fileno())
            identity = (stat.st_dev, stat.st_ino)
            if identity != self.identity or stat.st_size < self.offset:
                reset = True
            elif self.boundary:
                source.seek(self.offset - len(self.boundary))
                reset = source.read(len(self.boundary)) != self.boundary
            if reset:
                self._reset()
                before = 0
                self.identity = identity
            source.seek(self.offset)
            # Snapshot end excludes bytes appended while this poll is parsing.
            end = stat.st_size
            while source.tell() < end:
                if cancelled is not None and cancelled.is_set():
                    return None
                data = source.read(min(65536, end - source.tell()))
                if not data:
                    break
                self.pending += self.decoder.decode(data)
                self._parse(fixes)
            self.offset = source.tell()
            source.seek(max(0, self.offset - 128))
            self.boundary = source.read(self.offset - source.tell())
        return {"reset": reset, "fixes": list(fixes), "latest": self.latest,
                "new_records": self.records - before, "records": self.records,
                "invalid": self.invalid, "mtime": stat.st_mtime}
