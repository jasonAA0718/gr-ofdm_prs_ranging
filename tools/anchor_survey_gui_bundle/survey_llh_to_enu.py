#!/usr/bin/env python3
"""Convert Google Earth survey LLH into a common WGS84 local ENU frame.

Input CSV columns:
    anchor_id,latitude_deg,longitude_deg,ground_elevation_m,antenna_agl_m

Use numeric anchor_id (0,1,...) for responder anchors and the literal
'initiator' for the vehicle / initiator antenna. The initiator is *not*
counted as a positioning anchor. Anchor 0 is the default common ENU origin.
Google Earth elevations are generally sea-level-referenced. The optional
geoid correction is a common-offset approximation for a small survey area;
for survey-quality positioning, use consistent surveyed ellipsoidal heights.
"""
import argparse
import csv
import math
from pathlib import Path

A = 6378137.0
F = 1.0 / 298.257223563
E2 = F * (2.0 - F)


def llh_to_ecef(lat_deg, lon_deg, h_m):
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    prime_vertical = A / math.sqrt(1.0 - E2 * sin_lat * sin_lat)
    return ((prime_vertical + h_m) * cos_lat * cos_lon,
            (prime_vertical + h_m) * cos_lat * sin_lon,
            (prime_vertical * (1.0 - E2) + h_m) * sin_lat)


def ecef_to_enu(xyz, origin_xyz, lat0_deg, lon0_deg):
    dx, dy, dz = (a - b for a, b in zip(xyz, origin_xyz))
    lat, lon = math.radians(lat0_deg), math.radians(lon0_deg)
    slat, clat = math.sin(lat), math.cos(lat)
    slon, clon = math.sin(lon), math.cos(lon)
    return (-slon * dx + clon * dy,
            -slat * clon * dx - slat * slon * dy + clat * dz,
            clat * clon * dx + clat * slon * dy + slat * dz)


def read_sites(path, geoid_m):
    anchors = []
    initiator = None
    with path.open(newline='', encoding='utf-8-sig') as fp:
        reader = csv.DictReader(fp)
        required = {'anchor_id', 'latitude_deg', 'longitude_deg',
                    'ground_elevation_m', 'antenna_agl_m'}
        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise ValueError(f'CSV 必須包含欄位：{", ".join(sorted(required))}')
        for line, row in enumerate(reader, start=2):
            try:
                label = row['anchor_id'].strip()
                is_initiator = label.lower() == 'initiator'
                anchor_id = None if is_initiator else int(label)
                lat = float(row['latitude_deg'])
                lon = float(row['longitude_deg'])
                ground = float(row['ground_elevation_m'])
                agl = float(row['antenna_agl_m'])
                if ((anchor_id is not None and anchor_id < 0)
                        or not -90 <= lat <= 90 or not -180 <= lon <= 180
                        or not all(map(math.isfinite, (lat, lon, ground, agl)))):
                    raise ValueError('座標或數值超出有效範圍')
            except (ValueError, TypeError, AttributeError) as exc:
                raise ValueError(f'CSV 第 {line} 行無效：{exc}') from exc
            site = dict(anchor_id='initiator' if is_initiator else anchor_id,
                        lat=lat, lon=lon, ground=ground, agl=agl,
                        antenna_h=ground + agl + geoid_m)
            if is_initiator:
                if initiator is not None:
                    raise ValueError('CSV 只能有一筆 initiator')
                initiator = site
            else:
                anchors.append(site)
    if not anchors:
        raise ValueError('CSV 沒有 Anchor')
    ids = [s['anchor_id'] for s in anchors]
    if len(ids) != len(set(ids)):
        raise ValueError('anchor_id 不可重複')
    if sorted(ids) != list(range(len(anchors))):
        raise ValueError('目前 Position Solver 要求 anchor_id 連續且從 0 開始')
    if len(anchors) > 8:
        raise ValueError('Position Solver 至多支援 8 個 Anchors（不含 initiator）')
    return sorted(anchors, key=lambda s: s['anchor_id']), initiator


def main():
    parser = argparse.ArgumentParser(description='Google Earth LLH → ENU Survey Tool')
    parser.add_argument('input_csv', type=Path, help='站點 CSV 檔案')
    parser.add_argument('--origin-id', type=int, default=0,
                        help='ENU 原點的 Anchor ID (預設 0)')
    parser.add_argument('--output', type=Path, default=Path('anchors_enu.csv'),
                        help='輸出 CSV 路徑 (預設 anchors_enu.csv)')
    parser.add_argument('--geoid-undulation-m', type=float, default=0.0,
                        help='共同大地起伏 N；高程 H 轉橢球高 h=H+N（未知時以 0 近似）')
    args = parser.parse_args()
    if not math.isfinite(args.geoid_undulation_m):
        parser.error('--geoid-undulation-m 必須為有限數值')
    anchors, initiator = read_sites(args.input_csv, args.geoid_undulation_m)
    by_id = {s['anchor_id']: s for s in anchors}
    if args.origin_id not in by_id:
        parser.error(f'找不到原點 anchor_id={args.origin_id}')
    origin = by_id[args.origin_id]
    origin_ecef = llh_to_ecef(origin['lat'], origin['lon'], origin['antenna_h'])
    rows = []
    print(f'ENU origin: Anchor {args.origin_id}, lat={origin["lat"]:.9f}, '
          f'lon={origin["lon"]:.9f}, antenna height={origin["antenna_h"]:.3f} m')
    print(' ID           East (m)      North (m)         Up (m)')
    for site in anchors + ([initiator] if initiator else []):
        xyz = llh_to_ecef(site['lat'], site['lon'], site['antenna_h'])
        e, n, u = ecef_to_enu(xyz, origin_ecef, origin['lat'], origin['lon'])
        rows.append(dict(anchor_id=site['anchor_id'], east_m=e, north_m=n,
                         up_m=u, latitude_deg=site['lat'],
                         longitude_deg=site['lon'],
                         ground_elevation_m=site['ground'],
                         antenna_agl_m=site['agl']))
        print(f'{str(site["anchor_id"]):>9} {e:14.4f} {n:14.4f} {u:14.4f}')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('w', newline='', encoding='utf-8') as fp:
        writer = csv.DictWriter(fp, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f'\n已輸出: {args.output}')
    print('GNU Radio anchor_positions_m = [')
    for row in rows[:len(anchors)]:
        print(f'    [{row["east_m"]:.6f}, {row["north_m"]:.6f}, {row["up_m"]:.6f}],')
    print(']')
    if initiator:
        print(f'vehicle_height_m = {rows[-1]["up_m"]:.6f}  # initiator ENU Up')
    else:
        print('警告：未提供 initiator，無法輸出 vehicle_height_m')
    print('\n注意：Google Earth 高程可能不是橢球高；請確認所有站點的高度基準一致。')


if __name__ == '__main__':
    main()
