#!/usr/bin/env python3
"""Prepare PNG MBTiles for offline field surveys from an authorized XYZ tile server.

This script intentionally has NO default tile server. OSM standard tile servers
explicitly prohibit prefetch/offline downloads. Use a service / local server
that explicitly permits offline downloading and retains required attribution.
"""
import argparse
import csv
import math
from pathlib import Path
import sqlite3
import time
import urllib.request


def xyz_tile(lat, lon, z):
    lat = max(-85.05112878, min(85.05112878, lat))
    n = 2 ** z
    x = int((lon + 180) / 360 * n)
    y = int((1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * n)
    return max(0, min(n - 1, x)), max(0, min(n - 1, y))


def main():
    parser = argparse.ArgumentParser(description='Download authorized raster PNG tiles as offline MBTiles')
    parser.add_argument('survey_csv', type=Path, help='LLH Survey CSV with latitude_deg/longitude_deg')
    parser.add_argument('--url', required=True, help='Authorized XYZ PNG tiles URL, e.g. https://YOUR_HOST/{z}/{x}/{y}.png')
    parser.add_argument('--output', type=Path, default=Path('survey_map.mbtiles'))
    parser.add_argument('--min-zoom', type=int, default=14)
    parser.add_argument('--max-zoom', type=int, default=17)
    parser.add_argument('--padding-m', type=float, default=250)
    parser.add_argument('--max-tiles', type=int, default=1200)
    parser.add_argument('--attribution', required=True, help='Tile source attribution/license notice to store')
    parser.add_argument('--user-agent', default='PRSAnchorSurveyOffline/1.0 (authorized downloads)')
    parser.add_argument('--i-have-permission', action='store_true', help='Confirm provider authorizes offline bulk downloading')
    args = parser.parse_args()
    if not args.i_have_permission:
        parser.error('Must pass --i-have-permission after verifying download license')
    if 'tile.openstreetmap.org' in args.url or 'googleapis.com' in args.url or 'mt0.google.com' in args.url:
        parser.error('OSM standard tiles and Google Maps tiles must not be bulk-downloaded for offline use')
    if not args.url.startswith('https://') and not args.url.startswith('http://localhost') and not args.url.startswith('http://127.0.0.1'):
        parser.error('Use HTTPS or a localhost tile server')
    if not all(t in args.url for t in ('{x}', '{y}', '{z}')):
        parser.error('URL must contain {z}, {x}, {y} placeholders')
    if not (0 <= args.min_zoom <= args.max_zoom <= 19) or args.max_tiles <= 0 or not (0 <= args.padding_m < 10000):
        parser.error('Invalid zoom/count/padding configuration')
    locations = []
    with args.survey_csv.open(newline='', encoding='utf-8-sig') as fp:
        for row in csv.DictReader(fp):
            locations.append((float(row['latitude_deg']), float(row['longitude_deg'])))
    if not locations:
        parser.error('Survey CSV has no coordinates')
    lats, lons = zip(*locations)
    midlat = (min(lats) + max(lats)) / 2
    dlat = args.padding_m / 111_200
    dlon = args.padding_m / max(111_200 * math.cos(math.radians(midlat)), 1)
    minlat, maxlat = max(-85, min(lats)-dlat), min(85, max(lats)+dlat)
    minlon, maxlon = max(-179.99, min(lons)-dlon), min(179.99, max(lons)+dlon)
    to_fetch = []
    for z in range(args.min_zoom, args.max_zoom + 1):
        x0, y0 = xyz_tile(maxlat, minlon, z)
        x1, y1 = xyz_tile(minlat, maxlon, z)
        for x in range(x0, x1 + 1):
            for y in range(y0, y1 + 1):
                to_fetch.append((z, x, y))
    print(f'Region: lat {minlat:.6f}..{maxlat:.6f}; lon {minlon:.6f}..{maxlon:.6f}')
    print(f'Tile count: {len(to_fetch)}; zoom {args.min_zoom}..{args.max_zoom}')
    if len(to_fetch) > args.max_tiles:
        parser.error(f'Requested {len(to_fetch)} tiles exceeds --max-tiles {args.max_tiles}. Reduce zoom/radius.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    db = sqlite3.connect(str(args.output))
    try:
        db.execute('CREATE TABLE IF NOT EXISTS metadata (name TEXT PRIMARY KEY, value TEXT)')
        db.execute('CREATE TABLE IF NOT EXISTS tiles (zoom_level INTEGER, tile_column INTEGER, tile_row INTEGER, tile_data BLOB, PRIMARY KEY (zoom_level, tile_column, tile_row))')
        for name, value in {'name': 'PRS survey offline map', 'format': 'png', 'type': 'baselayer',
                            'version': '1.0', 'scheme': 'tms', 'attribution': args.attribution,
                            'bounds': f'{minlon},{minlat},{maxlon},{maxlat}',
                            'minzoom': str(args.min_zoom), 'maxzoom': str(args.max_zoom)}.items():
            db.execute('INSERT OR REPLACE INTO metadata VALUES (?,?)', (name, value))
        db.commit()
        for i, (z, x, y) in enumerate(to_fetch, 1):
            tms_y = 2 ** z - 1 - y
            existing = db.execute('SELECT 1 FROM tiles WHERE zoom_level=? AND tile_column=? AND tile_row=?',
                                  (z, x, tms_y)).fetchone()
            if existing:
                continue
            url = args.url.format(z=z, x=x, y=y)
            req = urllib.request.Request(url, headers={'User-Agent': args.user_agent})
            try:
                with urllib.request.urlopen(req, timeout=15) as response:
                    blob = response.read(2_000_000)
                if not blob.startswith(b'\x89PNG\r\n\x1a\n'):
                    raise ValueError('Returned image is not PNG')
                db.execute('INSERT INTO tiles VALUES (?,?,?,?)', (z, x, tms_y, blob))
                db.commit()
            except Exception as exc:
                print(f'Failed tile {z}/{x}/{y}: {exc}')
            if i % 20 == 0 or i == len(to_fetch):
                print(f'{i}/{len(to_fetch)} processed')
            time.sleep(0.12)
    finally:
        db.close()
    print('Saved:', args.output)


if __name__ == '__main__':
    main()
