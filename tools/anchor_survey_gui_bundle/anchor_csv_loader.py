"""Load anchor and initiator ENU from survey_llh_to_enu.py output.

load_anchor_config(path) returns:
    [0] anchor_ids: list[int]
    [1] anchor_positions_m: list[list[float]]
    [2] vehicle_height_m: float (initiator ENU Up), or None for old CSVs

The row with anchor_id='initiator' is kept separate from positioning anchors.
"""
import csv
import math
from pathlib import Path


def load_anchor_config(csv_path):
    path = Path(csv_path).expanduser()
    if not path.is_file():
        raise FileNotFoundError(f"Anchor CSV not found: {path}")

    positions_by_id = {}
    vehicle_height_m = None
    with path.open('r', newline='', encoding='utf-8-sig') as f:
        reader = csv.DictReader(f)
        required = {'anchor_id', 'east_m', 'north_m', 'up_m'}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise ValueError(f"CSV requires columns: {', '.join(sorted(required))}")
        for lineno, row in enumerate(reader, 2):
            try:
                label = row['anchor_id'].strip()
                is_initiator = label.lower() == 'initiator'
                anchor_id = None if is_initiator else int(label)
                position = [float(row[k]) for k in ('east_m', 'north_m', 'up_m')]
            except (AttributeError, TypeError, ValueError) as exc:
                raise ValueError(f"Invalid site data on CSV line {lineno}") from exc
            if not all(math.isfinite(v) for v in position):
                raise ValueError(f"Non-finite ENU coordinate on line {lineno}")
            if is_initiator:
                if vehicle_height_m is not None:
                    raise ValueError('Duplicate initiator row')
                vehicle_height_m = position[2]
            else:
                if anchor_id < 0 or anchor_id in positions_by_id:
                    raise ValueError(f"Invalid or duplicate anchor_id {anchor_id} on line {lineno}")
                positions_by_id[anchor_id] = position

    ids = sorted(positions_by_id)
    if not ids or ids != list(range(len(ids))):
        raise ValueError('Anchor IDs must be contiguous starting at 0')
    if len(ids) > 8:
        raise ValueError('Position Solver supports at most 8 anchors')

    return ids, [positions_by_id[i] for i in ids], vehicle_height_m


if __name__ == '__main__':
    import sys
    anchor_ids, anchor_positions_m, vehicle_height_m = load_anchor_config(sys.argv[1])
    print('anchor_ids =', anchor_ids)
    print('anchor_positions_m =', anchor_positions_m)
    print('vehicle_height_m =', vehicle_height_m)
