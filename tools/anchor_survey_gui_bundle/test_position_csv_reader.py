#!/usr/bin/env python3
"""Offline unit checks: CSV tailing, survey frame and headless map updates."""
import csv
import io
import math
from pathlib import Path
import queue
import tempfile
import threading
import time
import unittest

from position_csv_reader import MAX_TRAIL_POINTS, PositionCsvReader, enu_to_latlon
from survey_llh_to_enu import llh_to_ecef, ecef_to_enu
from survey_map_gui import SurveyApp

HEADERS = ['cycle_sequence', 'poll_id', 'reference_timestamp', 'success', 'reason',
           'x_m', 'y_m', 'z_m', 'observed_anchor_count', 'used_anchor_count',
           'anchor_ids', 'residual_rms_m', 'residual_max_abs_m']


def record(cycle=0, success=1, reason='OK', xyz=(1., 2., 0.)):
    return [cycle, cycle % 65536, 1234., success, reason, *xyz, 3, 3, '0;1;2', .1, .2]


def encode(rows, header=False):
    output = io.StringIO(newline='')
    writer = csv.writer(output, lineterminator='\n')
    if header:
        writer.writerow(HEADERS)
    writer.writerows(rows)
    return output.getvalue().encode('utf-8')


class ReaderTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.path = Path(self.temp.name) / 'positions.csv'
        self.reader = PositionCsvReader(self.path)

    def tearDown(self):
        self.temp.cleanup()

    def append(self, data):
        with self.path.open('ab') as dest:
            dest.write(data)

    def test_append_failure_and_no_repeated_positions(self):
        self.path.write_bytes(encode([record(65535), record(65536)], True))
        first = self.reader.poll()
        self.assertTrue(first['reset'])
        self.assertEqual(len(first['fixes']), 2)
        self.assertEqual(first['latest']['cycle_sequence'], '65536')
        self.assertEqual(self.reader.poll()['new_records'], 0)
        self.assertEqual(self.reader.poll()['fixes'], [])
        self.append(encode([record(65537, 0, 'INSUFFICIENT_OBSERVATIONS', ('', '', ''))]))
        failed = self.reader.poll()
        self.assertEqual(failed['fixes'], [])
        self.assertEqual(failed['latest']['success'], '0')
        self.append(encode([record(65538)]))
        self.assertTrue(self.reader.poll()['fixes'][0]['break_before'])

    def test_unterminated_coordinate_and_utf8_are_deferred(self):
        self.path.write_bytes(encode([], True))
        row = encode([record(reason='定位完成', xyz=(1.234, 2., 0.))])
        index = row.index('定位'.encode()) + 1
        self.append(row[:index])
        self.assertEqual(self.reader.poll()['new_records'], 0)
        self.append(row[index:-1])
        self.assertEqual(self.reader.poll()['new_records'], 0)
        self.append(b'\n')
        result = self.reader.poll()
        self.assertEqual(result['latest']['xyz'], (1.234, 2., 0.))
        self.assertEqual(result['records'], 1)

    def test_multiline_quoted_failure_does_not_create_a_fix(self):
        row = encode([record(success=0, reason='bad, "geometry"\n下一行', xyz=('', '', ''))])
        self.path.write_bytes(encode([], True) + row[:row.index(b'\n') + 1])
        self.assertEqual(self.reader.poll()['records'], 0)
        self.append(row[row.index(b'\n') + 1:])
        result = self.reader.poll()
        self.assertEqual(result['latest']['reason'], 'bad, "geometry"\n下一行')
        self.assertEqual(result['fixes'], [])
        self.assertEqual(result['records'], 1)

    def test_invalid_success_coordinates_are_skipped(self):
        self.path.write_bytes(encode([record(xyz=('nan', 1, 0)),
                                     record(xyz=(1, 'inf', 0)),
                                     record(xyz=('', 1, 0)), record(9)], True))
        result = self.reader.poll()
        self.assertEqual(result['invalid'], 3)
        self.assertEqual(result['records'], 1)
        self.assertEqual(result['latest']['cycle_sequence'], '9')

    def test_rotation_truncation_and_same_inode_rewrite_reset(self):
        self.path.write_bytes(encode([record(i) for i in range(10)], True))
        self.reader.poll()
        self.path.write_bytes(encode([record(22)], True))
        self.assertTrue(self.reader.poll()['reset'])
        replacement = self.path.with_suffix('.new')
        replacement.write_bytes(encode([record(23)], True))
        replacement.replace(self.path)
        result = self.reader.poll()
        self.assertTrue(result['reset'])
        self.assertEqual(result['records'], 1)
        self.path.write_bytes(encode([record(i) for i in range(30, 50)], True))
        result = self.reader.poll()
        self.assertTrue(result['reset'])
        self.assertEqual(result['latest']['cycle_sequence'], '49')

    def test_invalid_header_recovers_after_replacement(self):
        self.path.write_bytes(b'not,a,position,csv\n')
        with self.assertRaises(ValueError): self.reader.poll()
        self.path.write_bytes(encode([record()], True))
        self.assertEqual(self.reader.poll()['records'], 1)

    def test_missing_file_can_appear_later(self):
        with self.assertRaises(FileNotFoundError): self.reader.poll()
        self.path.write_bytes(encode([record()], True))
        self.assertEqual(self.reader.poll()['records'], 1)

    def test_history_is_bounded_and_cancellation_is_supported(self):
        self.path.write_bytes(encode([record(i) for i in range(2*MAX_TRAIL_POINTS)], True))
        result = self.reader.poll()
        self.assertEqual(len(result['fixes']), MAX_TRAIL_POINTS)
        self.assertEqual(result['records'], 2*MAX_TRAIL_POINTS)
        cancelled = threading.Event()
        cancelled.set()
        self.append(encode([record(3000)]))
        self.assertIsNone(self.reader.poll(cancelled))

    def test_enu_matches_survey_export_frame(self):
        for origin in [(24., 120., 32.), (-33., 151., 100.), (80., -170., 50.)]:
            xyz0 = llh_to_ecef(*origin)
            for lat, lon, height in [(origin[0], origin[1], origin[2]),
                                     (origin[0]+.001, origin[1]-.001, origin[2]+5)]:
                enu = ecef_to_enu(llh_to_ecef(lat, lon, height), xyz0, *origin[:2])
                out = enu_to_latlon(*enu, origin)
                self.assertAlmostEqual(out[0], lat, places=8)
                self.assertAlmostEqual(out[1], lon, places=8)
        east = enu_to_latlon(10., 0., 0., (24., 120., 32.))
        north = enu_to_latlon(0., 10., 0., (24., 120., 32.))
        self.assertGreater(east[1], 120.)
        self.assertGreater(north[0], 24.)


class Variable:
    def __init__(self, value): self.value = value
    def get(self): return self.value
    def set(self, value): self.value = value


class Canvas:
    def __init__(self): self.items = []
    def winfo_width(self): return 800
    def winfo_height(self): return 600
    def delete(self, tag): self.items = []
    def create_line(self, *args, **kwargs): self.items.append(('line', args, kwargs))
    def create_oval(self, *args, **kwargs): self.items.append(('oval', args, kwargs))
    def create_text(self, *args, **kwargs): self.items.append(('text', args, kwargs))


class HeadlessMap:
    """Exercise actual GUI result/plot logic without connecting to a display or tiles."""
    _get_position_origin = SurveyApp._get_position_origin
    _poll_position_results = SurveyApp._poll_position_results
    _draw_positions = SurveyApp._draw_positions
    _position_state = SurveyApp._position_state
    _update_position_status = SurveyApp._update_position_status
    stop_positions = SurveyApp.stop_positions
    _launch_position_read = SurveyApp._launch_position_read
    center_position = SurveyApp.center_position

    def __init__(self):
        from collections import deque
        self.sites = [dict(id=0, lat=24., lon=120., ground=30., agl=2.)]
        self.geoid = Variable('0')
        self.position_hz = Variable('0.2')
        self.position_follow = Variable(False)
        self.position_status = Variable('')
        self.position_origin = (24., 120., 32.)
        self.position_running = True
        self.position_generation = 0
        self.position_cancel = threading.Event()
        self.position_after = None
        self.position_queue = queue.Queue()
        self.position_latest = self.position_fix = None
        self.position_last_record_at = None
        self.position_error = ''
        self.position_track = deque(maxlen=MAX_TRAIL_POINTS)
        self.map_lat, self.map_lon, self.zoom = 24., 120., 19
        self.canvas = Canvas()
        self.delays = []

    def after(self, delay, callback): self.delays.append(delay); return len(self.delays)
    def after_cancel(self, _): pass
    def center_at(self, lat, lon): self.map_lat, self.map_lon = lat, lon


class MapUpdateTests(unittest.TestCase):
    def test_failure_retains_fix_and_poll_schedules_five_seconds(self):
        app = HeadlessMap()
        row = dict(success='1', reason='OK', xyz=(10., 20., 1.), break_before=True)
        app.position_queue.put((0, dict(reset=True, fixes=[row], latest=row,
                                       invalid=0, new_records=1), ''))
        app._poll_position_results()
        last_fix = app.position_fix
        self.assertEqual(app.delays, [5000])
        self.assertEqual(len(app.position_track), 1)
        failed = dict(success='0', reason='INSUFFICIENT_OBSERVATIONS')
        app.position_queue.put((0, dict(reset=False, fixes=[], latest=failed,
                                       invalid=0, new_records=1), ''))
        app._poll_position_results()
        self.assertEqual(app.position_fix, last_fix)
        self.assertEqual(app._position_state()[0], '定位失敗')
        self.assertIn('INSUFFICIENT_OBSERVATIONS', app.position_status.get())
        self.assertTrue(any(item[0] == 'oval' for item in app.canvas.items))
        app.position_last_record_at = time.monotonic() - 16
        self.assertEqual(app._position_state()[0], '資料未更新')

    def test_old_csv_is_marked_stale_on_initial_load(self):
        app = HeadlessMap()
        row = dict(success='1', reason='OK', xyz=(0., 0., 0.), break_before=True)
        app.position_queue.put((0, dict(reset=True, fixes=[row], latest=row,
                                       invalid=0, new_records=1, mtime=time.time()-60), ''))
        app._poll_position_results()
        self.assertEqual(app._position_state()[0], '資料未更新')
        self.assertIsNotNone(app.position_fix)

    def test_origin_edit_and_stopped_worker_results_are_rejected(self):
        app = HeadlessMap()
        app.sites[0]['lat'] = 25.
        app.position_queue.put((0, dict(reset=True, fixes=[], latest=None,
                                       invalid=0, new_records=0), ''))
        app._poll_position_results()
        self.assertFalse(app.position_running)
        self.assertIn('原點已變更', app.position_status.get())
        app.position_queue.put((0, {}, 'old worker'))
        app._poll_position_results()
        self.assertEqual(app.position_error, '')


if __name__ == '__main__':
    unittest.main()
