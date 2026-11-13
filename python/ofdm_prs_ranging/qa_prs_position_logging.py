#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
"""Partial-cycle positioning and success/failure CSV contracts (synthetic only)."""
import csv
import math
from pathlib import Path
import tempfile
import time

import pmt
from gnuradio import blocks, gr, gr_unittest, ofdm_prs_ranging as prs


POSITIONS = [[0., 0., 0.], [10., 0., 0.], [0., 10., 0.], [10., 10., 0.]]
TARGET = [3., 4., 1.]


def dictionary(**values):
    result = pmt.make_dict()
    for key, value in values.items():
        if isinstance(value, bool): value = pmt.from_bool(value)
        elif isinstance(value, int): value = pmt.from_uint64(value)
        elif isinstance(value, float): value = pmt.from_double(value)
        elif isinstance(value, str): value = pmt.intern(value)
        result = pmt.dict_add(result, pmt.intern(key), value)
    return result


def field(message, key):
    meta = message if pmt.is_dict(message) else pmt.car(message)
    return pmt.dict_ref(meta, pmt.intern(key), pmt.PMT_NIL)


def value(message, key):
    return pmt.to_python(field(message, key))


def observation(ids=(0, 1, 2), ranges=None, corrected=None, flags=None, **extra):
    if ranges is None:
        ranges = [math.dist(TARGET, POSITIONS[i]) for i in ids]
    if corrected is None: corrected = [float('nan')] * len(ids)
    if flags is None: flags = [0] * len(ids)
    fields = dict(poll_id=65535, cycle_sequence=70000, reference_timestamp=1234.5,
                  completion_reason='WINDOWS_EXPIRED',
                  observed_anchor_ids=pmt.init_u32vector(len(ids), ids),
                  integer_ranges_m=pmt.init_f64vector(len(ranges), ranges),
                  corrected_ranges_m=pmt.init_f64vector(len(corrected), corrected),
                  fine_correction_valid_flags=pmt.init_u8vector(len(flags), flags))
    fields.update(extra)
    return pmt.cons(dictionary(**fields), pmt.PMT_NIL)


class qa_prs_position_logging(gr_unittest.TestCase):
    def setUp(self):
        self.tb = gr.top_block()
        self.running = False
        self.temp = tempfile.TemporaryDirectory(prefix='prs_position_')
        self.path = Path(self.temp.name) / 'positions.csv'

    def tearDown(self):
        if self.running:
            self.tb.stop()
            self.tb.wait()
        self.temp.cleanup()

    def wait_for(self, condition):
        deadline = time.monotonic() + 3
        while not condition() and time.monotonic() < deadline:
            time.sleep(.002)
        self.assertTrue(condition())

    def post(self, block, port, message):
        block.to_basic_block()._post(pmt.intern(port), message)

    def rows(self):
        with self.path.open(newline='') as source:
            return list(csv.DictReader(source))

    def pipeline(self, positions=POSITIONS, ids=(0, 1, 2, 3)):
        self.solver = prs.prs_position_solver(list(ids), positions, TARGET[2])
        self.logger = prs.prs_position_csv_logger(str(self.path), False)
        self.success, self.failures = blocks.message_debug(), blocks.message_debug()
        for port, target, input_port in (
                ('position_out', self.logger, 'position_in'),
                ('diagnostic_out', self.logger, 'diagnostic_in'),
                ('position_out', self.success, 'store'),
                ('diagnostic_out', self.failures, 'store')):
            self.tb.msg_connect(self.solver, port, target, input_port)

    def start(self):
        self.tb.start()
        self.running = True

    def test_partial_fix_and_fine_integer_selection(self):
        self.pipeline()
        self.start()
        true_ranges = [math.dist(TARGET, POSITIONS[i]) for i in (0, 1, 2)]
        self.post(self.solver, 'epoch_in', observation(
            ranges=[true_ranges[0] + 8, true_ranges[1], true_ranges[2]],
            corrected=[true_ranges[0], float('nan'), true_ranges[2] + 9], flags=[1, 1, 0]))
        self.wait_for(lambda: self.success.num_messages() == 1 and len(self.rows()) == 1)
        message = self.success.get_message(0)
        for observed, expected in zip(value(message, 'position_m'), TARGET):
            self.assertAlmostEqual(observed, expected, delta=1e-5)
        self.assertEqual(list(value(message, 'used_fine_correction_flags')), [1, 0, 0])
        row = self.rows()[0]
        self.assertEqual(row['cycle_sequence'], '70000')
        self.assertEqual(row['poll_id'], '65535')
        self.assertEqual(float(row['reference_timestamp']), 1234.5)
        self.assertEqual(row['success'], '1')
        self.assertEqual(row['reason'], 'OK')
        self.assertEqual(row['observed_anchor_count'], '3')
        self.assertEqual(row['used_anchor_count'], '3')
        self.assertEqual(row['anchor_ids'], '0;1;2')
        self.assertLess(float(row['residual_rms_m']), 1e-5)
        self.assertNotIn('expected_anchor_count', row)
        self.assertNotIn('missing_anchor_ids', row)
        self.assertFalse(any('range' in name for name in row))
        self.assertEqual(self.failures.num_messages(), 0)


    def test_invalid_fourth_range_still_allows_three_anchor_fix(self):
        self.pipeline()
        self.start()
        ranges = [math.dist(TARGET, POSITIONS[i]) for i in (0, 1, 2)] + [0.]
        self.post(self.solver, 'epoch_in', observation(ids=[0, 1, 2, 3], ranges=ranges))
        self.wait_for(lambda: len(self.rows()) == 1)
        row = self.rows()[0]
        self.assertEqual(row['success'], '1')
        self.assertEqual(row['observed_anchor_count'], '4')
        self.assertEqual(row['used_anchor_count'], '3')
        self.assertEqual(row['anchor_ids'], '0;1;2')
        self.assertAlmostEqual(float(row['x_m']), TARGET[0], delta=1e-5)
        self.assertAlmostEqual(float(row['y_m']), TARGET[1], delta=1e-5)

    def test_zero_two_and_invalid_ranges_have_blank_coordinates(self):
        self.pipeline()
        self.start()
        for index, message in enumerate((observation(ids=[]), observation(ids=[0, 1]),
                                        observation(ids=[0, 1, 2, 3], ranges=[5., 0., float('nan'), -1.]))):
            self.post(self.solver, 'epoch_in', message)
            self.wait_for(lambda: len(self.rows()) == index + 1)
        for row, observed, used in zip(self.rows(), (0, 2, 4), (0, 2, 1)):
            self.assertEqual(row['reason'], 'INSUFFICIENT_OBSERVATIONS')
            self.assertEqual(row['success'], '0')
            self.assertEqual(row['observed_anchor_count'], str(observed))
            self.assertEqual(row['used_anchor_count'], str(used))
            self.assertEqual([row[name] for name in ('x_m', 'y_m', 'z_m')], ['', '', ''])
            self.assertEqual(row['residual_rms_m'], '')
        self.assertEqual(self.success.num_messages(), 0)

    def test_malformed_unknown_duplicate_and_aborted_inputs_are_reported(self):
        self.pipeline()
        self.start()
        cases = [(pmt.PMT_NIL, 'INVALID_INPUT'),
                 (observation(ranges=[1.]), 'INVALID_OBSERVATION_LAYOUT'),
                 (observation(ids=[0, 0, 1]), 'INVALID_ANCHOR_ID'),
                 (observation(ids=[0, 1, 7], ranges=[1., 2., 3.]), 'INVALID_ANCHOR_ID'),
                 (observation(completion_reason='STOPPED_INCOMPLETE'), 'CYCLE_ABORTED')]
        for index, (message, reason) in enumerate(cases):
            self.post(self.solver, 'epoch_in', message)
            self.wait_for(lambda: len(self.rows()) == index + 1)
            self.assertEqual(self.rows()[-1]['reason'], reason)
        self.assertEqual(self.success.num_messages(), 0)
        self.assertEqual(self.rows()[0]['poll_id'], '')

    def test_singular_geometry_is_logged(self):
        self.pipeline(positions=[[0., 0., 0.], [10., 0., 0.], [20., 0., 0.]], ids=[0, 1, 2])
        self.start()
        self.post(self.solver, 'epoch_in', observation(ranges=[5., 6., 7.]))
        self.wait_for(lambda: len(self.rows()) == 1)
        self.assertEqual(self.rows()[0]['reason'], 'SINGULAR_GEOMETRY')
        self.assertEqual(self.rows()[0]['x_m'], '')

    def test_nonconvergence_is_logged(self):
        self.pipeline()
        self.start()
        self.post(self.solver, 'epoch_in', observation(ranges=[1., 1., 1.]))
        self.wait_for(lambda: len(self.rows()) == 1)
        self.assertEqual(self.rows()[0]['reason'], 'NOT_CONVERGED')
        self.assertEqual(self.rows()[0]['x_m'], '')

    def test_csv_escaping_precision_and_diagnostic_trial_coordinates(self):
        logger = prs.prs_position_csv_logger(str(self.path), False)
        source = blocks.message_strobe(pmt.PMT_NIL, 10)
        self.tb.msg_connect(source, 'strobe', logger, 'diagnostic_in')
        self.start()
        reason = 'bad, "geometry"\nnext line'
        message = dictionary(reason=reason, cycle_sequence=2**63 + 17, poll_id=9,
                             position_m=pmt.init_f64vector(3, TARGET),
                             selected_anchor_ids=pmt.init_u32vector(3, [0, 1, 2]),
                             residual=pmt.init_f64vector(3, [3., 4., 0.]),
                             observed_anchor_count=3, used_anchor_count=3)
        self.post(logger, 'diagnostic_in', pmt.cons(message, pmt.PMT_NIL))
        self.wait_for(lambda: len(self.rows()) == 1)
        row = self.rows()[0]
        self.assertEqual(row['reason'], reason)
        self.assertEqual(row['cycle_sequence'], str(2**63 + 17))
        self.assertEqual(row['x_m'], '')
        self.assertAlmostEqual(float(row['residual_rms_m']), math.sqrt(25/3))
        self.assertEqual(float(row['residual_max_abs_m']), 4.)

    def test_append_validates_header_and_open_failure(self):
        logger = prs.prs_position_csv_logger(str(self.path), False)
        del logger
        original = self.path.read_text()
        logger = prs.prs_position_csv_logger(str(self.path), True)
        del logger
        self.assertEqual(self.path.read_text(), original)
        self.path.write_text('wrong,header\n')
        with self.assertRaises(ValueError):
            prs.prs_position_csv_logger(str(self.path), True)
        self.assertEqual(self.path.read_text(), 'wrong,header\n')
        with self.assertRaises(RuntimeError):
            prs.prs_position_csv_logger(str(self.path / 'cannot_open.csv'), False)


    def test_append_preserves_rows_and_writes_one_header(self):
        for append, poll in ((False, 7), (True, 8)):
            self.tb = gr.top_block()
            logger = prs.prs_position_csv_logger(str(self.path), append)
            source = blocks.message_strobe(pmt.PMT_NIL, 10)
            self.tb.msg_connect(source, 'strobe', logger, 'diagnostic_in')
            self.start()
            message = dictionary(poll_id=poll, cycle_sequence=poll, reason='CYCLE_ABORTED')
            self.post(logger, 'diagnostic_in', pmt.cons(message, pmt.PMT_NIL))
            self.wait_for(lambda: len(self.rows()) == poll - 6)
            self.tb.stop()
            self.tb.wait()
            self.running = False
        self.assertEqual([row['poll_id'] for row in self.rows()], ['7', '8'])
        self.assertEqual(self.path.read_text().count('cycle_sequence,poll_id'), 1)

    def test_finalized_cycles_reach_solver_and_both_csvs(self):
        self.pipeline()
        cycle = prs.prs_multi_anchor_cycle(anchor_ids=[0, 1, 2, 3],
                                          time_delays=[.031, .081, .131, .181])
        range_solver = prs.prs_ssrtt_solver(30e6)
        measurement_path = Path(self.temp.name) / 'measurements.csv'
        measurements = prs.prs_csv_logger(str(measurement_path), False)
        windows, accepted, epochs, observations = [blocks.message_debug() for _ in range(4)]
        for source, port, target, target_port in (
                (cycle, 'solver_tx_out', range_solver, 'tx_time_in'),
                (cycle, 'solver_measurement_out', range_solver, 'measurement_in'),
                (range_solver, 'ssrtt_out', cycle, 'range_in'),
                (cycle, 'observation_out', self.solver, 'epoch_in'),
                (cycle, 'observation_out', observations, 'store'),
                (cycle, 'measurement_out', measurements, 'measurement_in'),
                (cycle, 'measurement_out', accepted, 'store'),
                (cycle, 'epoch_out', epochs, 'store'),
                (cycle, 'window_out', windows, 'store')):
            self.tb.msg_connect(source, port, target, target_port)
        self.start()
        total = 0
        # Same wire poll ID across cycles: generation and timestamp disambiguate.
        for index, count in enumerate((3, 2, 0, 4)):
            t1 = 100. + index
            self.post(cycle, 'tx_time_in', dictionary(packet_type=1, poll_id=42,
                                                     tx_time_secs=int(t1), tx_time_frac=0.))
            self.wait_for(lambda: windows.num_messages() == 4 * (index + 1))
            for anchor in reversed(range(count)):
                window = windows.get_message(4 * index + anchor)
                delay = int(value(window, 'expected_reply_delay_samples'))
                distance = math.dist(TARGET, POSITIONS[anchor])
                t4 = t1 + delay/30e6 + 2*(distance/299792458. + 5.808278798530661e-6)
                packet = dictionary(packet_type=2, frame_id_valid=True, poll_id=42,
                                    cycle_sequence=int(value(window, 'cycle_sequence')),
                                    anchor_id=anchor, reply_delay_samples=delay,
                                    response_slot_index=int(value(window, 'response_slot_index')),
                                    rx_time=pmt.make_tuple(pmt.from_uint64(math.floor(t4)),
                                                           pmt.from_double(t4-math.floor(t4))))
                self.post(cycle, 'measurement_in', pmt.cons(packet, pmt.PMT_NIL))
            total += count
            self.wait_for(lambda: accepted.num_messages() == total)
            end = max(float(value(windows.get_message(4*index+i), 'window_end')) for i in range(4))
            self.post(cycle, 'clock_in', dictionary(clock_time=end + .101))
            self.wait_for(lambda: len(self.rows()) == index + 1)
            row = self.rows()[-1]
            self.assertEqual(row['observed_anchor_count'], str(count))
            self.assertEqual(row['used_anchor_count'], str(count))
            self.assertEqual(row['success'], '1' if count >= 3 else '0')
            self.assertEqual(row['cycle_sequence'], str(index))
            self.assertEqual(row['poll_id'], '42')
            self.assertEqual(float(row['reference_timestamp']), t1)
            if count >= 3:
                self.assertAlmostEqual(float(row['x_m']), TARGET[0], delta=1e-4)
                self.assertAlmostEqual(float(row['y_m']), TARGET[1], delta=1e-4)
            self.assertEqual(epochs.num_messages(), 1 if count == 4 else 0)
            self.assertEqual(observations.num_messages(), index + 1)
            ids = value(observations.get_message(index), 'observed_anchor_ids')
            self.assertEqual(list(ids), list(range(count)))
        with measurement_path.open(newline='') as source:
            ranges = list(csv.DictReader(source))
        self.assertEqual(len(ranges), total)
        self.assertEqual({row['cycle_sequence'] for row in ranges}, {'0', '1', '3'})
        # Explicit finalization while downstream handlers are still running.
        self.post(cycle, 'tx_time_in', dictionary(packet_type=1, poll_id=43,
                                                 tx_time_secs=104, tx_time_frac=0.))
        self.wait_for(lambda: windows.num_messages() == 20)
        cycle.stop()
        self.wait_for(lambda: len(self.rows()) == 5)
        self.assertEqual(self.rows()[-1]['reason'], 'CYCLE_ABORTED')
        self.assertEqual(epochs.num_messages(), 1)


if __name__ == '__main__':
    gr_unittest.run(qa_prs_position_logging)
