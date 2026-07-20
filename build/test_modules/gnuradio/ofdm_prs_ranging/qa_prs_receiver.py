#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2026 GNU Radio ZC TWR contributors.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

import os
import tempfile
import time

import numpy
import pmt
from gnuradio import blocks, gr, gr_unittest

try:
    from gnuradio import ofdm_prs_ranging
except ImportError:
    import sys

    dirname, filename = os.path.split(os.path.abspath(__file__))
    sys.path.append(os.path.join(dirname, "bindings"))
    from gnuradio import ofdm_prs_ranging


class qa_prs_receiver(gr_unittest.TestCase):
    def setUp(self):
        self.tb = gr.top_block()

    def tearDown(self):
        self.tb = None

    def test_clean_frame_pipeline_outputs_measurement(self):
        tx = ofdm_prs_ranging.prs_timed_burst_source(attach_tx_time=False)
        frame = list(tx.frame_samples())
        prefix = [0j] * 300
        data = prefix + frame + [0j] * 300
        timing = blocks.vector_source_c(data, False)

        detector = ofdm_prs_ranging.prs_frame_detector(threshold=0.30)
        fft = ofdm_prs_ranging.prs_fft_receiver()
        channel = ofdm_prs_ranging.prs_channel_estimator()
        phase = ofdm_prs_ranging.prs_phase_slope_estimator(max_residual_rms=1.0)
        debug = blocks.message_debug()

        self.tb.connect(timing, detector)
        self.tb.msg_connect((detector, "frame_out"), (fft, "frame_in"))
        self.tb.msg_connect((fft, "symbols_out"), (channel, "symbols_in"))
        self.tb.msg_connect((channel, "channel_out"), (phase, "channel_in"))
        self.tb.msg_connect((phase, "measurement_out"), (debug, "store"))
        self.tb.run()

        self.assertGreaterEqual(debug.num_messages(), 1)
        msg = debug.get_message(0)
        meta = pmt.car(msg)
        self.assertEqual(pmt.to_uint64(pmt.dict_ref(meta, pmt.intern("recv_id"), pmt.PMT_NIL)), 0)
        self.assertEqual(pmt.to_uint64(pmt.dict_ref(meta, pmt.intern("frame_id"), pmt.PMT_NIL)), 0)
        self.assertTrue(pmt.to_bool(pmt.dict_ref(meta, pmt.intern("frame_id_valid"), pmt.PMT_F)))
        self.assertEqual(pmt.to_long(pmt.dict_ref(meta, pmt.intern("coarse_zc_root"), pmt.PMT_NIL)), 25)
        self.assertEqual(pmt.to_long(pmt.dict_ref(meta, pmt.intern("channel_id"), pmt.PMT_NIL)), 0)
        self.assertEqual(pmt.to_uint64(pmt.dict_ref(meta, pmt.intern("frame_start"), pmt.PMT_NIL)), 300)
        self.assertLess(abs(pmt.to_double(pmt.dict_ref(meta, pmt.intern("fine_delay_samples"), pmt.PMT_NIL))), 0.25)
        self.assertGreater(pmt.to_double(pmt.dict_ref(meta, pmt.intern("peak_metric"), pmt.PMT_NIL)), 0.9)
        self.assertTrue(pmt.to_bool(pmt.dict_ref(meta, pmt.intern("valid"), pmt.PMT_F)))

    def test_coarse_zc_root_29_detects_and_labels_channel(self):
        tx = ofdm_prs_ranging.prs_timed_burst_source(
            attach_tx_time=False, coarse_zc_root=29)
        data = [0j] * 300 + list(tx.frame_samples()) + [0j] * 300
        timing = blocks.vector_source_c(data, False)
        detector = ofdm_prs_ranging.prs_frame_detector(
            threshold=0.30, coarse_zc_root=29, channel_id=1)
        debug = blocks.message_debug()

        self.tb.connect(timing, detector)
        self.tb.msg_connect((detector, "frame_out"), (debug, "store"))
        self.tb.run()

        self.assertGreaterEqual(debug.num_messages(), 1)
        meta = pmt.car(debug.get_message(0))
        self.assertEqual(pmt.to_long(pmt.dict_ref(meta, pmt.intern("coarse_zc_root"), pmt.PMT_NIL)), 29)
        self.assertEqual(pmt.to_long(pmt.dict_ref(meta, pmt.intern("channel_id"), pmt.PMT_NIL)), 1)
        self.assertGreater(pmt.to_double(pmt.dict_ref(meta, pmt.intern("coarse_metric"), pmt.PMT_NIL)), 0.9)

    def test_coarse_zc_root_mismatch_rejects_frame(self):
        tx = ofdm_prs_ranging.prs_timed_burst_source(
            attach_tx_time=False, coarse_zc_root=29)
        data = [0j] * 300 + list(tx.frame_samples()) + [0j] * 300
        timing = blocks.vector_source_c(data, False)
        detector = ofdm_prs_ranging.prs_frame_detector(
            threshold=0.30, coarse_zc_root=25)
        debug = blocks.message_debug()

        self.tb.connect(timing, detector)
        self.tb.msg_connect((detector, "frame_out"), (debug, "store"))
        self.tb.run()

        self.assertEqual(debug.num_messages(), 0)

    def test_csv_logger_writes_measurement_row(self):
        fd, path = tempfile.mkstemp(prefix="prs_meas_", suffix=".csv")
        os.close(fd)
        try:
            logger = ofdm_prs_ranging.prs_csv_logger(path, False)
            meta = pmt.make_dict()
            for key, value in (
                ("poll_frame_id", pmt.from_uint64(7)),
                ("response_frame_id", pmt.from_uint64(9)),
                ("t1_tx_time", pmt.from_double(100.0)),
                ("t4_rx_time", pmt.from_double(100.000011)),
                ("reply_delay_samples", pmt.from_uint64(50)),
                ("reply_delay_s", pmt.from_double(5.0e-6)),
                ("rtt_s", pmt.from_double(6.0e-6)),
                ("tof_s", pmt.from_double(3.0e-6)),
                ("range_m", pmt.from_double(899.377374)),
                ("frame_id_valid", pmt.PMT_T),
                ("peak_metric", pmt.from_double(0.95)),
                ("payload_metric", pmt.from_double(0.98)),
                ("phase_residual", pmt.from_double(0.01)),
                ("snr", pmt.from_double(30.0)),
                ("quality", pmt.from_double(0.9)),
            ):
                meta = pmt.dict_add(meta, pmt.intern(key), value)
            strobe = blocks.message_strobe(pmt.cons(meta, pmt.PMT_NIL), 50)
            self.tb.msg_connect((strobe, "strobe"), (logger, "measurement_in"))
            self.tb.start()
            time.sleep(0.12)
            self.tb.stop()
            self.tb.wait()

            with open(path, "r", encoding="utf-8") as f:
                lines = [line.strip() for line in f.readlines()]
            self.assertEqual(lines[0], "poll_frame_id,response_frame_id,t1_tx_time,t4_rx_time,reply_delay_samples,reply_delay_s,rtt_s,tof_s,range_m,frame_id_valid,peak_metric,payload_metric,phase_residual,snr,quality")
            fields = lines[1].split(",")
            self.assertEqual(fields[0], "7")
            self.assertEqual(fields[1], "9")
            self.assertAlmostEqual(float(fields[2]), 100.0)
            self.assertAlmostEqual(float(fields[3]), 100.000011)
            self.assertEqual(fields[4], "50")
        finally:
            if os.path.exists(path):
                os.unlink(path)

    def test_ssrtt_responder_schedules_response_trigger(self):
        responder = ofdm_prs_ranging.prs_ssrtt_responder(10e6, 50000)
        debug = blocks.message_debug()

        meta = pmt.make_dict()
        for key, value in (
            ("frame_id_valid", pmt.PMT_T),
            ("packet_type", pmt.from_long(1)),
            ("poll_frame_id", pmt.from_uint64(123)),
            ("frame_id", pmt.from_uint64(123)),
            ("rx_time", pmt.make_tuple(pmt.from_uint64(10), pmt.from_double(0.25))),
        ):
            meta = pmt.dict_add(meta, pmt.intern(key), value)

        strobe = blocks.message_strobe(pmt.cons(meta, pmt.PMT_NIL), 50)
        self.tb.msg_connect((strobe, "strobe"), (responder, "measurement_in"))
        self.tb.msg_connect((responder, "trigger_out"), (debug, "store"))
        self.tb.start()
        time.sleep(0.12)
        self.tb.stop()
        self.tb.wait()

        self.assertGreaterEqual(debug.num_messages(), 1)
        trigger = debug.get_message(0)
        self.assertEqual(pmt.to_long(pmt.dict_ref(trigger, pmt.intern("packet_type"), pmt.PMT_NIL)), 2)
        self.assertEqual(pmt.to_uint64(pmt.dict_ref(trigger, pmt.intern("poll_frame_id"), pmt.PMT_NIL)), 123)
        self.assertEqual(pmt.to_uint64(pmt.dict_ref(trigger, pmt.intern("reply_delay_samples"), pmt.PMT_NIL)), 50000)
        self.assertEqual(pmt.to_uint64(pmt.dict_ref(trigger, pmt.intern("tx_time_secs"), pmt.PMT_NIL)), 10)
        self.assertAlmostEqual(pmt.to_double(pmt.dict_ref(trigger, pmt.intern("tx_time_frac"), pmt.PMT_NIL)), 0.255)

    def test_ssrtt_solver_computes_range_from_synthetic_timestamps(self):
        solver = ofdm_prs_ranging.prs_ssrtt_solver(10e6)
        debug = blocks.message_debug()

        tx = pmt.make_dict()
        for key, value in (
            ("packet_type", pmt.from_long(1)),
            ("poll_frame_id", pmt.from_uint64(42)),
            ("frame_id", pmt.from_uint64(42)),
            ("tx_time_secs", pmt.from_uint64(100)),
            ("tx_time_frac", pmt.from_double(0.0)),
        ):
            tx = pmt.dict_add(tx, pmt.intern(key), value)

        rx = pmt.make_dict()
        for key, value in (
            ("frame_id_valid", pmt.PMT_T),
            ("packet_type", pmt.from_long(2)),
            ("poll_frame_id", pmt.from_uint64(42)),
            ("response_frame_id", pmt.from_uint64(77)),
            ("reply_delay_samples", pmt.from_uint64(50000)),
            ("rx_time", pmt.make_tuple(pmt.from_uint64(100), pmt.from_double(0.005002))),
            ("peak_metric", pmt.from_double(0.9)),
            ("payload_metric", pmt.from_double(1.0)),
            ("phase_residual", pmt.from_double(0.01)),
            ("snr", pmt.from_double(20.0)),
            ("quality", pmt.from_double(0.8)),
        ):
            rx = pmt.dict_add(rx, pmt.intern(key), value)

        tx_strobe = blocks.message_strobe(tx, 40)
        rx_strobe = blocks.message_strobe(pmt.cons(rx, pmt.PMT_NIL), 80)
        self.tb.msg_connect((tx_strobe, "strobe"), (solver, "tx_time_in"))
        self.tb.msg_connect((rx_strobe, "strobe"), (solver, "measurement_in"))
        self.tb.msg_connect((solver, "ssrtt_out"), (debug, "store"))
        self.tb.start()
        time.sleep(0.20)
        self.tb.stop()
        self.tb.wait()

        self.assertGreaterEqual(debug.num_messages(), 1)
        out = pmt.car(debug.get_message(0))
        self.assertAlmostEqual(pmt.to_double(pmt.dict_ref(out, pmt.intern("rtt_s"), pmt.PMT_NIL)), 2.0e-6)
        self.assertAlmostEqual(pmt.to_double(pmt.dict_ref(out, pmt.intern("tof_s"), pmt.PMT_NIL)), 1.0e-6)
        self.assertAlmostEqual(pmt.to_double(pmt.dict_ref(out, pmt.intern("range_m"), pmt.PMT_NIL)), 299.792458, places=5)


if __name__ == "__main__":
    gr_unittest.run(qa_prs_receiver)
