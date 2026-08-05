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
        self.assertGreater(pmt.to_double(pmt.dict_ref(meta, pmt.intern("coarse_metric"), pmt.PMT_NIL)), 0.9)
        self.assertTrue(pmt.is_null(
            pmt.dict_ref(meta, pmt.intern("peak_metric"), pmt.PMT_NIL)))
        self.assertTrue(pmt.to_bool(pmt.dict_ref(meta, pmt.intern("valid"), pmt.PMT_F)))

    def test_prs_cp_cfo_recovers_payload_from_biased_preamble_cfo(self):
        samp_rate = 30e6
        tx = ofdm_prs_ranging.prs_timed_burst_source(
            samp_rate=samp_rate,
            preamble_len=256,
            preamble_repeats=4,
            coarse_sync_len=419,
            attach_tx_time=False)
        frame = numpy.asarray(tx.frame_samples(), dtype=numpy.complex64)

        preamble_start = 1000
        preamble_samples = 256 * 4
        biased_cfo_hz = 350.0
        indices = numpy.arange(preamble_samples, dtype=numpy.float64)
        frame[preamble_start:preamble_start + preamble_samples] *= numpy.exp(
            1j * 2.0 * numpy.pi * biased_cfo_hz * indices / samp_rate)

        source = blocks.vector_source_c(
            numpy.concatenate((numpy.zeros(300, numpy.complex64),
                               frame,
                               numpy.zeros(300, numpy.complex64))),
            False)
        detector = ofdm_prs_ranging.prs_frame_detector(
            samp_rate=samp_rate,
            preamble_len=256,
            preamble_repeats=4,
            coarse_sync_len=419,
            threshold=0.30)
        debug = blocks.message_debug()
        self.tb.connect(source, detector)
        self.tb.msg_connect((detector, "frame_out"), (debug, "store"))
        self.tb.run()

        self.assertGreaterEqual(debug.num_messages(), 1)
        meta = pmt.car(debug.get_message(0))
        self.assertTrue(pmt.to_bool(pmt.dict_ref(
            meta, pmt.intern("frame_id_valid"), pmt.PMT_F)))
        self.assertTrue(pmt.to_bool(pmt.dict_ref(
            meta, pmt.intern("payload_retry_used"), pmt.PMT_F)))
        self.assertAlmostEqual(pmt.to_double(pmt.dict_ref(
            meta, pmt.intern("preamble_cfo_hz"), pmt.PMT_NIL)),
            biased_cfo_hz, delta=2.0)
        self.assertAlmostEqual(pmt.to_double(pmt.dict_ref(
            meta, pmt.intern("prs_cp_cfo_hz"), pmt.PMT_NIL)),
            0.0, delta=1.0)
        self.assertGreater(pmt.to_double(pmt.dict_ref(
            meta, pmt.intern("prs_cp_cfo_coherence"), pmt.PMT_NIL)),
            0.99)

    def test_preamble_and_prs_cp_cfo_track_both_signs(self):
        samp_rate = 30e6
        tx = ofdm_prs_ranging.prs_timed_burst_source(
            samp_rate=samp_rate, attach_tx_time=False)
        frame = numpy.asarray(tx.frame_samples(), dtype=numpy.complex64)
        bursts = []
        expected = (600.0, -850.0)
        for cfo_hz in expected:
            indices = numpy.arange(frame.size, dtype=numpy.float64)
            bursts.append(frame * numpy.exp(
                1j * 2.0 * numpy.pi * cfo_hz * indices / samp_rate))
        data = numpy.concatenate((numpy.zeros(300, numpy.complex64),
                                  bursts[0],
                                  numpy.zeros(12000, numpy.complex64),
                                  bursts[1],
                                  numpy.zeros(300, numpy.complex64)))

        source = blocks.vector_source_c(data, False)
        detector = ofdm_prs_ranging.prs_frame_detector(
            samp_rate=samp_rate, threshold=0.30)
        debug = blocks.message_debug()
        self.tb.connect(source, detector)
        self.tb.msg_connect((detector, "frame_out"), (debug, "store"))
        self.tb.run()

        self.assertEqual(debug.num_messages(), 2)
        for index, cfo_hz in enumerate(expected):
            meta = pmt.car(debug.get_message(index))
            self.assertAlmostEqual(pmt.to_double(pmt.dict_ref(
                meta, pmt.intern("preamble_cfo_hz"), pmt.PMT_NIL)),
                cfo_hz, delta=1.0)
            self.assertAlmostEqual(pmt.to_double(pmt.dict_ref(
                meta, pmt.intern("prs_cp_cfo_hz"), pmt.PMT_NIL)),
                cfo_hz, delta=1.0)
            self.assertGreater(pmt.to_double(pmt.dict_ref(
                meta, pmt.intern("prs_cp_cfo_coherence"), pmt.PMT_NIL)),
                0.99)

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

    def test_zc_refinement_corrects_fractional_delay_boundary(self):
        tx = ofdm_prs_ranging.prs_timed_burst_source(
            attach_tx_time=False)
        frame = numpy.asarray(tx.frame_samples(), dtype=numpy.complex64)
        prefix = 300
        undelayed = numpy.concatenate((
            numpy.zeros(prefix, dtype=numpy.complex64),
            frame,
            numpy.zeros(302, dtype=numpy.complex64)))

        fraction = 0.75
        delayed = numpy.zeros_like(undelayed)
        delayed[1:] = ((1.0 - fraction) * undelayed[1:] +
                       fraction * undelayed[:-1])
        source = blocks.vector_source_c(delayed, False)
        detector = ofdm_prs_ranging.prs_frame_detector(
            threshold=0.30, zc_threshold=0.20)
        debug = blocks.message_debug()

        self.tb.connect(source, detector)
        self.tb.msg_connect((detector, "frame_out"), (debug, "store"))
        self.tb.run()

        self.assertGreaterEqual(debug.num_messages(), 1)
        meta = pmt.car(debug.get_message(0))
        self.assertEqual(
            pmt.to_uint64(pmt.dict_ref(
                meta, pmt.intern("frame_start"), pmt.PMT_NIL)),
            prefix + 1)
        self.assertGreater(
            pmt.to_double(pmt.dict_ref(
                meta, pmt.intern("coarse_metric"), pmt.PMT_NIL)),
            0.70)

    def test_golay_channel_estimator_unity_channel(self):
        csv_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "..", "lib",
            "golay_ofdm_1024x16.csv"))
        rows = numpy.loadtxt(csv_path, delimiter=",", skiprows=1)
        native = (rows[:, 3] + 1j * rows[:, 4]).reshape(16, 1024)
        signed_order = numpy.concatenate(
            (native[:, 512:], native[:, :512]), axis=1).reshape(-1)

        estimator = ofdm_prs_ranging.prs_channel_estimator(
            10e6, 1024, 1024, 16, 13990001)
        debug = blocks.message_debug()
        symbols = pmt.init_c32vector(
            len(signed_order), [complex(value) for value in signed_order])
        strobe = blocks.message_strobe(
            pmt.cons(pmt.make_dict(), symbols), 50)
        self.tb.msg_connect(
            (strobe, "strobe"), (estimator, "symbols_in"))
        self.tb.msg_connect(
            (estimator, "channel_out"), (debug, "store"))
        self.tb.start()
        time.sleep(0.12)
        self.tb.stop()
        self.tb.wait()

        self.assertGreaterEqual(debug.num_messages(), 1)
        channel = numpy.asarray(
            pmt.c32vector_elements(pmt.cdr(debug.get_message(0))))
        self.assertEqual(channel.size, 1024)
        numpy.testing.assert_allclose(
            channel, numpy.ones(1024), rtol=1e-6, atol=1e-6)

    def test_channel_estimator_removes_prs_symbol_cfo_rotation(self):
        samp_rate = 30e6
        cfo_hz = 350.0
        delay_samples = 0.25
        csv_path = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "..", "..", "lib",
            "golay_ofdm_1024x16.csv"))
        rows = numpy.loadtxt(csv_path, delimiter=",", skiprows=1)
        native = (rows[:, 3] + 1j * rows[:, 4]).reshape(16, 1024)
        signed = numpy.concatenate((native[:, 512:], native[:, :512]), axis=1)
        frequencies = numpy.arange(-512, 512, dtype=numpy.float64) * samp_rate / 1024.0
        expected_channel = numpy.exp(
            -1j * 2.0 * numpy.pi * frequencies * delay_samples / samp_rate)
        symbol_period = (1024 + 128) / samp_rate
        rotations = numpy.exp(
            1j * 2.0 * numpy.pi * cfo_hz * symbol_period *
            numpy.arange(16, dtype=numpy.float64))
        received = (signed * expected_channel[None, :] *
                    rotations[:, None]).reshape(-1)

        estimator = ofdm_prs_ranging.prs_channel_estimator(
            samp_rate, 1024, 1024, 16, 13990001)
        debug = blocks.message_debug()
        phase = ofdm_prs_ranging.prs_phase_slope_estimator(
            samp_rate, 1024, 1024, 1.0)
        phase_debug = blocks.message_debug()
        meta = pmt.dict_add(pmt.make_dict(),
                            pmt.intern("prs_cp_cfo_hz"),
                            pmt.from_double(cfo_hz))
        strobe = blocks.message_strobe(
            pmt.cons(meta, pmt.init_c32vector(
                len(received), [complex(value) for value in received])), 50)
        self.tb.msg_connect((strobe, "strobe"), (estimator, "symbols_in"))
        self.tb.msg_connect((estimator, "channel_out"), (debug, "store"))
        self.tb.msg_connect((estimator, "channel_out"), (phase, "channel_in"))
        self.tb.msg_connect((phase, "measurement_out"), (phase_debug, "store"))
        self.tb.start()
        time.sleep(0.12)
        self.tb.stop()
        self.tb.wait()

        self.assertGreaterEqual(debug.num_messages(), 1)
        msg = debug.get_message(0)
        result = pmt.car(msg)
        self.assertAlmostEqual(pmt.to_double(pmt.dict_ref(
            result, pmt.intern("prs_channel_cfo_hz"), pmt.PMT_NIL)),
            cfo_hz, delta=0.05)
        self.assertGreater(pmt.to_double(pmt.dict_ref(
            result, pmt.intern("channel_coherence"), pmt.PMT_NIL)), 0.999)
        channel = numpy.asarray(pmt.c32vector_elements(pmt.cdr(msg)))
        numpy.testing.assert_allclose(
            channel, expected_channel, rtol=1e-5, atol=1e-5)
        self.assertGreaterEqual(phase_debug.num_messages(), 1)
        phase_meta = pmt.car(phase_debug.get_message(0))
        self.assertAlmostEqual(pmt.to_double(pmt.dict_ref(
            phase_meta, pmt.intern("fine_delay_samples"), pmt.PMT_NIL)),
            delay_samples, delta=1e-4)

    def test_phase_slope_known_fractional_delay_full_band(self):
        samp_rate = 10e6
        delay_samples = 0.25
        signed_bins = numpy.arange(-512, 512, dtype=numpy.float64)
        frequencies = signed_bins * samp_rate / 1024.0
        channel = numpy.exp(
            -1j * 2.0 * numpy.pi * frequencies *
            (delay_samples / samp_rate))

        estimator = ofdm_prs_ranging.prs_phase_slope_estimator(
            samp_rate, 1024, 1024, 1.0)
        debug = blocks.message_debug()
        meta = pmt.make_dict()
        meta = pmt.dict_add(
            meta, pmt.intern("snr"), pmt.from_double(30.0))
        meta = pmt.dict_add(
            meta, pmt.intern("coarse_metric"), pmt.from_double(1.0))
        vector = pmt.init_c32vector(
            len(channel), [complex(value) for value in channel])
        strobe = blocks.message_strobe(pmt.cons(meta, vector), 50)
        self.tb.msg_connect(
            (strobe, "strobe"), (estimator, "channel_in"))
        self.tb.msg_connect(
            (estimator, "measurement_out"), (debug, "store"))
        self.tb.start()
        time.sleep(0.12)
        self.tb.stop()
        self.tb.wait()

        self.assertGreaterEqual(debug.num_messages(), 1)
        result = pmt.car(debug.get_message(0))
        estimate = pmt.to_double(pmt.dict_ref(
            result, pmt.intern("fine_delay_samples"), pmt.PMT_NIL))
        residual = pmt.to_double(pmt.dict_ref(
            result, pmt.intern("phase_residual"), pmt.PMT_NIL))
        self.assertAlmostEqual(estimate, delay_samples, places=5)
        self.assertLess(residual, 1e-5)

    def test_time_gating_only_detects_frame_inside_scheduled_window(self):
        samp_rate = 1e6
        tx = ofdm_prs_ranging.prs_timed_burst_source(
            samp_rate=samp_rate, attach_tx_time=False)
        frame = list(tx.frame_samples())
        target_start = 60000
        data = [0j] * 90000
        data[1000:1000 + len(frame)] = frame
        data[target_start:target_start + len(frame)] = frame

        rx_time = pmt.make_tuple(
            pmt.from_uint64(100), pmt.from_double(0.0))
        tags = [gr.tag_utils.python_to_tag(
            (0, pmt.intern("rx_time"), rx_time, pmt.intern("qa")))]
        timing = blocks.vector_source_c(data, False, 1, tags)
        throttle = blocks.throttle(gr.sizeof_gr_complex, samp_rate)
        detector = ofdm_prs_ranging.prs_frame_detector(
            samp_rate=samp_rate,
            threshold=0.30,
            time_gating=True,
            reply_delay_s=0.0,
            window_before_s=0.001,
            window_after_s=0.080)
        debug = blocks.message_debug()
        event_debug = blocks.message_debug()

        tx_meta = pmt.make_dict()
        tx_meta = pmt.dict_add(
            tx_meta, pmt.intern("attempt_id"), pmt.from_uint64(314))
        tx_meta = pmt.dict_add(
            tx_meta, pmt.intern("tx_time_secs"), pmt.from_uint64(100))
        tx_meta = pmt.dict_add(
            tx_meta, pmt.intern("tx_time_frac"), pmt.from_double(0.060))

        self.tb.connect(timing, throttle, detector)
        self.tb.msg_connect((detector, "frame_out"), (debug, "store"))
        self.tb.msg_connect((detector, "event_out"), (event_debug, "store"))
        self.tb.start()
        detector.to_basic_block()._post(pmt.intern("tx_time_in"), tx_meta)
        time.sleep(0.15)
        self.tb.stop()
        self.tb.wait()

        self.assertEqual(debug.num_messages(), 1)
        self.assertEqual(event_debug.num_messages(), 1)
        meta = pmt.car(debug.get_message(0))
        self.assertEqual(
            pmt.to_uint64(
                pmt.dict_ref(
                    meta, pmt.intern("frame_start"), pmt.PMT_NIL)),
            target_start)
        self.assertEqual(
            pmt.to_uint64(
                pmt.dict_ref(
                    meta, pmt.intern("attempt_id"), pmt.PMT_NIL)),
            314)
        self.assertEqual(
            pmt.symbol_to_string(
                pmt.dict_ref(
                    meta, pmt.intern("failure_reason"), pmt.PMT_NIL)),
            "NONE")

    def test_time_gating_logs_no_preamble_attempt(self):
        samp_rate = 1e6
        data = [0j] * 50000
        rx_time = pmt.make_tuple(
            pmt.from_uint64(100), pmt.from_double(0.0))
        tags = [gr.tag_utils.python_to_tag(
            (0, pmt.intern("rx_time"), rx_time, pmt.intern("qa")))]
        timing = blocks.vector_source_c(data, False, 1, tags)
        throttle = blocks.throttle(gr.sizeof_gr_complex, samp_rate)
        detector = ofdm_prs_ranging.prs_frame_detector(
            samp_rate=samp_rate,
            threshold=0.30,
            time_gating=True,
            reply_delay_s=0.0,
            window_before_s=0.001,
            window_after_s=0.010)
        event_debug = blocks.message_debug()

        tx_meta = pmt.make_dict()
        for key, value in (
            ("attempt_id", pmt.from_uint64(2718)),
            ("tx_time_secs", pmt.from_uint64(100)),
            ("tx_time_frac", pmt.from_double(0.020)),
        ):
            tx_meta = pmt.dict_add(tx_meta, pmt.intern(key), value)

        self.tb.connect(timing, throttle, detector)
        self.tb.msg_connect((detector, "event_out"), (event_debug, "store"))
        self.tb.start()
        detector.to_basic_block()._post(pmt.intern("tx_time_in"), tx_meta)
        time.sleep(0.10)
        self.tb.stop()
        self.tb.wait()

        self.assertEqual(event_debug.num_messages(), 1)
        meta = pmt.car(event_debug.get_message(0))
        self.assertEqual(
            pmt.to_uint64(
                pmt.dict_ref(
                    meta, pmt.intern("attempt_id"), pmt.PMT_NIL)),
            2718)
        self.assertEqual(
            pmt.symbol_to_string(
                pmt.dict_ref(
                    meta, pmt.intern("failure_reason"), pmt.PMT_NIL)),
            "NO_PREAMBLE")

    def test_time_gating_logs_zc_sync_failure(self):
        samp_rate = 1e6
        tx = ofdm_prs_ranging.prs_timed_burst_source(
            samp_rate=samp_rate,
            attach_tx_time=False,
            coarse_zc_root=29)
        frame = list(tx.frame_samples())
        target_start = 20000
        data = [0j] * 120000
        data[target_start:target_start + len(frame)] = frame
        rx_time = pmt.make_tuple(
            pmt.from_uint64(100), pmt.from_double(0.0))
        tags = [gr.tag_utils.python_to_tag(
            (0, pmt.intern("rx_time"), rx_time, pmt.intern("qa")))]
        timing = blocks.vector_source_c(data, False, 1, tags)
        throttle = blocks.throttle(gr.sizeof_gr_complex, samp_rate)
        detector = ofdm_prs_ranging.prs_frame_detector(
            samp_rate=samp_rate,
            threshold=0.30,
            coarse_zc_root=25,
            time_gating=True,
            reply_delay_s=0.0,
            window_before_s=0.001,
            window_after_s=0.080)
        event_debug = blocks.message_debug()

        tx_meta = pmt.make_dict()
        for key, value in (
            ("attempt_id", pmt.from_uint64(1618)),
            ("tx_time_secs", pmt.from_uint64(100)),
            ("tx_time_frac", pmt.from_double(0.020)),
        ):
            tx_meta = pmt.dict_add(tx_meta, pmt.intern(key), value)

        self.tb.connect(timing, throttle, detector)
        self.tb.msg_connect((detector, "event_out"), (event_debug, "store"))
        self.tb.start()
        detector.to_basic_block()._post(pmt.intern("tx_time_in"), tx_meta)
        time.sleep(0.25)
        self.tb.stop()
        self.tb.wait()

        self.assertEqual(event_debug.num_messages(), 1)
        meta = pmt.car(event_debug.get_message(0))
        self.assertEqual(
            pmt.to_uint64(
                pmt.dict_ref(
                    meta, pmt.intern("attempt_id"), pmt.PMT_NIL)),
            1618)
        self.assertEqual(
            pmt.symbol_to_string(
                pmt.dict_ref(
                    meta, pmt.intern("failure_reason"), pmt.PMT_NIL)),
            "ZC_SYNC")

    def test_acquisition_logger_writes_attempt_failure_columns(self):
        fd, path = tempfile.mkstemp(prefix="prs_acq_", suffix=".csv")
        os.close(fd)
        try:
            logger = ofdm_prs_ranging.prs_acquisition_logger(
                path, "initiator", False)
            meta = pmt.make_dict()
            for key, value in (
                ("attempt_id", pmt.from_uint64(55)),
                ("failure_reason", pmt.intern("ZC_SYNC")),
                ("channel_id", pmt.from_long(1)),
                ("coarse_zc_root", pmt.from_long(29)),
                ("recv_id", pmt.from_uint64(77)),
                ("frame_id", pmt.from_uint64(88)),
                ("poll_frame_id", pmt.from_uint64(55)),
                ("response_frame_id", pmt.from_uint64(66)),
                ("rx_time_tag_offset", pmt.from_uint64(1000)),
                ("absolute_sample_index", pmt.from_uint64(2000)),
                ("frame_start", pmt.from_uint64(2000)),
                ("coarse_peak", pmt.from_uint64(5048)),
                ("frame_id_valid", pmt.PMT_F),
            ):
                meta = pmt.dict_add(meta, pmt.intern(key), value)
            strobe = blocks.message_strobe(
                pmt.cons(meta, pmt.PMT_NIL), 50)
            self.tb.msg_connect(
                (strobe, "strobe"), (logger, "frame_in"))
            self.tb.start()
            time.sleep(0.12)
            self.tb.stop()
            self.tb.wait()

            with open(path, "r", encoding="utf-8") as f:
                lines = [line.strip() for line in f.readlines()]
            header = lines[0].split(",")
            fields = lines[1].split(",")
            self.assertEqual(header, [
                "log_time_unix", "node", "attempt_id", "failure_reason",
                "channel_id", "coarse_zc_root", "packet_type",
                "poll_frame_id", "response_frame_id", "reply_delay_samples",
                "frame_id_valid", "rx_time", "preamble_metric",
                "coarse_metric", "payload_metric", "cfo",
                "preamble_cfo_hz", "prs_cp_cfo_hz",
                "prs_cp_cfo_coherence", "selected_cfo_hz",
                "payload_retry_used", "samp_rate",
                "fft_len", "cp_len", "active_bins", "prs_symbols",
                "prs_start_rel", "prs_len", "pdu_len",
            ])
            self.assertEqual(header[2], "attempt_id")
            self.assertEqual(header[3], "failure_reason")
            self.assertNotIn("peak_metric", header)
            self.assertIn("coarse_metric", header)
            for removed in (
                "rx_time_tag_offset",
                "absolute_sample_index",
                "frame_start",
                "coarse_peak",
                "coarse_offset_samples",
                "recv_id",
                "frame_id",
            ):
                self.assertNotIn(removed, header)
            for retained in (
                "attempt_id",
                "poll_frame_id",
                "response_frame_id",
            ):
                self.assertIn(retained, header)
            self.assertEqual(len(header), len(fields))
            self.assertEqual(fields[1], "initiator")
            self.assertEqual(fields[2], "55")
            self.assertEqual(fields[3], "ZC_SYNC")
            self.assertEqual(fields[7], "55")
            self.assertEqual(fields[8], "66")
        finally:
            if os.path.exists(path):
                os.unlink(path)

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
                ("coarse_metric", pmt.from_double(0.95)),
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
            header = lines[0].split(",")
            fields = lines[1].split(",")
            self.assertEqual(len(header), len(fields))
            self.assertEqual(fields[header.index("poll_frame_id")], "7")
            self.assertEqual(fields[header.index("response_frame_id")], "9")
            self.assertAlmostEqual(float(fields[header.index("t1_tx_time")]), 100.0)
            self.assertAlmostEqual(float(fields[header.index("t4_rx_time")]), 100.000011)
            self.assertEqual(fields[header.index("reply_delay_samples")], "50")
            for name in (
                    "preamble_cfo_hz", "prs_cp_cfo_hz",
                    "prs_channel_cfo_hz", "phase_slope_rad_per_hz",
                    "fine_delay_samples", "phase_range_contribution_m"):
                self.assertIn(name, header)
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
            ("rx_time", pmt.make_tuple(pmt.from_uint64(100), pmt.from_double(0.0050136))),
            ("fine_delay", pmt.from_double(25.0e-9)),
            ("fine_delay_samples", pmt.from_double(0.25)),
            ("coarse_metric", pmt.from_double(0.9)),
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
        self.assertAlmostEqual(pmt.to_double(pmt.dict_ref(out, pmt.intern("rtt_s"), pmt.PMT_NIL)), 13.6e-6)
        self.assertAlmostEqual(pmt.to_double(pmt.dict_ref(out, pmt.intern("tof_s"), pmt.PMT_NIL)), 1.0e-6)
        self.assertAlmostEqual(pmt.to_double(pmt.dict_ref(out, pmt.intern("range_m"), pmt.PMT_NIL)), 299.792458, places=5)
        self.assertAlmostEqual(pmt.to_double(pmt.dict_ref(
            out, pmt.intern("integer_range_m"), pmt.PMT_NIL)),
            299.792458, places=5)
        self.assertAlmostEqual(pmt.to_double(pmt.dict_ref(
            out, pmt.intern("response_phase_range_correction_m"), pmt.PMT_NIL)),
            3.747405725, places=6)


if __name__ == "__main__":
    gr_unittest.run(qa_prs_receiver)
