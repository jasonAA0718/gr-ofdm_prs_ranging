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


def load_golay_prs_native():
    golay_a = numpy.ones(1, dtype=numpy.float32)
    golay_b = numpy.ones(1, dtype=numpy.float32)
    while golay_a.size < 512:
        previous_a = golay_a
        previous_b = golay_b
        golay_a = numpy.concatenate((previous_a, previous_b))
        golay_b = numpy.concatenate((previous_a, -previous_b))
    return numpy.stack(
        [golay_a if symbol % 2 == 0 else golay_b for symbol in range(127)]
    ).astype(numpy.complex64)


def load_gold_code(code_id=2):
    csv_path = os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..", "..", "lib", "DSP",
        "gold127_family_bipolar.csv"))
    return numpy.loadtxt(csv_path, delimiter=",", dtype=numpy.float32)[code_id]


MC_DS_CODE = load_gold_code(2)


def mc_ds_pilot_mask_native():
    return numpy.arange(512) % 4 != 3


def mc_ds_pilot_frequencies(samp_rate):
    ordered_native = (numpy.arange(512) + 256) % 512
    signed_bins = numpy.arange(-256, 256, dtype=numpy.float64)
    return signed_bins[ordered_native % 4 != 3] * samp_rate / 512.0


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

    def test_moderate_snr_mc_ds_payload_recovery(self):
        tx = ofdm_prs_ranging.prs_timed_burst_source(attach_tx_time=False)
        frame = numpy.asarray(tx.frame_samples(), dtype=numpy.complex64)
        active = frame[1000:-1000]
        signal_power = float(numpy.mean(numpy.abs(active) ** 2))
        noise_power = signal_power / (10.0 ** (15.0 / 10.0))
        rng = numpy.random.default_rng(20260905)
        noise = numpy.sqrt(noise_power / 2.0) * (
            rng.standard_normal(frame.size) + 1j * rng.standard_normal(frame.size))
        samples = numpy.concatenate((numpy.zeros(300, numpy.complex64),
                                     frame + noise.astype(numpy.complex64),
                                     numpy.zeros(300, numpy.complex64)))

        source = blocks.vector_source_c(samples, False)
        detector = ofdm_prs_ranging.prs_frame_detector(threshold=0.30)
        fft = ofdm_prs_ranging.prs_fft_receiver()
        channel = ofdm_prs_ranging.prs_channel_estimator()
        debug = blocks.message_debug()
        self.tb.connect(source, detector)
        self.tb.msg_connect((detector, "frame_out"), (fft, "frame_in"))
        self.tb.msg_connect((fft, "symbols_out"), (channel, "symbols_in"))
        self.tb.msg_connect((channel, "event_out"), (debug, "store"))
        self.tb.run()

        self.assertGreaterEqual(debug.num_messages(), 1)
        meta = pmt.car(debug.get_message(0))
        self.assertTrue(pmt.to_bool(pmt.dict_ref(
            meta, pmt.intern("frame_id_valid"), pmt.PMT_F)))
        self.assertEqual(pmt.symbol_to_string(pmt.dict_ref(
            meta, pmt.intern("failure_reason"), pmt.PMT_NIL)), "NONE")

    def test_gold127_processing_gain_and_wrong_code_rejection(self):
        rng = numpy.random.default_rng(12702)
        trials = 5000
        noise = (rng.standard_normal((trials, 127)) +
                 1j * rng.standard_normal((trials, 127)))
        observations = MC_DS_CODE[None, :] + noise
        estimate_8 = numpy.mean(
            observations[:, :8] * MC_DS_CODE[None, :8], axis=1)
        estimate_127 = numpy.mean(
            observations * MC_DS_CODE[None, :], axis=1)
        mse_8 = float(numpy.mean(numpy.abs(estimate_8 - 1.0) ** 2))
        mse_127 = float(numpy.mean(numpy.abs(estimate_127 - 1.0) ** 2))
        self.assertGreater(mse_8 / mse_127, 12.0)

        wrong_code = load_gold_code(3)
        normalized_cross_correlation = abs(
            float(numpy.dot(MC_DS_CODE, wrong_code))) / 127.0
        self.assertLess(normalized_cross_correlation, 0.25)

        with self.assertRaises(ValueError):
            ofdm_prs_ranging.prs_channel_estimator(mc_ds_gold_code_id=-1)
        with self.assertRaises(ValueError):
            ofdm_prs_ranging.prs_channel_estimator(mc_ds_gold_code_id=129)

    def test_sequential_poll_ids_round_trip(self):
        count = 3
        tx = ofdm_prs_ranging.prs_timed_burst_source(
            pings_per_trigger=count, attach_tx_time=False)
        frame_len = tx.frame_len()
        rx_time = pmt.make_tuple(pmt.from_uint64(0), pmt.from_double(0.0))
        tags = [gr.tag_utils.python_to_tag(
            (0, pmt.intern("rx_time"), rx_time, pmt.intern("qa")))]
        timing = blocks.vector_source_c(
            [0j] * (count * frame_len + 1000), False, 1, tags)
        head = blocks.head(gr.sizeof_gr_complex, count * frame_len)
        sink = blocks.vector_sink_c()
        self.tb.connect(timing, tx, head, sink)
        tx.to_basic_block()._post(pmt.intern("trigger"), pmt.PMT_T)
        self.tb.run()
        self.assertEqual(len(sink.data()), count * frame_len)

        self.tb = gr.top_block()
        source = blocks.vector_source_c(
            [0j] * 300 + list(sink.data()) + [0j] * 300, False)
        detector = ofdm_prs_ranging.prs_frame_detector(threshold=0.30)
        fft = ofdm_prs_ranging.prs_fft_receiver()
        channel = ofdm_prs_ranging.prs_channel_estimator()
        debug = blocks.message_debug()
        self.tb.connect(source, detector)
        self.tb.msg_connect((detector, "frame_out"), (fft, "frame_in"))
        self.tb.msg_connect((fft, "symbols_out"), (channel, "symbols_in"))
        self.tb.msg_connect((channel, "event_out"), (debug, "store"))
        self.tb.run()

        frame_ids = []
        for index in range(debug.num_messages()):
            meta = pmt.car(debug.get_message(index))
            if pmt.to_bool(pmt.dict_ref(
                    meta, pmt.intern("frame_id_valid"), pmt.PMT_F)):
                frame_ids.append(pmt.to_uint64(pmt.dict_ref(
                    meta, pmt.intern("poll_id"), pmt.PMT_NIL)))
        self.assertEqual(frame_ids, [0, 1, 2])

    def test_triggered_response_payload_round_trip(self):
        tx = ofdm_prs_ranging.prs_timed_burst_source(attach_tx_time=False)
        head = blocks.head(gr.sizeof_gr_complex, tx.frame_len())
        sink = blocks.vector_sink_c()
        self.tb.connect(tx, head, sink)
        trigger = pmt.make_dict()
        for key, value in (
            ("packet_type", pmt.from_long(2)),
            ("poll_id", pmt.from_uint64(42)),
            ("anchor_id", pmt.from_uint64(7)),
            ("reply_delay_samples", pmt.from_uint64(1500000)),
            ("poll_fine_delay_q10", pmt.from_long(-384)),
            ("fine_delay_valid", pmt.PMT_T),
            ("tx_time_secs", pmt.from_uint64(0)),
            ("tx_time_frac", pmt.from_double(0.0)),
        ):
            trigger = pmt.dict_add(trigger, pmt.intern(key), value)
        self.tb.start()
        tx.to_basic_block()._post(pmt.intern("trigger"), trigger)
        deadline = time.monotonic() + 2.0
        while len(sink.data()) < tx.frame_len() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.tb.stop()
        self.tb.wait()
        self.assertEqual(len(sink.data()), tx.frame_len())

        self.tb = gr.top_block()
        source = blocks.vector_source_c(
            [0j] * 300 + list(sink.data()) + [0j] * 300, False)
        detector = ofdm_prs_ranging.prs_frame_detector(threshold=0.30)
        fft = ofdm_prs_ranging.prs_fft_receiver()
        channel = ofdm_prs_ranging.prs_channel_estimator()
        debug = blocks.message_debug()
        self.tb.connect(source, detector)
        self.tb.msg_connect((detector, "frame_out"), (fft, "frame_in"))
        self.tb.msg_connect((fft, "symbols_out"), (channel, "symbols_in"))
        self.tb.msg_connect((channel, "event_out"), (debug, "store"))
        self.tb.run()

        self.assertGreaterEqual(debug.num_messages(), 1)
        meta = pmt.car(debug.get_message(0))
        self.assertTrue(pmt.to_bool(pmt.dict_ref(
            meta, pmt.intern("frame_id_valid"), pmt.PMT_F)))
        self.assertEqual(pmt.to_long(pmt.dict_ref(
            meta, pmt.intern("packet_type"), pmt.PMT_NIL)), 2)
        self.assertEqual(pmt.to_uint64(pmt.dict_ref(
            meta, pmt.intern("poll_id"), pmt.PMT_NIL)), 42)
        self.assertEqual(pmt.to_uint64(pmt.dict_ref(
            meta, pmt.intern("anchor_id"), pmt.PMT_NIL)), 7)
        self.assertTrue(pmt.to_bool(pmt.dict_ref(meta, pmt.intern("fine_delay_valid"), pmt.PMT_F)))
        self.assertAlmostEqual(pmt.to_double(pmt.dict_ref(
            meta, pmt.intern("poll_fine_delay_samples"), pmt.PMT_NIL)), -0.375)
        self.assertEqual(pmt.to_uint64(pmt.dict_ref(
            meta, pmt.intern("reply_delay_samples"), pmt.PMT_NIL)), 1500000)

    def test_prs_cp_cfo_does_not_depend_on_preamble_cfo(self):
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
        self.assertTrue(pmt.is_null(pmt.dict_ref(
            meta, pmt.intern("preamble_cfo_hz"), pmt.PMT_NIL)))
        self.assertAlmostEqual(pmt.to_double(pmt.dict_ref(
            meta, pmt.intern("prs_cp_cfo_hz"), pmt.PMT_NIL)),
            0.0, delta=1.0)
        self.assertGreater(pmt.to_double(pmt.dict_ref(
            meta, pmt.intern("prs_cp_cfo_coherence"), pmt.PMT_NIL)),
            0.99)

    def test_prs_cp_cfo_tracks_both_signs(self):
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
            self.assertTrue(pmt.is_null(pmt.dict_ref(
                meta, pmt.intern("preamble_cfo_hz"), pmt.PMT_NIL)))
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

    def test_mc_ds_channel_estimator_unity_channel(self):
        native = load_golay_prs_native().copy()
        native *= MC_DS_CODE[:, None]
        signed_order = numpy.concatenate(
            (native[:, 256:], native[:, :256]), axis=1).reshape(-1)

        estimator = ofdm_prs_ranging.prs_channel_estimator(
            10e6, 512, 512, 127, 13990001, False, 2)
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
        self.assertEqual(channel.size, 384)
        numpy.testing.assert_allclose(
            channel, numpy.ones(384), rtol=1e-6, atol=1e-6)

    def test_channel_estimator_removes_prs_symbol_cfo_rotation(self):
        samp_rate = 30e6
        cfo_hz = 350.0
        delay_samples = 0.25
        native = load_golay_prs_native().copy()
        native *= MC_DS_CODE[:, None]
        signed = numpy.concatenate((native[:, 256:], native[:, :256]), axis=1)
        frequencies = numpy.arange(-256, 256, dtype=numpy.float64) * samp_rate / 512.0
        expected_channel = numpy.exp(
            -1j * 2.0 * numpy.pi * frequencies * delay_samples / samp_rate)
        symbol_period = (512 + 64) / samp_rate
        rotations = numpy.exp(
            1j * 2.0 * numpy.pi * cfo_hz * symbol_period *
            numpy.arange(127, dtype=numpy.float64))
        received = (signed * expected_channel[None, :] *
                    rotations[:, None]).reshape(-1)

        estimator = ofdm_prs_ranging.prs_channel_estimator(
            samp_rate, 512, 512, 127, 13990001, False, 2)
        debug = blocks.message_debug()
        phase = ofdm_prs_ranging.prs_phase_slope_estimator(
            samp_rate, 512, 512, 1.0)
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
        expected_pilots = expected_channel[
            ((numpy.arange(512) + 256) % 512) % 4 != 3]
        numpy.testing.assert_allclose(
            channel, expected_pilots, rtol=1e-5, atol=1e-5)
        self.assertGreaterEqual(phase_debug.num_messages(), 1)
        phase_meta = pmt.car(phase_debug.get_message(0))
        self.assertAlmostEqual(pmt.to_double(pmt.dict_ref(
            phase_meta, pmt.intern("fine_delay_samples"), pmt.PMT_NIL)),
            delay_samples, delta=1e-4)

    def test_phase_slope_known_fractional_delay_full_band(self):
        samp_rate = 10e6
        delay_samples = 0.25
        frequencies = mc_ds_pilot_frequencies(samp_rate)
        channel = numpy.exp(
            -1j * 2.0 * numpy.pi * frequencies *
            (delay_samples / samp_rate))

        estimator = ofdm_prs_ranging.prs_phase_slope_estimator(
            samp_rate, 512, 512, 1.0)
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
        self.assertEqual(event_debug.num_messages(), 0)
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
        self.assertTrue(pmt.is_null(pmt.dict_ref(
            meta, pmt.intern("failure_reason"), pmt.PMT_NIL)))

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
        time.sleep(0.50)
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
                ("poll_id", pmt.from_uint64(55)),
                ("anchor_id", pmt.from_uint64(6)),
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
                "poll_id", "anchor_id", "reply_delay_samples",
                "frame_id_valid", "rx_time", "preamble_metric",
                "coarse_metric", "zc_peak_ratio", "zc_gate_offset_samples",
                "payload_initial_valid", "payload_initial_metric",
                "payload_retry_valid", "payload_retry_metric", "payload_metric", "cfo",
                "prs_cp_cfo_hz",
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
                "poll_id",
                "anchor_id",
            ):
                self.assertIn(retained, header)
            self.assertEqual(len(header), len(fields))
            self.assertEqual(fields[1], "initiator")
            self.assertEqual(fields[2], "55")
            self.assertEqual(fields[3], "ZC_SYNC")
            self.assertEqual(fields[7], "55")
            self.assertEqual(fields[8], "6")
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
                ("poll_id", pmt.from_uint64(7)),
                ("anchor_id", pmt.from_uint64(3)),
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
            self.assertEqual(fields[header.index("poll_id")], "7")
            self.assertEqual(fields[header.index("anchor_id")], "3")
            self.assertAlmostEqual(float(fields[header.index("t1_tx_time")]), 100.0)
            self.assertAlmostEqual(float(fields[header.index("t4_rx_time")]), 100.000011)
            self.assertEqual(fields[header.index("reply_delay_samples")], "50")
            for name in (
                    "prs_cp_cfo_hz",
                    "prs_channel_cfo_hz", "phase_slope_rad_per_hz",
                    "fine_delay_samples", "phase_range_contribution_m"):
                self.assertIn(name, header)
        finally:
            if os.path.exists(path):
                os.unlink(path)

    @staticmethod
    def _v2_meta(values):
        meta = pmt.make_dict()
        for key, value in values.items():
            if isinstance(value, bool):
                encoded = pmt.PMT_T if value else pmt.PMT_F
            elif isinstance(value, int):
                encoded = pmt.from_long(value)
            elif isinstance(value, float):
                encoded = pmt.from_double(value)
            else:
                encoded = value
            meta = pmt.dict_add(meta, pmt.intern(key), encoded)
        return meta

    def _await_messages(self, debug, count):
        deadline = time.monotonic() + 3.0
        while debug.num_messages() < count and time.monotonic() < deadline:
            time.sleep(0.002)
        self.assertEqual(debug.num_messages(), count)

    def test_v2_responder_quantization_validity_and_bounds(self):
        for anchor in (8, 255, 256):
            with self.assertRaises(ValueError):
                ofdm_prs_ranging.prs_ssrtt_responder(anchor_id=anchor)
        with self.assertRaises(ValueError):
            ofdm_prs_ranging.prs_ssrtt_responder(reply_delay_samples=0x1000000)
        responder = ofdm_prs_ranging.prs_ssrtt_responder(30e6, 1500000, False, 7)
        debug = blocks.message_debug()
        self.tb.msg_connect(responder, "trigger_out", debug, "store")
        cases = [(0.25, True, True, 256), (-0.375, True, True, -384),
                 (-32.0, True, True, -32768), (32767 / 1024.0, True, True, 32767),
                 (0.0, False, False, 0), (32.0, True, False, 0),
                 (-32.001, True, False, 0)]
        cases += [(value, True, False, 0) for value in (float('nan'), float('inf'), -float('inf'))]
        self.tb.start()
        try:
            for case_index, (samples, phase_valid, _, _) in enumerate(cases):
                meta = self._v2_meta(dict(frame_id_valid=True, packet_type=1, poll_id=65535 - case_index,
                    fine_delay_samples=samples, valid=phase_valid,
                    rx_time=pmt.make_tuple(pmt.from_uint64(10), pmt.from_double(0.25))))
                responder.to_basic_block()._post(pmt.intern("measurement_in"), pmt.cons(meta, pmt.PMT_NIL))
            self._await_messages(debug, len(cases))
            for index, (_, _, expected_valid, q) in enumerate(cases):
                trigger = debug.get_message(index)
                self.assertEqual(pmt.to_uint64(pmt.dict_ref(trigger, pmt.intern("anchor_id"), pmt.PMT_NIL)), 7)
                self.assertEqual(pmt.to_uint64(pmt.dict_ref(trigger, pmt.intern("poll_id"), pmt.PMT_NIL)), 65535 - index)
                self.assertEqual(pmt.to_long(pmt.dict_ref(trigger, pmt.intern("poll_fine_delay_q10"), pmt.PMT_NIL)), q)
                self.assertEqual(pmt.to_bool(pmt.dict_ref(trigger, pmt.intern("fine_delay_valid"), pmt.PMT_F)), expected_valid)
        finally:
            self.tb.stop()
            self.tb.wait()

    def test_v2_one_anchor_complete_software_chain(self):
        fs = 30e6
        poll = ofdm_prs_ranging.prs_timed_burst_source(samp_rate=fs, attach_tx_time=False)
        response = ofdm_prs_ranging.prs_timed_burst_source(samp_rate=fs, attach_tx_time=False,
                                                         coarse_zc_root=29)
        rx_tag = gr.tag_utils.python_to_tag((0, pmt.intern("rx_time"),
            pmt.make_tuple(pmt.from_uint64(10), pmt.from_double(0.0)), pmt.intern("qa")))
        source = blocks.vector_source_c(list(poll.frame_samples()), False, 1, [rx_tag])
        detector = ofdm_prs_ranging.prs_frame_detector(samp_rate=fs, threshold=0.30)
        fft = ofdm_prs_ranging.prs_fft_receiver(samp_rate=fs)
        channel = ofdm_prs_ranging.prs_channel_estimator(samp_rate=fs)
        phase = ofdm_prs_ranging.prs_phase_slope_estimator(samp_rate=fs)
        responder = ofdm_prs_ranging.prs_ssrtt_responder(fs, 1500000, False, 7)
        trigger_debug = blocks.message_debug()
        sink = blocks.vector_sink_c()
        head = blocks.head(gr.sizeof_gr_complex, response.frame_len())
        self.tb.connect(source, detector)
        self.tb.msg_connect(detector, "frame_out", fft, "frame_in")
        self.tb.msg_connect(fft, "symbols_out", channel, "symbols_in")
        self.tb.msg_connect(channel, "channel_out", phase, "channel_in")
        self.tb.msg_connect(phase, "measurement_out", responder, "measurement_in")
        self.tb.msg_connect(responder, "trigger_out", response, "trigger")
        self.tb.msg_connect(responder, "trigger_out", trigger_debug, "store")
        self.tb.connect(response, head, sink)
        self.tb.start()
        try:
            self._await_messages(trigger_debug, 1)
            deadline = time.monotonic() + 3.0
            while len(sink.data()) < response.frame_len() and time.monotonic() < deadline:
                time.sleep(0.002)
            self.assertEqual(len(sink.data()), response.frame_len())
        finally:
            self.tb.stop()
            self.tb.wait()
        trigger = trigger_debug.get_message(0)
        self.assertTrue(pmt.to_bool(pmt.dict_ref(trigger, pmt.intern("fine_delay_valid"), pmt.PMT_F)))

        self.tb = gr.top_block()
        rx_tag.value = pmt.make_tuple(pmt.from_uint64(10), pmt.from_double(0.0500136))
        source = blocks.vector_source_c(list(sink.data()), False, 1, [rx_tag])
        detector = ofdm_prs_ranging.prs_frame_detector(samp_rate=fs, threshold=0.30, coarse_zc_root=29)
        fft = ofdm_prs_ranging.prs_fft_receiver(samp_rate=fs)
        channel = ofdm_prs_ranging.prs_channel_estimator(samp_rate=fs)
        phase = ofdm_prs_ranging.prs_phase_slope_estimator(samp_rate=fs)
        solver = ofdm_prs_ranging.prs_ssrtt_solver(fs)
        debug = blocks.message_debug()
        self.tb.connect(source, detector)
        self.tb.msg_connect(detector, "frame_out", fft, "frame_in")
        self.tb.msg_connect(fft, "symbols_out", channel, "symbols_in")
        self.tb.msg_connect(channel, "channel_out", phase, "channel_in")
        self.tb.msg_connect(phase, "measurement_out", solver, "measurement_in")
        self.tb.msg_connect(solver, "ssrtt_out", debug, "store")
        solver.to_basic_block()._post(pmt.intern("tx_time_in"), self._v2_meta(
            dict(packet_type=1, poll_id=0, tx_time_secs=10, tx_time_frac=0.0)))
        self.tb.start()
        try:
            self._await_messages(debug, 1)
            out = pmt.car(debug.get_message(0))
            self.assertEqual(pmt.to_uint64(pmt.dict_ref(out, pmt.intern("anchor_id"), pmt.PMT_NIL)), 7)
            self.assertEqual(pmt.to_uint64(pmt.dict_ref(out, pmt.intern("poll_id"), pmt.PMT_NIL)), 0)
            self.assertTrue(pmt.to_bool(pmt.dict_ref(out, pmt.intern("fine_correction_valid"), pmt.PMT_F)))
            get = lambda key: pmt.to_double(pmt.dict_ref(out, pmt.intern(key), pmt.PMT_NIL))
            self.assertAlmostEqual(get("corrected_range_m"), get("integer_range_m") +
                299792458.0 / (2 * fs) * (get("poll_fine_delay_samples") + get("response_fine_delay_samples")), places=8)
        finally:
            self.tb.stop()
            self.tb.wait()

    def test_v2_bidirectional_correction_signs_and_invalid_fallback(self):
        fs = 30e6
        solver = ofdm_prs_ranging.prs_ssrtt_solver(fs)
        debug = blocks.message_debug()
        self.tb.msg_connect(solver, "ssrtt_out", debug, "store")
        cases = [(poll, response, True, True) for poll in (0.25, -0.25) for response in (0.375, -0.375)]
        cases += [(0.0, 0.25, False, True), (0.25, 0.0, True, False)]
        cases += [(value, 0.25, True, True) for value in (float('nan'), float('inf'), -float('inf'))]
        cases += [(0.25, value, True, True) for value in (float('nan'), float('inf'), -float('inf'))]
        self.tb.start()
        try:
            tx = self._v2_meta(dict(packet_type=1, poll_id=42, tx_time_secs=100, tx_time_frac=0.0))
            solver.to_basic_block()._post(pmt.intern("tx_time_in"), tx)
            deadline = time.monotonic() + 1.0
            while solver.to_basic_block().nmsgs(pmt.intern("tx_time_in")) and time.monotonic() < deadline:
                time.sleep(0.002)
            rejected = self._v2_meta(dict(frame_id_valid=False, packet_type=2, poll_id=42,
                anchor_id=7, reply_delay_samples=1500000, poll_fine_delay_samples=0.25,
                fine_delay_samples=0.25, fine_delay_valid=True, valid=True,
                rx_time=pmt.make_tuple(pmt.from_uint64(100), pmt.from_double(0.0500136))))
            solver.to_basic_block()._post(pmt.intern("measurement_in"), pmt.cons(rejected, pmt.PMT_NIL))
            for poll, response, poll_valid, response_valid in cases:
                meta = self._v2_meta(dict(frame_id_valid=True, packet_type=2, poll_id=42,
                    anchor_id=7, reply_delay_samples=1500000, poll_fine_delay_samples=poll,
                    fine_delay_samples=response, fine_delay_valid=poll_valid, valid=response_valid,
                    rx_time=pmt.make_tuple(pmt.from_uint64(100), pmt.from_double(0.0500136))))
                solver.to_basic_block()._post(pmt.intern("measurement_in"), pmt.cons(meta, pmt.PMT_NIL))
            self._await_messages(debug, len(cases))
            integer = (13.6e-6 + 4.841619800497349e-5) * 299792458.0 / 2
            for index, (poll, response, pv, rv) in enumerate(cases):
                out = pmt.car(debug.get_message(index))
                get = lambda key: pmt.to_double(pmt.dict_ref(out, pmt.intern(key), pmt.PMT_NIL))
                self.assertAlmostEqual(get("integer_range_m"), integer, delta=2e-6)
                expected_valid = pv and rv and numpy.isfinite(poll) and numpy.isfinite(response)
                self.assertEqual(pmt.to_bool(pmt.dict_ref(out, pmt.intern("fine_correction_valid"), pmt.PMT_F)), expected_valid)
                self.assertAlmostEqual(get("measurement_timestamp"), 100.0500136)
                if expected_valid:
                    correction = 299792458.0 / (2 * fs) * (poll + response)
                    self.assertAlmostEqual(get("fine_range_correction_m"), correction, places=9)
                    self.assertAlmostEqual(get("corrected_range_m"), integer + correction, delta=2e-6)
                else:
                    self.assertTrue(numpy.isnan(get("corrected_range_m")))
                    self.assertTrue(numpy.isnan(get("fine_range_correction_m")))
        finally:
            self.tb.stop()
            self.tb.wait()

    def test_v2_trigger_bounds_and_nonfinite_float(self):
        tx = ofdm_prs_ranging.prs_timed_burst_source(attach_tx_time=False)
        debug = blocks.message_debug()
        head = blocks.head(gr.sizeof_gr_complex, 5 * tx.frame_len())
        sink = blocks.vector_sink_c()
        self.tb.connect(tx, head, sink)
        self.tb.msg_connect(tx, "tx_time_out", debug, "store")
        base = dict(packet_type=2, poll_id=65535, anchor_id=7,
                    reply_delay_samples=0xffffff, poll_fine_delay_q10=-32768,
                    fine_delay_valid=True, tx_time_secs=0, tx_time_frac=0.0)
        invalid = [("anchor_id", value) for value in (8, 255, 256, -1)]
        invalid += [("poll_id", 65536), ("reply_delay_samples", 0x1000000),
                    ("poll_fine_delay_q10", -32769), ("poll_fine_delay_q10", 32768)]
        self.tb.start()
        try:
            for key, value in invalid:
                tx.to_basic_block()._post(pmt.intern("trigger"), self._v2_meta(dict(base, **{key: value})))
            for samples in (0.25, float('nan'), float('inf'), -float('inf')):
                tx.to_basic_block()._post(pmt.intern("trigger"), self._v2_meta(dict(base, poll_fine_delay_samples=samples)))
            missing_fine = dict(base)
            del missing_fine["poll_fine_delay_q10"]
            tx.to_basic_block()._post(pmt.intern("trigger"), self._v2_meta(missing_fine))
            self._await_messages(debug, 5)
            for index in range(5):
                out = debug.get_message(index)
                self.assertEqual(pmt.to_bool(pmt.dict_ref(out, pmt.intern("fine_delay_valid"), pmt.PMT_F)), index == 0)
            deadline = time.monotonic() + 3.0
            while len(sink.data()) < 5 * tx.frame_len() and time.monotonic() < deadline:
                time.sleep(0.002)
            self.assertEqual(len(sink.data()), 5 * tx.frame_len())
        finally:
            self.tb.stop()
            self.tb.wait()

    def test_ssrtt_responder_schedules_response_trigger(self):
        responder = ofdm_prs_ranging.prs_ssrtt_responder(10e6, 50000)
        debug = blocks.message_debug()

        meta = pmt.make_dict()
        for key, value in (
            ("frame_id_valid", pmt.PMT_T),
            ("packet_type", pmt.from_long(1)),
            ("poll_id", pmt.from_uint64(123)),
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
        self.assertEqual(pmt.to_uint64(pmt.dict_ref(trigger, pmt.intern("poll_id"), pmt.PMT_NIL)), 123)
        self.assertEqual(pmt.to_uint64(pmt.dict_ref(trigger, pmt.intern("reply_delay_samples"), pmt.PMT_NIL)), 50000)
        self.assertEqual(pmt.to_uint64(pmt.dict_ref(trigger, pmt.intern("tx_time_secs"), pmt.PMT_NIL)), 10)
        self.assertAlmostEqual(pmt.to_double(pmt.dict_ref(trigger, pmt.intern("tx_time_frac"), pmt.PMT_NIL)), 0.255)

    def test_ssrtt_solver_computes_range_from_synthetic_timestamps(self):
        solver = ofdm_prs_ranging.prs_ssrtt_solver(10e6)
        debug = blocks.message_debug()

        tx = pmt.make_dict()
        for key, value in (
            ("packet_type", pmt.from_long(1)),
            ("poll_id", pmt.from_uint64(42)),
            ("frame_id", pmt.from_uint64(42)),
            ("tx_time_secs", pmt.from_uint64(100)),
            ("tx_time_frac", pmt.from_double(0.0)),
        ):
            tx = pmt.dict_add(tx, pmt.intern(key), value)

        rx = pmt.make_dict()
        for key, value in (
            ("frame_id_valid", pmt.PMT_T),
            ("packet_type", pmt.from_long(2)),
            ("poll_id", pmt.from_uint64(42)),
            ("anchor_id", pmt.from_uint64(7)),
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
        self.assertAlmostEqual(pmt.to_double(pmt.dict_ref(out, pmt.intern("tof_s"), pmt.PMT_NIL)),
                               (13.6e-6 + 4.841619800497349e-5) / 2.0)
        calibrated_tof = (13.6e-6 + 4.841619800497349e-5) / 2.0
        calibrated_range = calibrated_tof * 299792458.0
        self.assertAlmostEqual(pmt.to_double(pmt.dict_ref(out, pmt.intern("range_m"), pmt.PMT_NIL)), calibrated_range, places=5)
        self.assertAlmostEqual(pmt.to_double(pmt.dict_ref(
            out, pmt.intern("integer_range_m"), pmt.PMT_NIL)),
            calibrated_range, places=5)
        self.assertAlmostEqual(pmt.to_double(pmt.dict_ref(
            out, pmt.intern("response_phase_range_correction_m"), pmt.PMT_NIL)),
            3.747405725, places=6)


if __name__ == "__main__":
    gr_unittest.run(qa_prs_receiver)
