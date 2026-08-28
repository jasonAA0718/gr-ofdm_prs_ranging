#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import time

import numpy
import pmt
from gnuradio import blocks, gr, gr_unittest, ofdm_prs_ranging


class qa_prs_fft_zc_detector(gr_unittest.TestCase):
    def setUp(self):
        self.tb = gr.top_block()

    def tearDown(self):
        self.tb = None

    def run_detector(self, samples, profiling=False):
        source = blocks.vector_source_c(samples, False)
        detector = ofdm_prs_ranging.prs_fft_zc_frame_detector(
            samp_rate=30e6,
            preamble_len=256,
            preamble_repeats=4,
            coarse_sync_len=419,
            threshold=0.35,
            zc_threshold=0.4,
            correlation_fft_len=2048,
            zc_search_before=1024,
            zc_search_after=1024,
            enable_profiling=profiling,
        )
        frame_debug = blocks.message_debug()
        timing_debug = blocks.message_debug()
        self.tb.connect(source, detector)
        self.tb.msg_connect((detector, "frame_out"), (frame_debug, "store"))
        self.tb.msg_connect((detector, "timing_out"), (timing_debug, "store"))
        self.tb.run()
        return frame_debug, timing_debug

    def test_clean_cfo_frame_finds_exact_boundary(self):
        tx = ofdm_prs_ranging.prs_timed_burst_source(
            samp_rate=30e6,
            preamble_len=256,
            preamble_repeats=4,
            coarse_sync_len=419,
            attach_tx_time=False,
        )
        frame = numpy.asarray(tx.frame_samples(), dtype=numpy.complex64)
        cfo_hz = 900.0
        phase = numpy.arange(frame.size) * (2.0 * math.pi * cfo_hz / 30e6)
        frame *= numpy.exp(1j * phase).astype(numpy.complex64)
        prefix = 1379
        samples = numpy.concatenate(
            (numpy.zeros(prefix, numpy.complex64), frame, numpy.zeros(2048, numpy.complex64))
        )

        frame_debug, timing_debug = self.run_detector(samples, profiling=True)

        self.assertEqual(frame_debug.num_messages(), 1)
        metadata = pmt.car(frame_debug.get_message(0))
        self.assertEqual(
            pmt.to_uint64(pmt.dict_ref(metadata, pmt.intern("frame_start"), pmt.PMT_NIL)),
            prefix,
        )
        self.assertGreater(
            pmt.to_double(pmt.dict_ref(metadata, pmt.intern("coarse_metric"), pmt.PMT_NIL)),
            0.99,
        )
        self.assertFalse(
            pmt.is_null(pmt.dict_ref(metadata, pmt.intern("zc_peak_ratio"), pmt.PMT_NIL))
        )
        self.assertLessEqual(
            abs(pmt.to_long(
                pmt.dict_ref(metadata, pmt.intern("zc_gate_offset_samples"), pmt.PMT_NIL)
            )),
            1024,
        )
        self.assertEqual(timing_debug.num_messages(), 1)
        timing = timing_debug.get_message(0)
        self.assertEqual(
            pmt.symbol_to_string(pmt.dict_ref(timing, pmt.intern("block"), pmt.PMT_NIL)),
            "fft_zc_frame_detector",
        )
        stages = pmt.dict_ref(timing, pmt.intern("stages"), pmt.PMT_NIL)
        for stage in (
            "preamble_gate_scan",
            "zc_input_prepare",
            "zc_fft_forward",
            "zc_spectrum_multiply",
            "zc_ifft",
            "zc_normalize_peak",
        ):
            self.assertFalse(pmt.is_null(pmt.dict_ref(stages, pmt.intern(stage), pmt.PMT_NIL)))

    def test_repeated_signal_without_zc_is_not_a_frame(self):
        rng = numpy.random.default_rng(7)
        seed = (rng.standard_normal(256) + 1j * rng.standard_normal(256)).astype(
            numpy.complex64
        )
        repeated = numpy.tile(seed, 4)
        samples = numpy.concatenate(
            (numpy.zeros(1000, numpy.complex64), repeated, numpy.zeros(60000, numpy.complex64))
        )

        frame_debug, _ = self.run_detector(samples)

        self.assertEqual(frame_debug.num_messages(), 0)

    def test_time_gate_finds_only_the_scheduled_frame(self):
        samp_rate = 1e6
        tx = ofdm_prs_ranging.prs_timed_burst_source(
            samp_rate=samp_rate, attach_tx_time=False
        )
        frame = numpy.asarray(tx.frame_samples(), dtype=numpy.complex64)
        target_start = 60000
        samples = numpy.zeros(target_start + frame.size + 2000, numpy.complex64)
        samples[1000 : 1000 + frame.size] = frame
        samples[target_start : target_start + frame.size] = frame
        rx_time = pmt.make_tuple(pmt.from_uint64(100), pmt.from_double(0.0))
        tags = [
            gr.tag_utils.python_to_tag(
                (0, pmt.intern("rx_time"), rx_time, pmt.intern("qa"))
            )
        ]
        source = blocks.vector_source_c(samples, False, 1, tags)
        throttle = blocks.throttle(gr.sizeof_gr_complex, samp_rate)
        detector = ofdm_prs_ranging.prs_fft_zc_frame_detector(
            samp_rate=samp_rate,
            threshold=0.30,
            time_gating=True,
            reply_delay_s=0.0,
            window_before_s=0.001,
            window_after_s=0.080,
        )
        debug = blocks.message_debug()
        tx_meta = pmt.make_dict()
        tx_meta = pmt.dict_add(tx_meta, pmt.intern("attempt_id"), pmt.from_uint64(314))
        tx_meta = pmt.dict_add(tx_meta, pmt.intern("tx_time_secs"), pmt.from_uint64(100))
        tx_meta = pmt.dict_add(
            tx_meta, pmt.intern("tx_time_frac"), pmt.from_double(0.060)
        )

        self.tb.connect(source, throttle, detector)
        self.tb.msg_connect((detector, "frame_out"), (debug, "store"))
        self.tb.start()
        detector.to_basic_block()._post(pmt.intern("tx_time_in"), tx_meta)
        time.sleep(0.18)
        self.tb.stop()
        self.tb.wait()

        self.assertEqual(debug.num_messages(), 1)
        metadata = pmt.car(debug.get_message(0))
        self.assertEqual(
            pmt.to_uint64(pmt.dict_ref(metadata, pmt.intern("frame_start"), pmt.PMT_NIL)),
            target_start,
        )
        self.assertEqual(
            pmt.to_uint64(pmt.dict_ref(metadata, pmt.intern("attempt_id"), pmt.PMT_NIL)),
            314,
        )
        self.assertEqual(
            pmt.to_long(
                pmt.dict_ref(metadata, pmt.intern("zc_gate_offset_samples"), pmt.PMT_NIL)
            ),
            0,
        )
        self.assertEqual(
            pmt.to_double(
                pmt.dict_ref(metadata, pmt.intern("preamble_metric"), pmt.PMT_NIL)
            ),
            0.0,
        )


if __name__ == "__main__":
    gr_unittest.run(qa_prs_fft_zc_detector)
