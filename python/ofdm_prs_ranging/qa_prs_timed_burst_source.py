#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2026 GNU Radio ZC TWR contributors.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

import math
import time

import numpy
import pmt
from gnuradio import blocks, gr, gr_unittest

try:
    from gnuradio.ofdm_prs_ranging import prs_rx_timekeeper, prs_timed_burst_source
except ImportError:
    import os
    import sys

    dirname, filename = os.path.split(os.path.abspath(__file__))
    sys.path.append(os.path.join(dirname, "bindings"))
    from gnuradio.ofdm_prs_ranging import prs_rx_timekeeper, prs_timed_burst_source


class qa_prs_timed_burst_source(gr_unittest.TestCase):
    def setUp(self):
        self.tb = gr.top_block()

    def tearDown(self):
        self.tb = None

    def test_waveform_length_and_offsets(self):
        src = prs_timed_burst_source()
        self.assertEqual(src.frame_len(), 23935)
        self.assertEqual(src.prs_start(), 1000 + 128 * 16 + 839 + 616)
        self.assertEqual(src.prs_len(), 16 * (1024 + 128))

    def test_active_subcarrier_mapping(self):
        src = prs_timed_burst_source()
        frame = numpy.asarray(src.frame_samples(), dtype=numpy.complex64)
        prs0 = frame[src.prs_start() + 128 : src.prs_start() + 128 + 1024]
        freq = numpy.fft.fft(prs0)
        mag = numpy.abs(freq)

        self.assertEqual(numpy.count_nonzero(mag > 1e-6), 1024)
        self.assertGreater(mag[0], 1e-6)
        numpy.testing.assert_allclose(
            mag, numpy.full(1024, mag[0]), rtol=1e-5, atol=1e-5)

    def test_amplitude_limit(self):
        src = prs_timed_burst_source()
        frame = numpy.asarray(src.frame_samples(), dtype=numpy.complex64)
        self.assertLessEqual(float(numpy.max(numpy.abs(frame))), 0.900001)
        rms = math.sqrt(float(numpy.mean(numpy.abs(frame) ** 2)))
        self.assertGreater(rms, 0.0)

    def test_trigger_emits_burst_tags(self):
        nitems = 26000
        rx_time = pmt.make_tuple(pmt.from_uint64(123), pmt.from_double(0.25))
        tags = [gr.tag_utils.python_to_tag((0, pmt.intern("rx_time"), rx_time, pmt.intern("qa")))]
        timing = blocks.vector_source_c([0j] * nitems, False, 1, tags)
        src = prs_timed_burst_source(tx_lead_time=0.5)
        head = blocks.head(gr.sizeof_gr_complex, src.frame_len())
        sink = blocks.vector_sink_c()

        self.tb.connect(timing, src, head, sink)
        src.to_basic_block()._post(pmt.intern("trigger"), pmt.PMT_T)
        self.tb.run()

        data = numpy.asarray(sink.data(), dtype=numpy.complex64)
        self.assertEqual(len(data), src.frame_len())
        self.assertGreater(float(numpy.max(numpy.abs(data[: src.frame_len()]))), 0.0)

        tags_by_key = {pmt.symbol_to_string(t.key): t for t in sink.tags()}
        for key in (
            "tx_time",
            "tx_sob",
            "frame_id",
            "burst_len",
            "prs_start",
            "prs_len",
            "fft_len",
            "cp_len",
            "active_bins",
            "samp_rate",
            "tx_eob",
        ):
            self.assertIn(key, tags_by_key)

        tx_time = tags_by_key["tx_time"].value
        self.assertEqual(pmt.to_uint64(pmt.tuple_ref(tx_time, 0)), 123)
        self.assertAlmostEqual(pmt.to_double(pmt.tuple_ref(tx_time, 1)), 0.75)
        self.assertEqual(pmt.to_long(tags_by_key["burst_len"].value), 23935)
        self.assertEqual(tags_by_key["tx_eob"].offset, src.frame_len() - 1)

    def test_attach_tx_time_false_omits_tx_time_tag(self):
        tb = gr.top_block()
        nitems = 26000
        timing = blocks.vector_source_c([0j] * nitems)
        src = prs_timed_burst_source(attach_tx_time=False)
        head = blocks.head(gr.sizeof_gr_complex, src.frame_len())
        sink = blocks.vector_sink_c()

        tb.connect(timing, src, head, sink)
        src.to_basic_block()._post(pmt.intern("trigger"), pmt.PMT_T)
        tb.run()

        self.assertEqual(len(sink.data()), src.frame_len())
        tags_by_key = {pmt.symbol_to_string(t.key): t for t in sink.tags()}
        self.assertNotIn("tx_time", tags_by_key)
        self.assertIn("tx_sob", tags_by_key)
        self.assertIn("tx_eob", tags_by_key)
        self.assertEqual(tags_by_key["tx_eob"].offset, src.frame_len() - 1)

    def test_explicit_time_trigger_needs_no_stream_input(self):
        src = prs_timed_burst_source()
        head = blocks.head(gr.sizeof_gr_complex, src.frame_len())
        sink = blocks.vector_sink_c()
        trigger = pmt.make_dict()
        trigger = pmt.dict_add(
            trigger, pmt.intern("tx_time_secs"), pmt.from_uint64(321))
        trigger = pmt.dict_add(
            trigger, pmt.intern("tx_time_frac"), pmt.from_double(0.125))

        self.tb.connect(src, head, sink)
        self.tb.start()
        src.to_basic_block()._post(pmt.intern("trigger"), trigger)
        deadline = time.monotonic() + 2.0
        while len(sink.data()) < src.frame_len() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.tb.stop()
        self.tb.wait()

        self.assertEqual(len(sink.data()), src.frame_len())
        tags_by_key = {pmt.symbol_to_string(t.key): t for t in sink.tags()}
        tx_time = tags_by_key["tx_time"].value
        self.assertEqual(pmt.to_uint64(pmt.tuple_ref(tx_time, 0)), 321)
        self.assertAlmostEqual(pmt.to_double(pmt.tuple_ref(tx_time, 1)), 0.125)

    def test_rx_timekeeper_stamps_trigger_from_sample_clock(self):
        samp_rate = 10e6
        nitems = 1000
        rx_time = pmt.make_tuple(pmt.from_uint64(123), pmt.from_double(0.25))
        tags = [
            gr.tag_utils.python_to_tag(
                (0, pmt.intern("rx_time"), rx_time, pmt.intern("qa")))
        ]
        timing = blocks.vector_source_c([0j] * nitems, False, 1, tags)
        keeper = prs_rx_timekeeper(samp_rate, 0.5)
        debug = blocks.message_debug()

        self.tb.connect(timing, keeper)
        self.tb.msg_connect(
            (keeper, "timed_trigger_out"), (debug, "store"))
        keeper.to_basic_block()._post(pmt.intern("trigger_in"), pmt.PMT_T)
        self.tb.run()

        self.assertEqual(debug.num_messages(), 1)
        trigger = debug.get_message(0)
        current_offset = pmt.to_uint64(
            pmt.dict_ref(
                trigger,
                pmt.intern("timekeeper_sample_offset"),
                pmt.PMT_NIL))
        tx_time = (
            pmt.to_uint64(
                pmt.dict_ref(trigger, pmt.intern("tx_time_secs"), pmt.PMT_NIL))
            + pmt.to_double(
                pmt.dict_ref(trigger, pmt.intern("tx_time_frac"), pmt.PMT_NIL))
        )
        expected = 123.25 + current_offset / samp_rate + 0.5
        self.assertAlmostEqual(tx_time, expected)


if __name__ == "__main__":
    gr_unittest.run(qa_prs_timed_burst_source)
