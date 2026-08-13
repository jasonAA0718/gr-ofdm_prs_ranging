#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2026 GNU Radio ZC TWR contributors.
#
# SPDX-License-Identifier: GPL-3.0-or-later

import time

import numpy
import pmt
from gnuradio import blocks, gr, gr_unittest

try:
    from gnuradio.ofdm_prs_ranging import (pluto_prs_burst_source,
                                           pluto_prs_responder)
except ImportError:
    import os
    import sys

    dirname, filename = os.path.split(os.path.abspath(__file__))
    sys.path.append(os.path.join(dirname, "bindings"))
    from gnuradio.ofdm_prs_ranging import (pluto_prs_burst_source,
                                           pluto_prs_responder)


class qa_pluto_prs(gr_unittest.TestCase):
    def test_idle_stream_is_continuous_zeros(self):
        tb = gr.top_block()
        source = pluto_prs_burst_source()
        head = blocks.head(gr.sizeof_gr_complex, 4096)
        sink = blocks.vector_sink_c()
        tb.connect(source, head, sink)
        tb.run()

        samples = numpy.asarray(sink.data())
        self.assertEqual(samples.size, 4096)
        self.assertEqual(numpy.count_nonzero(samples), 0)

    def test_trigger_inserts_frame_without_time_tags(self):
        tb = gr.top_block()
        source = pluto_prs_burst_source()
        head = blocks.head(gr.sizeof_gr_complex, source.frame_len() + 1024)
        sink = blocks.vector_sink_c()
        tb.connect(source, head, sink)
        source.to_basic_block()._post(pmt.intern("trigger"), pmt.PMT_T)
        tb.run()

        samples = numpy.asarray(sink.data())
        self.assertGreater(numpy.count_nonzero(numpy.abs(samples) > 1e-6), 10000)
        tags = {pmt.symbol_to_string(tag.key): tag for tag in sink.tags()}
        self.assertIn("tx_sob", tags)
        self.assertIn("tx_eob", tags)
        self.assertIn("attempt_id", tags)
        self.assertNotIn("tx_time", tags)
        self.assertNotIn("rx_time", tags)

    def test_responder_accepts_only_valid_poll(self):
        tb = gr.top_block()
        responder = pluto_prs_responder()
        debug = blocks.message_debug()
        tb.msg_connect((responder, "trigger_out"), (debug, "store"))
        tb.start()

        def post(valid, packet_type, poll_id):
            meta = pmt.make_dict()
            meta = pmt.dict_add(meta,
                                pmt.intern("frame_id_valid"),
                                pmt.PMT_T if valid else pmt.PMT_F)
            meta = pmt.dict_add(meta,
                                pmt.intern("packet_type"),
                                pmt.from_long(packet_type))
            meta = pmt.dict_add(meta,
                                pmt.intern("poll_frame_id"),
                                pmt.from_uint64(poll_id))
            responder.to_basic_block()._post(
                pmt.intern("frame_in"), pmt.cons(meta, pmt.PMT_NIL))

        post(False, 1, 75)
        post(True, 2, 76)
        post(True, 1, 77)
        time.sleep(0.1)
        tb.stop()
        tb.wait()

        self.assertEqual(debug.num_messages(), 1)
        trigger = debug.get_message(0)
        self.assertEqual(
            pmt.to_long(pmt.dict_ref(
                trigger, pmt.intern("packet_type"), pmt.PMT_NIL)), 2)
        self.assertEqual(
            pmt.to_uint64(pmt.dict_ref(
                trigger, pmt.intern("poll_frame_id"), pmt.PMT_NIL)), 77)
        self.assertTrue(pmt.is_true(pmt.dict_ref(
            trigger, pmt.intern("untimed_response"), pmt.PMT_F)))


if __name__ == "__main__":
    gr_unittest.run(qa_pluto_prs)
