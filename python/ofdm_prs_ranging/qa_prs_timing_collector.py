#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 GNU Radio ZC TWR contributors.
#
# SPDX-License-Identifier: GPL-3.0-or-later

import csv
import os
import tempfile
import time

import pmt
from gnuradio import blocks, gr, gr_unittest
from gnuradio import ofdm_prs_ranging


def timing_message():
    stages = pmt.make_dict()
    stages = pmt.dict_add(stages, pmt.intern("fft_execute"), pmt.from_uint64(12000))
    stages = pmt.dict_add(stages, pmt.intern("output_build"), pmt.from_uint64(3000))

    message = pmt.make_dict()
    message = pmt.dict_add(message, pmt.intern("block"), pmt.intern("fft_receiver"))
    message = pmt.dict_add(
        message, pmt.intern("handler_start_ns"), pmt.from_uint64(100000))
    message = pmt.dict_add(
        message, pmt.intern("handler_end_ns"), pmt.from_uint64(118000))
    message = pmt.dict_add(
        message, pmt.intern("handler_duration_ns"), pmt.from_uint64(18000))
    message = pmt.dict_add(message, pmt.intern("attempt_id"), pmt.from_uint64(7))
    message = pmt.dict_add(message, pmt.intern("poll_frame_id"), pmt.from_uint64(7))
    message = pmt.dict_add(message, pmt.intern("valid"), pmt.PMT_T)
    return pmt.dict_add(message, pmt.intern("stages"), stages)


class qa_prs_timing_collector(gr_unittest.TestCase):
    def test_writes_raw_rows_and_percentile_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            raw_path = os.path.join(directory, "raw.csv")
            summary_path = os.path.join(directory, "summary.csv")
            source = blocks.message_strobe(timing_message(), 5)
            collector = ofdm_prs_ranging.prs_timing_collector(
                raw_path, summary_path, "initiator")
            flowgraph = gr.top_block()
            flowgraph.msg_connect(source, "strobe", collector, "timing_in")
            flowgraph.start()
            time.sleep(0.05)
            flowgraph.stop()
            flowgraph.wait()

            with open(raw_path, newline="", encoding="ascii") as handle:
                raw_rows = list(csv.DictReader(handle))
            self.assertGreaterEqual(len(raw_rows), 3)
            self.assertEqual({row["role"] for row in raw_rows}, {"initiator"})
            self.assertIn("handler_total", {row["stage"] for row in raw_rows})
            self.assertIn("fft_execute", {row["stage"] for row in raw_rows})

            with open(summary_path, newline="", encoding="ascii") as handle:
                summary_rows = list(csv.DictReader(handle))
            fft_summary = next(
                row for row in summary_rows if row["stage"] == "fft_execute")
            self.assertEqual(fft_summary["block"], "fft_receiver")
            self.assertAlmostEqual(float(fft_summary["mean_us"]), 12.0)
            self.assertAlmostEqual(float(fft_summary["p99_us"]), 12.0)


if __name__ == "__main__":
    gr_unittest.run(qa_prs_timing_collector)
