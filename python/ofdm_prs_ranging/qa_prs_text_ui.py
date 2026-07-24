#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 GNU Radio ZC TWR contributors.
#
# SPDX-License-Identifier: GPL-3.0-or-later

import io
import os
import sys

import pmt
from gnuradio import gr_unittest

try:
    from gnuradio.ofdm_prs_ranging import prs_text_ui
except ImportError:
    dirname = os.path.dirname(os.path.abspath(__file__))
    sys.path.append(os.path.dirname(dirname))
    from ofdm_prs_ranging import prs_text_ui


def metadata(**values):
    result = pmt.make_dict()
    for key, value in values.items():
        if isinstance(value, bool):
            encoded = pmt.from_bool(value)
        elif isinstance(value, int) and value >= 0:
            encoded = pmt.from_uint64(value)
        elif isinstance(value, int):
            encoded = pmt.from_long(value)
        else:
            encoded = pmt.from_double(value)
        result = pmt.dict_add(result, pmt.intern(key), encoded)
    return result


class qa_prs_text_ui(gr_unittest.TestCase):
    def setUp(self):
        self.now = 10.0
        self.output = io.StringIO()

    def clock(self):
        return self.now

    def test_initiator_tracks_response_loss_snr_and_range(self):
        ui = prs_text_ui(
            "initiator",
            stale_timeout_s=1.5,
            response_timeout_s=1.0,
            use_ansi=False,
            output_stream=self.output,
            clock=self.clock)

        ui._handle_tx(metadata(
            packet_type=1, poll_frame_id=7, frame_id=7))
        pending = ui.snapshot()
        self.assertEqual(pending["polls"], 1)
        self.assertEqual(pending["pending"], 1)
        self.assertEqual(pending["status"], "WAITING")

        response = metadata(
            packet_type=2,
            poll_frame_id=7,
            frame_id_valid=True,
            snr=23.5,
            range_m=84.25)
        ui._handle_frame(pmt.cons(response, pmt.PMT_NIL))
        ui._handle_measurement(pmt.cons(response, pmt.PMT_NIL))
        received = ui.snapshot()
        self.assertEqual(received["responses"], 1)
        self.assertEqual(received["pending"], 0)
        self.assertEqual(received["status"], "RECEIVING")
        self.assertAlmostEqual(received["snr"], 23.5)
        self.assertAlmostEqual(received["range_m"], 84.25)

        ui._handle_tx(metadata(
            packet_type=1, poll_frame_id=8, frame_id=8))
        self.now += 1.1
        expired = ui.snapshot()
        self.assertEqual(expired["polls"], 2)
        self.assertEqual(expired["responses"], 1)
        self.assertEqual(expired["lost"], 1)
        self.assertAlmostEqual(expired["loss_rate"], 0.5)
        self.assertIn("responses/polls 1/2", ui.format_status())
        self.assertIn("range 84.25 m", ui.format_status())

        self.now += 0.5
        self.assertEqual(ui.snapshot()["status"], "NO SIGNAL")

    def test_responder_tracks_received_polls_and_sent_replies(self):
        ui = prs_text_ui(
            "responder",
            use_ansi=False,
            output_stream=self.output,
            clock=self.clock)
        poll = metadata(
            packet_type=1,
            poll_frame_id=12,
            frame_id_valid=True,
            snr=18.0)
        ui._handle_frame(pmt.cons(poll, pmt.PMT_NIL))
        ui._handle_measurement(pmt.cons(poll, pmt.PMT_NIL))
        ui._handle_tx(metadata(
            packet_type=2, poll_frame_id=12, response_frame_id=15))

        status = ui.snapshot()
        self.assertEqual(status["polls"], 1)
        self.assertEqual(status["responses"], 1)
        self.assertEqual(status["status"], "RECEIVING")
        self.assertAlmostEqual(status["success_rate"], 1.0)
        line = ui.format_status()
        self.assertIn("[RESPONDER]", line)
        self.assertIn("replies/polls 1/1", line)
        self.assertIn("SNR 18.0 dB", line)


if __name__ == "__main__":
    gr_unittest.run(qa_prs_text_ui)
