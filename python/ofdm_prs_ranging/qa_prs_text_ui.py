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
            packet_type=1, poll_id=7, frame_id=7))
        pending = ui.snapshot()
        self.assertEqual(pending["polls"], 1)
        self.assertEqual(pending["pending"], 1)
        self.assertEqual(pending["status"], "WAITING")

        response = metadata(
            packet_type=2,
            poll_id=7,
            frame_id_valid=True,
            snr=23.5,
            integer_range_m=84.25,
            range_m=999.0)
        ui._handle_frame(pmt.cons(response, pmt.PMT_NIL))
        ui._handle_measurement(pmt.cons(response, pmt.PMT_NIL))
        received = ui.snapshot()
        self.assertEqual(received["responses"], 1)
        self.assertEqual(received["pending"], 0)
        self.assertEqual(received["status"], "RECEIVING")
        self.assertAlmostEqual(received["snr"], 23.5)
        self.assertAlmostEqual(received["range_m"], 84.25)

        ui._handle_tx(metadata(
            packet_type=1, poll_id=8, frame_id=8))
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
            poll_id=12,
            frame_id_valid=True,
            snr=18.0)
        ui._handle_frame(pmt.cons(poll, pmt.PMT_NIL))
        ui._handle_measurement(pmt.cons(poll, pmt.PMT_NIL))
        ui._handle_tx(metadata(
            packet_type=2, poll_id=12, anchor_id=15))

        status = ui.snapshot()
        self.assertEqual(status["polls"], 1)
        self.assertEqual(status["responses"], 1)
        self.assertEqual(status["status"], "RECEIVING")
        self.assertAlmostEqual(status["success_rate"], 1.0)
        line = ui.format_status()
        self.assertIn("[RESPONDER]", line)
        self.assertIn("replies/polls 1/1", line)
        self.assertIn("SNR 18.0 dB", line)

    def test_responder_cumulative_counts_survive_restart_and_wrap(self):
        ui = prs_text_ui("responder", output_stream=self.output, clock=self.clock)
        # Restart reuses IDs; wrap also reuses zero. Receive times remain local
        # to the continuously running responder, so every transaction is new.
        for index, poll_id in enumerate((0, 1, 0, 1, 65535, 0)):
            poll = metadata(packet_type=1, poll_id=poll_id,
                            frame_id_valid=True, rx_time=100.0 + index, snr=19.0)
            # Either pipeline notification may arrive first.
            handlers = (ui._handle_frame, ui._handle_measurement)
            if index % 2:
                handlers = handlers[::-1]
            for handler in handlers:
                handler(poll)
            ui._handle_frame(poll)  # Repeated notification is not a new POLL.
            reply = metadata(packet_type=2, poll_id=poll_id,
                             tx_time_secs=100 + index, tx_time_frac=0.031)
            ui._handle_tx(reply)
            ui._handle_tx(reply)
            status = ui.snapshot()
            self.assertEqual(status["polls"], index + 1)
            self.assertEqual(status["responses"], index + 1)
            self.assertEqual(status["success_rate"], 1.0)
            ui._render()
            self.assertEqual(ui.snapshot(), status)

    def test_responder_legacy_metadata_and_frame_sample_fallback(self):
        ui = prs_text_ui("responder", output_stream=self.output, clock=self.clock)
        poll = metadata(packet_type=1, poll_id=0, frame_id_valid=True)
        ui._handle_frame(poll)
        ui._handle_measurement(poll)
        self.assertEqual(ui.snapshot()["polls"], 1)
        self.now += 1.1
        ui._handle_frame(poll)
        ui._handle_measurement(poll)
        self.assertEqual(ui.snapshot()["polls"], 2)
        for sample in (1000, 2000):
            stamped = pmt.dict_add(poll, pmt.intern("frame_start"), pmt.from_uint64(sample))
            ui._handle_frame(stamped)
            ui._handle_measurement(stamped)
        self.assertEqual(ui.snapshot()["polls"], 4)

    def test_responder_history_bounded_counters_keep_growing(self):
        ui = prs_text_ui("responder", output_stream=self.output, clock=self.clock)
        for index in range(300):
            ui._handle_frame(metadata(packet_type=1, poll_id=index,
                                      frame_id_valid=True, rx_time=float(index)))
            ui._handle_tx(metadata(packet_type=2, poll_id=index,
                                   tx_time_secs=index, tx_time_frac=0.031))
        self.assertEqual(ui.snapshot()["polls"], 300)
        self.assertEqual(ui.snapshot()["responses"], 300)
        self.assertLessEqual(len(ui._recent_polls), 256)
        self.assertLessEqual(len(ui._recent_responses), 256)
        ui._handle_frame(metadata(packet_type=1, poll_id=301, frame_id_valid=False))
        self.assertEqual(ui.snapshot()["polls"], 300)


if __name__ == "__main__":
    gr_unittest.run(qa_prs_text_ui)
