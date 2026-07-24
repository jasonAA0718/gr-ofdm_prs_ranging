#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright 2026 GNU Radio ZC TWR contributors.
#
# SPDX-License-Identifier: GPL-3.0-or-later

import math
import statistics
import sys
import threading
import time
from collections import deque

import pmt
from gnuradio import gr


POLL = 1
RESPONSE = 2


def _metadata(message):
    if pmt.is_dict(message):
        return message
    if pmt.is_pair(message) and pmt.is_dict(pmt.car(message)):
        return pmt.car(message)
    return None


def _number(meta, key, default=0.0):
    value = pmt.dict_ref(meta, pmt.intern(key), pmt.PMT_NIL)
    if pmt.is_real(value):
        return float(pmt.to_double(value))
    if pmt.is_uint64(value):
        return float(pmt.to_uint64(value))
    if pmt.is_integer(value):
        return float(pmt.to_long(value))
    return default


def _integer(meta, key, default=0):
    return int(_number(meta, key, default))


def _boolean(meta, key, default=False):
    value = pmt.dict_ref(meta, pmt.intern(key), pmt.PMT_NIL)
    return pmt.to_bool(value) if pmt.is_bool(value) else default


class prs_text_ui(gr.basic_block):
    """Compact terminal status display for the PRS initiator or responder."""

    def __init__(
            self,
            role="initiator",
            stale_timeout_s=1.5,
            response_timeout_s=1.0,
            refresh_period_s=0.25,
            use_ansi=True,
            output_stream=None,
            clock=None):
        gr.basic_block.__init__(
            self, name="prs_text_ui", in_sig=None, out_sig=None)

        role = str(role).lower()
        if role not in ("initiator", "responder"):
            raise ValueError("role must be 'initiator' or 'responder'")
        if stale_timeout_s <= 0.0 or response_timeout_s <= 0.0:
            raise ValueError("status timeouts must be positive")
        if refresh_period_s <= 0.0:
            raise ValueError("refresh_period_s must be positive")

        self._role = role
        self._stale_timeout_s = float(stale_timeout_s)
        self._response_timeout_s = float(response_timeout_s)
        self._refresh_period_s = float(refresh_period_s)
        self._stream = output_stream if output_stream is not None else sys.stdout
        self._clock = clock if clock is not None else time.monotonic
        self._ansi = bool(use_ansi) and bool(
            getattr(self._stream, "isatty", lambda: False)())

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None
        self._last_line = ""
        self._last_rx_time = None
        self._latest_snr = math.nan
        self._latest_range = math.nan
        self._ranges = deque(maxlen=20)
        self._polls = {}
        self._received_poll_ids = set()
        self._sent_response_ids = set()

        for port in ("tx_in", "frame_in", "measurement_in"):
            self.message_port_register_in(pmt.intern(port))
        self.set_msg_handler(pmt.intern("tx_in"), self._handle_tx)
        self.set_msg_handler(pmt.intern("frame_in"), self._handle_frame)
        self.set_msg_handler(
            pmt.intern("measurement_in"), self._handle_measurement)

    def start(self):
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._refresh_loop,
            name="prs-text-ui",
            daemon=True)
        self._thread.start()
        return True

    def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0 * self._refresh_period_s + 0.1)
            self._thread = None
        if self._ansi and self._last_line:
            self._stream.write("\n")
            self._stream.flush()
        return True

    def _handle_tx(self, message):
        meta = _metadata(message)
        if meta is None:
            return
        packet_type = _integer(meta, "packet_type")
        poll_id = _integer(
            meta, "poll_frame_id", _integer(meta, "frame_id"))
        now = self._clock()
        with self._lock:
            if self._role == "initiator" and packet_type == POLL:
                if poll_id not in self._polls:
                    self._polls[poll_id] = {
                        "state": "pending",
                        "deadline": now + self._response_timeout_s,
                    }
            elif self._role == "responder" and packet_type == RESPONSE:
                self._sent_response_ids.add(poll_id)
        self._render()

    def _handle_frame(self, message):
        meta = _metadata(message)
        if meta is None or not _boolean(meta, "frame_id_valid"):
            return
        packet_type = _integer(meta, "packet_type")
        expected = RESPONSE if self._role == "initiator" else POLL
        if packet_type != expected:
            return
        poll_id = _integer(
            meta, "poll_frame_id", _integer(meta, "frame_id"))
        with self._lock:
            self._last_rx_time = self._clock()
            if self._role == "responder":
                self._received_poll_ids.add(poll_id)
        self._render()

    def _handle_measurement(self, message):
        meta = _metadata(message)
        if meta is None or not _boolean(meta, "frame_id_valid"):
            return
        packet_type = _integer(meta, "packet_type")
        expected = RESPONSE if self._role == "initiator" else POLL
        if packet_type != expected:
            return

        poll_id = _integer(
            meta, "poll_frame_id", _integer(meta, "frame_id"))
        snr = _number(meta, "snr", math.nan)
        range_m = _number(meta, "range_m", math.nan)
        with self._lock:
            self._last_rx_time = self._clock()
            if math.isfinite(snr):
                self._latest_snr = snr
            if self._role == "initiator":
                poll = self._polls.get(poll_id)
                if poll is not None:
                    poll["state"] = "responded"
                if math.isfinite(range_m):
                    self._latest_range = range_m
                    self._ranges.append(range_m)
            else:
                self._received_poll_ids.add(poll_id)
        self._render()

    def _expire_polls(self, now):
        for poll in self._polls.values():
            if poll["state"] == "pending" and now >= poll["deadline"]:
                poll["state"] = "lost"

    def snapshot(self):
        now = self._clock()
        with self._lock:
            self._expire_polls(now)
            age = (math.inf if self._last_rx_time is None
                   else max(0.0, now - self._last_rx_time))
            signal_received = age <= self._stale_timeout_s
            if signal_received:
                status = "RECEIVING"
            elif (self._role == "initiator" and
                  any(p["state"] == "pending" for p in self._polls.values())):
                status = "WAITING"
            else:
                status = "NO SIGNAL"

            if self._role == "initiator":
                polls = len(self._polls)
                responses = sum(
                    p["state"] == "responded" for p in self._polls.values())
                lost = sum(p["state"] == "lost" for p in self._polls.values())
                pending = polls - responses - lost
            else:
                polls = len(self._received_poll_ids)
                responses = len(self._sent_response_ids)
                lost = max(0, polls - responses)
                pending = 0

            success_rate = responses / polls if polls else 0.0
            loss_rate = lost / polls if polls else 0.0
            median_range = (
                statistics.median(self._ranges) if self._ranges else math.nan)
            return {
                "role": self._role,
                "status": status,
                "signal_received": signal_received,
                "polls": polls,
                "responses": responses,
                "pending": pending,
                "lost": lost,
                "success_rate": success_rate,
                "loss_rate": loss_rate,
                "snr": self._latest_snr,
                "range_m": self._latest_range,
                "median_range_m": median_range,
                "last_rx_age_s": age,
            }

    @staticmethod
    def _format_value(value, suffix, precision=1):
        if not math.isfinite(value):
            return "--"
        return f"{value:.{precision}f}{suffix}"

    def format_status(self):
        status = self.snapshot()
        if self._role == "initiator":
            return (
                f"[INITIATOR] RX {status['status']:<9} | "
                f"responses/polls {status['responses']}/{status['polls']} "
                f"({100.0 * status['success_rate']:.1f}%) | "
                f"loss {100.0 * status['loss_rate']:.1f}% | "
                f"pending {status['pending']} | "
                f"SNR {self._format_value(status['snr'], ' dB')} | "
                f"range {self._format_value(status['range_m'], ' m', 2)}")
        return (
            f"[RESPONDER] RX {status['status']:<9} | "
            f"replies/polls {status['responses']}/{status['polls']} "
            f"({100.0 * status['success_rate']:.1f}%) | "
            f"SNR {self._format_value(status['snr'], ' dB')}")

    def _render(self):
        line = self.format_status()
        if self._ansi:
            padding = max(0, len(self._last_line) - len(line))
            self._stream.write("\r" + line + " " * padding)
            self._stream.flush()
            self._last_line = line
        elif line != self._last_line:
            self._stream.write(line + "\n")
            self._stream.flush()
            self._last_line = line

    def _refresh_loop(self):
        self._render()
        while not self._stop_event.wait(self._refresh_period_s):
            self._render()
