#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#
# SPDX-License-Identifier: GPL-3.0
#
# GNU Radio Python Flow Graph
# Title: record prs iq
# GNU Radio version: 3.10.11.0

from gnuradio import blocks
from gnuradio import gr
from gnuradio.filter import firdes
from gnuradio.fft import window
import sys
import signal
from argparse import ArgumentParser
from gnuradio.eng_arg import eng_float, intx
from gnuradio import eng_notation
from gnuradio import uhd
import time
import threading




class record_prs_iq(gr.top_block):

    def __init__(self):
        gr.top_block.__init__(self, "record prs iq", catch_exceptions=True)
        self.flowgraph_started = threading.Event()

        ##################################################
        # Variables
        ##################################################
        self.samp_rate = samp_rate = 30e6
        self.rx_gain = rx_gain = 30
        self.rx_antenna = rx_antenna = "TX/RX"
        self.iq_file = iq_file = "handoff.cf32"
        self.center_freq = center_freq = 1330e6
        self.capture_seconds = capture_seconds = 5

        ##################################################
        # Blocks
        ##################################################

        self.save_iq = blocks.file_sink(gr.sizeof_gr_complex*1, iq_file, False)
        self.save_iq.set_unbuffered(False)
        self.rx = uhd.usrp_source(
            ",".join(("", "recv_buff_size=20000000,num_recv_frames=700")),
            uhd.stream_args(
                cpu_format="fc32",
                args='',
                channels=list(range(0,1)),
            ),
        )
        self.rx.set_subdev_spec('A:A', 0)
        self.rx.set_samp_rate(samp_rate)
        _last_pps_time = self.rx.get_time_last_pps().get_real_secs()
        # Poll get_time_last_pps() every 50 ms until a change is seen
        while(self.rx.get_time_last_pps().get_real_secs() == _last_pps_time):
            time.sleep(0.05)
        # Set the time to PC time on next PPS
        self.rx.set_time_next_pps(uhd.time_spec(int(time.time()) + 1.0))
        # Sleep 1 second to ensure next PPS has come
        time.sleep(1)

        self.rx.set_center_freq(center_freq, 0)
        self.rx.set_antenna(rx_antenna, 0)
        self.rx.set_bandwidth(samp_rate, 0)
        self.rx.set_gain(rx_gain, 0)
        self.limit = blocks.head(gr.sizeof_gr_complex*1, (int(samp_rate * capture_seconds)))


        ##################################################
        # Connections
        ##################################################
        self.connect((self.limit, 0), (self.save_iq, 0))
        self.connect((self.rx, 0), (self.limit, 0))


    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate
        self.limit.set_length((int(self.samp_rate * self.capture_seconds)))
        self.rx.set_samp_rate(self.samp_rate)
        self.rx.set_bandwidth(self.samp_rate, 0)

    def get_rx_gain(self):
        return self.rx_gain

    def set_rx_gain(self, rx_gain):
        self.rx_gain = rx_gain
        self.rx.set_gain(self.rx_gain, 0)

    def get_rx_antenna(self):
        return self.rx_antenna

    def set_rx_antenna(self, rx_antenna):
        self.rx_antenna = rx_antenna
        self.rx.set_antenna(self.rx_antenna, 0)

    def get_iq_file(self):
        return self.iq_file

    def set_iq_file(self, iq_file):
        self.iq_file = iq_file
        self.save_iq.open(self.iq_file)

    def get_center_freq(self):
        return self.center_freq

    def set_center_freq(self, center_freq):
        self.center_freq = center_freq
        self.rx.set_center_freq(self.center_freq, 0)

    def get_capture_seconds(self):
        return self.capture_seconds

    def set_capture_seconds(self, capture_seconds):
        self.capture_seconds = capture_seconds
        self.limit.set_length((int(self.samp_rate * self.capture_seconds)))




def main(top_block_cls=record_prs_iq, options=None):
    tb = top_block_cls()

    def sig_handler(sig=None, frame=None):
        tb.stop()
        tb.wait()

        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    tb.start()
    tb.flowgraph_started.set()

    tb.wait()


if __name__ == '__main__':
    main()
