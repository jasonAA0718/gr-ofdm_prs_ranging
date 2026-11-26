#!/usr/bin/env python3

from gnuradio import blocks, gr, gr_unittest, ofdm_prs_ranging
import numpy
import pmt
import time


class qa_prs_window_capture(gr_unittest.TestCase):
    L = ofdm_prs_ranging.PHY_PREAMBLE_LEN
    R = ofdm_prs_ranging.PHY_PREAMBLE_REPEATS
    NZ = ofdm_prs_ranging.PHY_ZC_CHIP_COUNT * ofdm_prs_ranging.PHY_ZC_SAMPLES_PER_CHIP
    NG = ofdm_prs_ranging.PHY_LEADING_GUARD_SAMPLES
    NT = ofdm_prs_ranging.PHY_TAIL_GUARD_SAMPLES
    FFT = 512
    CP = 64
    NSYM = ofdm_prs_ranging.PHY_PRS_SYMBOLS
    MARGIN = 64

    def _make_capture(self, queue_depth=3, enabled=True, threshold=0.8):
        return ofdm_prs_ranging.prs_window_capture(
            samp_rate=1e6,
            threshold=threshold,
            min_frame_gap=100,
            candidate_margin_before=self.MARGIN,
            candidate_margin_after=self.MARGIN,
            queue_depth=queue_depth,
            acquisition_enabled=enabled)

    def _period(self):
        rng = numpy.random.default_rng(17)
        return (rng.choice([-1, 1], self.L) +
                1j * rng.choice([-1, 1], self.L)) / numpy.sqrt(2.0)

    def _samples(self, burst_starts, length):
        samples = numpy.zeros(length, dtype=numpy.complex64)
        preamble = numpy.tile(self._period(), self.R)
        for start in burst_starts:
            samples[start:start + len(preamble)] = preamble
            # Nonperiodic samples prevent a second preamble event in the same frame.
            samples[start + len(preamble):start + len(preamble) + self.NZ] = (
                0.31 + 0.17j)
        return samples

    def _run(self, samples, capture, window_message=None, tags=(),
             max_noutput_items=None):
        tb = gr.top_block()
        source = blocks.vector_source_c(samples, False, 1, list(tags))
        candidate_debug = blocks.message_debug()
        event_debug = blocks.message_debug()
        if window_message is None:
            tb.connect(source, capture)
        else:
            throttle = blocks.throttle(gr.sizeof_gr_complex, 1e6, True)
            tb.connect(source, throttle, capture)
        tb.msg_connect(capture, "candidate_out", candidate_debug, "store")
        tb.msg_connect(capture, "event_out", event_debug, "store")
        if window_message is not None:
            if max_noutput_items is None:
                tb.start()
            else:
                tb.start(max_noutput_items)
            capture.to_basic_block()._post(pmt.intern("window_in"), window_message)
            deadline = time.monotonic() + 1.0
            while (capture.to_basic_block().nmsgs(pmt.intern("window_in")) > 0 and
                   time.monotonic() < deadline):
                time.sleep(0.001)
            tb.wait()
        else:
            if max_noutput_items is None:
                tb.run()
            else:
                tb.run(max_noutput_items)
        return candidate_debug, event_debug

    @staticmethod
    def _meta(message):
        return pmt.car(message) if pmt.is_pair(message) else message

    @staticmethod
    def _uint(meta, key):
        return pmt.to_uint64(pmt.dict_ref(meta, pmt.intern(key), pmt.PMT_NIL))

    def _reference_first_crossing(self, samples, threshold, minimum_index):
        span = self.L * (self.R - 1)
        total = self.L * self.R
        for p in range(0, len(samples) - total + 1):
            first = samples[p:p + span]
            second = samples[p + self.L:p + self.L + span]
            corr = numpy.vdot(first, second)
            first_power = float(numpy.vdot(first, first).real)
            second_power = float(numpy.vdot(second, second).real)
            if first_power <= 0.0 or second_power <= 0.0:
                continue
            metric = abs(corr) / numpy.sqrt(first_power * second_power)
            history = self.NG + total + self.MARGIN
            if p >= history and metric >= threshold and p >= minimum_index:
                return p
        self.fail("reference preamble crossing not found")

    def test_metric_and_candidate_geometry(self):
        samples = self._samples([5000], 90000)
        capture = self._make_capture()
        rx_tag = gr.tag_t()
        rx_tag.offset = 0
        rx_tag.key = pmt.intern("rx_time")
        rx_tag.value = pmt.make_tuple(pmt.from_uint64(12), pmt.from_double(0.25))
        rx_tag.srcid = pmt.intern("qa")
        candidates, events = self._run(samples, capture, tags=[rx_tag])
        self.assertEqual(candidates.num_messages(), 1)
        self.assertGreaterEqual(events.num_messages(), 1)

        candidate = candidates.get_message(0)
        meta = self._meta(candidate)
        self.assertEqual(
            pmt.symbol_to_string(
                pmt.dict_ref(meta, pmt.intern("acquisition_state"), pmt.PMT_NIL)),
            "LOCKOUT")
        self.assertEqual(
            pmt.symbol_to_string(pmt.dict_ref(meta, pmt.intern("event"), pmt.PMT_NIL)),
            "CANDIDATE_READY")
        expected_crossing = self._reference_first_crossing(samples, 0.8, 0)
        self.assertEqual(
            self._uint(meta, "first_preamble_crossing_index"), expected_crossing)
        self.assertGreater(self._uint(meta, "preamble_threshold_crossings"), 1)
        self.assertEqual(
            self._uint(meta, "candidate_window_length"), pmt.length(pmt.cdr(candidate)))
        candidate_start = self._uint(meta, "candidate_window_start")
        candidate_end = self._uint(meta, "candidate_window_end")
        numpy.testing.assert_array_equal(
            numpy.asarray(pmt.c32vector_elements(pmt.cdr(candidate)),
                          dtype=numpy.complex64),
            samples[candidate_start:candidate_end])

        search_start = self._uint(meta, "zc_search_start_offset")
        search_end = self._uint(meta, "zc_search_end_offset")
        first_rel = self._uint(meta, "first_preamble_crossing_offset")
        self.assertLess(search_start, first_rel)
        self.assertGreater(search_end, first_rel + self.L * self.R)
        self.assertGreater(self._uint(meta, "block_a_samples_consumed"), 0)
        self.assertGreater(self._uint(meta, "block_a_work_calls"), 0)
        candidate_time = pmt.dict_ref(
            meta, pmt.intern("candidate_rx_time"), pmt.PMT_NIL)
        self.assertTrue(pmt.is_tuple(candidate_time))
        observed_time = (pmt.to_uint64(pmt.tuple_ref(candidate_time, 0)) +
                         pmt.to_double(pmt.tuple_ref(candidate_time, 1)))
        expected_time = 12.25 + self._uint(meta, "candidate_window_start") / 1e6
        self.assertAlmostEqual(observed_time, expected_time, places=9)
        self.assertEqual(self._uint(meta, "rx_time_tag_offset"), 0)

    def test_window_open_reconstructs_metric_from_history(self):
        # Keep the scheduled boundary beyond the first scheduler buffer so the
        # asynchronous command is installed before that part of the stream arrives.
        burst_start = 50000
        samples = self._samples([burst_start], 135000)
        capture = self._make_capture(enabled=False)
        # The complete preamble is already in history when this two-sample window opens.
        window_start = burst_start + self.L * self.R - 1
        command = pmt.make_dict()
        command = pmt.dict_add(command, pmt.intern("start_sample"),
                               pmt.from_uint64(window_start))
        command = pmt.dict_add(command, pmt.intern("end_sample"),
                               pmt.from_uint64(window_start + 2))
        command = pmt.dict_add(command, pmt.intern("window_id"), pmt.from_uint64(17))
        command = pmt.dict_add(command, pmt.intern("attempt_id"), pmt.from_uint64(23))
        candidates, _ = self._run(
            samples, capture, command, max_noutput_items=4096)

        self.assertEqual(candidates.num_messages(), 1)
        meta = self._meta(candidates.get_message(0))
        self.assertEqual(self._uint(meta, "first_preamble_crossing_index"), burst_start)
        self.assertEqual(self._uint(meta, "window_id"), 17)
        self.assertEqual(self._uint(meta, "attempt_id"), 23)
        self.assertTrue(pmt.to_bool(pmt.dict_ref(
            meta, pmt.intern("receive_window_enabled"), pmt.PMT_F)))

    def test_absolute_time_window(self):
        burst_start = 50000
        samples = self._samples([burst_start], 135000)
        capture = self._make_capture(enabled=False)
        window_start = burst_start + self.L * self.R - 1
        command = pmt.make_dict()
        command = pmt.dict_add(command, pmt.intern("start_time"),
                               pmt.from_double(100.0 + window_start / 1e6))
        command = pmt.dict_add(command, pmt.intern("end_time"),
                               pmt.from_double(100.0 + (window_start + 2) / 1e6))
        command = pmt.dict_add(command, pmt.intern("window_id"), pmt.from_uint64(31))
        rx_tag = gr.tag_t()
        rx_tag.offset = 0
        rx_tag.key = pmt.intern("rx_time")
        rx_tag.value = pmt.make_tuple(pmt.from_uint64(100), pmt.from_double(0.0))
        rx_tag.srcid = pmt.intern("qa")
        candidates, _ = self._run(
            samples, capture, command, tags=[rx_tag], max_noutput_items=4096)

        self.assertEqual(candidates.num_messages(), 1)
        meta = self._meta(candidates.get_message(0))
        self.assertEqual(self._uint(meta, "first_preamble_crossing_index"), burst_start)
        self.assertEqual(self._uint(meta, "window_id"), 31)

    def test_responder_feedback_limits_next_poll_and_expires(self):
        for hit in (True, False):
            with self.subTest(poll_in_window=hit):
                # A decoy before the window and a POLL after it expires.
                burst_starts = [110000, 311000, 430000] if hit else [110000, 430000]
                samples = self._samples(burst_starts, 540000)
                capture = self._make_capture()
                responder = ofdm_prs_ranging.prs_ssrtt_responder(
                    1e6, 5000, 0, poll_period_s=0.15, poll_window_margin_s=0.0005)
                candidates = blocks.message_debug()
                triggers = blocks.message_debug()
                source = blocks.vector_source_c(samples)
                throttle = blocks.throttle(gr.sizeof_gr_complex, 1e6, True)
                tb = gr.top_block()
                tb.connect(source, throttle, capture)
                tb.msg_connect(responder, "trigger_out", capture, "window_in")
                tb.msg_connect(responder, "trigger_out", triggers, "store")
                tb.msg_connect(capture, "candidate_out", candidates, "store")
                tb.start(4096)
                try:
                    # Inject two decoded POLLs to establish the period independently
                    # of CSV logging and use real stream processing for the next window.
                    for poll_id, frame_start in ((8, 10000), (9, 160000)):
                        meta = pmt.make_dict()
                        for key, value in (
                            ("frame_id_valid", pmt.PMT_T),
                            ("packet_type", pmt.from_long(1)),
                            ("poll_id", pmt.from_uint64(poll_id)),
                            ("frame_start", pmt.from_uint64(frame_start)),
                            ("rx_time", pmt.make_tuple(
                                pmt.from_uint64(10), pmt.from_double(frame_start / 1e6))),
                        ):
                            meta = pmt.dict_add(meta, pmt.intern(key), value)
                        responder.to_basic_block()._post(
                            pmt.intern("measurement_in"), pmt.cons(meta, pmt.PMT_NIL))
                    deadline = time.monotonic() + 3.0
                    expected_count = 2 if hit else 1
                    while time.monotonic() < deadline:
                        if (capture.nitems_read(0) == len(samples) and
                                candidates.num_messages() >= expected_count):
                            break
                        time.sleep(0.001)
                    self.assertEqual(capture.nitems_read(0), len(samples))
                finally:
                    tb.stop()
                    tb.wait()

                self.assertEqual(triggers.num_messages(), 2)
                marker = pmt.intern("poll_tracking_window")
                self.assertTrue(pmt.is_null(pmt.dict_ref(
                    triggers.get_message(0), marker, pmt.PMT_NIL)))
                prediction = triggers.get_message(1)
                self.assertTrue(pmt.to_bool(pmt.dict_ref(prediction, marker, pmt.PMT_F)))
                self.assertAlmostEqual(self._uint(prediction, "start_sample"), 309500, delta=1)
                self.assertAlmostEqual(
                    self._uint(prediction, "end_sample"),
                    310500 + self.NG + self.L * self.R, delta=1)
                self.assertEqual(self._uint(prediction, "poll_id"), 9)
                reply_frac = pmt.dict_ref(prediction, pmt.intern("tx_time_frac"), pmt.PMT_NIL)
                self.assertAlmostEqual(pmt.to_double(reply_frac), 0.165)
                self.assertEqual(candidates.num_messages(), expected_count)
                window_ids = [
                    self._uint(self._meta(candidates.get_message(i)), "window_id")
                    for i in range(candidates.num_messages())
                ]
                self.assertEqual(window_ids, [10, 0] if hit else [0])

    def test_stalled_consumer_drops_without_stalling_stream(self):
        burst_starts = [5000, 90000, 175000, 260000, 345000]
        samples = self._samples(burst_starts, 430000)
        capture = self._make_capture(queue_depth=2)
        candidates, events = self._run(samples, capture)

        self.assertEqual(candidates.num_messages(), 2)
        queue_full = 0
        last_full_count = 0
        for index in range(events.num_messages()):
            meta = self._meta(events.get_message(index))
            reason = pmt.dict_ref(meta, pmt.intern("failure_reason"), pmt.PMT_NIL)
            if pmt.is_symbol(reason) and pmt.symbol_to_string(reason) == "WINDOW_QUEUE_FULL":
                queue_full += 1
                last_full_count = self._uint(meta, "window_queue_full_count")
        self.assertEqual(queue_full, len(burst_starts) - 2)
        self.assertEqual(last_full_count, len(burst_starts) - 2)

    def test_chunk_boundaries_preserve_candidate(self):
        samples = self._samples([5000], 90000)
        reference, _ = self._run(
            samples, self._make_capture(), max_noutput_items=4096)
        self.assertEqual(reference.num_messages(), 1)
        reference_pdu = reference.get_message(0)
        reference_meta = self._meta(reference_pdu)
        fields = (
            "first_preamble_crossing_index",
            "preamble_threshold_crossings",
            "candidate_window_start",
            "candidate_window_end",
            "candidate_window_length",
        )
        reference_values = tuple(self._uint(reference_meta, key) for key in fields)
        reference_metric = pmt.to_double(pmt.dict_ref(
            reference_meta, pmt.intern("max_preamble_metric"), pmt.PMT_NIL))
        reference_samples = numpy.asarray(
            pmt.c32vector_elements(pmt.cdr(reference_pdu)), dtype=numpy.complex64)

        # These limits place the repeated preamble, rolling metric, and ring wrap
        # at different scheduler-call boundaries.
        for chunk_size in (17, 31, 127):
            observed, _ = self._run(
                samples, self._make_capture(), max_noutput_items=chunk_size)
            self.assertEqual(observed.num_messages(), 1)
            observed_pdu = observed.get_message(0)
            observed_meta = self._meta(observed_pdu)
            self.assertEqual(
                tuple(self._uint(observed_meta, key) for key in fields),
                reference_values)
            self.assertAlmostEqual(
                pmt.to_double(pmt.dict_ref(
                    observed_meta, pmt.intern("max_preamble_metric"), pmt.PMT_NIL)),
                reference_metric,
                places=7)
            numpy.testing.assert_array_equal(
                numpy.asarray(
                    pmt.c32vector_elements(pmt.cdr(observed_pdu)),
                    dtype=numpy.complex64),
                reference_samples)

    def test_lockout_end_inside_chunk(self):
        samples = self._samples([5000, 90000], 175000)
        candidates, _ = self._run(
            samples, self._make_capture(), max_noutput_items=1024)
        self.assertEqual(candidates.num_messages(), 2)
        starts = [self._uint(self._meta(candidates.get_message(index)),
                             "first_preamble_crossing_index")
                  for index in range(candidates.num_messages())]
        self.assertEqual(
            starts,
            [self._reference_first_crossing(samples, 0.8, 0),
             self._reference_first_crossing(samples, 0.8, 88000)])


if __name__ == '__main__':
    gr_unittest.run(qa_prs_window_capture)
