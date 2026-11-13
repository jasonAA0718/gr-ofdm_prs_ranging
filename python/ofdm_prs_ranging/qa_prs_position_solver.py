from gnuradio import gr, blocks, gr_unittest
from gnuradio import ofdm_prs_ranging
import time
import pmt
nan = float("nan")


class qa_prs_position_solver(gr_unittest.TestCase):
    def epoch_create(self, observed_anchor_ids, integer_ranges_m, corrected_ranges_m, fine_correction_valid_flags, poll_id):
        meta = pmt.make_dict()
        meta = pmt.dict_add(
            meta,
            pmt.intern("observed_anchor_ids"),
            pmt.init_u32vector(len(observed_anchor_ids), observed_anchor_ids)
        )
        meta = pmt.dict_add(
            meta,
            pmt.intern("integer_ranges_m"),
            pmt.init_f64vector(len(integer_ranges_m), integer_ranges_m)
        )
        meta = pmt.dict_add(
            meta,
            pmt.intern("corrected_ranges_m"),
            pmt.init_f64vector(len(corrected_ranges_m), corrected_ranges_m)
        )
        meta = pmt.dict_add(
            meta,
            pmt.intern("fine_correction_valid_flags"),
            pmt.init_u8vector(len(fine_correction_valid_flags), fine_correction_valid_flags)
        )
        meta = pmt.dict_add(
            meta,
            pmt.intern("poll_id"),
            pmt.from_uint64(poll_id)
        )    
        epoch_pdu = pmt.cons(meta, pmt.PMT_NIL)
        return epoch_pdu


    def setUp(self):
        self.tb = gr.top_block()

        self.solver = ofdm_prs_ranging.prs_position_solver(
            [0, 1, 2, 3, 4, 5],
            [
                [0.0, 0.0, 0.0],
                [6.0, 0.0, 0.0],
                [0.0, 8.0, 0.0],
                [0.0, -1.0, 1.0],
                [0.0, -2.0, 0.0],
                [0.0, -8.0, 0.0]
            ],
            0.0
        )
        self.debug = blocks.message_debug()
        self.tb.msg_connect(
            (self.solver, "diagnostic_out"),
            (self.debug, "store")
        )

    def tearDown(self):
        self.tb.stop()
        self.tb.wait()

    def test_001_range_selection(self):
        observed_anchor_ids = [0, 1, 2, 3, 4, 5]
        integer_ranges_m = [1.0, 2.0, 3.0, 4.0, nan, -6.0]
        corrected_ranges_m = [1.11, 2.22, nan, -4.44, nan, nan]
        fine_correction_valid_flags = [1, 0, 1, 1, 0, 0]
        poll_id = 100
        epoch_pdu = self.epoch_create(observed_anchor_ids, integer_ranges_m, corrected_ranges_m, fine_correction_valid_flags, poll_id)

        # Range selection metadata is present on either solver result port.
        self.tb.msg_connect(self.solver, "position_out", self.debug, "store")
        self.tb.start()


        self.solver.to_basic_block()._post(
            pmt.intern("epoch_in"),
            epoch_pdu
        )

        deadline = time.monotonic() + 1.0

        while self.debug.num_messages() == 0:
            if time.monotonic() >= deadline:
                self.fail("No solver result received")
            time.sleep(0.01)

        msg = self.debug.get_message(0)
        result = pmt.car(msg)

        def get_field(key):
            return pmt.dict_ref(
                result,
                pmt.intern(key),
                pmt.PMT_NIL
            )

        selected_ids = list(
            pmt.u32vector_elements(get_field("selected_anchor_ids"))
        )

        selected_ranges = list(
            pmt.f64vector_elements(get_field("selected_ranges_m"))
        )

        selected_flags = list(
            pmt.u8vector_elements(get_field("used_fine_correction_flags"))
        )

        count = pmt.to_uint64(
            get_field("used_anchor_count")
        )

        self.assertEqual(selected_ids, [0, 1, 2])
        self.assertEqual(selected_flags, [1, 0, 0])
        self.assertEqual(count, 3)

        expected_ranges = [1.11, 2.0, 3.0]

        self.assertEqual(len(selected_ranges), len(expected_ranges))

        for actual, expected in zip(selected_ranges, expected_ranges):
            self.assertAlmostEqual(actual, expected, places=9)

        print("Observation Selection QA: PASS")
            # print("Received PMT:")
            # print(pmt.write_string(msg))
        pass

    def test_002_invalid_vector_length(self):
        # PMT vector 長度不一致
        observed_anchor_ids = [0, 1, 2]
        integer_ranges_m = [1.0, 2.0]  # 只有兩筆
        corrected_ranges_m = [1.1, 2.2, 3.3]
        fine_correction_valid_flags = [1, 1, 1]
        poll_id = 100
        invalid_epoch_pdu = self.epoch_create(observed_anchor_ids, integer_ranges_m, corrected_ranges_m, fine_correction_valid_flags, poll_id)
        
        observed_anchor_ids = [0, 1, 2]
        integer_ranges_m = [1.0, 2.0, 3.0]  # 正確長度
        corrected_ranges_m = [1.1, 2.2, 3.3]
        fine_correction_valid_flags = [1, 1, 1]
        poll_id = 101
        valid_epoch_pdu = self.epoch_create(observed_anchor_ids, integer_ranges_m, corrected_ranges_m, fine_correction_valid_flags, poll_id)

        self.tb.start()
        self.solver.to_basic_block()._post(
            pmt.intern("epoch_in"), invalid_epoch_pdu
        )

        self.solver.to_basic_block()._post(
            pmt.intern("epoch_in"), valid_epoch_pdu
        )
        
        deadline = time.monotonic() + 1.0

        while True:
            if self.debug.num_messages() > 0:
                msg = self.debug.get_message(self.debug.num_messages() - 1)
                result = pmt.car(msg)

                received_poll_id = pmt.to_uint64(
                    pmt.dict_ref(
                        result,
                        pmt.intern("poll_id"),
                        pmt.PMT_NIL
                    )
                )

                if received_poll_id == 101:
                    break

            if time.monotonic() >= deadline:
                self.fail("Valid sentinel epoch was not received")

            time.sleep(0.01)

        self.assertEqual(self.debug.num_messages(), 1)
        pass

    def test_003_unknown_anchor_id(self):
        # Anchor ID 超出配置範圍
        observed_anchor_ids = [0, 1, 6]
        integer_ranges_m = [1.0, 2.0, 3.0] 
        corrected_ranges_m = [1.1, 2.2, 3.3]
        fine_correction_valid_flags = [1, 1, 1]
        poll_id = 100
        invalid_epoch_pdu = self.epoch_create(observed_anchor_ids, integer_ranges_m, corrected_ranges_m, fine_correction_valid_flags, poll_id)
        
        observed_anchor_ids = [0, 1, 5]
        integer_ranges_m = [1.0, 2.0, 3.0] 
        corrected_ranges_m = [1.1, 2.2, 3.3]
        fine_correction_valid_flags = [1, 1, 1]
        poll_id = 101
        valid_epoch_pdu = self.epoch_create(observed_anchor_ids, integer_ranges_m, corrected_ranges_m, fine_correction_valid_flags, poll_id)

        self.tb.start()
        self.solver.to_basic_block()._post(
            pmt.intern("epoch_in"), invalid_epoch_pdu
        )

        self.solver.to_basic_block()._post(
            pmt.intern("epoch_in"), valid_epoch_pdu
        )
        
        deadline = time.monotonic() + 1.0

        while True:
            if self.debug.num_messages() > 0:
                msg = self.debug.get_message(self.debug.num_messages() - 1)
                result = pmt.car(msg)

                received_poll_id = pmt.to_uint64(
                    pmt.dict_ref(
                        result,
                        pmt.intern("poll_id"),
                        pmt.PMT_NIL
                    )
                )

                if received_poll_id == 101:
                    break

            if time.monotonic() >= deadline:
                self.fail("Valid sentinel epoch was not received")

            time.sleep(0.01)

        self.assertEqual(self.debug.num_messages(), 1)
        pass


if __name__ == "__main__":
    gr_unittest.run(qa_prs_position_solver)