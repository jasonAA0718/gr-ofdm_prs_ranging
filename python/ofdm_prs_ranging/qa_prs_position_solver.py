from gnuradio import gr, blocks, gr_unittest
from gnuradio import ofdm_prs_ranging
import pmt

meta = pmt.make_dict()

meta = pmt.dict_add(
    meta,
    pmt.intern("observed_anchor_ids"),
    pmt.init_u32vector(3, [0, 1, 2])
)
meta = pmt.dict_add(
    meta,
    pmt.intern("integer_ranges_m"),
    pmt.init_f64vector(3, [5.0, 6.0, 7.0])
)

meta = pmt.dict_add(
    meta,
    pmt.intern("corrected_ranges_m"),
    pmt.init_f64vector(3, [5.12, 6.08, 7.15])
)

meta = pmt.dict_add(
    meta,
    pmt.intern("fine_correction_valid_flags"),
    pmt.init_u8vector(3, [1, 1, 1])
)

meta = pmt.dict_add(
    meta,
    pmt.intern("poll_id"),
    pmt.from_uint64(100)
)

epoch_pdu = pmt.cons(meta, pmt.PMT_NIL)
solver = ofdm_prs_ranging.prs_position_solver(
    [0, 1, 2],
    [
        [0.0, 0.0, 0.0],
        [6.0, 0.0, 0.0],
        [0.0, 8.0, 0.0]
    ],
    0.0
)
tb = gr.top_block()
debug = blocks.message_debug()
tb.msg_connect(
    (solver, "diagnostic_out"),
    (debug, "store")
)

tb.start()

solver.to_basic_block()._post(
    pmt.intern("epoch_in"),
    epoch_pdu
)

tb.stop()
tb.wait()