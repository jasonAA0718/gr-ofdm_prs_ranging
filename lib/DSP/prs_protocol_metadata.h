/* SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef INCLUDED_OFDM_PRS_RANGING_PRS_PROTOCOL_METADATA_H
#define INCLUDED_OFDM_PRS_RANGING_PRS_PROTOCOL_METADATA_H
#include <pmt/pmt.h>
#include <cstdint>

namespace gr {
namespace ofdm_prs_ranging {

// Check PMT type, sign and full-width bounds before any protocol narrowing.
inline bool bounded_uint(const pmt::pmt_t& meta, const char* key,
                         uint64_t fallback, uint64_t maximum, uint64_t& result)
{
    const auto value = pmt::dict_ref(meta, pmt::mp(key), pmt::PMT_NIL);
    if (pmt::is_null(value)) {
        result = fallback;
    } else if (pmt::is_uint64(value)) {
        result = pmt::to_uint64(value);
    } else if (pmt::is_integer(value) && pmt::to_long(value) >= 0) {
        result = static_cast<uint64_t>(pmt::to_long(value));
    } else {
        return false;
    }
    return result <= maximum;
}

inline bool bounded_q10(const pmt::pmt_t& meta, int64_t& result)
{
    const auto value = pmt::dict_ref(meta, pmt::mp("poll_fine_delay_q10"), pmt::PMT_NIL);
    if (pmt::is_null(value)) {
        result = 0;
    } else if (pmt::is_uint64(value)) {
        const auto raw = pmt::to_uint64(value);
        if (raw > 32767) return false;
        result = static_cast<int64_t>(raw);
    } else if (pmt::is_integer(value)) {
        result = pmt::to_long(value);
    } else {
        return false;
    }
    return result >= -32768 && result <= 32767;
}

inline bool metadata_true(const pmt::pmt_t& meta, const char* key)
{
    return pmt::eq(pmt::dict_ref(meta, pmt::mp(key), pmt::PMT_F), pmt::PMT_T);
}
} // namespace ofdm_prs_ranging
} // namespace gr
#endif
