#!/usr/bin/env python3

import argparse
import csv
import hashlib
from pathlib import Path


SYMBOLS = 16
FFT_LEN = 1024


def cpp_float(value):
    number = float(value)
    text = format(number, ".9g")
    if "." not in text and "e" not in text:
        text += ".0"
    return text + "f"


def load_rows(path):
    with path.open(newline="", encoding="ascii") as source:
        rows = list(csv.DictReader(source))

    expected_count = SYMBOLS * FFT_LEN
    if len(rows) != expected_count:
        raise ValueError(f"expected {expected_count} rows, found {len(rows)}")

    values = []
    for flat_index, row in enumerate(rows):
        symbol = flat_index // FFT_LEN
        fft_bin = flat_index % FFT_LEN
        if int(row["symbol_index"]) != symbol or int(row["fft_bin"]) != fft_bin:
            raise ValueError(
                f"row {flat_index + 2} is not symbol-major native FFT order"
            )
        expected_signed = fft_bin if fft_bin < FFT_LEN // 2 else fft_bin - FFT_LEN
        if int(row["signed_bin"]) != expected_signed:
            raise ValueError(f"row {flat_index + 2} has an invalid signed_bin")
        values.append((row["pilot_real"], row["pilot_imag"]))
    return values


def render(values, csv_hash):
    entries = []
    for real, imag in values:
        entries.append(f"{{ {cpp_float(real)}, {cpp_float(imag)} }}")

    lines = []
    for index in range(0, len(entries), 8):
        lines.append("    " + ", ".join(entries[index : index + 8]) + ",")

    table = "\n".join(lines)
    return f"""/* -*- c++ -*- */
/*
 * Generated from lib/golay_ofdm_1024x16.csv.
 * CSV SHA-256: {csv_hash}
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_OFDM_PRS_RANGING_GOLAY_PRS_TABLE_H
#define INCLUDED_OFDM_PRS_RANGING_GOLAY_PRS_TABLE_H

#include <array>
#include <cstddef>

namespace gr {{
namespace ofdm_prs_ranging {{

constexpr std::size_t golay_prs_symbol_count = {SYMBOLS};
constexpr std::size_t golay_prs_fft_len = {FFT_LEN};

struct golay_prs_value {{
    float real;
    float imag;
}};

inline constexpr std::array<golay_prs_value,
                            golay_prs_symbol_count * golay_prs_fft_len>
    golay_prs_table = {{{{
{table}
}}}};

constexpr const golay_prs_value& golay_prs_at(std::size_t symbol,
                                              std::size_t fft_bin)
{{
    return golay_prs_table[symbol * golay_prs_fft_len + fft_bin];
}}

}} // namespace ofdm_prs_ranging
}} // namespace gr

#endif /* INCLUDED_OFDM_PRS_RANGING_GOLAY_PRS_TABLE_H */
"""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("header_path", type=Path)
    args = parser.parse_args()

    csv_bytes = args.csv_path.read_bytes()
    values = load_rows(args.csv_path)
    output = render(values, hashlib.sha256(csv_bytes).hexdigest())
    args.header_path.write_text(output, encoding="ascii")


if __name__ == "__main__":
    main()
