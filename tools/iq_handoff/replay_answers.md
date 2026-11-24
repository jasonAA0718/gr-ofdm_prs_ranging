# handoff.cf32 離線驗收答案

以 `replay_prs_iq` 從檔案開頭重播，Window Capture 門檻為 `0.09`，ZC Frame Sync 的 preamble／ZC 門檻為 `0.15 / 0.35`。以下十筆全部解為 `packet_type=1`（POLL）、`anchor_id=0`、`frame_id_valid=#t`（通過 CRC 與保留位元檢查）。

| poll_id | frame_start | ZC 起點（coarse_peak） | GNU Radio SNR（dB） |
|---:|---:|---:|---:|
| 344 | 14022979 | 14026155 | 9.09 |
| 345 | 29025142 | 29028318 | 9.21 |
| 346 | 44027305 | 44030481 | 7.87 |
| 347 | 59029468 | 59032644 | 14.26 |
| 348 | 74033672 | 74036848 | 10.65 |
| 349 | 89033795 | 89036971 | 10.97 |
| 350 | 104037998 | 104041174 | 11.40 |
| 351 | 119040161 | 119043337 | 10.88 |
| 352 | 134042325 | 134045501 | 11.20 |
| 353 | 149044488 | 149047664 | 11.02 |

表中位置都是從檔案開頭算起的 **0-based complex sample offset**。`frame_start` 包含前置 guard，第一個 OFDM CP 在 `frame_start + 3995`。SNR 是專案的 pilot 通道估計指標，核對時先以 frame 邊界和 CRC 為準。

延伸題：保留 Window Capture 門檻 `0.09`，把 ZC Frame Sync 的 preamble 門檻從 `0.15` 降到 `0.09`。對照前六包的 ZC 選峰和 FFT 視窗。先前的 replay 在這段錄音選到早 262 samples 的 frame 起點，六筆 CRC 都失敗。
