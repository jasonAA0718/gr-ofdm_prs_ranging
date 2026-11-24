# PRS IQ 交接工具

1. **先錄 Initiator 的 POLL。** 用現有實驗設定讓 Initiator 週期發送，另一台 USRP 開啟 `record_prs_iq.grc` 接收。錄製器會在收到指定數量的 samples 後停止。若只有原本的 Initiator／Responder 兩台設備，也可以先停掉 Responder flowgraph，再由錄製器使用它的 USRP。若要在正常測距時同步錄製，可在既有 flowgraph 的 UHD Source 輸出分接 `Head → File Sink`，讓原接收鏈繼續跑。
2. **核對錄製設定。** 將 `samp_rate`、`center_freq` 改成實際 TX 設定，依設備設定 RX antenna、gain 與 UHD device address。預設 30 MS/s、1330 MHz、TX/RX、30 dB、1 秒是起始設定。錄製時間至少涵蓋數個 POLL 週期。30 MS/s 的 complex float32 每秒約 240 MB。每次錄製使用新檔名，因為 File Sink 設為覆寫。
3. **自己先重播驗收。** 開啟 `replay_prs_iq.grc`，把 `iq_file` 指向錄製檔，設定相同取樣率及 TX 的 preamble、ZC 長度、root、Gold code ID。預設使用 128／17／819、root 25、Gold ID 2。錄 RESPONSE 時使用該 Responder TX 的 root，專案範例常用 29。用絕對路徑可避免從不同目錄啟動時找錯檔。
4. **交付可核對的結果。** 把 IQ、`replay_prs_iq.grc`、`docs/signal.md`、錄製參數與一筆成功訊息一起交給學弟。成功訊息記下 `frame_start`、`packet_type`、`poll_id`、`anchor_id`、`frame_id_valid=#t`。POLL 的 anchor ID 通常為 0。

錄製內容是 raw GNU Radio Complex samples，每筆為交錯的 32-bit float I/Q，在此 Linux x86 環境為 little-endian，沒有檔頭。離線 GRC 需要安裝本專案的 GNU Radio OOT blocks。它使用 Responder 的 Window Capture 與 ZC Frame Sync 連續搜尋 IQ，到 EOF 停止。終端顯示同步診斷（含失敗原因）與解碼的 PDU metadata。若有 frame 但 CRC 失敗，優先核對 Gold code、取樣率及接收品質。若沒有 frame，先核對中心頻率、訊號是否錄到、ZC root 和 preamble。

raw IQ 沒有 UHD `rx_time` tags 或原始 TDMA 排程。這份練習用於同步與解碼，完整 SS-TWR 重現另需時間與排程紀錄。
