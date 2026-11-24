# handoff.cf32 錄製設定

| 項目 | 設定 |
|---|---|
| 內容 | Initiator 傳送的 POLL |
| IQ 格式 | Little-endian float32，每筆 complex sample 為 I、Q 各一個 float32，無檔頭 |
| 樣本數與長度 | 150,000,000 complex samples，5 秒，1,200,000,000 bytes |
| 取樣率與中心頻率 | 30 MS/s、1330 MHz |
| 錄製 GRC 的 RX 設定 | `TX/RX`、gain 30 dB、bandwidth 30 MHz |
| Preamble | 每段 128 samples，重複 17 次 |
| ZC | 長度 819、root 25 |
| OFDM | FFT 512、CP 64、127 symbols |
| Gold code | ID 2，取 `lib/DSP/gold127_family_bipolar.csv` 的第 3 列（MATLAB 從 1 開始） |
| Guard 與 frame | 前後各 1000 samples，完整 frame 共 78,147 samples |
| Frame 內偏移（從 0 起算） | ZC 起點 3176，第一個 OFDM symbol 的 CP 起點 3995 |
| GNU Radio 驗收設定 | 從檔案開頭連續重播。Window Capture preamble 門檻 0.09，ZC Frame Sync preamble 門檻 0.15，ZC 門檻 0.35 |

MATLAB 可用 `fopen('handoff.cf32','rb','ieee-le')` 開檔。用 `fread(fid,[2,N],'single=>single')` 分批讀取，再以 `complex(iq(1,:),iq(2,:))` 組成 IQ。檔案 sample offset 從 0 起算，MATLAB 陣列索引則加 1。以 `signal.md` 核對波形、scrambler 與 CRC，另外附上 Gold code CSV，讓學弟能從 MATLAB 重現解碼。
