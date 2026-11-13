# Survey 地圖與 SDR 定位 Debug

執行：

```bash
python3 tools/anchor_survey_gui_bundle/survey_map_gui.py
```

使用 Python 標準函式庫與 tkinter，不需要 GNU Radio 或額外 pip 套件。

## 顯示 Position CSV

1. 按「載入 Survey CSV」，開啟產生實驗用 `anchors_enu.csv` 的同一份 LLH Survey。
   也可直接在 GUI 輸入站點；至少必須有 A0。
2. 確認 A0 與共同 Geoid N 和產生 solver ENU 設定時相同。
3. 在「Position CSV 即時 Debug」選取 `prs_position_csv_logger` 正在寫入的檔案，
   例如 `examples/CSV/complete_epoch_positions.csv`。
4. 按「開始 / 重新載入」。預設 0.2 Hz，每 5 秒讀取一次新增資料。
   檔案尚未建立時會顯示讀取狀態，建立後自動繼續。

綠點是最近的有效 SDR 位置，青色線是成功定位的歷史軌跡，最多保留 1000 點。
`success=0` 只更新失敗原因，不移動位置；失敗後恢復成功會另起一段軌跡。
失敗時保留的點呈橘色，暫停或長時間沒有新紀錄時呈灰色，避免把舊座標當成新定位。
狀態列顯示最近一筆紀錄的 cycle、poll、reason 及上次有效 ENU。

「跟隨定位」會在新定位到達時置中；取消後可自由拖曳／縮放地圖。
「定位置中」回到上次有效位置，「清除軌跡」清除線條但保留定位點。
「暫停」停止讀檔；再次按「開始 / 重新載入」會重新讀取檔案，重建最近的軌跡。
切換成離線模式可以完全不要求線上圖磚；沒有 MBTiles 時仍可在空白網格顯示軌跡。

## 座標與資料語意

- `x_m/y_m/z_m` 必須是 East/North/Up，單位為公尺。
- ENU 原點使用 A0 的天線位置，橢球高為 `ground_elevation_m + antenna_agl_m + Geoid N`，
  與本工具的 ENU 匯出相同。GUI 反轉 ENU/ECEF 座標後，將 WGS84 經緯度畫在地圖上。
  若 solver 使用自訂平移／旋轉座標系，須先轉成這個 Survey ENU 才能套用。
- 執行中若修改 A0 或 Geoid，顯示會暫停並清除舊位置；重新載入後才繼續。
- 使用紀錄順序更新，不以會回繞的 `poll_id` 去重。
- 必要 CSV 欄位為 `success,reason,x_m,y_m,z_m`；另會顯示可用的 `cycle_sequence`、`poll_id`。
  支援 logger 輸出的標準 CSV 引號及多行 reason；只接受成功且 XYZ 均為有限值的紀錄。
- 使用背景執行緒增量讀檔；沒有完整結尾換行的紀錄會等到下一次讀取。
  偵測檔案縮短、替換或已讀邊界被覆寫時，會清除先前軌跡並重新讀取。
- 初次讀取可顯示既有 CSV 的最近 1000 個成功位置；大檔案解析期間地圖仍可操作。
- 無新增紀錄達 `max(15 秒, 3 個更新週期)` 時顯示「資料未更新」。
  新鮮度由本機觀察到的新增紀錄與檔案修改時間估計，**不將 UHD `reference_timestamp`
  當成 Unix 時間**。這是檔案更新狀態，不是 RF 延遲量測。

此功能僅顯示原始定位結果供實驗 debug，沒有平滑、航向推估、路徑規劃或導航控制。

## 離線驗證

```bash
python3 -B tools/anchor_survey_gui_bundle/test_position_csv_reader.py
```

測試不連線下載地圖、不啟動桌面視窗，涵蓋持續追加／不完整紀錄、失敗列、檔案重建、
UTF-8／多行 CSV、ENU 轉換，以及 5 秒輪詢與地圖標記狀態。
