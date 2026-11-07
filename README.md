# M5Stack Tab5 Dual Camera

M5Stack Tab5（ESP32-P4）で、内蔵カメラ（SC202CS、MIPI-CSI）と USB-A の UVC カメラ（MJPEG）を横に並べて表示するアプリです。画面下のタッチ操作帯で、どちらかのカメラを選んで明るさ・コントラスト・彩度・色温度を変えられます。

操作帯の右端の **REC** で、内蔵カメラの映像（ハードウェア JPEG、MJPEG 画質70、1280x720 30fps）と内蔵マイクの音声（16kHz モノラル PCM）を、microSD に AVI 形式で録画できます。

操作帯の **WIFI** で、Tab5 がアクセスポイント `Tab5-Camera` になります（内蔵の ESP32-C6 を使用）。スマホやPCからつないで `http://192.168.4.1/` を開くと、動作状態の確認、録画ファイルのダウンロード、Wi-Fi の設定（アクセスポイントのパスワード、接続先のネットワーク）ができます。

> **アクセスポイントの初期パスワードは `APtest1234` で、このソースに書かれている公開の値です。** 使い始めたら、設定ページで変えてください（変えた値は本体の NVS に保存されます）。

> このリポジトリは、非公開の開発リポジトリから自動で同期している読み取り専用のコピーです。ここに直接 push した変更は、次の同期で上書きされます。

## 動作を確認したハードウェア

| 項目 | 内容 |
|---|---|
| 本体 | M5Stack Tab5（LCD ILI9881C ＋ タッチ GT911 の版） |
| SoC | ESP32-P4 **revision v1.0** |
| 内蔵カメラ | SC202CS、1280x720 RGB565 |
| UVC カメラ | ELP 製 USB カメラ（MJPEG 640x480 を要求） |

- LCD・タッチが ST7123 の新しい版の Tab5 では、確認していません。
- ESP32-P4 v3.0 以降のチップでも確認していません（下の「rev1.0 向けの設定」を参照）。

## 必要なもの

- **ESP-IDF 5.5.6**（`main/idf_component.yml` で `>=5.5.6,<5.6` に固定しています）
- IDF Component Manager（ESP-IDF に同梱）。初回のビルドで、`dependencies.lock` に書かれた版の部品をダウンロードします。

## ビルドと書き込み

ESP-IDF 5.5.6 の環境（`export.sh` / `export.ps1` などを実行したシェル）で、リポジトリのルートから実行します。

```bash
idf.py build
idf.py -p <PORT> flash monitor
```

- `<PORT>` は Tab5 の USB-C（USB-Serial/JTAG）のポートです（Windows なら `COM7` など、Linux なら `/dev/ttyACM0` など）。
- ターゲットは `sdkconfig` で `esp32p4` に設定済みです。`idf.py set-target` は不要です。実行すると `sdkconfig` が作り直されるので、実行しないでください。
- パーティションテーブルは `partitions.csv`（アプリ領域 6MB）です。書き込みは、bootloader（0x2000）・パーティションテーブル（0x8000）・アプリ（0x10000）の3つが対象です。`idf.py flash` なら全部まとめて書かれます。

## rev1.0 向けの設定（重要）

Tab5 に載っている ESP32-P4 は revision v1.0 です。v3.0 以降とは違う点があり、このリポジトリはそれに合わせています。

- **`sdkconfig.defaults` に `CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y` と `CONFIG_ESP32P4_REV_MIN_100=y` を入れています。** v3 以降向けの設定でビルドしたものは、v1.0 では動かない前提で扱ってください。
- **`override/esp_video/` は、Espressif の `esp_video` 2.0.1 にパッチを当てたコピーです。** v3.0 未満のチップは CSI ブリッジで色変換ができないため、`src/device/esp_video_csi_device.c` の `csi_video_start` 付近を変更しています（コメント「Local patch」）。`main/idf_component.yml` の `override_path` でこのコピーを使います。部品を更新するときに、この上書きを外さないでください。外すと内蔵カメラが映りません。
- ハードウェアのホワイトバランス（WBG）は、v1.0 では実行時に `ESP_ERR_NOT_SUPPORTED` になります。色温度の調整は ISP の CCM（色補正行列）で行っています。

## その他の必要な設定

- `CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE=4096`: UVC カメラの Configuration Descriptor が大きく、既定の 256 バイトでは受け取れません。
- `CONFIG_SPIRAM_SPEED_200M=y`、`CONFIG_FREERTOS_HZ=1000`、`CONFIG_BSP_LCD_DPI_BUFFER_NUMS=2`（表示の二重バッファ）。
- 内蔵カメラは `CONFIG_CAMERA_SC202CS_MIPI_RAW8_1280X720_30FPS` を使います。

これらはすべて `sdkconfig.defaults` と `sdkconfig` に入っています。

## 動作の目安

- 内蔵カメラ: 約30fps
- UVC（640x480 MJPEG）: 受信・デコードとも約16〜26fps
- 表示: 約15fps（PSRAM 上の画像を PPA で回転・縮小するところの帯域が上限）

## 既知の注意点

- UVC 受信中に `uvc-isoc: missed EoF` が周期的に出ます。MJPEG が大きい場面では `usb err 1` や `invalid MJPEG SOI` も出ます。壊れたフレームはドライバで捨てられ、`decode_errors` は 0 のままなので、動作は続きます。
- ELP のカメラは、`uvc_host_stream_stop()` を呼ぶと制御エンドポイントが応答しなくなります。このアプリでは、カメラを止めたいときに USB-A の 5V を入れ直しています。
- 画像の設定は RAM にだけ持っていて、再起動すると標準値に戻ります。
- 録画には **FAT32 の microSD カード**が必要です（ESP-IDF は NTFS を読めず、exFAT はこの設定では無効です）。アプリはカードをフォーマットしません。
- 録画ファイルは `REC/R0001.AVI` から順に番号が付き、5分ごとに次のファイルに切り替わります。録画を始めた直後の約0.2秒の音声は、マイクの起動音を避けるために捨てています。
- Wi-Fi は ESP32-C6 の工場出荷ファーム（1.4.1）と、ホスト側 `esp_hosted` 2.12 系の組み合わせで確認しています。C6 のファームを更新していない Tab5 で、`esp_hosted` を 3.x に上げないでください。
- `sdkconfig.defaults` の `CONFIG_ESP_HOSTED_SDIO_RESET_ACTIVE_HIGH=y` は、確認した実機で必要だった設定です（Tab5 用プリセットの既定は Low アクティブで、C6 とつながりませんでした）。
- 録画ファイルのダウンロードは約0.6Mbit/s と遅いです（5分の録画で40分程度）。改善は今後の課題です。
- 録画中は、REC ボタンが明るい赤になり、経過時間が表示されます。カードが使えないときは `NOSD`、書き込みに失敗したときは `ERR` と表示されます。

## 固定している部品の版（`dependencies.lock`）

| 部品 | 版 |
|---|---|
| espressif/m5stack_tab5 | 1.3.2 |
| espressif/esp_video | 2.0.1（`override/esp_video` のローカルコピー） |
| espressif/esp_cam_sensor | 2.0.1 |
| espressif/esp_ipa | 1.3.1 |
| espressif/esp_h264 | 1.0.4 |
| espressif/usb_host_uvc | 2.4.2 |
| espressif/usb | 1.6.0 |
| espressif/esp_lcd_ili9881c | 1.1.0 |
| espressif/esp_lcd_touch_gt911 | 1.2.1 |
| espressif/esp_lvgl_port | 2.9.0 |
| lvgl/lvgl | 9.6.0~1 |
| espressif/esp_hosted | 2.12.13 |
| espressif/esp_wifi_remote | 1.6.5 |
| espressif/mdns | 1.15.1 |

`dependencies.lock` 内のローカル部品の場所は、同期のときにリポジトリからの相対パス（`override/esp_video`）に書き換えています。初回のビルドで、部品管理ツールがこれを手元の絶対パスに書き戻します。そのとき `Can't reliably evaluate relative path` という警告が1回出ますが、部品の版は変わらず、ビルドにも影響しません（クリーンな環境でビルドできることを確認済みです）。

## ディレクトリ

| パス | 内容 |
|---|---|
| `main/` | アプリ本体（カメラ、表示合成、UVC 制御、タッチ UI、録画、Wi-Fi、Web ページ） |
| `main/app_config.h` | 解像度、タスク優先度、UVC の見張り時間などの設定。`APP_*_TEST` は診断用なので 0 のままにしてください |
| `override/esp_video/` | rev1.0 用パッチを当てた `esp_video` |
| `partitions.csv` | パーティションテーブル |
| `sdkconfig.defaults` / `sdkconfig` | ビルド設定 |

## ライセンス

- このリポジトリのファイルは、`override/esp_video/` を除き **MIT License** です（`LICENSE`）。改変・再配布は自由です。無保証で、使用によって生じたことについて作者は責任を負いません。
- `override/esp_video/` は Espressif Systems のコードで、Apache License 2.0 のままです（`override/esp_video/LICENSE`）。ローカルで変更した箇所には、コメントで印を付けています。
