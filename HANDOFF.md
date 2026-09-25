# livox-mid360-core 引き継ぎドキュメント

作成日: 2026-09-25
目的: Livox Mid-360 用 SDK を公式 Livox-SDK2 に依存せずスクラッチで再実装する。本ドキュメントは別セッション（人間または AI）が作業を継続するための前提・決定事項・次の手順をまとめたもの。

---

## 1. 決定済み事項

| 項目 | 決定 |
|---|---|
| リポジトリ名 | `livox-mid360-core` |
| ライセンス | Apache-2.0 |
| 言語 | C++20 相当、`-std=c++23` でビルド（GCC 13 / Clang 19 以降を想定） |
| 対応 OS | Ubuntu 24.04 以降のみ（Windows・18.04/20.04/22.04 は非対応） |
| 対応機種 | Livox Mid-360 のみ（HAP 等は非対応） |
| 公式 SDK2 との関係 | コードは一切コピーしない。公式プロトコル仕様のみに基づくクリーンルーム実装。SDK2 は「実機挙動の照合先」としてのみ参照 |
| ROS 2 対応 | 予定あり。ただしコアと ROS 2 ノードは**別リポジトリ**に分離する（`livox-mid360-ros2`） |
| 「Livox」の商標 | README 冒頭に「Livox / DJI とは無関係の非公式実装」と明記する |

### 命名規約
- C++ 名前空間: `livox::mid360`
- CMake ターゲット: `livox::mid360_core`
- 共有ライブラリ: `liblivox_mid360_core.so`
- インクルードパス: `<livox/mid360/...>`
- 姉妹リポジトリ: `livox-mid360-ros2`（ROS 2 パッケージ名 `livox_mid360_ros2`、ノード名 `livox_mid360_driver`）、`livox-mid360-cli`
- GitHub topics: `livox`, `mid-360`, `lidar`, `ros2`, `cpp20`

### 決定済み（2026-09-25 のフェーズ 0 で確定）
- **バリアント範囲**: 無印 Mid-360 のみ。360S/360L 専用 key（`speed_mode` 0x0021、`pc_freq_mod` 0x0029）は enum に列挙するだけで型付きヘルパは持たない。
- **v1 スコープ**: ログ系（0x03xx）・ファーム更新系（0x04xx）は対象外（フェーズ 3 の任意項目）。
- **テスト**: Catch2 v3。apt の `catch2` があればそれを使い、無ければ FetchContent で取得。
- **言語標準**: `-std=c++23`。コードは C++20 相当だが `std::expected` が libstdc++/libc++ とも C++23 でしか有効にならないため。Clang 18 + libstdc++ は `__cpp_concepts` が 201907 のため `<expected>` が使えず、Clang は 19 以降（Ubuntu 24.04 の apt に存在）とする。
- **対応ファームウェア下限**: 未確定（実機で v13.18.0244 を基準に検証してから決める）。

---

## 2. 参照資料

### 公式
- **プロトコル本体**（最重要。パケット定義・key-value 一覧・CRC はここ）
  https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html
  - 最新改訂 v1.4.12（2026-09-21）で Mid-360L の記述が追加されている。
- Mid-360 目次ページ（時刻同期方式・HMS へのリンク）
  https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/mid360.html
- 時刻同期の共通ページ（PTP / gPTP / GPS）
  https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/common/time_sync.html
- HMS 診断コード（key 0x8011 のデコード表）
  https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/hms_code_mid360.html
- 座標系・スキャンパターン
  https://livox-wiki-en.readthedocs.io/en/latest/introduction/Point_Cloud_Characteristics_and_Coordinate_System%20.html
- ダウンロード（User Manual 2024-04-25、Firmware v13.18.0244 とリリースノート 2025-04-11、Livox Viewer 2 Ubuntu 版）
  https://www.livoxtech.com/mid-360/downloads
- 公式 SDK2（照合用のみ。コピー禁止）
  https://github.com/Livox-SDK/Livox-SDK2
- 公式 ROS 2 ドライバ（下流が期待する点群フォーマットの参照用）
  https://github.com/Livox-SDK/livox_ros_driver2

### 非公式
- SDK2 非依存の軽量実装。最小構成の見本。ライセンス要確認、コードコピー禁止。
  https://github.com/Yancey2023/mid360_driver

---

## 3. プロトコル要点（wiki からの抜粋。実装時は必ず原典を再確認）

すべて UDP、リトルエンディアン。

### ポート
| 用途 | LiDAR 側 | ホスト側 | 備考 |
|---|---|---|---|
| Discovery (0x0000 のみ) | 56000 | 任意 | broadcast |
| 制御コマンド | 56100 | 任意（56101 推奨） | unicast |
| 状態 push (0x0102) | 56200 | 設定可（既定 56201） | |
| 点群 | 56300 | 設定可（既定 56301） | multicast 可（360L 不可） |
| IMU | 56400 | 設定可（既定 56401） | multicast 可（360L 不可） |
| ログ | 56500 | 任意（56501 推奨） | |

LiDAR 既定 IP は `192.168.1.1XX`（XX = SN 末尾2桁）。ホストは慣例的に `192.168.1.5` や `192.168.1.50`。

### 制御コマンドフレーム（24 バイトヘッダ + data、最大 1400 バイト）
| offset | size | field | 備考 |
|---|---|---|---|
| 0 | 1 | sof | 0xAA 固定 |
| 1 | 1 | version | 0 |
| 2 | 2 | length | sof から data 末尾まで |
| 4 | 4 | seq_num | REQ ごとに +1、ACK は同値 |
| 8 | 2 | cmd_id | |
| 10 | 1 | cmd_type | 0x00 REQ / 0x01 ACK |
| 11 | 1 | sender_type | 0x00 host / 0x01 LiDAR |
| 12 | 6 | resv | |
| 18 | 2 | crc16 | 先頭 18 バイト対象 |
| 20 | 4 | crc32 | data 対象。data 長 0 なら 0 埋め |
| 24 | n | data | |

### 主要 cmd_id
- 0x0000 Discovery（ACK: ret_code, dev_type, SN[16], lidar_ip[4], cmd_port）
- 0x0100 パラメータ設定 / 0x0101 参照 / 0x0102 push（key-value list 形式: key u16, length u16, value）
- 0x0200 再起動 / 0x0201 工場出荷 / 0x0202 GPS 時刻設定
- 0x03xx ログ、0x04xx ファーム更新（v1 では対象外予定）

### 主要 key（0x0100 で設定可）
| key | name | 内容 |
|---|---|---|
| 0x0000 | pcl_data_type | 1: Cartesian 32bit, 2: Cartesian 16bit, 3: Spherical |
| 0x0001 | pattern_mode | 0 のみ有効（非反復） |
| 0x0004 | lidar_ipcfg | IP/mask/gateway |
| 0x0005 | state_info_host_ipcfg | push 先 IP/port/src port |
| 0x0006 | pointcloud_host_ipcfg | 点群送信先 |
| 0x0007 | imu_host_ipcfg | IMU 送信先 |
| 0x0012 | install_attitude | roll/pitch/yaw/x/y/z |
| 0x0015/0x0016/0x0017 | fov_cfg0/1, fov_cfg_en | FOV 制限 |
| 0x0018 | detect_mode | 0 normal / 1 sensitive |
| 0x001A | work_tgt_mode | 目標状態（0x01 SAMPLING 等） |
| 0x001C | imu_data_en | IMU push 有効化 |
| 0x0021 | speed_mode | モータ速度（360S/360L） |
| 0x0026 | time_filter | GPS 時刻ロールバック対策 |
| 0x002B | imu_sensor_cfg | IMU レート/レンジ |

読み取り専用: 0x8000 sn, 0x8001 product_info, 0x8002 version_app, 0x8005 mac, 0x8006 cur_work_state, 0x8007 core_temp, 0x8009 local_time_now, 0x800B time_offset, 0x800C time_sync_type, 0x800E lidar_diag_status, 0x8011 hms_code[8] など。

### 動作状態
SAMPLING 0x01 / IDLE 0x02 / ERROR 0x04 / SELFCHECK 0x05 / MOTORSTARTUP 0x06 / UPGRADE 0x08 / READY 0x09

### 点群・IMU パケット（36 バイトヘッダ + data）
| offset | size | field |
|---|---|---|
| 0 | 1 | version (0) |
| 1 | 2 | length |
| 3 | 2 | time_interval（0.1µs 単位、先頭点と末尾点の差） |
| 5 | 2 | dot_num |
| 7 | 2 | udp_cnt（フレーム開始で 0 リセット） |
| 9 | 1 | frame_cnt（非反復スキャンでは無効） |
| 10 | 1 | data_type |
| 11 | 1 | time_type（0 未同期 / 1 PTP・gPTP / 2 GPS） |
| 12 | 12 | reserved（`pack_info.tag_type` を含むとされるが詳細は原典確認） |
| 24 | 4 | crc32（timestamp + data 対象） |
| 28 | 8 | timestamp（ns、先頭点の時刻） |
| 36 | – | data |

data_type: 0 IMU（gyro xyz rad/s, acc xyz g、float×6）、1 Cartesian 32bit（x,y,z int32 mm, reflectivity u8, tag u8 = 14 バイト × 96 点）、2 Cartesian 16bit（int16 10mm 単位、8 バイト × 96）、3 Spherical（depth u32 mm, theta u16, phi u16 0.01°, refl, tag = 10 バイト × 96）。
各点の時刻は `timestamp + i * time_interval / (N-1)` で等間隔補間。
tag: bit0-1 隣接物の接着点、bit2-3 雨・霧・塵、bit4-5 その他。各 0 高信頼 / 1 中 / 2 低。

### CRC
- CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, xorout 0, 入出力反転なし
- CRC-32: poly 0x04C11DB7, init 0xFFFFFFFF, xorout 0xFFFFFFFF, 入出力反転あり（= 標準 zlib CRC32）

### 時刻同期
PTP (IEEE1588v2.0 UDP)、gPTP (L2)、GPS (PPS + GPRMC)。v2.1 非対応、1588v2.0 と gPTP の併用非推奨、360L は gPTP 非対応。Linux 側は linuxptp (`ptp4l`) をマスターにした構成でテストする。

### 戻り値コード
0x00 成功 / 0x01 失敗 / 0x02 現状態では不可 / 0x03 範囲外 / 0x20 未対応パラメータ / 0x21 再起動で有効 / 0x22 読み取り専用 / 0x23 長さ不正 / 0x24 key_num 不一致 / 0x30〜0x34 更新系エラー

---

## 4. アーキテクチャ方針

- **層を分ける**: ①純粋関数のプロトコル層（CRC・シリアライズ・パース、I/O なし）→ ②トランスポート層（UDP ソケット、受信スレッド）→ ③デバイス層（セッション・状態機械・再送・push 処理）→ ④公開 API。①はソケットなしで完全にテスト可能にする。
- **状態機械を明示的に持つ**: 0x0102 push の `cur_work_state` / `lidar_diag_status` / `hms_code` で更新し、上位に通知。SDK2 はここが暗黙的で、差別化ポイント。
- **接続シーケンス**: 56000 へ broadcast 0x0000 → ACK の lidar_ip/cmd_port で unicast セッション → 0x0100 で host_ipcfg 3 種と pcl_data_type/imu_data_en を設定 → work_tgt_mode=SAMPLING → 点群・IMU 受信開始。
- **ACK 管理**: seq_num で突き合わせ、タイムアウト + 再送。1400 バイト上限を守る。
- **受信**: 受信スレッドとパース/コールバックを分離。`udp_cnt` の飛びでドロップを検出・計数。`recvmmsg` / `SO_REUSEPORT` は 24.04 前提で利用可。
- **時刻**: `time_type` を露出。未同期時は「最初のパケットでホスト時刻との差を 1 回だけ取る」をデフォルトに、PTP 時は無変換モードを用意。
- **依存ゼロに近づける**: SDK2 の rapidjson/spdlog 同梱のようなことはしない。設定ファイルは SDK2 の `MID360_config.json` を読める互換ローダを用意すると移行が楽。
- **出力の 3 層**: 生パケット / フレーム単位（時間窓で区切り） / ROS 2 互換（livox_ros_driver2 の PointXYZRTLT・CustomMsg 相当: x,y,z,intensity,tag,line,timestamp または offset_time）。
- **公開 API**: C++20 ライブラリ + 将来の他言語バインディングのための薄い C ABI。
- **安全性**: `std::span` / `std::expected` でパーサを書く。ASan/UBSan、fuzz テストを CI に入れる。

---

## 5. ディレクトリ構成（叩き台）

```
livox-mid360-core/
├── CMakeLists.txt
├── cmake/                     # config/version ファイル、警告設定
├── include/livox/mid360/
│   ├── crc.hpp                # CRC-16/CCITT-FALSE, CRC-32
│   ├── protocol.hpp           # コマンドフレーム/点群パケットの型とパーサ
│   ├── keys.hpp               # key 定義（0x0000〜0x8011）
│   ├── device.hpp             # 状態機械・セッション
│   └── mid360.hpp             # 公開 API のまとめ
├── src/
├── tools/                     # pcap 解析用 Python（リファレンス実装）
├── tests/
│   ├── fixtures/*.pcap        # 実機から採取
│   └── *.cpp                  # Catch2 or GoogleTest
├── docs/
│   └── protocol_notes.md      # wiki に無い実機挙動のメモ
├── README.md                  # 非公式実装であることを冒頭に明記
└── LICENSE                    # Apache-2.0
```

---

## 6. ロードマップ

### フェーズ 0（決定）
- [x] バリアント範囲・v1 スコープを確定（§1）。ファーム下限のみ実機待ち
- [x] リポジトリ作成、LICENSE (Apache-2.0)、README 骨格、CI（GitHub Actions、Ubuntu 24.04、GCC/Clang、ASan/UBSan）

### フェーズ 1（実機なしで進められる）
- [x] CRC-16 / CRC-32 実装と既知ベクタによるテスト
- [x] コマンドフレームのシリアライザ/パーサ + テスト
- [x] 点群/IMU パケットパーサ（data_type 0/1/2/3）+ テスト
- [x] key-value list エンコード/デコード + key 定義
- [x] Python pcap 解析スクリプト（tools/）を書き、C++ 実装の照合先にする

### フェーズ 2（実機なしで進められる分は GitHub issue #2〜#10 参照）
- [x] UDP トランスポート（`transport.hpp`、#2、docs/transport.md）
- [x] LiDAR シミュレータ（`tools/livox_mid360_sim.py`、#3、docs/simulator.md）。実機で要検証の仮定は docs/simulator.md 末尾と #11
- [ ] 実機で pcap 採取（discovery〜SAMPLING 遷移、点群、IMU、push）→ tests/fixtures/
- [ ] discovery、セッション確立、パラメータ設定、SAMPLING 遷移
- [ ] 点群/IMU 受信、ドロップ計数、フレーム分割
- [ ] 状態機械と push 処理、HMS デコード
- [ ] 切断・再起動からの自動復帰、複数台
- [ ] 数時間の長時間受信テスト、Livox Viewer 2 との目視比較

### フェーズ 3
- [ ] PTP (linuxptp) / GPS 時刻同期の検証、タイムスタンプ単調性
- [ ] C ABI、CLI ツール（`livox-mid360-cli`）
- [ ] `livox-mid360-ros2`（rclcpp、コンポーザブルノード、livox_ros_driver2 互換出力）
- [ ] （任意）ログ 0x03xx、ファーム更新 0x04xx

---

## 7. 別セッションへの最初の依頼文（例）

> `livox-mid360-core`（HANDOFF.md 参照）のフェーズ 1 を進めてください。C++20、Apache-2.0、Ubuntu 24.04 前提。CMake 骨格、`crc.hpp` と `protocol.hpp` の実装、Catch2（または GoogleTest）によるテスト、GitHub Actions の CI を作成してください。公式 SDK2 のコードは参照・コピーしないこと。プロトコル仕様は https://livox-wiki-en.readthedocs.io/en/latest/tutorials/new_product/mid360/livox_eth_protocol_mid360.html を原典としてください。
