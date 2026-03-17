<a name="readme-top"></a>

[JA](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SOBITS VLA Tools

<!-- 概要 -->
## 概要

SOBITS VLA Toolsは，SOBITS自作ロボットをVision-Language-Action（VLA）モデルで制御するための統合パイプラインを提供するモノレポです．データ収集から学習，リアルタイム推論まで，すべてROS 2上で動作します．

### パッケージ一覧

| パッケージ | 説明 |
| ---------- | ---- |
| [sobits_vla_collection](./sobits_vla_collection/) | ゲームパッドによるマルチモーダルrosbag記録（リアルタイム品質監視付き） |
| [sobits_vla_conversion](./sobits_vla_conversion/) | rosbagを[LeRobot](https://github.com/huggingface/lerobot)データセット形式に変換 |
| [sobits_vla_training](./sobits_vla_training/)` | モデル学習ユーティリティ（TBD） |
| [sobits_vla_deploy](./sobits_vla_deploy/) | ロボット制御用リアルタイムVLA推論ノード（TBD） |
| [sobits_vla_visualization](./sobits_vla_visualization/) | データセット・推論の可視化（TBD） |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- セットアップ -->
## 環境構築

### 環境条件

| System | Version |
| ------ | ------- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS    | Humble Hawksbill        |
| Python | ≥3.10                  |

> [!NOTE]
> `Ubuntu`や`ROS`のインストール方法に関しては，[SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6)を参照してください．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### インストール方法

1. ROSの`src`フォルダに移動します．
    ```sh
    $ cd ~/colcon_ws/src/
    ```
2. 本リポジトリをcloneします．
    ```sh
    $ git clone https://github.com/TeamSOBITS/sobits_vla_tools
    ```
3. 必要な依存パッケージをインストールします．
    ```sh
    $ cd sobits_vla_tools/
    $ bash install.sh
    ```
4. パッケージをコンパイルします．
    ```sh
    $ cd ~/colcon_ws
    $ rosdep update
    $ rosdep install --from-paths src -y --ignore-src
    $ colcon build
    $ source install/setup.bash
    ```

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- パッケージ -->
## パッケージ

### 1. データ収集

**パッケージ:** [sobits_vla_collection](./sobits_vla_collection/)

ゲームパッドコントローラーを使用して，カメラ・関節状態・オドメトリ・LiDAR・TFなどのマルチモーダルセンサーデータをrosbagエピソードとして記録します．

#### 起動方法

```bash
ros2 launch sobits_vla_rosbag_collection rosbag_collection.launch.py \
  robot_name:=sobit_light \
  record_directory:=/path/to/rosbags
```

| 引数 | デフォルト | 説明 |
| ---- | ---------- | ---- |
| `robot_name` | （必須） | ロボット名 — `record_settings_<robot_name>.yaml`設定ファイルと一致する必要あり |
| `record_directory` | `<package_share>/rosbags` | rosbagエピソードの保存先の絶対パス |

#### ゲームパッド操作

| ボタン | 動作 |
| ------ | ---- |
| Record/Pause | 記録開始 / 一時停止 / 再開 |
| Save | 現在のエピソードを保存 |
| Save（停止中） | 最後に保存したエピソードを削除（取り消し） |

ボタンマッピングは[gamepad_settings.yaml](./sobits_vla_rosbag_collection/config/gamepad_settings.yaml)で設定します．

#### 記録品質モニタリング

収集ノードは記録中にデータ品質をリアルタイムで監視します：

| モニター | 説明 |
| -------- | ---- |
| **FPS監視** | カメラの配信レートが設定閾値を下回った場合に警告 |
| **ディスク容量** | 空き容量が閾値を下回った場合に警告；危険レベルで記録停止 |
| **最小エピソード長** | 設定時間より短いエピソードを拒否 |
| **タイムスタンプジャンプ** | ROSクロックとウォールタイムの不整合を検出 |
| **Bag整合性** | 保存後にbagファイルが読み取り可能で空でないことを検証 |
| **設定一致性** | 再開時に現在の設定が既存の`recorded_bags_meta.yaml`と一致するか検証 |

#### 設定

ロボット固有設定: `config/record_settings_<robot_name>.yaml`

| グループ | 主要パラメータ |
| -------- | -------------- |
| ロボット形態 | `parts`, `joint_names`, `is_actionable`, `joint_states_topic` |
| センサー | カメラトピック, LiDAR, IMU |
| 記録 | `topics_to_record`, 圧縮形式/モード |
| モニタリング | `expected_sensor_fps`, `min_disk_space_warning_gb`, `min_episode_duration` |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 2. データセット変換

**パッケージ:** [sobits_vla_conversion](./sobits_vla_conversion/)

生のrosbag記録を時刻同期されたマルチモーダルフレームで[LeRobot](https://github.com/huggingface/lerobot)データセット形式に変換します．

#### 起動方法

```bash
ros2 launch sobits_vla_rosbag_conversion rosbag_conversion.launch.py \
  config_file:=conversion_settings.yaml \
  rosbag_directory:=/path/to/rosbags \
  dataset_name:=MyDataset
```

| 引数 | デフォルト | 説明 |
| ---- | ---------- | ---- |
| `config_file` | `conversion_settings.yaml` | 変換設定ファイル |
| `rosbag_directory` | （収集パッケージから） | 記録済みrosbagエピソードのパス |
| `recorded_bags_meta_file` | `<rosbag_directory>/recorded_bags_meta.yaml` | 収集時のメタデータファイル |
| `dataset_name` | （設定から） | 出力データセット名 |

#### 主な機能

- **フレーム同期**: プライマリカメラトリガーと設定可能な同期閾値でカメラ・関節状態・コマンドデータを整列
- **ダウンサンプリング**: 設定可能なターゲットFPS — 要求レートを満たせないbagはスキップ
- **デルタアクション**: フレームごとに`action.delta`（指令値 - 測定値）を計算
- **エンドエフェクター姿勢**: TFツリーによるオプションの6-DOF姿勢抽出
- **静止フレームフィルタリング**: 関節が動いていないフレームをオプションでスキップ
- **変換統計**: エピソードごとの品質指標，スキップされたbagとその理由のYAMLレポートを生成

#### 設定

設定ファイル: [conversion_settings.yaml](./sobits_vla_rosbag_conversion/config/conversion_settings.yaml)

| パラメータ | デフォルト | 説明 |
| ---------- | ---------- | ---- |
| `fps` | `10` | ターゲットデータセットフレームレート |
| `sync_threshold` | `0.1` | 同期センサー間の最大時間差（秒） |
| `primary_camera` | `head_camera` | 同期トリガーとして使用するカメラ |
| `cameras` | `head_camera, hand_camera` | データセットに含めるカメラ |
| `ee_pose.enabled` | `false` | エンドエフェクター姿勢抽出を有効化 |
| `skip_static_threshold` | `0.0` | 静止フレームフィルタリングの関節移動閾値（0 = 無効） |
| `push_to_hub` | `false` | 結果のデータセットをHuggingFace Hubにプッシュ |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 3. 学習

**パッケージ:** [sobits_vla_training](./sobits_vla_training/)

> [!NOTE]
> TBD — 学習ユーティリティは開発中です．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 4. 推論・デプロイ

**パッケージ:** [sobits_vla_deploy](./sobits_vla_deploy/)

ロボット上でリアルタイムVLA推論を実行します．カメラストリームと関節状態をサブスクライブし，ポリシーモデルを実行して軌道コマンドをパブリッシュします．

> [!NOTE]
> TBD — 推論ユーティリティは開発中です．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 5. 可視化

**パッケージ:** [sobits_vla_visualization](./sobits_vla_visualization/)

> [!NOTE]
> TBD — 可視化ツールは開発中です．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- ワークフロー -->
## ワークフロー

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  1. 収集         │────▶│  2. 変換         │────▶│  3. 学習         │────▶│  4. デプロイ     │
│  (rosbag_        │     │  (rosbag_        │     │  (training)      │     │  (deploy)        │
│   collection)    │     │   conversion)    │     │                  │     │                  │
│                  │     │                  │     │                  │     │                  │
│  ゲームパッドで   │     │  Rosbags →       │     │  VLAモデルの     │     │  ロボット上で    │
│  エピソード      │     │  LeRobotデータ   │     │  ファインチューン │     │  リアルタイム    │
│  記録            │     │  セットに変換    │     │                  │     │  推論            │
└─────────────────┘     └─────────────────┘     └─────────────────┘     └─────────────────┘
```

1. ゲームパッドでロボットを遠隔操作してデモデータを**収集**
2. 記録したrosbagをLeRobot互換データセットに**変換**
3. 収集したデータセットでVLAモデルを**学習**
4. 学習済みモデルを自律ロボット制御に**デプロイ**

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 参考文献 -->
## 参考文献

- [LeRobot](https://github.com/huggingface/lerobot) — データセット形式と学習フレームワーク
<!-- - [SmolVLA](https://huggingface.co/HuggingFaceTB/SmolVLA-256) — VLAモデルアーキテクチャ -->
- [ROS 2 Humble](https://docs.ros.org/en/humble/) — ロボットミドルウェア

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- MARKDOWN LINKS & IMAGES -->
[contributors-shield]: https://img.shields.io/github/contributors/TeamSOBITS/sobits_vla_tools.svg?style=for-the-badge
[contributors-url]: https://github.com/TeamSOBITS/sobits_vla_tools/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/TeamSOBITS/sobits_vla_tools.svg?style=for-the-badge
[forks-url]: https://github.com/TeamSOBITS/sobits_vla_tools/network/members
[stars-shield]: https://img.shields.io/github/stars/TeamSOBITS/sobits_vla_tools.svg?style=for-the-badge
[stars-url]: https://github.com/TeamSOBITS/sobits_vla_tools/stargazers
[issues-shield]: https://img.shields.io/github/issues/TeamSOBITS/sobits_vla_tools.svg?style=for-the-badge
[issues-url]: https://github.com/TeamSOBITS/sobits_vla_tools/issues
[license-shield]: https://img.shields.io/github/license/TeamSOBITS/sobits_vla_tools.svg?style=for-the-badge
[license-url]: LICENSE
