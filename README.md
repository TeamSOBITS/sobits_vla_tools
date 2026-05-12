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
| [sobits_vla_rosbag_collection](./sobits_vla_rosbag_collection/) | ゲームパッドによるマルチモーダルrosbag記録（リアルタイム品質監視付き） |
| [sobits_vla_rosbag_conversion](./sobits_vla_rosbag_conversion/) | rosbagを[LeRobot](https://github.com/huggingface/lerobot)データセット形式に変換 |
| [sobits_vla_training](./sobits_vla_training/) | モデル学習ユーティリティ（TBD） |
| [sobits_vla_deploy](./sobits_vla_deploy/) | ロボット制御用リアルタイムVLA推論ノード（TBD） |
| [sobits_vla_visualization](./sobits_vla_visualization/) | データセット・推論の可視化（TBD） |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- セットアップ -->
## 環境構築

### 環境条件

| System | Version |
| ------ | ------- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS    | Jazzy Jalisco           |
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

**パッケージ:** [sobits_vla_rosbag_collection](./sobits_vla_rosbag_collection/)

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
現在のコントローラープロファイルは`quest`，`dualshock4`，`keyboard`に対応しています．

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

本リポジトリで主に使う設定:
- `config/record_settings_sobit_home.yaml`
- `config/record_settings_sobit_light.yaml`

| グループ | 主要パラメータ |
| -------- | -------------- |
| ロボット形態 | `parts`, `joint_names`, `is_actionable`, `joint_states_topic` |
| センサー | カメラトピック, LiDAR, IMU |
| 記録 | `topics_to_record`, 圧縮形式/モード |
| モニタリング | `expected_sensor_fps`, `min_disk_space_warning_gb`, `min_episode_duration` |

#### SOBIT HOME の起動

テレオペレーションノードや Quest アプリを起動する**前に**，ロボットを先に起動してください．

**実機：**

```bash
ros2 launch sobit_home_bringup real_minimal.launch.py \
  enable_teleop:=true
```

**シミュレーション（Gazebo）：**

```bash
ros2 launch sobit_home_bringup gz_minimal.launch.py \
  world_model:=simple_data_collection \
  enable_teleop:=true
```

`world_model` に指定できる値: `empty`, `wrs`, `small_house`, `rcjo2025_arena`, `rcjo2026_arena`, `simple_data_collection`．

**テレオペレーションノード（Quest，実機）：**

```bash
ros2 launch sobits_teleop sobits_teleop.launch.py \
  robot_name:=sobit_home \
  device:=quest \
  use_moveit:=true \
  ros_ip:=127.0.0.1
```

**テレオペレーションノード（Quest，シミュレーション）：**

```bash
ros2 launch sobits_teleop sobits_teleop.launch.py \
  robot_name:=sobit_home \
  device:=quest \
  use_moveit:=true \
  ros_ip:=127.0.0.1 \
  use_sim_time:=true
```

#### SOBIT HOME + Meta Quest でのデータ収集

SOBIT HOME のテレオペレーションは，[sobits_teleop](https://github.com/TeamSOBITS/sobits_teleop) を通じて Meta Quest ヘッドセットを使用します．Quest アプリは TCP ポート 10000 で PC と通信します．接続方法は2種類あります：

| 方法 | 使いどき |
| ---- | -------- |
| **有線（ADB）** | USB ケーブルで Quest を PC に接続 — 最も安定，ネットワーク不要 |
| **無線（Wi-Fi）** | ケーブルなしで操作；Quest と PC が同じネットワーク上にある必要あり |

---

##### オプション A — 有線接続（ADB）

launch ファイルが自動的に `adb reverse tcp:10000 tcp:10000` を実行し，USB 経由でアプリの接続をトンネリングします．Quest 側の IP 設定は不要です．

1. ヘッドセット**左側のボタン**を押して Quest を起動する．
2. USB ケーブルで Quest を PC に接続し，以下のコマンドを PC で**一度だけ**実行する（sobits_teleop のインストーラー実行済みであること）：
   ```bash
   sudo adb kill-server
   sudo adb start-server
   ```
3. ヘッドセット内に表示される **"このコンピュータからのUSBデバッグを常に許可する"** を選択して承認する．
4. デバイスが認識されているか確認する：
   ```bash
   adb devices
   ```
5. PC でテレオペレーションノードを起動する：
   ```bash
   ros2 launch sobits_teleop sobits_teleop.launch.py \
     robot_name:=sobit_home \
     device:=quest \
     use_moveit:=true \
     ros_ip:=127.0.0.1
   ```
6. ヘッドセット内でライブラリウィンドウを開き（**右コントローラーのMetaボタン**），**Menu → Unknown Sources** に移動して **Quest Teleoperation** アプリを起動する．
7. **左コントローラーの3本線ボタン**を押して設定パネルを開き，IP に `127.0.0.1` を入力して **OK** を押すと，ロボットのカメラ映像が表示される．

---

##### オプション B — 無線接続（Wi-Fi）

Quest と PC が同じ Wi-Fi ネットワークに接続されている必要があります．

**Quest ヘッドセット側：**

1. ヘッドセット**左側のボタン**を押して Quest を起動する．
2. **右コントローラーのMetaボタン**を押してライブラリウィンドウを開く．
3. **クイックコントロール**（2つのドットと3本線のアイコン）を開き，**Wi-Fi** を押す．
4. ネットワークを選択してパスワードを入力し，**"Connected"** が表示されるまで待つ．
5. 戻る矢印を押してから **Done** を押して設定を完了する．

**PC 側：**

6. 共有ネットワーク上の PC の IP アドレスを指定してテレオペレーションノードを起動する：
   ```bash
   ros2 launch sobits_teleop sobits_teleop.launch.py \
     robot_name:=sobit_home \
     device:=quest \
     use_moveit:=true \
     ros_ip:=<PC_IP_ADDRESS>
   ```
   `<PC_IP_ADDRESS>` を PC の IP アドレスに置き換えてください（例: `192.168.11.10`）．

**Quest に戻る：**

7. ライブラリウィンドウで **Menu → Unknown Sources** に移動し，**Quest Teleoperation** アプリを起動する．
8. **左コントローラーの3本線ボタン**を押して設定パネルを開き，PC の IP アドレスを入力して **OK** を押すと，ロボットのカメラ映像が表示される．

---

##### テレオペレーション起動引数

| 引数 | デフォルト | 説明 |
| ---- | ---------- | ---- |
| `robot_name` | `sobit_home` | ロボット設定プロファイル |
| `device` | `quest` | 入力デバイス — Quest コントローラー |
| `ros_ip` | `127.0.0.1` | Quest アプリが接続する PC の IP（有線 ADB の場合は `127.0.0.1`，無線の場合は PC の IP を確認） |
| `use_moveit` | `true` | `true` にすると MoveIt ベースのアーム制御が有効になる |

> [!TIP]
> Quest アプリを接続する**前に**，PC 側でロボットの bringup を先に起動しておくと，カメラトピックがすでに配信された状態でアプリが接続できます．

##### タスク名の設定

収集ノードはエピソードをタスクごとのディレクトリに整理します．記録を開始する前に，`/vla_task_update` サービスでタスク名を設定してください：

```bash
ros2 service call /rosbag_collection/vla_task_update sobits_interfaces/srv/VlaUpdateTask "{label: 'pick up the bottle'}"
```

タスク名を変更すると，以降のエピソードは新しいディレクトリに保存されます．

##### rosbag 収集ノードの起動とエピソードの記録

別ターミナルで rosbag 収集ノードを起動する：

```bash
ros2 launch sobits_vla_rosbag_collection rosbag_collection.launch.py \
  robot_name:=sobit_home \
  record_directory:=/path/to/rosbags
```

ノードが起動したら，Quest コントローラーで以下の操作を行います：

| ボタン | 状態 | 動作 |
| ------ | ---- | ---- |
| **A ボタン**（右コントローラー） | 待機中 | 記録開始 |
| **A ボタン**（右コントローラー） | 記録中 | 一時停止 / 再開 |
| **B ボタン**（右コントローラー） | 記録中 | エピソードを保存して停止 |
| **B ボタン**（右コントローラー） | 待機中 | 最後に保存したエピソードを削除 |

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 2. データセット変換

**パッケージ:** [sobits_vla_rosbag_conversion](./sobits_vla_rosbag_conversion/)

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
| `config_file` | `conversion_settings.yaml` | 変換設定ファイル（ロボットに応じて切り替え） |
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

ロボット別プリセット例: [conversion_settings_sobit_home.yaml](./sobits_vla_rosbag_conversion/config/conversion_settings_sobit_home.yaml)

| パラメータ | デフォルト | 説明 |
| ---------- | ---------- | ---- |
| `fps` | `10` | ターゲットデータセットフレームレート |
| `sync_threshold` | `0.1` | 同期センサー間の最大時間差（秒） |
| `primary_camera` | `head_camera` | 同期トリガーとして使用するカメラ |
| `cameras` | `head_camera, hand_left_camera, hand_right_camera` | データセットに含めるカメラ |
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

ロボット上でリアルタイムVLA推論を実行します．ポリシーが対応している場合，非同期チャンク実行（Async）とRTCによるチャンク接続の滑らかさ向上を利用できます．

#### 実行方法

```bash
ros2 run sobits_vla_deploy sobits_vla_deploy.py --ros-args \
  --params-file $(ros2 pkg prefix sobits_vla_deploy)/share/sobits_vla_deploy/config/robot_config.yaml
```

ロボット別設定を使う場合は，`robot_config_<robot_name>.yaml`を指定してください（例: `robot_config_sobit_home.yaml`）．

#### launchによる起動

```bash
ros2 launch sobits_vla_deploy sobits_vla_deploy.launch.py
```

ロボット別設定ファイルを指定する場合:

```bash
ros2 launch sobits_vla_deploy sobits_vla_deploy.launch.py \
  config_file:=$(ros2 pkg prefix sobits_vla_deploy)/share/sobits_vla_deploy/config/robot_config_sobit_home.yaml
```

#### 設定ファイル構成

- 汎用テンプレート: [robot_config.yaml](./sobits_vla_deploy/config/robot_config.yaml)
- ロボット別プリセット例: [robot_config_sobit_home.yaml](./sobits_vla_deploy/config/robot_config_sobit_home.yaml)

`robot`直下（`robot.name`でロボット名を指定）で以下を設定できます:
- `joint_states_topic` と `odom_topic`
- 複数の関節軌道コントローラグループ
- モバイルベース指令トピックと特徴量
- カメラトピックと画像エンコーディング

#### 複数コントローラー対応（gamepad）

デプロイ側gamepad設定は，コントローラーごとのボタンマッピングに対応しています．

```yaml
gamepad:
  topic: /joy
  name: quest
  controllers: [quest, dualshock4]
  quest:
    button_mapping:
      play: 4
      stop: 5
  dualshock4:
    button_mapping:
      play: 7
      stop: 6
```

この設定により，複数コントローラーから同一ノードのplay/stop制御が可能です．

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
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) — ロボットミドルウェア

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
