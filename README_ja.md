<a name="readme-top"></a>

[JA](README_ja.md) | [EN](README.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SOBITS VLA Tools

SOBITS VLA Toolsは，SOBITSが開発したロボットをVision-Language-Action (VLA)
モデルで制御するためのフルパイプラインを提供するモノレポです — データ収集から
学習，リアルタイムデプロイまで，すべてROS 2で統合されています。新規コードが
従うべき規約は[CONTRIBUTING.md](CONTRIBUTING.md)を参照してください。

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## パッケージ

| パッケージ | 役割 | README |
| ------- | ------- | ------ |
| `sobits_vla_common` | 共有ハブ: ロボット記述子，パラメータスキーマローダー，lerobot互換層，ゲームパッドクライアント，ワールドリセット | [README](sobits_vla_common/README.md) |
| `sobits_vla_rosbag_collection` | ゲームパッド駆動のrosbag記録（C++） | [README](sobits_vla_rosbag_collection/README.md) |
| `sobits_vla_rosbag_conversion` | rosbag → [LeRobot](https://github.com/huggingface/lerobot)データセット変換 | [README](sobits_vla_rosbag_conversion/README.md) |
| `sobits_vla_training` | lerobot経由でVLAポリシーを学習/ファインチューン | [README](sobits_vla_training/README.md) |
| `sobits_vla_deploy` | リアルタイムVLA推論 + オフライン評価 | [README](sobits_vla_deploy/README.md) |
| `sobits_vla_visualization` | 将来のデバッグ/可視化ノード用の予約領域（空のスケルトン） | [README](sobits_vla_visualization/README.md) |

すべてのパイプライン段階は，1つの**ロボット記述子**
（`sobits_vla_common/robots/<robot_id>.robot.yaml`）からロボットの構造を
読み込みます — 関節グループ，コマンドトピック，センサー，モバイルベースの
唯一の情報源です。新規記述子は以下でスキャフォールドできます:

```bash
ros2 run sobits_vla_common new_robot \
  --robot_id sobit_mini --dof 7 --cameras head,hand_left --mobile_base diff \
  --gen_collection_config
ros2 run sobits_vla_common new_robot --robot_id sobit_mini --validate_only
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## パイプライン概要

```
┌────────────┐   ┌────────────┐   ┌────────────┐   ┌────────────┐   ┌────────────┐
│ Collection │──▶│ Conversion │──▶│  Training  │──▶│   Deploy   │──▶│    Eval    │
│ (C++,       │   │  rosbag →  │   │ ポリシーを │   │  ロボット上│   │  ログの    │
│  ゲームパッド│  │ LeRobot    │   │ ファイン   │   │ でリアル   │   │ オフライン │
│  駆動記録)  │   │ データセット│  │ チューン   │   │ タイム推論 │   │ 分析       │
└────────────┘   └────────────┘   └────────────┘   └────────────┘   └────────────┘
```

1. ゲームパッドでロボットを遠隔操作して**収集**する
   （`sobits_vla_rosbag_collection`）。
2. 記録したrosbagをLeRobotデータセットに**変換**する
   （`sobits_vla_rosbag_conversion`）。
3. データセット上でVLAポリシーを**学習**する（`sobits_vla_training`）。
4. 学習したポリシーを自律制御用に**デプロイ**する
   （`sobits_vla_deploy`）。
5. 得られたエピソードログをオフラインで**評価**する
   （`sobits_vla_deploy`の`vla_eval`）。

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## 名前空間モデル

すべてのノードは自身のトピック/サービスを**プライベート**名で公開します:
`~/<channel>`は無名前空間下では`/<node_name>/<channel>`，名前空間付き
launchでは`/<robot_name>/<node_name>/<channel>`に解決されます。他のノードは
所有ノードの相対名`<owner_node>/<channel>`で参照します（例:
ゲームパッドクライアントの`command_service`のデフォルトは
`sobits_vla_deploy/command`または`vla_rosbag_collection/command`）。これは
同じ名前空間下で並んで解決されます。ロボットI/Oトピック（関節状態，カメラ，
cmd_vel — 記述子由来）は絶対名のままです。名前空間なしの`ros2 run`では
すべて`/`直下に解決されるため，単一ロボットは名前空間なしで動作し，
名前空間付きlaunchでは複数ロボットが同一ROSドメインを共有できます。

## 出力ルート

各パッケージは生成物（rosbag，データセット，チェックポイント，ログ）を
`<pkg_src>/<artifact-dir>/`以下に書き込みます。これはソースツリー，colconの
`--symlink-install`，通常インストールのいずれから実行しても
`sobits_vla_common.output_root.output_root()`が解決します。各
`<artifact-dir>/`は統一された`*\n!.gitignore\n`の無視設定を持ちます —
ディレクトリ自体は追跡され，生成される中身は追跡されません。具体的な出力先は
各パッケージのREADMEを参照してください。

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## はじめに

### 前提条件

| システム | バージョン |
| ------ | ------- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS    | Jazzy Jalisco           |
| Python | ≥3.10                  |

> [!NOTE]
> `Ubuntu`や`ROS`のインストールが必要な場合は，[SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6)を確認してください。

### インストール

```sh
cd ~/colcon_ws/src/
git clone https://github.com/TeamSOBITS/sobits_vla_tools
cd sobits_vla_tools/
bash install.sh
cd ~/colcon_ws
rosdep update
rosdep install --from-paths src -y --ignore-src
colcon build
source install/setup.bash
```

ROS以外のPython依存関係（numpy，pandas，torch，lerobot）はシステムの
インタプリタではなく，パッケージごとの`pixi`環境に存在します — リポジトリ
ルートの`pixi.toml`と，各パッケージREADMEの「How to run」節にある
`enable_gpu:=`/`pixi_env:=`のlaunch引数を参照してください。

### クイックスタート

```sh
# 1. デモンストレーションを記録する（ゲームパッド駆動，実機/シミュレーション）
ros2 launch sobits_vla_rosbag_collection rosbag_collection.launch.py enable_world_reset:=false robot_name:=sobit_home

# 2. 記録したrosbagをLeRobotデータセットに変換する
ros2 launch sobits_vla_rosbag_conversion rosbag_conversion.launch.py robot:=sobit_home

# 3. データセット上でポリシーを学習する
ros2 launch sobits_vla_training sobits_vla_training.launch.py robot:=sobit_home_left_smolvla_fft steps:=30000

# 4. 学習したポリシーをデプロイする
ros2 launch sobits_vla_deploy sobits_vla_deploy.launch.py deploy_config:=deploy_config_sobit_home robot_name:=sobit_home
```

上記の引数はすべて実際に検証済みのlaunch引数です — 各ファイルで
`--show-args`を実行すると全リストが確認できます。

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## 謝辞

- [LeRobot](https://github.com/huggingface/lerobot) — データセット形式と学習フレームワーク
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/) — ロボットミドルウェア

<p align="right">(<a href="#readme-top">back to top</a>)</p>

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
