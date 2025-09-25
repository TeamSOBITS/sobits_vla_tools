<a name="readme-top"></a>

[JA](README.md) | [EN](README_en.md)

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![License][license-shield]][license-url]

# SOBITS VLA Tools

<!-- 目次 -->
<details>
    <summary>目次</summary>
    <ol>
        <li><a href="#概要">概要</a></li>
        <li>
            <a href="#環境構築">環境構築</a>
            <ul>
                <li><a href="#環境条件">環境条件</a></li>
                <li><a href="#インストール方法">インストール方法</a></li>
            </ul>
        </li>
        <li><a href="#実行操作方法">実行・操作方法</a></li>
        <li><a href="#参考文献">参考文献</a></li>
    </ol>
</details>



> [!WARNING]
> 本リポジトリは現時点で「データ収集」のみに対応しています．今後，収集されたRosbagをLeRobotデータセット型に変換できるパッケージも提供する予定です．

<!-- リポジトリの概要 -->
## 概要

SOBITS VLA ToolsはSOBITS自作ロボットをVLAモデルで制御するため，ロボットデータの収集から推論の行動をROS上で通信できるように必要なリソースが用されているリポジトリです．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- セットアップ -->
## セットアップ

ここで，本リポジトリのセットアップ方法について説明します．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


### 環境条件

まず，以下の環境を整えてから，次のインストール段階に進んでください．

| System | Version |
| ------ | ------- |
| Ubuntu | 22.04 (Jammy Jellyfish) |
| ROS    | Humble Hawksbill        |
| Python | ≥3.10                  |

> [!NOTE]
> `Ubuntu`や`ROS`のインストール方法に関しては，[SOBITS Manual](https://github.com/TeamSOBITS/sobits_manual#%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AB%E3%81%A4%E3%81%84%E3%81%A6)に参照してください．

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
3. 本リポジトリに必要な依存パッケージをインストールします．
    ```sh
    $ cd sobits_vla_tools
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


## データ収集セットアップ

ロボットからデータ収集を開始するには，まず [record_settings.yaml](./sobits_vla_rosbag_collection/config/record_settings.yaml) を設定する必要があります．

1. [record_settings.yaml](./sobits_vla_rosbag_collection/config/record_settings.yaml) を同じフォルダ内でコピーし，`record_settings_{robot_name}.yaml` という名前で保存してください．

2. 新しいyamlファイルを作成したら，メタデータの更新を始めましょう．ここでは，**ロボットの説明情報（必須）** と **ユーザー情報（任意）** を記入します．ロボットの種類を把握し，データセットに関してユーザーに連絡できるようにします．
```yaml
# ロボット情報
robot_info:
    # ロボット名
    name: "robot_name"
    # ロボットのバージョン
    version: "1.0.0"
    # ロボットの説明
    morphology:
    # ロボットのタイプ（例："mobile_manipulator", "humanoid", "quadruped" など）
    type: "mobile_manipulator"
    parts:
        - head
        - manipulator
        - end_effector
        - mobile_base
        # - legs
    head:
        joint_names:
        - "head_pan"
        - "head_tilt"
    manipulator:
        joint_names:
        - "arm_joint_1"
        - "arm_joint_2"
        - "arm_joint_3"
        - "arm_joint_4"
        - "arm_joint_5"
        - "arm_joint_6"
    end_effector:
        joint_names:
        - "gripper"
    mobile_base:
        joint_names:
        - "base_wheel_left"
        - "base_wheel_right"
    # legs:
    #   joint_names:
    #     - ""
    # ロボットに搭載されているセンサーの一覧
    sensors:
    types:
        - rgbd # カラー・深度カメラ
        - lidar # ライダーセンサー
        - imu # 慣性計測装置
        - camera # 通常カメラ
        - depth_camera # カラーなし深度カメラ
        - fish_eye # フィッシュアイカメラ
    rgbd:
        names: [""]
        models: [""]
    lidar:
        names: [""]
        models: [""]
    imu:
        names: [""]
        models: [""]
    camera:
        names: [""]
        models: [""]
    depth_camera:
        names: [""]
        models: [""]
    fish_eye:
        names: [""]
        models: [""]

# 連絡用ユーザー情報
user_info:
    # ユーザー名
    name: ""
    # ユーザーの所在地
    location: ""
    # ユーザーの連絡先メールアドレス
    email: ""
```

3. 次に，どのデータをRosbagとして収集し，どのように圧縮・変換するかを決めます．
```yaml
# ROS bag記録設定
rosbag_config:
    # 記録したbagファイルを保存するディレクトリのパス
    record_directory: "/path/to/recorded_bags"
    # 記録するトピックのリスト（カメラ，joint_states，tf，センサーなど）
    topics_to_record:
    - "/camera/image_raw"
    - "/camera/camera_info"
    # 記録するサービスのリスト（TODO: ROS 2 Jazzyまで未対応）
    services_to_record:
    - "/camera/get_image"
    # 記録するアクションのリスト（TODO: ROS 2 Kiltedまで未対応）
    actions_to_record:
    - "/robot/move_base"
    # 記録するbagファイルのフォーマット（例："mcap", "sqlite3"）
    conversion_format: "mcap" 
    # 圧縮モードの指定（"none", "file", "message"）
    compression_mode: "none"
    # 圧縮を有効にする場合の圧縮フォーマット（"zstd" のみ対応）
    compression_format: "zstd"
```

4. （非推奨）コントローラの設定を変更できますが，テレオペ設定と重複する可能性があるため，変更は推奨しません．
```yaml
# ゲームパッドのコントロール設定
gamepad_config:
    # ゲームパッドの種類（例："dualshock4", "keyboard"）
    name: "dualshock4"
    # ゲームパッドボタンのアクション割り当て
    dualshock4:
        button_mapping:
            record: 7 # 上
            save  : 6 # 左
            delete: 6 # 右
    keyboard:
        button_mapping:
            record: "r"
            save  : "s"
            delete: "d"
```

>![WARNING]
> 現時点ではデータ収集に対応しているコントローラは `dualshock4` のみです．今後，他のコントローラにも対応予定です．

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


## 起動と使い方

収集したいデータに合わせてyamlファイルを設定したら，プログラムを起動する準備が整います．

1. コントローラ（ここでは `DualShock4`）をUSBまたはBluetoothで接続します．

2. ロボットを起動します．

3. [rosbag_collection.launch.py](./sobits_vla_rosbag_collection/launch/rosbag_collection.launch.py) ファイル内のロボット名（yamlファイルで設定した名前）を更新します．

```python
def generate_launch_description():
    robot_name = "sobit_light" # ここを目的のロボット名に変更
```

4. 変更を保存し，以下のコマンドで起動します．
```bash
ros2 launch sobits_vla_rosbag_collection rosbag_collection.launch.py
```

5. 最後に，タスク名を更新して記録を開始します．
```bash
ros2 service call /{robot_name}/vla_task_update sobits_interfaces/srv/VlaUpdateTask "label: '{タスク名}'" 
```

6. タスクラベルを更新したら，記録を開始できます．
- **上ボタン**: 記録開始
- **左ボタン**: rosbag保存
- **右ボタン**: rosbag削除

7. データ収集をお楽しみください！

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>


<!-- 参考文献 -->
## 参考文献

* [Rosbag2](https://github.com/ros2/rosbag2)

<p align="right">(<a href="#readme-top">上に戻る</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
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
