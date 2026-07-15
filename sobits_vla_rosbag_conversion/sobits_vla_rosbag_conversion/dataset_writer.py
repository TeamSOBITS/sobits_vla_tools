# Copyright (c) 2026, Team SOBITS
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Dataset writer module for creating, populating and finalising LeRobot datasets."""

from pathlib import Path
import shutil
import subprocess

import numpy as np
import pandas as pd
from sobits_vla_common.lerobot_adapter import HF_LEROBOT_HOME, LEROBOT_VERSION, LeRobotDataset
import yaml


def _sobits_vla_tools_rev() -> str:
    """Return `git describe --always --dirty` for this checkout, or 'unknown'."""
    try:
        result = subprocess.run(
            ['git', 'describe', '--always', '--dirty'],
            cwd=Path(__file__).resolve().parent,
            capture_output=True,
            text=True,
            timeout=5,
        )
        if result.returncode == 0:
            return result.stdout.strip() or 'unknown'
    except Exception:
        pass
    return 'unknown'


def _make_create_kwargs(
    dataset_name: str,
    fps: int,
    features: dict,
    output_directory: Path | None,
    robot_type: str | None,
    vcodec: str,
) -> dict:
    """
    Build the kwargs dict for `LeRobotDataset.create(...)`.

    Factored out so the seam test suite exercises the exact same
    creation path as production conversion (see test_dataset_roundtrip).
    """
    return {
        'repo_id': dataset_name,
        'fps': fps,
        'features': features,
        'root': output_directory,
        'robot_type': robot_type,
        'video_backend': 'auto',
        'vcodec': vcodec,
        'streaming_encoding': True,
    }


class DatasetWriter:
    """Writer for LeRobot datasets, handling creation, framing, and finalization."""

    def __init__(
        self,
        dataset_name: str,
        fps: int,
        features: dict,
        output_directory: Path | None,
        robot_type: str,
        vcodec: str,
        overwrite: bool,
        robot_info: dict,
        user_info: list | str | None,
        has_subtasks: bool,
        all_subtasks_list: list,
        push_to_hub: bool,
        hub_private: bool,
        logger=None,
    ):
        """Initialize DatasetWriter and create or clean target dataset directory."""
        self.dataset_name = dataset_name
        self.fps = fps
        self.features = features
        self.output_directory = output_directory
        self.robot_type = robot_type
        self.vcodec = vcodec
        self.overwrite = overwrite
        self.robot_info = robot_info
        self.user_info = user_info
        self.has_subtasks = has_subtasks
        self.all_subtasks_list = all_subtasks_list
        self.push_to_hub = push_to_hub
        self.hub_private = hub_private
        self.logger = logger
        self.dataset = None

        self._init_dataset()

    def log_info(self, msg: str):
        """Log info messages using target logger or print."""
        if self.logger:
            self.logger.info(msg)
        else:
            print(f'[INFO] {msg}')

    def log_warn(self, msg: str):
        """Log warning messages using target logger or print."""
        if self.logger:
            self.logger.warn(msg)
        else:
            print(f'[WARN] {msg}')

    def log_error(self, msg: str):
        """Log error messages using target logger or print."""
        if self.logger:
            self.logger.error(msg)
        else:
            print(f'[ERROR] {msg}')

    def _init_dataset(self):
        """Initialize LeRobotDataset instance and handle metadata details."""
        if self.output_directory:
            dataset_root = self.output_directory
        else:
            dataset_root = HF_LEROBOT_HOME / self.dataset_name

        if self.overwrite and dataset_root.exists():
            self.log_warn(
                f'overwrite=true: deleting existing dataset at {dataset_root}'
            )
            shutil.rmtree(dataset_root)

        create_kwargs = _make_create_kwargs(
            dataset_name=self.dataset_name,
            fps=self.fps,
            features=self.features,
            output_directory=self.output_directory,
            robot_type=self.robot_type,
            vcodec=self.vcodec,
        )
        self.dataset = LeRobotDataset.create(**create_kwargs)

        self.dataset.meta.info['robot_info'] = self.robot_info

        # Version provenance: which lerobot + which sobits_vla_tools revision
        # produced this dataset. Helps triage a bad conversion after a
        # lerobot bump.
        provenance = {
            'lerobot_version': '.'.join(str(p) for p in LEROBOT_VERSION),
            'sobits_vla_tools_rev': _sobits_vla_tools_rev(),
        }
        if self.user_info:
            if len(self.user_info) > 1 or not isinstance(self.user_info, list):
                user_info = self.user_info
            else:
                user_info = self.user_info[0]
            if isinstance(user_info, dict):
                user_info = {**user_info, **provenance}
            else:
                user_info = {'user_info': user_info, **provenance}
            self.dataset.meta.info['user_info'] = user_info
        else:
            self.dataset.meta.info['user_info'] = provenance

    def add_frame(self, frame):
        """Add a single frame to the dataset."""
        self.dataset.add_frame(frame)

    def save_episode(self):
        """Save the current episode's buffered frames."""
        self.dataset.save_episode()

    def _persist_subtasks_metadata(self) -> None:
        """Persist subtask mapping to meta/subtasks.parquet for LeRobot reload compatibility."""
        if not self.has_subtasks:
            return

        subtasks_df = pd.DataFrame(
            {'subtask_index': np.arange(len(self.all_subtasks_list), dtype=np.int64)},
            index=pd.Index(self.all_subtasks_list, name='subtask'),
        )

        subtasks_path = Path(self.dataset.root) / 'meta' / 'subtasks.parquet'
        subtasks_path.parent.mkdir(parents=True, exist_ok=True)
        subtasks_df.to_parquet(subtasks_path)
        self.dataset.meta.subtasks = subtasks_df
        self.log_info(f'Subtasks metadata saved to: {subtasks_path}')

    def _verify_subtasks_metadata(self) -> None:
        """Reload dataset and verify that subtasks are persisted and loadable."""
        if not self.has_subtasks:
            return

        reloaded = LeRobotDataset(self.dataset_name, root=Path(self.dataset.root))
        if reloaded.meta.subtasks is None:
            raise RuntimeError(
                'Subtasks metadata was not persisted (meta.subtasks is None after reload).'
            )
        if len(reloaded.meta.subtasks) != len(self.all_subtasks_list):
            raise RuntimeError(
                f'Subtasks metadata size mismatch after reload: '
                f'expected {len(self.all_subtasks_list)}, '
                f'got {len(reloaded.meta.subtasks)}'
            )
        if 'subtask_index' not in reloaded.features:
            raise RuntimeError("Feature 'subtask_index' is missing after reload.")

    def finalize(
        self,
        skipped_bags: list,
        fps_warnings: list,
        episode_stats: list,
        conversion_params: dict,
    ):
        """Finalize the LeRobot dataset and write conversion stats YAML."""
        if not episode_stats:
            self.log_error(
                'No episodes were successfully converted. Dataset is empty.'
            )
            return

        self.dataset.finalize()
        self._persist_subtasks_metadata()
        self._verify_subtasks_metadata()
        self.log_info(f'Dataset saved to: {self.dataset.root}')
        self.log_info('Dataset creation completed!')

        if self.push_to_hub:
            self.log_info(
                f"Pushing dataset to HuggingFace Hub as '{self.dataset_name}'..."
            )
            self.dataset.push_to_hub(private=self.hub_private)
            self.log_info('Push to Hub completed!')

        # Build stats report
        stats_report = {
            'dataset_name': self.dataset_name,
            **conversion_params,
            'total_episodes': len(episode_stats),
            'total_frames': sum(s['frames'] for s in episode_stats),
            'skipped_bags': skipped_bags if skipped_bags else [],
            'fps_warnings': fps_warnings if fps_warnings else [],
            'episodes': episode_stats,
        }

        stats_path = Path(self.dataset.root) / 'conversion_stats.yaml'
        with open(stats_path, 'w') as f:
            yaml.dump(stats_report, f, default_flow_style=False, sort_keys=False)
        self.log_info(f'Conversion stats saved to: {stats_path}')

        self.log_info(
            f'Summary: {len(episode_stats)} episodes, '
            f'{stats_report["total_frames"]} frames, '
            f'{len(skipped_bags)} skipped bags, '
            f'{len(fps_warnings)} fps warnings'
        )
