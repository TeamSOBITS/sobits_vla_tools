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

import gc
from importlib import import_module
import json
from pathlib import Path
from typing import Any, Dict, List, Optional

import torch

# LeRobot imports (via the single seam — see lerobot_adapter.py)
try:
    from sobits_vla_common.lerobot_adapter import make_pre_post_processors
    _LEROBOT_AVAILABLE = True
except ImportError:
    _LEROBOT_AVAILABLE = False

try:
    from sobits_vla_common.lerobot_adapter import RTCAttentionSchedule, RTCConfig
    _RTC_AVAILABLE = True
except ImportError:
    RTCConfig = None
    RTCAttentionSchedule = None
    _RTC_AVAILABLE = False


def _registry_flag(policy_class_path: str, index: int, default: bool) -> bool:
    try:
        from sobits_vla_common.policy_registry import get_entry
        entry = get_entry(policy_class_path)
    except Exception:
        return default
    if index == 2:
        return entry.has_device_field
    elif index == 3:
        return entry.supports_rtc
    elif index == 4:
        return entry.cast_bf16
    return default


def _state_dim_from_preprocessor(preprocessor) -> Optional[int]:
    """
    Infer expected_state_dim by introspecting the built preprocessor pipeline.

    Replaces the old `policy_preprocessor_step_*_normalizer_processor.safetensors`
    filename convention + hub file listing with a read of the normalizer
    step's own stats object — the same `_tensor_stats` dict the bool-
    normalization compat patch already touches, so this is one private-API
    coupling instead of two.
    """
    # Per-dimension stats only — the stats dict also holds scalars like
    # 'count' (shape (1,)), and grabbing an arbitrary entry once returned
    # expected_state_dim=1, truncating the 19-dim state to garbage.
    _PER_DIM_STATS = ('mean', 'std', 'q01', 'q99', 'q10', 'q90', 'min', 'max')
    for step in getattr(preprocessor, 'steps', []):
        stats = getattr(step, '_tensor_stats', None) or {}
        state_stats = stats.get('observation.state')
        if state_stats:
            for key in _PER_DIM_STATS:
                stat = state_stats.get(key)
                if stat is not None:
                    return int(stat.shape[-1])
    return None


class PolicyLoader:
    def __init__(
        self,
        model_repo_id: str,
        policy_class_path: str,
        model_device: str,
        model_use_amp: bool,
        model_dataset_repo_id: str,
        rtc_enabled: bool,
        rtc_execution_horizon: int,
        rtc_max_guidance_weight: float,
        rtc_prefix_attention_schedule: str,
        rtc_inference_delay: int,
        rtc_debug: bool,
        control_hz: float,
        logger=None,
    ):
        self.model_repo_id = model_repo_id
        self.policy_class_path = policy_class_path
        self.model_device = model_device
        self.model_use_amp = model_use_amp
        self.model_dataset_repo_id = model_dataset_repo_id
        self.rtc_enabled = rtc_enabled
        self.rtc_execution_horizon = rtc_execution_horizon
        self.rtc_max_guidance_weight = rtc_max_guidance_weight
        self.rtc_prefix_attention_schedule = rtc_prefix_attention_schedule
        self.rtc_inference_delay = rtc_inference_delay
        self.rtc_debug = rtc_debug
        self.control_hz = control_hz
        self.logger = logger

        # Deploy node publishes via x.vel/y.vel/theta.vel. Map between the two conventions.
        self._BASE_KEY_ALIASES: Dict[str, str] = {
            'base_x': 'x.vel',
            'base_y': 'y.vel',
            'base_z': 'z.vel',
            'base_theta': 'theta.vel',
        }

    def log_info(self, msg: str):
        if self.logger:
            self.logger.info(msg)
        else:
            print(f'[INFO] {msg}')

    def log_warn(self, msg: str):
        if self.logger:
            self.logger.warn(msg)
        else:
            print(f'[WARN] {msg}')

    def log_error(self, msg: str):
        if self.logger:
            self.logger.error(msg)
        else:
            print(f'[ERROR] {msg}')

    def _build_rtc_config(self) -> Optional[Any]:
        """Build RTCConfig if RTC enabled and policy supports it."""
        if not self.rtc_enabled or not _RTC_AVAILABLE:
            return None
        if not _registry_flag(self.policy_class_path, 3, default=True):
            self.log_info(
                'Policy {!r} does not support RTC (no rtc_config field) — '
                'disabling RTC; chunked execution will be used.'.format(
                    self.policy_class_path
                )
            )
            self.rtc_enabled = False
            return None
        try:
            schedule = RTCAttentionSchedule[self.rtc_prefix_attention_schedule]
            return RTCConfig(
                enabled=True,
                execution_horizon=self.rtc_execution_horizon,
                max_guidance_weight=self.rtc_max_guidance_weight,
                prefix_attention_schedule=schedule,
                debug=self.rtc_debug,
            )
        except Exception as exc:
            self.log_warn(
                'RTCConfig build failed: {}. RTC disabled.'.format(exc)
            )
            self.rtc_enabled = False
            return None

    def _build_policy_config(self, rtc_cfg: Optional[Any]) -> Optional[Any]:
        """Build typed policy config using registry, injecting RTCConfig if supported."""
        try:
            from sobits_vla_common.policy_registry import get_entry

            entry = get_entry(self.policy_class_path)
        except ValueError:
            self.log_info(
                'Policy {!r} not in registry — loading with pretrained defaults.'.format(
                    self.policy_class_path
                )
            )
            return None

        try:
            config_mod = import_module(entry.config_module)
            config_cls = getattr(config_mod, entry.config_class)
            kwargs: Dict[str, Any] = {}
            if rtc_cfg is not None and entry.supports_rtc:
                kwargs['rtc_config'] = rtc_cfg
            if entry.has_device_field and self.model_device:
                kwargs['device'] = self.model_device
            cfg = config_cls(**kwargs)
            self.log_info(
                'Built {} with RTC={}, device={}.'.format(
                    entry.config_class, rtc_cfg is not None, self.model_device
                )
            )
            return cfg
        except Exception as exc:
            self.log_warn(
                'Could not build {}: {}. Loading with pretrained defaults.'.format(
                    entry.config_class, exc
                )
            )
            self.rtc_enabled = False
            return None

    def _build_cfg_from_repo_json(self, policy_cls) -> Optional[Any]:
        """Build a typed policy config from the model repo's config.json."""
        try:
            from dataclasses import fields as _dc_fields

            from sobits_vla_common.lerobot_adapter import FeatureType as FT
            from sobits_vla_common.lerobot_adapter import PolicyFeature

            cfg_path = self._fetch_model_file(self.model_repo_id, 'config.json')
            with open(cfg_path) as fh:
                d = json.load(fh)
            config_cls = policy_cls.config_class
            valid = {f.name for f in _dc_fields(config_cls)}
            kwargs: Dict[str, Any] = {k: v for k, v in d.items() if k in valid}
            for feat_key in ('input_features', 'output_features'):
                if d.get(feat_key):
                    kwargs[feat_key] = {
                        k: PolicyFeature(
                            type=FT[v['type']], shape=tuple(v['shape'])
                        )
                        for k, v in d[feat_key].items()
                    }
            if isinstance(kwargs.get('image_resolution'), list):
                kwargs['image_resolution'] = tuple(kwargs['image_resolution'])

            kwargs.pop('rtc_config', None)
            kwargs.pop('normalization_mapping', None)

            for _flag in ('compile_model', 'gradient_checkpointing'):
                if _flag in valid:
                    kwargs[_flag] = False
            kwargs['device'] = 'cpu'
            cfg = config_cls(**kwargs)
            self.log_info(
                'Built {} from repo config.json. Image features: {}'.format(
                    config_cls.__name__,
                    list(getattr(cfg, 'image_features', {}) or {}),
                )
            )
            return cfg
        except Exception as exc:
            self.log_warn(
                'Could not build config from repo config.json: {}'.format(exc)
            )
            return None

    def _build_pi_adapter_config(self, policy_cls, fallback_cfg) -> Optional[Any]:
        """
        Rebuild a pi-family policy config from the adapter repo's config.json.

        Pi configs (pi0/pi05/pi0_fast) need explicit variant/dim fields; the
        generic repo-json rebuild is used for every other policy family.
        """
        try:
            from sobits_vla_common.lerobot_adapter import FeatureType as FT
            from sobits_vla_common.lerobot_adapter import PolicyFeature

            adapter_policy_json_path = self._fetch_model_file(
                self.model_repo_id, 'config.json'
            )
            with open(adapter_policy_json_path) as fh:
                adapter_policy_dict = json.load(fh)
            in_feats = {
                k: PolicyFeature(type=FT[v['type']], shape=tuple(v['shape']))
                for k, v in adapter_policy_dict.get('input_features', {}).items()
            }
            out_feats = {
                k: PolicyFeature(type=FT[v['type']], shape=tuple(v['shape']))
                for k, v in adapter_policy_dict.get('output_features', {}).items()
            }
            _img_res_raw = adapter_policy_dict.get('image_resolution', [224, 224])
            _img_res = (
                tuple(_img_res_raw)
                if not isinstance(_img_res_raw, tuple)
                else _img_res_raw
            )
            adapter_policy_cfg = policy_cls.config_class(
                input_features=in_feats,
                output_features=out_feats,
                device='cpu',
                chunk_size=adapter_policy_dict.get('chunk_size', 50),
                n_action_steps=adapter_policy_dict.get('n_action_steps', 50),
                paligemma_variant=adapter_policy_dict.get(
                    'paligemma_variant', 'gemma_2b'
                ),
                action_expert_variant=adapter_policy_dict.get(
                    'action_expert_variant', 'gemma_300m'
                ),
                max_action_dim=adapter_policy_dict.get('max_action_dim', 32),
                max_state_dim=adapter_policy_dict.get('max_state_dim', 32),
                image_resolution=_img_res,
                dtype=adapter_policy_dict.get('dtype', 'bfloat16'),
            )
            for _key in (
                'action_feature_names',
                'use_relative_actions',
                'relative_exclude_joints',
            ):
                if _key in adapter_policy_dict:
                    setattr(adapter_policy_cfg, _key, adapter_policy_dict[_key])
            self.log_info(
                'Built adapter policy config. Image features: {}'.format(
                    list(getattr(adapter_policy_cfg, 'image_features', {}).keys())
                )
            )
            return adapter_policy_cfg
        except Exception as exc:
            self.log_warn(
                'Could not build adapter policy config ({}). Using ROS-built config.'.format(
                    exc
                )
            )
            return fallback_cfg

    def _repo_has_serialized_processors(self) -> bool:
        """Check whether the model repo ships a serialized postprocessor pipeline."""
        try:
            if Path(self.model_repo_id).is_dir():
                files = [p.name for p in Path(self.model_repo_id).iterdir()]
            else:
                from huggingface_hub import list_repo_files

                files = list_repo_files(self.model_repo_id)
            return 'policy_postprocessor.json' in files
        except Exception:
            return False

    @staticmethod
    def _fetch_model_file(repo_id: str, filename: str) -> str:
        local = Path(repo_id) / filename
        if local.is_file():
            return str(local)
        from huggingface_hub import hf_hub_download

        return hf_hub_download(repo_id, filename)

    def _is_peft_adapter_repo(self, repo_id: str) -> bool:
        try:
            from huggingface_hub import file_exists

            return file_exists(repo_id, 'adapter_config.json')
        except Exception:
            pass
        try:
            return (Path(repo_id) / 'adapter_config.json').exists()
        except Exception:
            return False

    def load_policy(
        self, joint_features: List[str], mobile_base_features: List[str]
    ) -> Dict[str, Any]:
        module_path, class_name = self.policy_class_path.rsplit('.', 1)
        policy_module = import_module(module_path)
        policy_cls = getattr(policy_module, class_name)

        # Register the policy's custom processor steps (e.g.
        # vla_jepa_clip_actions, molmoact2 steps): they live in a sibling
        # processor_<pkg> module that neither modeling_<pkg> nor lerobot's
        # make_pre_post_processors pretrained-path branch imports — without
        # this, loading a serialized pipeline fails registry lookup and the
        # node silently falls back to NO postprocessor, executing normalized
        # [-1, 1] actions as radians.
        if module_path.startswith('lerobot.policies.'):
            pkg = module_path.split('.')[2]
            try:
                import_module(f'lerobot.policies.{pkg}.processor_{pkg}')
            except ImportError:
                pass

        rtc_cfg = self._build_rtc_config()
        cfg = self._build_policy_config(rtc_cfg)

        load_device = self.model_device
        if cfg is not None and hasattr(cfg, 'device'):
            cfg.device = 'cpu'

        if self._is_peft_adapter_repo(self.model_repo_id):
            adapter_cfg_path = self._fetch_model_file(
                self.model_repo_id, 'adapter_config.json'
            )
            with open(adapter_cfg_path) as fh:
                adapter_meta = json.load(fh)
            base_model_id = adapter_meta.get('base_model_name_or_path', '')
            self.log_info(
                'LoRA adapter detected. Loading base model {!r} ...'.format(
                    base_model_id
                )
            )

            from dataclasses import fields as _cfg_fields
            _is_pi_family = 'paligemma_variant' in {
                f.name for f in _cfg_fields(policy_cls.config_class)
            }
            if not _is_pi_family:
                # Generic policies (vla_jepa, molmoact2, ...): the adapter
                # repo's config.json is a complete serialized policy config —
                # rebuild it generically. It carries reinit_modules, so base
                # weights with mismatched shapes (e.g. our 19-dim projections
                # vs the 7-dim pretrained base) re-initialise and the fully
                # trained modules_to_save from the adapter overwrite them.
                load_cfg = self._build_cfg_from_repo_json(policy_cls) or cfg
                self.log_info(
                    'Non-pi adapter repo: using generic repo-json config '
                    'rebuild for {}.'.format(policy_cls.config_class.__name__)
                )
            else:
                load_cfg = self._build_pi_adapter_config(policy_cls, cfg)

            load_kwargs: Dict[str, Any] = {'strict': False}
            if _registry_flag(self.policy_class_path, 4, default=True):
                load_kwargs['torch_dtype'] = torch.bfloat16
            if load_cfg is not None:
                load_kwargs['config'] = load_cfg

            self.log_info(
                'from_pretrained START: base={!r}  kwargs={}'.format(
                    base_model_id,
                    {
                        k: (
                            v
                            if not hasattr(v, '__class__')
                            else v.__class__.__name__
                        )
                        for k, v in load_kwargs.items()
                    },
                )
            )

            policy = policy_cls.from_pretrained(base_model_id, **load_kwargs)

            self.log_info(
                'Applying LoRA adapter from {!r} ...'.format(self.model_repo_id)
            )
            try:
                from peft import PeftModel

                policy = PeftModel.from_pretrained(policy, self.model_repo_id)
                policy = policy.merge_and_unload()
                self.log_info('LoRA adapter merged.')
            except Exception as exc:
                # The repo IS the adapter — running the bare base model would
                # silently evaluate untrained weights.
                raise RuntimeError(
                    'PEFT adapter merge from {!r} failed: {}. Refusing to '
                    'run the bare base model.'.format(self.model_repo_id, exc)
                ) from exc
        else:
            repo_cfg = self._build_cfg_from_repo_json(policy_cls)
            load_cfg = repo_cfg if repo_cfg is not None else cfg
            load_kwargs: Dict[str, Any] = {'strict': False}
            if _registry_flag(self.policy_class_path, 4, default=True):
                load_kwargs['torch_dtype'] = torch.bfloat16
            if load_cfg is not None:
                load_kwargs['config'] = load_cfg
            policy = policy_cls.from_pretrained(self.model_repo_id, **load_kwargs)

        policy.eval()

        cast_bf16 = _registry_flag(self.policy_class_path, 4, default=True)
        if load_device != 'cpu' and not cast_bf16:
            policy.to(torch.device(load_device))
            self.log_info('Model moved to GPU (float32).')

        if load_device != 'cpu' and cast_bf16:
            gc.collect()
            torch.cuda.empty_cache()
            policy.to(dtype=torch.bfloat16)

            gpu_device = torch.device(load_device)
            self.log_info('Moving bfloat16 model to {}...'.format(load_device))
            policy.to(gpu_device)
            gc.collect()
            torch.cuda.empty_cache()
            self.log_info('Model moved to GPU.')

            _pwe = getattr(getattr(policy, 'model', None), 'paligemma_with_expert', None)
            if _pwe is not None and hasattr(_pwe, 'to_bfloat16_for_selected_params'):
                _pwe.to_bfloat16_for_selected_params('bfloat16')
                self.log_info(
                    'Re-applied selective precision: vision tower + norms float32.'
                )

            _mdl = getattr(policy, 'model', None)
            if _mdl is not None:
                _restored = []
                for _attr in (
                    'action_in_proj',
                    'action_out_proj',
                    'time_mlp_in',
                    'time_mlp_out',
                    'action_time_mlp_in',
                    'action_time_mlp_out',
                    'state_proj',
                ):
                    _sub = getattr(_mdl, _attr, None)
                    if _sub is not None:
                        _sub.to(dtype=torch.float32)
                        _restored.append(_attr)
                if _restored:
                    self.log_info(
                        'Restored float32 model-level projections: {}'.format(
                            _restored
                        )
                    )

        if hasattr(policy, 'reset'):
            policy.reset()

        if self.rtc_enabled and _RTC_AVAILABLE:
            rtc_cfg_for_init = self._build_rtc_config()
            if rtc_cfg_for_init is not None and hasattr(policy, 'config'):
                if getattr(policy.config, 'rtc_config', None) is None:
                    policy.config.rtc_config = rtc_cfg_for_init
            if hasattr(policy, 'init_rtc_processor'):
                policy.init_rtc_processor()
                self.log_info('RTC processor initialized on policy.')
            else:
                self.log_warn(
                    'Policy does not support init_rtc_processor(). RTC disabled.'
                )
                self.rtc_enabled = False

        model_action_feature_names = getattr(
            policy.config, 'action_feature_names', None
        )

        model_relative = False
        if hasattr(policy, 'config') and policy.config is not None:
            model_relative = getattr(policy.config, 'use_relative_actions', False)

        _deploy_exclude = []
        if hasattr(policy, 'config') and policy.config is not None:
            _deploy_exclude = list(
                getattr(policy.config, 'relative_exclude_joints', []) or []
            )
        if model_relative and _deploy_exclude == ['gripper']:
            self.log_warn(
                'use_relative_actions=true but relative_exclude_joints is '
                'the default ["gripper"], which matches no joint in SOBIT HOME '
                '(fingers are hand_left_finger_*). Velocity joints '
                '(base_x, base_y, base_theta) may be incorrectly delta-converted.'
            )

        if model_action_feature_names:
            self.log_info(
                'Model action_feature_names: {}'.format(
                    model_action_feature_names
                )
            )
            yaml_features = joint_features + mobile_base_features
            _alias_rev = {v: k for k, v in self._BASE_KEY_ALIASES.items()}
            model_names_set = set(model_action_feature_names)
            missing = [
                f
                for f in yaml_features
                if f not in model_names_set and _alias_rev.get(f) not in model_names_set
            ]
            wired = set(yaml_features) | {
                _alias_rev.get(f, f) for f in yaml_features
            }
            unknown = [
                f for f in model_action_feature_names if f not in wired
            ]
            if missing:
                raise RuntimeError(
                    'Deploy joint names do not match model action_feature_names — '
                    'zeros would be inserted at wrong positions causing bad actions. '
                    'Missing from model: {}. Update your deploy_config YAML.'.format(
                        missing
                    )
                )
            if unknown:
                self.log_warn(
                    'Model outputs joints not wired to any controller (ignored): '
                    '{}'.format(unknown)
                )

        if hasattr(policy, 'config') and policy.config is not None:
            _policy_cfg = policy.config
            _max_action_dim = getattr(_policy_cfg, 'max_action_dim', None)
            _max_state_dim = getattr(_policy_cfg, 'max_state_dim', None)
            _actual_action_dim = len(joint_features + mobile_base_features)
            _actual_state_dim = _actual_action_dim
            if _max_action_dim is not None and _max_action_dim < _actual_action_dim:
                raise RuntimeError(
                    'max_action_dim={} < actual joint count={} '
                    '— joints would be silently truncated.'.format(
                        _max_action_dim, _actual_action_dim
                    )
                )
            if _max_state_dim is not None and _max_state_dim < _actual_state_dim:
                raise RuntimeError(
                    'max_state_dim={} < actual joint count={} '
                    '— state would be silently truncated.'.format(
                        _max_state_dim, _actual_state_dim
                    )
                )

        preprocessor = None
        postprocessor = None
        if _LEROBOT_AVAILABLE:
            processor_kwargs: Dict[str, Any] = {}
            is_groot = 'groot' in self.policy_class_path.lower()
            if self.model_dataset_repo_id:
                try:
                    from sobits_vla_common.lerobot_adapter import LeRobotDatasetMetadata

                    _ds_meta = LeRobotDatasetMetadata(
                        self.model_dataset_repo_id
                    )
                    processor_kwargs['dataset_stats'] = _ds_meta.stats
                    self.log_info(
                        'Loaded dataset stats from {!r} for processor build.'.format(
                            self.model_dataset_repo_id
                        )
                    )
                except Exception as exc:
                    self.log_warn(
                        'Could not load dataset stats from {!r}: {}'.format(
                            self.model_dataset_repo_id, exc
                        )
                    )
            elif is_groot:
                self.log_warn(
                    'GR00T policy without model.dataset_repo_id — normalization will be BROKEN.'
                )
            try:
                preprocessor, postprocessor = make_pre_post_processors(
                    policy.config, self.model_repo_id, **processor_kwargs
                )
            except Exception as exc:
                # If the repo ships serialized pipelines, running without them
                # executes NORMALIZED [-1, 1] actions as radians — refuse
                # instead of degrading silently (10 wasted eval episodes, and
                # dangerous on real hardware).
                if self._repo_has_serialized_processors():
                    raise RuntimeError(
                        'Repo {!r} ships processor pipelines but they failed '
                        'to build: {}. Refusing to run without '
                        'normalization.'.format(self.model_repo_id, exc)
                    ) from exc
                self.log_warn(
                    'Could not build pre/post processors: {}. '
                    'Direct policy.select_action will be used.'.format(exc)
                )

        if postprocessor is not None:
            try:
                from sobits_vla_common.lerobot_adapter import AbsoluteActionsProcessorStep

                _abs_steps = [
                    s
                    for s in postprocessor.steps
                    if isinstance(s, AbsoluteActionsProcessorStep)
                ]
                _has_abs_step_active = any(
                    getattr(s, 'enabled', True) for s in _abs_steps
                )
                _has_abs_step = bool(_abs_steps)
                self.log_info(
                    'Postprocessor AbsoluteActionsProcessorStep present: {} (active: {})'.format(
                        _has_abs_step, _has_abs_step_active
                    )
                )
                if model_relative and not _has_abs_step_active:
                    self.log_warn(
                        'use_relative_actions=true but postprocessor '
                        'has no active AbsoluteActionsProcessorStep.'
                    )
                elif not model_relative and _has_abs_step_active:
                    self.log_warn(
                        'use_relative_actions=false but postprocessor '
                        'contains an active AbsoluteActionsProcessorStep.'
                    )
            except ImportError:
                pass

        expected_state_dim = None
        _state_dim_source = 'disabled'
        if preprocessor is not None:
            try:
                expected_state_dim = _state_dim_from_preprocessor(preprocessor)
                _state_dim_source = (
                    'preprocessor_tensor_stats' if expected_state_dim is not None
                    else 'preprocessor_has_no_observation_state_stats'
                )
            except Exception as exc:
                _state_dim_source = 'exception ({})'.format(exc)
        else:
            _state_dim_source = 'no_preprocessor_built'

        if expected_state_dim is None and model_action_feature_names is not None:
            expected_state_dim = len(model_action_feature_names)
            _state_dim_source = 'action_feature_names_len (fallback)'

        self.log_info(
            'expected_state_dim={} (source: {})'.format(
                expected_state_dim
                if expected_state_dim is not None
                else 'disabled',
                _state_dim_source,
            )
        )

        return {
            'policy': policy,
            'rtc_enabled': self.rtc_enabled,
            'model_action_feature_names': model_action_feature_names,
            'model_use_relative_actions': model_relative,
            'expected_state_dim': expected_state_dim,
            'preprocessor': preprocessor,
            'postprocessor': postprocessor,
        }
