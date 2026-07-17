from __future__ import annotations

import logging

import numpy as np

# Single source of truth for lerobot version gating — see lerobot_adapter.py.
from sobits_vla_common.lerobot_adapter import IS_V06, LEROBOT_VERSION  # noqa: F401


logger = logging.getLogger(__name__)

# Idempotency flags
_uint8_quantile_stats_patched = False
_bool_quantile_normalization_patched = False
_pi05_action_dim_padding_patched = False
_processor_registry_patched = False
_pi0fast_peft_targets_patched = False
_pi05_from_pretrained_patched = False


def _patch_uint8_quantile_stats() -> None:
    global _uint8_quantile_stats_patched
    if _uint8_quantile_stats_patched:
        return
    try:
        from lerobot.datasets.compute_stats import RunningQuantileStats

        _orig_update = RunningQuantileStats.update

        def _patched_update(self, batch):
            if np.issubdtype(batch.dtype, np.integer):
                batch = batch.astype(np.float64)
            return _orig_update(self, batch)

        RunningQuantileStats.update = _patched_update
        logger.info('Patched RunningQuantileStats.update for uint8 overflow prevention.')
    except Exception as e:
        logger.debug(f'Could not patch RunningQuantileStats: {e}')
    finally:
        _uint8_quantile_stats_patched = True


def _patch_bool_quantile_normalization() -> None:
    global _bool_quantile_normalization_patched
    if _bool_quantile_normalization_patched:
        return
    try:
        import torch
        from lerobot.processor.normalize_processor import _NormalizationMixin

        orig = _NormalizationMixin._apply_transform

        def _patched(self, tensor, key, feature_type, *, inverse=False):
            if tensor.dtype == torch.bool:
                tensor = tensor.float()
                # Also cast cached stats so q99 - q01 doesn't fail
                if key in self._tensor_stats:
                    stats = self._tensor_stats[key]
                    for stat_key in ('q01', 'q99', 'q10', 'q90', 'min', 'max', 'mean', 'std'):
                        if stat_key in stats and stats[stat_key].dtype == torch.bool:
                            stats[stat_key] = stats[stat_key].float()
            return orig(self, tensor, key, feature_type, inverse=inverse)

        _NormalizationMixin._apply_transform = _patched
        logger.info(
            'Patched _NormalizationMixin._apply_transform '
            'for boolean quantile normalization.'
        )
    except Exception as e:
        logger.debug(f'Could not patch _NormalizationMixin: {e}')
    finally:
        _bool_quantile_normalization_patched = True


def _patch_pi05_action_dim_padding() -> None:
    global _pi05_action_dim_padding_patched
    if _pi05_action_dim_padding_patched:
        return
    try:
        import torch

        def make_patched_fix(orig_fix):
            def _patched_fix(self, state_dict, model_config):
                fixed = orig_fix(self, state_dict, model_config)

                # Action dimension remapping
                model_action_dim = self.model.action_in_proj.in_features

                # State dimension remapping (PI0 has state_proj, PI05 does not)
                model_state_dim = None
                if hasattr(self.model, 'state_proj'):
                    model_state_dim = self.model.state_proj.in_features

                for key in list(fixed.keys()):
                    val = fixed[key]

                    # action_in_proj.weight: (width, ckpt_dim) → (width, model_dim)
                    if key.endswith('action_in_proj.weight') and val.ndim == 2:
                        if val.shape[1] < model_action_dim:
                            extra = model_action_dim - val.shape[1]
                            pad = torch.zeros(
                                val.shape[0], extra,
                                dtype=val.dtype, device=val.device
                            )
                            fixed[key] = torch.cat([val, pad], dim=1)
                        elif val.shape[1] > model_action_dim:
                            fixed[key] = val[:, :model_action_dim]

                    # action_out_proj.weight: (ckpt_dim, width) → (model_dim, width)
                    elif key.endswith('action_out_proj.weight') and val.ndim == 2:
                        if val.shape[0] < model_action_dim:
                            extra = model_action_dim - val.shape[0]
                            pad = torch.zeros(
                                extra, val.shape[1],
                                dtype=val.dtype, device=val.device
                            )
                            fixed[key] = torch.cat([val, pad], dim=0)
                        elif val.shape[0] > model_action_dim:
                            fixed[key] = val[:model_action_dim, :]

                    # action_out_proj.bias: (ckpt_dim,) → (model_dim,)
                    elif key.endswith('action_out_proj.bias') and val.ndim == 1:
                        if val.shape[0] < model_action_dim:
                            extra = model_action_dim - val.shape[0]
                            pad = torch.zeros(extra, dtype=val.dtype, device=val.device)
                            fixed[key] = torch.cat([val, pad], dim=0)
                        elif val.shape[0] > model_action_dim:
                            fixed[key] = val[:model_action_dim]

                    # state_proj.weight: (width, ckpt_dim) → (width, model_dim)
                    elif (
                        key.endswith('state_proj.weight') and val.ndim == 2
                        and model_state_dim is not None
                    ):
                        if val.shape[1] < model_state_dim:
                            extra = model_state_dim - val.shape[1]
                            pad = torch.zeros(
                                val.shape[0], extra,
                                dtype=val.dtype, device=val.device
                            )
                            fixed[key] = torch.cat([val, pad], dim=1)
                        elif val.shape[1] > model_state_dim:
                            fixed[key] = val[:, :model_state_dim]

                return fixed
            return _patched_fix

        # Intercept PI05 Policy if available
        try:
            from lerobot.policies.pi05.modeling_pi05 import PI05Policy
            orig_fix_pi05 = PI05Policy._fix_pytorch_state_dict_keys
            PI05Policy._fix_pytorch_state_dict_keys = make_patched_fix(orig_fix_pi05)
            logger.info(
                'Patched PI05Policy._fix_pytorch_state_dict_keys '
                'for action/state dim padding.'
            )
        except ImportError:
            pass

        # Intercept PI0 Policy if available
        try:
            from lerobot.policies.pi0.modeling_pi0 import PI0Policy
            orig_fix_pi = PI0Policy._fix_pytorch_state_dict_keys
            PI0Policy._fix_pytorch_state_dict_keys = make_patched_fix(orig_fix_pi)
            logger.info(
                'Patched PI0Policy._fix_pytorch_state_dict_keys '
                'for action/state dim padding.'
            )
        except ImportError:
            pass

        # Intercept PI0Fast Policy if available
        try:
            from lerobot.policies.pi0_fast.modeling_pi0_fast import PI0FastPolicy
            orig_fix_pi_fast = PI0FastPolicy._fix_pytorch_state_dict_keys
            PI0FastPolicy._fix_pytorch_state_dict_keys = make_patched_fix(orig_fix_pi_fast)
            logger.info(
                'Patched PI0FastPolicy._fix_pytorch_state_dict_keys '
                'for action/state dim padding.'
            )
        except ImportError:
            pass
    except Exception as e:
        logger.debug(f'Could not patch policies for action dim padding: {e}')
    finally:
        _pi05_action_dim_padding_patched = True


def _patch_processor_registry() -> None:
    global _processor_registry_patched
    if _processor_registry_patched:
        return
    try:
        from sobits_vla_common.lerobot_adapter import ProcessorStepRegistry

        # Preferred: public register()/get() API. get() raises KeyError if
        # 'delta_actions_processor' isn't registered (nothing to alias);
        # register() raises ValueError if 'relative_actions_processor' is
        # already registered natively (0.6.0) — either way, skip quietly.
        try:
            step_cls = ProcessorStepRegistry.get('delta_actions_processor')
            # register() also stamps step_cls._registry_name with the new
            # name, which is the key used when SERIALIZING pipelines — keep
            # the native name so repos we push stay loadable by stock
            # lerobot 0.5.1 (the alias is for loading only).
            native_name = getattr(step_cls, '_registry_name', 'delta_actions_processor')
            ProcessorStepRegistry.register('relative_actions_processor')(step_cls)
            step_cls._registry_name = native_name
            logger.info(
                "Registered alias 'relative_actions_processor' "
                "-> 'delta_actions_processor' via ProcessorStepRegistry.register()."
            )
        except KeyError:
            logger.debug("'delta_actions_processor' not registered — nothing to alias.")
        except ValueError:
            logger.debug(
                "'relative_actions_processor' already registered — alias not needed."
            )
    except Exception as e:
        logger.warning(f'Could not register relative_actions_processor alias: {e}')
    finally:
        _processor_registry_patched = True


def _patch_pi0fast_peft_targets() -> None:
    global _pi0fast_peft_targets_patched
    if _pi0fast_peft_targets_patched:
        return
    try:
        from lerobot.policies.pi0_fast.modeling_pi0_fast import PI0FastPolicy
        qualname = getattr(PI0FastPolicy._get_default_peft_targets, '__qualname__', '')
        if not qualname.startswith('PI0FastPolicy'):
            def _targets(self) -> dict:
                return {
                    'target_modules': r'(.*\.language_model\..*\.self_attn\.(q|v)_proj)',
                    'modules_to_save': [],
                }
            PI0FastPolicy._get_default_peft_targets = _targets
            logger.info('Patched PI0FastPolicy._get_default_peft_targets (LM q/v projections).')
    except ImportError:
        pass
    finally:
        _pi0fast_peft_targets_patched = True


def _patch_pi05_from_pretrained() -> None:
    global _pi05_from_pretrained_patched
    if _pi05_from_pretrained_patched:
        return
    try:
        import gc
        import torch as _torch
        from lerobot.policies.pi05.modeling_pi05 import PI05Policy
        from safetensors.torch import load_file as _sf_load_file
        from transformers.utils import cached_file as _cached_file

        @classmethod
        def _patched_from_pretrained(
            cls,
            pretrained_name_or_path,
            *,
            config=None,
            strict: bool = True,
            **kwargs,
        ):
            from lerobot.configs.policies import PreTrainedConfig

            torch_dtype = kwargs.get('torch_dtype', None)

            # Build config if not provided
            if config is None:
                config = PreTrainedConfig.from_pretrained(
                    pretrained_name_or_path=pretrained_name_or_path, **kwargs
                )

            # Construct skeleton on CPU
            target_device = getattr(config, 'device', 'cpu') or 'cpu'
            config.device = 'cpu'
            model = cls(config, **kwargs)
            config.device = target_device

            if torch_dtype is not None:
                model.to(dtype=torch_dtype)

            # Safetensors load
            try:
                resolved_file = _cached_file(
                    pretrained_name_or_path,
                    'model.safetensors',
                    cache_dir=kwargs.get('cache_dir'),
                    force_download=kwargs.get('force_download', False),
                    resume_download=kwargs.get('resume_download'),
                    proxies=kwargs.get('proxies'),
                    token=kwargs.get('token'),
                    revision=kwargs.get('revision'),
                    local_files_only=kwargs.get('local_files_only', False),
                )
                state_dict = _sf_load_file(resolved_file)
                if torch_dtype is not None:
                    state_dict = {k: v.to(dtype=torch_dtype) for k, v in state_dict.items()}
            except Exception as exc:
                logger.warning('PI05 patch: could not load state dict: %s', exc)
                return model

            # Key remapping
            state_dict = model._fix_pytorch_state_dict_keys(state_dict, model.config)
            state_dict = {
                (k if k.startswith('model.') else f'model.{k}'): v
                for k, v in state_dict.items()
            }
            model.load_state_dict(state_dict, strict=strict)
            del state_dict
            gc.collect()

            # Move model to target device
            if target_device and target_device != 'cpu':
                model.model.to(target_device)
                gc.collect()
                if _torch.cuda.is_available():
                    _torch.cuda.empty_cache()

            return model

        PI05Policy.from_pretrained = _patched_from_pretrained
        logger.info(
            'Patched PI05Policy.from_pretrained for efficient CPU/VRAM dtype load.'
        )
    except Exception as e:
        logger.debug(f'Could not patch PI05Policy.from_pretrained: {e}')
    finally:
        _pi05_from_pretrained_patched = True


def apply_conversion_patches() -> None:
    _patch_uint8_quantile_stats()


def apply_training_patches() -> None:
    _patch_bool_quantile_normalization()
    _patch_pi05_action_dim_padding()
    _patch_processor_registry()
    _patch_pi0fast_peft_targets()


def apply_deploy_patches() -> None:
    _patch_pi05_action_dim_padding()
    _patch_processor_registry()
    _patch_pi05_from_pretrained()
