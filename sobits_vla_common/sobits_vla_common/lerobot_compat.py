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


"""
Monkey patches applied on top of lerobot 0.6.0.

sobits_vla_tools targets lerobot >= 0.6.0 only (0.5.1 support was dropped —
see docs/lerobot_v060_migration_plan.md). Each patch below documents why it
is still needed against the 0.6.0 source; patches that upstream fixed are
removed rather than kept as version-gated no-ops.
"""

from __future__ import annotations

import logging

# Single source of truth for lerobot version gating — see lerobot_adapter.py.
from sobits_vla_common.lerobot_adapter import IS_V06, LEROBOT_VERSION  # noqa: F401


logger = logging.getLogger(__name__)

# Idempotency flags
_bool_quantile_normalization_patched = False
_pi05_action_dim_padding_patched = False
_processor_registry_patched = False
_pi0fast_peft_targets_patched = False
_pi05_from_pretrained_patched = False
_vla_jepa_image_resize_patched = False


def _resize_image_features(batch: dict, image_keys, target: tuple) -> dict:
    """
    Return a copy of ``batch`` with every image feature resized to ``target``.

    Handles both [B, C, H, W] frames and [B, T, C, H, W] video windows.
    Area interpolation matches what VLAJEPAPolicy.predict_action uses for
    its own ``resize_images_to`` handling.
    """
    import torch.nn.functional as F

    h, w = target
    out = dict(batch)
    for key in image_keys:
        t = out.get(key)
        if t is None or t.shape[-2:] == (h, w):
            continue
        if t.ndim == 5:
            b, n = t.shape[:2]
            t = F.interpolate(t.flatten(0, 1), size=(h, w), mode='area')
            t = t.reshape(b, n, *t.shape[1:])
        else:
            t = F.interpolate(t, size=(h, w), mode='area')
        out[key] = t
    return out


def _patch_bool_quantile_normalization() -> None:
    """
    Cast torch.bool tensors/stats to float before quantile normalization.

    `_NormalizationMixin._apply_transform` in lerobot 0.6.0 is byte-identical
    to 0.5.1 in this regard — still no `torch.bool` handling — so this patch
    remains necessary. Upstreaming candidate.
    """
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
    """
    Zero-pad/truncate pi0/pi05/pi0fast action & state projections on load.

    `_fix_pytorch_state_dict_keys` is byte-identical for pi0/pi05/pi0_fast in
    lerobot 0.6.0 — still no upstream action/state-dim padding or truncation
    — so this patch remains necessary. Upstreaming candidate.
    """
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
    """
    Register 'delta_actions_processor' as a loader alias for the native key.

    The native key is 'relative_actions_processor'. lerobot 0.6.0 natively
    registers the relative-action step under
    'relative_actions_processor' (RelativeActionsProcessorStep). Processor
    pipelines serialized under lerobot 0.5.1 used the old key
    'delta_actions_processor' — without this alias, loading such a pipeline
    in 0.6.0 fails registry lookup. This mirrors (in reverse direction) the
    alias we used to need on 0.5.1.
    """
    global _processor_registry_patched
    if _processor_registry_patched:
        return
    try:
        from sobits_vla_common.lerobot_adapter import ProcessorStepRegistry

        # Public register()/get() API: get() KeyErrors if nothing to alias from,
        # register() ValueErrors if the alias already exists — skip quietly either way.
        try:
            step_cls = ProcessorStepRegistry.get('relative_actions_processor')
            # register() also overwrites step_cls._registry_name (used when SERIALIZING) —
            # restore the native name so we keep writing stock-0.6.0-loadable pipelines.
            native_name = getattr(step_cls, '_registry_name', 'relative_actions_processor')
            ProcessorStepRegistry.register('delta_actions_processor')(step_cls)
            step_cls._registry_name = native_name
            logger.info(
                "Registered alias 'delta_actions_processor' "
                "-> 'relative_actions_processor' via ProcessorStepRegistry.register()."
            )
        except KeyError:
            logger.debug("'relative_actions_processor' not registered — nothing to alias.")
        except ValueError:
            logger.debug(
                "'delta_actions_processor' already registered — alias not needed."
            )
    except Exception as e:
        logger.warning(f'Could not register delta_actions_processor alias: {e}')
    finally:
        _processor_registry_patched = True


def _patch_pi0fast_peft_targets() -> None:
    """
    Supply default LoRA target modules for PI0FastPolicy.

    `PI0FastPolicy` in lerobot 0.6.0 still has no `_get_default_peft_targets`
    override (the base `PreTrainedPolicy` implementation returns `None`), so
    pi0fast LoRA still needs this patch. Upstreaming candidate. The
    qualname check below makes this idempotent-safe even if upstream adds
    the override in a future release: the patch becomes a no-op instead of
    shadowing a real implementation.
    """
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
    """
    Load PI05 on CPU (skeleton) then dtype-cast the safetensors state dict.

    `PreTrainedPolicy.from_pretrained` in lerobot 0.6.0 is unchanged: it does
    a full-size CPU init with no meta-device construction and no `torch_dtype`
    handling, so this patch is still needed for a memory-efficient load.

    `transformers.utils.cached_file(path_or_repo_id, filename, **kwargs)` is
    re-verified against transformers 5.5.4: the kwargs passed below
    (cache_dir, force_download, resume_download, proxies, token, revision,
    local_files_only) are all still accepted — `resume_download` is silently
    absorbed as a deprecated kwarg (transformers.utils.hub.cached_files),
    the rest are named parameters — no signature drift.
    """
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
    """
    No-op placeholder kept for call-site stability.

    Previously applied `_patch_uint8_quantile_stats` (uint8 overflow
    prevention for RunningQuantileStats.update). lerobot 0.6.0 fixed this
    upstream: `update()` now promotes via
    `np.result_type(batch.dtype, np.float32)` (upstream fix #3697), so the
    patch was removed rather than kept as a dead version gate. Conversion
    call sites still call this function so they don't need a version check.
    """


def _patch_vla_jepa_image_resize() -> None:
    """
    Apply ``resize_images_to`` in VLAJEPAPolicy's training/inference input path.

    Upstream inconsistency (lerobot 0.6.0): ``config.resize_images_to`` is
    honored in ``predict_action`` but NOT in ``_prepare_model_inputs``, whose
    world-model video assembly does ``torch.stack`` over the per-camera
    tensors — multi-camera datasets with heterogeneous resolutions (SOBIT
    HOME: head 480x640, hand 1200x1920) crash with "stack expects each
    tensor to be equal size". Resize every image feature to
    ``resize_images_to`` before the original method runs. Upstreaming
    candidate.
    """
    global _vla_jepa_image_resize_patched
    if _vla_jepa_image_resize_patched:
        return
    try:
        from lerobot.policies.vla_jepa.modeling_vla_jepa import VLAJEPAPolicy

        _orig_prepare = VLAJEPAPolicy._prepare_model_inputs

        def _patched_prepare(self, batch, training):
            target = getattr(self.config, 'resize_images_to', None)
            if target is not None:
                batch = _resize_image_features(
                    batch, list(self.config.image_features.keys()), tuple(target)
                )
            return _orig_prepare(self, batch, training)

        VLAJEPAPolicy._prepare_model_inputs = _patched_prepare
        logger.info(
            'Patched VLAJEPAPolicy._prepare_model_inputs to apply '
            'resize_images_to (heterogeneous camera resolutions).'
        )
    except ImportError:
        pass
    except Exception as e:
        logger.debug(f'Could not patch VLAJEPAPolicy._prepare_model_inputs: {e}')
    finally:
        _vla_jepa_image_resize_patched = True


def apply_training_patches() -> None:
    _patch_bool_quantile_normalization()
    _patch_pi05_action_dim_padding()
    _patch_processor_registry()
    _patch_pi0fast_peft_targets()
    _patch_vla_jepa_image_resize()


def apply_deploy_patches() -> None:
    _patch_pi05_action_dim_padding()
    _patch_processor_registry()
    _patch_pi05_from_pretrained()
    _patch_vla_jepa_image_resize()
