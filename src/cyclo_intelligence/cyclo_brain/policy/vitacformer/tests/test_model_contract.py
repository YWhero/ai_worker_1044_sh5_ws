#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import hashlib
import json
from pathlib import Path

import pytest
import torch


MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "vitacformer_engine"
    / "model.py"
)
spec = importlib.util.spec_from_file_location("vitacformer_model", MODEL_PATH)
model = importlib.util.module_from_spec(spec)
spec.loader.exec_module(model)


def _train_config():
    return {
        "architecture": "ViTacFormer",
        "source_repo_commit": model._EXPECTED_SOURCE_COMMIT,
        "data_contract": {
            "action_chunk": 100,
            "action_dim": 54,
            "state_dim": 54,
            "state_history": 6,
            "state_stride": 3,
            "tactile_history": 18,
            "tactile_raw_dim": 90,
            "tactile_rep_dim": 180,
            "tactile_taxels_per_hand": 45,
            "image_height": 188,
            "image_width": 336,
            "fps": 30,
            "image_key": "observation.images.rgb.cam_left_head",
            "tactile_order": "left[5,3,3],right[5,3,3]",
            "baseline": (
                "already corrected per episode from first 20 raw 100Hz samples"
            ),
        },
    }


def _h200_config():
    config = _train_config()
    config.update(
        architecture_version="vitacformer_sh5_h200_v1",
        training_recipe_version="sh5_h200_original_tactile_r1",
        model_config={
            "architecture_version": "vitacformer_sh5_h200_v1",
            "source_repo_commit": model._EXPECTED_SOURCE_COMMIT,
            "action_chunk": 200, "action_dim": 54,
            "state_dim": 54, "state_history": 6,
            "tactile_dim": 180, "tactile_history": 18,
            "camera_names": ["observation.images.rgb.cam_left_head"],
            "hidden_dim": 512, "dim_feedforward": 3200,
            "enc_layers": 4, "dec_layers": 7, "nheads": 8,
            "backbone": "resnet18", "action_bound_beta": 1000.0,
        },
    )
    config["data_contract"]["action_chunk"] = 200
    return config


def test_h200_contract_requires_matching_architecture_and_recipe():
    model._validate_contract(_h200_config())
    for key, value in (
        ("architecture_version", "unknown"),
        ("training_recipe_version", "unknown"),
    ):
        bad = _h200_config()
        bad[key] = value
        with pytest.raises(ValueError):
            model._validate_contract(bad)


@pytest.mark.parametrize("chunk_size", [100, 201, 0, -1])
def test_h200_contract_rejects_a_different_horizon(chunk_size):
    config = _h200_config()
    config["data_contract"]["action_chunk"] = chunk_size
    with pytest.raises(ValueError, match="action_chunk"):
        model._validate_contract(config)


def test_h200_contract_rejects_conflicting_model_dimensions():
    config = _h200_config()
    config["model_config"]["action_chunk"] = 100
    with pytest.raises(ValueError, match="model_config.*action_chunk"):
        model._validate_contract(config)


@pytest.mark.parametrize("chunk_size", [100, 200])
def test_core_sizes_checkpoint_embeddings_for_the_horizon(chunk_size):
    with torch.device("meta"):
        core = model._ViTacFormerCore(chunk_size)
    assert core.query_embed.weight.shape == (chunk_size, 512)
    assert core.pos_table.shape == (1, chunk_size + 2, 512)
    assert core.query_embed_tactile.weight.shape == (18, 512)


def _h200_checkpoint_metadata(root):
    config = _h200_config()
    (root / "normalization_stats.pt").write_bytes(b"test normalization")
    stats_hash = hashlib.sha256(b"test normalization").hexdigest()
    (root / "inference_config.json").write_text(json.dumps(
        dict(config, normalization_stats_sha256=stats_hash),
    ))
    checkpoint = {
        "architecture_version": config["architecture_version"],
        "training_recipe_version": config["training_recipe_version"],
        "action_chunk": 200,
        "global_step": 142000,
        "normalization_stats_sha256": stats_hash,
        "inference_config_sha256": hashlib.sha256(
            (root / "inference_config.json").read_bytes(),
        ).hexdigest(),
    }
    return config, checkpoint


def test_h200_checkpoint_requires_original_normalization_and_config(tmp_path):
    config, checkpoint = _h200_checkpoint_metadata(tmp_path)
    model._validate_checkpoint_metadata(config, checkpoint, tmp_path)
    (tmp_path / "normalization_stats.pt").write_bytes(b"other model stats")
    with pytest.raises(ValueError, match="normalization_stats_sha256"):
        model._validate_checkpoint_metadata(config, checkpoint, tmp_path)


@pytest.mark.parametrize("key,value", [
    ("architecture_version", "vitacformer_sh5_v2_1_right_persistence"),
    ("training_recipe_version", "unknown"),
    ("action_chunk", 100),
    ("global_step", 0),
    ("inference_config_sha256", "incorrect"),
])
def test_h200_checkpoint_rejects_mismatched_metadata(tmp_path, key, value):
    config, checkpoint = _h200_checkpoint_metadata(tmp_path)
    checkpoint[key] = value
    with pytest.raises(ValueError, match=key):
        model._validate_checkpoint_metadata(config, checkpoint, tmp_path)


def test_h200_checkpoint_rejects_changed_train_config(tmp_path):
    config, checkpoint = _h200_checkpoint_metadata(tmp_path)
    config["data_contract"]["action_chunk"] = 100
    with pytest.raises(ValueError, match="inference/train config data_contract"):
        model._validate_checkpoint_metadata(config, checkpoint, tmp_path)


def test_contract_accepts_only_the_audited_sh5_layout():
    model._validate_contract(_train_config())

    bad = _train_config()
    bad["data_contract"]["action_dim"] = 53
    with pytest.raises(ValueError, match="action_dim"):
        model._validate_contract(bad)

    bad = _train_config()
    bad["source_repo_commit"] = "unknown"
    with pytest.raises(ValueError, match="source commit"):
        model._validate_contract(bad)


def test_contract_accepts_original_export_without_descriptive_annotations():
    original_export = _train_config()
    del original_export["data_contract"]["tactile_order"]
    del original_export["data_contract"]["baseline"]

    model._validate_contract(original_export)

    contradictory = _train_config()
    contradictory["data_contract"]["tactile_order"] = "right,left"
    with pytest.raises(ValueError, match="tactile_order"):
        model._validate_contract(contradictory)


def test_layout_resolves_run_and_numeric_checkpoint(tmp_path):
    run = tmp_path / "run"
    checkpoint = run / "checkpoints" / "1000"
    checkpoint.mkdir(parents=True)
    (run / "train_config.json").write_text(json.dumps(_train_config()))
    weights = checkpoint / "model.pt"
    weights.touch()

    assert model._resolve_vitacformer_layout(run) == (run, None)
    assert model._resolve_vitacformer_layout(checkpoint) == (run, weights)
    assert model._resolve_vitacformer_layout(weights) == (run, weights)
    assert model._resolve_vitacformer_layout("") is None


def test_stats_require_exact_finite_positive_normalization_vectors():
    stats = {
        "state_mean": torch.zeros(54),
        "state_std": torch.ones(54),
        "action_mean": torch.zeros(54),
        "action_std": torch.ones(54),
        "tactile_history_mean": torch.zeros(180),
        "tactile_history_std": torch.ones(180),
        "tactile_future_mean": torch.zeros(180),
        "tactile_future_std": torch.ones(180),
    }
    model._validate_stats(stats)

    bad = dict(stats, state_mean=torch.zeros(53))
    with pytest.raises(ValueError, match="state_mean.*shape"):
        model._validate_stats(bad)

    bad = dict(stats, action_std=torch.zeros(54))
    with pytest.raises(ValueError, match="action_std.*strictly positive"):
        model._validate_stats(bad)


def test_default_checkpoint_prefers_best_model(tmp_path):
    root = tmp_path / "run"
    best = root / "checkpoints" / "best_model.pt"
    best.parent.mkdir(parents=True)
    best.touch()
    (root / "model.pt").touch()

    assert model._checkpoint_path(root) == best


def test_bounded_decoder_keeps_all_actions_inside_sh5_limits():
    logits = torch.zeros(1, 100, 54)
    # Reproduce the online failure: right-hand joint 3 is global index 36.
    logits[:, 12, 36] = -0.060661
    normalized, bounded = model._bound_action(
        logits,
        torch.zeros(54),
        torch.ones(54),
    )

    lower = model._JOINT_LOWER + 1e-5
    upper = model._JOINT_UPPER - 1e-5
    assert torch.all(bounded >= lower)
    assert torch.all(bounded <= upper)
    assert bounded[0, 12, 36] >= lower[36]
    assert torch.allclose(normalized, bounded)


def test_warm_start_ramps_only_arms_over_sixteen_rows():
    action = torch.zeros(1, 100, 54)
    action[:, :, :14] = 0.2
    action[:, :, 14:] = 0.4
    current = torch.zeros(1, 54)

    ramped = model._apply_warm_start_ramp(action, current)
    safe_current = torch.maximum(
        model._JOINT_LOWER + 1e-5,
        torch.minimum(model._JOINT_UPPER - 1e-5, current),
    )

    assert torch.allclose(ramped[:, 0, :14], safe_current[:, :14])
    assert torch.allclose(ramped[:, 15, :14], action[:, 15, :14])
    assert torch.allclose(ramped[:, :, 14:], action[:, :, 14:])
    assert torch.allclose(ramped[:, 16:], action[:, 16:])


def test_right_tactile_future_uses_exact_persistence_fallback():
    history = torch.zeros(1, 18, 180)
    history[:, -1, :90] = torch.arange(90, dtype=torch.float32)
    persistence, residual_scale = model._build_tactile_persistence(
        history,
        torch.zeros(180),
        torch.ones(180),
    )

    assert persistence.shape == (1, 18, 180)
    assert torch.allclose(persistence[:, :, :90], history[:, -1:, :90])
    assert torch.count_nonzero(persistence[:, :, 90:]) == 0
    assert torch.count_nonzero(residual_scale[45:90]) == 0
    assert torch.count_nonzero(residual_scale[135:180]) == 0
    assert torch.all(residual_scale[:45] == 1)
    assert torch.all(residual_scale[90:135] == 1)


def test_h200_keeps_learned_residuals_for_both_tactile_sides():
    history = torch.zeros(1, 18, 180)
    history[:, -1, :90] = 10.0
    persistence, residual_scale = model._build_tactile_persistence(
        history, torch.ones(180), torch.full((180,), 2.0),
        right_persistence=False,
    )
    assert persistence.shape == (1, 18, 180)
    assert torch.all(persistence[:, :, :90] == 4.5)
    assert torch.all(persistence[:, :, 90:] == -0.5)
    assert torch.all(residual_scale == 1)
