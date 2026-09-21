"""Pour H100 format compatibility and optional real-package parity tests."""

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys

import numpy as np
import pytest
import torch

from test_model_contract import model, _train_config, _h200_config


FIXTURES = Path(__file__).parent / "fixtures" / "pour_h100"


@pytest.fixture
def pour_run(tmp_path):
    for name in ("train_config.json", "inference_config.json"):
        (tmp_path / name).write_bytes((FIXTURES / name).read_bytes())
    (tmp_path / "checkpoints").mkdir()
    (tmp_path / "checkpoints" / "best_model.pt").touch()
    return tmp_path


def _config(root):
    return json.loads((root / "train_config.json").read_text())


def _metadata(root):
    (root / "normalization_stats.pt").write_bytes(b"normalization fixture")
    stats_hash = hashlib.sha256((root / "normalization_stats.pt").read_bytes()).hexdigest()
    inference = json.loads((root / "inference_config.json").read_text())
    inference["normalization_stats_sha256"] = stats_hash
    (root / "inference_config.json").write_text(json.dumps(inference))
    return {
        "architecture_version": "vitacformer_sh5_pour_h100_v2",
        "training_recipe_version": "task519_folder159_gt75_residual_unpenalized_r1",
        "action_chunk": 100, "global_step": 50000,
        "normalization_stats_sha256": stats_hash,
        "inference_config_sha256": hashlib.sha256(
            (root / "inference_config.json").read_bytes(),
        ).hexdigest(),
    }


def test_pour_resolves_all_browser_selections_and_adapts_without_editing(pour_run):
    weights = pour_run / "checkpoints" / "best_model.pt"
    assert model._resolve_vitacformer_layout(pour_run) == (pour_run, None)
    assert model._resolve_vitacformer_layout(weights.parent) == (pour_run, None)
    assert model._resolve_vitacformer_layout(weights) == (pour_run, weights)
    before = (pour_run / "train_config.json").read_bytes()
    original = _config(pour_run)
    config = model._runtime_train_config(original, pour_run)
    model._validate_contract(config)
    assert config["data_contract"]["action_chunk"] == 100
    assert "data_contract" not in original
    assert (pour_run / "train_config.json").read_bytes() == before


@pytest.mark.parametrize("key,value", [
    ("joint_order", ["wrong"]), ("action_units", "degrees"),
    ("state_offsets", [-5, -4, -3, -2, -1, 0]),
    ("tactile_future_offsets", list(range(100))),
    ("image_transform", {"output_hw": [224, 224]}),
    ("tactile_preprocessing", ["right45 then left45"]),
    ("normalization_stats_file", "other.pt"),
    ("joint_lower", [0.0] * 54),
])
def test_pour_rejects_incompatible_preprocessing(pour_run, key, value):
    path = pour_run / "inference_config.json"
    config = json.loads(path.read_text())
    config[key] = value
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError, match=key):
        model._runtime_train_config(_config(pour_run), pour_run)


@pytest.mark.parametrize("section,key,value", [
    (None, "training_recipe_version", "unknown"),
    ("model_config", "action_chunk", 200),
    ("model_config", "hidden_dim", 256),
    ("model_config", "source_repo_commit", "unknown"),
])
def test_pour_rejects_incompatible_model(pour_run, section, key, value):
    config = _config(pour_run)
    (config if section is None else config[section])[key] = value
    with pytest.raises(ValueError):
        model._validate_contract(model._runtime_train_config(config, pour_run))


@pytest.mark.parametrize("architecture", ["unknown", "vitacformer_sh5_pour_h100_v3"])
def test_unknown_architecture_cannot_fall_back_to_legacy(architecture):
    config = _train_config()
    config["architecture_version"] = architecture
    with pytest.raises(ValueError, match="architecture_version"):
        model._validate_contract(config)


@pytest.mark.parametrize("key,value", [
    ("architecture_version", "vitacformer_sh5_h200_v1"),
    ("training_recipe_version", "unknown"), ("action_chunk", 200),
    ("global_step", 0), ("normalization_stats_sha256", "wrong"),
    ("inference_config_sha256", "wrong"),
])
def test_pour_rejects_checkpoint_binding_mismatch(pour_run, key, value):
    checkpoint = _metadata(pour_run)
    config = model._runtime_train_config(_config(pour_run), pour_run)
    model._validate_checkpoint_metadata(config, checkpoint, pour_run)
    checkpoint[key] = value
    with pytest.raises(ValueError, match=key):
        model._validate_checkpoint_metadata(config, checkpoint, pour_run)


@pytest.mark.parametrize("filename", ["normalization_stats.pt", "inference_config.json"])
def test_pour_rejects_modified_bound_files(pour_run, filename):
    checkpoint = _metadata(pour_run)
    config = model._runtime_train_config(_config(pour_run), pour_run)
    path = pour_run / filename
    path.write_bytes(path.read_bytes() + b" ")
    with pytest.raises(ValueError, match="sha256"):
        model._validate_checkpoint_metadata(config, checkpoint, pour_run)


def test_pour_rejects_train_inference_disagreement(pour_run):
    checkpoint = _metadata(pour_run)
    config = model._runtime_train_config(_config(pour_run), pour_run)
    config["model_config"]["action_chunk"] = 200
    with pytest.raises(ValueError, match="inference/train config model_config"):
        model._validate_checkpoint_metadata(config, checkpoint, pour_run)


@pytest.mark.parametrize("family", ["legacy", "h200", "pour"])
def test_policy_selects_the_trained_tactile_residual_behavior(pour_run, monkeypatch, family):
    captured = {}

    class Core(torch.nn.Module):
        def __init__(self, chunk_size):
            super().__init__()
            self.chunk_size = chunk_size

        def forward(self, state, image, tactile, persistence, residual_scale):
            captured["scale"] = residual_scale
            return torch.zeros(1, self.chunk_size, 54), persistence

    monkeypatch.setattr(model, "_ViTacFormerCore", Core)
    configs = {"legacy": _train_config(), "h200": _h200_config(),
               "pour": model._runtime_train_config(_config(pour_run), pour_run)}
    stats = {f"{name}_{kind}": torch.full((dim,), float(kind == "std"))
             for name, dim in (("state", 54), ("action", 54),
                               ("tactile_history", 180), ("tactile_future", 180))
             for kind in ("mean", "std")}
    policy = model.ViTacFormerPolicy(configs[family], stats)
    result = policy.predict_action_chunk({
        model._STATE_KEY: torch.zeros(1, 6, 54),
        model._IMAGE_KEY: torch.zeros(1, 3, 188, 336),
        model.VITACFORMER_TACTILE_BATCH_KEY: torch.zeros(1, 18, 180),
    })
    assert result.shape == (1, 200 if family == "h200" else 100, 54)
    assert torch.all(captured["scale"][:45] == 1)
    assert torch.all(captured["scale"][45:90] == (0 if family == "legacy" else 1))
    assert torch.all(captured["scale"][135:] == (0 if family == "legacy" else 1))


@pytest.mark.parametrize("device", ["cpu", "cuda"])
def test_real_pour_package_matches_original_inference(device, monkeypatch):
    """Opt in with VITACFORMER_POUR_TEST_ROOT; never connects to a robot."""
    root_value = os.environ.get("VITACFORMER_POUR_TEST_ROOT")
    if not root_value:
        pytest.skip("Set VITACFORMER_POUR_TEST_ROOT to the original pour package")
    if device == "cuda" and not torch.cuda.is_available():
        pytest.skip("CUDA unavailable")
    root = Path(root_value)
    monkeypatch.syspath_prepend(str(root / "source_snapshot" / "ViTacFormer_SH5"))
    # The packaged DETR argument parser otherwise consumes pytest's CLI flags.
    monkeypatch.setattr(sys, "argv", ["offline-parity"])
    from inference_loader import load_run, predict_normalized
    spec = importlib.util.spec_from_file_location("pour_preprocess", root / "preprocess.py")
    preprocess = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(preprocess)
    from vitacformer_engine.prediction import PredictionMixin

    torch.set_num_threads(2)
    policy = model.load_vitacformer_policy(str(root / "checkpoints"), torch.device(device))
    recipe = json.loads((root / "train_config.json").read_text())["training_recipe_version"]
    checkpoint_name = ("best_validation.pt" if recipe == model._POUR_H100_LR1E4_TRAINING_RECIPE_VERSION
                       else "best_model.pt")
    reference, stats, _ = load_run(root, checkpoint=checkpoint_name, device=device)
    assert policy.config.checkpoint_path == str(root / "checkpoints" / checkpoint_name)
    rng = np.random.default_rng(519)
    for contact in (False, True):
        image = rng.integers(0, 256, (376, 672, 3), dtype=np.uint8)
        state = stats["state_mean"].cpu().numpy() + (
            rng.normal(0, 0.1, (6, 54)).astype(np.float32) * stats["state_std"].cpu().numpy()
        )
        baseline = rng.integers(0, 15, 90).astype(np.float32)
        raw = np.broadcast_to(baseline, (18, 90)).copy()
        if contact:
            raw += rng.integers(-5, 80, (18, 90)).astype(np.float32)
        original_batch = preprocess.prepare_observation(image, state, raw, baseline, stats, device)
        import cv2
        pressure = np.maximum(raw - baseline, 0)
        batch = {
            model._IMAGE_KEY: torch.from_numpy(cv2.resize(image, (336, 188))).permute(2, 0, 1).float().div(255)[None].to(device),
            model._STATE_KEY: torch.from_numpy(state)[None].to(device),
            model.VITACFORMER_TACTILE_BATCH_KEY: torch.from_numpy(
                np.concatenate((pressure, pressure - pressure[:1]), axis=-1),
            )[None].to(device),
        }
        with torch.inference_mode(), torch.autocast(
            device_type=device, dtype=torch.bfloat16, enabled=device == "cuda",
        ):
            expected, expected_tactile = predict_normalized(reference, stats, original_batch)
            actual = policy.predict_action_chunk(batch)
            persistence, scale = model._build_tactile_persistence(
                batch[model.VITACFORMER_TACTILE_BATCH_KEY],
                policy._tactile_future_mean, policy._tactile_future_std,
                right_persistence=False,
            )
            _, actual_tactile = policy.model(
                original_batch["state"].flatten(1), original_batch["image"][:, None],
                original_batch["tactile_history"].flatten(1), persistence, scale,
            )
        error = float((actual - expected).abs().max())
        tactile_error = float((actual_tactile - expected_tactile).abs().max())
        print(f"{device} contact={contact} action_max_error={error} tactile_max_error={tactile_error}")
        torch.testing.assert_close(actual, expected, atol=1e-5, rtol=1e-5)
        torch.testing.assert_close(actual_tactile, expected_tactile, atol=1e-5, rtol=1e-5)
        assert torch.isfinite(actual).all()
        assert torch.all(actual >= model._JOINT_LOWER.to(device))
        assert torch.all(actual <= model._JOINT_UPPER.to(device))
        chunk = PredictionMixin._predict_policy_chunk(policy, batch)
        assert chunk.shape == (100, 54)
        np.testing.assert_allclose(chunk, expected[0].double().cpu().numpy(), atol=1e-5, rtol=1e-5)
