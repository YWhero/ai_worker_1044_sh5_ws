import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys

import numpy as np
import pytest
import torch

from test_model_contract import model


FIXTURE = Path(__file__).parent / "fixtures/upright_h100/inference_config.json"
ARCHITECTURE = "vitacformer_sh5_upright_h100_v2"


@pytest.fixture
def upright_run(tmp_path):
    inference = json.loads(FIXTURE.read_text())
    train = {key: inference[key] for key in (
        "architecture_version", "training_recipe_version", "model_config",
    )}
    (tmp_path / "train_config.json").write_text(json.dumps(train))
    (tmp_path / "normalization_stats.pt").write_bytes(b"synthetic normalization")
    inference["normalization_stats_sha256"] = hashlib.sha256(
        (tmp_path / "normalization_stats.pt").read_bytes(),
    ).hexdigest()
    (tmp_path / "inference_config.json").write_text(json.dumps(inference))
    (tmp_path / "checkpoints").mkdir()
    for filename in ("best_validation.pt", "latest_model.pt", "best_model.pt"):
        (tmp_path / "checkpoints" / filename).touch()
    return tmp_path


def runtime_config(root):
    config = json.loads((root / "train_config.json").read_text())
    return model._runtime_train_config(config, root)


def checkpoint_metadata(root):
    inference = json.loads((root / "inference_config.json").read_text())
    return {
        "architecture_version": ARCHITECTURE,
        "training_recipe_version": inference["training_recipe_version"],
        "action_chunk": 100,
        "global_step": 1,
        "normalization_stats_sha256": inference["normalization_stats_sha256"],
        "inference_config_sha256": hashlib.sha256(
            (root / "inference_config.json").read_bytes(),
        ).hexdigest(),
    }


def test_reference_fixture_matches_published_hash():
    assert hashlib.sha256(FIXTURE.read_bytes()).hexdigest() == (
        "c6a2cce55589a5d21dba736911577f69bfd0f853633d2dbe86864fa242acc6a1"
    )


def test_upright_contract_and_checkpoint_bindings_without_artifact_edits(upright_run):
    files = {name: (upright_run / name).read_bytes()
             for name in ("train_config.json", "inference_config.json")}
    config = runtime_config(upright_run)
    model._validate_contract(config)
    model._validate_checkpoint_metadata(config, checkpoint_metadata(upright_run), upright_run)
    assert config["data_contract"]["action_dim"] == 54
    assert config["data_contract"]["tactile_rep_dim"] == 180
    for name, original in files.items():
        assert (upright_run / name).read_bytes() == original


@pytest.mark.parametrize("selection", ["", "checkpoints", "checkpoints/best_validation.pt", "checkpoints/latest_model.pt"])
def test_upright_browser_paths_and_explicit_checkpoint_selection(upright_run, selection):
    root, explicit = model._resolve_vitacformer_layout(upright_run / selection)
    assert root == upright_run
    selected = model._checkpoint_path(root, explicit, architecture_version=ARCHITECTURE)
    expected = "latest_model.pt" if selection.endswith("latest_model.pt") else "best_validation.pt"
    assert selected == upright_run / "checkpoints" / expected


def test_upright_never_silently_falls_back_to_latest_or_legacy_best(upright_run):
    (upright_run / "checkpoints/best_validation.pt").unlink()
    with pytest.raises(FileNotFoundError, match="best_validation.pt"):
        model._checkpoint_path(upright_run, architecture_version=ARCHITECTURE)


@pytest.mark.parametrize("key,value", [
    ("architecture_version", "vitacformer_sh5_pour_h100_v2"),
    ("training_recipe_version", "task519_folder159_gt75_residual_unpenalized_r1"),
    ("action_chunk", 200), ("global_step", 0),
    ("normalization_stats_sha256", "wrong"), ("inference_config_sha256", "wrong"),
])
def test_upright_rejects_wrong_checkpoint_metadata(upright_run, key, value):
    checkpoint = checkpoint_metadata(upright_run)
    checkpoint[key] = value
    with pytest.raises(ValueError):
        model._validate_checkpoint_metadata(runtime_config(upright_run), checkpoint, upright_run)


@pytest.mark.parametrize("key,value", [
    ("joint_order", ["wrong"]), ("action_units", "degrees"),
    ("state_offsets", list(range(6))), ("tactile_preprocessing", ["right before left"]),
    ("joint_upper", [10] * 54),
])
def test_upright_preserves_preprocessing_and_joint_contract_checks(upright_run, key, value):
    path = upright_run / "inference_config.json"
    config = json.loads(path.read_text())
    config[key] = value
    path.write_text(json.dumps(config))
    with pytest.raises(ValueError, match=key):
        runtime_config(upright_run)


def test_upright_keeps_both_hands_learned_tactile_residuals(upright_run, monkeypatch):
    scales = []

    class Core(torch.nn.Module):
        def __init__(self, chunk_size):
            super().__init__()
            self.chunk_size = chunk_size

        def forward(self, state, image, tactile, persistence, residual_scale):
            scales.append(residual_scale)
            return torch.zeros(1, self.chunk_size, 54), persistence

    monkeypatch.setattr(model, "_ViTacFormerCore", Core)
    stats = {f"{name}_{kind}": torch.full((dimension,), float(kind == "std"))
             for name, dimension in (("state", 54), ("action", 54),
                                     ("tactile_history", 180), ("tactile_future", 180))
             for kind in ("mean", "std")}
    policy = model.ViTacFormerPolicy(runtime_config(upright_run), stats)
    action = policy.predict_action_chunk({
        model._STATE_KEY: torch.zeros(1, 6, 54),
        model._IMAGE_KEY: torch.zeros(1, 3, 188, 336),
        model.VITACFORMER_TACTILE_BATCH_KEY: torch.zeros(1, 18, 180),
    })
    assert action.shape == (1, 100, 54)
    assert torch.all(scales[0] == 1)


def test_upright_load_uses_reference_default_and_strict_state_dict(upright_run, monkeypatch):
    loaded = []
    strict_values = []
    stats = {f"{name}_{kind}": torch.full((dimension,), float(kind == "std"))
             for name, dimension in (("state", 54), ("action", 54),
                                     ("tactile_history", 180), ("tactile_future", 180))
             for kind in ("mean", "std")}

    def load(path, *, map_location, weights_only):
        loaded.append(Path(path).name)
        assert map_location == "cpu" and weights_only
        return stats if Path(path).name == "normalization_stats.pt" else {
            **checkpoint_metadata(upright_run), "model": {},
        }

    class Policy(torch.nn.Module):
        def __init__(self, config, normalization):
            super().__init__()
            self.config = type("Config", (), {})()

        def load_state_dict(self, state, strict):
            strict_values.append(strict)

    monkeypatch.setattr(torch, "load", load)
    monkeypatch.setattr(model, "ViTacFormerPolicy", Policy)
    policy = model.load_vitacformer_policy(str(upright_run), torch.device("cpu"))
    assert loaded == ["normalization_stats.pt", "best_validation.pt"]
    assert strict_values == [True]
    assert policy.config.checkpoint_path.endswith("/checkpoints/best_validation.pt")


def test_upright_random_weight_parity_with_reference_source(upright_run, monkeypatch):
    root_value = os.environ.get("VITACFORMER_UPRIGHT_REFERENCE_ROOT")
    if not root_value:
        pytest.skip("Set VITACFORMER_UPRIGHT_REFERENCE_ROOT to the pinned reference source")
    root = Path(root_value)
    monkeypatch.syspath_prepend(str(root / "source_snapshot/ViTacFormer_SH5"))
    monkeypatch.setattr(sys, "argv", ["offline-source-parity"])
    from sh5_model import build_policy
    from inference_loader import predict_normalized
    spec = importlib.util.spec_from_file_location("upright_reference_preprocess", root / "preprocess.py")
    preprocess = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(preprocess)
    import cv2

    torch.set_num_threads(2)
    torch.manual_seed(608)
    stats = {f"{name}_{kind}": torch.full((dimension,), float(kind == "std"))
             for name, dimension in (("state", 54), ("action", 54),
                                     ("tactile_history", 180), ("tactile_future", 180))
             for kind in ("mean", "std")}
    reference = build_policy(pretrained_backbone=False).eval()
    policy = model.ViTacFormerPolicy(runtime_config(upright_run), stats).eval()
    policy.load_state_dict(reference.state_dict(), strict=True)
    random = np.random.default_rng(608)
    for contact in (False, True):
        image = random.integers(0, 256, (376, 672, 3), dtype=np.uint8)
        state = random.normal(0, 0.1, (6, 54)).astype(np.float32)
        baseline = random.integers(0, 15, 90).astype(np.float32)
        raw = np.broadcast_to(baseline, (18, 90)).copy()
        if contact:
            raw += random.integers(-5, 80, (18, 90)).astype(np.float32)
        normalized = preprocess.prepare_observation(image, state, raw, baseline, stats, "cpu")
        pressure = np.maximum(raw - baseline, 0)
        batch = {
            model._IMAGE_KEY: torch.from_numpy(cv2.resize(image, (336, 188))).permute(2, 0, 1).float().div(255)[None],
            model._STATE_KEY: torch.from_numpy(state)[None],
            model.VITACFORMER_TACTILE_BATCH_KEY: torch.from_numpy(
                np.concatenate((pressure, pressure - pressure[:1]), axis=-1),
            )[None],
        }
        with torch.inference_mode():
            expected, expected_tactile = predict_normalized(reference, stats, normalized)
            actual = policy.predict_action_chunk(batch)
            persistence, scale = model._build_tactile_persistence(
                batch[model.VITACFORMER_TACTILE_BATCH_KEY],
                policy._tactile_future_mean, policy._tactile_future_std,
                right_persistence=False,
            )
            _, actual_tactile = policy.model(
                normalized["state"].flatten(1), normalized["image"][:, None],
                normalized["tactile_history"].flatten(1), persistence, scale,
            )
        torch.testing.assert_close(actual, expected, atol=1e-5, rtol=1e-5)
        torch.testing.assert_close(actual_tactile, expected_tactile, atol=1e-5, rtol=1e-5)
