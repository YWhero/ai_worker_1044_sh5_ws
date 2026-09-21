"""Pinned corrected Pour LR1e-4 export, without rewriting published assets."""

import hashlib
import json
from pathlib import Path

import pytest

from test_model_contract import model


FIXTURE = Path(__file__).parent / "fixtures/pour_h100_lr1e4"
ARCHITECTURE = "vitacformer_sh5_pour_h100_v2"
RECIPE = "task519_folder159_lr1e4_b512_w5_gt75_scratch_r1"
OLD_RECIPE = "task519_folder159_gt75_residual_unpenalized_r1"


@pytest.fixture
def corrected_run(tmp_path):
    for name in ("train_config.json", "inference_config.json"):
        (tmp_path / name).write_bytes((FIXTURE / name).read_bytes())
    (tmp_path / "checkpoints").mkdir()
    for name in ("best_validation.pt", "latest_model.pt", "best_model.pt"):
        (tmp_path / "checkpoints" / name).touch()
    return tmp_path


def runtime_config(root):
    return model._runtime_train_config(json.loads((root / "train_config.json").read_text()), root)


def test_corrected_fixture_pins_source_revision_and_hashes():
    source = json.loads((FIXTURE / "source.json").read_text())
    assert source["revision"] == "b4f21caa4e6a7a05814958b63d5ced46a9374321"
    assert source["sha256"] == {
        "train_config.json": "1706377f2b5176bb8a8e9dc4677078fb69bd90dc57e12f04d9b6fccf5517ed77",
        "inference_config.json": "899f6bbf2c1fc8949f34275d3abeaef2925ac356b1f19b44c3a8cc0c0a8a576d",
    }
    for name, digest in source["sha256"].items():
        assert hashlib.sha256((FIXTURE / name).read_bytes()).hexdigest() == digest


def test_corrected_recipe_preserves_audited_contract_and_source_assets(corrected_run):
    original = {name: (corrected_run / name).read_bytes()
                for name in ("train_config.json", "inference_config.json")}
    config = runtime_config(corrected_run)
    model._validate_contract(config)
    assert config["training_recipe_version"] == RECIPE
    assert config["data_contract"]["action_chunk"] == 100
    assert config["data_contract"]["state_dim"] == config["data_contract"]["action_dim"] == 54
    old = json.loads((FIXTURE.parent / "pour_h100/inference_config.json").read_text())
    new = json.loads(original["inference_config.json"])
    for key in ("model_config", "joint_order", "image_transform", "tactile_preprocessing",
                "normalization_stats_sha256", "joint_lower", "joint_upper"):
        assert new[key] == old[key]
    for name, content in original.items():
        assert (corrected_run / name).read_bytes() == content


@pytest.mark.parametrize("selection", ["", "checkpoints", "checkpoints/best_validation.pt",
                                        "checkpoints/latest_model.pt"])
def test_corrected_default_and_explicit_browser_paths(corrected_run, selection):
    root, explicit = model._resolve_vitacformer_layout(corrected_run / selection)
    assert root == corrected_run
    selected = model._checkpoint_path(root, explicit, architecture_version=ARCHITECTURE,
                                      training_recipe_version=RECIPE)
    expected = "latest_model.pt" if selection.endswith("latest_model.pt") else "best_validation.pt"
    assert selected == corrected_run / "checkpoints" / expected


def test_corrected_missing_validation_best_never_falls_back(corrected_run):
    (corrected_run / "checkpoints/best_validation.pt").unlink()
    with pytest.raises(FileNotFoundError, match="best_validation.pt"):
        model._checkpoint_path(corrected_run, architecture_version=ARCHITECTURE,
                               training_recipe_version=RECIPE)


def test_old_pour_default_remains_best_model(corrected_run):
    assert model._checkpoint_path(corrected_run, architecture_version=ARCHITECTURE,
                                  training_recipe_version=OLD_RECIPE).name == "best_model.pt"


def test_checkpoint_cannot_cross_bind_the_two_supported_pour_recipes(corrected_run):
    # Recipe validation precedes file hashes; a valid old recipe still cannot
    # load into a new training run just because tensor architecture matches.
    checkpoint = {"architecture_version": ARCHITECTURE,
                  "training_recipe_version": OLD_RECIPE, "action_chunk": 100}
    with pytest.raises(ValueError, match="training_recipe_version mismatch"):
        model._validate_checkpoint_metadata(runtime_config(corrected_run), checkpoint, corrected_run)


def test_unreviewed_pour_recipe_still_fails_closed(corrected_run):
    config = runtime_config(corrected_run)
    config["training_recipe_version"] = "unreviewed_lr_recipe"
    with pytest.raises(ValueError, match="training_recipe_version"):
        model._validate_contract(config)
