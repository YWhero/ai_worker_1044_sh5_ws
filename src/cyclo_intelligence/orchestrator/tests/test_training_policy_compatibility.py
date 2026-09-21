import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def training_policies():
    source = ROOT / "orchestrator/orchestrator/training/zenoh_training_manager.py"
    for node in ast.walk(ast.parse(source.read_text())):
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == "SUPPORTED_POLICIES"
            for target in node.targets
        ):
            return ast.literal_eval(node.value)
    raise AssertionError("Training policy list is missing")


def test_baseline_training_policies_are_preserved():
    assert training_policies() == [
        "tdmpc", "diffusion", "act", "vqbet", "pi0", "pi0_fast", "pi05",
        "smolvla", "groot", "xvla", "sac",
    ]


def test_training_choices_exist_in_pinned_lerobot():
    policy_root = ROOT / "cyclo_brain/policy/lerobot/lerobot/src/lerobot/policies"
    for name in training_policies():
        if name == "groot":
            continue
        assert (policy_root / name / f"configuration_{name}.py").is_file(), name
