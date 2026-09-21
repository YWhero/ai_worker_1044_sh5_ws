import ast
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import tomllib


ROOT = Path(__file__).resolve().parents[1]
LEROBOT = ROOT / "cyclo_brain/policy/lerobot/lerobot"
BACKPORT = ROOT / "cyclo_brain/policy/lerobot/backports"


def test_upstream_package_is_preserved_except_compatibility_field():
    manifest = json.loads((BACKPORT / "upstream_sha256.json").read_text())
    for relative, expected in manifest.items():
        data = (BACKPORT / relative).read_bytes()
        if relative.endswith("configuration_fastwam.py"):
            data = data.replace(b"    pretrained_revision: str | None = None\n", b"", 1)
        assert hashlib.sha256(data).hexdigest() == expected, relative


def test_build_backport_applies_to_pin_and_preserves_other_policies(tmp_path):
    for relative in ("pyproject.toml", "src/lerobot/policies/factory.py"):
        destination = tmp_path / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(LEROBOT / relative, destination)
    patch = BACKPORT / "fastwam-c8ce413.patch"
    subprocess.run(["git", "apply", "--check", str(patch)], cwd=tmp_path, check=True)
    subprocess.run(["git", "apply", str(patch)], cwd=tmp_path, check=True)
    baseline = tomllib.loads((LEROBOT / "pyproject.toml").read_text())
    patched = tomllib.loads((tmp_path / "pyproject.toml").read_text())
    assert patched["project"]["version"] == baseline["project"]["version"] == "0.5.2"
    extra = patched["project"]["optional-dependencies"].pop("fastwam")
    assert extra == ["lerobot[transformers-dep]", "lerobot[diffusers-dep]"]
    assert patched == baseline
    source = (tmp_path / "src/lerobot/policies/factory.py").read_text()
    ast.parse(source)
    assert source.count('elif name == "fastwam"') == 1
    assert source.count('elif policy_type == "fastwam"') == 1
    assert source.count('elif isinstance(policy_cfg, FastWAMConfig)') == 1
    assert source.count('elif isinstance(policy_cfg, VQBeTConfig)') == 1
    extras = set(baseline["project"]["optional-dependencies"]) | {"fastwam"}
    for architecture in ("amd64", "arm64"):
        dockerfile = (BACKPORT.parent / f"Dockerfile.{architecture}").read_text()
        install = next(line for line in dockerfile.splitlines() if "default)" in line)
        requested = re.search(r'\.\[([^]]+)\]', install).group(1).split(',')
        assert set(requested) <= extras
        assert dockerfile.count("COPY lerobot/lerobot/ ./") == 1


def test_vendored_python_files_compile_without_execution():
    for path in (BACKPORT / "fastwam").rglob("*.py"):
        compile(path.read_text(), str(path), "exec")


def test_build_context_excludes_host_submodule_git_pointer():
    ignored = (ROOT / "cyclo_brain/policy/.dockerignore").read_text().splitlines()
    assert "**/.git" in ignored
