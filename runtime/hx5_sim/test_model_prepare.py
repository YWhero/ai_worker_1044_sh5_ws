"""Verify fixed-revision model preparation without network or real weights."""
import hashlib
import importlib.util
import io
import json
from pathlib import Path

import pytest


SCRIPT = Path(__file__).with_name('prepare_model.py')


def model_preparer(monkeypatch, tmp_path):
    spec = importlib.util.spec_from_file_location('hx5_model_prepare_test', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, '__file__', str(tmp_path / 'runtime/hx5_sim/prepare_model.py'))
    return module


def install_export(monkeypatch, module, *, revision=None, corrupt_checkpoint=False):
    payloads = {name: f'fixture:{name}'.encode() for name in module.FILES if name != 'SHA256SUMS'}
    payloads['SHA256SUMS'] = ''.join(
        f'{hashlib.sha256(data).hexdigest()}  {name}\n'
        for name, data in payloads.items()
    ).encode()
    siblings = []
    for name, data in payloads.items():
        entry = {'rfilename': name, 'size': len(data)}
        if name.endswith('.pt'):
            entry['lfs'] = {'sha256': hashlib.sha256(data).hexdigest()}
        siblings.append(entry)
    metadata = json.dumps({'sha': revision or module.REVISION, 'siblings': siblings}).encode()
    requests = []

    def urlopen(url, timeout):
        requests.append(url)
        if '/api/models/' in url:
            return io.BytesIO(metadata)
        name = url.split(f'/resolve/{module.REVISION}/', 1)[1]
        data = payloads[name]
        if corrupt_checkpoint and name == 'checkpoints/best_validation.pt':
            data = b'X' + data[1:]
        return io.BytesIO(data)

    monkeypatch.setattr(module.urllib.request, 'urlopen', urlopen)
    return payloads, requests


def test_preparation_verifies_export_and_receipt_then_reuses_verified_checkpoint(monkeypatch, tmp_path):
    module = model_preparer(monkeypatch, tmp_path)
    payloads, requests = install_export(monkeypatch, module)
    module.main()
    destination = tmp_path / 'simulation/cyclo/model/vitacformer/task519_pour_h100'
    receipt = json.loads((destination / 'download_receipt.json').read_text())
    assert receipt['revision'] == module.REVISION
    assert receipt['repo_id'] == module.REPO
    assert receipt['checkpoint'] == 'checkpoints/best_validation.pt'
    assert receipt['files'] == {
        name: hashlib.sha256(payloads[name]).hexdigest() for name in module.FILES
    }
    assert not list(destination.rglob('*.part'))
    checkpoint_downloads = sum('/resolve/' in url and url.endswith('best_validation.pt') for url in requests)
    assert checkpoint_downloads == 1
    module.main()
    assert sum('/resolve/' in url and url.endswith('best_validation.pt') for url in requests) == 1


def test_unexpected_remote_revision_fails_before_any_model_download(monkeypatch, tmp_path):
    module = model_preparer(monkeypatch, tmp_path)
    _, requests = install_export(monkeypatch, module, revision='0' * 40)
    with pytest.raises(RuntimeError, match='Model revision changed unexpectedly'):
        module.main()
    assert len(requests) == 1
    assert '/api/models/' in requests[0]
    assert not list(tmp_path.rglob('download_receipt.json'))


def test_corrupt_checkpoint_is_rejected_without_overwriting_existing_checkpoint(monkeypatch, tmp_path):
    module = model_preparer(monkeypatch, tmp_path)
    install_export(monkeypatch, module, corrupt_checkpoint=True)
    destination = tmp_path / 'simulation/cyclo/model/vitacformer/task519_pour_h100'
    checkpoint = destination / 'checkpoints/best_validation.pt'
    checkpoint.parent.mkdir(parents=True)
    checkpoint.write_bytes(b'previous checkpoint')
    with pytest.raises(RuntimeError, match='Checksum mismatch: checkpoints/best_validation.pt'):
        module.main()
    assert checkpoint.read_bytes() == b'previous checkpoint'
    assert not (destination / 'download_receipt.json').exists()
