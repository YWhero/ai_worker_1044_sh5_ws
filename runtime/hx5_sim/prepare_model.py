#!/usr/bin/env python3
"""Download the user-selected Pour H100 export without executing remote code."""
import hashlib
import json
import os
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import urllib.request

REPO = 'Dongkkka/Task000519_PourWater_ViTacFormer_H100_LR1e4_B512_Hand_Intern'
REVISION = 'b4f21caa4e6a7a05814958b63d5ced46a9374321'
FILES = (
    'README.md', 'SHA256SUMS', 'config.json', 'train_config.json',
    'inference_config.json', 'normalization_stats.pt', 'normalization_stats.json',
    'artifact_receipt.json', 'offline_verification.json',
    'checkpoints/best_validation.pt',
)


def digest(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def main():
    root = Path(__file__).resolve().parents[2]
    target = root / 'simulation/cyclo/model/vitacformer/task519_pour_h100'
    target.mkdir(parents=True, exist_ok=True)
    with urllib.request.urlopen(
        f'https://huggingface.co/api/models/{REPO}/revision/{REVISION}?blobs=true',
        timeout=30,
    ) as response:
        info = json.load(response)
    if info['sha'] != REVISION:
        raise RuntimeError('Model revision changed unexpectedly')
    entries = {item['rfilename']: item for item in info['siblings']}

    def download(name):
        path = target / name
        item = entries[name]
        expected = item.get('lfs', {}).get('sha256')
        if path.is_file() and expected and digest(path) == expected:
            return name
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(path.name + '.part')
        with urllib.request.urlopen(
            f'https://huggingface.co/{REPO}/resolve/{REVISION}/{name}', timeout=60,
        ) as response, temporary.open('wb') as stream:
            while block := response.read(1024 * 1024):
                stream.write(block)
        if temporary.stat().st_size != item['size']:
            raise RuntimeError(f'Incomplete download: {name}')
        if expected and digest(temporary) != expected:
            raise RuntimeError(f'Checksum mismatch: {name}')
        os.replace(temporary, path)
        return name

    with ThreadPoolExecutor(max_workers=4) as pool:
        for name in pool.map(download, FILES):
            print(f'Prepared {name}', flush=True)
    hashes = {}
    for row in (target / 'SHA256SUMS').read_text().splitlines():
        if row.strip():
            value, name = row.split(maxsplit=1)
            hashes[name.lstrip('*').removeprefix('./')] = value
    for name in FILES:
        if name in hashes and digest(target / name) != hashes[name]:
            raise RuntimeError(f'Package checksum mismatch: {name}')
    (target / 'download_receipt.json').write_text(json.dumps({
        'repo_id': REPO, 'revision': REVISION,
        'files': {name: digest(target / name) for name in FILES},
        'checkpoint': 'checkpoints/best_validation.pt',
    }, indent=2) + '\n')
    print(f'Model ready: {target}', flush=True)


if __name__ == '__main__':
    main()
