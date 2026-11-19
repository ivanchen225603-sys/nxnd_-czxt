"""校验归档完整性、脱敏结果、Linux 权限位以及当前源码是否被纳入。"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import tarfile
from pathlib import Path, PurePosixPath

from tools.build_kylin_delivery import ROOT, sha, write_json


def verify(output: Path, bash: str) -> dict:
    """只检查交付字节内容，不解包，也不启动任何服务。"""
    delivery = json.loads((output / 'DELIVERY.json').read_text(encoding='utf-8'))
    results = []
    for entry in delivery['packages']:
        archive = output / entry['file']
        assert sha(archive) == entry['sha256'], archive.name
        prefix = archive.name.removesuffix('.tar.gz')
        with tarfile.open(archive, 'r:gz') as bundle:
            members = bundle.getmembers()
            file_hashes = {}
            configs = []
            script_count = 0
            source_count = 0
            for member in members:
                path = PurePosixPath(member.name)
                assert not path.is_absolute() and '..' not in path.parts, member.name
                assert path.parts[0] == prefix, member.name
                assert not member.issym() and not member.isdev(), member.name
                if not member.isfile() and not member.islnk():
                    continue
                relative = PurePosixPath(*path.parts[1:])
                assert relative.suffix.lower() not in {'.pcd', '.pdb', '.log'}, member.name
                assert not any(p in {'logs', '.uploads', 'AGENTS.md'} for p in relative.parts), member.name
                stream = bundle.extractfile(member)
                assert stream is not None
                digest = hashlib.file_digest(stream, 'sha256').hexdigest()
                file_hashes[relative.as_posix()] = digest
                if relative.name.lower().startswith('appsettings'):
                    assert relative.as_posix() == 'app/appsettings.json'
                    configs.append(json.load(bundle.extractfile(member)))
                    assert member.mode == 0o640
                if relative.suffix == '.sh' and len(relative.parts) == 1:
                    content = bundle.extractfile(member).read()
                    assert b'\r' not in content and content.startswith(b'#!/usr/bin/env bash\n')
                    assert member.mode & 0o111 == 0o111
                    subprocess.run([bash, '-n'], input=content, check=True, capture_output=True)
                    script_count += 1
                source = None
                if relative.as_posix().startswith('app/carpointcloud_analyzer/'):
                    source = ROOT / 'CarPointCloud_Analyzer_fixed' / str(relative.relative_to('app'))
                elif relative.as_posix().startswith('app/wwwroot/'):
                    source = ROOT / 'CarPointCloudWebApi' / str(relative.relative_to('app'))
                elif relative.as_posix() in {'native/ladar_bridge.cpp', 'native/ladar_bridge.h'}:
                    source = ROOT / 'CarPointCloudLadarWorker/native/linux' / relative.name
                if source is not None:
                    assert source.is_file() and sha(source) == digest, member.name
                    source_count += 1
            assert len(configs) == 1
            config = configs[0]
            encoded = json.dumps(config)
            assert ':\\\\' not in encoded and 'D:/' not in encoded
            if 'LadarCapture' in config:
                cfg = config['LadarCapture']
                assert not cfg['AppSecret'] and not cfg['EnableApiRequests']
                assert cfg['HttpApiHost'] == '127.0.0.1'
                assert not cfg['ContinuousFrameContractEvidence']
                assert not cfg['MaximumFrameDeliveryDelayMilliseconds']
                assert all(not d['Address'] for d in cfg['Devices'])
            elif 'Auth' in config:
                assert not config['Auth']['JwtSecret']
                assert not config['ConnectionStrings']['CarPointCloudDb']
                assert not config['Integration']['AppSecret']
                assert config['Database']['InitializeOnStartup'] is False
            else:
                assert not config['CARPOINT_DB_PASSWORD'] and not config['CARPOINT_DB_USER']
            manifest = bundle.extractfile(f'{prefix}/SHA256SUMS').read().decode('utf-8')
            expected = dict(line.split('  ', 1)[::-1] for line in manifest.splitlines())
            assert expected == {k: v for k, v in file_hashes.items() if k != 'SHA256SUMS'}
            results.append({'file': archive.name, 'verifiedFiles': len(file_hashes),
                            'bashSyntaxChecked': script_count, 'currentSourceFilesMatched': source_count})
    return {'passed': True, 'linuxExecutionVerified': False, 'packages': results}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--bash', required=True)
    args = parser.parse_args()
    result = verify(args.output_root, args.bash)
    write_json(args.output_root / 'VERIFICATION.json', result)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
