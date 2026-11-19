"""交付包不得包含现场凭据，也不得包含仅 Windows 可用或生成的产物。"""

import struct

import pytest

from tools.build_kylin_delivery import (
    NATIVE,
    analyzer_config,
    check_elf,
    copy_publish,
    web_config,
    worker_config,
    write,
)


def test_native_build_requires_cpp14_and_checks_loader_stderr():
    # std::make_unique 要求 C++14。ldd 返回 0 时带版本符号的动态库缺失仍可能
    # 只出现在 stderr，因此检查 stdout 并不可靠。
    assert '-std=gnu++14' in NATIVE
    assert 'ldd app/libladar_bridge.so 2>&1' in NATIVE
    assert "grep -q 'not found'" in NATIVE


def elf_header(machine=62):
    return b'\x7fELF\x02\x01' + bytes(12) + struct.pack('<H', machine)


def test_linux_architecture_guard(tmp_path):
    executable = tmp_path / 'app'
    executable.write_bytes(elf_header())
    check_elf(executable)
    executable.write_bytes(elf_header(183))
    with pytest.raises(ValueError, match='Not ELF64'):
        check_elf(executable)


def test_publish_filter_excludes_field_configuration_and_captures(tmp_path):
    source, target = tmp_path / 'published', tmp_path / 'safe'
    source.mkdir()
    (source / 'Worker').write_bytes(elf_header())
    for name in ('Worker.dll', 'appsettings.json', 'appsettings.Development.json',
                 'Worker.pdb', 'capture.pcd', 'debug.log'):
        (source / name).write_text('fixture')
    (source / 'wwwroot').mkdir()
    (source / 'wwwroot/live.html').write_text('page')
    (source / 'wwwroot/scan.pcd').write_text('private capture')
    copy_publish(source, target, 'Worker')
    assert {p.relative_to(target).as_posix() for p in target.rglob('*') if p.is_file()} == {
        'Worker', 'Worker.dll', 'wwwroot/live.html',
    }


def test_generated_scripts_have_linux_line_endings(tmp_path):
    script = tmp_path / 'run.sh'
    write(script, '#!/usr/bin/env bash\r\nexit 0\r\n')
    assert script.read_bytes() == b'#!/usr/bin/env bash\nexit 0\n'


def test_delivery_configuration_is_safe_but_requires_site_input():
    analyzer = analyzer_config()
    assert analyzer['CARPOINT_DB_PASSWORD'] == analyzer['CARPOINT_DB_USER'] == ''
    assert analyzer['CARPOINT_SERVICE_ALLOWED_ROOT'] == '/data/carpointcloud'
    web = web_config()
    assert web['ConnectionStrings']['CarPointCloudDb'] == web['Auth']['JwtSecret'] == ''
    assert web['Database']['InitializeOnStartup'] is False
    worker = worker_config()['LadarCapture']
    assert worker['HttpApiHost'] == '127.0.0.1'
    assert worker['EnableSyntheticRadar'] is False
    assert worker['EnableApiRequests'] is False
    assert worker['EnablePythonAnalyzer'] is True
    assert worker['AnalysisTaskDirectory'] == '/data/carpointcloud/worker-state/analysis-tasks'
    assert worker['PermanentPointCloudDirectories'] == ['/data/carpointcloud/empty-vehicle-profiles']
    assert not {'Enabled', 'EnableResultUploadToWebApi', 'AllowNonLoopbackControlApi',
                'PairManifestDirectory', 'AnalysisQueueDirectory', 'UploadQueueDirectory',
                'ExpectedFrameCycleSeconds', 'ContinuousFrameContractEvidence'} & worker.keys()
    assert worker['MaximumFrameDeliveryDelayMilliseconds'] is None
    assert all(device['Address'] == '' for device in worker['Devices'])
    assert all('MinCompleteFrameIntervalMilliseconds' not in device for device in worker['Devices'])
