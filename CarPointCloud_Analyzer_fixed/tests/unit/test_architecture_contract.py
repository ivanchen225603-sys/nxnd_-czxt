"""当前 HTTP 契约、共享数值行为与分层依赖。"""

import hashlib
import json
import ast
from pathlib import Path

import numpy as np

import pytest

from carpointcloud_analyzer.infrastructure.persistence import DisabledDimensionStore
from carpointcloud_analyzer.infrastructure.profile_storage import DisabledEmptyVehicleProfileStorage
from carpointcloud_analyzer.composition import create_app
from carpointcloud_analyzer.domain.utils.environment import env_bool


def test_http_openapi_remains_identical(tmp_path, monkeypatch):
    """字段、默认值、路由、模型名、响应声明均不得因拆文件发生变化。"""
    monkeypatch.delenv('CARPOINT_SERVICE_MAX_BATCH_SIZE', raising=False)
    monkeypatch.delenv('CARPOINT_EXTRINSIC_PATH', raising=False)
    app = create_app(
        dimension_store=DisabledDimensionStore(),
        empty_profile_storage=DisabledEmptyVehicleProfileStorage(),
    )
    try:
        encoded = json.dumps(app.openapi(), sort_keys=True).encode()
        assert hashlib.sha256(encoded).hexdigest() == (
            '18f43b7625aa79113e694848cdfd3942e0d63de0eecf0590b790ff324d4a7a20'
        )
    finally:
        app.state.analysis_service.shutdown()


@pytest.mark.parametrize('raw,expected', [
    ('1', True), (' TRUE ', True), ('yes', True), ('on', True),
    ('0', False), (' false ', False), ('no', False), ('off', False),
])
def test_shared_environment_boolean(raw, expected, monkeypatch):
    monkeypatch.setenv('CARPOINT_REFACTOR_TEST_FLAG', raw)
    assert env_bool('CARPOINT_REFACTOR_TEST_FLAG', not expected) is expected


def test_environment_defaults_and_errors(monkeypatch):
    name = 'CARPOINT_REFACTOR_TEST_FLAG'
    monkeypatch.delenv(name, raising=False)
    assert env_bool(name, True) is True
    assert env_bool(name, False) is False
    monkeypatch.setenv(name, '')
    with pytest.raises(ValueError, match=f'{name} 必须是 true/false'):
        env_bool(name, False)


def _module_dependencies():
    root = Path(__file__).resolve().parents[2] / 'carpointcloud_analyzer'
    graph = {}
    for path in root.rglob('*.py'):
        if path.name == '__init__.py':
            continue  # 兼容再导出，不作为实现层依赖边。
        module = '.'.join(path.relative_to(root).with_suffix('').parts)
        dependencies = set()
        for node in ast.walk(ast.parse(path.read_text(encoding='utf-8-sig'))):
            if isinstance(node, ast.ImportFrom) and node.level and node.module:
                parent = module.split('.')[:-node.level]
                dependencies.add('.'.join([*parent, node.module]))
            elif isinstance(node, ast.ImportFrom) and node.module:
                if node.module.startswith('carpointcloud_analyzer.'):
                    dependencies.add(node.module.removeprefix('carpointcloud_analyzer.'))
        graph[module] = dependencies
    return graph


def test_implementation_modules_have_no_import_cycle():
    graph = _module_dependencies()
    visited = set()

    def visit(module, stack):
        assert module not in stack, ' -> '.join([*stack, module])
        if module in visited:
            return
        for dependency in graph.get(module, ()):
            visit(dependency, [*stack, module])
        visited.add(module)

    for module in graph:
        visit(module, [])


@pytest.mark.parametrize('axis', [0, 1, 2])
def test_shared_surface_sampler_preserves_coordinates_and_order(axis):
    from carpointcloud_analyzer.domain.utils.surfaces import axis_aligned_face

    axes = [np.linspace(-1.2, 1.1, 7), np.linspace(-3.1, 3.2, 11),
            np.linspace(0.1, 2.9, 5)]
    value = .12345
    # 抽取前两份实现共同使用的原始网格公式，要求逐位相等。
    free = [i for i in range(3) if i != axis]
    aa, bb = np.meshgrid(axes[free[0]], axes[free[1]], indexing='ij')
    expected = np.empty((aa.size, 3))
    expected[:, axis], expected[:, free[0]], expected[:, free[1]] = value, aa.ravel(), bb.ravel()
    np.testing.assert_array_equal(axis_aligned_face(axes, axis, value), expected)


def test_new_layer_dependencies_point_towards_domain():
    graph = _module_dependencies()
    assert graph['application.ports'] == set()
    assert graph['application.service_contracts'] == set()
    for module, dependencies in graph.items():
        if module.startswith('domain.'):
            assert all(dependency.startswith('domain.') for dependency in dependencies), (module, dependencies)
        if module.startswith('application.'):
            assert not any(dependency.startswith('api.') or dependency == 'composition' for dependency in dependencies)
        if module.startswith('infrastructure.'):
            assert not any(dependency.startswith('api.') or dependency == 'composition' for dependency in dependencies)
