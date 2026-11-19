import json
import os


from carpointcloud_analyzer.infrastructure.runtime_configuration import load_runtime_configuration


def test_file_is_authoritative_and_removes_stale_environment(monkeypatch, tmp_path):
    monkeypatch.setenv("CARPOINT_DB_USER", "inherited-account")
    monkeypatch.setenv("CARPOINT_STALE", "stale")
    monkeypatch.setenv("CARPOINT_DB_ENABLED", "false")
    path = tmp_path / "appsettings.json"
    path.write_text(json.dumps({"CARPOINT_DB_USER": "file-account", "CARPOINT_DB_ENABLED": True}), encoding="utf-8")
    load_runtime_configuration(path)
    assert os.environ["CARPOINT_DB_USER"] == "file-account"
    assert os.environ["CARPOINT_DB_ENABLED"] == "true"
    assert "CARPOINT_STALE" not in os.environ
