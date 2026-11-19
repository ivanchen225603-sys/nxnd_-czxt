from carpointcloud_analyzer.domain import stage_timing


def test_marks_are_noops_without_collection():
    stage_timing.lap("ignored")
    with stage_timing.stage("ignored"):
        stage_timing.lap("inner")


def test_nested_laps_are_prefixed_and_parent_laps_restart_after_stage():
    with stage_timing.collect() as timer:
        stage_timing.lap("before")
        with stage_timing.stage("algorithm"):
            stage_timing.lap("load-pcd")
            stage_timing.lap("ribs")
        stage_timing.lap("after")

    names = [name for name, _ in timer.entries]
    assert names == ["before", "algorithm/load-pcd", "algorithm/ribs", "algorithm", "after"]
    assert all(ms >= 0 for _, ms in timer.entries)
    assert "algorithm/ribs=" in timer.describe()
    # 采样结束后恢复为空操作，不会串到下一次请求。
    stage_timing.lap("later")
    assert len(timer.entries) == 5


def test_stage_is_recorded_when_the_body_raises():
    with stage_timing.collect() as timer:
        try:
            with stage_timing.stage("fusion"):
                raise ValueError("boom")
        except ValueError:
            pass
    assert [name for name, _ in timer.entries] == ["fusion"]


def test_stage_logs_correlate_workflow_and_separate_wall_and_cpu(caplog):
    with caplog.at_level('INFO'):
        with stage_timing.collect(workflow_id='workflow-1', pair_id='pair-1'):
            with stage_timing.stage('persist'):
                with stage_timing.stage('database'):
                    pass
    assert 'workflowId=workflow-1 pairId=pair-1 stage=persist/database' in caplog.text
    assert 'wall_ms=' in caplog.text
    assert 'thread_cpu_ms=' in caplog.text
    assert 'process_cpu_ms=' in caplog.text


def test_state_failure_does_not_replace_algorithm_exception(caplog):
    import pytest

    def unavailable(boundary):
        raise OSError('metrics unavailable')

    with caplog.at_level('INFO'):
        with pytest.raises(ValueError, match='algorithm failure'):
            with stage_timing.collect(workflow_id='one', snapshot=unavailable):
                with stage_timing.stage('fusion'):
                    raise ValueError('algorithm failure')
    assert '运行状态采样失败' in caplog.text
    stage_timing.lap('outside')


def test_state_boundaries_include_driver_stages_and_remain_outside_payload(caplog):
    with caplog.at_level('INFO'):
        with stage_timing.collect(workflow_id='one', snapshot=lambda boundary: {'nativeThreadId': 42}) as timer:
            with stage_timing.stage('persist'):
                with stage_timing.stage('database'):
                    with stage_timing.stage('db-commit'):
                        pass
    assert 'boundary=request-start' in caplog.text
    assert 'boundary=persist/database/db-commit' in caplog.text
    assert 'boundary=request-end' in caplog.text
    assert '"nativeThreadId":42' in caplog.text
    assert [x[0] for x in timer.entries] == ['persist/database/db-commit', 'persist/database', 'persist']
