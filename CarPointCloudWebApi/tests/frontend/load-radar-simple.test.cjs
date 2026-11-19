// 用户确认的精简范围：只有状态、点云、结果及按需历史，没有逐车审批。
const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const root = path.resolve(__dirname, '../../wwwroot/pointcloud-viewer');
const html = fs.readFileSync(path.join(root, 'load-radar.html'), 'utf8');
const script = fs.readFileSync(path.join(root, 'js/load-radar.js'), 'utf8');
const css = fs.readFileSync(path.join(root, 'css/load-radar.css'), 'utf8');

test('载货页面沿用系统深色主题并固定呈现四通道', () => {
    assert.match(html, /css\/style\.css\?v=9/);
    assert.match(html, /css\/load-radar\.css\?v=20260928-3/);
    assert.match(html, /class="radar-header"/);
    assert.match(html, /class="chantabs"/);
    assert.match(html, /class="grid"/);
    assert.match(css, /repeat\(4,minmax\(0,1fr\)\)/);
    assert.match(script, /FIXED_CHANNELS = \[/);
    assert.match(html, /id="runLogs"/);
    assert.match(css, /grid-template-columns:minmax\(0,1\.6fr\) minmax\(340px,1fr\)/);
    assert.match(html, /class="lane-log-host" id="laneLogs"/);
    assert.doesNotMatch(css, /\.lane-logs-grid/);
    assert.match(script, /全部车道车辆运行日志/);
    assert.match(css, /--bg:var\(--bg-primary\)/);
    assert.match(css, /--tx:var\(--text-primary\)/);
    assert.doesNotMatch(css, /\.load-radar-page \.global-nav\s*\{/);
    assert.match(css, /\.live-layout\.view-single \.log-table-scroll \{ max-height:calc\(100vh - 260px\); min-height:380px; \}/);
    assert.match(css, /\.lane-log \.log-empty \{ height:380px;[^}]+vertical-align:middle;/);
    assert.doesNotMatch(html, /live-panel/);
});
test('主页面不再提供逐车关联、基线审批和命令管理表单', () => {
    for (const id of ['loadBindForm', 'loadBaselineForm', 'loadActivateForm', 'loadPublishForm',
        'queryCommand', 'retryCommand', 'captureCandidate', 'remeasureCurrent', 'baselineRows']) {
        assert.ok(!html.includes('id="' + id + '"'), '仍有多余入口：' + id);
    }
    assert.doesNotMatch(html, /SHA-256|入库依据|关联依据|来车流程编号|车辆 \/ 挂车身份标识/);
});
test('历史只有按需入口，提供车牌时间通道查询', () => {
    assert.match(html, /id="openHistory"/);
    assert.match(html, /<dialog[^>]+id="historyDialog"/);
    for (const id of ['filterPlate', 'filterFrom', 'filterTo', 'filterChannel']) assert.ok(html.includes('id="' + id + '"'));
});
test('客户页面无业务写请求、模拟入口和技术诊断面板', () => {
    assert.doesNotMatch(script, /method:\s*['"](?:POST|PUT|DELETE)|\/baselines|\/commands|\/binding|\/publish|\/api\/demo/);
    assert.doesNotMatch(script, /判定链与拟合诊断|体积原始结果|baselineSha|configurationHash|JSON\.stringify\(\{/);
});
