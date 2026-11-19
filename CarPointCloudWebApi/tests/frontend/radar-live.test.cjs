const { test } = require('node:test');
const assert = require('node:assert/strict');
const vm = require('node:vm');
const fs = require('node:fs');
const path = require('node:path');

const scriptRoot = path.resolve(__dirname, '../../wwwroot/pointcloud-viewer/js');
const flush = () => new Promise(resolve => setImmediate(resolve));
function harness(search = '', roleCode = 'admin') {
    const elements = new Map(), timers = [], requests = [], drawings = [];
    function element(id) {
        if (!elements.has(id)) elements.set(id, {
            handlers: {}, children: [], style: {}, value: '', checked: false,
            addEventListener(event, fn) { this.handlers[event] = fn; },
            appendChild(child) { this.children.push(child); }, focus() {},
            set innerHTML(value) { this.html = value; this.children = []; },
            get innerHTML() { return this.html; }
        });
        return elements.get(id);
    }
    let initialize;
    const context = vm.createContext({
        ArrayBuffer, Float32Array, Uint8Array, DataView, TextDecoder, URLSearchParams, console,
        location: { search },
        atob: value => Buffer.from(value, 'base64').toString('binary'),
        document: {
            addEventListener: (_, fn) => { initialize = fn; },
            getElementById: element, querySelectorAll: () => [],
            createElement: () => element(Symbol())
        },
        requestAnimationFrame() {},
        setInterval(fn, ms) { const timer = { fn, ms, active: true }; timers.push(timer); return timer; },
        clearInterval(timer) { if (timer) timer.active = false; },
        API: {
            getCurrentUser: () => ({ roleCode }),
            request(url, options) { return new Promise((resolve, reject) => requests.push({ url, options, resolve, reject })); },
            arrayBuffer(url) { return new Promise((resolve, reject) => requests.push({ url, binary: true, resolve, reject })); }
        },
        PointCloudGL: { create: () => ({
            setPoints(points, a, b, fit, synthetic, surface) { drawings.push({ count: points.length, a, b, fit, synthetic, surface }); },
            render() {}, setView() {}, setPointSize() {}
        }) }
    });
    vm.runInContext(fs.readFileSync(path.join(scriptRoot, 'pcd-parser.js'), 'utf8'), context);
    vm.runInContext(fs.readFileSync(path.join(scriptRoot, 'radar-live.js'), 'utf8'), context);
    initialize();
    const take = part => {
        const index = requests.findIndex(r => r.url.includes(part));
        assert.notEqual(index, -1, `missing request: ${part}`);
        return requests.splice(index, 1)[0];
    };
    return { elements, requests, drawings, take, element,
        advance(ms) { vm.runInContext(`Date.now = () => ${Date.now() + ms}`, context); },
        tick(ms) { timers.filter(t => t.active && t.ms === ms).forEach(t => t.fn()); },
        click(id) { return element(id).handlers.click(); },
        latest() { return drawings.at(-1); }
    };
}

test('viewer can preview but cannot dispatch, force redispatch or cancel', async () => {
    const h = harness('', 'viewer');
    assert.equal(h.element('btnCapture').hidden, true);
    assert.equal(h.element('btnCancelCapture').hidden, true);
    h.element('plateInput').value = '宁A25771';
    await h.click('btnCapture');
    await h.click('btnConfirmRedispatch');
    await h.click('btnCancelCapture');
    assert.equal(h.requests.filter(r => r.options?.method === 'POST').length, 0);
    assert.ok(h.requests.some(r => r.url.includes('/fused-frame')));
});

test('unreadable original terminal keeps polling without redispatch even after the resume timeout', async () => {
    const h = harness(); await beginCapture(h);
    for (let attempt = 0; attempt < 3; attempt++) {
        if (attempt) { h.advance(attempt * 40000); h.tick(1500); }
        h.take('/captures/workflow-a').resolve({ data: {
            stage: 'analysis_failed', terminal: false, error: 'analysis_terminal_unavailable',
            message: '原分析终局暂不可读，已暂停自动重算'
        } });
        await flush();
        assert.match(h.element('captureBanner').textContent, /暂停自动重算/);
        assert.equal(h.requests.filter(r => r.options?.method === 'POST').length, 0);
    }
    h.tick(1500);
    h.take('/captures/workflow-a').resolve({ data: { stage: 'completed_without_dimensions', terminal: true,
        message: '原分析终局已恢复' } });
    await flush();
    h.tick(1500);
    assert.equal(h.requests.filter(r => r.url.includes('/captures')).length, 0);
});
function frame(version = 9, sessionId = 'session-a') {
    const points = Buffer.from(new Float32Array([1, 2, 3, 4, 5, 6]).buffer).toString('base64');
    return { updated: true, version, sessionId, points, pointCountA: 1, pointCountB: 1,
        cameraA: 'CAM_004', cameraB: 'CAM_005', pairDeltaMilliseconds: 100 };
}
async function ready(h) {
    h.take('/status').resolve({ data: { preview: { available: true } } });
    h.take('/fused-frame').resolve({ data: frame() });
    await flush();
    assert.equal(h.latest().count, 6);
}

test('radar sidebar omits service, warmup and decoded totals but keeps live connection data', async () => {
    const h = harness();
    const cameras = ['CAM_004', 'CAM_005'];
    h.take('/status').resolve({ data: {
        preview: { available: true, cameras: cameras.map(cameraIdentifier => ({
            cameraIdentifier, hasSample: true, fresh: true, ageSeconds: 1, sourcePointCount: 20000
        })) },
        scan: { safetyBlockReason: 'acquisition_unverified', warmupCameras: cameras.map(camera => ({
            camera, phase: 'timed_out'
        })) },
        radars: cameras.map(cameraIdentifier => ({ cameraIdentifier, connected: true,
            rawFrameCount: 123, decodedFrameCount: 999, warmup: { phase: 'timed_out' } }))
    } });
    h.take('/fused-frame').resolve({ data: frame() });
    await flush();
    for (const id of ['cameraAStatus', 'cameraBStatus']) {
        const html = h.element(id).innerHTML;
        assert.match(html, /已连接/);
        assert.match(html, /最近解析点云/);
        assert.match(html, /原始点数/);
        assert.match(html, /累计原始包/);
        assert.doesNotMatch(html, /预热|已解析点云|999/);
    }
    assert.equal(h.elements.has('serviceStatus'), false);
    const html = fs.readFileSync(path.join(scriptRoot, '../live.html'), 'utf8');
    assert.doesNotMatch(html, /id="serviceStatus"|<h3>服务<\/h3>/);
});

test('pair status follows displayed frame, not latest independent samples, and clears on pause', async () => {
    const h = harness(); await ready(h);
    h.tick(2000);
    h.take('/status').resolve({ data: { preview: { available: true, latestVersion: 99, cameras: [
        { cameraIdentifier: 'CAM_004', capturedAt: '2026-09-07T10:00:00Z' },
        { cameraIdentifier: 'CAM_005', capturedAt: '2026-09-07T10:00:05Z' }
    ] } } });
    await flush();
    assert.match(h.element('pairStatus').innerHTML, /0\.10 秒/);
    assert.doesNotMatch(h.element('pairStatus').innerHTML, /5\.00 秒|99/);
    assert.doesNotMatch(h.element('canvasOverlay').innerHTML, /融合正常/);
    h.click('btnStopWatch');
    assert.doesNotMatch(h.element('pairStatus').innerHTML, /0\.10 秒/);
});

test('completion mode replaces whole frame, separates synthetic points and rejects late mode responses', async () => {
    const h = harness(); await ready(h);
    h.element('showCompletion').checked = true;
    h.element('showCompletion').handlers.change();
    const pending = h.take('/completed-frame');
    assert.equal(h.latest().count, 0);
    pending.resolve({ data: { ...frame(10), pointCountA: 1, pointCountB: 0,
        syntheticPointCount: 1, referenceOnly: true, expiresAt: new Date(Date.now() + 5000).toISOString() } });
    await flush();
    assert.equal(h.latest().synthetic, 1);
    assert.match(h.element('completionStatus').textContent, /紫色合成点/);
    h.tick(1000); const late = h.take('/completed-frame');
    h.element('showCompletion').checked = false; h.element('showCompletion').handlers.change();
    late.resolve({ data: frame(11) }); await flush();
    assert.equal(h.latest().count, 0);
});

test('expired completion is not rendered even when radars are still fresh', async () => {
    const h = harness(); await ready(h);
    h.element('showCompletion').checked = true; h.element('showCompletion').handlers.change();
    h.take('/completed-frame').resolve({ data: { ...frame(10), referenceOnly: true,
        expiresAt: new Date(Date.now() - 1).toISOString() } });
    await flush(); assert.equal(h.latest().count, 0);
});

test('display surface is reference-only, cleared with frames, and unsupported lip is explained', async () => {
    const h = harness(); await ready(h);
    h.element('showCompletion').checked = true; h.element('showCompletion').handlers.change();
    const mesh = { encoding: 'xyz-triangles-v1', displayOnly: true, coordinates: [0, 0, 0, 1, 0, 0, 1, 0, 1] };
    h.take('/completed-frame').resolve({ data: { ...frame(10), pointCountA: 1, pointCountB: 0,
        syntheticPointCount: 1, referenceOnly: true, displayMesh: mesh,
        expiresAt: new Date(Date.now() + 5000).toISOString(),
        wallCompletion: { frontLip: { completed: false, reason: 'insufficient_support' } } } });
    await flush();
    assert.equal(h.latest().surface.length, 9);
    assert.match(h.element('completionStatus').textContent, /四壁连续显示面/);
    assert.match(h.element('completionStatus').textContent, /insufficient_support/);
    h.click('btnStopWatch');
    assert.equal(h.latest().count, 0);
    assert.ok(!h.latest().surface);
});

for (const coordinates of [[0, 1], Array(189).fill(0), [0, 0, 0, 1, 0, 0, 1, 0, Infinity]]) {
    test('malformed display surface clears frame: ' + coordinates.length, async () => {
        const h = harness(); await ready(h);
        h.element('showCompletion').checked = true; h.element('showCompletion').handlers.change();
        h.take('/completed-frame').resolve({ data: { ...frame(10), referenceOnly: true,
            expiresAt: new Date(Date.now() + 5000).toISOString(),
            displayMesh: { encoding: 'xyz-triangles-v1', displayOnly: true, coordinates } } });
        await flush(); assert.equal(h.latest().count, 0);
    });
}

test('unavailable status clears old points and invalidates an in-flight frame', async () => {
    const h = harness(); await ready(h);
    h.tick(1000); const delayed = h.take('/fused-frame');
    h.tick(2000); h.take('/status').resolve({ data: { preview: { available: false, unavailableReason: 'stale' } } });
    await flush(); assert.equal(h.latest().count, 0);
    delayed.resolve({ data: frame(10) }); await flush();
    assert.equal(h.latest().count, 0);
});

test('paused preview cannot be revived by a late frame or status', async () => {
    const h = harness(); await ready(h);
    h.tick(1000); h.tick(2000);
    h.click('btnStopWatch');
    h.take('/status').resolve({ data: { preview: { available: true } } });
    h.take('/fused-frame').resolve({ data: frame(10) }); await flush();
    assert.equal(h.latest().count, 0);
    assert.match(h.element('canvasOverlay').textContent, /暂停/);
});

test('slow frame and status requests remain single-flight', async () => {
    const h = harness();
    for (let i = 0; i < 10; i++) { h.tick(1000); h.tick(2000); }
    assert.equal(h.requests.length, 2);
    await ready(h);
});

test('worker restart clears old cloud and fits first new-session frame', async () => {
    const h = harness(); await ready(h);
    h.tick(2000); h.take('/status').resolve({ data: { preview: { available: true, sessionId: 'session-b' } } });
    await flush(); assert.equal(h.latest().count, 0);
    h.tick(1000); const request = h.take('/fused-frame');
    assert.match(request.url, /afterVersion=0$/);
    request.resolve({ data: frame(1, 'session-b') }); await flush();
    assert.equal(h.latest().count, 6); assert.equal(h.latest().fit, true);
});

test('connection errors and expired no-update replies clear the cloud', async () => {
    const h = harness(); await ready(h);
    h.tick(2000); h.take('/status').reject(new Error('offline')); await flush();
    assert.equal(h.latest().count, 0);
    h.tick(1000); h.take('/fused-frame').resolve({ data: frame(10) }); await flush();
    h.tick(1000); h.take('/fused-frame').resolve({ data: { updated: false, preview: { available: false } } });
    await flush(); assert.equal(h.latest().count, 0);
});

test('explicit acquisition rejection stops pending UI and preserves the retry key', async () => {
    const h = harness(); await ready(h);
    h.element('plateInput').value = 'test100';
    h.click('btnCapture');
    const attempt = h.take('/captures');
    assert.equal(attempt.options.body.confirmOverwriteEmptyProfile, true);
    const key = attempt.options.headers['Idempotency-Key'];
    attempt.reject({ code: 503, message: '采集安全门禁未满足：acquisition_unverified' });
    await flush();
    assert.match(h.element('progressSteps').innerHTML, /采集未启动/);
    assert.doesNotMatch(h.element('progressSteps').innerHTML, /class="progress-step active"/);
    assert.match(h.element('resultArea').textContent, /未保存 PCD、未进行计算/);
    assert.match(h.element('resultSummary').innerHTML, /采集未启动/);
    h.click('btnCloseModal');
    assert.equal(h.element('btnReopenProgress').style.display, 'block');
    h.click('btnReopenProgress');
    assert.match(h.element('captureModalMeta').textContent, /未收到流程编号/);
    assert.equal(h.element('btnCancelCapture').disabled, true);
    assert.equal(h.element('btnCapture').disabled, false);
    h.tick(1500);
    assert.equal(h.requests.filter(r => r.url.includes('/captures')).length, 0);
    h.click('btnCapture');
    assert.equal(h.take('/captures').options.headers['Idempotency-Key'], key);
});

for (const code of [0, 503, 409]) {
    test('unconfirmed dispatch does not claim no capture occurred: ' + code, async () => {
        const h = harness(); await ready(h);
        h.element('plateInput').value = 'test100';
        h.click('btnCapture');
        h.take('/captures').reject({ code, message: '无法核实本次请求' });
        await flush();
        assert.doesNotMatch(h.element('progressSteps').innerHTML, /采集未启动/);
        assert.match(h.element('resultArea').textContent, /网络异常不代表采集未执行/);
        assert.match(h.element('resultSummary').innerHTML, /请求未完成|未取得采集受理确认/);
        assert.doesNotMatch(h.element('resultSummary').innerHTML, /正在处理|已入库/);
        assert.equal(h.element('btnCancelCapture').disabled, true);
    });
}

test('capture dispatch and progress polling are single-flight', async () => {
    const h = harness(); await ready(h);
    h.element('plateInput').value = '川A12047';
    h.click('btnCapture'); h.click('btnCapture');
    assert.equal(h.requests.length, 1);
    h.take('/captures').resolve({ data: { workflowId: 'workflow-a', licensePlate: '川A12047' } }); await flush();
    for (let i = 0; i < 10; i++) h.tick(1500);
    assert.equal(h.requests.length, 1);
    h.take('/captures/workflow-a').resolve({ data: { workflowId: 'workflow-a', stage: 'completed', terminal: true } });
    await flush(); assert.equal(h.element('btnCapture').disabled, false);
});

async function beginCapture(h) {
    await ready(h);
    h.element('plateInput').value = '川A12047'; h.click('btnCapture');
    h.take('/captures').resolve({ data: { workflowId: 'workflow-a', licensePlate: '川A12047' } });
    await flush();
}

for (const stage of ['saved', 'queued', 'analyzing']) {
    test('capture cancellation ends when collection is finished: ' + stage, async () => {
        const h = harness(); await beginCapture(h);
        h.take('/captures/workflow-a').resolve({ data: { workflowId: 'workflow-a', stage, terminal: false } });
        await flush();
        assert.equal(h.element('btnCancelCapture').disabled, true);
        h.click('btnCancelCapture');
        assert.equal(h.requests.filter(r => r.url.includes('/cancel')).length, 0);
    });
}

test('capture cancellation shows the worker response while waiting for frames', async () => {
    const h = harness(); await beginCapture(h);
    h.take('/captures/workflow-a').resolve({ data: { workflowId: 'workflow-a', stage: 'waiting', terminal: false } });
    await flush();
    assert.equal(h.element('btnCancelCapture').disabled, false);
    h.click('btnCancelCapture');
    h.take('/cancel').resolve({ data: { success: true, message: '人工停止单次扫描' } }); await flush();
    assert.equal(h.element('captureBanner').textContent, '人工停止单次扫描');
});

function reviewProgress() {
    return { workflowId: 'workflow-a', stage: 'pending_review', terminal: true,
        message: 'PCD 已保存，融合计算完成，待人工审核（尚未入库）',
        persistenceState: 'saved', persistedCameras: ['CAM_004', 'CAM_005'],
        analysis: { stage: 'completed', hasDimensionPayload: false,
            measurementStatus: { reviewStatus: 'pending', calculationSucceeded: true },
            reviewMeasurements: { unit: 'm',
                dimensions: { vehicle: { length: 14.8588, width: 2.5611, height: 3.4433 },
                    carriage: { length: 5.9579, width: 2.5611, height: 2.6447 } },
                cabToTrailerFrontDist: 8.6853, vehicleTopToWeighbridgeHeight: 4.1424,
                modelLowerEdgeToWeighbridgeHeight: 1.4977, modelUpperEdgeToWeighbridgeHeight: 4.1424,
                trailerBottomToGroundHeight: null, ribCount: null, detectedRibCount: 0,
                measuredPointCount: 115401, syntheticPointCount: 15932,
                ribs: [], ribIntervals: [], highLowPlates: [], floorProfile: null,
                fields: { wires: 'detector_not_implemented', note: '<img src=x onerror=alert(1)>' }
            } } };
}

test('result summary preserves database evidence, review status, units and missing values', async () => {
    for (const stage of ['analyzing', 'completed', 'pending_review']) {
        const h = harness(); await beginCapture(h);
        const progress = reviewProgress(); progress.stage = stage;
        progress.terminal = stage !== 'analyzing';
        if (stage !== 'pending_review') progress.dimensions = { reviewStatus: 'pending',
            dimensions: { trailerLength: 1067.8, trailerWidth: null, trailerBodyHeight: 264.5 } };
        h.take('/captures/workflow-a').resolve({ data: progress }); await flush();
        const summary = h.element('resultSummary').innerHTML;
        if (stage === 'completed') {
            assert.match(summary, /已入库.*待人工审核/);
            assert.match(summary, /1067\.8 cm/);
            assert.match(summary, /车厢宽（采用值）<\/span><strong>—/);
        } else {
            assert.doesNotMatch(summary, /已入库/);
            assert.match(summary, stage === 'pending_review' ? /5\.958 m/ : /入库待确认/);
        }
        if (stage === 'completed') {
            h.click('btnCapture');
            assert.doesNotMatch(h.element('resultSummary').innerHTML, /1067\.8|已入库/);
        }
    }
});

test('pending review values are only shown when the review unit is metres', async () => {
    for (const unit of ['cm', undefined]) {
        const h = harness(); await beginCapture(h);
        const progress = reviewProgress();
        progress.analysis.reviewMeasurements.dimensions.carriage = { length: 540, width: 226, height: 264 };
        if (unit === undefined) delete progress.analysis.reviewMeasurements.unit;
        else progress.analysis.reviewMeasurements.unit = unit;
        h.take('/captures/workflow-a').resolve({ data: progress }); await flush();
        const summary = h.element('resultSummary').innerHTML;
        assert.doesNotMatch(summary, /540\.000 m|540 m/, 'unit=' + unit);
    }
});

test('partial review with a nested non-metre unit is not shown as metres', async () => {
    for (const level of ['dimensions', 'carriage', 'vehicle']) {
        const h = harness(); await beginCapture(h);
        const progress = reviewProgress();
        const review = progress.analysis.reviewMeasurements;
        review.partial = true; review.calculationSucceeded = true; review.reviewStatus = 'pending';
        review.dimensions.carriage = { length: 540, width: null, height: null };
        (level === 'dimensions' ? review.dimensions : review.dimensions[level]).unit = 'cm';
        h.take('/captures/workflow-a').resolve({ data: progress }); await flush();
        assert.doesNotMatch(h.element('resultSummary').innerHTML, /540\.000 m|540 m/, 'level=' + level);
    }
});

test('manual-review results include carriage inner length and stop database polling', async () => {
    const h = harness(); await beginCapture(h);
    h.take('/captures/workflow-a').resolve({ data: { stage: 'saved', terminal: false } }); await flush();
    h.tick(1500);
    h.take('/captures/workflow-a').resolve({ data: reviewProgress() }); await flush();
    const html = h.element('resultArea').innerHTML;
    // 内长独立呈现，不能要求操作员把含缺测值的拉筋间距加起来猜总长。
    for (const label of ['车头到车厢前内壁', '车厢内长', '车厢宽（采用值）', '车厢内高', '前端底板距磅台', '拉筋数量']) {
        assert.ok(html.includes(label), `missing card: ${label}`);
    }
    assert.match(html, /8\.685 m/);          // 车头到车厢前内壁
    assert.match(html, /车厢内长<\/div><div class="dim-value">5\.958 m/);
    assert.match(html, /2\.561 m/);          // 车厢宽（外沿）
    assert.match(html, /2\.645 m/);          // 车厢内高
    assert.match(html, /模型\/计算值，尚未经人工核验/);
    // 排障信息不属于这里：融合质量、点数、PCD 路径都不该出现在操作员的结果区。
    for (const noise of ['融合匹配率', '融合点数', 'PCD 落盘', '四壁新增补点数', '拉筋明细']) {
        assert.ok(!html.includes(noise), `unexpected diagnostic on result panel: ${noise}`);
    }
    assert.match(h.element('progressSteps').innerHTML, /融合与计算完成.*待人工审核/);
    assert.equal(h.element('captureBanner').className, 'banner warn');
    assert.equal(h.element('btnCapture').disabled, false);
    h.tick(1500); await flush();
    assert.equal(h.requests.filter(r => r.url.includes('/captures/')).length, 0);
});

test('rib gaps and high-low plates render from the stored record', async () => {
    const h = harness(); await beginCapture(h);
    const progress = reviewProgress();
    progress.stage = 'completed';
    progress.dimensions = { dimensions: {
        trailerLength: 1107.2, trailerWidth: 242.0, trailerBodyHeight: 180.7,
        cabToTrailerFrontDist: 464.5, trailerBottomToGroundHeight: 155.3,
        highLowPlateHeight: 5.8,
        ribs: [{ ribId: 1, distToPrev: 208.0 }, { ribId: 2, distToPrev: 222.1 },
               { ribId: 3, distToPrev: 229.2 }, { ribId: 4, distToPrev: 229.1 },
               { ribId: 5, distToPrev: 218.8 }],
        highLowPlates: [{ plateId: 1, highestToGround: 161.1, lowestToGround: 155.3 }] } };
    h.take('/captures/workflow-a').resolve({ data: progress }); await flush();
    const html = h.element('resultArea').innerHTML;

    assert.match(html, /464\.5 cm/);                 // 车头到车厢前内壁
    // Analyzer 输出 N+1 段（exporters.py：N 根拉筋输出 N+1 段），
    // 所以 5 段对应 4 根 —— 把段数当根数会多报一根。
    assert.match(html, /拉筋数量<\/div><div class="dim-value">4 根/);
    assert.ok(!html.includes('根 ⚠'), '2~6 根属正常范围，不应告警');
    // 顺序从前壁起算：首段前壁→筋1，末段筋4→后壁。写反会让现场按错误次序对尺。
    assert.match(html, /前壁→筋1/);
    assert.match(html, /筋4→后壁/);
    assert.ok(!html.includes('后壁→筋1'), '拉筋方向不得反向');
    assert.match(html, /208\.0 cm/);
    assert.match(html, /218\.8 cm/);
    assert.match(html, /161\.1 cm/);                 // 高低板最高点
    assert.match(html, /落差 5\.8 cm/);
    assert.ok(!html.includes('plate-warn'), '1 组高低板不应告警');
});

test('rib and plate counts outside the expected range are flagged', async () => {
    const h = harness(); await beginCapture(h);
    const progress = reviewProgress();
    progress.stage = 'completed';
    progress.dimensions = { dimensions: {
        trailerLength: 1107.2, trailerWidth: 242.0, trailerBodyHeight: 180.7,
        cabToTrailerFrontDist: 464.5, trailerBottomToGroundHeight: 155.3,
        highLowPlateHeight: 5.8,
        ribs: Array.from({ length: 10 }, (_, i) => ({ ribId: i + 1, distToPrev: 110 })),
        highLowPlates: Array.from({ length: 4 }, (_, i) => ({
            plateId: i + 1, highestToGround: 161.1, lowestToGround: 155.3 })) } };
    h.take('/captures/workflow-a').resolve({ data: progress }); await flush();
    const html = h.element('resultArea').innerHTML;

    // 现场拉筋 2~6 根、高低板最多 3 组；超出范围必须让操作员看见，而不是照单全收。
    assert.match(html, /9 根 ⚠/);   // 10 段 = 9 根，超出 2~6
    assert.match(html, /超出常见的 3 组/);
});

test('stored record takes precedence over review candidates and uses centimetres', async () => {
    const h = harness(); await beginCapture(h);
    const progress = reviewProgress(); progress.stage = 'completed';
    progress.dimensions = { dimensions: {
        trailerLength: 1173.7, trailerWidth: 258.6, trailerBodyHeight: 203.3 } };
    h.take('/captures/workflow-a').resolve({ data: progress }); await flush();
    const html = h.element('resultArea').innerHTML;
    assert.match(html, /258\.6 cm/);
    assert.match(html, /203\.3 cm/);
    assert.match(html, /车厢内长<\/div><div class="dim-value">1173\.7 cm/);
    // 已入库记录在场时不得混入未入库的米制计算值。
    assert.doesNotMatch(html, /2\.561 m|模型\/计算值/);
    assert.equal(h.element('captureBanner').className, 'banner ok');
});

for (const workerAvailable of [true, false]) {
    test('stored review shows database dimensions and terminates, worker available=' + workerAvailable, async () => {
        const h = harness(); await beginCapture(h);
        const progress = reviewProgress(); progress.stage = 'completed';
        progress.message = '已入库，待人工审核';
        progress.dimensions = { reviewStatus: 'pending', dimensions: {
            trailerLength: 595.8, trailerWidth: 256.1, trailerBodyHeight: 264.5,
            cabToTrailerFrontDist: 448.0, trailerBottomToGroundHeight: null } };
        progress.emptyVehiclePointCloudUrl = '/api/empty-vehicle-profiles/plate/pointcloud';
        if (!workerAvailable) progress.analysis = null;
        h.take('/captures/workflow-a').resolve({ data: progress }); await flush();
        const html = h.element('resultArea').innerHTML;
        assert.match(html, /448\.0 cm/);
        assert.match(html, /256\.1 cm/);
        assert.match(html, /264\.5 cm/);
        assert.match(html, /车厢内长<\/div><div class="dim-value">595\.8 cm/);
        assert.doesNotMatch(html, /内长测线/);
        // 底板缺测时留空，不能借用别的字段顶上。
        assert.match(html, /前端底板距磅台<\/div><div class="dim-value">—/);
        assert.match(html, /模型\/计算值，尚未经人工核验/);
        assert.match(h.element('progressSteps').innerHTML, /已入库，待人工审核/);
        assert.equal(h.element('captureBanner').className, 'banner warn');
        assert.equal(h.element('resultArea').children.length, 1);
        h.tick(1500); await flush();
        assert.equal(h.requests.filter(r => r.url.includes('/captures/')).length, 0);
    });
}

function sideLengthProgress() {
    return {
        workflowId: 'workflow-a', stage: 'completed', terminal: true, message: '已入库，待人工审核',
        dimensions: { reviewStatus: 'pending', dimensions: {
            trailerLength: 1070.0, trailerWidth: 230.0, trailerBodyHeight: 160.0,
            ribs: [{ ribId: 1, distToPrev: null }, { ribId: 2, distToPrev: null },
                   { ribId: 3, distToPrev: 370.0 }] } },
        analysis: { stage: 'completed', measurementStatus: { reviewStatus: 'pending',
            lengthMeasurement: { valid: true,
                definition: 'same_side_lower_band_longitudinal_projection',
                heightAboveFloorM: [0.15, 0.35], inwardBandM: [0.06, 0.10],
                note: '<img src=x onerror=alert(1)>', ribPositionsValid: false } } }
    };
}

test('side length uses stored centimetres, explains its measurement line and preserves unknown gaps', async () => {
    const h = harness(); await beginCapture(h);
    h.take('/captures/workflow-a').resolve({ data: sideLengthProgress() }); await flush();
    const html = h.element('resultArea').innerHTML;
    assert.match(html, /车厢内长<\/div><div class="dim-value">1070\.0 cm/);
    assert.match(html, /同侧下部，底板以上 15～35 cm，侧壁参考面向内 6～10 cm/);
    assert.match(html, /前壁→筋1<\/i>缺测/);
    assert.match(html, /筋1→筋2<\/i>缺测/);
    assert.match(html, /筋2→后壁<\/i>370\.0 cm/);
    assert.match(html, /拉筋数量<\/div><div class="dim-value">2 根/);
    assert.match(html, /部分拉筋间距缺测，不以 0 代替/);
    assert.doesNotMatch(html, /<img|onerror/);
});

for (const unavailable of ['worker_absent', 'analysis_failed', 'invalid_height_range', 'different_method']) {
    test('stored length does not claim a new measurement line without valid diagnostics: ' + unavailable, async () => {
        const h = harness(); await beginCapture(h);
        const progress = sideLengthProgress();
        if (unavailable === 'worker_absent') progress.analysis = null;
        else if (unavailable === 'analysis_failed') progress.analysis.stage = 'failed';
        else if (unavailable === 'invalid_height_range') {
            progress.analysis.measurementStatus.lengthMeasurement.heightAboveFloorM = ['<img src=x>', 0.35];
        } else progress.analysis.measurementStatus.lengthMeasurement.definition = 'legacy_wall_fit';
        h.take('/captures/workflow-a').resolve({ data: progress }); await flush();
        const html = h.element('resultArea').innerHTML;
        assert.match(html, /车厢内长<\/div><div class="dim-value">1070\.0 cm/);
        assert.doesNotMatch(html, /内长测线|15～35|6～10|<img/);
        assert.match(html, /前壁→筋1<\/i>缺测/);
    });
}

test('missing database length is not replaced by a stale review candidate', async () => {
    const h = harness(); await beginCapture(h);
    const progress = reviewProgress();
    progress.stage = 'completed';
    progress.dimensions = { reviewStatus: 'pending', dimensions: { trailerLength: null } };
    h.take('/captures/workflow-a').resolve({ data: progress }); await flush();
    const html = h.element('resultArea').innerHTML;
    assert.match(html, /车厢内长<\/div><div class="dim-value">—/);
    assert.doesNotMatch(html, /5\.958 m/);
});

test('stored partial dimensions do not invent rib or plate results or require review', async () => {
    const h = harness(); await beginCapture(h);
    const progress = partialProgress();
    progress.stage = 'completed';
    progress.message = '计算完成，尺寸已写入数据库';
    progress.analysis.hasDimensionPayload = true;
    progress.analysis.measurementStatus = { valid: false, partial: true, calculationSucceeded: true };
    progress.dimensions = { dimensions: {
        trailerLength: null, trailerWidth: 238.8, trailerBodyHeight: 180.5,
        trailerBottomToGroundHeight: null, ribs: [], highLowPlates: []
    } };
    h.take('/captures/workflow-a').resolve({ data: progress }); await flush();
    const html = h.element('resultArea').innerHTML;
    assert.match(html, /车厢宽（采用值）<\/div><div class="dim-value">238\.8 cm/);
    assert.match(html, /车厢内长<\/div><div class="dim-value">—/);
    assert.match(html, /拉筋数量<\/div><div class="dim-value">未确认/);
    assert.match(html, /未取得有效明细，不能据此断言没有高低板/);
    assert.doesNotMatch(html, /0 根|未检出|待人工审核|尚未经人工核验/);
});

for (const missing of ['trailerLength', 'trailerWidth', 'trailerBodyHeight']) {
    test('stored partial record keeps rib count unknown after worker history is lost: ' + missing, async () => {
        const h = harness(); await beginCapture(h);
        const progress = { workflowId: 'workflow-a', stage: 'completed', terminal: true,
            message: '计算完成，尺寸已写入数据库', analysis: null,
            dimensions: { dimensions: { trailerLength: 1124.1, trailerWidth: 238.3,
                trailerBodyHeight: 181.3, ribs: [], highLowPlates: [] } } };
        progress.dimensions.dimensions[missing] = null;
        h.take('/captures/workflow-a').resolve({ data: progress }); await flush();
        const html = h.element('resultArea').innerHTML;
        assert.match(html, /拉筋数量<\/div><div class="dim-value">未确认/);
        assert.doesNotMatch(html, /0 根|未检出/);
    });
}

test('legacy review gap nulls remain missing while a measured zero remains zero', async () => {
    const h = harness(); await beginCapture(h);
    const progress = reviewProgress();
    progress.analysis.reviewMeasurements.ribIntervals = [null, 0, 2.3];
    h.take('/captures/workflow-a').resolve({ data: progress }); await flush();
    const html = h.element('resultArea').innerHTML;
    assert.match(html, /车厢内长<\/div><div class="dim-value">5\.958 m/);
    assert.match(html, /前壁→筋1<\/i>缺测/);
    assert.match(html, /筋1→筋2<\/i>0\.000 m/);
    assert.match(html, /筋2→后壁<\/i>2\.300 m/);
});

test('read-only viewer UI script has a new cache version', () => {
    const html = fs.readFileSync(path.join(scriptRoot, '../live.html'), 'utf8');
    assert.match(html, /js\/radar-live\.js\?v=20261106-result/);
});

test('failed analysis cannot show stale review candidates as completed calculations', async () => {
    const h = harness(); await beginCapture(h);
    const progress = reviewProgress(); progress.stage = 'analysis_failed'; progress.analysis.stage = 'failed';
    h.take('/captures/workflow-a').resolve({ data: progress }); await flush();
    assert.doesNotMatch(h.element('resultArea').innerHTML, /dim-card|2\.561 m/);
    assert.equal(h.element('captureBanner').className, 'banner error');
});

test('capture flow runs in the modal, not in the side panel', async () => {
    const h = harness(); await ready(h);
    assert.equal(h.element('captureModal').style.display, 'none');

    h.element('plateInput').value = '川A12047';
    h.click('btnCapture');
    // 请求刚发出、还没有流程编号时模态框就该在了：采集已经开始，
    // 进度必须从第一刻起就有地方显示。
    assert.equal(h.element('captureModal').style.display, 'flex');
    assert.match(h.element('captureModalMeta').textContent, /川A12047/);

    h.take('/captures').resolve({ data: { workflowId: 'workflow-a', licensePlate: '川A12047' } });
    await flush();
    assert.match(h.element('captureModalMeta').textContent, /workflow-a/);
});

test('missing plate is rejected in place without opening the modal', async () => {
    const h = harness(); await ready(h);
    h.element('plateInput').value = '   ';

    h.click('btnCapture');

    // 弹出模态框只会盖住要填的输入框；这一步还没有向雷达发出任何请求。
    assert.equal(h.element('captureModal').style.display, 'none');
    assert.equal(h.element('captureHint').className, 'banner error');
    assert.equal(h.requests.filter(r => r.url.includes('/captures')).length, 0);
});

test('closing the modal mid-capture keeps polling and offers a way back', async () => {
    const h = harness(); await beginCapture(h);
    h.take('/captures/workflow-a').resolve({ data: { stage: 'saved', terminal: false } });
    await flush();

    h.click('btnCloseModal');

    assert.equal(h.element('captureModal').style.display, 'none');
    // 关掉窗口不等于取消采集：它已经在写业务数据，轮询必须继续。
    assert.equal(h.element('btnReopenProgress').style.display, 'block');
    h.tick(1500);
    h.take('/captures/workflow-a').resolve({ data: { stage: 'queued', terminal: false } });
    await flush();

    h.click('btnReopenProgress');
    assert.equal(h.element('captureModal').style.display, 'flex');
    assert.equal(h.element('btnReopenProgress').style.display, 'none');
});

test('terminal capture can reopen its details without resuming acquisition', async () => {
    const h = harness(); await beginCapture(h);
    h.take('/captures/workflow-a').resolve({ data: reviewProgress() });
    await flush();

    h.click('btnCloseModal');

    assert.equal(h.element('btnReopenProgress').style.display, 'block');
    const before = h.requests.length;
    h.click('btnReopenProgress');
    assert.equal(h.element('captureModal').style.display, 'flex');
    h.tick(1500);
    assert.equal(h.requests.length, before);
});

test('saved binary PCD is shown as non-live and resume clears it', async () => {
    const h = harness(); await ready(h);
    h.element('plateInput').value = '川A12047'; h.click('btnCapture');
    h.take('/captures').resolve({ data: { workflowId: 'workflow-a', licensePlate: '川A12047' } }); await flush();
    h.take('/captures/workflow-a').resolve({ data: { workflowId: 'workflow-a', stage: 'completed', terminal: true,
        emptyVehiclePointCloudUrl: '/api/empty-vehicle-profiles/plate/pointcloud?expectedWorkflowId=workflow-a' } });
    await flush();
    h.element('resultArea').children[0].handlers.click();
    const header = Buffer.from('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 2\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 2\nDATA binary\n');
    const bytes = Buffer.concat([header, Buffer.from(new Float32Array([1, 2, 3, 4, 5, 6]).buffer)]);
    h.take('/pointcloud?').resolve(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength));
    await flush(); assert.equal(h.latest().count, 6, h.element('previewBanner').textContent);
    assert.match(h.element('canvasOverlay').textContent, /非实时/);
    h.click('btnStartWatch'); assert.equal(h.latest().count, 0);
});

function partialProgress() {
    return { workflowId: 'workflow-a', stage: 'pending_review', terminal: true,
        message: '部分计算结果，尚未入库', analysis: { stage: 'completed', hasDimensionPayload: false,
            measurementStatus: { reviewStatus: 'pending', calculationSucceeded: true,
                reason: 'inner_wall_boundaries_unverified' },
            reviewMeasurements: { unit: 'm', partial: true,
                dimensions: { vehicle: { length: 15.7, width: 2.49, height: 2.4 },
                    carriage: { length: null, width: 2.387602, height: 1.805 } },
                ribCount: null, ribIntervals: null, wireCount: null, highLowPlates: null,
                referenceDimensions: { carriage: { length: 10.8, width: 2.49 } },
                fields: { 'dimensions.carriage.length': { reason: 'inner_wall_boundaries_unverified' },
                    wireCount: { reason: 'detector_not_implemented', source: '<img src=x onerror=alert(1)>' } }
            } } };
}

test('partial results render available dimensions and explain missing fields without inventing zero counts', async () => {
    const h = harness(); await beginCapture(h);
    h.take('/captures/workflow-a').resolve({ data: partialProgress() }); await flush();
    const html = h.element('resultArea').innerHTML;
    assert.match(html, /车厢内宽<\/div><div class="dim-value">2\.388 m/);
    assert.match(html, /车厢内长<\/div><div class="dim-value">未算出/);
    assert.match(html, /拉筋数量<\/div><div class="dim-value">未确认/);
    assert.match(html, /15\.700 m/);
    assert.match(html, /非离地高度/);
    assert.match(html, /前后内壁未确认/);
    assert.match(html, /尚未实现该项检测/);
    assert.match(html, /不是已确认的净内长或净内宽/);
    assert.match(html, /不能据此断言没有高低板/);
    assert.doesNotMatch(html, /0 根|<img/);
    h.tick(1500); await flush();
    assert.equal(h.requests.filter(r => r.url.includes('/captures/')).length, 0);
});

test('legacy completed-without-dimensions exposes only proven current width and empty wall depth', async () => {
    const h = harness(); await beginCapture(h);
    const progress = { stage: 'completed_without_dimensions', terminal: true,
        message: '如需尺寸请在空车时重新采集', analysis: { stage: 'completed',
            measurementStatus: { reason: 'inner_wall_boundaries_unverified', widthMeasurement: {
                valid: true, adoptedSource: 'observed_inner_side_faces', widthM: 2.387602 } },
            loadStatus: { status: 'Empty', metrics: { wallDepth: 1.805 } } } };
    h.take('/captures/workflow-a').resolve({ data: progress }); await flush();
    const html = h.element('resultArea').innerHTML;
    assert.match(html, /2\.388 m|1\.805 m/);
    assert.match(html, /车头到车厢前内壁<\/div><div class="dim-value">未算出/);
    assert.match(html, /车厢内长<\/div><div class="dim-value">未算出/);
    assert.doesNotMatch(h.element('captureBanner').textContent, /空车时重新采集/);
    assert.match(h.element('captureBanner').textContent, /未生成完整尺寸入库记录/);
});

for (const load of ['Loaded', 'Uncertain']) {
    test('legacy partial does not relabel cargo height as inner height: ' + load, async () => {
        const h = harness(); await beginCapture(h);
        h.take('/captures/workflow-a').resolve({ data: {
            stage: 'completed_without_dimensions', terminal: true, analysis: { stage: 'completed',
                loadStatus: { status: load, metrics: { wallDepth: 9.123 } },
                measurementStatus: { widthMeasurement: { valid: false,
                    adoptedSource: 'observed_outer_span_fallback', widthM: 8.765 } } }
        } }); await flush();
        const html = h.element('resultArea').innerHTML;
        assert.match(html, /缺测 \/ 未执行/);
        assert.doesNotMatch(html, /9\.123|8\.765|0 根/);
    });
}

test('database record still wins over partial review and old worker failure', async () => {
    const h = harness(); await beginCapture(h);
    const progress = partialProgress(); progress.stage = 'completed';
    progress.dimensions = { dimensions: { trailerLength: 1124.9, trailerWidth: 239.3, trailerBodyHeight: 180.9 } };
    h.take('/captures/workflow-a').resolve({ data: progress }); await flush();
    const html = h.element('resultArea').innerHTML;
    assert.match(html, /1124\.9 cm/);
    assert.doesNotMatch(html, /2\.388 m|尚未入库|参考候选跨度/);
});

test('failed analysis cannot display stale partial candidates or diagnostics', async () => {
    const h = harness(); await beginCapture(h);
    const progress = partialProgress(); progress.stage = 'analysis_failed'; progress.analysis.stage = 'failed';
    h.take('/captures/workflow-a').resolve({ data: progress }); await flush();
    assert.doesNotMatch(h.element('resultArea').innerHTML, /2\.388|15\.700|dim-card/);
});

test('workflow link and refresh are GET-only and never resume or cancel acquisition', async () => {
    const h = harness('?workflowId=sess_20260926112617554_f0553abe295a'); await ready(h);
    const request = h.take('/captures/sess_20260926112617554_f0553abe295a');
    assert.equal(request.options, undefined);
    request.resolve({ data: { workflowId: 'sess_20260926112617554_f0553abe295a',
        licensePlate: '宁A227351', stage: 'unknown', terminal: false } }); await flush();
    assert.equal(h.element('captureModal').style.display, 'flex');
    assert.match(h.element('captureModalMeta').textContent, /宁A227351/);
    assert.equal(h.element('btnCancelCapture').disabled, true);
    h.click('btnCancelCapture');
    h.tick(1500); await flush();
    assert.equal(h.requests.filter(r => r.url.includes('/captures')).length, 0);
    h.click('btnRefreshResult'); h.click('btnRefreshResult');
    assert.equal(h.requests.filter(r => r.url.includes('/captures')).length, 1);
    h.take('/captures/sess_').resolve({ data: partialProgress() }); await flush();
    h.click('btnRefreshResult');
    assert.equal(h.take('/captures/sess_').options, undefined);
});

for (const workflow of ['', '../other', 'abc/def', '<script>', 'a'.repeat(129)]) {
    test('invalid workflow link is rejected without any capture request: ' + workflow.slice(0, 20), async () => {
        const h = harness('?workflowId=' + encodeURIComponent(workflow)); await ready(h);
        assert.equal(h.requests.filter(r => r.url.includes('/captures')).length, 0);
        assert.match(h.element('captureHint').textContent, /编号格式无效/);
    });
}


test('completed calculation without database row displays exact current payload and stops waiting', async () => {
    const h = harness(); await beginCapture(h);
    h.take('/captures/workflow-a').resolve({ data: { workflowId: 'workflow-a',
        stage: 'completed_unpersisted', terminal: true, message: '计算完成，尚未查到数据库记录',
        analysis: { stage: 'completed', dimensionPayload: { dimensions: {
            trailerLength: 1000, trailerWidth: 240, trailerBodyHeight: 200,
            cabToTrailerFrontDist: 450, trailerBottomToGroundHeight: null
        }, actualRibCount: 2, ribs: [{distToPrev: 300}, {distToPrev: null}, {distToPrev: 200}] } } } });
    await flush();
    assert.match(h.element('resultArea').innerHTML, /1000\.0 cm/);
    assert.match(h.element('resultArea').innerHTML, /200\.0 cm/);
    assert.match(h.element('resultArea').innerHTML, /300\.0 cm/);
    assert.match(h.element('resultArea').innerHTML, /缺测/);
    assert.match(h.element('resultSummary').innerHTML, /未入库/);
    assert.doesNotMatch(h.element('resultSummary').innerHTML, />已入库</);
    assert.equal(h.element('btnCapture').disabled, false);
    h.tick(1500); assert.equal(h.requests.filter(r => r.url.includes('/captures')).length, 0);
    h.click('btnRefreshResult');
    h.take('/captures/workflow-a').resolve({data: {workflowId: 'workflow-a', stage: 'completed', terminal: true,
        dimensions: {dimensions: {trailerLength: 999, trailerWidth: 240, trailerBodyHeight: 200}}}});
    await flush();
    assert.match(h.element('resultArea').innerHTML, /999\.0 cm/);
    assert.match(h.element('resultSummary').innerHTML, />已入库</);
});

test('changed plate after failed dispatch gets its own key while same plate retry keeps the key', async () => {
    const h = harness(); h.element('plateInput').value = '宁A25771';
    h.click('btnCapture'); const first = h.take('/captures');
    first.reject({code:502,message:'Worker不可达'}); await flush();
    h.element('plateInput').value = ' 宁a25771 ';
    h.click('btnCapture'); const same = h.take('/captures');
    assert.equal(same.options.body.idempotencyKey, first.options.body.idempotencyKey);
    same.reject({code:502,message:'Worker不可达'}); await flush();
    h.element('plateInput').value = '宁B89815';
    h.click('btnCapture'); const changed = h.take('/captures');
    assert.notEqual(changed.options.body.idempotencyKey, first.options.body.idempotencyKey);
    assert.equal(changed.options.body.licensePlate, '宁B89815');
});

test('missing Worker task ends progress without reposting an accepted capture', async () => {
    const h = harness(); await beginCapture(h);
    h.take('/captures/workflow-a').resolve({data: {workflowId:'workflow-a', stage:'progress_unavailable',
        terminal:true, message:'Worker中未找到本次任务，未查到数据库结果'}}); await flush();
    assert.match(h.element('captureBanner').textContent, /未找到本次任务/);
    assert.equal(h.element('btnCapture').disabled,false);
    h.advance(60000); h.tick(1500);
    assert.equal(h.requests.filter(r => r.url.includes('/captures')).length,0);
});


test('progress query failure stops waiting and refresh resumes the original workflow without POST', async () => {
    const h = harness(); await beginCapture(h);
    h.take('/captures/workflow-a').reject({code:502,message:'WebApi查询失败'}); await flush();
    assert.equal(h.element('btnCapture').disabled,false);
    assert.match(h.element('captureBanner').textContent,/刷新本次结果/);
    h.tick(1500); assert.equal(h.requests.filter(r => r.url.includes('/captures')).length,0);
    h.click('btnRefreshResult');
    h.take('/captures/workflow-a').resolve({data:{stage:'analyzing',terminal:false}}); await flush();
    h.tick(1500);
    assert.equal(h.take('/captures/workflow-a').options,undefined);
    assert.equal(h.requests.filter(r => r.options?.method==='POST').length,0);
});
