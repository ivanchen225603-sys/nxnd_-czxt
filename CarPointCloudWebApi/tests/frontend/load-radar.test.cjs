// 四路演示样式接入真实后台后的通道、异步结果与历史隔离测试。
const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const viewerRoot = path.resolve(__dirname, '../../wwwroot/pointcloud-viewer');
const flush = () => new Promise(resolve => setImmediate(resolve));
const plain = value => JSON.parse(JSON.stringify(value));
const realApiFixture = process.env.LOAD_RADAR_E2E_FIXTURE;

function harness() {
    const nodes = new Map(), requests = [], timers = [], events = {}, clouds = [];
    let initialize, now = Date.now();
    class Clock extends Date { static now() { return now; } }
    function create(tag = 'div') {
        const node = { tag, children: [], handlers: {}, attributes: {}, value: '', disabled: false, hidden: false, open: false, _text: '', className: '', parent: null,
            addEventListener(name, fn) { this.handlers[name] = fn; },
            append(...items) { for (const item of items) { item.remove(); item.parent = this; this.children.push(item); } },
            remove() { if (this.parent) this.parent.children = this.parent.children.filter(n => n !== this); this.parent = null; },
            replaceChildren(...items) { for (const c of [...this.children]) c.remove(); this._text = ''; this.append(...items); },
            setAttribute(name, value) { this.attributes[name] = value; },
            showModal() { this.open = true; }, close() { this.open = false; this.handlers.close?.(); }
        };
        Object.defineProperty(node, 'textContent', {
            get() { return this._text + this.children.map(c => c.textContent).join(''); },
            set(value) { this._text = String(value); for (const c of [...this.children]) c.remove(); }
        });
        return node;
    }
    const html = fs.readFileSync(path.join(viewerRoot, 'load-radar.html'), 'utf8');
    for (const match of html.matchAll(/\bid="([^"]+)"/g)) nodes.set(match[1], create());
    const get = id => nodes.get(id) || null;
    const context = vm.createContext({
        console, URLSearchParams, Date: Clock, Map, Set,
        document: { hidden: false, getElementById: get, createElement: create,
            addEventListener(name, fn) { if (name === 'DOMContentLoaded') initialize = fn; else events['document:' + name] = fn; } },
        window: { addEventListener(name, fn) { events[name] = fn; } },
        setInterval(fn) { const timer = { fn, active: true }; timers.push(timer); return timer; },
        clearInterval(timer) { timer.active = false; },
        API: { request(url, options) { return new Promise((resolve, reject) => requests.push({ url, options, resolve, reject })); } },
        LoadRadarCloud: { create(canvas) {
            const cloud = { canvas, points: [], latest: [], bounds: null, frozen: false, disposed: false, fits: 0,
                setPoints(points) { this.latest = points; if (!this.frozen) this.points = points; },
                setBounds(bounds) { this.bounds = bounds; }, clear() { this.points = []; this.latest = []; this.frozen = false; },
                setFrozen(value) { this.frozen = value; if (!value) this.points = this.latest; }, reset() {}, fit() { this.fits++; },
                dispose() { this.disposed = true; }
            }; clouds.push(cloud); return cloud;
        } }
    });
    vm.runInContext(fs.readFileSync(path.join(viewerRoot, 'js/load-radar.js'), 'utf8'), context); initialize();
    function find(node, cls) { if (node.className.split(' ').includes(cls)) return node; for (const child of node.children) { const found = find(child, cls); if (found) return found; } }
    return { get, requests, clouds, events, document: context.document,
        take(part) { const index = requests.findIndex(r => r.url.includes(part)); assert.notEqual(index, -1, '缺少请求 ' + part); return requests.splice(index, 1)[0]; },
        tick(ms = 4000) { now += ms; timers.filter(t => t.active).forEach(t => t.fn()); },
        click(id) { return get(id).handlers.click(); },
        submit() { get('loadHistoryForm').handlers.submit({ preventDefault() {} }); },
        view(index = 0) { get('loadHistoryRows').children[index].children[6].children[0].handlers.click(); },
        select(index = -1) { get('laneTabs').children[index + 2].handlers.click(); },
        card(index = 0) {
            const node = get('loadLanes').children[index];
            return { node, status: find(node, 'big'), note: find(node, 'sub'), plate: find(node, 'result-plate'),
                cargo: find(node, 'result-cargo'), volume: find(node, 'result-volume'), time: find(node, 'result-time'), freeze: find(node, 'freeze'),
                canvasStatus: find(node, 'canvas-status'), badge: find(node, 'badge'), estimate: find(node, 'volume-estimate') };
        },
        log(index = 0) {
            const node = get('laneLogs').children[0];
            return { node, title: find(node, 'lane-log-header').children[0], live: find(node, 'lane-log-live'),
                feedback: find(node, 'lane-log-feedback'), rows: find(node, 'lane-log-rows').children,
                more: find(node, 'log-more'), refresh: find(node, 'log-refresh') };
        },
        runtime() {
            const node = get('laneLogs').children[0];
            return { rows: find(node, 'runtime-log-rows').children,
                feedback: find(node, 'runtime-log-feedback'), heading: find(node, 'runtime-log-heading') };
        }
    };
}
function lane(nodeId = 'N1', channelId = 'CH1', id = null) {
    return { nodeId, channelId, name: '通道-' + channelId, online: true,
        state: { online: true, cloud: [[1, 2, 3]], roi: [[-2, -10, -6], [2, 8, 0]], stability: { state: 2 },
            current_result: id ? { measurement_id: id } : null,
            last_result: { empty: false, volume: { ok: true, volume_m3: 999 }, reason: '旧结果不可冒充本车' } } };
}
function record(id = 'm1', extra = {}) {
    return { binding: { licensePlate: '宁A96523' }, publication: null, measurement: {
        measurementId: id, nodeId: 'N1', channelId: 'CH1', measuredAt: '2026-09-18T03:00:00Z',
        verdict: 'loaded', frameDegraded: false, volumeStatus: 'success', volumeM3: 52.125, points: [[4, 5, 6]], ...extra
    } };
}

if (realApiFixture) test('真实测试库 API 回执在管理页按原字段显示并继续刷新', async () => {
    const fixture = JSON.parse(fs.readFileSync(realApiFixture, 'utf8'));
    const row = fixture.measurement, m = row.measurement;
    assert.equal(fixture.historyMatch.measurement.measurementId, m.measurementId);
    const h = harness(), current = lane(m.nodeId, m.channelId, m.measurementId);
    current.state.cloud = m.points;
    await state(h, [current]);
    h.take('/measurements/' + m.measurementId).resolve({ data: row });
    await flush();
    assert.equal(h.card().status.textContent, '空车');
    assert.equal(h.card().plate.textContent, row.binding.licensePlate);
    assert.equal(h.card().volume.textContent, '--');
    assert.equal(h.card().time.textContent,
        new Date(m.measuredAt).toLocaleString('zh-CN', { hour12: false }));
    assert.equal(h.clouds[0].points.length, m.points.length);
    h.tick(1500); await state(h, [current]);
    assert.equal(h.card().plate.textContent, row.binding.licensePlate);
    assert.equal(h.requests.filter(r => r.url.includes('/measurements/')).length, 0);
    h.tick(1500); await state(h, [current]);
    h.take('/measurements/' + m.measurementId).resolve({ data: row });
    await flush();
    assert.equal(h.card().status.textContent, '空车');
    await history(h, [fixture.historyMatch]);
    const cells = h.get('loadHistoryRows').children[0].children;
    assert.equal(cells[2].textContent, row.binding.licensePlate);
    assert.equal(cells[3].textContent, '空车');
    assert.equal(cells[4].textContent, '空车');
    assert.equal(cells[5].textContent, '--');
    assert.equal(cells.length, 7);
});
async function state(h, items = [], logRows = {}, autoResolveLogs = true) {
    h.take('/state').resolve({ data: { enabled: true, lanes: items } }); await flush();
    if (!autoResolveLogs) return;
    const pending = h.requests.filter(request => {
        if (!request.url.includes('/history?')) return false;
        return new URL(request.url, 'http://fixture').searchParams.has('nodeId');
    });
    for (const request of pending) {
        const index = h.requests.indexOf(request); if (index >= 0) h.requests.splice(index, 1);
        const channelId = new URL(request.url, 'http://fixture').searchParams.get('channelId');
        request.resolve({ data: logRows[channelId] || [] });
    }
    await flush();
}
async function history(h, rows = []) { h.click('openHistory'); h.take('/history?').resolve({ data: rows }); await flush(); }
async function detail(h, row, index = 0) { h.view(index); h.take('/measurements/').resolve({ data: row }); await flush(); }

function runtimeEvent(id, kind, ts, channel = 'CH1', message = kind) {
    return { event_id: id, channel, session: 1, ts, kind, message, source: 'live' };
}

if (process.env.LOAD_RADAR_RUNTIME_PIPELINE_FIXTURE) test('真实Python过程经WebApi净化后的同份JSON在页面完整显示', async () => {
    const response = JSON.parse(fs.readFileSync(process.env.LOAD_RADAR_RUNTIME_PIPELINE_FIXTURE, 'utf8'));
    const h = harness(); await state(h, response.lanes); h.select(0);
    assert.equal(h.runtime().rows.length, 4);
    assert.match(h.runtime().rows[0].textContent, /等待本车车牌确认/);
    assert.match(h.runtime().rows[2].textContent, /车辆已停稳/);
    assert.match(h.runtime().rows[3].textContent, /检测到车辆/);
    assert.equal(h.card().volume.textContent, '--');
});

test('未形成测量时到场停稳过程仍显示，且不会伪造正式结果', async () => {
    const h = harness(), current = lane();
    current.state.runtime_events = [runtimeEvent('r-1', 'vehicle_detected', 1700000000, 'CH1', '车辆进入通道'),
        runtimeEvent('r-2', 'vehicle_stable', 1700000005, 'CH1', '车辆已停稳，等待观测')];
    await state(h, [current]); h.select(0);
    h.take('/history?').resolve({ data: [] }); await flush();
    assert.equal(h.runtime().rows.length, 2);
    assert.match(h.runtime().rows[0].textContent, /车辆已停稳/);
    assert.match(h.runtime().rows[1].textContent, /车辆进入通道/);
    assert.match(h.log().rows[0].textContent, /暂无符合条件的车辆记录/);
    assert.equal(h.card().volume.textContent, '--');
    assert.equal(h.requests.some(r => r.options?.method), false);
});

test('过程快照重读不重复，离线保留历史但不当作当前状态', async () => {
    const h = harness(), current = lane();
    current.state.runtime_events = [runtimeEvent('r-1', 'vehicle_stable', 1700000000)];
    await state(h, [current]); h.select(0);
    h.tick(); await state(h, [current]);
    assert.equal(h.runtime().rows.length, 1);
    const offline = { ...current, online: false, state: null, error: '采集服务离线' };
    h.tick(); await state(h, [offline]);
    assert.equal(h.runtime().rows.length, 1);
    assert.match(h.runtime().feedback.textContent, /离线.*已发生/);
    assert.match(h.log().live.textContent, /离线/);
    assert.equal(h.card().volume.textContent, '--');
});

test('过程日志按固定通道切换，节点换绑或源重启不会混入旧过程', async () => {
    const h = harness(), one = lane(), two = lane('N1', 'CH2');
    one.state.runtime_events = [runtimeEvent('one', 'vehicle_arrived', 1700000000, 'CH1', '一号过程')];
    two.state.runtime_events = [runtimeEvent('two', 'vehicle_stable', 1700000001, 'CH2', '二号过程')];
    await state(h, [one, two]);
    assert.equal(h.runtime().rows.length, 2);
    h.select(1); assert.match(h.runtime().rows[0].textContent, /二号过程/);
    assert.doesNotMatch(h.runtime().rows[0].textContent, /一号过程/);
    two.state.runtime_events = [];
    h.tick(); await state(h, [one, two]);
    assert.equal(h.runtime().rows.length, 1);
    assert.match(h.runtime().rows[0].textContent, /暂无过程日志/);
    const replacement = lane('N2', 'CH2');
    h.tick(); await state(h, [one, replacement]);
    assert.doesNotMatch(h.runtime().rows[0].textContent, /二号过程/);
    assert.match(h.runtime().feedback.textContent, /更新采集服务/);
});

test('过程事件限量、去重并拒绝串道和坏时间，说明以纯文本显示', async () => {
    const h = harness(), current = lane();
    current.state.runtime_events = Array.from({ length: 120 }, (_, i) => runtimeEvent('r-' + i, 'vehicle_arrived', 1700000000 + i));
    await state(h, [current]); h.select(0);
    assert.equal(h.runtime().rows.length, 100);
    current.state.runtime_events = [runtimeEvent('ok', 'error', 1700000000, 'CH1', '<img onerror="alert(1)">'),
        runtimeEvent('ok', 'error', 1700000000), runtimeEvent('wrong', 'vehicle_stable', 1700000001, 'CH2'),
        runtimeEvent('bad', 'vehicle_stable', Infinity)];
    h.tick(); await state(h, [current]);
    assert.equal(h.runtime().rows.length, 1);
    assert.equal(h.runtime().rows[0].children[3].textContent, '<img onerror="alert(1)">');
});

test('同帧过程按源先后逆序展示，采集服务回滚协议时提示而不冒充实时过程', async () => {
    const h = harness(), current = lane();
    current.state.runtime_events = [runtimeEvent('epoch:9', 'vehicle_detected', 1700000000),
        runtimeEvent('epoch:10', 'vehicle_stable', 1700000000)];
    await state(h, [current]); h.select(0);
    assert.match(h.runtime().rows[0].textContent, /车辆已停稳/);
    delete current.state.runtime_events;
    h.tick(); await state(h, [current]);
    assert.match(h.runtime().feedback.textContent, /更新采集服务.*历史参考/);
    assert.equal(h.runtime().rows.length, 2);
});

test('过程保留真实来源标记，非实机事件不会冒充正式观测', async () => {
    const h = harness(), current = lane();
    current.state.runtime_events = [{ ...runtimeEvent('demo-1', 'vehicle_stable', 1700000000), source: 'sim-radar' }];
    await state(h, [current]); h.select(0);
    assert.match(h.runtime().rows[0].children[2].textContent, /车辆已停稳（演示）/);
});

test('首次只取状态，不读取基线、历史或发送业务写请求', async () => {
    const h = harness(); assert.equal(h.requests.length, 1); await state(h); assert.equal(h.requests.length, 0);
    assert.equal(h.clouds.length, 4); assert.equal(h.get('channelCount').textContent, '4');
    assert.equal(h.get('loadLanes').children.length, 4); assert.equal(h.get('laneLogs').children.length, 1);
});
test('首测前识牌和缺基线只显示必要提示，不增加操作入口', async () => {
    for (const [preparation, expected] of [['awaiting_plate', '正在识别车牌'],
        ['plate_unavailable', '未获取到本车车牌'], ['baseline_required', '需维护本车空车基线']]) {
        const h = harness(), current = lane(); current.state.preparation = preparation;
        await state(h, [current]);
        assert.equal(h.card().status.textContent, expected);
        assert.equal(h.card().volume.textContent, '--');
        assert.equal(h.requests.length, 0);
    }
});
test('等待车牌的持久留样显示后台处理，不冒充当前车结果', async () => {
    const h = harness(), current = lane();
    current.state.pending_compute = {pending_jobs: 2, ingress_jobs: 0};
    await state(h, [current]);
    assert.match(h.card().note.textContent, /2.*车次留样.*采集继续/);
    assert.equal(h.requests.filter(request => request.url.includes('/measurements/')).length, 0);
});
test('体积条件缺失时显示维护提示，不把估值显示为入库完成', async () => {
    const h = harness(), current = lane('N1', 'CH1', 'm1');
    current.state.volume_preparation = 'parameters_required';
    await state(h, [current]);
    h.take('/measurements/').resolve({data: record('m1', {volumeStatus: 'low_confidence'})});
    await flush();
    assert.match(h.card().note.textContent, /体积参数未就绪.*估值不入库/);
});
test('固定显示CH1-CH4；状态缺失时保留离线占位，不随后端列表增减', async () => {
    const h = harness(); await state(h, [lane('N1', 'CH2'), lane('N1', 'CH4')]);
    assert.equal(h.get('loadLanes').children.length, 4); assert.equal(h.clouds.length, 4);
    assert.ok(h.get('loadLanes').children.every(n => !n.hidden)); assert.equal(h.get('onlineCount').textContent, '2');
    assert.equal(h.get('channelCount').textContent, '4');
    assert.equal(h.card(0).badge.textContent, '离线'); assert.equal(h.card(1).badge.textContent, '在线');
    assert.equal(h.get('laneTabs').children.length, 6); assert.equal(h.get('laneLogs').children.length, 1);
    h.select(3); assert.deepEqual(h.get('loadLanes').children.map(n => n.hidden), [true, true, true, false]);
    assert.equal(h.get('laneLogs').children.length, 1); assert.equal(h.log().title.textContent, '通道-CH4车辆运行日志');
    h.select(); assert.ok(h.get('loadLanes').children.every(n => !n.hidden));
    assert.equal(h.get('laneLogs').children.length, 1); assert.equal(h.log().title.textContent, '全部车道车辆运行日志');
});
test('全部视图按 nodeId+channelId 分别读取逐车日志，并正确绑定车牌、判定、体积和说明', async () => {
    const h = harness(), items = [1, 2, 3, 4].map(i => lane('N1', 'CH' + i));
    await state(h, items, {}, false);
    const samples = [
        record('loaded-1', { channelId: 'CH1', volumeM3: 9 }),
        record('empty-2', { channelId: 'CH2', verdict: 'empty', volumeStatus: 'unavailable', volumeM3: 0 }),
        record('low-3', { channelId: 'CH3', volumeStatus: 'low_confidence', volumeM3: 12.345 }),
        record('bad-4', { channelId: 'CH4', verdict: 'unknown', volumeStatus: 'failed', frameDegraded: true })
    ];
    samples.forEach((row, index) => { row.binding.licensePlate = '宁A1000' + (index + 1); row.measurement.reason = '样例说明-' + (index + 1); });
    for (let index = 0; index < 4; index++) {
        const request = h.take('channelId=CH' + (index + 1));
        const query = new URL(request.url, 'http://fixture').searchParams;
        assert.equal(query.get('nodeId'), 'N1'); assert.equal(query.get('channelId'), 'CH' + (index + 1));
        assert.equal(query.get('take'), '50'); request.resolve({ data: [samples[index]] });
    }
    await flush();
    assert.equal(h.get('liveLayout').className, 'live-layout');
    assert.equal(h.get('laneLogs').children.length, 1);
    const logRows = h.log().rows.map(row => row.children.map(cell => cell.textContent));
    assert.equal(logRows.length, 4);
    assert.deepEqual(logRows.map(row => row[1]), ['通道-CH1', '通道-CH2', '通道-CH3', '通道-CH4']);
    assert.deepEqual(logRows[0].slice(2), [samples[0].binding.licensePlate, '计算完成', '已载货', '9.000', '样例说明-1']);
    assert.equal(h.log().feedback.textContent,
        '最近更新 ' + new Date(Date.now()).toLocaleString('zh-CN', { hour12: false }));
    assert.deepEqual(logRows[1].slice(2), [samples[1].binding.licensePlate, '空车', '空车', '--', '样例说明-2']);
    assert.deepEqual(logRows[2].slice(2), [samples[2].binding.licensePlate, '计算完成', '已载货', '12.345', '样例说明-3']);
    assert.equal(logRows[3][3], '无法确认载货状态');
});
test('单车道选择保留统一日志控件，日志只显示所选车道', async () => {
    const h = harness(); await state(h, [1, 2, 3, 4].map(i => lane('N1', 'CH' + i)));
    h.select(1);
    assert.equal(h.get('liveLayout').className, 'live-layout view-single');
    assert.deepEqual(h.get('loadLanes').children.map(node => node.hidden), [true, false, true, true]);
    assert.equal(h.get('laneLogs').children.length, 1);
    assert.equal(h.log().title.textContent, '通道-CH2车辆运行日志');
    const requests = h.requests.filter(request => request.url.includes('/history?'));
    assert.equal(requests.length, 1); assert.match(requests[0].url, /channelId=CH2/);
    h.take('channelId=CH2').resolve({ data: [record('single-ch2', { channelId: 'CH2' })] }); await flush();
    assert.equal(h.log().rows.length, 1);
    assert.equal(h.log().rows[0].children[1].textContent, '通道-CH2');
});
test('逐车日志拒绝跨通道记录，并为已读记录提供按车道游标分页', async () => {
    const h = harness(); await state(h, [1, 2, 3, 4].map(i => lane('N1', 'CH' + i)), {}, false);
    const mixed = record('right', { channelId: 'CH1' }), foreign = record('wrong', { channelId: 'CH2' });
    h.take('channelId=CH1').resolve({ data: [mixed, foreign] }); await flush();
    assert.equal(h.log().rows.length, 1); assert.match(h.log().feedback.textContent, /1 条通道不匹配/);
    const full = Array.from({ length: 50 }, (_, i) => record('page1-' + i, { channelId: 'CH1' }));
    h.log().refresh.handlers.click(); const refresh = h.take('channelId=CH1'); refresh.resolve({ data: full }); await flush();
    assert.equal(h.log().rows.length, 51); assert.equal(h.log().more.hidden, false);
    h.log().more.handlers.click(); const next = h.take('beforeId=right');
    next.resolve({ data: [record('page2-0', { channelId: 'CH1' })] }); await flush();
    assert.equal(h.log().rows.length, 52); assert.equal(h.log().more.hidden, true);
});
test('筛选仅改变显示，隐藏通道继续更新；轮询不重建卡片和筛选按钮', async () => {
    const h = harness(); await state(h, [lane(), lane('N1', 'CH2')]);
    const node = h.card().node, tab = h.get('laneTabs').children[2], option = h.get('filterChannel').children[1];
    h.select(1); h.tick(); const first = lane(); first.state.cloud = [[7, 8, 9]];
    await state(h, [first, lane('N1', 'CH2')]);
    assert.equal(h.card().node, node); assert.equal(h.get('laneTabs').children[2], tab);
    assert.equal(h.get('filterChannel').children[1], option); assert.deepEqual(plain(h.clouds[0].points), [[7, 8, 9]]);
    assert.equal(h.card().node.hidden, true);
});
test('无本车编号不显示 last_result、旧车牌或占位体积', async () => {
    const h = harness(); await state(h, [lane()]); assert.equal(h.card().volume.textContent, '--');
    assert.equal(h.card().plate.textContent, '--'); assert.equal(h.card().status.textContent, '正在测量');
});
test('空闲和移动只显示本次状态', async () => {
    for (const [value, expected] of [[0, '等待车辆'], [1, '等待车辆停稳']]) {
        const h = harness(), row = lane(); row.state.stability.state = value; await state(h, [row]);
        assert.equal(h.card().status.textContent, expected); assert.equal(h.card().volume.textContent, '--');
    }
});
test('四路分别读取明确结果编号，不把第一路车牌体积套到其它通道', async () => {
    const h = harness(), items = [1, 2, 3, 4].map(i => lane('N1', 'CH' + i, 'm' + i)); await state(h, items);
    for (let i = 4; i >= 1; i--) {
        const row = record('m' + i, { channelId: 'CH' + i, volumeM3: i * 10 }); row.binding.licensePlate = '宁A0000' + i;
        h.take('/measurements/m' + i).resolve({ data: row });
    }
    await flush();
    for (let i = 0; i < 4; i++) { assert.equal(h.card(i).volume.textContent, ((i + 1) * 10).toFixed(3)); assert.equal(h.card(i).plate.textContent, '宁A0000' + (i + 1)); }
});
test('错误编号、跨通道和跨节点结果拒绝显示', async () => {
    for (const row of [record('other'), record('m1', { channelId: 'CH2' }), record('m1', { nodeId: 'N2' })]) {
        const h = harness(); await state(h, [lane('N1', 'CH1', 'm1')]);
        h.take('/measurements/m1').resolve({ data: row }); await flush(); assert.equal(h.card().volume.textContent, '--');
    }
});
test('换车后旧请求迟到不能覆盖新车，新编号随后可查询', async () => {
    const h = harness(); await state(h, [lane('N1', 'CH1', 'first')]); const first = h.take('/measurements/first');
    h.tick(); await state(h, [lane('N1', 'CH1', 'second')]); first.resolve({ data: record('first') }); await flush();
    assert.equal(h.card().volume.textContent, '--'); h.tick(); await state(h, [lane('N1', 'CH1', 'second')]);
    h.take('/measurements/second').resolve({ data: record('second', { volumeM3: 2 }) }); await flush();
    assert.equal(h.card().volume.textContent, '2.000');
});
test('固定通道状态消失时保留卡位并清空实时数据，迟到结果不覆盖后续状态', async () => {
    const h = harness(); await state(h, [lane('N1', 'CH1', 'm1')]); const first = h.take('/measurements/m1');
    h.tick(); await state(h); assert.equal(h.clouds[0].disposed, false); assert.equal(h.get('loadLanes').children.length, 4);
    assert.equal(h.card(0).badge.textContent, '离线');
    h.tick(); await state(h, [lane('N1', 'CH1', 'm1')]); first.resolve({ data: record() }); await flush();
    assert.equal(h.card().volume.textContent, '--');
    h.tick(); await state(h, [lane('N1', 'CH1', 'm1')]);
    h.take('/measurements/m1').resolve({ data: record('m1', { volumeM3: 8 }) }); await flush(); assert.equal(h.card().volume.textContent, '8.000');
});
test('状态请求与每通道结果请求分别单飞', async () => {
    const h = harness(); h.tick(); assert.equal(h.requests.length, 1); await state(h, [lane('N1', 'CH1', 'm1')]);
    h.tick(); await state(h, [lane('N1', 'CH1', 'm1')]); assert.equal(h.requests.filter(r => r.url.includes('/measurements/')).length, 1);
});
test('冻结仅作用于该路，非实时说明明确，恢复不影响其它路', async () => {
    const h = harness(); await state(h, [lane(), lane('N1', 'CH2')]); h.card().freeze.handlers.click();
    h.tick(); const first = lane(); first.state.cloud = [[9, 9, 9]]; await state(h, [first, lane('N1', 'CH2')]);
    assert.deepEqual(plain(h.clouds[0].points), [[1, 2, 3]]); assert.match(h.card().canvasStatus.textContent, /非实时/);
    assert.equal(h.clouds[1].frozen, false); h.card().freeze.handlers.click();
    assert.deepEqual(plain(h.clouds[0].points), [[9, 9, 9]]);
});
test('断流、超龄和状态请求失败清空旧点云与本车结果，包括冻结画面', async () => {
    for (const mode of ['offline', 'stale', 'failure']) {
        const h = harness(); await state(h, [lane('N1', 'CH1', 'm1')]);
        h.take('/measurements/m1').resolve({ data: record() }); await flush(); h.card().freeze.handlers.click(); h.tick();
        if (mode === 'failure') { h.take('/state').reject(new Error('offline')); await flush(); }
        else { const row = lane('N1', 'CH1', 'm1'); if (mode === 'offline') row.online = false; else row.state.frame_age_sec = 6; await state(h, [row]); }
        assert.equal(h.card().volume.textContent, '--'); assert.deepEqual(plain(h.clouds[0].points), []); assert.equal(h.card().freeze.disabled, true);
    }
});
test('后台停用后不会继续显示旧结果或读取测量', async () => {
    const h = harness(); h.take('/state').resolve({ data: { enabled: false, lanes: [lane('N1', 'CH1', 'm1')] } }); await flush();
    assert.equal(h.requests.length, 0); assert.equal(h.card().volume.textContent, '--'); assert.equal(h.get('onlineCount').textContent, '0');
});
test('结果尚未保存或查询失败后只读重试，不重新采集', async () => {
    const h = harness(); await state(h, [lane('N1', 'CH1', 'm1')]); h.take('/measurements/m1').reject(new Error('404')); await flush();
    assert.equal(h.card().status.textContent, '等待测量结果'); h.tick(); await state(h, [lane('N1', 'CH1', 'm1')]);
    h.take('/measurements/m1').resolve({ data: record() }); await flush(); assert.equal(h.card().volume.textContent, '52.125');
    assert.match(h.card().note.textContent, /煤量尚未保存/);
});
test('体积空值、非法值不补零，真实零值与历史计算值直接展示', async () => {
    for (const [value, status, expected] of [[null, 'success', '--'], [-1, 'success', '--'], [Infinity, 'success', '--'], [0, 'success', '0.000'], [12, 'low_confidence', '12.000'], [12, 'failed', '--']]) {
        const h = harness(); await state(h, [lane('N1', 'CH1', 'm1')]); h.take('/measurements/m1').resolve({ data: record('m1', { volumeM3: value, volumeStatus: status }) }); await flush();
        assert.equal(h.card().volume.textContent, expected);
        assert.equal(h.card().estimate.hidden, true);
        if (status === 'low_confidence') {
            assert.match(h.card().note.textContent, /煤量尚未保存/);
            assert.equal(h.card().status.textContent, '计算完成');
            h.card().freeze.handlers.click(); assert.equal(h.card().estimate.hidden, true);
        }
    }
});
test('只有正式发布回执显示煤量已保存，空车不显示占位零', async () => {
    const h = harness(); await state(h, [lane('N1', 'CH1', 'm1')]); const row = record(); row.publication = { coalVolumeId: 17 };
    h.take('/measurements/m1').resolve({ data: row }); await flush(); assert.equal(h.card().note.textContent, '煤量已保存');
    h.tick(); await state(h, [lane('N1', 'CH1', 'empty')]);
    h.take('/measurements/empty').resolve({ data: record('empty', { verdict: 'empty', volumeStatus: 'unavailable', volumeM3: 0 }) }); await flush();
    assert.equal(h.card().volume.textContent, '--'); assert.equal(h.card().status.textContent, '空车');
});

test('显示CH4事后复算修订体积，同时保留原测量与当前车辆归属', async () => {
    const h = harness(); await state(h, [lane('N1', 'CH1', 'm1')]);
    const row = record('m1', { volumeM3: 12, volumeStatus: 'success' });
    row.latestRevision = { id: 'revision-1', calculation: { ...row.measurement, volumeM3: 15.75 } };
    h.take('/measurements/m1').resolve({ data: row }); await flush();
    assert.equal(h.card().volume.textContent, '15.750');
    assert.match(h.card().note.textContent, /CH4.*重新计算/);
    assert.equal(row.measurement.volumeM3, 12);
    h.tick(); await state(h, [lane('N1', 'CH1', 'm2')]);
    assert.equal(h.card().volume.textContent, '--');
});
test('历史在独立详情看云，不替换四路实时结果；关闭清理', async () => {
    const h = harness(); await state(h, [lane()]); await history(h, [record()]); await detail(h, record());
    assert.equal(h.get('historyDetail').hidden, false); assert.equal(h.get('historyResultVolume').textContent, '52.125');
    assert.equal(h.card().volume.textContent, '--'); assert.deepEqual(plain(h.clouds[0].points), [[1, 2, 3]]);
    assert.deepEqual(plain(h.clouds[4].points), [[4, 5, 6]]);
    assert.match(h.get('historySummary').textContent, /非实时/); h.click('closeHistory');
    assert.equal(h.get('historyDetail').hidden, true); assert.deepEqual(plain(h.clouds[4].points), []);
});
test('历史详情迟到、关闭、重新筛选后不得恢复旧详情', async () => {
    for (const action of ['close', 'query']) {
        const h = harness(); await state(h); await history(h, [record()]); h.view(); const request = h.take('/measurements/');
        if (action === 'close') h.click('closeHistory'); else h.submit();
        request.resolve({ data: record() }); await flush(); assert.equal(h.get('historyDetail').hidden, true);
    }
});
test('连续选择历史时只显示最后选择，当前通道消失不清除历史', async () => {
    const h = harness(); await state(h, [lane()]); await history(h, [record('first'), record('second')]);
    h.view(0); const first = h.take('/measurements/first'); h.view(1); const second = h.take('/measurements/second');
    second.resolve({ data: record('second', { volumeM3: 23 }) }); await flush();
    first.resolve({ data: record('first') }); await flush(); h.tick(); await state(h);
    assert.equal(h.get('historyResultVolume').textContent, '23.000'); assert.equal(h.get('historyDetail').hidden, false);
    assert.deepEqual(plain(h.clouds[4].points), [[4, 5, 6]]);
});
test('车牌、通道和告警作为文本显示，不注入HTML', async () => {
    const h = harness(), item = lane('N1', 'CH1', 'm1'), row = record();
    item.name = '<img src=x onerror=alert(1)>'; item.state.health = { alerts: [{ message: '<script>bad</script>' }] };
    row.binding.licensePlate = item.name; await state(h, [item]); h.take('/measurements/m1').resolve({ data: row }); await flush();
    assert.equal(h.card().plate.textContent, item.name); assert.equal(h.card().plate.children.length, 0);
});
test('历史查询编码车牌、成对通道和带时区时间，不需技术编号输入框', async () => {
    const h = harness(); await state(h, [lane('N1', 'CH1')]);
    h.get('filterPlate').value = '宁A&?'; h.get('filterChannel').value = JSON.stringify(['N1', 'CH1']);
    h.get('filterFrom').value = '2026-09-18T08:00'; h.get('filterTo').value = '2026-09-18T09:00';
    h.submit(); const url = new URL(h.take('/history?').url, 'http://fixture');
    assert.equal(url.searchParams.get('licensePlate'), '宁A&?'); assert.equal(url.searchParams.get('nodeId'), 'N1');
    assert.equal(url.searchParams.get('from'), new Date('2026-09-18T08:00').toISOString());
    assert.equal(h.get('filterNode'), null);
});
test('历史时间倒置先提示，不发查询', async () => {
    const h = harness(); await state(h); h.get('filterFrom').value = '2026-09-19T00:00'; h.get('filterTo').value = '2026-09-18T00:00';
    h.submit(); assert.equal(h.requests.length, 0); assert.match(h.get('historySummary').textContent, /开始时间/);
});
test('历史分页使用原查询和末条游标，新查询丢弃旧响应', async () => {
    const h = harness(); await state(h);
    await history(h, Array.from({ length: 50 }, (_, i) => record('m' + i)));
    h.get('filterPlate').value = '未提交修改'; h.click('loadHistoryMore'); const more = h.take('/history?');
    assert.match(more.url, /beforeId=m49/); assert.doesNotMatch(more.url, /licensePlate/);
    h.submit(); assert.equal(h.requests.length, 0);
    more.resolve({ data: [record('old')] }); await flush();
    const fresh = h.take('/history?'); assert.match(decodeURIComponent(fresh.url), /未提交修改/);
    assert.equal(h.get('loadHistoryRows').children.length, 1); // 空列表提示，不显示旧结果。
    fresh.resolve({ data: [record('fresh')] }); await flush();
    assert.equal(h.get('loadHistoryRows').children.length, 1);
    assert.equal(h.get('historySummary').textContent, '共显示 1 条');
});

test('连续查询与关闭重开共用一个在途请求，只补查最后条件', async () => {
    const h = harness(); await state(h); h.click('openHistory'); const active = h.take('/history?');
    for (let i = 0; i < 10; i++) { h.get('filterPlate').value = '宁A' + i; h.submit(); }
    h.click('closeHistory'); h.click('openHistory');
    assert.equal(h.requests.length, 0);
    active.resolve({ data: [record('obsolete')] }); await flush();
    assert.equal(h.requests.length, 1); const latest = h.take('/history?');
    assert.equal(new URL(latest.url, 'http://fixture').searchParams.get('licensePlate'), '宁A9');
    latest.resolve({ data: [record('latest')] }); await flush();
    assert.equal(h.requests.length, 0); h.view(); assert.match(h.take('/measurements/').url, /latest$/);
});

test('历史查询失败后仍执行最后排队条件，离开页面不执行排队查询', async () => {
    for (const leave of [false, true]) {
        const h = harness(); await state(h); h.click('openHistory'); const active = h.take('/history?');
        h.get('filterPlate').value = '宁A最后'; h.submit();
        if (leave) h.events.pagehide();
        active.reject(new Error('offline')); await flush();
        assert.equal(h.requests.length, leave ? 0 : 1);
        if (!leave) assert.equal(new URL(h.take('/history?').url, 'http://fixture').searchParams.get('licensePlate'), '宁A最后');
    }
});
test('历史请求失败不会显示查询成功但零条记录', async () => {
    const h = harness(); await state(h); h.click('openHistory');
    assert.match(h.get('loadHistoryRows').textContent, /正在查询/);
    h.take('/history?').reject(new Error('offline')); await flush();
    assert.match(h.get('historySummary').textContent, /查询失败/);
    assert.doesNotMatch(h.get('loadHistoryRows').textContent, /没有符合条件/);
    h.submit(); h.take('/history?').resolve({ data: [] }); await flush();
    assert.match(h.get('loadHistoryRows').textContent, /没有符合条件/);
});

test('加载更多失败保留历史且允许原游标重试', async () => {
    const h = harness(); await state(h); await history(h, Array.from({ length: 50 }, (_, i) => record('m' + i)));
    h.click('loadHistoryMore'); h.take('/history?').reject(new Error('offline')); await flush();
    assert.equal(h.get('loadHistoryRows').children.length, 50); assert.equal(h.get('loadHistoryMore').disabled, false);
});

test('所有页面请求只读，无手工关联或审批入口', async () => {
    const h = harness(); await state(h); await history(h);
    for (const id of ['loadPublishForm', 'loadBindForm', 'loadBaselineForm', 'queryCommand']) assert.equal(h.get(id), null);
    assert.ok(h.requests.every(r => !r.options?.method));
});
test('离开页面释放所有画布，晚到结果和状态不得恢复', async () => {
    const h = harness(); await state(h, [lane('N1', 'CH1', 'm1')]); const result = h.take('/measurements/m1');
    h.tick(); const request = h.take('/state'); h.events.pagehide();
    result.resolve({ data: record() }); request.resolve({ data: { enabled: true, lanes: [lane()] } }); await flush(); h.tick();
    assert.equal(h.clouds[0].disposed, true); assert.equal(h.requests.length, 0); assert.equal(h.card().volume.textContent, '--');
});
test('保留后台导航与鉴权，增加只读体积查询，无技术诊断或演示入口', () => {
    const html = fs.readFileSync(path.join(viewerRoot, 'load-radar.html'), 'utf8');
    assert.match(html, /js\/api-client\.js/); assert.match(html, /js\/auth-guard\.js/);
    assert.equal((html.match(/class="nav-link(?:\s|")/g) || []).length, 6);
    assert.match(html, /href="volume-query.html" class="nav-link" data-role="admin,viewer"/);
    assert.match(html, /id="navLogout"/);
    assert.doesNotMatch(html, /基线维护|审批|高级设置|自动演示|强制重测|生效配置/);
});
test('画布过滤非法点，冻结保留旧帧，恢复使用最新帧并释放监听', () => {
    const listeners = [], jobs = [], labels = [], drawn = [];
    const surface = { addEventListener(name, fn) { listeners.push([this, name, fn]); }, removeEventListener(name, fn) { const i = listeners.findIndex(v => v[0] === this && v[1] === name && v[2] === fn); if (i >= 0) listeners.splice(i, 1); } };
    const ctx = { fillRect(...rectangle) { drawn.push(rectangle); }, fillText(text) { labels.push(text); } };
    const canvas = { ...surface, getContext: () => ctx, getBoundingClientRect: () => ({ width: 400, height: 300 }) };
    const context = vm.createContext({ window: { ...surface, devicePixelRatio: 1 }, canvas, setTimeout(fn) { jobs.push(fn); return jobs.length; }, clearTimeout() {} });
    vm.runInContext(fs.readFileSync(path.join(viewerRoot, 'js/load-radar-cloud.js'), 'utf8'), context);
    const cloud = vm.runInContext('LoadRadarCloud.create(canvas)', context);
    cloud.setBounds([[-2, -10, -6], [2, 8, 0]]); cloud.setPoints([[0, 0, -2], [1, 2, Infinity], [NaN, 1, 2]]);
    assert.equal(cloud.pointCount, 1); jobs.splice(0).forEach(fn => fn());
    const originalPoint = drawn.at(-1);
    // 按固定范围投影，而不是跟随单点自动居中；冻结后的实际像素也必须不变。
    assert.ok(Math.abs(originalPoint[0] - 200) > 1, '固定 ROI 内的非中心点不能被逐帧自动居中');
    cloud.setFrozen(true); cloud.setPoints([[0, 0, -1], [1, 1, -1]]); assert.equal(cloud.pointCount, 1);
    jobs.splice(0).forEach(fn => fn()); assert.deepEqual(drawn.at(-1), originalPoint);
    assert.equal(labels.some(s => s.includes('ROI')), false);
    cloud.setFrozen(false); assert.equal(cloud.pointCount, 2); cloud.clear(); assert.equal(cloud.pointCount, 0);
    cloud.dispose(); assert.equal(listeners.length, 0);
});

test('长条固定 ROI 在总览和窄屏内完整取景，缩放后双击复位恢复像素', () => {
    for (const [width, height] of [[570, 200], [340, 230], [1000, 520]]) {
        const jobs = [], drawn = [], handlers = {};
        const ctx = { fillRect(...rectangle) { drawn.push(rectangle); }, fillText() {} };
        const canvas = { getContext: () => ctx, getBoundingClientRect: () => ({ width, height }),
            addEventListener(name, fn) { handlers[name] = fn; }, removeEventListener() {} };
        const context = vm.createContext({ window: { devicePixelRatio: 1, addEventListener() {}, removeEventListener() {} },
            canvas, setTimeout(fn) { jobs.push(fn); return jobs.length; }, clearTimeout() {} });
        vm.runInContext(fs.readFileSync(path.join(viewerRoot, 'js/load-radar-cloud.js'), 'utf8'), context);
        const cloud = vm.runInContext('LoadRadarCloud.create(canvas)', context);
        const corners = [];
        for (const x of [-1.5, 1.5]) for (const y of [-5.7, 5.7]) for (const z of [-3.2, 0]) corners.push([x, y, z]);
        cloud.setBounds([[-1.5, -5.7, -3.2], [1.5, 5.7, 0]]); cloud.setPoints(corners);
        jobs.splice(0).forEach(fn => fn());
        const initial = drawn.slice(-8);
        assert.ok(initial.every(([x, y, w, h]) => x >= 0 && y >= 0 && x + w <= width && y + h <= height));
        handlers.wheel({ preventDefault() {}, deltaY: -1 }); jobs.splice(0).forEach(fn => fn());
        assert.notDeepEqual(drawn.slice(-8), initial);
        handlers.dblclick(); jobs.splice(0).forEach(fn => fn());
        assert.deepEqual(drawn.slice(-8), initial);
        cloud.dispose();
    }
});
test('后台标签页暂停 /state 轮询，回到前台立即补一次并恢复周期', async () => {
    const h = harness();
    const states = () => h.requests.filter(r => r.url.includes('/state')).length;
    await state(h, [lane('N1', 'CH1', 'm1')]);
    assert.equal(states(), 0);
    h.document.hidden = true; h.events['document:visibilitychange']();
    h.tick(); h.tick();
    assert.equal(states(), 0, '后台标签页仍在轮询采集端快照');
    h.document.hidden = false; h.events['document:visibilitychange']();
    assert.equal(states(), 1, '回到前台应立即补一次');
    await state(h, [lane('N1', 'CH1', 'm1')]);
    h.tick();
    assert.equal(states(), 1, '回到前台后应恢复周期轮询');
});

test('实时卡片、运行记录与历史弹窗展示装载分级和实测体积，体积缺测时显示占位符', async () => {
    const h = harness();
    await state(h, [lane('N1', 'CH1', 'm-half')], {}, false);
    const half = record('m-half', { channelId: 'CH1', volumeM3: 27.5 });
    half.binding.licensePlate = '宁A88801';
    half.cargoEstimate = {
        boxCapacityM3: 50, loadRatio: 0.55, loadLevel: 'half',
        loadLevelLabel: '半载'
    };
    const emptyRow = record('m-empty', { channelId: 'CH1', verdict: 'empty', volumeStatus: 'unavailable', volumeM3: null });
    emptyRow.binding.licensePlate = '宁A88802';
    emptyRow.cargoEstimate = {
        boxCapacityM3: 50, loadRatio: null, loadLevel: 'empty',
        loadLevelLabel: '空车'
    };
    h.take('/measurements/m-half').resolve({ data: half });
    h.take('channelId=CH1').resolve({ data: [half, emptyRow] });
    await flush();

    const card = h.card(0);
    assert.equal(card.cargo.textContent, '半载 · 55%');
    assert.equal(card.volume.textContent, '27.500');

    const logRows = h.log().rows.map(r => r.children.map(c => c.textContent));
    assert.equal(logRows[0][4], '半载 · 55%');
    assert.equal(logRows[0][5], '27.500');
    assert.equal(logRows[1][4], '空车');
    assert.equal(logRows[1][5], '--');
    assert.deepEqual(logRows.map(row => row.length), [7, 7]);

    await history(h, [half, emptyRow]);
    const historyRows = h.get('loadHistoryRows').children.map(r => r.children.map(c => c.textContent));
    assert.deepEqual(historyRows[0].slice(4, 6), ['半载 · 55%', '27.500']);
    assert.deepEqual(historyRows[1].slice(4, 6), ['空车', '--']);
    assert.deepEqual(historyRows.map(row => row.length), [7, 7]);
    h.view();
    h.take('/measurements/m-half').resolve({ data: half });
    await flush();
    assert.equal(h.get('historyResultCargo').textContent, '半载 · 55%');
    assert.equal(h.get('historyResultVolume').textContent, '27.500');
});
