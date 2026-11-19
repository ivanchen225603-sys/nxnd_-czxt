const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

function createPage(get) {
    const elements = new Map();
    const escape = text => String(text).replace(/[&<>"']/g, ch => ({
        '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'
    })[ch]);
    const element = id => {
        if (!elements.has(id)) {
            let html;
            elements.set(id, {
                value: '', style: {}, textContent: '', disabled: false, hidden: true,
                listeners: {}, children: [],
                get innerHTML() { return html ?? escape(this.textContent); },
                set innerHTML(value) { html = value; },
                addEventListener(name, callback) { this.listeners[name] = callback; },
                appendChild(child) { this.children.push(child); },
                focus() { document.activeElement = this; }
            });
        }
        return elements.get(id);
    };
    const document = {
        activeElement: null, listeners: {},
        addEventListener(name, callback) { this.listeners[name] = callback; },
        getElementById: element, querySelector: element,
        createElement: () => element(Symbol()), body: element('documentBody')
    };
    const context = vm.createContext({
        URLSearchParams, document, API: { get },
        setTimeout() {}, console: { error() {} }
    });
    vm.runInContext(fs.readFileSync(path.resolve(__dirname,
        '../../wwwroot/pointcloud-viewer/js/logs.js'), 'utf8'), context);
    return { context, element, document };
}

test('log query masks previous rows while pending, remains single flight and preserves every filter', async () => {
    let finish;
    const requests = [];
    const page = createPage(url => {
        requests.push(url);
        return new Promise(resolve => { finish = resolve; });
    });
    const filters = {
        keywordInput: '尺寸 & 空车', startTimeInput: '2026-10-20T00:00',
        endTimeInput: '2026-10-21T23:59', levelSelect: 'warn',
        sourceSelect: 'PartnerClient', interactionIdInput: 'request-123',
        workflowIdInput: 'workflow-456', licensePlateInput: '宁A81530M',
        integrationTypeSelect: 'VehicleDimensions'
    };
    Object.entries(filters).forEach(([id, value]) => { page.element(id).value = value; });
    page.element('logsTableBody').innerHTML = '<tr>previous result</tr>';
    const pending = page.context.loadLogs();
    assert.equal(page.element('logsResults').ariaBusy, 'true');
    assert.equal(page.element('logsTableBody').innerHTML, '');
    assert.match(page.element('.logs-empty-text').textContent, /正在查询/);
    assert.equal(page.element('statFiltered').textContent, '—');
    assert.equal(page.element('searchBtn').disabled, true);
    assert.equal(await page.context.loadLogs(), false);
    assert.equal(requests.length, 1);
    const query = new URL(requests[0], 'http://localhost').searchParams;
    for (const [key, inputId] of Object.entries({
        keyword: 'keywordInput', startTime: 'startTimeInput', endTime: 'endTimeInput',
        level: 'levelSelect', source: 'sourceSelect', interactionId: 'interactionIdInput',
        workflowId: 'workflowIdInput', licensePlate: 'licensePlateInput',
        integrationType: 'integrationTypeSelect'
    })) assert.equal(query.get(key), filters[inputId]);
    finish({ data: { items: [], total: 0 } });
    assert.equal(await pending, true);
    assert.equal(page.element('logsResults').ariaBusy, 'false');
    assert.equal(page.element('searchBtn').disabled, false);
    assert.equal(page.element('statFiltered').textContent, '0');
    assert.equal(page.element('.logs-empty-text').textContent, '暂无符合条件的日志');
});

test('ISO timestamps remain readable and full IDs and escaped messages survive table rendering', async () => {
    const id = '0be5c587-f4fe-40a9-949c-4e3ba6d934f8';
    const time = '2026-10-21T06:32:18.123Z';
    const page = createPage(async () => ({ data: { items: [{
        id: 1, time, level: 'warn', source: 'PartnerClient', integrationType: 'CoalVolume',
        direction: 'Outbound', interactionId: id, httpStatus: 429, message: '<script>unsafe</script>'
    }] } }));
    await page.context.loadLogs();
    const html = page.element('logsTableBody').innerHTML;
    assert.ok(html.includes(`title="${time}"`));
    assert.ok(html.includes(new Date(time).toLocaleString('zh-CN', { hour12: false })));
    assert.ok(html.includes(`title="${id}"`));
    assert.ok(html.includes(`data-interaction-id="${id}"`));
    assert.match(html, /获取煤炭体积/);
    assert.match(html, /出站/);
    assert.match(html, />429<\/span>/);
    assert.match(html, /&lt;script&gt;unsafe&lt;\/script&gt;/);
    assert.ok(!html.includes('<script>unsafe'));
});

test('chain dialog preserves metadata, restores keyboard focus and ignores a late closed response', async () => {
    const pending = new Map();
    const page = createPage(url => new Promise(resolve => { pending.set(url, resolve); }));
    const trigger = page.element('rowAction');
    const first = page.context.showInteractionDetails('first/id', trigger);
    assert.ok(pending.has('/api/logs/interaction/first%2Fid'));
    assert.equal(page.document.activeElement, page.element('interactionModalClose'));
    page.context.closeInteractionModal();
    assert.equal(page.document.activeElement, trigger);
    const second = page.context.showInteractionDetails('current', trigger);
    pending.get('/api/logs/interaction/current')({ data: [{
        recordKind: 'Lifecycle', lifecycleStage: 'QueryExecuted', integrationType: 'VehicleDimensions',
        direction: 'Inbound', callerSystem: 'Scale', targetSystem: 'WebApi', httpMethod: 'GET',
        requestPath: '/api/dimensions', startTime: '2026-10-21 14:32:00', time: '2026-10-21 14:32:01',
        elapsedMs: 12, businessCode: 200, attemptNo: 2, workflowId: 'workflow', licensePlate: '宁A81530M',
        cameraIdentifier: 'CAM_004', traceIdentifier: 'trace', clientIp: '127.0.0.1',
        requestContentLength: 20, responseContentLength: 30,
        requestHeadersJson: '{"Authorization":"[REDACTED]"}', requestBody: '{"query":1}',
        responseHeadersJson: '{"Content-Type":"application/json"}', responseBody: '{"length":1067.8}'
    }] });
    await second;
    const body = page.element('interactionModalBody').innerHTML;
    for (const text of ['QueryExecuted', 'Scale', 'WebApi', 'workflow', '宁A81530M', 'CAM_004',
        'trace', '127.0.0.1', '1067.8', 'REDACTED', '请求体', '响应体', '第 2 次']) {
        assert.ok(body.includes(text), `missing ${text}`);
    }
    pending.get('/api/logs/interaction/first%2Fid')({ data: [{ message: 'stale response' }] });
    await first;
    assert.equal(page.element('interactionModalBody').innerHTML, body);
});
