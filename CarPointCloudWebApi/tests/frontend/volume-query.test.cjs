const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const root = path.resolve(__dirname, '../../wwwroot/pointcloud-viewer');
const source = fs.readFileSync(path.join(root, 'js/volume-query.js'), 'utf8');
const flush = () => new Promise(resolve => setImmediate(resolve));

async function createPage(get) {
    const listeners = new Map();
    const elements = new Map();
    const requests = [];
    function element() {
        return { value: '', textContent: '', hidden: false, disabled: false, children: [], attrs: {},
            appendChild(child) { this.children.push(child); },
            replaceChildren() { this.children = []; },
            setAttribute(name, value) { this.attrs[name] = value; },
            reportValidity() { return true; } };
    }
    function byId(id) {
        if (!elements.has(id)) {
            const el = element();
            el.addEventListener = (type, handler) => listeners.set(id + ':' + type, handler);
            elements.set(id, el);
        }
        return elements.get(id);
    }
    // 本地已是11月2日，UTC仍是11月1日；不能用toISOString截取日期。
    class LocalDate extends Date {
        constructor(value) { super(value === undefined ? '2026-11-02T00:30:00+08:00' : value); }
        getFullYear() { return 2026; }
        getMonth() { return 10; }
        getDate() { return 2; }
    }
    vm.runInNewContext(source, {
        Date: LocalDate, URLSearchParams, console,
        document: { getElementById: byId, createElement: element,
            addEventListener(type, handler) { listeners.set('document:' + type, handler); } },
        API: { async get(url) {
            requests.push(new URL(url, 'http://test.invalid'));
            if (get) return get(requests.at(-1), requests.length);
            const query = requests.at(-1).searchParams;
            return { data: { items: [{ collectedAt: '2026-11-02T08:00:00', licensePlate: '宁A48394Q',
                scaleName: '4号磅', volumeM3: 0 }], total: 25,
                page: Number(query.get('page')), pageSize: Number(query.get('pageSize')) } };
        } }
    });
    listeners.get('document:DOMContentLoaded')();
    await flush();
    return { byId, requests, listeners, async event(id, event = 'click') {
        listeners.get(id + ':' + event)({ preventDefault() {} });
        await flush();
    } };
}

test('defaults use the local calendar day and all lanes, with initial server pagination', async () => {
    const p = await createPage();
    assert.equal(p.byId('startDate').value, '2026-11-02');
    assert.equal(p.byId('endDate').value, '2026-11-02');
    assert.equal(p.byId('channelId').value, '');
    const query = p.requests[0].searchParams;
    assert.equal(query.get('startDate'), '2026-11-02');
    assert.equal(query.get('endDate'), '2026-11-02');
    assert.equal(query.get('channelId'), '');
    assert.equal(query.get('page'), '1');
    assert.equal(query.get('pageSize'), '10');
    assert.equal(p.byId('previousPage').disabled, true);
    assert.equal(p.byId('nextPage').disabled, false);
});

test('zero volume is shown in cubic meters and plate text cannot insert HTML', async () => {
    const p = await createPage(async () => ({ data: { items: [{ collectedAt: '2026-11-02T09:10:00',
        licensePlate: '<img src=x onerror=alert(1)>', scaleName: '1号磅', volumeM3: 0 }], total: 1, page: 1 } }));
    const cells = p.byId('volumeTableBody').children[0].children;
    assert.equal(cells.length, 4);
    assert.equal(cells[1].textContent, '<img src=x onerror=alert(1)>');
    assert.equal(cells[2].textContent, '1号磅');
    assert.equal(cells[3].textContent, '0.0');
    assert.equal(cells[3].className, 'volume-number');
    assert.equal(p.byId('nextPage').disabled, true);
});

test('search normalizes plate, resets page, and pagination preserves submitted filters', async () => {
    const p = await createPage();
    await p.event('nextPage');
    assert.equal(p.requests.at(-1).searchParams.get('page'), '2');
    p.byId('licensePlate').value = ' 宁a92531s ';
    p.byId('channelId').value = 'CH4';
    p.byId('startDate').value = '2026-10-31';
    await p.event('volumeQueryForm', 'submit');
    assert.equal(p.requests.at(-1).searchParams.get('page'), '1');
    assert.equal(p.requests.at(-1).searchParams.get('licensePlate'), '宁A92531S');
    // 未点查询的输入不能悄悄改变下一页的条件。
    p.byId('licensePlate').value = 'different';
    await p.event('nextPage');
    const query = p.requests.at(-1).searchParams;
    assert.equal(query.get('page'), '2');
    assert.equal(query.get('licensePlate'), '宁A92531S');
    assert.equal(query.get('channelId'), 'CH4');
    assert.equal(query.get('startDate'), '2026-10-31');
});

test('reset restores today/all lanes and size changes reset to first server page', async () => {
    const p = await createPage();
    p.byId('licensePlate').value = '宁A07488S';
    p.byId('channelId').value = 'CH2';
    p.byId('startDate').value = '2026-10-01';
    await p.event('volumeQueryForm', 'submit');
    await p.event('resetButton');
    assert.equal(p.byId('licensePlate').value, '');
    assert.equal(p.byId('channelId').value, '');
    assert.equal(p.byId('startDate').value, '2026-11-02');
    await p.event('nextPage');
    p.byId('pageSize').value = '20';
    await p.event('pageSize', 'change');
    assert.equal(p.requests.at(-1).searchParams.get('page'), '1');
    assert.equal(p.requests.at(-1).searchParams.get('pageSize'), '20');
});

test('inverted date range is visible and sends no request', async () => {
    const p = await createPage();
    p.byId('startDate').value = '2026-11-03';
    await p.event('volumeQueryForm', 'submit');
    assert.equal(p.requests.length, 1);
    assert.equal(p.byId('queryError').hidden, false);
    assert.match(p.byId('queryError').textContent, /开始日期不能晚于结束日期/);
});

test('no records and request failures have distinct states and cannot page', async () => {
    const p = await createPage(async (_, count) => {
        if (count > 1) throw new Error('offline');
        return { data: { items: [], total: 0, page: 1 } };
    });
    assert.match(p.byId('volumeEmpty').textContent, /暂无符合条件/);
    assert.equal(p.byId('volumeEmpty').hidden, false);
    assert.equal(p.byId('nextPage').disabled, true);
    await p.event('volumeQueryForm', 'submit');
    assert.match(p.byId('queryError').textContent, /offline/);
    assert.match(p.byId('pageInfo').textContent, /数量未知/);
    assert.equal(p.byId('volumeTableBody').children.length, 0);
    assert.equal(p.byId('volumeResults').attrs['aria-busy'], 'false');
});

test('a late previous response cannot overwrite a newer query', async () => {
    let finishFirst;
    const p = await createPage((_, count) => count === 1 ? new Promise(resolve => { finishFirst = resolve; }) :
        Promise.resolve({ data: { items: [], total: 0, page: 1 } }));
    p.byId('licensePlate').value = '宁A49856T';
    await p.event('volumeQueryForm', 'submit');
    finishFirst({ data: { items: [{ licensePlate: 'old', collectedAt: '2026-11-02', scaleName: '1号磅', volumeM3: 12 }],
        total: 999, page: 1 } });
    await flush();
    assert.equal(p.byId('volumeTableBody').children.length, 0);
    assert.match(p.byId('pageInfo').textContent, /共 0 条/);
});

test('page exposes required filters, units, auth guard and navigation from every management page', () => {
    const html = fs.readFileSync(path.join(root, 'volume-query.html'), 'utf8');
    for (const id of ['startDate', 'endDate', 'licensePlate', 'channelId']) assert.match(html, new RegExp(`id="${id}"`));
    for (const title of ['采集时间', '车牌号', '几号磅', '体积（立方米）']) assert.ok(html.includes(title));
    assert.match(html, /value="" selected>全部/);
    assert.match(html, /js\/auth-guard.js/);
    assert.match(html, /css\/admin-shell.css/);
    for (const name of ['index', 'live', 'load-radar', 'logs', 'users']) {
        const page = fs.readFileSync(path.join(root, name + '.html'), 'utf8');
        assert.match(page, /href="volume-query.html" class="nav-link" data-role="admin,viewer"/);
    }
});
