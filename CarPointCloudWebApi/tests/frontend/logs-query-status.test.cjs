const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

test('failed log query stays distinct from an empty success through rerender and retry', async () => {
    const elements = new Map();
    const element = id => {
        if (!elements.has(id)) elements.set(id, { value: '', style: {}, innerHTML: '', textContent: '' });
        return elements.get(id);
    };
    let failed = true;
    const context = vm.createContext({
        URLSearchParams, console: { error() {} }, setTimeout() {},
        document: {
            addEventListener() {}, getElementById: element,
            querySelector: element, createElement: () => element(Symbol()),
            body: { appendChild() {} }
        },
        API: { async get() {
            if (failed) throw new Error('offline');
            return { data: { items: [], total: 0 } };
        } }
    });
    vm.runInContext(fs.readFileSync(path.resolve(__dirname,
        '../../wwwroot/pointcloud-viewer/js/logs.js'), 'utf8'), context);
    await context.loadLogs();
    context.applyFilters();
    assert.match(element('.logs-empty-text').textContent, /查询失败/);
    assert.equal(element('statTotal').textContent, '—');
    assert.match(element('paginationInfo').textContent, /记录数量未知/);
    failed = false;
    await context.loadLogs();
    assert.equal(element('.logs-empty-text').textContent, '暂无符合条件的日志');
    assert.equal(element('statTotal').textContent, '0');
    assert.equal(element('paginationInfo').textContent, '显示 0 条，共 0 条');
});
