const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

function harness() {
    const timers = new Map(), events = {}, dots = [];
    let sequence = 0, width = 0, height = 0;
    const stats = { draws: 0, resizes: 0 }, rect = { width: 800, height: 600 };
    const ctx = { fillRect(...args) { if (args.length && args[0] === 0 && args[1] === 0) { stats.draws++; dots.length = 0; } else dots.push([...args, this.fillStyle]); }, fillText() {} };
    const canvas = { getContext: () => ctx, getBoundingClientRect: () => rect,
        addEventListener(name, handler) { events[name] = handler; }, removeEventListener() {} };
    Object.defineProperties(canvas, {
        width: { get: () => width, set(value) { width = value; stats.resizes++; } },
        height: { get: () => height, set(value) { height = value; stats.resizes++; } }
    });
    const context = vm.createContext({ canvas, window: { devicePixelRatio: 1, addEventListener(name, handler) { events[name] = handler; }, removeEventListener() {} },
        setTimeout(fn) { timers.set(++sequence, fn); return sequence; }, clearTimeout(id) { timers.delete(id); } });
    vm.runInContext(fs.readFileSync(path.resolve(__dirname, '../../wwwroot/pointcloud-viewer/js/load-radar-cloud.js'), 'utf8'), context);
    const cloud = vm.runInContext('LoadRadarCloud.create(canvas)', context);
    function flush() { const pending = [...timers.values()]; timers.clear(); pending.forEach(fn => fn()); }
    return { cloud, events, stats, dots, flush, timers, rect };
}

test('identical point snapshots and ROI avoid repaint; coordinate changes still repaint', () => {
    const { cloud, stats, dots, flush } = harness();
    const bounds = [[-5, -5, -5], [5, 5, 5]], points = [[1, 2, 3], [2, 3, 4]];
    cloud.setBounds(bounds); cloud.setPoints(points); flush();
    const image = JSON.stringify(dots);
    for (let i = 0; i < 100; i++) {
        cloud.setBounds(bounds.map(row => row.slice()));
        cloud.setPoints(points.map(row => row.slice())); flush();
    }
    assert.equal(stats.draws, 1);
    assert.equal(stats.resizes, 2, 'unchanged dimensions must not reset the canvas backing buffer');
    points[0][0] = -2; cloud.setPoints(points); flush();
    assert.equal(stats.draws, 2);
    assert.notEqual(JSON.stringify(dots), image);
    cloud.setBounds([[-10, -10, -10], [10, 10, 10]]); flush();
    assert.equal(stats.draws, 3);
});

test('frozen updates keep the picture and resume with the latest owned coordinates', () => {
    const { cloud, stats, dots, flush } = harness();
    cloud.setBounds([[-5, -5, -5], [5, 5, 5]]); cloud.setPoints([[1, 2, 3]]); flush();
    cloud.setFrozen(true); flush();
    const image = JSON.stringify(dots), count = stats.draws;
    const rows = [[-3, -2, -1], [2, 3, 4]];
    cloud.setPoints(rows); flush();
    rows[0][0] = 100;
    assert.equal(stats.draws, count);
    assert.equal(JSON.stringify(dots), image);
    assert.equal(cloud.pointCount, 1);
    cloud.setFrozen(false); flush();
    assert.equal(cloud.pointCount, 2);
    assert.equal(stats.draws, count + 1);
    const resumed = JSON.stringify(dots);
    cloud.setPoints([[-3, -2, -1], [2, 3, 4]]); flush();
    assert.equal(stats.draws, count + 1, 'callers cannot mutate the stored coordinates');
    assert.equal(JSON.stringify(dots), resumed);
});

test('sampling, invalid inputs, clearing, interaction and disposal remain functional', () => {
    const { cloud, stats, events, flush, timers } = harness();
    cloud.setPoints(Array.from({ length: 10001 }, (_, i) => [i, i / 2, i / 3])); flush();
    assert.equal(cloud.pointCount, 3334);
    cloud.setPoints([null, [], [NaN, 1, 2], [1, Infinity, 2], [1, 2, 3, 4]]); flush();
    assert.equal(cloud.pointCount, 1);
    const before = stats.draws;
    events.pointerdown({ clientX: 0, clientY: 0 }); events.pointermove({ clientX: 20, clientY: 10 }); events.pointerup(); flush();
    events.wheel({ preventDefault() {}, deltaY: -1 }); flush();
    events.dblclick(); flush(); events.resize(); flush();
    assert.equal(stats.draws, before + 4);
    cloud.clear(); flush(); const cleared = stats.draws;
    cloud.clear(); cloud.setPoints([]); cloud.setBounds(null); flush();
    assert.equal(cloud.pointCount, 0);
    assert.equal(stats.draws, cleared);
    cloud.setPoints([[1, 2, 3]]); cloud.dispose(); flush();
    assert.equal(stats.draws, cleared);
    assert.equal(timers.size, 0);
});

test('hidden cards retain incoming frames and draw the latest frame when selected', () => {
    const { cloud, stats, rect, flush } = harness();
    rect.width = 0; rect.height = 0;
    cloud.setPoints([[1, 2, 3]]); flush();
    cloud.setPoints([[2, 3, 4], [3, 4, 5]]); flush();
    assert.equal(stats.draws, 0);
    assert.equal(stats.resizes, 0);
    rect.width = 800; rect.height = 600;
    cloud.fit(); flush();
    assert.equal(stats.draws, 1);
    assert.equal(cloud.pointCount, 2);
});
