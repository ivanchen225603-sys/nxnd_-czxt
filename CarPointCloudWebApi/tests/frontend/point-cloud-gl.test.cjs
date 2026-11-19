const { test } = require('node:test');
const assert = require('node:assert/strict');
const vm = require('node:vm');
const fs = require('node:fs');
const path = require('node:path');

function harness() {
    const draws = [];
    const events = {}, stats = { clears: 0 };
    const gl = new Proxy({
        TRIANGLES: 'TRIANGLES', POINTS: 'POINTS', LINES: 'LINES',
        getShaderParameter: () => true, getProgramParameter: () => true,
        clear: () => stats.clears++,
        drawArrays: (mode, first, count) => draws.push({ mode, count })
    }, { get: (object, key) => key in object ? object[key] : () => {} });
    const canvas = { getContext: () => gl, addEventListener(name, handler) { events[name] = handler; }, clientWidth: 800, clientHeight: 600 };
    const window = { addEventListener(name, handler) { events[name] = handler; }, devicePixelRatio: 1 };
    const context = vm.createContext({ Float32Array, canvas, window });
    vm.runInContext(fs.readFileSync(path.resolve(__dirname, '../../wwwroot/pointcloud-viewer/js/point-cloud-gl.js'), 'utf8'), context);
    const renderer = vm.runInContext('PointCloudGL.create(canvas)', context);
    return { renderer, draws, canvas, window, events, stats };
}

test('continuous mesh draws triangles and never survives an empty or raw replacement frame', () => {
    const { renderer, draws } = harness();
    const points = new Float32Array([0, 0, 0, 1, 0, 0, 1, 0, 1]);
    renderer.setPoints(points, 0, 0, true, 3, points);
    renderer.render();
    assert.equal(draws.filter(d => d.mode === 'TRIANGLES')[0].count, 3);
    draws.length = 0;
    renderer.setPoints(points, 3, 0, false);
    renderer.render();
    assert.ok(!draws.some(d => d.mode === 'TRIANGLES'));
    draws.length = 0;
    renderer.setPoints(new Float32Array(0), 0, 0, false);
    renderer.render();
    assert.deepEqual(draws, []);
    assert.equal(renderer.bounds, null);
});

test('unchanged animation frames do not clear or redraw the scene', () => {
    const { renderer, draws, stats } = harness();
    renderer.setPoints(new Float32Array([0, 0, 0, 1, 2, 3]), 1, 1, true);
    renderer.render();
    const count = draws.length;
    for (let i = 0; i < 600; i++) renderer.render();
    assert.equal(stats.clears, 1);
    assert.equal(draws.length, count);
    renderer.setPoints(new Float32Array(0), 0, 0, false);
    renderer.render();
    assert.equal(stats.clears, 2, 'clearing the cloud must erase the old image');
    assert.equal(renderer.bounds, null);
});

test('camera controls, point size, canvas size and pixel ratio invalidate the image', () => {
    const { renderer, canvas, window, events, stats } = harness();
    renderer.setPoints(new Float32Array([0, 0, 0, 1, 2, 3]), 1, 1, true);
    renderer.render();
    const changes = [
        () => renderer.setView('top'), () => renderer.setView('side'), () => renderer.setView('front'),
        () => renderer.setView('free'), () => renderer.fit(), () => renderer.setPointSize(4),
        () => { canvas.clientWidth += 20; }, () => { canvas.clientHeight += 20; },
        () => { window.devicePixelRatio = 2; },
        () => events.wheel({ preventDefault() {}, deltaY: 1 }),
        () => { events.mousedown({ button: 0, clientX: 10, clientY: 10 }); events.mousemove({ clientX: 20, clientY: 30 }); events.mouseup(); },
        () => { events.mousedown({ button: 2, clientX: 10, clientY: 10 }); events.mousemove({ clientX: 20, clientY: 30 }); events.mouseup(); }
    ];
    for (const change of changes) {
        const before = stats.clears;
        change(); renderer.render(); renderer.render();
        assert.equal(stats.clears, before + 1);
    }
    const before = stats.clears;
    renderer.setPointSize(4);
    events.mousemove({ clientX: 40, clientY: 40 });
    renderer.render();
    assert.equal(stats.clears, before);
});
