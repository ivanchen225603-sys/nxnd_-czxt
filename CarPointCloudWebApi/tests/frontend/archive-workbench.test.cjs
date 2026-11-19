const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const root = path.resolve(__dirname, '../../wwwroot/pointcloud-viewer');
const sample = {
    licensePlate: '宁A81530M', scanTime: '2026-10-12T10:37:22+08:00',
    pointCloudUrl: '/sample.pcd', pointCount: 42, reviewStatus: 'pending',
    dimensions: { trailerLength: 1067.8, trailerWidth: 226, trailerBodyHeight: 264.5,
        trailerBottomToGroundHeight: 0, cabToTrailerFrontDist: null, highLowPlateHeight: null }
};

function harness(api = { get: async () => ({ data: sample }), arrayBuffer: async () => new ArrayBuffer(0) }) {
    const nodes = new Map();
    const listeners = {};
    const calls = { loaded: 0, resets: 0, screenshots: 0, errors: [] };
    const get = id => {
        if (!nodes.has(id)) {
            const classes = new Set();
            nodes.set(id, { value: '', textContent: '', hidden: false, style: {}, dataset: {},
                disabled: ['resetBtn', 'screenshotBtn'].includes(id), events: {},
                classList: { toggle(name, enabled) { enabled ? classes.add(name) : classes.delete(name); }, contains: name => classes.has(name) },
                addEventListener(type, listener) { this.events[type] = listener; }, focus() {} });
        }
        return nodes.get(id);
    };
    const context = vm.createContext({
        document: { getElementById: get, addEventListener(type, listener) { listeners[type] = listener; } },
        window: {}, console: { error() {} }, API: api,
        formatNumber: (value, digits) => value.toFixed(digits),
        updatePointCount: count => { get('statusPoints').textContent = String(count); },
        updateStatus() {}, showLoading() {}, hideLoading() {},
        showError: message => calls.errors.push(message),
        PointCloudViewer: class {
            loadPointCloud() { calls.loaded++; }
            resetView() { calls.resets++; }
            takeScreenshot() { calls.screenshots++; }
        },
        InteractionController: class {},
        PCDParser: class { parse() { return { pointCount: 42 }; } }
    });
    vm.runInContext(fs.readFileSync(path.join(root, 'js/main.js'), 'utf8'), context);
    listeners.DOMContentLoaded();
    return { context, get, calls, listeners };
}

test('archive dimensions preserve missing versus actual zero, with conditional review status', () => {
    const h = harness();
    h.context.renderEmptyVehicleProfile(sample);
    assert.equal(h.get('dimension-length').textContent, '1067.8');
    assert.equal(h.get('dimension-bottom').textContent, '0.0');
    assert.equal(h.get('dimension-cab').textContent, '--');
    assert.equal(h.get('dimension-high-low').textContent, '--');
    assert.equal(h.get('profile-plate').textContent, sample.licensePlate);
    assert.equal(h.get('profile-quality').textContent, '已保存，待人工审核');
    assert.equal(h.get('profile-quality').classList.contains('is-pending'), true);
    h.context.renderEmptyVehicleProfile({ ...sample, reviewStatus: null, dimensions: { trailerWidth: '', trailerLength: NaN } });
    assert.equal(h.get('dimension-length').textContent, '--');
    assert.equal(h.get('dimension-width').textContent, '--');
    assert.equal(h.get('profile-quality').textContent, '已保存');
    assert.equal(h.get('profile-quality').classList.contains('is-pending'), false);
});

test('archive query uses existing exact-plate and PCD APIs, suppressing duplicate Enter while loading', async () => {
    let finish;
    const requests = [];
    const h = harness({
        get: url => { requests.push(url); return new Promise(resolve => { finish = resolve; }); },
        arrayBuffer: async url => { requests.push(url); return new ArrayBuffer(0); }
    });
    h.get('plateInput').value = ' 宁a81530m ';
    const pending = h.get('searchPlateBtn').events.click();
    assert.equal(h.get('canvas-wrapper').dataset.state, 'loading');
    assert.equal(h.get('archiveCanvasNotice').hidden, false);
    assert.equal(h.get('screenshotBtn').disabled, true);
    h.get('plateInput').events.keydown({ key: 'Enter', preventDefault() {} });
    assert.equal(requests.length, 1);
    finish({ data: sample });
    await pending;
    assert.deepEqual(requests, ['/api/empty-vehicle-profiles/' + encodeURIComponent(sample.licensePlate), '/sample.pcd']);
    assert.equal(h.calls.loaded, 1);
    assert.equal(h.get('canvas-wrapper').dataset.state, 'loaded');
    assert.equal(h.get('archiveCanvasNotice').hidden, true);
    assert.equal(h.get('archiveCloudLabel').textContent, '已保存云图 · 非实时');
    assert.equal(h.get('searchPlateBtn').disabled, false);
    assert.equal(h.get('screenshotBtn').disabled, false);
});

test('failed archive lookup masks the prior cloud, hides prior dimensions, and disables old-cloud screenshot', async () => {
    let fail = false;
    const h = harness({ get: async () => { if (fail) throw new Error('档案不存在'); return { data: sample }; }, arrayBuffer: async () => new ArrayBuffer(0) });
    h.get('plateInput').value = sample.licensePlate;
    await h.get('searchPlateBtn').events.click();
    fail = true;
    h.get('plateInput').value = '宁A52522S';
    await h.get('searchPlateBtn').events.click();
    assert.equal(h.get('canvas-wrapper').dataset.state, 'error');
    assert.equal(h.get('archiveCanvasNotice').hidden, false);
    assert.equal(h.get('archiveNoticeMessage').textContent, '档案不存在');
    assert.equal(h.get('empty-vehicle-panel').style.display, 'none');
    assert.equal(h.get('bbox-panel').style.display, 'none');
    assert.equal(h.get('resetBtn').disabled, true);
    assert.equal(h.get('screenshotBtn').disabled, true);
    assert.equal(h.get('searchPlateBtn').disabled, false);
    assert.equal(h.calls.loaded, 1);
});

test('archive R/S shortcuts still work on the canvas without intercepting plate entry', () => {
    const h = harness();
    h.context.setArchiveState('loaded');
    h.listeners.keydown({ key: 's', target: { closest: () => ({}) } });
    h.listeners.keydown({ key: 'r', target: { closest: () => ({}) } });
    assert.equal(h.calls.screenshots, 0);
    assert.equal(h.calls.resets, 0);
    h.listeners.keydown({ key: 's', target: { closest: () => null } });
    h.listeners.keydown({ key: 'r', target: { closest: () => null } });
    assert.equal(h.calls.screenshots, 1);
    assert.equal(h.calls.resets, 1);
    h.context.setArchiveState('error');
    h.listeners.keydown({ key: 's', target: { closest: () => null } });
    assert.equal(h.calls.screenshots, 1);
});

test('archive framing keeps an offset cloud inside desktop and narrow viewports in all orientations', () => {
    const context = vm.createContext({ console: { log() {} }, document: { getElementById() {} } });
    for (const file of ['vendor/three-r128/three.min.js', 'js/pcd-parser.js', 'js/viewer.js']) {
        vm.runInContext(fs.readFileSync(path.join(root, file), 'utf8'), context);
    }
    const THREE = context.THREE;
    const Viewer = vm.runInContext('PointCloudViewer', context);
    const corners = [];
    for (const x of [3, 13.678]) for (const y of [2, 4.26]) for (const z of [1.5, 4.145]) corners.push(x, y, z);
    for (const [width, height] of [[870, 728], [380, 560], [340, 440]]) {
        for (const orientation of ['auto', 'landscape', 'portrait']) {
            const viewer = Object.create(Viewer.prototype);
            Object.assign(viewer, { camera: new THREE.PerspectiveCamera(75, width / height, .01, 1000000),
                renderer: { domElement: { width, height } }, scene: new THREE.Scene(),
                controls: { target: new THREE.Vector3(), update() {} }, pointCloudGroup: new THREE.Group(), gridHelper: new THREE.GridHelper(1, 1) });
            viewer.fitCameraToObject(new Float32Array(corners), { orientation });
            viewer.camera.updateMatrixWorld();
            viewer.pointCloudGroup.updateMatrixWorld();
            for (let i = 0; i < corners.length; i += 3) {
                const projected = new THREE.Vector3(...corners.slice(i, i + 3)).applyMatrix4(viewer.pointCloudGroup.matrixWorld).project(viewer.camera);
                assert.ok(Math.abs(projected.x) < 1 && Math.abs(projected.y) < 1, `${width}x${height} ${orientation}: corner outside viewport`);
            }
        }
    }
});
