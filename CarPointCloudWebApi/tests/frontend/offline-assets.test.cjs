const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const root = path.resolve(__dirname, '../../wwwroot/pointcloud-viewer');

test('all viewer HTML scripts and styles resolve locally for offline deployment', () => {
    for (const file of fs.readdirSync(root).filter(f => f.endsWith('.html'))) {
        const html = fs.readFileSync(path.join(root, file), 'utf8');
        const resources = [...html.matchAll(/<(?:script|link)\b[^>]*?\b(?:src|href)=["']([^"']+)["']/gi)];
        for (const [, resource] of resources) {
            assert.doesNotMatch(resource, /^(?:[a-z]+:)?\/\//i, `${file}: external resource ${resource}`);
            const local = path.resolve(root, resource.split(/[?#]/)[0]);
            assert.ok(local.startsWith(root + path.sep), `${file}: resource outside viewer`);
            assert.ok(fs.statSync(local).isFile(), `${file}: missing ${resource}`);
        }
    }
});

test('bundled Three r128 and matching OrbitControls load in browser-global order without network', () => {
    const html = fs.readFileSync(path.join(root, 'index.html'), 'utf8');
    const scripts = [...html.matchAll(/<script\b[^>]*src="([^"]+)"/g)].map(m => m[1]);
    const three = scripts.indexOf('vendor/three-r128/three.min.js');
    const controls = scripts.indexOf('vendor/three-r128/OrbitControls.js');
    const viewer = scripts.findIndex(s => s.startsWith('js/viewer.js'));
    assert.ok(three >= 0 && controls > three && viewer > controls);
    const context = vm.createContext({ console });
    for (const script of scripts.slice(three, controls + 1)) {
        vm.runInContext(fs.readFileSync(path.join(root, script), 'utf8'), context);
    }
    assert.equal(context.THREE.REVISION, '128');
    assert.equal(typeof context.THREE.OrbitControls, 'function');
    const scene = vm.runInContext('new THREE.Scene()', context);
    assert.equal(scene.isScene, true);
    assert.match(fs.readFileSync(path.join(root, 'vendor/three-r128/LICENSE'), 'utf8'), /MIT License/);
});
