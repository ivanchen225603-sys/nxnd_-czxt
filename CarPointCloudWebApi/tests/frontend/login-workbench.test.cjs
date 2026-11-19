const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const sourcePath = path.resolve(__dirname, '../../wwwroot/pointcloud-viewer/js/login.js');
const source = fs.readFileSync(sourcePath, 'utf8');
const authKey = 'car_pointcloud_auth';
const rememberKey = 'car_pointcloud_remember';
const fixtureUser = { id: 7, username: 'fixture-viewer', displayName: '离线查看员',
    roleCode: 'viewer', roleName: '查看员', status: 1 };

function response(payload = { success: true, data: { token: 'offline-fixture-not-a-real-jwt', user: fixtureUser } },
    status = 200) {
    return { ok: status >= 200 && status < 300, status, async json() { return payload; } };
}

function storage(initial = {}) {
    const values = new Map(Object.entries(initial));
    return {
        getItem(key) { return values.has(key) ? values.get(key) : null; },
        setItem(key, value) { values.set(key, String(value)); },
        removeItem(key) { values.delete(key); }
    };
}

function loginPage({ fetch: fetchResult = async () => response(), remembered = null, auth = null,
    apiBase = '' } = {}) {
    const nodes = new Map();
    const listeners = new Map();
    const requests = [];
    const timers = [];
    let document;
    function makeElement() {
        const attributes = new Map();
        const node = {
            value: '', type: 'text', checked: false, disabled: false, dataset: {}, style: {},
            innerHTML: '', textContent: '', className: '', children: [], parentElement: null,
            setAttribute(name, value) { attributes.set(name, String(value)); },
            getAttribute(name) { return attributes.get(name) ?? null; },
            removeAttribute(name) { attributes.delete(name); },
            addEventListener(name, listener) { listeners.set(node.id + ':' + name, listener); },
            focus() { document.activeElement = node; },
            appendChild(child) { node.children.push(child); child.parentElement = node; },
            remove() {
                if (node.parentElement) {
                    const parent = node.parentElement;
                    parent.children = parent.children.filter(child => child !== node);
                    node.parentElement = null;
                }
            }
        };
        function changeClass(name, enabled) {
            const classes = new Set(node.className.split(/\s+/).filter(Boolean));
            enabled ? classes.add(name) : classes.delete(name);
            node.className = [...classes].join(' ');
        }
        node.classList = {
            add(name) { changeClass(name, true); },
            remove(name) { changeClass(name, false); },
            contains(name) { return node.className.split(/\s+/).includes(name); }
        };
        return node;
    }
    function element(id) {
        if (!nodes.has(id)) nodes.set(id, Object.assign(makeElement(), { id }));
        return nodes.get(id);
    }
    document = {
        activeElement: null,
        getElementById: element,
        addEventListener(name, listener) { listeners.set('document:' + name, listener); },
        createElement: makeElement,
        querySelector(selector) {
            if (selector === '.login-card') return element('loginCard');
            if (selector === '.login-toast') {
                return element('loginCard').children.find(child => child.classList.contains('login-toast')) || null;
            }
            return null;
        }
    };
    element('password').type = 'password';
    element('loginForm').setAttribute('aria-busy', 'false');
    element('username').setAttribute('aria-invalid', 'false');
    element('password').setAttribute('aria-invalid', 'false');
    const originalButton = '<span class="btn-label">登录工作台</span>';
    element('loginBtn').innerHTML = originalButton;
    const sessionStorage = storage(auth === null ? {} : { [authKey]: auth });
    const localStorage = storage(remembered === null ? {} : { [rememberKey]: remembered });
    const window = { __API_BASE__: apiBase, location: { href: 'login.html' } };
    const context = vm.createContext({
        document, window, sessionStorage, localStorage,
        console: { error() {} },
        requestAnimationFrame(callback) { callback(); },
        setTimeout(callback, delay) { timers.push({ callback, delay }); return timers.length; },
        async fetch(url, options) {
            requests.push({ url, ...JSON.parse(JSON.stringify(options)) });
            return fetchResult(url, options);
        }
    });
    vm.runInContext(source, context, { filename: sourcePath });
    listeners.get('document:DOMContentLoaded')();
    return {
        element, requests, sessionStorage, localStorage, window, document, originalButton,
        fire(id, event) { return listeners.get(id + ':' + event)({ preventDefault() {} }); },
        submit() { return this.fire('loginForm', 'submit'); },
        runTimers(delay) {
            const ready = timers.filter(timer => timer.delay === delay);
            for (const timer of ready) {
                timers.splice(timers.indexOf(timer), 1);
                timer.callback();
            }
        },
        toast() { return document.querySelector('.login-toast'); }
    };
}

function fillCredentials(page) {
    page.element('username').value = ' fixture-viewer ';
    page.element('password').value = '  offline fixture password  ';
}

test('empty credentials stay local and expose errors that clear when corrected', async () => {
    for (const [username, password] of [['', ''], ['   ', 'fixture'], ['fixture-viewer', '']]) {
        const page = loginPage();
        page.element('username').value = username;
        page.element('password').value = password;
        await page.submit();
        assert.equal(page.requests.length, 0);
        assert.equal(page.sessionStorage.getItem(authKey), null);
        assert.equal(page.window.location.href, 'login.html');
        for (const id of [!username.trim() && 'username', !password && 'password'].filter(Boolean)) {
            assert.match(page.element(id + 'Error').textContent, /请输入/);
            assert.equal(page.element(id).getAttribute('aria-invalid'), 'true');
            page.fire(id, 'input');
            assert.equal(page.element(id + 'Error').textContent, '');
            assert.notEqual(page.element(id).getAttribute('aria-invalid'), 'true');
        }
    }
});

test('successful form submission keeps the existing API and full session identity', async () => {
    const page = loginPage({ apiBase: 'http://offline-fixture.invalid/root/' });
    fillCredentials(page);
    await page.submit();
    assert.equal(page.requests.length, 1);
    const request = page.requests[0];
    assert.equal(request.url, 'http://offline-fixture.invalid/root/api/auth/login');
    assert.equal(request.method, 'POST');
    assert.deepEqual(request.headers, { 'Content-Type': 'application/json' });
    assert.deepEqual(JSON.parse(request.body), {
        username: 'fixture-viewer', password: '  offline fixture password  '
    });
    const auth = JSON.parse(page.sessionStorage.getItem(authKey));
    assert.equal(auth.username, 'fixture-viewer');
    assert.equal(auth.token, 'offline-fixture-not-a-real-jwt');
    assert.deepEqual(auth.user, fixtureUser);
    assert.ok(Number.isFinite(Date.parse(auth.loggedInAt)));
    assert.equal(Object.hasOwn(auth, 'password'), false);
    assert.equal(page.window.location.href, 'login.html');
    assert.match(page.toast().textContent, /登录成功/);
    assert.equal(page.toast().getAttribute('role'), 'status');
    page.runTimers(800);
    assert.equal(page.window.location.href, 'live.html');
});

test('remembering an account persists only its username, never the password or token', async () => {
    const page = loginPage();
    fillCredentials(page);
    page.element('rememberMe').checked = true;
    await page.submit();
    assert.deepEqual(JSON.parse(page.localStorage.getItem(rememberKey)), { username: 'fixture-viewer' });
    assert.equal(page.localStorage.getItem(authKey), null);
});

test('unchecking remember removes an earlier username only after successful authentication', async () => {
    const page = loginPage({ remembered: JSON.stringify({ username: 'previous-fixture' }) });
    fillCredentials(page);
    page.element('rememberMe').checked = false;
    await page.submit();
    assert.equal(page.localStorage.getItem(rememberKey), null);
    assert.equal(JSON.parse(page.sessionStorage.getItem(authKey)).username, 'fixture-viewer');
});

test('a pending login keeps one request in flight and exposes its busy state', async () => {
    let finish;
    const page = loginPage({ fetch: () => new Promise(resolve => { finish = resolve; }) });
    fillCredentials(page);
    const first = page.submit();
    assert.equal(page.element('loginBtn').disabled, true);
    assert.equal(page.element('loginForm').getAttribute('aria-busy'), 'true');
    assert.match(page.element('loginBtn').innerHTML, /登录中/);
    assert.equal(page.sessionStorage.getItem(authKey), null);
    await page.submit();
    assert.equal(page.requests.length, 1);
    finish(response({ success: false, message: '离线拒绝' }, 401));
    await first;
    assert.equal(page.element('loginBtn').disabled, false);
    assert.equal(page.element('loginForm').getAttribute('aria-busy'), 'false');
    assert.equal(page.element('loginBtn').innerHTML, page.originalButton);
});

const failureCases = [
    ['HTTP rejection', () => response({ success: false, message: '账号或密码错误' }, 401), /账号或密码错误/],
    ['business rejection', () => response({ success: false, message: '账号不可用' }), /账号不可用/],
    ['network rejection', () => { throw new Error('offline'); }, /无法连接服务器/],
    ['invalid JSON', () => ({ ok: true, status: 200,
        async json() { throw new Error('invalid JSON'); }, async text() { return 'not-json'; } }), /响应格式错误/],
    ['HTML instead of API data', () => ({ ok: true, status: 200,
        async json() { throw new Error('invalid JSON'); },
        async text() { return '<!DOCTYPE html><html>offline fixture</html>'; } }), /返回了页面而非接口数据/]
];

for (const [name, failedResponse, message] of failureCases) {
    test(name + ' does not establish a session and can be corrected and retried', async () => {
        let attempts = 0;
        const remembered = JSON.stringify({ username: 'previous-fixture' });
        const page = loginPage({ remembered, fetch: () => ++attempts === 1 ? failedResponse() : response() });
        fillCredentials(page);
        page.element('rememberMe').checked = false;
        await page.submit();
        assert.equal(page.requests[0].url, '/api/auth/login');
        assert.equal(page.sessionStorage.getItem(authKey), null);
        assert.equal(page.localStorage.getItem(rememberKey), remembered);
        assert.equal(page.window.location.href, 'login.html');
        assert.equal(page.element('loginBtn').disabled, false);
        assert.equal(page.element('loginForm').getAttribute('aria-busy'), 'false');
        assert.equal(page.element('loginBtn').innerHTML, page.originalButton);
        assert.match(page.element('passwordError').textContent, message);
        assert.equal(page.element('password').getAttribute('aria-invalid'), 'true');
        page.fire('password', 'input');
        assert.equal(page.element('passwordError').textContent, '');
        assert.notEqual(page.element('password').getAttribute('aria-invalid'), 'true');
        await page.submit();
        assert.equal(page.requests.length, 2);
        assert.equal(JSON.parse(page.sessionStorage.getItem(authKey)).token, 'offline-fixture-not-a-real-jwt');
        page.runTimers(800);
        assert.equal(page.window.location.href, 'live.html');
    });
}

test('password visibility preserves its value and updates the accessible toggle state', () => {
    const page = loginPage();
    fillCredentials(page);
    const password = page.element('password');
    const original = password.value;
    page.fire('togglePassword', 'click');
    assert.equal(password.type, 'text');
    assert.equal(password.value, original);
    assert.equal(page.element('togglePassword').getAttribute('aria-pressed'), 'true');
    assert.equal(page.element('togglePassword').getAttribute('aria-label'), '隐藏密码');
    page.fire('togglePassword', 'click');
    assert.equal(password.type, 'password');
    assert.equal(password.value, original);
    assert.equal(page.element('togglePassword').getAttribute('aria-pressed'), 'false');
    assert.equal(page.element('togglePassword').getAttribute('aria-label'), '显示密码');
    assert.equal(page.requests.length, 0);
});

test('forgotten-password help contacts the administrator without requesting credentials or changing accounts', () => {
    const page = loginPage();
    page.fire('forgotLink', 'click');
    assert.match(page.toast().textContent, /联系管理员重置密码/);
    assert.equal(page.toast().getAttribute('role'), 'status');
    assert.equal(page.requests.length, 0);
    assert.equal(page.sessionStorage.getItem(authKey), null);
    assert.equal(page.window.location.href, 'login.html');
    page.runTimers(3000);
    page.runTimers(300);
    assert.equal(page.toast(), null);
});

test('initial remembered username focuses the password without creating a logged-in session', () => {
    const page = loginPage({ remembered: JSON.stringify({ username: 'remembered-fixture' }) });
    assert.equal(page.element('username').value, 'remembered-fixture');
    assert.equal(page.element('rememberMe').checked, true);
    assert.equal(page.element('password').value, '');
    assert.equal(page.document.activeElement, page.element('password'));
    assert.equal(page.sessionStorage.getItem(authKey), null);
    assert.equal(page.requests.length, 0);
});

test('a new login focuses the username and malformed remembered data does not authenticate', () => {
    const fresh = loginPage();
    assert.equal(fresh.document.activeElement, fresh.element('username'));
    const damaged = loginPage({ remembered: '{broken-json' });
    assert.equal(damaged.element('username').value, '');
    assert.equal(damaged.element('rememberMe').checked, false);
    assert.equal(damaged.sessionStorage.getItem(authKey), null);
    assert.equal(damaged.window.location.href, 'login.html');
    assert.equal(damaged.requests.length, 0);
});

test('an existing session retains the original immediate workspace redirect without another login request', () => {
    const auth = JSON.stringify({ username: 'session-fixture', token: 'offline-session' });
    const page = loginPage({ auth });
    assert.equal(page.window.location.href, 'live.html');
    assert.equal(page.sessionStorage.getItem(authKey), auth);
    assert.equal(page.requests.length, 0);
});
