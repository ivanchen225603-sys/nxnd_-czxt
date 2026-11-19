const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const source = fs.readFileSync(path.resolve(__dirname,
    '../../wwwroot/pointcloud-viewer/js/users.js'), 'utf8');

async function usersPage({ role = 'admin', failUsers = false,
    prompt = () => null, confirm = () => false } = {}) {
    const elements = new Map();
    const listeners = new Map();
    const requests = [];
    const getElement = id => {
        if (!elements.has(id)) {
            const classes = new Set();
            elements.set(id, {
                value: '', style: {}, disabled: false, innerHTML: '', textContent: '',
                focus() {}, setAttribute() {},
                classList: {
                    toggle(name, enabled) { enabled ? classes.add(name) : classes.delete(name); },
                    remove(name) { classes.delete(name); }
                },
                addEventListener(event, handler) { listeners.set(id + ':' + event, handler); }
            });
        }
        return elements.get(id);
    };
    const API = {
        getAuth: () => ({ token: 'test-only' }),
        setAuth() {},
        async get(url) {
            requests.push({ method: 'GET', url });
            if (url === '/api/auth/me') return { data: { username: 'admin', roleCode: role } };
            if (url === '/api/roles') return { data: [] };
            if (failUsers) throw new Error('offline');
            return { data: { items: [{ id: 3, username: 'viewer01', displayName: '只读查看',
                roleCode: 'viewer', status: 1 }], total: 1 } };
        },
        async post(url, body) { requests.push({ method: 'POST', url, body }); },
        async put(url, body) { requests.push({ method: 'PUT', url, body }); },
        async del(url) { requests.push({ method: 'DELETE', url }); }
    };
    const context = vm.createContext({
        API, URLSearchParams, console, prompt, confirm, setTimeout() {}, clearTimeout() {},
        window: { location: {} },
        document: {
            getElementById: getElement,
            addEventListener(event, handler) { listeners.set('document:' + event, handler); },
            body: { appendChild() {} },
            createElement() {
                return { textContent: '', style: {}, get innerHTML() { return this.textContent; } };
            }
        }
    });
    vm.runInContext(source, context);
    await listeners.get('document:DOMContentLoaded')();
    return { elements, requests, getElement, listeners };
}

test('new-account form restores editable username after editing an existing account', async () => {
    const page = await usersPage();
    await page.listeners.get('usersTableBody:click')({ target: {
        closest: () => ({ dataset: { action: 'edit', id: '3' } })
    } });
    assert.equal(page.getElement('username').disabled, true);
    page.listeners.get('addUserBtn:click')();
    assert.equal(page.getElement('username').disabled, false);
    assert.equal(page.getElement('userId').value, '');
    assert.equal(page.getElement('password').placeholder, '12–128 位');
});

test('new account rejects passwords outside the existing 12–128 character contract', async () => {
    const page = await usersPage();
    page.listeners.get('addUserBtn:click')();
    page.getElement('username').value = 'newviewer';
    for (const length of [11, 129]) {
        page.getElement('password').value = 'x'.repeat(length);
        await page.listeners.get('userForm:submit')({ preventDefault() {} });
        assert.equal(page.requests.some(request => request.method === 'POST'), false);
        assert.match(page.getElement('passwordError').textContent, /12–128/);
    }
    page.getElement('password').value = 'x'.repeat(12);
    await page.listeners.get('userForm:submit')({ preventDefault() {} });
    const request = page.requests.find(item => item.method === 'POST');
    assert.equal(request.url, '/api/users');
    assert.equal(request.body.password.length, 12);
    assert.equal(request.body.roleCode, 'viewer');
});

test('editing keeps the username immutable and omits unchanged password from the update', async () => {
    const page = await usersPage();
    await page.listeners.get('usersTableBody:click')({ target: {
        closest: () => ({ dataset: { action: 'edit', id: '3' } })
    } });
    page.getElement('displayName').value = '复核查看';
    await page.listeners.get('userForm:submit')({ preventDefault() {} });
    const request = page.requests.find(item => item.method === 'PUT');
    assert.equal(request.url, '/api/users/3');
    assert.equal(request.body.displayName, '复核查看');
    assert.equal(Object.hasOwn(request.body, 'username'), false);
    assert.equal(Object.hasOwn(request.body, 'newPassword'), false);
});

test('authoritative viewer role prevents loading or binding account-management actions', async () => {
    const page = await usersPage({ role: 'viewer' });
    assert.deepEqual(page.requests.map(request => request.url), ['/api/auth/me']);
    assert.equal(page.listeners.has('addUserBtn:click'), false);
});

test('failed account query reports unknown count rather than an empty successful list', async () => {
    const page = await usersPage({ failUsers: true });
    assert.match(page.getElement('usersTableBody').innerHTML, /加载失败：offline/);
    assert.equal(page.getElement('pageInfo').textContent, '加载未完成，记录数量未知');
    assert.equal(page.getElement('pageBtns').innerHTML, '');
    assert.equal(page.getElement('usersEmpty').style.display, 'none');
});

async function clickRowAction(page, action) {
    await page.listeners.get('usersTableBody:click')({ target: {
        closest: () => ({ dataset: { action, id: '3' } })
    } });
}

test('cancelled or empty password-reset prompt never updates the account', async () => {
    for (const value of [null, '']) {
        const page = await usersPage({ prompt: () => value });
        await clickRowAction(page, 'reset');
        assert.equal(page.requests.some(request => request.method === 'PUT'), false);
    }
});

test('password reset rejects both sides of the 12–128 character boundary without dispatch', async () => {
    for (const length of [11, 129]) {
        const page = await usersPage({ prompt: () => 'x'.repeat(length) });
        await clickRowAction(page, 'reset');
        assert.equal(page.requests.some(request => request.method === 'PUT'), false);
    }
});

test('password reset accepts both valid boundaries and sends only newPassword to the selected user', async () => {
    for (const length of [12, 128]) {
        let promptMessage;
        const password = 'x'.repeat(length);
        const page = await usersPage({ prompt: message => {
            promptMessage = message;
            return password;
        } });
        await clickRowAction(page, 'reset');
        assert.match(promptMessage, /viewer01/);
        const updates = page.requests.filter(request => request.method === 'PUT');
        assert.equal(updates.length, 1);
        assert.equal(updates[0].url, '/api/users/3');
        assert.deepEqual(Object.keys(updates[0].body), ['newPassword']);
        assert.equal(updates[0].body.newPassword, password);
        assert.equal(page.requests.some(request => request.method === 'POST' || request.method === 'DELETE'), false);
    }
});

test('cancelling irreversible account deletion sends no delete request', async () => {
    let confirmMessage;
    const page = await usersPage({ confirm: message => {
        confirmMessage = message;
        return false;
    } });
    await clickRowAction(page, 'delete');
    assert.match(confirmMessage, /viewer01/);
    assert.match(confirmMessage, /不可恢复/);
    assert.equal(page.requests.some(request => request.method === 'DELETE'), false);
});

test('confirmed account deletion targets only the selected account and refreshes the list', async () => {
    const page = await usersPage({ confirm: () => true });
    await clickRowAction(page, 'delete');
    const deletions = page.requests.filter(request => request.method === 'DELETE');
    assert.deepEqual(deletions, [{ method: 'DELETE', url: '/api/users/3' }]);
    assert.equal(page.requests.filter(request => request.method === 'GET' && request.url.startsWith('/api/users?')).length, 2);
});
