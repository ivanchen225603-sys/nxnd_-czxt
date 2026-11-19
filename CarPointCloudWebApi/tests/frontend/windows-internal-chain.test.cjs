const { test } = require('node:test');
const assert = require('node:assert/strict');
const { spawnSync } = require('node:child_process');
const fs = require('node:fs');
const os = require('node:os');
const path = require('node:path');
const script = path.resolve(__dirname, '../../../scripts/windows-internal-chain.ps1');

test('launcher never injects environment credentials or business overrides', () => {
    const source = fs.readFileSync(script, 'utf8');
    assert.doesNotMatch(source, /SetEnvironmentVariable|\$env:|DOTNET_ENVIRONMENT|ASPNETCORE_ENVIRONMENT|RuntimeConfigPath|LadarCapture__/);
    assert.match(source, /appsettings\.json/);
    assert.match(source, /-WindowStyle Hidden/);
    assert.match(source, /port_in_use_not_restarted/);
    assert.match(source, /DatabaseConnectionVerified=\$false/);
});

test('Windows PowerShell 5 reads UTF8 script and missing configuration fails without leaking secrets', { skip: process.platform !== 'win32' }, () => {
    const root = fs.mkdtempSync(path.join(os.tmpdir(), 'carpoint-start-'));
    const scripts = path.join(root, 'scripts');
    const project = path.join(root, 'CarPointCloudWebApi');
    fs.mkdirSync(scripts);
    fs.mkdirSync(project);
    const copy = path.join(scripts, path.basename(script));
    fs.copyFileSync(script, copy);
    const run = () => spawnSync('powershell.exe', ['-NoProfile', '-NonInteractive', '-File', copy, '-Component', 'WebApi'], {
        encoding: 'utf8', windowsHide: true, timeout: 20000
    });
    try {
        const missing = run();
        assert.equal(missing.status, 1);
        assert.match(missing.stderr, /appsettings.json/);
        fs.writeFileSync(path.join(project, 'appsettings.json'), '{ secret-do-not-print');
        const malformed = run();
        assert.equal(malformed.status, 1);
        assert.ok(!(malformed.stdout + malformed.stderr).includes('secret-do-not-print'));
        assert.deepEqual(fs.readdirSync(project), ['appsettings.json']);
    } finally { fs.rmSync(root, { recursive: true, force: true }); }
});
