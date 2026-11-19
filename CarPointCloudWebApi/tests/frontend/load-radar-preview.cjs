/**
 * 离线界面验收专用服务器，不是真业务服务，也不验证真实 JWT、雷达或 DM8。
 * 运行：node tests/frontend/load-radar-preview.cjs
 * 仅监听 127.0.0.1:18761，所有写请求均返回 405；不读取 appsettings 或任何业务凭据。
 * 测试登录态与醒目标识只在 HTML 响应中注入，不修改生产静态资源。
 */
const http = require('node:http');
const fs = require('node:fs');
const path = require('node:path');

const HOST = '127.0.0.1';
const PORT = 18761;
const STATIC_ROOT = fs.realpathSync(path.resolve(__dirname, '../../wwwroot/pointcloud-viewer'));
const API_ROOT = '/api/v1/load-radar';
// 可选显式参考文件，仅供本地视觉对照；不暴露任意目录，不进入生产资源。
const referenceHtml = process.argv[2] ? fs.readFileSync(process.argv[2], 'utf8') : null;
const startedAt = Date.now();
const sha = 'a'.repeat(64);
const iso = offset => new Date(startedAt + offset).toISOString();

// 固定公式构造点云，仅用于检查高度配色、固定 ROI 和交互，不作为算法样本。
function makeCloud(loaded) {
    const points = [];
    for (let iy = 0; iy <= 88; iy++) {
        const y = -5.5 + iy * 0.125;
        for (let ix = 0; ix <= 20; ix++) {
            const x = -1.25 + ix * 0.125;
            const z = loaded ? -2.3 + 0.8 * Math.cos(x) * Math.cos(y * 0.22) : -4.1 + y * 0.025;
            points.push([x, y, Number(z.toFixed(4))]);
        }
        for (let iz = 0; iz <= 15; iz++) {
            const z = -4.1 + iz * 0.14;
            points.push([-1.3, y, Number(z.toFixed(4))], [1.3, y, Number(z.toFixed(4))]);
        }
    }
    return points;
}
const emptyCloud = makeCloud(false);
const loadedCloud = makeCloud(true);
const binding = {
    passageKey: 'fixture-pass-loaded', workflowId: 'fixture-workflow-loaded', licensePlate: '宁A96523',
    cameraIdentifier: 'FIXTURE_CAM_01', vehicleIdentity: 'fixture-tractor-01/trailer-01',
    actor: '离线验收账号', reason: '固定展示数据，不代表真实车牌关联', boundAt: iso(-5000)
};
const emptyMeasurement = {
    measurementId: 'fixture-empty-001', nodeId: 'FIXTURE_NODE', channelId: 'CH1', passageId: 'fixture-pass-empty',
    calibrationId: 'fixture-calibration', sourceMode: 'live', algorithmVersion: 'fixture-only', configurationHash: 'b'.repeat(64),
    baselineCandidateSha256: sha, measuredAt: iso(-120000), verdict: 'empty', frameDegraded: false,
    volumeStatus: 'unavailable', volumeM3: null, baselineVersionId: null, filledRatio: null, boundSpread: null,
    holeRatio: null, pointCount: emptyCloud.length, reason: '空车（离线界面固定样例，不是实际判定）', points: emptyCloud
};
const loadedMeasurement = {
    ...emptyMeasurement, measurementId: 'fixture-loaded-002', passageId: 'fixture-pass-loaded',
    baselineCandidateSha256: null, measuredAt: iso(-30000), verdict: 'loaded', volumeStatus: 'success', volumeM3: 55.234,
    baselineVersionId: 'fixture-baseline-v1', filledRatio: 0.32, boundSpread: 0.021, holeRatio: 0.06,
    pointCount: loadedCloud.length, reason: '底面内点占比过低（离线界面固定样例）', points: loadedCloud
};
const liveRecords = Array.from({ length: 4 }, (_, channelIndex) => [0, 1].map(vehicleIndex => {
    const channelId = 'CH' + (channelIndex + 1), loaded = (channelIndex + vehicleIndex) % 2 === 0;
    const source = loaded ? loadedMeasurement : emptyMeasurement;
    const plate = '宁A' + String(10001 + channelIndex * 10 + vehicleIndex);
    const measurementId = `fixture-live-${channelIndex + 1}-${vehicleIndex + 1}`;
    const passageId = `fixture-pass-${channelIndex + 1}-${vehicleIndex + 1}`;
    const measurement = { ...source, measurementId, channelId, passageId,
        measuredAt: iso(vehicleIndex === 0 ? -30000 - channelIndex * 1000 : -310000 - channelIndex * 1000),
        verdict: loaded ? 'loaded' : 'empty', volumeStatus: loaded ? 'success' : 'unavailable',
        volumeM3: loaded ? Number((48.2 + channelIndex * 2.1 + vehicleIndex * 0.8).toFixed(3)) : null,
        pointCount: loaded ? loadedCloud.length : emptyCloud.length,
        reason: loaded ? '底面内点占比不足，保留本次体积样例' : '空车，本次体积为空值样例',
        points: loaded ? loadedCloud : emptyCloud };
    return { passageKey: passageId, payloadHash: String(channelIndex + vehicleIndex + 1).repeat(64).slice(0, 64),
        measurement, receivedAt: measurement.measuredAt,
        binding: { ...binding, passageKey: passageId, workflowId: `fixture-workflow-${channelIndex + 1}-${vehicleIndex + 1}`,
            licensePlate: plate, cameraIdentifier: `FIXTURE_CAM_0${channelIndex + 1}`, boundAt: measurement.measuredAt },
        publication: null };
})).flat();
const records = [...liveRecords].sort((a, b) => Date.parse(b.measurement.measuredAt) - Date.parse(a.measurement.measuredAt));
const baseline = {
    versionId: 'fixture-baseline-v1', scopeKey: 'fixture-scope', nodeId: 'FIXTURE_NODE', channelId: 'CH1',
    calibrationId: 'fixture-calibration', vehicleIdentity: binding.vehicleIdentity, licensePlate: binding.licensePlate,
    measurementId: emptyMeasurement.measurementId, assetSha256: sha, actor: '离线验收账号',
    reason: '仅展示版本列表，不存在真实基线资产', createdAt: iso(-100000), active: true, activatedAt: iso(-95000)
};
const versions = [baseline, { ...baseline, versionId: 'fixture-baseline-v2', active: false, activatedAt: null, createdAt: iso(-80000) }];

function state() {
    const now = Date.now() / 1000;
    return {
        enabled: true,
        lanes: Array.from({ length: 4 }, (_, index) => {
            const current = liveRecords.find(row => row.measurement.channelId === 'CH' + (index + 1));
            const loaded = current.measurement.verdict === 'loaded';
            const gates = Array.from({ length: 11 }, (_, step) => ({
                step: step + 1, name: ['点数', '包围盒', '底面拟合', '法向', '跨度', '内点数', '底面内点占比', '壁面', '高度', '底面余高', '空车结论'][step],
                state: loaded ? (step < 6 ? 'pass' : step === 6 ? 'block' : 'skip') : 'pass',
                value: step === 6 ? (loaded ? 0.36 : 0.85) : null, op: '>=', limit: step === 6 ? 0.7 : null, margin: step === 6 ? (loaded ? -0.34 : 0.15) : null
            }));
            return {
                nodeId: 'FIXTURE_NODE', channelId: 'CH' + (index + 1), name: (index + 1) + '号通道',
                cameraIdentifier: 'FIXTURE_CAM_0' + (index + 1), online: true, error: null,
                state: {
                    id: 'CH' + (index + 1), online: true, mode: 'simulate', source_kind: '离线样例', frame_age_sec: 0.1,
                    fps: 9.8, frames: 520 + index, last_plate: current.binding.licensePlate, saved_count: 12 + index, session: 3,
                    updated: now, save_enabled: false, baseline: index === 0, cloud: loaded ? loadedCloud : emptyCloud,
                    roi: [[-2, -8, -6], [2, 8, 0]], stability: { deviation: 0.012 + index * 0.002, state: 2 },
                    current_result: { measurement_id: current.measurement.measurementId },
                    health: '',
                    source: { source_ip: '127.0.0.1', packet_rate: 782, rpm: 600, frame_mode: 'rotation' },
                    last_result: {
                        empty: !loaded, reason: loaded ? '底面内点占比过低（展示样例）' : '空车（展示样例）',
                        n_points: loaded ? loadedCloud.length : emptyCloud.length, ts: now - 1, frame_degraded: false,
                        gates, diagnostics: { floor_tilt_deg: 3.8, freeboard: loaded ? 0.6 : 2.1 },
                        planes: { floor: { a: 0, b: 0.03, c: 1, d: 4.1, inliers: 1680, rms: 0.014, valid: true } },
                        load_profile: null,
                        volume: loaded ? { ok: true, volume_m3: 55.234, filled_ratio: 0.32, hole_ratio: 0.06, bound_spread: 0.021, text: '仅用于界面验收的体积样例' }
                            : { ok: false, volume_m3: 0, message: '无可用体积（验证占位零不显示为测量）' }
                    }
                }
            };
        })
    };
}

const injection = `<script>
window.__API_BASE__ = window.location.origin;
sessionStorage.setItem('car_pointcloud_auth', JSON.stringify({ username: 'fixture-admin', token: 'offline-fixture-not-a-real-jwt', user: { username: 'fixture-admin', roleCode: 'admin', roleName: '离线验收管理员' }, loggedInAt: new Date().toISOString() }));
</script><style>.fixture-banner{position:fixed;bottom:0;left:0;right:0;z-index:9999;padding:4px 12px;background:#6b3906;color:#fff3d5;font:600 12px/1.6 sans-serif;pointer-events:none}</style>`;
const banner = '<div class="fixture-banner" role="note">离线界面验收 FIXTURE：全部点云、车牌、体积与版本均为合成展示数据。未连接雷达或数据库；所有写入请求均被拒绝。</div>';
const mime = { '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8', '.css': 'text/css; charset=utf-8', '.png': 'image/png', '.svg': 'image/svg+xml', '.ico': 'image/x-icon' };

function send(response, status, body, type = 'application/json; charset=utf-8', head = false) {
    const bytes = Buffer.isBuffer(body) ? body : Buffer.from(body);
    response.writeHead(status, {
        'Content-Type': type, 'Content-Length': bytes.length, 'Cache-Control': 'no-store', 'X-Content-Type-Options': 'nosniff',
        'Content-Security-Policy': "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; img-src 'self' data: blob:; connect-src 'self'; object-src 'none'; frame-ancestors 'none'"
    });
    response.end(head ? undefined : bytes);
}
function json(response, data, head = false) { send(response, 200, JSON.stringify({ code: 200, message: '离线界面样例', data, success: true }), undefined, head); }
function fail(response, status, message) { send(response, status, JSON.stringify({ code: status, message, data: null, success: false })); }

const server = http.createServer((request, response) => {
    // 静态页面和假 API 均只读；测试管理员不会获得任何真实写入能力。
    if (!['GET', 'HEAD'].includes(request.method)) {
        request.resume(); return fail(response, 405, '离线界面验收服务器拒绝所有写入，不会修改真实数据');
    }
    const head = request.method === 'HEAD';
    try {
        const url = new URL(request.url, `http://${HOST}:${PORT}`);
        const pathname = decodeURIComponent(url.pathname);
        if (referenceHtml && pathname === '/reference.html') return send(response, 200, referenceHtml.replace(/(<body\b[^>]*>)/i, '$1' + banner), 'text/html; charset=utf-8', head);
        if (referenceHtml && (pathname === '/api/state' || pathname === '/api/events')) {
            const snapshot = { service: { channels: 4, online: 4, saved_total: 0, uptime: 1, subscribers: 1 },
                channels: state().lanes.map((lane, index) => ({ ...lane.state, name: lane.name, source_kind: 'live',
                    last_plate: liveRecords[index].binding.licensePlate, last_result: { ...lane.state.last_result, session: lane.state.session } })) };
            // 同一批假点云供两种布局对照；参考脚本的 SSE 保持只读连接。
            if (pathname.endsWith('/events') && !head) {
                response.writeHead(200, { 'Content-Type': 'text/event-stream', 'Cache-Control': 'no-store' });
                response.write('event: snapshot\ndata: ' + JSON.stringify(snapshot) + '\n\n');
                return;
            }
            return send(response, 200, JSON.stringify(snapshot), 'application/json; charset=utf-8', head);
        }
        if (pathname === API_ROOT + '/state') return json(response, state(), head);
        if (pathname === API_ROOT + '/baselines') return json(response, versions, head);
        if (pathname === API_ROOT + '/history') {
            let rows = records;
            const before = url.searchParams.get('beforeId');
            if (before) { const index = rows.findIndex(r => r.measurement.measurementId === before); rows = index >= 0 ? rows.slice(index + 1) : []; }
            for (const key of ['nodeId', 'channelId']) if (url.searchParams.get(key)) rows = rows.filter(r => r.measurement[key] === url.searchParams.get(key));
            if (url.searchParams.get('licensePlate')) rows = rows.filter(r => r.binding?.licensePlate === url.searchParams.get('licensePlate'));
            const from = url.searchParams.get('from'), to = url.searchParams.get('to');
            if ((from && !Number.isFinite(Date.parse(from))) || (to && !Number.isFinite(Date.parse(to))) ||
                (from && to && Date.parse(from) > Date.parse(to))) return fail(response, 400, '时间范围无效');
            if (from) rows = rows.filter(r => Date.parse(r.measurement.measuredAt) >= Date.parse(from));
            if (to) rows = rows.filter(r => Date.parse(r.measurement.measuredAt) <= Date.parse(to));
            return json(response, rows.slice(0, Math.min(50, Math.max(1, Number(url.searchParams.get('take')) || 50))), head);
        }
        if (pathname.startsWith(API_ROOT + '/measurements/')) {
            const id = pathname.slice((API_ROOT + '/measurements/').length);
            const found = records.find(r => r.measurement.measurementId === id);
            return found ? json(response, found, head) : fail(response, 404, '没有该测试测量');
        }
        if (pathname.startsWith('/api/')) return fail(response, 404, '验收夹具未提供该接口');
        let relative;
        if (pathname === '/') relative = 'load-radar.html';
        else if (pathname.startsWith('/pointcloud-viewer/')) relative = pathname.slice('/pointcloud-viewer/'.length) || 'load-radar.html';
        else return fail(response, 404, '仅提供 pointcloud-viewer 静态资源');
        if (/[\x00:\\]/.test(relative)) return fail(response, 400, '非法资源路径');
        const absolute = path.resolve(STATIC_ROOT, relative);
        const prefix = STATIC_ROOT.toLowerCase() + path.sep;
        if (!absolute.toLowerCase().startsWith(prefix)) return fail(response, 403, '资源路径超出静态目录');
        const real = fs.realpathSync(absolute);
        if (!real.toLowerCase().startsWith(prefix) || !fs.statSync(real).isFile()) return fail(response, 403, '资源不在允许目录');
        const extension = path.extname(real).toLowerCase(), contentType = mime[extension];
        if (!contentType) return fail(response, 403, '不提供此类型文件');
        let body = fs.readFileSync(real);
        if (extension === '.html') body = Buffer.from(body.toString('utf8').replace(/<head>/i, '<head>' + injection).replace(/(<body\b[^>]*>)/i, '$1' + banner));
        return send(response, 200, body, contentType, head);
    } catch (error) {
        return fail(response, error.code === 'ENOENT' ? 404 : 400, '静态资源不存在或请求路径无效');
    }
});
server.on('error', error => { console.error('离线验收服务器启动失败：' + error.code); process.exitCode = 1; });
server.listen(PORT, HOST, () => console.log(`离线界面验收已启动：http://${HOST}:${PORT}/pointcloud-viewer/load-radar.html（全部写入被禁用）`));
function stop() { server.close(() => process.exit(0)); server.closeAllConnections(); }
process.on('SIGINT', stop);
process.on('SIGTERM', stop);
