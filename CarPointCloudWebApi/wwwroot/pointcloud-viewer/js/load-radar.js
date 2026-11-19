/* 样式来自四路仿真看板；正式页面只读后台数据，不包含模拟或调试控制。 */
(function () {
    'use strict';
    const ROOT = '/api/v1/load-radar', PAGE_SIZE = 50, LOG_PAGE_SIZE = 50;
    const LOG_REFRESH_MS = 15000;
    const FIXED_CHANNELS = [
        { id: 'CH1', label: '1号通道' }, { id: 'CH2', label: '2号通道' },
        { id: 'CH3', label: '3号通道' }, { id: 'CH4', label: '4号通道' }
    ];
    const dom = {}, lanes = new Map(), views = new Map(), logView = {};
    let stopped = false, stateBusy = false, timer, historyBusy = false;
    let stateAvailable = false, selectedKey = '', detailGeneration = 0, historyCloud = null;
    let records = [], cursor = null, historyGeneration = 0, historyQuery = '', pendingHistoryQuery = null;
    let laneOptionsSignature = '', allTab;
    const laneKey = lane => typeof lane?.channelId === 'string' ? lane.channelId : '';
    function element(tag, text, className) {
        const node = document.createElement(tag);
        if (text !== undefined) node.textContent = String(text);
        if (className) node.className = className;
        return node;
    }
    function number(value) { return typeof value === 'number' && Number.isFinite(value) && value >= 0 ? value.toFixed(3) : '--'; }
    function when(value) {
        if (value === undefined || value === null || value === '') return '--';
        const date = new Date(typeof value === 'number' ? value * 1000 : value);
        return Number.isFinite(date.getTime()) ? date.toLocaleString('zh-CN', { hour12: false }) : '--';
    }
    function textNotice(value) {
        if (typeof value === 'string') return value;
        if (Array.isArray(value)) return value.map(textNotice).filter(Boolean).join('；');
        if (value && typeof value === 'object') return textNotice(value.message || value.alerts);
        return '';
    }
    function measurementStatus(m) {
        if (m.verdict === 'empty') return '空车';
        if (m.verdict === 'unknown') return '无法确认载货状态';
        if (['success', 'low_confidence'].includes(m.volumeStatus) && number(m.volumeM3) !== '--') return '计算完成';
        if (m.frameDegraded) return '扫描不完整';
        if (m.volumeStatus === 'failed') return '计算失败';
        return '暂时无法计算';
    }
    function volumeText(m) {
        if (m?.verdict !== 'loaded' || !['success', 'low_confidence'].includes(m.volumeStatus)) return '--';
        const value = number(m.volumeM3);
        return value;
    }
    function laneLabel(channelId) { return FIXED_CHANNELS.find(x => x.id === channelId)?.label || '通道'; }
    async function read(path) { return (await API.request(ROOT + path)).data; }
    function resetResult(v, status) {
        v.status.textContent = status; v.plate.textContent = '--'; v.volume.textContent = '--';
        v.time.textContent = '--'; v.note.textContent = '';
        if (v.estimate) v.estimate.hidden = true;
        v.volume.className = 'v result-volume';
        if (v.verdict) v.verdict.className = 'verdict is-wait';
    }
    function showRecord(v, row) {
        const m = row.latestRevision?.calculation || row.measurement;
        v.status.textContent = measurementStatus(m);
        v.plate.textContent = row.binding?.licensePlate || '等待车牌';
        v.volume.textContent = volumeText(m);
        v.time.textContent = when(m.measuredAt);
        v.note.textContent = row.latestRevision ? '已按CH4当日空车基线重新计算并保存'
            : row.publication ? '煤量已保存'
            : ['success', 'low_confidence'].includes(m.volumeStatus) && m.verdict === 'loaded' ? '测量已记录，煤量尚未保存。'
            : m.verdict === 'empty' ? '本次为空车。' : '本次结果已记录，可检查雷达或基线后处理。';
        if (v.verdict) v.verdict.className = 'verdict' + (m.verdict === 'empty' ? ' is-empty'
            : m.verdict === 'loaded' && v.volume.textContent !== '--' ? ' is-loaded' : ' is-wait');
        if (v.estimate) v.estimate.hidden = true;
        v.volume.className = 'v result-volume';
    }
    function makeCloud(canvas, notice) {
        try { return LoadRadarCloud.create(canvas); }
        catch (_) {
            notice.textContent = '无法显示点云，请检查浏览器';
            return { unavailable: true, frozen: false, clear() {}, setBounds() {}, setPoints() {}, setFrozen() {}, reset() {}, fit() {}, dispose() {} };
        }
    }
    function createCard(key) {
        const v = { key, currentId: null, currentRow: null, currentChecked: 0, currentBusy: false, currentGeneration: 0,
            logRows: [], logCursor: null, logBusy: false, logError: '', logChecked: 0, logGeneration: 0,
            logIdentity: '', logMismatch: 0, logUpdated: 0,
            runtimeRows: [], runtimeSupported: false, runtimeSignature: '' };
        v.card = element('section', undefined, 'card'); v.card.setAttribute('aria-label', '雷达通道');
        const heading = element('h2'), body = element('div', undefined, 'body'), kpis = element('div', undefined, 'kpis');
        v.name = element('span', '', 'name'); v.meta = element('span', '', 'meta'); v.badge = element('span', '', 'badge');
        heading.append(v.name, v.meta, v.badge);
        v.verdict = element('div', undefined, 'verdict is-wait');
        v.status = element('div', '等待雷达数据', 'big'); v.status.setAttribute('role', 'status');
        v.note = element('div', '', 'sub'); v.verdict.append(v.status);
        for (const [field, label] of [['plate', '车牌'], ['volume', '载货体积 m³'], ['time', '测量时间']]) {
            const kpi = element('div', undefined, 'kpi kpi-' + field + (field === 'time' ? ' time' : ''));
            v[field] = element('div', '--', 'v result-' + field); kpi.append(element('div', label, 'k'), v[field]); kpis.append(kpi);
            if (field === 'volume') {
                v.estimate = element('span', '（估值）', 'volume-estimate'); v.estimate.hidden = true; kpi.append(v.estimate);
                const status = element('div', undefined, 'kpi kpi-status');
                status.append(element('div', '测量状态', 'k'), v.verdict); kpis.append(status);
            }
        }
        v.canvas = element('canvas', undefined, 'cloud');
        v.canvasStatus = element('p', '等待点云', 'canvas-status');
        const legend = element('div', undefined, 'legend cloud-legend');
        legend.append(element('span', '低（地板）'), element('span', '高（货堆顶）'));
        v.freeze = element('button', '冻结画面', 'freeze'); v.freeze.type = 'button';
        v.freeze.addEventListener('click', () => { v.cloud.setFrozen(!v.cloud.frozen); renderCard(v); });
        legend.append(v.freeze);
        const controls = element('div', undefined, 'cloud-footer');
        controls.append(v.canvasStatus, element('span', '拖拽旋转 · 滚轮缩放 · 双击复位', 'cloud-help'));
        const preview = element('div', undefined, 'cloud-preview'); preview.append(v.canvas, legend, controls);
        v.health = element('div', '', 'chan-health'); v.health.hidden = true;
        body.append(kpis, preview, v.note, v.health); v.card.append(heading, body);
        dom.loadLanes.append(v.card); v.cloud = makeCloud(v.canvas, v.canvasStatus);
        v.tab = element('button', undefined, 'tab'); v.tab.type = 'button';
        v.dot = element('span', undefined, 'dot'); v.tabName = element('span', '');
        v.tab.append(v.dot, v.tabName); v.tab.addEventListener('click', () => selectLane(key));
        dom.laneTabs.append(v.tab);
        return v;
    }
    function createLogPanel() {
        const panel = element('section', undefined, 'lane-log');
        panel.setAttribute('aria-labelledby', 'lane-log-title');
        const header = element('div', undefined, 'lane-log-header');
        logView.title = element('h3', '全部车道车辆运行日志');
        logView.title.id = 'lane-log-title';
        logView.count = element('span', '0 条车次', 'meta');
        const actions = element('div', undefined, 'lane-log-actions');
        logView.refresh = element('button', '刷新', 'log-refresh'); logView.refresh.type = 'button';
        logView.refresh.setAttribute('aria-label', '刷新当前显示的车辆运行日志');
        logView.refresh.addEventListener('click', () => {
            for (const v of visibleLogViews()) void loadLaneLog(v, false, true);
        });
        logView.more = element('button', '加载更早记录', 'log-more'); logView.more.type = 'button'; logView.more.hidden = true;
        logView.more.setAttribute('aria-label', '加载当前显示车道更早的车辆运行日志');
        logView.more.addEventListener('click', () => {
            for (const v of visibleLogViews()) if (v.logCursor) void loadLaneLog(v, true, true);
        });
        actions.append(logView.refresh, logView.more); header.append(logView.title, logView.count, actions);
        logView.live = element('p', '当前状态：等待通道数据', 'lane-log-live'); logView.live.setAttribute('role', 'status');
        logView.feedback = element('p', '正在等待通道状态', 'lane-log-feedback'); logView.feedback.setAttribute('role', 'status');
        const runtime = element('section', undefined, 'runtime-log-section');
        runtime.setAttribute('aria-label', '近期车辆过程日志');
        const runtimeHeading = element('div', undefined, 'runtime-log-heading');
        logView.runtimeCount = element('span', '0 条过程', 'meta');
        runtimeHeading.append(element('h4', '近期过程 · 服务启动后'), logView.runtimeCount);
        logView.runtimeFeedback = element('p', '', 'runtime-log-feedback');
        logView.runtimeFeedback.setAttribute('role', 'status');
        const runtimeScroll = element('div', undefined, 'log-table-scroll runtime-log-scroll');
        const runtimeTable = element('table'), runtimeHead = element('thead'), runtimeHeader = element('tr');
        runtimeTable.setAttribute('aria-label', '车辆到场停稳与检测过程');
        for (const label of ['时间', '通道', '运行情况', '说明']) runtimeHeader.append(element('th', label));
        runtimeHead.append(runtimeHeader);
        logView.runtimeBody = element('tbody', undefined, 'runtime-log-rows');
        runtimeTable.append(runtimeHead, logView.runtimeBody); runtimeScroll.append(runtimeTable);
        runtime.append(runtimeHeading, logView.runtimeFeedback, runtimeScroll);
        logView.scroll = element('div', undefined, 'log-table-scroll');
        const table = element('table'); table.setAttribute('aria-label', '车辆测量运行记录');
        const head = element('thead'), tr = element('tr');
        for (const label of ['测量时间', '通道', '车牌', '运行情况', '体积 m³', '说明']) {
            const th = element('th', label); th.setAttribute('scope', 'col'); tr.append(th);
        }
        head.append(tr); logView.body = element('tbody', undefined, 'lane-log-rows'); table.append(head, logView.body);
        logView.scroll.append(table);
        panel.append(header, logView.live, runtime, element('h4', '已保存测量', 'saved-log-heading'), logView.feedback, logView.scroll);
        dom.laneLogs.append(panel); logView.panel = panel;
    }
    function clearCurrent(v) { v.currentGeneration++; v.currentRow = null; v.currentId = null; v.currentChecked = 0; }
    function online(lane) {
        return stateAvailable && lane?.online === true && lane.state?.online !== false
            && !(typeof lane.state?.frame_age_sec === 'number' && (!Number.isFinite(lane.state.frame_age_sec) || lane.state.frame_age_sec > 5));
    }
    function laneRunStatus(v, lane) {
        if (!stateAvailable) return '实时状态不可用';
        if (!lane?.nodeId) return lane?.error || '通道未配置';
        if (!online(lane)) return textNotice(lane.error) || '雷达离线或暂无新鲜帧';
        const state = lane.state || {};
        if (!state.current_result?.measurement_id && state.pending_compute?.pending_jobs > 0)
            return '已留样 ' + state.pending_compute.pending_jobs + ' 车次，识牌与计算后台进行';
        const preparation = { awaiting_plate: '正在识别本车车牌', plate_unavailable: '未识别到本车车牌',
            baseline_required: '缺少空车基线，尚未计算体积' }[state.preparation];
        if (preparation) return preparation;
        if (state.volume_preparation === 'parameters_required') return '体积参数未就绪，暂不能发布体积';
        if (state.current_result?.measurement_id) {
            const m = v.currentRow?.measurement;
            if (m?.measurementId === state.current_result.measurement_id) {
                const plate = v.currentRow.binding?.licensePlate || '未关联车牌';
                return plate + ' · ' + measurementStatus(m);
            }
            return '检测到本车，正在读取测量结果';
        }
        if (state.stability?.state === 0) return '等待车辆进入通道';
        if (state.stability?.state === 1) return '车辆移动中，等待停稳';
        if (state.stability?.state === 2) return '车辆已停稳，等待检测结果';
        return '等待车辆数据';
    }
    function setText(node, value) { if (node.textContent !== value) node.textContent = value; }
    function visibleLogViews() { return selectedKey ? [views.get(selectedKey)].filter(Boolean) : [...views.values()]; }
    function measuredAtMillis(value) {
        if (typeof value === 'number' && Number.isFinite(value)) return value > 1e12 ? value : value * 1000;
        const parsed = Date.parse(value);
        return Number.isFinite(parsed) ? parsed : 0;
    }
    function readRuntimeEvents(v, lane) {
        const events = lane.state?.runtime_events;
        if (!Array.isArray(events)) { v.runtimeSupported = false; return false; }
        v.runtimeSupported = true;
        const seen = new Set();
        const accepted = events.slice(-100).filter(event => event && event.channel === lane.channelId
            && typeof event.event_id === 'string' && /^[A-Za-z0-9_.:-]{1,100}$/.test(event.event_id)
            && typeof event.kind === 'string' && /^[a-z_]{1,40}$/.test(event.kind)
            && typeof event.ts === 'number' && Number.isFinite(event.ts) && event.ts > 0 && event.ts <= 253402300799
            && Number.isSafeInteger(event.session) && event.session >= 0
            && typeof event.message === 'string' && event.message.length > 0 && event.message.length <= 500
            && !seen.has(event.event_id) && seen.add(event.event_id))
            .map(event => ({ event_id: event.event_id, channel: event.channel, session: event.session,
                ts: event.ts, kind: event.kind, message: event.message,
                source: typeof event.source === 'string' && /^[A-Za-z0-9_-]{1,40}$/.test(event.source) ? event.source : '' }));
        const signature = JSON.stringify(accepted);
        if (signature === v.runtimeSignature) return false;
        v.runtimeRows = accepted; v.runtimeSignature = signature;
        return true;
    }
    function renderRuntimeLogs() {
        const visible = visibleLogViews();
        const rows = visible.flatMap(view => view.runtimeRows.map((event, index) => ({ event, view, index })))
            .sort((a, b) => b.event.ts - a.event.ts || a.view.key.localeCompare(b.view.key) || b.index - a.index);
        setText(logView.runtimeCount, rows.length + ' 条过程');
        const mapped = visible.filter(v => lanes.get(v.key)?.nodeId);
        let feedback = '';
        if (!stateAvailable) feedback = '实时连接中断；下方保留最近读取的过程日志。';
        else if (mapped.some(v => online(lanes.get(v.key)) && !v.runtimeSupported))
            feedback = '在线采集服务未提供过程日志，需更新采集服务；已读过程仅供历史参考。';
        else if (!rows.length && !mapped.some(v => online(lanes.get(v.key)))) feedback = '等待采集服务连接后读取过程日志。';
        else if (!rows.length) feedback = '尚无车辆到场或停稳事件。';
        else if (visible.some(v => !online(lanes.get(v.key)))) feedback = '通道当前离线；以下为已发生的过程，不代表当前车辆状态。';
        else feedback = '每通道保留最近 100 条过程；正式结果见下方已保存测量。';
        setText(logView.runtimeFeedback, feedback);
        const signature = JSON.stringify(rows.map(({ event, view }) => [view.key, lanes.get(view.key)?.name, event]));
        if (signature === logView.runtimeSignature) return;
        logView.runtimeSignature = signature;
        logView.runtimeBody.replaceChildren();
        if (!rows.length) {
            const row = element('tr'), cell = element('td', '暂无过程日志', 'runtime-log-empty');
            cell.setAttribute('colspan', '4'); row.append(cell); logView.runtimeBody.append(row); return;
        }
        const labels = { vehicle_arrived: '车辆到场', vehicle_detected: '车辆到场', vehicle_moving: '车辆移动中', vehicle_stable: '车辆已停稳',
            settling: '等待观测稳定', awaiting_plate: '等待车牌', plate_unavailable: '未获取车牌',
            baseline_required: '缺少空车基线', measurement_pending: '已留样待计算',
            preparation_wait: '等待处理', measurement_queued: '已留样待计算', observation_gap: '观测中断',
            observation_resumed: '观测恢复', preparation_ready: '准备就绪', measurement_calculated: '已计算待保存',
            measurement_completed: '检测完成', vehicle_departed: '车辆离场', error: '运行异常' };
        for (const { event, view } of rows) {
            const row = element('tr');
            const origin = { 'sim-radar': '演示', synthetic: '合成', replay: '回放' }[event.source];
            row.append(element('td', when(event.ts)), element('td', lanes.get(view.key)?.name || laneLabel(view.key)),
                element('td', (labels[event.kind] || '运行状态') + (origin ? '（' + origin + '）' : '')), element('td', event.message));
            logView.runtimeBody.append(row);
        }
    }
    function renderLogs(renderRows = false) {
        renderRuntimeLogs();
        const visible = visibleLogViews(), single = !!selectedKey;
        const rows = visible.flatMap(v => v.logRows.map(record => ({ record, view: v })));
        rows.sort((a, b) => measuredAtMillis(b.record.measurement.measuredAt) - measuredAtMillis(a.record.measurement.measuredAt)
            || FIXED_CHANNELS.findIndex(channel => channel.id === a.view.key) - FIXED_CHANNELS.findIndex(channel => channel.id === b.view.key));
        setText(logView.title, single
            ? (lanes.get(selectedKey)?.name || laneLabel(selectedKey)) + '车辆运行日志'
            : '全部车道车辆运行日志');
        setText(logView.live, single
            ? '当前状态：' + laneRunStatus(views.get(selectedKey), lanes.get(selectedKey))
            : '当前实时状态请查看上方四个车道卡片。');
        setText(logView.count, rows.length + ' 条车次');
        const busy = visible.some(v => v.logBusy), hasMore = visible.some(v => v.logCursor);
        const canRead = visible.some(v => lanes.get(v.key)?.nodeId);
        logView.refresh.disabled = !stateAvailable || !canRead || busy;
        logView.more.hidden = !hasMore;
        logView.more.disabled = busy || !hasMore;

        const failures = visible.filter(v => v.logError);
        const mismatchCount = visible.reduce((sum, v) => sum + v.logMismatch, 0);
        const lastUpdated = Math.max(0, ...visible.map(v => v.logUpdated));
        const feedbackClass = 'lane-log-feedback' + (failures.length ? ' is-error' : '');
        if (logView.feedback.className !== feedbackClass) logView.feedback.className = feedbackClass;
        let feedback = '';
        if (!stateAvailable) feedback = '实时状态不可用；不会发起历史查询。';
        else if (failures.length) feedback = failures.length === visible.length
            ? failures.map(v => (lanes.get(v.key)?.name || laneLabel(v.key)) + '：' + v.logError).join('；')
            : failures.length + ' 个通道日志读取异常：' + failures.map(v => laneLabel(v.key)).join('、');
        else if (mismatchCount) feedback = '发现 ' + mismatchCount + ' 条通道不匹配记录，已跳过。';
        else if (busy) feedback = rows.length ? '正在更新车辆记录…' : '正在读取车辆记录…';
        else if (lastUpdated) feedback = '最近更新 ' + when(new Date(lastUpdated));
        else if (!rows.length) feedback = '尚无已保存的车辆运行记录。';
        setText(logView.feedback, feedback);
        if (!renderRows) return;

        logView.body.replaceChildren();
        if (!rows.length) {
            const row = element('tr'), cell = element('td', busy ? '正在读取车辆记录…' : '暂无符合条件的车辆记录');
            cell.setAttribute('colspan', '6'); cell.className = 'log-empty'; row.append(cell); logView.body.append(row); return;
        }
        for (const { record, view } of rows) {
            const m = record.latestRevision?.calculation || record.measurement, row = element('tr');
            const plate = typeof record.binding?.licensePlate === 'string' && record.binding.licensePlate.trim()
                ? record.binding.licensePlate.trim() : '未关联车牌';
            const lane = lanes.get(view.key);
            row.append(element('td', when(m.measuredAt)), element('td', lane?.name || laneLabel(view.key)), element('td', plate));
            const state = element('td'), badge = element('span', measurementStatus(m), 'run-state');
            badge.className = 'run-state' + (m.verdict === 'unknown'
                ? ' is-warn' : m.verdict === 'empty' ? ' is-empty' : m.verdict === 'loaded' ? ' is-loaded' : '');
            state.append(badge); row.append(state);
            row.append(element('td', volumeText(m)));
            const reason = typeof m.reason === 'string' && m.reason.trim() ? m.reason.trim() : '—';
            const detail = element('td', reason); detail.setAttribute('title', reason); row.append(detail);
            logView.body.append(row);
        }
    }
    function laneIdentity(lane) {
        return lane?.nodeId && lane?.channelId ? JSON.stringify([lane.nodeId, lane.channelId]) : '';
    }
    async function loadLaneLog(v, more = false, force = false) {
        if (stopped || v.logBusy || more && !v.logCursor || !more && !force && Date.now() - v.logChecked < LOG_REFRESH_MS) return;
        const lane = lanes.get(v.key), identity = laneIdentity(lane);
        if (!stateAvailable || !identity) {
            if (!more) v.logChecked = Date.now();
            v.logError = !stateAvailable ? '实时状态不可用，暂不能确认节点映射。'
                : lane?.error || '该固定通道当前没有节点映射，暂不能读取历史记录。';
            renderLogs(true); return;
        }
        const generation = ++v.logGeneration, cursor = v.logCursor;
        v.logBusy = true; v.logError = '';
        if (!more) v.logChecked = Date.now();
        renderLogs(!v.logRows.length);
        const params = new URLSearchParams({ nodeId: lane.nodeId, channelId: lane.channelId, take: String(LOG_PAGE_SIZE) });
        if (more) params.set('beforeId', cursor);
        try {
            const rows = await read('/history?' + params.toString());
            if (stopped || generation !== v.logGeneration || views.get(v.key) !== v
                || laneIdentity(lanes.get(v.key)) !== identity) return;
            if (!Array.isArray(rows)) throw new Error('车辆记录格式无效');
            const accepted = rows.filter(row => row?.measurement && typeof row.measurement.measurementId === 'string'
                && row.measurement.measurementId.length > 0 && row.measurement.nodeId === lane.nodeId
                && row.measurement.channelId === lane.channelId);
            v.logMismatch = rows.length - accepted.length;
            const oldCursor = v.logCursor, seen = new Set(v.logRows.map(row => row.measurement.measurementId));
            if (more) {
                v.logRows = v.logRows.concat(accepted.filter(row => !seen.has(row.measurement.measurementId)));
                v.logCursor = accepted.length === LOG_PAGE_SIZE ? accepted[accepted.length - 1].measurement.measurementId : null;
            } else {
                const fresh = new Set(accepted.map(row => row.measurement.measurementId));
                const older = v.logRows.filter(row => !fresh.has(row.measurement.measurementId));
                v.logRows = accepted.concat(older);
                if (accepted.length === LOG_PAGE_SIZE) v.logCursor = v.logRows[v.logRows.length - 1]?.measurement.measurementId || null;
                else v.logCursor = older.length ? oldCursor : null;
            }
            v.logUpdated = Date.now(); v.logError = '';
        } catch (_) {
            if (!stopped && generation === v.logGeneration) v.logError = v.logRows.length
                ? '日志刷新失败，当前保留最近一次成功结果。' : '日志读取失败，请点击“刷新”重试。';
        } finally {
            if (!stopped && generation === v.logGeneration) {
                v.logBusy = false; renderLogs(true);
            }
        }
    }
    function refreshVisibleLogs(force = false) {
        if (stopped || !stateAvailable) return;
        const visible = selectedKey ? [views.get(selectedKey)].filter(Boolean) : [...views.values()];
        for (const v of visible) void loadLaneLog(v, false, force);
    }
    function renderCard(v) {
        const lane = lanes.get(v.key), state = lane?.state || {}, live = online(lane);
        v.name.textContent = lane?.name || lane?.channelId || '通道'; v.meta.textContent = lane?.channelId || '';
        v.card.setAttribute('aria-label', v.name.textContent);
        v.canvas.setAttribute('aria-label', v.name.textContent + '实时点云');
        v.tabName.textContent = v.name.textContent + ' ' + v.meta.textContent;
        v.badge.textContent = live ? '在线' : '离线'; v.badge.className = 'badge ' + (live ? 'ok' : 'bad');
        v.dot.className = 'dot ' + (live ? 'ok' : 'bad'); v.freeze.disabled = !live || v.cloud.unavailable === true;
        v.health.textContent = live ? textNotice(state.health)
            : textNotice(lane?.error) || (!stateAvailable ? '实时状态不可用' : '该固定通道暂无在线快照');
        v.health.hidden = !v.health.textContent;
        if (!live) {
            clearCurrent(v); v.cloud.clear(); resetResult(v, '雷达未连接');
            v.canvasStatus.textContent = lane?.error ? textNotice(lane.error) : '等待新点云';
        } else if (v.cloud.frozen) {
            resetResult(v, '画面已冻结'); v.note.textContent = '恢复画面后查看本车结果。';
            v.canvasStatus.textContent = '画面已冻结，非实时；检测继续';
        } else {
            v.cloud.setBounds(state.roi || null); v.cloud.setPoints(state.cloud || []);
            v.canvasStatus.textContent = v.cloud.unavailable ? '无法显示点云，请检查浏览器' : state.cloud?.length ? '实时点云' : '等待点云';
            const id = state.current_result?.measurement_id;
            if (!id) {
                clearCurrent(v); const status = state.stability?.state;
                const preparing = {awaiting_plate: '正在识别车牌', plate_unavailable: '未获取到本车车牌',
                    baseline_required: '需维护本车空车基线'}[state.preparation];
                resetResult(v, status === 0 ? '等待车辆' : preparing || (status === 1 ? '等待车辆停稳' : '正在测量'));
            } else {
                if (v.currentId !== id) { clearCurrent(v); v.currentId = id; }
                if (v.currentRow) showRecord(v, v.currentRow); else resetResult(v, '正在保存测量结果');
                refreshCurrent(v, id);
            }
            if (state.preparation === 'baseline_required') v.note.textContent = '请联系维护人员准备本车空车基线，暂不能保存载货体积。';
            else if (state.preparation === 'plate_unavailable') v.note.textContent = '本次车牌未确认，未计算或保存载货体积。';
            else if (state.volume_preparation === 'parameters_required') v.note.textContent = '本车体积参数未就绪，请联系维护人员；当前估值不入库。';
            if (state.pending_compute?.pending_jobs > 0)
                v.note.textContent = '已有 ' + state.pending_compute.pending_jobs + ' 车次留样，后台补全车牌与计算；采集继续。';
            else if (state.pending_compute?.ingress_jobs > 0)
                v.note.textContent = '正在保存原始观测，落盘后确认留样。';
        }
        v.freeze.textContent = v.cloud.frozen ? '恢复画面' : '冻结画面';
        v.freeze.setAttribute('aria-pressed', String(v.cloud.frozen));
    }
    async function refreshCurrent(v, id) {
        if (v.currentBusy || Date.now() - v.currentChecked < 3000) return;
        v.currentBusy = true; v.currentChecked = Date.now();
        const generation = v.currentGeneration;
        const stillCurrent = () => !stopped && generation === v.currentGeneration && views.get(v.key) === v && online(lanes.get(v.key))
            && v.currentId === id && lanes.get(v.key).state.current_result?.measurement_id === id;
        try {
            const row = await read('/measurements/' + encodeURIComponent(id));
            if (!stillCurrent()) return;
            const lane = lanes.get(v.key);
            if (!row?.measurement || row.measurement.measurementId !== id || row.measurement.nodeId !== lane.nodeId
                || row.measurement.channelId !== lane.channelId) throw new Error('测量通道或编号不匹配');
            v.currentRow = row; renderCard(v);
        } catch (_) {
            if (stillCurrent() && !v.cloud.frozen) {
                v.currentRow = null; resetResult(v, '等待测量结果'); v.note.textContent = '结果尚未确认，系统会继续查询。';
            }
        } finally { v.currentBusy = false; }
    }
    function selectLane(key, refreshLogs = true) {
        const previousKey = selectedKey;
        selectedKey = FIXED_CHANNELS.some(channel => channel.id === key) ? key : '';
        const single = !!selectedKey;
        dom.liveLayout.className = 'live-layout' + (single ? ' view-single' : '');
        dom.loadLanes.className = 'grid' + (single ? ' view-single' : '');
        allTab.className = single ? 'tab' : 'tab on'; allTab.setAttribute('aria-pressed', String(!single));
        for (const { id } of FIXED_CHANNELS) {
            const v = views.get(id);
            v.card.hidden = single && selectedKey !== id;
            v.tab.className = selectedKey === id ? 'tab on' : 'tab';
            v.tab.setAttribute('aria-pressed', String(selectedKey === id));
            if (!v.card.hidden) v.cloud.fit();
        }
        if (refreshLogs || previousKey !== selectedKey) renderLogs(true);
        else renderLogs(false);
        if (refreshLogs) refreshVisibleLogs(true);
    }
    function syncHistoryChannels() {
        const configured = FIXED_CHANNELS.map(({ id }) => lanes.get(id)).filter(lane => lane?.nodeId);
        const signature = JSON.stringify(configured.map(lane => [lane.nodeId, lane.channelId, lane.name]));
        if (signature === laneOptionsSignature) return;
        const previous = dom.filterChannel.value, options = [element('option', '全部通道')]; options[0].value = '';
        for (const lane of configured) {
            const option = element('option', lane.name || lane.channelId);
            option.value = JSON.stringify([lane.nodeId, lane.channelId]); options.push(option);
        }
        dom.filterChannel.replaceChildren(...options);
        dom.filterChannel.value = options.some(option => option.value === previous) ? previous : '';
        laneOptionsSignature = signature;
    }
    function renderState(data) {
        if (!data || !Array.isArray(data.lanes) || data.lanes.some(lane => !lane || typeof lane.nodeId !== 'string'
            || typeof lane.channelId !== 'string' || !lane.channelId)
            || new Set(data.lanes.map(lane => JSON.stringify([lane.nodeId, lane.channelId]))).size !== data.lanes.length)
            throw new Error('通道状态无效');
        stateAvailable = data.enabled === true;
        const warnings = [], next = new Map();
        for (const { id, label } of FIXED_CHANNELS) {
            const matches = data.lanes.filter(lane => lane.channelId === id);
            const lane = matches.length === 1 ? matches[0] : {
                nodeId: '', channelId: id, name: label, online: false, state: null,
                error: matches.length > 1 ? '多个节点使用相同通道编号，无法唯一匹配。'
                    : !data.enabled ? '载货雷达服务未启用。' : '后台未返回此通道的状态。'
            };
            if (matches.length > 1) warnings.push(label + '存在重复通道编号');
            else if (stateAvailable && matches.length === 0) warnings.push(label + '暂无状态');
            next.set(id, lane);
        }
        const unexpected = data.lanes.filter(lane => !FIXED_CHANNELS.some(channel => channel.id === lane.channelId));
        if (unexpected.length) warnings.push('后端含 CH1–CH4 以外通道，固定页面未显示');
        lanes.clear();
        let logsChanged = false;
        for (const { id } of FIXED_CHANNELS) {
            const lane = next.get(id), v = views.get(id), identity = laneIdentity(lane);
            const identityChanged = v.logIdentity !== identity;
            if (identityChanged) {
                logsChanged = true;
                v.logGeneration++; v.logIdentity = identity; v.logRows = []; v.logCursor = null;
                v.logBusy = false; v.logError = ''; v.logChecked = 0; v.logUpdated = 0; v.logMismatch = 0;
                v.runtimeRows = []; v.runtimeSignature = ''; v.runtimeSupported = false;
                clearCurrent(v);
            }
            lanes.set(id, lane);
            if (readRuntimeEvents(v, lane)) logsChanged = true;
            dom.loadLanes.append(v.card);
            renderCard(v);
        }
        syncHistoryChannels();
        dom.channelCount.textContent = String(FIXED_CHANNELS.length);
        dom.onlineCount.textContent = String(FIXED_CHANNELS.filter(channel => online(lanes.get(channel.id))).length);
        dom.loadConnection.textContent = !data.enabled ? '载货雷达未启用'
            : Number(dom.onlineCount.textContent) > 0 ? '实时连接' : '等待雷达数据';
        dom.connectionDot.className = 'dot' + (Number(dom.onlineCount.textContent) > 0 ? ' ok' : '');
        if (!data.enabled) warnings.unshift('载货雷达服务未启用；四个固定车道仍保留。');
        dom.laneNotice.hidden = warnings.length === 0;
        dom.laneNotice.textContent = warnings.join('；');
        selectLane(selectedKey, false);
        if (logsChanged) renderLogs(true);
        refreshVisibleLogs(false);
    }
    async function pollState() {
        if (stopped || stateBusy) return;
        stateBusy = true;
        try { const data = await read('/state'); if (!stopped) renderState(data); }
        catch (_) {
            if (!stopped) {
                stateAvailable = false; dom.loadConnection.textContent = '连接中断，正在重连';
                dom.connectionDot.className = 'dot bad'; dom.onlineCount.textContent = '0';
                for (const v of views.values()) renderCard(v);
                renderLogs(false);
            }
        } finally { stateBusy = false; }
    }
    function renderHistory(emptyMessage = '没有符合条件的测量') {
        dom.loadHistoryRows.replaceChildren();
        for (const row of records) {
            const m = row.latestRevision?.calculation || row.measurement, tr = element('tr'), lane = lanes.get(laneKey(m));
            const volume = volumeText(m, true);
            for (const value of [when(m.measuredAt), lane?.name || m.channelId, row.binding?.licensePlate || '未取得', measurementStatus(m), volume]) tr.append(element('td', value));
            const td = element('td'), button = element('button', '查看'); button.type = 'button';
            button.addEventListener('click', () => selectMeasurement(m.measurementId));
            td.append(button); tr.append(td); dom.loadHistoryRows.append(tr);
        }
        if (!records.length) {
            const tr = element('tr'), td = element('td', emptyMessage); td.colSpan = 6; tr.append(td); dom.loadHistoryRows.append(tr);
        }
    }
    function queryString() {
        const params = new URLSearchParams(); params.set('take', String(PAGE_SIZE));
        const plate = dom.filterPlate.value.trim(); if (plate) params.set('licensePlate', plate);
        if (dom.filterChannel.value) {
            const pair = JSON.parse(dom.filterChannel.value); params.set('nodeId', pair[0]); params.set('channelId', pair[1]);
        }
        const from = dom.filterFrom.value, to = dom.filterTo.value;
        if (from && !Number.isFinite(new Date(from).getTime()) || to && !Number.isFinite(new Date(to).getTime())) throw new Error('请输入有效时间');
        if (from && to && new Date(from) > new Date(to)) throw new Error('开始时间不能晚于结束时间');
        if (from) params.set('from', new Date(from).toISOString());
        if (to) params.set('to', new Date(to).toISOString());
        return params.toString();
    }
    async function loadHistory(more = false, preparedQuery = null) {
        if (stopped || more && (historyBusy || !cursor)) return;
        let query;
        try { query = preparedQuery !== null ? preparedQuery : more ? historyQuery : queryString(); } catch (error) { dom.historySummary.textContent = error.message; return; }
        if (historyBusy) {
            pendingHistoryQuery = query; historyGeneration += 1;
            records = []; cursor = null; historyQuery = query; renderHistory('正在查询…');
            dom.loadHistoryMore.disabled = true; dom.historySummary.textContent = '正在查询';
            return;
        }
        const generation = ++historyGeneration; historyBusy = true; dom.loadHistoryMore.disabled = true;
        if (!more) { records = []; cursor = null; historyQuery = query; renderHistory('正在查询…'); }
        dom.historySummary.textContent = '正在查询';
        try {
            const rows = await read('/history?' + query + (more ? '&beforeId=' + encodeURIComponent(cursor) : ''));
            if (stopped || generation !== historyGeneration) return;
            if (!Array.isArray(rows) || rows.some(row => !row?.measurement?.measurementId)) throw new Error('历史数据不完整');
            const seen = new Set(records.map(row => row.measurement.measurementId));
            records = records.concat(rows.filter(row => !seen.has(row.measurement.measurementId)));
            cursor = rows.length === PAGE_SIZE ? rows[rows.length - 1].measurement.measurementId : null;
            renderHistory(); dom.historySummary.textContent = '共显示 ' + records.length + ' 条';
        } catch (_) {
            if (!stopped && generation === historyGeneration) {
                dom.historySummary.textContent = '查询失败，请重试';
                renderHistory('未取得查询结果；请点击查询重试');
            }
        }
        finally {
            historyBusy = false;
            if (!stopped && pendingHistoryQuery !== null) {
                const next = pendingHistoryQuery; pendingHistoryQuery = null; loadHistory(false, next);
            } else if (generation === historyGeneration) dom.loadHistoryMore.disabled = !cursor;
        }
    }

    function clearHistoryDetail() {
        detailGeneration += 1; dom.historyDetail.hidden = true;
        if (historyCloud) historyCloud.clear();
    }
    async function selectMeasurement(id) {
        clearHistoryDetail(); const generation = detailGeneration;
        dom.historySummary.textContent = '正在读取点云';
        try {
            const row = await read('/measurements/' + encodeURIComponent(id));
            if (stopped || generation !== detailGeneration || !dom.historyDialog.open) return;
            if (!row?.measurement || row.measurement.measurementId !== id) throw new Error('测量响应不匹配');
            dom.historyDetail.hidden = false;
            if (!historyCloud) historyCloud = makeCloud(dom.historyCanvas, dom.historyResultNote);
            historyCloud.clear(); historyCloud.setBounds(null); historyCloud.setPoints(row.measurement.points || []);
            showRecord({status: dom.historyResultStatus, plate: dom.historyResultPlate, volume: dom.historyResultVolume,
                time: dom.historyResultTime, note: dom.historyResultNote}, row);
            dom.historySummary.textContent = '历史点云 · 非实时';
        } catch (_) { if (!stopped && generation === detailGeneration) dom.historySummary.textContent = '点云读取失败，请重试'; }
    }
    function init() {
        for (const id of ['loadConnection', 'connectionDot', 'channelCount', 'onlineCount', 'laneTabs', 'loadLanes', 'laneNotice',
            'liveLayout', 'runLogs', 'laneLogs',
            'openHistory', 'historyDialog', 'closeHistory', 'loadHistoryForm', 'filterPlate', 'filterChannel',
            'filterFrom', 'filterTo', 'historySummary', 'loadHistoryRows', 'loadHistoryMore', 'historyDetail',
            'historyResultStatus', 'historyResultPlate', 'historyResultVolume', 'historyResultTime', 'historyResultNote', 'historyCanvas']) dom[id] = document.getElementById(id);
        dom.laneTabs.append(element('span', '通道', 'lbl'));
        allTab = element('button', '全部', 'tab on'); allTab.type = 'button'; allTab.setAttribute('aria-pressed', 'true');
        allTab.addEventListener('click', () => selectLane('')); dom.laneTabs.append(allTab);
        dom.channelCount.textContent = String(FIXED_CHANNELS.length); dom.onlineCount.textContent = '0';
        createLogPanel();
        for (const { id, label } of FIXED_CHANNELS) {
            const view = createCard(id);
            views.set(id, view);
            lanes.set(id, { nodeId: '', channelId: id, name: label, online: false, state: null, error: '等待后台通道状态。' });
            renderCard(view);
        }
        selectLane('', false);
        renderLogs(true);
        dom.openHistory.addEventListener('click', () => { clearHistoryDetail(); dom.historyDialog.showModal(); loadHistory(); });
        dom.closeHistory.addEventListener('click', () => { clearHistoryDetail(); dom.historyDialog.close(); });
        dom.historyDialog.addEventListener('cancel', clearHistoryDetail);
        dom.historyDialog.addEventListener('close', clearHistoryDetail);
        dom.loadHistoryForm.addEventListener('submit', e => { e.preventDefault(); clearHistoryDetail(); loadHistory(); });
        dom.loadHistoryMore.addEventListener('click', () => loadHistory(true));
        // 标签页不可见时停止轮询：每次 /state 都会让采集端在帧锁内拼一份快照，
        // 没人看的后台标签页不该持续占用它；回到前台立即补一次再恢复周期。
        function startPolling() { if (!stopped && !timer) { pollState(); timer = setInterval(pollState, 1500); } }
        function stopPolling() { if (timer) { clearInterval(timer); timer = undefined; } }
        startPolling();
        document.addEventListener('visibilitychange', () => (document.hidden ? stopPolling() : startPolling()));
        window.addEventListener('pagehide', () => {
            stopped = true; detailGeneration += 1; historyGeneration += 1; clearInterval(timer);
            for (const v of views.values()) { v.logGeneration += 1; v.cloud.dispose(); }
            if (historyCloud) historyCloud.dispose();
        });
    }
    document.addEventListener('DOMContentLoaded', init);
})();
