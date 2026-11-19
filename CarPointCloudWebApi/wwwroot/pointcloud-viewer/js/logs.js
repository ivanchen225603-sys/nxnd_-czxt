/**
 * 系统日志查看器
 * 支持关键字搜索、时间范围过滤、级别/来源筛选、分页展示
 */

// ============================
// 配置区域
// ============================
const CONFIG = {
    // 后端日志接口地址（由 CarPointCloudWebApi 的 LogController 提供）
    // 返回格式：{ code: 200, message: "success", data: { items, total, page, pageSize } }
    // 生产环境必须以数据库审计日志为准，不使用模拟数据兜底
    apiUrl: '/api/logs',

    // 保留配置项兼容旧代码，但生产默认关闭模拟数据
    useMockData: false,

    // 模拟数据条数
    mockCount: 128,

    // 默认分页大小
    defaultPageSize: 20,

    // 后端分页：默认每页拉取条数（覆盖前端分页，简单场景下一次性拉取全部）
    apiPageSize: 100
};

// ============================
// 状态管理
// ============================
const state = {
    allLogs: [],           // 全部日志
    filteredLogs: [],      // 筛选后日志
    currentPage: 1,        // 当前页码
    pageSize: CONFIG.defaultPageSize, // 每页条数
    filters: {
        keyword: '',
        startTime: '',
        endTime: '',
        level: '',
        source: '',
        interactionId: '',
        workflowId: '',
        licensePlate: '',
        integrationType: ''
    },
    isLoading: false,
    loadError: false
};

// ============================
// 常量定义
// ============================
const LEVELS = {
    debug: { label: 'Debug', class: 'debug' },
    info: { label: 'Info', class: 'info' },
    warn: { label: 'Warn', class: 'warn' },
    error: { label: 'Error', class: 'error' }
};

const LOG_SOURCES = [
    'PointCloudViewer',
    'PCDParser',
    'OrbitControls',
    'FileUploader',
    'RenderLoop',
    'WebApiClient',
    'LidarDriver',
    'CalibrationService'
];

const LOG_MESSAGES = {
    info: [
        '点云数据加载完成，共 {n} 个点',
        '视角已重置到默认位置',
        '截图已保存至本地',
        'PCD 文件解析成功: {name}',
        '渲染循环启动，目标帧率 60 FPS',
        '与雷达设备建立连接',
        '校准参数已更新',
        '开始接收雷达数据帧'
    ],
    debug: [
        '顶点缓冲区大小: {n} bytes',
        '相机位置: ({x}, {y}, {z})',
        '着色器编译耗时: {n} ms',
        '鼠标事件坐标: ({x}, {y})',
        '请求参数: {json}'
    ],
    warn: [
        'PCD 文件包含未知字段，已跳过',
        '点云密度较低，可能影响识别效果',
        '网络延迟较高，当前延迟 {n} ms',
        '检测到异常帧，已自动丢弃',
        '内存占用接近阈值: {n} MB'
    ],
    error: [
        '加载 PCD 文件失败: {msg}',
        '解析点云数据出错，格式不兼容',
        'WebGL 上下文丢失，请刷新页面',
        '与雷达设备连接断开',
        '上传文件超出大小限制: {name}'
    ]
};

// ============================
// 工具函数
// ============================

/**
 * 格式化日期时间为本地字符串
 */
function formatDateTime(date) {
    const pad = (n) => String(n).padStart(2, '0');
    const y = date.getFullYear();
    const m = pad(date.getMonth() + 1);
    const d = pad(date.getDate());
    const h = pad(date.getHours());
    const min = pad(date.getMinutes());
    const s = pad(date.getSeconds());
    const ms = pad(String(date.getMilliseconds()).padStart(3, '0'));
    return `${y}-${m}-${d} ${h}:${min}:${s}.${ms}`;
}

/**
 * 格式化日期为 datetime-local 输入框所需格式
 */
function formatDateTimeLocal(date) {
    const pad = (n) => String(n).padStart(2, '0');
    const y = date.getFullYear();
    const m = pad(date.getMonth() + 1);
    const d = pad(date.getDate());
    const h = pad(date.getHours());
    const min = pad(date.getMinutes());
    return `${y}-${m}-${d}T${h}:${min}`;
}

/**
 * 转义 HTML 特殊字符
 */
function escapeHtml(text) {
    const div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
}

/**
 * 高亮关键字
 */
function highlightKeyword(text, keyword) {
    if (!keyword) return escapeHtml(text);
    const safeKeyword = escapeHtml(keyword).replace(/[.*+?^${}()|[\]\\]/g, '\\$&');
    const regex = new RegExp(`(${safeKeyword})`, 'gi');
    return escapeHtml(text).replace(regex, '<span class="highlight">$1</span>');
}

/**
 * 生成随机整数
 */
function randomInt(min, max) {
    return Math.floor(Math.random() * (max - min + 1)) + min;
}

/**
 * 生成模拟日志数据
 */
function generateMockLogs(count) {
    const logs = [];
    const now = new Date();
    const levels = Object.keys(LEVELS);

    for (let i = 0; i < count; i++) {
        const level = levels[randomInt(0, levels.length - 1)];
        const source = LOG_SOURCES[randomInt(0, LOG_SOURCES.length - 1)];
        const templates = LOG_MESSAGES[level];
        let message = templates[randomInt(0, templates.length - 1)];

        message = message
            .replace('{n}', randomInt(100, 99999))
            .replace('{x}', randomInt(-100, 100))
            .replace('{y}', randomInt(-100, 100))
            .replace('{z}', randomInt(-100, 100))
            .replace('{name}', `sample_${randomInt(1, 99)}.pcd`)
            .replace('{msg}', '文件头信息缺失')
            .replace('{json}', JSON.stringify({ id: i, enabled: true }));

        const time = new Date(now.getTime() - randomInt(0, 7 * 24 * 60 * 60 * 1000));

        logs.push({
            id: `log-${i + 1}`,
            time: formatDateTime(time),
            timestamp: time.getTime(),
            level,
            source,
            message
        });
    }

    // 按时间倒序排列
    return logs.sort((a, b) => b.timestamp - a.timestamp);
}

/**
 * 从接口加载日志数据
 * 适配后端统一响应 { code, message, data: { items, total, page, pageSize } }
 */
async function fetchLogsFromApi() {
    try {
        // 先由数据库按交互 ID、流程 ID等条件精确过滤，再在前端分页展示。
        const query = new URLSearchParams({ page: '1', pageSize: String(CONFIG.apiPageSize) });
        const filters = state.filters;
        const queryFields = {
            keyword: filters.keyword,
            startTime: filters.startTime,
            endTime: filters.endTime,
            level: filters.level,
            source: filters.source,
            interactionId: filters.interactionId,
            workflowId: filters.workflowId,
            licensePlate: filters.licensePlate,
            integrationType: filters.integrationType
        };
        Object.entries(queryFields).forEach(([key, value]) => {
            if (value) query.set(key, value);
        });
        const result = await API.get(`${CONFIG.apiUrl}?${query.toString()}`);
        // 兼容后端标准响应：data 为 { items, total }；同时兼容旧的 { success, data: [] } 形态
        const payload = result?.data ?? result;
        const items = Array.isArray(payload) ? payload : payload?.items;
        if (!Array.isArray(items)) {
            throw new Error('接口返回格式不正确');
        }
        return items.map(item => ({
            id: item.id ?? `log-${Date.now()}-${Math.random()}`,
            time: item.time || formatDateTime(new Date(item.timestamp || Date.now())),
            startTime: item.startTime || item.time || '',
            timestamp: item.timestamp ? new Date(item.timestamp).getTime() : new Date(item.time).getTime(),
            level: item.level || 'info',
            source: item.source || 'Unknown',
            message: item.message || '',
            recordKind: item.recordKind || 'Access',
            lifecycleStage: item.lifecycleStage || '',
            interactionId: item.interactionId || '',
            integrationType: item.integrationType || '',
            direction: item.direction || '',
            callerSystem: item.callerSystem || '',
            targetSystem: item.targetSystem || '',
            workflowId: item.workflowId || '',
            eventId: item.eventId || '',
            licensePlate: item.licensePlate || '',
            cameraIdentifier: item.cameraIdentifier || '',
            attemptNo: item.attemptNo,
            requestHeadersJson: item.requestHeadersJson || '',
            requestBody: item.requestBody || '',
            responseHeadersJson: item.responseHeadersJson || '',
            responseBody: item.responseBody || '',
            requestPath: item.requestPath || '',
            scheme: item.scheme || '',
            host: item.host || '',
            traceIdentifier: item.traceIdentifier || '',
            clientIp: item.clientIp || '',
            requestContentType: item.requestContentType || '',
            responseContentType: item.responseContentType || '',
            timestampHeader: item.timestampHeader || '',
            signatureHeader: item.signatureHeader || '',
            idempotencyKeyHeader: item.idempotencyKeyHeader || '',
            requestContentLength: item.requestContentLength,
            responseContentLength: item.responseContentLength,
            httpStatus: item.httpStatus || 0,
            businessCode: item.businessCode || 0,
            elapsedMs: item.elapsedMs || 0,
            errorMessage: item.errorMessage || ''
        }));
    } catch (error) {
        console.error('加载日志接口失败:', error);
        throw error;
    }
}

// ============================
// 数据加载
// ============================

/**
 * 加载日志数据
 */
async function loadLogs() {
    if (state.isLoading) return false;
    state.isLoading = true;
    setLogQueryBusy(true);
    document.getElementById('logsTableBody').innerHTML = '';
    document.getElementById('logsEmpty').style.display = 'flex';
    document.querySelector('.logs-empty-text').textContent = '正在查询日志…';
    document.getElementById('statTotal').textContent = '—';
    document.getElementById('statFiltered').textContent = '—';
    document.getElementById('paginationInfo').textContent = '正在查询日志…';
    document.getElementById('paginationControls').innerHTML = '';

    try {
        readFiltersFromUI();
        if (CONFIG.apiUrl) {
            try {
                state.allLogs = await fetchLogsFromApi();
            } catch (error) {
                if (CONFIG.useMockData) {
                    showToast('接口请求失败，已切换到模拟数据', 'warn');
                    state.allLogs = generateMockLogs(CONFIG.mockCount);
                } else {
                    state.allLogs = [];
                    throw error;
                }
            }
        } else {
            state.allLogs = [];
        }

        state.loadError = false;
        populateSourceFilter();
        applyFilters();
        return true;
    } catch (error) {
        console.error('加载日志失败:', error);
        showToast(`加载日志失败: ${error.message}`, 'error');
        state.allLogs = [];
        state.loadError = true;
        applyFilters();
        return false;
    } finally {
        state.isLoading = false;
        setLogQueryBusy(false);
    }
}

function setLogQueryBusy(busy) {
    document.getElementById('logsResults').ariaBusy = String(busy);
    ['searchBtn', 'resetBtn', 'refreshBtn', 'pageSizeSelect'].forEach(id => {
        document.getElementById(id).disabled = busy;
    });
}

// ============================
// 筛选逻辑
// ============================

/**
 * 应用筛选条件
 */
function applyFilters() {
    const { keyword, startTime, endTime, level, source, interactionId, workflowId, licensePlate, integrationType } = state.filters;

    const startTimestamp = startTime ? new Date(startTime).getTime() : null;
    const endTimestamp = endTime ? new Date(endTime).getTime() : null;
    const keywordLower = keyword.toLowerCase().trim();

    state.filteredLogs = state.allLogs.filter(log => {
        // 时间范围过滤
        if (startTimestamp && log.timestamp < startTimestamp) return false;
        if (endTimestamp && log.timestamp > endTimestamp) return false;

        // 级别过滤
        if (level && log.level !== level) return false;

        // 来源过滤
        if (source && log.source !== source) return false;

        if (interactionId && log.interactionId !== interactionId.trim()) return false;
        if (workflowId && log.workflowId !== workflowId.trim()) return false;
        if (licensePlate && log.licensePlate !== licensePlate.trim()) return false;
        if (integrationType && log.integrationType !== integrationType) return false;

        // 关键字过滤（时间、级别、来源、内容）
        if (keywordLower) {
            const combined = `${log.time} ${log.level} ${log.source} ${log.message} ${log.interactionId} ${log.workflowId} ${log.licensePlate} ${log.integrationType}`.toLowerCase();
            if (!combined.includes(keywordLower)) return false;
        }

        return true;
    });

    state.currentPage = 1;
    updateStats();
    renderTable();
    renderPagination();
}

/**
 * 读取当前筛选条件
 */
function readFiltersFromUI() {
    state.filters.keyword = document.getElementById('keywordInput').value;
    state.filters.startTime = document.getElementById('startTimeInput').value;
    state.filters.endTime = document.getElementById('endTimeInput').value;
    state.filters.level = document.getElementById('levelSelect').value;
    state.filters.source = document.getElementById('sourceSelect').value;
    state.filters.interactionId = document.getElementById('interactionIdInput').value.trim();
    state.filters.workflowId = document.getElementById('workflowIdInput').value.trim();
    state.filters.licensePlate = document.getElementById('licensePlateInput').value.trim();
    state.filters.integrationType = document.getElementById('integrationTypeSelect').value;
}

/**
 * 填充来源下拉框
 */
function populateSourceFilter() {
    const sourceSelect = document.getElementById('sourceSelect');
    const sources = [...new Set(state.allLogs.map(log => log.source))].sort();

    // 保留第一个 "全部来源" 选项
    sourceSelect.innerHTML = '<option value="">全部来源</option>';
    sources.forEach(source => {
        const option = document.createElement('option');
        option.value = source;
        option.textContent = source;
        sourceSelect.appendChild(option);
    });
}

// ============================
// 渲染逻辑
// ============================

/**
 * 更新统计信息
 */
function updateStats() {
    document.getElementById('statTotal').textContent = state.loadError ? '—' : state.allLogs.length.toLocaleString();
    document.getElementById('statFiltered').textContent = state.loadError ? '—' : state.filteredLogs.length.toLocaleString();
}

/**
 * 渲染日志表格
 */
function renderTable() {
    const tbody = document.getElementById('logsTableBody');
    const emptyEl = document.getElementById('logsEmpty');
    document.querySelector('.logs-empty-text').textContent = state.loadError
        ? '查询失败，未取得日志；请点击查询重试' : '暂无符合条件的日志';

    if (state.filteredLogs.length === 0) {
        tbody.innerHTML = '';
        emptyEl.style.display = 'flex';
        return;
    }

    emptyEl.style.display = 'none';

    const start = (state.currentPage - 1) * state.pageSize;
    const end = start + state.pageSize;
    const pageLogs = state.filteredLogs.slice(start, end);
    const keyword = state.filters.keyword.trim();

    tbody.innerHTML = pageLogs.map(log => {
        const levelMeta = LEVELS[log.level] || LEVELS.info;
        const integrationLabel = {
            LicensePlate: '获取车牌号',
            VehicleDimensions: '获取车辆尺寸',
            CoalVolume: '获取煤炭体积',
            VehicleArrival: '来车通知'
        }[log.integrationType] || log.source;
        const directionLabel = log.direction === 'Outbound' ? '出站' : log.direction === 'Inbound' ? '入站' : '';
        const date = new Date(log.time);
        const displayTime = Number.isNaN(date.getTime()) ? log.time
            : date.toLocaleString('zh-CN', { hour12: false });
        const interactionButton = log.interactionId
            ? `<button type="button" class="interaction-action-btn" data-interaction-id="${escapeHtml(log.interactionId)}" aria-label="查看交互 ${escapeHtml(log.interactionId)} 的完整链路">查看</button>`
            : '<span class="log-direction">无交互 ID</span>';
        return `
            <tr>
                <td class="log-time" title="${escapeHtml(log.time)}">${escapeHtml(displayTime)}</td>
                <td>
                    <span class="log-level ${levelMeta.class}">${levelMeta.label}</span>
                </td>
                <td class="log-source">${escapeHtml(integrationLabel)}${directionLabel ? `<span class="log-direction"> / ${escapeHtml(directionLabel)}</span>` : ''}</td>
                <td class="log-interaction-id" title="${escapeHtml(log.interactionId)}">${escapeHtml(log.interactionId || '-')}</td>
                <td>${log.httpStatus ? `<span class="log-level ${log.httpStatus >= 500 ? 'error' : log.httpStatus >= 400 ? 'warn' : 'http-success'}">${escapeHtml(String(log.httpStatus))}</span>` : '-'}</td>
                <td class="log-message">${highlightKeyword(log.message, keyword)}</td>
                <td>${interactionButton}</td>
            </tr>
        `;
    }).join('');
}

/**
 * 渲染分页控件
 */
function renderPagination() {
    if (state.loadError) {
        document.getElementById('paginationInfo').textContent = '查询未完成，记录数量未知';
        document.getElementById('paginationControls').innerHTML = '';
        return;
    }
    const total = state.filteredLogs.length;
    const totalPages = Math.max(1, Math.ceil(total / state.pageSize));

    // 修正当前页码
    if (state.currentPage > totalPages) {
        state.currentPage = totalPages;
    }

    const start = total === 0 ? 0 : (state.currentPage - 1) * state.pageSize + 1;
    const end = Math.min(state.currentPage * state.pageSize, total);
    document.getElementById('paginationInfo').textContent =
        total === 0 ? '显示 0 条，共 0 条' : `显示 ${start}-${end} 条，共 ${total.toLocaleString()} 条`;

    const controls = document.getElementById('paginationControls');
    controls.innerHTML = buildPaginationButtons(state.currentPage, totalPages);
}

/**
 * 构成分页按钮 HTML
 */
function buildPaginationButtons(current, total) {
    if (total <= 1) return '';

    const buttons = [];

    // 上一页
    buttons.push(`
        <button class="page-btn" data-page="${current - 1}" ${current === 1 ? 'disabled' : ''}>
            上一页
        </button>
    `);

    // 页码按钮
    const maxVisible = 7;
    let startPage = 1;
    let endPage = total;

    if (total > maxVisible) {
        const half = Math.floor(maxVisible / 2);
        if (current <= half + 1) {
            endPage = maxVisible - 2;
        } else if (current >= total - half) {
            startPage = total - maxVisible + 3;
        } else {
            startPage = current - half + 1;
            endPage = current + half - 1;
        }
    }

    if (startPage > 1) {
        buttons.push(`<button class="page-btn ${current === 1 ? 'active' : ''}" data-page="1">1</button>`);
        if (startPage > 2) {
            buttons.push(`<span class="page-ellipsis">...</span>`);
        }
    }

    for (let i = startPage; i <= endPage; i++) {
        buttons.push(`
            <button class="page-btn ${i === current ? 'active' : ''}" data-page="${i}">${i}</button>
        `);
    }

    if (endPage < total) {
        if (endPage < total - 1) {
            buttons.push(`<span class="page-ellipsis">...</span>`);
        }
        buttons.push(`
            <button class="page-btn ${current === total ? 'active' : ''}" data-page="${total}">${total}</button>
        `);
    }

    // 下一页
    buttons.push(`
        <button class="page-btn" data-page="${current + 1}" ${current === total ? 'disabled' : ''}>
            下一页
        </button>
    `);

    return buttons.join('');
}

// ============================
// 事件处理
// ============================

/**
 * 绑定事件
 */
function bindEvents() {
    document.getElementById('searchBtn').addEventListener('click', () => {
        readFiltersFromUI();
        loadLogs();
    });

    document.getElementById('resetBtn').addEventListener('click', resetFilters);

    document.getElementById('refreshBtn').addEventListener('click', () => {
        loadLogs().then(loaded => {
            if (loaded) showToast('日志数据已刷新', 'info');
        });
    });

    document.getElementById('pageSizeSelect').addEventListener('change', (e) => {
        state.pageSize = parseInt(e.target.value, 10);
        state.currentPage = 1;
        renderTable();
        renderPagination();
    });

    document.getElementById('paginationControls').addEventListener('click', (e) => {
        const btn = e.target.closest('.page-btn');
        if (!btn || btn.disabled) return;

        const page = parseInt(btn.dataset.page, 10);
        if (page && page !== state.currentPage) {
            state.currentPage = page;
            renderTable();
            renderPagination();
        }
    });

    // 输入框回车触发搜索
    document.getElementById('keywordInput').addEventListener('keydown', (e) => {
        if (e.key === 'Enter') {
            readFiltersFromUI();
            loadLogs();
        }
    });

    ['interactionIdInput', 'workflowIdInput', 'licensePlateInput'].forEach(id => {
        document.getElementById(id).addEventListener('keydown', (e) => {
            if (e.key === 'Enter') {
                readFiltersFromUI();
                loadLogs();
            }
        });
    });

    document.getElementById('logsTableBody').addEventListener('click', (e) => {
        const button = e.target.closest('.interaction-action-btn');
        if (button?.dataset.interactionId) {
            showInteractionDetails(button.dataset.interactionId, button);
        }
    });

    document.getElementById('interactionModalClose').addEventListener('click', closeInteractionModal);
    document.getElementById('interactionModal').addEventListener('click', (e) => {
        if (e.target.id === 'interactionModal') closeInteractionModal();
    });
    document.addEventListener('keydown', (e) => {
        const modal = document.getElementById('interactionModal');
        if (modal.hidden) return;
        if (e.key === 'Escape') {
            e.preventDefault();
            closeInteractionModal();
        } else if (e.key === 'Tab') {
            const close = document.getElementById('interactionModalClose');
            const body = document.getElementById('interactionModalBody');
            if (e.shiftKey && document.activeElement === close) {
                e.preventDefault();
                body.focus();
            } else if (!e.shiftKey && document.activeElement === body) {
                e.preventDefault();
                close.focus();
            }
        }
    });
}

/**
 * 重置筛选条件
 */
function resetFilters() {
    document.getElementById('keywordInput').value = '';
    document.getElementById('startTimeInput').value = '';
    document.getElementById('endTimeInput').value = '';
    document.getElementById('levelSelect').value = '';
    document.getElementById('sourceSelect').value = '';
    document.getElementById('interactionIdInput').value = '';
    document.getElementById('workflowIdInput').value = '';
    document.getElementById('licensePlateInput').value = '';
    document.getElementById('integrationTypeSelect').value = '';

    state.filters = {
        keyword: '',
        startTime: '',
        endTime: '',
        level: '',
        source: '',
        interactionId: '',
        workflowId: '',
        licensePlate: '',
        integrationType: ''
    };

    loadLogs();
}

function formatBody(body) {
    if (!body) return '';
    try {
        return JSON.stringify(JSON.parse(body), null, 2);
    } catch {
        return body;
    }
}

function interactionTypeLabel(type) {
    return {
        LicensePlate: '获取车牌号',
        VehicleDimensions: '获取车辆尺寸',
        CoalVolume: '获取煤炭体积',
        VehicleArrival: '来车通知'
    }[type] || type || '未知交互';
}

let interactionTrigger = null;
let interactionRequest = 0;

async function showInteractionDetails(interactionId, trigger) {
    const request = ++interactionRequest;
    interactionTrigger = trigger || document.activeElement;
    const modal = document.getElementById('interactionModal');
    const body = document.getElementById('interactionModalBody');
    document.getElementById('interactionModalSubtitle').textContent = interactionId;
    body.innerHTML = '<div class="interaction-hop-meta">正在加载完整链路日志…</div>';
    modal.hidden = false;
    document.getElementById('interactionModalClose').focus();

    try {
        const result = await API.get(`${CONFIG.apiUrl}/interaction/${encodeURIComponent(interactionId)}`);
        if (request !== interactionRequest) return;
        const payload = result?.data ?? result;
        const items = Array.isArray(payload) ? payload : [];
        if (items.length === 0) {
            body.innerHTML = '<div class="interaction-hop-meta">未找到该交互 ID 的链路日志。可能是数据库迁移尚未执行，或日志已超过保留期。</div>';
            return;
        }

        body.innerHTML = items.map((item, index) => {
            const status = Number(item.httpStatus || 0);
            const itemLevel = LEVELS[item.level] ? item.level : (status >= 500 ? 'error' : status >= 400 ? 'warn' : 'info');
            const levelMeta = LEVELS[itemLevel];
            const isLifecycle = (item.recordKind || '').toLowerCase() === 'lifecycle';
            const stageLabel = item.lifecycleStage || (isLifecycle ? 'Lifecycle' : 'HTTP Access');
            const requestHeaders = formatBody(item.requestHeadersJson);
            const requestBody = formatBody(item.requestBody);
            const responseHeaders = formatBody(item.responseHeadersJson);
            const responseBody = formatBody(item.responseBody);
            return `
                <article class="interaction-hop">
                    <div class="interaction-hop-title">
                        <span>${index + 1}. ${escapeHtml(interactionTypeLabel(item.integrationType))} · ${escapeHtml(item.direction === 'Outbound' ? '出站' : '入站')} · ${escapeHtml(stageLabel)}</span>
                        <span class="log-level ${levelMeta.class}">${escapeHtml(levelMeta.label)}${status ? ` · HTTP ${escapeHtml(String(status))}` : ''}</span>
                    </div>
                    ${item.message ? `<div class="interaction-hop-meta"><strong>${escapeHtml(item.message)}</strong></div>` : ''}
                    <div class="interaction-hop-meta">
                        开始 ${escapeHtml(item.startTime || item.time || '')} · 完成 ${escapeHtml(item.time || '')} · ${escapeHtml(item.callerSystem || '-')} → ${escapeHtml(item.targetSystem || '-')} · ${escapeHtml(item.httpMethod || '')} ${escapeHtml(item.requestPath || '')}${escapeHtml(item.queryString || '')}
                        · 耗时 ${escapeHtml(String(item.elapsedMs ?? 0))} ms ${status ? `· 业务码 ${escapeHtml(String(item.businessCode ?? 0))}` : ''}
                        · 第 ${escapeHtml(String(item.attemptNo ?? 1))} 次
                        ${item.workflowId ? ` · 流程 ${escapeHtml(item.workflowId)}` : ''}
                        ${item.licensePlate ? ` · 车牌 ${escapeHtml(item.licensePlate)}` : ''}
                        ${item.cameraIdentifier ? ` · 相机 ${escapeHtml(item.cameraIdentifier)}` : ''}
                    </div>
                    <div class="interaction-hop-meta">
                        ${item.clientIp ? `调用 IP ${escapeHtml(item.clientIp)} · ` : ''}
                        ${item.traceIdentifier ? `Trace ${escapeHtml(item.traceIdentifier)} · ` : ''}
                        ${item.idempotencyKeyHeader ? `幂等键 ${escapeHtml(item.idempotencyKeyHeader)} · ` : ''}
                        ${item.signatureHeader ? `签名摘要 ${escapeHtml(item.signatureHeader)} · ` : ''}
                        请求 ${escapeHtml(String(item.requestContentLength ?? 0))} 字节 · 响应 ${escapeHtml(String(item.responseContentLength ?? 0))} 字节
                    </div>
                    ${item.errorMessage ? `<div class="interaction-hop-meta" style="color: var(--accent-danger);">错误：${escapeHtml(item.errorMessage)}</div>` : ''}
                    ${requestHeaders ? `<pre><strong>请求头（已脱敏）</strong>\n${escapeHtml(requestHeaders)}</pre>` : ''}
                    ${requestBody ? `<pre><strong>请求体</strong>\n${escapeHtml(requestBody)}</pre>` : ''}
                    ${responseHeaders ? `<pre><strong>响应头（已脱敏）</strong>\n${escapeHtml(responseHeaders)}</pre>` : ''}
                    ${responseBody ? `<pre><strong>响应体</strong>\n${escapeHtml(responseBody)}</pre>` : ''}
                </article>
            `;
        }).join('');
    } catch (error) {
        if (request !== interactionRequest) return;
        body.innerHTML = `<div class="interaction-hop-meta" style="color: var(--accent-danger);">链路查询失败：${escapeHtml(error.message || String(error))}</div>`;
    }
}

function closeInteractionModal() {
    interactionRequest++;
    document.getElementById('interactionModal').hidden = true;
    interactionTrigger?.focus();
}

/**
 * 显示临时提示
 */
function showToast(message, type = 'info') {
    const toast = document.createElement('div');
    toast.className = 'error-toast';
    toast.style.background = type === 'warn'
        ? 'linear-gradient(135deg, var(--accent-warning), #d97706)'
        : type === 'error'
            ? 'linear-gradient(135deg, var(--accent-danger), #dc2626)'
            : 'linear-gradient(135deg, var(--accent-primary), var(--accent-secondary))';
    toast.textContent = message;
    document.body.appendChild(toast);

    setTimeout(() => {
        toast.style.opacity = '0';
        toast.style.transform = 'translateX(-50%) translateY(-10px)';
        toast.style.transition = 'all 0.3s ease';
        setTimeout(() => toast.remove(), 300);
    }, 3000);
}

// ============================
// 初始化
// ============================

document.addEventListener('DOMContentLoaded', () => {
    // 设置默认时间范围：最近 24 小时
    const now = new Date();
    const yesterday = new Date(now.getTime() - 24 * 60 * 60 * 1000);
    document.getElementById('endTimeInput').value = formatDateTimeLocal(now);
    document.getElementById('startTimeInput').value = formatDateTimeLocal(yesterday);

    bindEvents();

    loadLogs().then(() => {
        // 初始读取默认时间范围
        readFiltersFromUI();
        applyFilters();
    });
});
