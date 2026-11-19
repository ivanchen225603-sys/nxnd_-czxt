/**
 * 磅房双雷达实时点云页面
 *
 * 两条链路在页面上必须始终可区分：
 * 1. 实时预览：只读、不落盘、不产生业务数据，按秒轮询版本号，画面随雷达周期约 7 秒一变。
 * 2. 保存并计算：仅管理员可驱动采集和写入，viewer 只能预览与查询已有结果。
 *
 * 页面刻意不把"已捕获""已落盘""分析完成""已入库"合并成一个笼统的成功态：
 * 把收到点云当成数据已安全，是这条链路上最容易造成误判的地方。
 */
(function () {
    'use strict';

    /** 预览帧轮询间隔。雷达周期 7 秒，1 秒轮询足以让画面第一时间更新。 */
    const FRAME_POLL_MS = 1000;
    /** 状态区轮询间隔。状态变化远慢于画面，2 秒足够且更省。 */
    const STATUS_POLL_MS = 2000;
    /** 采集进度轮询间隔。 */
    const CAPTURE_POLL_MS = 1500;
    const STAGE_FLOW = [
        { key: 'waiting', label: '正在采集双雷达点云' },
        { key: 'captured', label: '双雷达点云已捕获' },
        { key: 'saved', label: '两份 PCD 已落盘' },
        { key: 'queued', label: '已配对，排队分析' },
        { key: 'analyzing', label: 'Analyzer 正在融合计算' },
        { key: 'completed', label: '计算完成并写入数据库' }
    ];
    const FAILURE_STAGES = {
        progress_unavailable: '本次进度不可查询，可刷新结果',
        capture_failed: '采集失败',
        persistence_failed: 'PCD 落盘失败',
        analysis_failed: '分析失败',
        // 不是故障，但同样是终局：必须让进度停下来，
        // 否则页面会一直等一条永远不会出现的数据库记录。
        completed_without_dimensions: '分析完成，逐项结果待核对（未入库）'
    };

    const dom = {};
    let renderer = null;
    let frameTimer = null;
    let statusTimer = null;
    let captureTimer = null;
    let currentVersion = 0;
    let displayExpiresAt = 0;
    let displayedPairDelta = null;
    let observedCycle = null;
    let currentSessionId = null;
    let currentWorkflowId = null;
    let captureIdempotencyKey = null;
    let capturePlate = null;
    /** 待人工确认时保留的车牌，供"确认未采集"按钮复用同一次流程。 */
    let pendingConfirmPlate = null;
    let hasFittedOnce = false;
    let watching = false;
    let watchGeneration = 0;
    let previewGeneration = 0;
    let frameBusy = false;
    let statusBusy = false;
    let captureGeneration = 0;
    let dispatchBusy = false;
    let progressBusy = false;
    /** 本次采集是否仍在进行。关掉模态框不会停止轮询，只是把它转入后台。 */
    let captureActive = false;
    /** 从已存在流程链接打开时仅查询，绝不走自动重发或取消采集。 */
    let readOnlyResult = false;
    let canCapture = false;

    document.addEventListener('DOMContentLoaded', init);

    function init() {
        cacheDom();
        canCapture = API.getCurrentUser()?.roleCode === 'admin';
        if (!canCapture) {
            dom.btnCapture.disabled = true;
            dom.btnCapture.hidden = true;
            dom.btnCancelCapture.hidden = true;
            dom.btnConfirmRedispatch.hidden = true;
            dom.plateInput.disabled = true;
            showBanner(dom.captureHint, 'info', '当前账号仅可查看预览和采集结果；发起或取消采集需要管理员。');
        }
        bindEvents();

        try {
            renderer = PointCloudGL.create(dom.canvas);
            requestAnimationFrame(function loop() {
                renderer.render();
                requestAnimationFrame(loop);
            });
        } catch (err) {
            showBanner(dom.previewBanner, 'error', err.message);
            return;
        }

        renderProgress(null);
        hideConfirmAction();
        // 初始可见性由脚本决定，不依赖内联样式：模板改动不该悄悄改变启动状态。
        closeCaptureModal();
        startWatching();
        openLinkedResult();
    }

    function cacheDom() {
        dom.canvas = document.getElementById('liveCanvas');
        dom.overlay = document.getElementById('canvasOverlay');
        dom.previewBanner = document.getElementById('previewBanner');
        dom.previewTitle = document.getElementById('previewTitle');
        dom.captureBanner = document.getElementById('captureBanner');
        dom.cameraAStatus = document.getElementById('cameraAStatus');
        dom.cameraBStatus = document.getElementById('cameraBStatus');
        dom.cameraATitle = document.getElementById('cameraATitle');
        dom.cameraBTitle = document.getElementById('cameraBTitle');
        dom.pairStatus = document.getElementById('pairStatus');
        dom.captureHint = document.getElementById('captureHint');
        dom.captureModal = document.getElementById('captureModal');
        dom.captureModalMeta = document.getElementById('captureModalMeta');
        dom.btnCloseModal = document.getElementById('btnCloseModal');
        dom.btnReopenProgress = document.getElementById('btnReopenProgress');
        dom.btnRefreshResult = document.getElementById('btnRefreshResult');
        dom.progressSteps = document.getElementById('progressSteps');
        dom.resultArea = document.getElementById('resultArea');
        dom.resultSummary = document.getElementById('resultSummary');
        dom.plateInput = document.getElementById('plateInput');
        dom.btnStartWatch = document.getElementById('btnStartWatch');
        dom.btnStopWatch = document.getElementById('btnStopWatch');
        dom.btnCapture = document.getElementById('btnCapture');
        dom.btnCancelCapture = document.getElementById('btnCancelCapture');
        dom.confirmAction = document.getElementById('confirmAction');
        dom.btnConfirmRedispatch = document.getElementById('btnConfirmRedispatch');
        dom.pointSize = document.getElementById('pointSize');
        dom.showCompletion = document.getElementById('showCompletion');
        dom.completionStatus = document.getElementById('completionStatus');
    }

    function bindEvents() {
        dom.showCompletion.addEventListener('change', () => {
            clearPreview('预览模式已切换，等待新画面');
            pollFrame();
        });
        dom.btnStartWatch.addEventListener('click', startWatching);
        dom.btnStopWatch.addEventListener('click', stopWatching);
        dom.btnCapture.addEventListener('click', () => startCapture());
        dom.btnConfirmRedispatch.addEventListener('click', () => startCapture({ force: true }));
        dom.btnCancelCapture.addEventListener('click', cancelCapture);
        dom.btnCloseModal.addEventListener('click', closeCaptureModal);
        dom.btnReopenProgress.addEventListener('click', openCaptureModal);
        dom.btnRefreshResult.addEventListener('click', pollCapture);
        dom.pointSize.addEventListener('input', () => {
            renderer.setPointSize(Number(dom.pointSize.value) || 2);
        });
        document.querySelectorAll('[data-view]').forEach((button) => {
            button.addEventListener('click', () => renderer.setView(button.getAttribute('data-view')));
        });
    }

    // ---------------------------------------------------------------- 实时预览

    /**
     * 开始接收数据。
     * 只影响浏览器是否拉取，不向雷达下发任何指令——雷达全程持续扫描，
     * 页面开合不会让它反复休眠唤醒。
     */
    function startWatching() {
        if (watching) return;
        watching = true;
        watchGeneration++;
        clearPreview('正在连接实时预览');
        dom.btnStartWatch.disabled = true;
        dom.btnStopWatch.disabled = false;
        pollStatus();
        pollFrame();
        statusTimer = setInterval(pollStatus, STATUS_POLL_MS);
        frameTimer = setInterval(pollFrame, FRAME_POLL_MS);
    }

    function stopWatching() {
        watching = false;
        watchGeneration++;
        clearPreview('预览已暂停，不显示历史画面');
        dom.btnStartWatch.disabled = false;
        dom.btnStopWatch.disabled = true;
        clearInterval(statusTimer);
        clearInterval(frameTimer);
        statusTimer = null;
        frameTimer = null;
    }

    function clearPreview(message) {
        previewGeneration++;
        currentVersion = 0;
        displayExpiresAt = 0;
        dom.completionStatus.textContent = dom.showCompletion.checked
            ? '蓝色：实测车辆点；紫色：合成壁面（只读参考）' : '原始双雷达叠加，不显示合成点';
        displayedPairDelta = null;
        renderPairStatus();
        hasFittedOnce = false;
        if (renderer) renderer.setPoints(new Float32Array(0), 0, 0, false);
        dom.previewTitle.textContent = watching ? '实时预览 · 仅供监看' : '预览已暂停';
        dom.overlay.textContent = message || '当前没有有效双雷达画面';
    }

    async function pollFrame() {
        if (displayExpiresAt && Date.now() >= displayExpiresAt) clearPreview('补点源帧已过期，等待新画面');
        if (!watching || frameBusy) return;
        frameBusy = true;
        const generation = watchGeneration;
        const previewAtStart = previewGeneration;
        try {
            const endpoint = dom.showCompletion.checked ? 'completed-frame' : 'fused-frame';
            const result = await API.request('/api/v1/radar-live/' + endpoint + '?afterVersion=' + currentVersion);
            if (!watching || generation !== watchGeneration || previewAtStart !== previewGeneration) return;
            const frame = result.data;
            if (!frame) {
                clearPreview('预览响应缺少有效画面');
                return;
            }

            // Worker 重启后版本号从零重新开始。只比版本号的话，浏览器手里的旧版本会一直
            // 大于新版本，新帧被当成旧帧丢掉，画面可以几分钟不动且没有任何报错。
            if (frame.sessionId && frame.sessionId !== currentSessionId) {
                currentSessionId = frame.sessionId;
                clearPreview('Worker 会话已更新，等待新画面');
            }

            if (frame.updated !== true) {
                if (frame.preview && !frame.preview.available) clearPreview(frame.preview.unavailableReason);
                return;
            }

            currentVersion = frame.version;
            displayExpiresAt = frame.expiresAt ? Date.parse(frame.expiresAt) : 0;
            if (frame.referenceOnly && (!Number.isFinite(displayExpiresAt) || displayExpiresAt <= Date.now())) {
                clearPreview('补点结果已过期，等待新画面');
                return;
            }
            const coordinates = decodePoints(frame.points);
            const synthetic = frame.syntheticPointCount || 0;
            if (coordinates.length !== (frame.pointCountA + frame.pointCountB + synthetic) * 3 ||
                !coordinates.every(Number.isFinite)) throw new Error('点云数据格式错误');
            let surface = null;
            if (frame.displayMesh != null) {
                const mesh = frame.displayMesh;
                if (frame.referenceOnly !== true || mesh.displayOnly !== true || mesh.encoding !== 'xyz-triangles-v1' ||
                    !Array.isArray(mesh.coordinates) || mesh.coordinates.length < 9 || mesh.coordinates.length > 180 ||
                    mesh.coordinates.length % 9 !== 0 ||
                    mesh.coordinates.some(v => typeof v !== 'number' || !Number.isFinite(v) || Math.abs(v) > 1000)) {
                    throw new Error('补点显示面数据不符合契约');
                }
                surface = new Float32Array(mesh.coordinates);
            }
            renderer.setPoints(coordinates, frame.pointCountA, frame.pointCountB, !hasFittedOnce, synthetic, surface);
            hasFittedOnce = true;
            updateOverlay(frame);
            if (frame.referenceOnly) {
                const status = (frame.loadStatus || {}).status;
                dom.completionStatus.textContent = synthetic > 0
                    ? '已显示 ' + formatNumber(synthetic) + ' 个紫色合成点（不参与正式测量，不入库）'
                    : '本帧未补点：' + (status === 'Loaded' ? '载货车' : status === 'Uncertain' ? '载荷不确定' : '内壁不可靠或无需补齐');
                if (surface) {
                    const lip = frame.wallCompletion && frame.wallCompletion.frontLip;
                    dom.completionStatus.textContent += '；四壁连续显示面已生成，保持开顶、不补底板。' +
                        (lip && lip.completed ? '前壁折边已生成' + (lip.innerDropGenerated ? '（含短下折面）' : '（无可靠短下折面）')
                            : '前壁折边未生成：实测支持未通过；原因：' + (lip && lip.reason || '未返回'));
                }
            }
        } catch (err) {
            if (!watching || generation !== watchGeneration) return;
            clearPreview('预览连接中断，等待有效新帧');
            if (dom.showCompletion.checked && err && err.code === 404) {
                dom.completionStatus.textContent = '补点接口尚未加载：需要更新本机后台和 Analyzer；可取消勾选继续看原始点云。';
            }
            if (err && err.code === 503) {
                showBanner(dom.previewBanner, 'warn', err.message);
            } else if (err && err.code !== 401) {
                showBanner(dom.previewBanner, 'error', err.message || '预览拉取失败');
            }
        } finally { frameBusy = false; }
    }

    /**
     * 解码服务端的 base64(Float32 小端 xyz) 点云缓冲。
     * 与 JSON 数字数组相比体积约为三分之一，且可直接映射成 Float32Array。
     */
    function decodePoints(base64) {
        const binary = atob(base64);
        const bytes = new Uint8Array(binary.length);
        for (let i = 0; i < binary.length; i++) {
            bytes[i] = binary.charCodeAt(i);
        }
        return new Float32Array(bytes.buffer);
    }

    function updateOverlay(frame) {
        displayedPairDelta = frame.pairDeltaMilliseconds;
        renderPairStatus();
        dom.overlay.innerHTML = [
            '更新时间：' + formatTime(frame.capturedAt),
            frame.cameraA + '：' + formatNumber(frame.sourcePointCountA) + ' 点',
            frame.cameraB + '：' + formatNumber(frame.sourcePointCountB) + ' 点',
            '预览点数：' + formatNumber(frame.previewPointCount) + ' 点',
            '配对时间差：' + (frame.pairDeltaMilliseconds / 1000).toFixed(2) + ' 秒',
            frame.referenceOnly ? '数据状态：车厢补点参考（非正式测量，未入库）' : '数据状态：双雷达预览叠加（非测量质量结论）'
        ].map(escapeHtml).join('<br>');
    }

    async function pollStatus() {
        if (!watching || statusBusy) return;
        statusBusy = true;
        const generation = watchGeneration;
        try {
            const result = await API.request('/api/v1/radar-live/status');
            if (!watching || generation !== watchGeneration) return;
            renderStatus(result.data);
        } catch (err) {
            if (!watching || generation !== watchGeneration) return;
            clearPreview('雷达状态不可用，等待重新连接');
            if (err && err.code === 401) return;
            showBanner(dom.previewBanner, 'error', err.message || '雷达状态不可用');
        } finally { statusBusy = false; }
    }

    function renderStatus(status) {
        const preview = status.preview || {};
        const scan = status.scan || {};
        const radars = status.radars || [];
        const cameras = preview.cameras || [];

        if (preview.available) {
            hideBanner(dom.previewBanner);
        } else {
            clearPreview(preview.unavailableReason);
            showBanner(dom.previewBanner, 'warn', preview.unavailableReason || '预览不可用');
        }

        const nameA = (cameras[0] && cameras[0].cameraIdentifier) || 'CAM_004';
        const nameB = (cameras[1] && cameras[1].cameraIdentifier) || 'CAM_005';
        dom.cameraATitle.textContent = '■ ' + nameA + '（基准 A）';
        dom.cameraBTitle.textContent = '■ ' + nameB + '（B → A）';
        dom.cameraAStatus.innerHTML = renderCamera(radars, cameras[0], nameA);
        dom.cameraBStatus.innerHTML = renderCamera(radars, cameras[1], nameB);

        observedCycle = scan.estimatedCycleSeconds;
        if (preview.sessionId && preview.sessionId !== currentSessionId) {
            // 状态区通常先于帧接口发现重启，这里立刻重置，避免多等一个轮询周期。
            // hasFittedOnce 必须一起重置：重启后的第一帧要重新自适应视角，
            // 否则相机还停在上一进程的取景范围，画面可能整个在视野之外。
            currentSessionId = preview.sessionId;
            clearPreview('Worker 会话已更新，等待新画面');
        }

        renderPairStatus();

    }

    function renderPairStatus() {
        dom.pairStatus.innerHTML = [
            row('画面配对时间差', displayedPairDelta == null ? '—' : (displayedPairDelta / 1000).toFixed(2) + ' 秒'),
            row('解析点云周期', observedCycle == null ? '待测' : observedCycle.toFixed(3) + ' 秒'),
            row('画面版本', String(currentVersion))
        ].join('');
    }

    function renderCamera(radars, previewCamera, name) {
        const radar = radars.find((item) => item.cameraIdentifier === name) || {};
        const connected = radar.connected;

        let connectBadge;
        if (connected === true) {
            connectBadge = badge('ok', '已连接');
        } else if (connected === false) {
            connectBadge = badge('bad', '断开');
        } else {
            connectBadge = badge('idle', '未知');
        }

        const ageSeconds = previewCamera && previewCamera.ageSeconds;
        const freshBadge = !previewCamera || !previewCamera.hasSample
            ? badge('idle', '无数据')
            : previewCamera.fresh
                ? badge('ok', ageSeconds.toFixed(1) + 's 前')
                : badge('bad', '已过期 ' + ageSeconds.toFixed(0) + 's');

        return [
            row('连接', connectBadge),
            row('最近解析点云', freshBadge),
            row('原始点数', previewCamera && previewCamera.sourcePointCount != null
                ? formatNumber(previewCamera.sourcePointCount)
                : '—'),
            row('累计原始包', radar.rawFrameCount != null ? formatNumber(radar.rawFrameCount) : '—')
        ].join('');
    }

    // ------------------------------------------------------------ 保存并计算

    function openLinkedResult() {
        const search = typeof location === 'undefined' ? '' : location.search;
        const workflowId = new URLSearchParams(search).get('workflowId');
        if (workflowId === null) return;
        // 现有手动流程使用 sess_...；只接纳路径安全的既有 ID，不把查询参数当指令。
        if (!/^[A-Za-z0-9_-]{1,128}$/.test(workflowId)) {
            showBanner(dom.captureHint, 'error', '结果流程编号格式无效');
            return;
        }
        currentWorkflowId = workflowId;
        readOnlyResult = true;
        captureGeneration++;
        openCaptureModal();
        setModalMeta(workflowId, null);
        dom.btnCancelCapture.disabled = true;
        showBanner(dom.captureBanner, 'warn', '正在只读查询本次结果，不会重新采集');
        pollCapture();
    }

    async function startCapture(options) {
        if (!canCapture || dispatchBusy) return;
        const force = !!(options && options.force);
        hideBanner(dom.captureBanner);
        hideConfirmAction();
        const plate = (force && pendingConfirmPlate ? pendingConfirmPlate
            : (dom.plateInput.value || '')).replace(/\s/g, '').toUpperCase();
        if (!plate) {
            // 还没发起任何请求，弹模态框只会挡住要填的输入框。
            showBanner(dom.captureHint, 'error', '请先输入本次过车的车牌号');
            dom.plateInput.focus();
            return;
        }
        hideBanner(dom.captureHint);
        readOnlyResult = false;
        currentWorkflowId = null;
        dom.btnCancelCapture.disabled = true;
        captureActive = true;
        openCaptureModal();
        setModalMeta(null, plate);
        renderProgress(null);
        dom.resultArea.className = 'result-empty';
        dom.resultArea.innerHTML = '正在向雷达发起采集…';
        renderSummary(null, null);

        // 幂等键在第一次点击时生成并保留：请求超时后无法判断 Worker 是否已受理，
        // 同一车牌重试沿用原键；操作员修改车牌则建立新请求，避免旧键与新车牌冲突。
        if (!captureIdempotencyKey || capturePlate !== plate) {
            capturePlate = plate;
            captureIdempotencyKey = 'live-' + Date.now().toString(36) + '-'
                + Math.random().toString(36).slice(2, 10);
        }

        dom.btnCapture.disabled = true;
        dispatchBusy = true;
        captureGeneration++;
        clearInterval(captureTimer);
        captureTimer = null;
        try {
            const result = await API.request('/api/v1/radar-live/captures', {
                method: 'POST',
                headers: { 'Idempotency-Key': captureIdempotencyKey },
                body: {
                    licensePlate: plate,
                    idempotencyKey: captureIdempotencyKey,
                    confirmOverwriteEmptyProfile: true,
                    forceRedispatch: force
                }
            });
            currentWorkflowId = result.data.workflowId;
            pendingConfirmPlate = result.data.licensePlate;
            dom.btnCancelCapture.disabled = false;
            setModalMeta(currentWorkflowId, result.data.licensePlate);
            showBanner(dom.captureBanner, result.data.duplicate ? 'warn' : 'ok',
                (result.data.duplicate ? '该请求已在进行中（未重复采集），' : '已受理，')
                + '流程编号 ' + currentWorkflowId + '；车牌 ' + result.data.licensePlate);
            dom.resultArea.className = 'result-empty';
            dom.resultArea.innerHTML = '采集进行中…';
            pollCapture();
            captureTimer = setInterval(pollCapture, CAPTURE_POLL_MS);
        } catch (err) {
            dom.btnCapture.disabled = false;
            captureActive = false;
            // 409 有两种：车牌已有空车档案（勾选确认后重试），
            // 或服务端无法核实上一次派发结果（必须人工判断本车是否已经采过）。
            // 两种都保留幂等键：确认后再点必须收敛到同一次采集。
            const level = err.code === 409 ? 'warn' : 'error';
            showBanner(dom.captureBanner, level, err.message || '发起采集失败');
            renderDispatchFailure(plate, err);
            if (err.code === 409 && /无法核实/.test(err.message || '')) {
                showConfirmAction();
            }
        } finally { dispatchBusy = false; }
    }

    function renderDispatchFailure(plate, err) {
        // 仅识别当前服务器明确的门禁拒绝；普通 503/超时不能证明雷达没有执行。
        const gateRejected = err.code === 503 &&
            /^采集安全门禁未满足[：:]\s*acquisition_unverified(?:\s|$)/.test(err.message || '');
        const title = gateRejected ? '采集未启动：完整场采集条件尚未确认'
            : err.code === 409 ? '请求未完成：需要确认后重试'
                : '未取得采集受理确认';
        dom.btnCancelCapture.disabled = true;
        dom.captureModalMeta.textContent = '车牌 ' + plate + ' · 未收到流程编号';
        dom.progressSteps.innerHTML = '<div class="progress-step failed"><span>✕</span><span>'
            + escapeHtml(title) + '</span></div>';
        dom.resultArea.className = 'result-empty';
        dom.resultArea.textContent = gateRejected
            ? '本次请求已被雷达拒绝，未保存 PCD、未进行计算。需要先核实完整场、最大交付延迟及扫描周期；实时预览可继续使用。'
            : '尚未取得本次计算结果。请按上方提示处理；网络异常不代表采集未执行，请保留当前页面并使用同一请求重试，勿换编号重复采集。';
        dom.resultSummary.innerHTML = '<p class="summary-empty">' + escapeHtml(title)
            + '。请保留当前请求，按流程详情中的提示处理。</p>';
        // 来车记录可能已经建立。保留幂等键，不自动重发，也不假称流程不存在。
        updateReopenButton();
    }

    async function cancelCapture() {
        if (!canCapture || !currentWorkflowId || readOnlyResult || dom.btnCancelCapture.disabled) return;
        try {
            const result = await API.request('/api/v1/radar-live/captures/' + encodeURIComponent(currentWorkflowId) + '/cancel', {
                method: 'POST'
            });
            showBanner(dom.captureBanner, 'warn', result.data.message || '停止请求已发送');
        } catch (err) {
            showBanner(dom.captureBanner, 'error', err.message || '取消失败');
        }
    }

    async function pollCapture() {
        if (!currentWorkflowId || progressBusy) return;
        progressBusy = true;
        dom.btnRefreshResult.disabled = true;
        const generation = captureGeneration;
        const workflowId = currentWorkflowId;
        try {
            const result = await API.request(
                '/api/v1/radar-live/captures/' + encodeURIComponent(workflowId));
            if (generation !== captureGeneration || workflowId !== currentWorkflowId) return;
            const progress = result.data;
            dom.btnCancelCapture.disabled = !canCapture || readOnlyResult || progress.terminal
                || !['waiting', 'captured'].includes(progress.stage)
                || (progress.capturedCameras || []).length >= 2;
            setModalMeta(workflowId, progress.licensePlate);
            renderProgress(progress);
            renderResult(progress);
            if (!progress.terminal && progress.error === 'analysis_terminal_unavailable') {
                showBanner(dom.captureBanner, 'warn', progress.message);
            }

            if (!progress.terminal && !readOnlyResult && !captureTimer) {
                captureActive = true;
                dom.btnCapture.disabled = true;
                captureTimer = setInterval(pollCapture, CAPTURE_POLL_MS);
            }
            if (progress.terminal) {
                clearInterval(captureTimer);
                captureTimer = null;
                dom.btnCapture.disabled = false;
                dom.btnCancelCapture.disabled = true;
                // 本次流程已定案，下一次点击必须是一次全新的采集。
                if (!readOnlyResult) captureIdempotencyKey = null;
                pendingConfirmPlate = null;
                captureActive = false;
                hideConfirmAction();
                updateReopenButton();
                if (progress.stage === 'completed') {
                    showBanner(dom.captureBanner, progress.dimensions && progress.dimensions.reviewStatus === 'pending' ? 'warn' : 'ok', progress.message);
                } else if (progress.stage === 'completed_unpersisted' || progress.stage === 'progress_unavailable') {
                    showBanner(dom.captureBanner, 'warn', progress.message);
                } else if (progress.stage === 'completed_without_dimensions' || progress.stage === 'pending_review') {
                    // 待审核计算值与不产出尺寸都是正常终局，不是分析故障。
                    showBanner(dom.captureBanner, 'warn', progress.stage === 'completed_without_dimensions'
                        ? '分析完成，已计算项目和缺测原因已分别显示；本次未生成完整尺寸入库记录。'
                        : progress.message);
                } else {
                    showBanner(dom.captureBanner, 'error',
                        progress.message + (progress.error ? '：' + progress.error : ''));
                }
            }
        } catch (err) {
            if (generation !== captureGeneration) return;
            if (err && err.code === 401) return;
            clearInterval(captureTimer);
            captureTimer = null;
            captureActive = false;
            dom.btnCapture.disabled = false;
            dom.btnCancelCapture.disabled = true;
            updateReopenButton();
            showBanner(dom.captureBanner, 'error', (err.message || '进度查询失败') + '；可点击“刷新本次结果”继续查询原流程。');
        } finally {
            progressBusy = false;
            dom.btnRefreshResult.disabled = !currentWorkflowId;
        }
    }

    function renderProgress(progress) {
        const stage = progress ? progress.stage : null;
        if (stage === 'pending_review' || stage === 'completed_unpersisted') {
            dom.progressSteps.innerHTML = STAGE_FLOW.slice(0, -1).map(item =>
                '<div class="progress-step done"><span>●</span><span>'
                + escapeHtml(item.key === 'analyzing' ? '融合与计算完成' : item.label) + '</span></div>'
            ).join('') + '<div class="progress-step active"><span>◍</span><span>'
                + (stage === 'completed_unpersisted' ? '计算完成（未查到入库记录）' : '待人工审核（未入库）')
                + '</span></div>';
            return;
        }
        const failureLabel = stage ? FAILURE_STAGES[stage] : null;
        const activeIndex = STAGE_FLOW.findIndex((item) => item.key === stage);

        const rows = STAGE_FLOW.map((item, index) => {
            let cls = '';
            let icon = '○';
            if (failureLabel) {
                // 失败时不再假装后续步骤"未开始"：已经完成的保持绿色，其余置灰。
                cls = index < failedBoundary(stage) ? 'done' : '';
                icon = index < failedBoundary(stage) ? '●' : '○';
            } else if (activeIndex >= 0 && index < activeIndex) {
                cls = 'done';
                icon = '●';
            } else if (index === activeIndex) {
                cls = stage === 'completed' ? 'done' : 'active';
                icon = stage === 'completed' ? '●' : '◍';
            }
            return '<div class="progress-step ' + cls + '"><span>' + icon + '</span><span>'
                + escapeHtml(item.label) + '</span></div>';
        });

        if (failureLabel) {
            rows.push('<div class="progress-step failed"><span>✕</span><span>'
                + escapeHtml(failureLabel) + '</span></div>');
        }

        if (stage === 'completed' && progress.dimensions && progress.dimensions.reviewStatus === 'pending') {
            rows.push('<div class="progress-step active"><span>◍</span><span>已入库，待人工审核</span></div>');
        }
        dom.progressSteps.innerHTML = rows.join('');
    }

    /** 失败发生在哪一步之后：用于把失败前已完成的阶段保持为完成态。 */
    function failedBoundary(stage) {
        if (stage === 'capture_failed') return 1;
        if (stage === 'persistence_failed') return 2;
        if (stage === 'analysis_failed') return 4;
        if (stage === 'completed_without_dimensions') return 5;
        return 0;
    }

    /**
     * 渲染本次采集的业务结果。
     *
     * 呈现磅房实际要核对的字段：车头到车厢前内壁、车厢宽、车厢内高、
     * 底板高度、拉筋数量与间距、高低板。融合质量、点数、PCD 路径这些排障信息
     * 不在这里 —— 操作员要的是能和实车对上的数，不是算法内部状态。
     *
     * @param {object} progress 采集进度。
     */
    function renderResult(progress) {
        const analysis = progress.analysis || {};
        const dimensions = progress.dimensions;
        const storedReview = !!(dimensions && dimensions.reviewStatus === 'pending');

        // 待审核计算值与明细必须同源：分析失败时若这里仍按上一轮数据渲染，
        // 页面会把一次失败的采集显示成有尺寸结果。
        const showReview = (progress.stage === 'pending_review'
            || (progress.stage === 'completed' && storedReview))
            && analysis.stage === 'completed'
            && analysis.measurementStatus && analysis.measurementStatus.reviewStatus === 'pending';

        const data = collectResultFields(progress, showReview);
        renderSummary(progress, data);
        if (data) {
            const cards = [
                resultCard('车头到车厢前内壁', data.cabToFront, data.pending),
                resultCard('车厢内长', data.innerLength, data.pending),
                resultCard(data.widthLabel || '车厢宽（采用值）', data.width, data.pending),
                resultCard('车厢内高', data.innerHeight, data.pending),
                resultCard('前端底板距磅台', data.floorHeight, data.pending),
                resultCard('拉筋数量', data.ribCountText, data.pending)
            ].join('');
            dom.resultArea.className = '';
            dom.resultArea.innerHTML =
                (progress.stage === 'completed_unpersisted' ? '<div class="result-note">本次计算值，尚未查到数据库记录</div>' : '')
                + (data.pending
                    ? '<div class="result-note">模型/计算值，尚未经人工核验</div>'
                    : '')
                + '<div class="dim-cards">' + cards + '</div>'
                + (data.partial ? partialResultDetails(data) : '')
                + lengthMeasurementNote(progress)
                + ribSection(data)
                + plateSection(data);
        } else {
            dom.resultArea.className = 'result-empty';
            dom.resultArea.innerHTML = progress.terminal
                ? '本次未产出车辆尺寸。'
                : '正在计算…';
        }
        // 云图按钮与有没有尺寸无关：本次不产出尺寸时，操作员更需要看点云确认原因。
        if (progress.emptyVehiclePointCloudUrl) {
            const button = document.createElement('button');
            button.className = 'btn btn-secondary';
            button.textContent = '查看本次融合车辆云图（非实时）';
            button.addEventListener('click', async () => {
                stopWatching();
                const generation = watchGeneration;
                button.disabled = true;
                try {
                    const bytes = await API.arrayBuffer(progress.emptyVehiclePointCloudUrl);
                    if (generation !== watchGeneration || watching) return;
                    if (bytes.byteLength > 64 * 1024 * 1024) throw new Error('车辆档案超过网页预览的 64 MiB 上限');
                    const cloud = new PCDParser().parse(bytes);
                    const positions = cloud.positions;
                    if (!positions.length || positions.length % 3 !== 0 || !positions.every(Number.isFinite)) {
                        throw new Error('车辆档案没有有效的有限坐标');
                    }
                    renderer.setPoints(positions, storedReview ? 0 : positions.length / 3, 0, true,
                        storedReview ? positions.length / 3 : 0);
                    dom.overlay.textContent = '本次融合车辆档案（非实时） · 流程 ' + progress.workflowId;
                    dom.previewTitle.textContent = '本次车辆云图 · 非实时';
                    showBanner(dom.previewBanner, 'warn', (storedReview ? '该档案含模型推断点，已入库、待人工审核。' : '') +
                        '正在查看已保存的车辆云图；点击开始预览返回实时地磅画面');
                } catch (err) {
                    if (generation === watchGeneration) {
                        clearPreview('车辆档案读取失败');
                        showBanner(dom.previewBanner, 'error', err.message || '车辆档案读取失败');
                    }
                } finally { button.disabled = false; }
            });
            dom.resultArea.appendChild(button);
        }
    }

    function renderSummary(progress, data) {
        if (!dom.resultSummary) return;
        if (!progress) {
            dom.resultSummary.innerHTML = '<p class="summary-empty">本次请求正在处理；尚未取得计算结果。</p>';
            return;
        }
        const stored = progress.stage === 'completed' && !!progress.dimensions;
        const status = stored ? '已入库' : progress.terminal ? '未入库' : '处理中 · 入库待确认';
        const meta = [progress.licensePlate, progress.workflowId].filter(Boolean).join(' · ');
        dom.resultSummary.innerHTML = '<div class="summary-status"><span>' + escapeHtml(meta)
            + '</span><span class="status-tag ' + (stored ? 'ok' : 'warn') + '">'
            + escapeHtml(status) + '</span>'
            + (stored && progress.dimensions.reviewStatus === 'pending'
                ? '<span class="status-tag warn">待人工审核</span>' : '') + '</div>'
            + (data ? '<div class="summary-metrics">'
                + [['车厢内长', data.innerLength], [data.widthLabel || '车厢宽（采用值）', data.width],
                    ['车厢内高', data.innerHeight], ['车头至前内壁', data.cabToFront]]
                    .map(([label, value]) => '<div class="summary-metric"><span>' + escapeHtml(label)
                        + '</span><strong>' + escapeHtml(value) + '</strong></div>').join('') + '</div>'
                : '<p class="summary-empty">' + escapeHtml(progress.terminal
                    ? '本次未产出尺寸；请查看流程详情中的原因。' : '正在等待本次计算结果。') + '</p>');
    }

    /**
     * 把两种来源的尺寸归一成页面要显示的字段。
     *
     * 已入库记录和本次业务载荷用厘米，旧review计算值用米，两者分支显示：
     * 混排会让操作员把"待核验的推算值"当成已入库的实测结果。
     *
     * @param {object} progress 采集进度。
     * @param {boolean} showReview 是否允许使用未入库的计算值。
     * @returns {object|null} 归一后的字段；本次没有任何尺寸时返回 null。
     */
    function collectResultFields(progress, showReview) {
        const dimensions = progress.dimensions;
        const payload = progress.stage === 'completed_unpersisted' && progress.analysis?.stage === 'completed'
            ? progress.analysis.dimensionPayload : null;
        const stored = dimensions && dimensions.dimensions || (payload ? {
            ...payload.dimensions, ribs: payload.ribs, highLowPlates: payload.highLowPlates,
            actualRibCount: payload.actualRibCount
        } : null);
        if (stored && typeof stored === 'object' && 'trailerLength' in stored) {
            // Analyzer 的约定（exporters.py）：拉筋按前壁→后壁排序，
            // N 根拉筋输出 N+1 段，首段是前壁到第一根。段数不是根数。
            const gaps = (stored.ribs || []).map(item => formatRibGap(item && item.distToPrev, formatCm));
            const analysis = progress.analysis || {};
            const calculated = analysis.reviewMeasurements || {};
            const partial = (analysis.measurementStatus || {}).partial === true
                || stored.trailerLength == null || stored.trailerWidth == null
                || stored.trailerBodyHeight == null;
            const count = Number.isInteger(stored.actualRibCount) ? stored.actualRibCount
                : gaps.length ? gaps.length - 1
                : (Number.isInteger(calculated.ribCount) ? calculated.ribCount : null);
            return {
                pending: dimensions?.reviewStatus === 'pending',
                cabToFront: formatCm(stored.cabToTrailerFrontDist),
                innerLength: formatCm(stored.trailerLength),
                width: formatCm(stored.trailerWidth),
                innerHeight: formatCm(stored.trailerBodyHeight),
                floorHeight: formatCm(stored.trailerBottomToGroundHeight),
                ribCount: count,
                ribCountText: describeRibCount(count),
                ribGaps: gaps,
                plateMeasureUnavailable: (partial || stored.trailerBottomToGroundHeight == null)
                    && !(stored.highLowPlates || []).length,
                plates: (stored.highLowPlates || []).map(plate => ({
                    id: plate && plate.plateId,
                    high: formatCm(plate && plate.highestToGround),
                    low: formatCm(plate && plate.lowestToGround)
                })),
                plateDrop: formatCm(stored.highLowPlateHeight)
            };
        }
        const analysis = progress.analysis || {};
        const review = analysis.reviewMeasurements || {};
        const partial = analysis.stage === 'completed'
            && (progress.stage === 'completed_without_dimensions'
                || (progress.stage === 'pending_review' && review.partial === true));
        if (partial || progress.stage === 'completed_unpersisted' && analysis.stage === 'completed')
            return collectPartialFields(analysis);
        if (!showReview) return null;
        // 下面全部按米格式化；单位不是 m（或缺失，含内层分组）就不显示数值，宁可不出数也不把 cm 当 m。
        if (!isMetreReview(review)) return null;
        const carriage = (review.dimensions || {}).carriage || {};
        const gaps = Array.isArray(review.ribIntervals) ? review.ribIntervals : [];
        // ribCount 已经是根数；ribIntervals 是 N+1 段，兜底时要减 1 才是根数。
        const count = review.ribCount == null
            ? (gaps.length ? gaps.length - 1 : 0)
            : review.ribCount;
        return {
            pending: true,
            cabToFront: formatMeters(review.cabToTrailerFrontDist),
            innerLength: formatMeters(carriage.length),
            width: formatMeters(carriage.width),
            innerHeight: formatMeters(carriage.height),
            floorHeight: formatMeters(review.trailerBottomToGroundHeight),
            ribCount: count,
            ribCountText: describeRibCount(count),
            ribGaps: gaps.map(value => formatRibGap(value, formatMeters)),
            plates: (review.highLowPlates || []).map((plate, index) => ({
                id: plate && plate.plateId != null ? plate.plateId : index + 1,
                high: formatMeters(plate && plate.highestToGround),
                low: formatMeters(plate && plate.lowestToGround)
            })),
            plateDrop: formatMeters(review.highLowPlateHeight)
        };
    }

    /**
     * 审核/部分结果按米显示的前提：顶层 unit 为 m，dimensions 及 vehicle/carriage 若自带 unit 也必须是 m。
     * 与 Worker、WebApi 同口径；只看顶层时，内层 cm 的 540 会被显示成 540 m。
     */
    function isMetreReview(review) {
        if (!review || review.unit !== 'm') return false;
        const dims = review.dimensions;
        if (!dims || typeof dims !== 'object') return true;
        if (dims.unit != null && dims.unit !== 'm') return false;
        return ['vehicle', 'carriage'].every(group => !dims[group] || typeof dims[group] !== 'object'
            || dims[group].unit == null || dims[group].unit === 'm');
    }

    /** 部分结果只是当前分析的展示副本，不生成数据库尺寸，也不猜缺失端点。 */
    function collectPartialFields(analysis) {
        const candidate = analysis.reviewMeasurements;
        const review = candidate && candidate.partial === true && isMetreReview(candidate)
            && candidate.dimensions && candidate.fields ? candidate : null;
        const status = analysis.measurementStatus || {};
        const width = status.widthMeasurement || {};
        const load = analysis.loadStatus || {};
        const carriage = review ? review.dimensions.carriage || {} : {};
        const vehicle = review ? review.dimensions.vehicle || {} : {};
        // 兼容本次旧 Worker 的无尺寸结果：只取已明确有效的侧壁净距，
        // 栏板深度只在物理 Empty 时展示。其余项目仍缺测，不使用 split 候选猜长度。
        const widthValue = review ? carriage.width
            : width.valid === true && width.adoptedSource === 'observed_inner_side_faces'
                ? width.widthM : null;
        const height = review ? carriage.height
            : load.status === 'Empty' ? (load.metrics || {}).wallDepth : null;
        const gaps = review && Array.isArray(review.ribIntervals) ? review.ribIntervals : [];
        const ribCount = review && Number.isInteger(review.ribCount) && review.ribCount >= 0
            ? review.ribCount : null;
        const fields = review ? review.fields : {};
        const reason = status.reason || 'measurement_unavailable';
        const definitions = [
            ['dimensions.vehicle.length', '整车可见长度', vehicle.length],
            ['dimensions.vehicle.width', '整车可见宽度', vehicle.width],
            ['dimensions.vehicle.height', '整车可见高度跨度（非离地高度）', vehicle.height],
            ['cabToTrailerFrontDist', '车头到车厢前内壁', review && review.cabToTrailerFrontDist],
            ['dimensions.carriage.length', '车厢内长', carriage.length],
            ['dimensions.carriage.width', '车厢内宽', widthValue],
            ['dimensions.carriage.height', '车厢内高（栏板深度）', height],
            ['trailerBottomToGroundHeight', '底板距地磅', review && review.trailerBottomToGroundHeight],
            ['highLowPlateHeight', '高低板落差', review && review.highLowPlateHeight],
            ['ribCount', '拉筋数量', ribCount, '根'],
            ['wireCount', '钢丝数量', review && review.wireCount, '根']
        ];
        return {
            pending: true, partial: true, widthLabel: '车厢内宽',
            cabToFront: formatMeters(review && review.cabToTrailerFrontDist),
            innerLength: formatMeters(carriage.length), width: formatMeters(widthValue),
            innerHeight: formatMeters(height),
            floorHeight: formatMeters(review && review.trailerBottomToGroundHeight),
            ribCount, ribCountText: describeRibCount(ribCount),
            ribGaps: gaps.map(value => formatRibGap(value, formatMeters)),
            plates: (review && Array.isArray(review.highLowPlates) ? review.highLowPlates : []).map((plate, index) => ({
                id: plate && plate.plateId != null ? plate.plateId : index + 1,
                high: formatMeters(plate && plate.highestToGround),
                low: formatMeters(plate && plate.lowestToGround)
            })),
            plateDrop: formatMeters(review && review.highLowPlateHeight),
            fieldRows: definitions.map(([key, label, value, unit]) => {
                const field = fields[key] || fields[key.replace('dimensions.', '')] || {};
                const available = typeof value === 'number' && Number.isFinite(value);
                const explanation = field.reason || (available
                    ? 'current_capture_measurement' : reason);
                return { label, value: available ? (unit ? value + ' ' + unit : formatMeters(value)) : '缺测 / 未执行',
                    explanation: measurementReason(explanation), source: field.source || '' };
            }),
            reference: review && review.referenceDimensions && review.referenceDimensions.carriage
        };
    }

    function measurementReason(reason) {
        const descriptions = {
            inner_wall_boundaries_unverified: '前后内壁未确认，不能确定相应测量端点',
            longitudinal_inner_edges_unavailable: '侧壁下部端点缺测，内长尚未量取',
            lower_inner_edges_unavailable: '侧壁下部端点缺测',
            cab_front_at_measurement_height_unavailable: '1.40 米测量截面未找到有效车头前沿',
            vehicle_not_empty: '本次判为载货，内部结构测量未执行',
            load_status_uncertain: '载荷状态不确定，内部结构尚未确认',
            current_capture_measurement: '本次点云计算值，待人工核对',
            current_capture_not_confirmed_empty: '本次未确认空车，内部结构值不可量取',
            inner_width_unavailable: '未取得有效的双侧内壁测量支持',
            empty_floor_or_wall_height_unavailable: '空车底板或壁顶支持不足，内高未确认',
            front_inner_wall_unavailable: '前内壁未确认，车头距缺测',
            floor_profile_unavailable: '本次未取得有效底板剖面',
            rib_detection_not_performed: '本次未执行拉筋检测',
            wire_detector_not_implemented: '本系统尚未实现钢丝检测',
            vehicle_extent_unavailable: '本次车辆可见包络未取得',
            measurement_unavailable: '本次未获得该项测量结果',
            detector_not_implemented: '本系统尚未实现该项检测',
            not_measured: '本次未执行该项测量'
        };
        return descriptions[reason] || String(reason);
    }

    function partialResultDetails(data) {
        const rows = data.fieldRows.map(field => '<tr><td>' + escapeHtml(field.label)
            + '</td><td>' + escapeHtml(field.value) + '</td><td>' + escapeHtml(field.explanation)
            + (field.source ? '（' + escapeHtml(measurementSource(field.source)) + '）' : '') + '</td></tr>').join('');
        const ref = data.reference;
        const reference = ref && (Number.isFinite(ref.length) || Number.isFinite(ref.width))
            ? '<div class="result-note">参考候选跨度：长 ' + escapeHtml(formatMeters(ref.length))
                + '，外沿宽 ' + escapeHtml(formatMeters(ref.width)) + '；不是已确认的净内长或净内宽。</div>' : '';
        return '<div class="result-note">部分计算结果，尚未入库；缺测项目不以 0 或其它尺寸代替。</div>'
            + '<table class="plate-table"><thead><tr><th>项目</th><th>结果</th><th>来源 / 缺测原因</th></tr></thead>'
            + '<tbody>' + rows + '</tbody></table>' + reference;
    }

    function measurementSource(source) {
        return ({
            observed_vehicle_extent: '当前实测车辆点云范围',
            observed_vertical_span_not_ground_height: '当前实测垂直跨度，不是离地高度',
            observed_inner_side_faces: '实测左右内壁面',
            current_empty_carriage_wall_depth: '本次空车底板到壁顶',
            observed_ground_height_slice: '离地 1.40 米截面',
            observed_floor_profile: '实测底板剖面',
            current_observed_rib_detection: '本次实测拉筋检测'
        })[source] || String(source);
    }

    /** 现场车厢拉筋为 2~6 根；超出范围时标出来，让操作员知道该核对而不是照抄。 */
    function describeRibCount(count) {
        if (count == null) return '未确认';
        return count >= 2 && count <= 6 ? String(count) + ' 根' : String(count) + ' 根 ⚠';
    }

    function resultCard(label, value, pending) {
        return '<div class="dim-card' + (pending ? ' pending' : '') + '">'
            + '<div class="dim-label">' + escapeHtml(label) + '</div>'
            + '<div class="dim-value">' + escapeHtml(String(value)) + '</div></div>';
    }

    /** 只说明当次分析明确携带的测线，不为仅剩数据库字段的旧记录补写新口径。 */
    function lengthMeasurementNote(progress) {
        const analysis = progress.analysis;
        const measurement = analysis && analysis.stage === 'completed'
            && analysis.measurementStatus && analysis.measurementStatus.lengthMeasurement;
        if (!measurement || measurement.valid !== true
            || measurement.definition !== 'same_side_lower_band_longitudinal_projection') return '';
        const height = measurementRangeCm(measurement.heightAboveFloorM);
        const inward = measurementRangeCm(measurement.inwardBandM);
        if (!height || !inward) return '';
        const note = '内长测线：同侧下部，底板以上 ' + height
            + ' cm，侧壁参考面向内 ' + inward + ' cm（纵向投影）。';
        return '<div class="result-note">' + escapeHtml(note) + '</div>';
    }

    function measurementRangeCm(values) {
        if (!Array.isArray(values) || values.length !== 2
            || !values.every(value => typeof value === 'number' && Number.isFinite(value))
            || values[0] < 0 || values[1] <= values[0] || values[1] > 10) return '';
        return values.map(value => String(Math.round(value * 1000) / 10)).join('～');
    }

    /** null 是未能量取的间距，不能经 Number(null) 变成零。 */
    function formatRibGap(value, formatter) {
        return typeof value === 'number' && Number.isFinite(value) && value >= 0
            ? formatter(value) : '缺测';
    }

    /**
     * 拉筋间距。
     *
     * 顺序与 Analyzer 的输出一致：从**前壁**起算，首段是前壁到第一根，
     * 末段是最后一根到后壁。方向写反会让现场按错误的次序对尺。
     */
    function ribSection(data) {
        if (!data.ribGaps.length) return '';
        const total = data.ribGaps.length;
        const chips = data.ribGaps.map((gap, index) => {
            const from = index === 0 ? '前壁' : '筋' + index;
            const to = index === total - 1 ? '后壁' : '筋' + (index + 1);
            return '<span class="rib-seg"><i>' + escapeHtml(from + '→' + to)
                + '</i>' + escapeHtml(String(gap)) + '</span>';
        }).join('');
        const missing = data.ribGaps.includes('缺测')
            ? '<div class="result-note">部分拉筋间距缺测，不以 0 代替；已计算的内长和其余间距分别保留。</div>'
            : '';
        return '<h4>拉筋间距</h4>' + missing + '<div class="rib-strip">' + chips + '</div>';
    }

    /** 高低板最多 3 组；多于 3 组同样标出来提示核对。 */
    function plateSection(data) {
        if (!data.plates.length) {
            return '<h4>高低板</h4><div class="result-note">'
                + (data.partial || data.plateMeasureUnavailable
                    ? '未取得有效明细，不能据此断言没有高低板' : '未检出') + '</div>';
        }
        const warn = data.plates.length > 3
            ? '<span class="plate-warn">检出 ' + data.plates.length + ' 组，超出常见的 3 组 ⚠</span>'
            : '';
        const rows = data.plates.map(plate =>
            '<tr><td>' + escapeHtml('第 ' + plate.id + ' 组') + '</td><td>'
            + escapeHtml(String(plate.high)) + '</td><td>'
            + escapeHtml(String(plate.low)) + '</td></tr>').join('');
        return '<h4>高低板' + warn + '</h4>'
            + '<table class="plate-table"><thead><tr><th></th><th>最高点离地</th>'
            + '<th>最低点离地</th></tr></thead><tbody>' + rows + '</tbody></table>'
            + '<div class="result-note">落差 ' + escapeHtml(String(data.plateDrop)) + '</div>';
    }

    // ------------------------------------------------------------------ 工具

    function row(label, valueHtml) {
        return '<div class="status-row"><span class="status-label">' + escapeHtml(label)
            + '</span><span class="status-value">' + valueHtml + '</span></div>';
    }

    function badge(kind, text) {
        return '<span class="dot dot-' + kind + '"></span>' + text;
    }

    function formatNumber(value) {
        return Number(value || 0).toLocaleString('zh-CN');
    }

    function formatCm(value) {
        return value == null ? '—' : Number(value).toFixed(1) + ' cm';
    }

    function formatMeters(value) {
        return typeof value === 'number' && Number.isFinite(value) ? value.toFixed(3) + ' m' : '未算出';
    }

    function formatTime(iso) {
        if (!iso) return '—';
        const date = new Date(iso);
        return isNaN(date.getTime()) ? String(iso) : date.toLocaleTimeString('zh-CN', { hour12: false });
    }

    /**
     * 展开"确认未采集，重新发起"入口。
     * 只在服务端明确表示无法核实时出现——这一步是让人替系统做它证明不了的判断，
     * 不能自动点，也不能默认展开。
     */
    // -------------------------------------------------------------- 模态框

    function openCaptureModal() {
        dom.captureModal.style.display = 'flex';
        dom.btnReopenProgress.style.display = 'none';
    }

    /**
     * 关闭模态框。
     * 采集仍在进行时只是转入后台：轮询不停、幂等键不清，
     * 否则操作员一关窗口就失去了这次流程的去向，而它已经在写业务数据了。
     */
    function closeCaptureModal() {
        dom.captureModal.style.display = 'none';
        updateReopenButton();
    }

    function updateReopenButton() {
        dom.btnReopenProgress.style.display =
            (captureActive || currentWorkflowId || captureIdempotencyKey)
                && dom.captureModal.style.display === 'none' ? 'block' : 'none';
    }

    /** 流程编号与车牌常驻标题栏：出问题时这两项是唯一能对上号的凭据。 */
    function setModalMeta(workflowId, plate) {
        dom.btnRefreshResult.disabled = !workflowId || progressBusy;
        const parts = [];
        if (plate) parts.push('车牌 ' + plate);
        parts.push(workflowId ? '流程 ' + workflowId : '流程编号待分配');
        dom.captureModalMeta.textContent = parts.join(' · ');
    }

    function showConfirmAction() {
        dom.confirmAction.style.display = 'block';
        dom.btnCancelCapture.disabled = true;
    }

    function hideConfirmAction() {
        dom.confirmAction.style.display = 'none';
    }

    function showBanner(element, kind, message) {
        element.className = 'banner ' + kind;
        element.textContent = message;
    }

    function hideBanner(element) {
        element.className = 'banner';
        element.textContent = '';
    }

    function escapeHtml(value) {
        return String(value == null ? '' : value)
            .replace(/&/g, '&amp;')
            .replace(/</g, '&lt;')
            .replace(/>/g, '&gt;')
            .replace(/"/g, '&quot;');
    }
})();
