/**
 * 主入口文件
 * 初始化应用并绑定所有事件
 */

// 全局变量
let viewer = null;
let controller = null;

// DOM加载完成后初始化
document.addEventListener('DOMContentLoaded', () => {
    initApp();
});

/**
 * 初始化应用程序
 */
function initApp() {
    try {
        // 1. 创建3D查看器
        viewer = new PointCloudViewer('canvas-container');
        
        // 2. 创建交互控制器
        controller = new InteractionController(viewer);
        
        // 3. 绑定车牌查询事件
        bindPlateSearch();
        
        // 4. 禁止右键默认菜单（画布区域右键用于点云操作）
        document.getElementById('canvas-wrapper').addEventListener('contextmenu', (e) => {
            e.preventDefault();
        });

        // 5. 在document级别（capture阶段）禁止右键菜单，确保不遗漏
        document.addEventListener('contextmenu', (e) => {
            e.preventDefault();
            e.stopPropagation();
            return false;
        }, true);
        
        updateStatus('应用初始化完成，请输入车牌号查询最新空车云图');
        
    } catch (error) {
        console.error('应用初始化失败:', error);
        showError(`应用初始化失败: ${error.message}`);
    }
}

/**
 * 绑定车牌查询事件
 */
function bindPlateSearch() {
    const searchButton = document.getElementById('searchPlateBtn');
    const plateInput = document.getElementById('plateInput');

    const search = async () => {
        if (searchButton.disabled) return;
        const plate = plateInput.value.trim().toUpperCase();
        if (!plate) {
            showError('请输入车牌号');
            plateInput.focus();
            return;
        }

        searchButton.disabled = true;
        setArchiveState('loading');
        showLoading(`正在查询 ${plate} 的最新空车档案...`);
        try {
            const response = await API.get(
                `/api/empty-vehicle-profiles/${encodeURIComponent(plate)}`
            );
            const profile = response.data;
            showLoading('正在读取融合 PCD 云图...');
            const fileContent = await API.arrayBuffer(profile.pointCloudUrl);
            showLoading('正在解析PCD文件...');
            const parser = new PCDParser();
            const pcdData = parser.parse(fileContent);
            showLoading('正在渲染点云...');
            viewer.loadPointCloud(pcdData);
            renderEmptyVehicleProfile(profile);
            setArchiveState('loaded');
            hideLoading();
            updateStatus(
                `${profile.licensePlate} 最新车辆云图加载成功（${profile.pointCount.toLocaleString()} 点）` +
                (profile.reviewStatus === 'pending' ? '：已保存，待人工审核，含模型推断点' : '')
            );
        } catch (error) {
            console.error('空车云图查询失败:', error);
            hideLoading();
            setArchiveState('error', error.message || '空车云图查询失败');
            showError(error.message || '空车云图查询失败');
        } finally {
            searchButton.disabled = false;
        }
    };

    searchButton.addEventListener('click', search);
    plateInput.addEventListener('keydown', (event) => {
        if (event.key === 'Enter') {
            event.preventDefault();
            search();
        }
    });
}

function setArchiveState(state, message) {
    const loaded = state === 'loaded';
    document.getElementById('canvas-wrapper').dataset.state = state;
    document.getElementById('archiveCanvasNotice').hidden = loaded;
    document.getElementById('archiveProfileEmpty').hidden = loaded;
    document.getElementById('resetBtn').disabled = !loaded;
    document.getElementById('screenshotBtn').disabled = !loaded;
    document.getElementById('archiveCloudLabel').textContent = loaded
        ? '已保存云图 · 非实时' : '空车云图 · ' + (state === 'loading' ? '正在查询' : '尚未加载');
    if (loaded) return;

    document.getElementById('empty-vehicle-panel').style.display = 'none';
    document.getElementById('bbox-panel').style.display = 'none';
    document.getElementById('pointInfo').textContent = '未加载数据';
    updatePointCount(0);
    document.getElementById('archiveNoticeTitle').textContent = state === 'loading'
        ? '正在读取空车档案' : '未能加载空车档案';
    document.getElementById('archiveNoticeMessage').textContent = message || '正在查询该车最新保存的云图与尺寸，请稍候。';
    document.getElementById('archiveProfileEmpty').textContent = state === 'loading'
        ? '正在查询车辆尺寸及扫描时间…' : '本次查询未成功，请核对车牌后重试。';
}

function renderEmptyVehicleProfile(profile) {
    const dimensions = profile.dimensions || {};
    const setDimension = (id, value) => {
        const number = Number(value);
        document.getElementById(id).textContent =
            value === null || value === undefined || value === '' || !Number.isFinite(number)
                ? '--' : formatNumber(number, 1);
    };

    document.getElementById('profile-plate').textContent = profile.licensePlate || '--';
    document.getElementById('profile-time').textContent = profile.scanTime
        ? new Date(profile.scanTime).toLocaleString('zh-CN', { hour12: false })
        : '--';
    const pending = profile.reviewStatus === 'pending';
    const quality = document.getElementById('profile-quality');
    quality.textContent = pending
        ? '已保存，待人工审核' : '已保存';
    quality.classList.toggle('is-pending', pending);
    setDimension('dimension-length', dimensions.trailerLength);
    setDimension('dimension-width', dimensions.trailerWidth);
    setDimension('dimension-height', dimensions.trailerBodyHeight);
    setDimension('dimension-bottom', dimensions.trailerBottomToGroundHeight);
    setDimension('dimension-cab', dimensions.cabToTrailerFrontDist);
    setDimension('dimension-high-low', dimensions.highLowPlateHeight);
    document.getElementById('empty-vehicle-panel').style.display = 'block';
}

/**
 * 键盘快捷键支持
 */
document.addEventListener('keydown', (e) => {
    if (!viewer || document.getElementById('screenshotBtn').disabled ||
        e.target.closest('input, select, textarea, [contenteditable]')) return;
    
    switch (e.key.toLowerCase()) {
        case 'r':
            // R键重置视角
            viewer.resetView();
            break;
        case 's':
            // S键截图
            viewer.takeScreenshot();
            break;
    }
});

// 导出全局函数（供调试使用）
window.PointCloudApp = {
    viewer,
    controller
};
