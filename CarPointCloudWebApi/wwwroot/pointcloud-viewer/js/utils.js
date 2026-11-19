/**
 * 工具函数集合
 */

// 显示加载提示
function showLoading(message = '正在加载...') {
    const indicator = document.getElementById('loadingIndicator');
    if (indicator) {
        indicator.textContent = message;
        indicator.style.display = 'inline';
    }
}

// 隐藏加载提示
function hideLoading() {
    const indicator = document.getElementById('loadingIndicator');
    if (indicator) {
        indicator.style.display = 'none';
    }
}

// 更新状态栏消息
function updateStatus(message) {
    const statusMessage = document.getElementById('statusMessage');
    if (statusMessage) {
        statusMessage.textContent = message;
    }
}

// 更新点数显示
function updatePointCount(count) {
    const statusPoints = document.getElementById('statusPoints');
    if (statusPoints) {
        statusPoints.textContent = `点数: ${count.toLocaleString()}`;
    }
}

// 更新FPS显示
function updateFPS(fps) {
    const statusFPS = document.getElementById('statusFPS');
    if (statusFPS) {
        statusFPS.textContent = `FPS: ${fps}`;
    }
}

// 更新距离显示
function updateDistance(distance) {
    const statusDistance = document.getElementById('statusDistance');
    if (statusDistance) {
        statusDistance.textContent = `距离: ${distance.toFixed(2)}m`;
    }
}

// 显示错误消息
function showError(message) {
    alert(`错误: ${message}`);
    updateStatus(`错误: ${message}`);
}

// 格式化数字（保留指定位数小数）
function formatNumber(num, decimals = 2) {
    return num.toFixed(decimals);
}

// 计算两点之间的距离
function calculateDistance(point1, point2) {
    const dx = point2.x - point1.x;
    const dy = point2.y - point1.y;
    const dz = point2.z - point1.z;
    return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

// 生成唯一ID
function generateId() {
    return Date.now().toString(36) + Math.random().toString(36).substr(2);
}

// 防抖函数
function debounce(func, wait) {
    let timeout;
    return function executedFunction(...args) {
        const later = () => {
            clearTimeout(timeout);
            func(...args);
        };
        clearTimeout(timeout);
        timeout = setTimeout(later, wait);
    };
}

// 节流函数
function throttle(func, limit) {
    let inThrottle;
    return function(...args) {
        if (!inThrottle) {
            func.apply(this, args);
            inThrottle = true;
            setTimeout(() => inThrottle = false, limit);
        }
    };
}

// 颜色插值（蓝->绿->黄->红）
function getColorFromHeight(t) {
    // t 范围 0-1
    t = Math.max(0, Math.min(1, t));
    
    let r, g, b;
    
    if (t < 0.25) {
        // 蓝到绿
        r = 0;
        g = t * 4;
        b = 1 - t * 4;
    } else if (t < 0.5) {
        // 绿到黄
        r = (t - 0.25) * 4;
        g = 1;
        b = 0;
    } else if (t < 0.75) {
        // 黄到橙
        r = 1;
        g = 1 - (t - 0.5) * 4;
        b = 0;
    } else {
        // 橙到红
        r = 1;
        g = 0;
        b = 0;
    }
    
    return [r, g, b];
}

// 将RGB值转换为0-1范围
function normalizeRGB(r, g, b) {
    return [r / 255, g / 255, b / 255];
}

// 文件大小格式化
function formatFileSize(bytes) {
    if (bytes === 0) return '0 Bytes';
    const k = 1024;
    const sizes = ['Bytes', 'KB', 'MB', 'GB'];
    const i = Math.floor(Math.log(bytes) / Math.log(k));
    return parseFloat((bytes / Math.pow(k, i)).toFixed(2)) + ' ' + sizes[i];
}

// 获取URL参数
function getUrlParameter(name) {
    const urlParams = new URLSearchParams(window.location.search);
    return urlParams.get(name);
}

// 下载文件
function downloadFile(content, filename, mimeType = 'text/plain') {
    const blob = new Blob([content], { type: mimeType });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
}

// 复制文本到剪贴板
async function copyToClipboard(text) {
    try {
        await navigator.clipboard.writeText(text);
        return true;
    } catch (err) {
        console.error('复制到剪贴板失败:', err);
        return false;
    }
}
