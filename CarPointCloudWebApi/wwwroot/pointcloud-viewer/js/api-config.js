/**
 * 后端 API 地址自动配置（必须在其他脚本之前加载）
 *
 * 背景：系统后端运行在 http://<host>:5031。若页面通过其他方式打开
 * （如预览面板 http://127.0.0.1:6478、或直接双击 HTML），相对路径
 * /api/xxx 会请求到错误服务器（返回 HTML 而非 JSON），导致登录/数据加载失败。
 *
 * 解决：自动检测当前页面是否由后端 5031 提供服务；若不是，
 * 自动将 window.__API_BASE__ 指向本机后端 5031（http://<hostname>:5031）。
 * 也支持手动覆盖：在引入本脚本前设置 window.__API_BASE__ 即可。
 */
(function () {
    'use strict';

    // 已手动配置则不覆盖
    if (typeof window.__API_BASE__ === 'string' && window.__API_BASE__.trim()) {
        return;
    }

    var port = window.location.port;
    var isDefaultPort = port === '' || port === '80' || port === '443';
    var hostname = window.location.hostname || 'localhost';

    // 后端服务端口（与 appsettings.json Kestrel 配置一致）
    var BACKEND_PORT = 5031;

    // 页面由后端提供（端口 = 后端端口，或标准端口经反向代理转发）→ 使用相对路径
    if (port === String(BACKEND_PORT) || isDefaultPort) {
        return;
    }

    // 其他端口（预览面板/静态服务器/开发服务器）→ 指向本机后端
    // 仅对 localhost / 127.0.0.1 生效，避免误伤内网部署
    if (hostname === 'localhost' || hostname === '127.0.0.1' || hostname === '[::1]') {
        window.__API_BASE__ = window.location.protocol + '//' + hostname + ':' + BACKEND_PORT;
        console.info('[API] 当前页面非后端端口，API 已指向: ' + window.__API_BASE__);
    }
})();
