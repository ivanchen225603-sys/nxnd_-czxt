/**
 * 登录守卫（可选引入）
 * 用法：在需要登录的页面（index.html / logs.html）引入本脚本：
 *   <script src="js/auth-guard.js"></script>
 * 行为：sessionStorage 中不存在登录态（car_pointcloud_auth）时跳转到 login.html。
 * 说明：纯前端守卫用于提升体验，真正的安全校验由后端 JwtAuthMiddleware 负责
 *       （API 返回 401 时页面会收到认证失败提示）。
 */
(function () {
    'use strict';

    const AUTH_KEY = 'car_pointcloud_auth';
    const LOGIN_URL = 'login.html';

    function getAuth() {
        try {
            const raw = sessionStorage.getItem(AUTH_KEY);
            return raw ? JSON.parse(raw) : null;
        } catch (e) {
            return null;
        }
    }

    const auth = getAuth();
    if (!auth || !auth.token) {
        const current = window.location.pathname.split('/').pop() || 'index.html';
        // 避免登录页自身触发
        if (current !== LOGIN_URL) {
            window.location.href = LOGIN_URL;
        }
    }
})();
