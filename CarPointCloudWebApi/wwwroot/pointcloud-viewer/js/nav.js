/**
 * 全局导航脚本
 * 职责：
 * 1. 按当前登录用户的角色动态显示/隐藏菜单项（菜单项声明 data-role 属性，如 data-role="admin"）
 * 2. 退出登录
 *
 * 使用约定：在导航链接上添加 data-role 属性声明所需角色，多个角色用逗号分隔，
 * 例如：<a href="users.html" data-role="admin">用户管理</a>
 * 未声明 data-role 的菜单项对所有已登录用户可见。
 */
(function () {
    'use strict';

    document.addEventListener('DOMContentLoaded', () => {
        applyRoleBasedMenu();
        const user = typeof API !== 'undefined' ? API.getCurrentUser() : null;
        const name = document.getElementById('navUser');
        const role = document.getElementById('navRole');
        if (name) name.textContent = user && (user.displayName || user.username) || '';
        if (role) role.textContent = user && user.roleCode === 'admin' ? '管理员' : '当前用户';

        const logoutBtn = document.getElementById('navLogout');
        if (logoutBtn) {
            logoutBtn.addEventListener('click', (e) => {
                e.preventDefault();
                try {
                    // 通知后端登出（尽力而为，失败不影响本地清理）
                    if (typeof API !== 'undefined') {
                        API.request('/api/auth/logout', { method: 'POST' }).catch(() => {});
                    }
                } catch (err) { /* ignore */ }
                if (typeof API !== 'undefined') {
                    API.clearAuth();
                } else {
                    sessionStorage.removeItem('car_pointcloud_auth');
                }
                window.location.href = 'login.html';
            });
        }
    });

    /**
     * 根据当前用户角色过滤导航菜单项。
     * 未登录时：隐藏所有声明了 data-role 的菜单项（未登录无法确认权限）。
     * 已登录时：仅显示角色匹配（或未声明 data-role）的菜单项。
     */
    function applyRoleBasedMenu() {
        const items = document.querySelectorAll('[data-role]');
        if (items.length === 0) return;

        // 获取当前用户角色：优先用本地缓存的 user.roleCode
        let roleCode = null;
        if (typeof API !== 'undefined') {
            const user = API.getCurrentUser();
            roleCode = (user && user.roleCode) ? user.roleCode : null;
        }

        items.forEach(item => {
            const requiredRoles = (item.getAttribute('data-role') || '')
                .split(',')
                .map(r => r.trim())
                .filter(r => r.length > 0);

            const hasAccess = roleCode !== null && requiredRoles.some(r =>
                r === '*' || r.toLowerCase() === roleCode.toLowerCase());

            if (!hasAccess) {
                item.style.display = 'none';
            }
        });
    }
})();
