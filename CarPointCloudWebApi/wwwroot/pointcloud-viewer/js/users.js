/**
 * 用户管理页面脚本
 * 功能：用户列表（分页/搜索）、新增、编辑（角色/状态/重置密码）、删除、退出登录
 */

(function () {
    'use strict';

    const state = {
        page: 1,
        pageSize: 10,
        total: 0,
        users: [],
        roles: [],
        keyword: ''
    };

    // ============================
    // 初始化
    // ============================
    document.addEventListener('DOMContentLoaded', init);

    async function init() {
        // 校验登录态（token 存在性）
        const cached = API.getAuth();
        if (!cached || !cached.token) {
            window.location.href = 'login.html';
            return;
        }

        // 从后端获取权威用户信息（含角色），避免本地缓存缺少 roleCode 导致误判
        try {
            const res = await API.get('/api/auth/me');
            const me = res.data;
            if (!me) {
                throw new Error('无法获取当前用户信息');
            }

            // 同步最新用户信息到本地缓存（供导航等其他页面使用）
            API.setAuth(me.username, cached.token, me);

            if (me.roleCode !== 'admin') {
                showToast('当前账号无管理权限（仅 admin 角色可访问用户管理）', 'error');
                setTimeout(() => { window.location.href = 'index.html'; }, 1500);
                return;
            }
        } catch (e) {
            // 401 已由 api-client 统一处理跳转登录页；其他错误提示后返回
            if (e && e.code !== 401) {
                showToast('权限校验失败：' + e.message, 'error');
                setTimeout(() => { window.location.href = 'index.html'; }, 1500);
            }
            return;
        }

        bindEvents();
        await loadRoles();
        await loadUsers();
    }

    // ============================
    // 数据加载
    // ============================
    async function loadRoles() {
        try {
            const res = await API.get('/api/roles');
            state.roles = res.data || [];
        } catch (e) {
            console.error('加载角色失败:', e);
        }
    }

    async function loadUsers() {
        const tbody = document.getElementById('usersTableBody');
        document.getElementById('usersEmpty').style.display = 'none';
        document.getElementById('pageInfo').textContent = '正在加载用户…';
        document.getElementById('pageBtns').innerHTML = '';
        tbody.innerHTML = '<tr><td colspan="7" style="text-align:center;padding:40px;color:var(--text-secondary);">加载中...</td></tr>';

        try {
            const query = new URLSearchParams({ page: String(state.page), pageSize: String(state.pageSize) });
            if (state.keyword) query.set('keyword', state.keyword);
            const res = await API.get('/api/users?' + query.toString());
            state.users = res.data.items || [];
            state.total = res.data.total || 0;
            renderTable();
            renderPagination();
        } catch (e) {
            tbody.innerHTML = `<tr><td colspan="7" style="text-align:center;padding:40px;color:var(--accent-danger);">加载失败：${escapeHtml(e.message || '请重试')}</td></tr>`;
            document.getElementById('pageInfo').textContent = '加载未完成，记录数量未知';
            document.getElementById('pageBtns').innerHTML = '';
        }
    }

    // ============================
    // 渲染
    // ============================
    function renderTable() {
        const tbody = document.getElementById('usersTableBody');
        const emptyEl = document.getElementById('usersEmpty');

        if (state.users.length === 0) {
            tbody.innerHTML = '';
            emptyEl.style.display = 'flex';
            return;
        }
        emptyEl.style.display = 'none';

        tbody.innerHTML = state.users.map(user => {
            const isAdmin = user.roleCode === 'admin';
            const roleName = user.roleName || (isAdmin ? '系统管理员' : '查看者');
            const roleClass = isAdmin ? 'admin' : 'viewer';
            const statusOn = user.status === 1;
            const lastLogin = user.lastLoginAt ? new Date(user.lastLoginAt).toLocaleString('zh-CN', { hour12: false }) : '-';

            return `
                <tr>
                    <td>${user.id}</td>
                    <td><strong>${escapeHtml(user.username)}</strong></td>
                    <td>${escapeHtml(user.displayName || '-')}</td>
                    <td><span class="role-badge ${roleClass}">${escapeHtml(roleName)}</span></td>
                    <td><span class="status-badge ${statusOn ? 'on' : 'off'}">${statusOn ? '启用' : '禁用'}</span></td>
                    <td class="user-last-login">${lastLogin}</td>
                    <td>
                        <div class="row-actions">
                            <button data-action="edit" data-id="${user.id}">编辑</button>
                            <button data-action="reset" data-id="${user.id}" title="重置密码">重置密码</button>
                            <button data-action="delete" data-id="${user.id}" class="danger">删除</button>
                        </div>
                    </td>
                </tr>
            `;
        }).join('');
    }

    function renderPagination() {
        const totalPages = Math.max(1, Math.ceil(state.total / state.pageSize));
        document.getElementById('pageInfo').textContent =
            `共 ${state.total} 条 · 第 ${state.page}/${totalPages} 页`;

        const container = document.getElementById('pageBtns');
        let html = `<button class="page-btn" data-page="${state.page - 1}" ${state.page === 1 ? 'disabled' : ''}>上一页</button>`;

        for (let i = 1; i <= totalPages; i++) {
            if (totalPages > 7 && i > 2 && i < totalPages - 1 && Math.abs(i - state.page) > 1) {
                if (i === 3) html += '<span class="page-ellipsis">…</span>';
                continue;
            }
            html += `<button class="page-btn ${i === state.page ? 'active' : ''}" data-page="${i}">${i}</button>`;
        }

        html += `<button class="page-btn" data-page="${state.page + 1}" ${state.page === totalPages ? 'disabled' : ''}>下一页</button>`;
        container.innerHTML = html;
    }

    // ============================
    // 弹窗
    // ============================
    function openCreateModal() {
        document.getElementById('modalTitle').textContent = '新增用户';
        document.getElementById('userId').value = '';
        document.getElementById('username').value = '';
        document.getElementById('username').disabled = false;
        document.getElementById('passwordLabel').textContent = '初始密码';
        document.getElementById('password').value = '';
        document.getElementById('password').placeholder = '12–128 位';
        document.getElementById('displayName').value = '';
        document.getElementById('roleCode').value = 'viewer';
        document.getElementById('status').value = '1';
        document.getElementById('passwordGroup').style.display = 'block';
        clearErrors();
        showModal(true);
        document.getElementById('username').focus();
    }

    function openEditModal(userId) {
        const user = state.users.find(u => u.id === Number(userId));
        if (!user) return;

        document.getElementById('modalTitle').textContent = '编辑用户';
        document.getElementById('userId').value = user.id;
        document.getElementById('username').value = user.username;
        document.getElementById('username').disabled = true; // 用户名不可修改
        document.getElementById('passwordLabel').textContent = '新密码（选填）';
        document.getElementById('password').value = '';
        document.getElementById('password').placeholder = '留空则不修改密码';
        document.getElementById('displayName').value = user.displayName || '';
        document.getElementById('roleCode').value = user.roleCode;
        document.getElementById('status').value = String(user.status);
        clearErrors();
        showModal(true);
        document.getElementById('displayName').focus();
    }

    let modalTrigger = null;

    function showModal(show) {
        const modal = document.getElementById('userModal');
        if (show) modalTrigger = document.activeElement;
        modal.classList.toggle('show', show);
        modal.setAttribute('aria-hidden', String(!show));
        if (!show) modalTrigger?.focus();
    }

    function clearErrors() {
        document.getElementById('usernameError').textContent = '';
        document.getElementById('passwordError').textContent = '';
        document.getElementById('username').classList.remove('error');
        document.getElementById('password').classList.remove('error');
    }

    // ============================
    // 事件
    // ============================
    function bindEvents() {
        // 新增按钮
        document.getElementById('addUserBtn').addEventListener('click', openCreateModal);

        // 取消
        document.getElementById('modalCancel').addEventListener('click', () => showModal(false));
        document.getElementById('userModal').addEventListener('click', (e) => {
            if (e.target === e.currentTarget) showModal(false);
        });
        document.addEventListener('keydown', (e) => {
            const modal = document.getElementById('userModal');
            if (!modal.classList.contains('show')) return;
            if (e.key === 'Escape') {
                e.preventDefault();
                showModal(false);
            } else if (e.key === 'Tab') {
                const fields = modal.querySelectorAll('input:not([type="hidden"]):not(:disabled), select, button');
                const first = fields[0];
                const last = fields[fields.length - 1];
                if (e.shiftKey && document.activeElement === first) {
                    e.preventDefault();
                    last.focus();
                } else if (!e.shiftKey && document.activeElement === last) {
                    e.preventDefault();
                    first.focus();
                }
            }
        });

        // 搜索
        let searchTimer;
        document.getElementById('searchInput').addEventListener('input', (e) => {
            clearTimeout(searchTimer);
            searchTimer = setTimeout(() => {
                state.keyword = e.target.value.trim();
                state.page = 1;
                loadUsers();
            }, 400);
        });

        // 分页
        document.getElementById('pageBtns').addEventListener('click', (e) => {
            const btn = e.target.closest('.page-btn');
            if (!btn || btn.disabled) return;
            const page = parseInt(btn.dataset.page, 10);
            if (page >= 1) {
                state.page = page;
                loadUsers();
            }
        });

        // 行操作（事件委托）
        document.getElementById('usersTableBody').addEventListener('click', async (e) => {
            const btn = e.target.closest('button[data-action]');
            if (!btn) return;
            const { action, id } = btn.dataset;
            if (action === 'edit') openEditModal(id);
            if (action === 'delete') await handleDelete(id);
            if (action === 'reset') await handleReset(id);
        });

        // 表单提交
        document.getElementById('userForm').addEventListener('submit', async (e) => {
            e.preventDefault();
            clearErrors();
            const userId = document.getElementById('userId').value;

            if (!userId) {
                await handleCreate();
            } else {
                await handleUpdate(userId);
            }
        });
        // 退出登录统一由 nav.js 处理
    }

    // ============================
    // 操作
    // ============================
    async function handleCreate() {
        const username = document.getElementById('username').value.trim();
        const password = document.getElementById('password').value;
        const displayName = document.getElementById('displayName').value.trim();
        const roleCode = document.getElementById('roleCode').value;
        const status = parseInt(document.getElementById('status').value, 10);

        if (!username) {
            document.getElementById('usernameError').textContent = '请输入用户名';
            return;
        }
        if (password.length < 12 || password.length > 128) {
            document.getElementById('passwordError').textContent = '密码须为 12–128 位';
            return;
        }

        try {
            await API.post('/api/users', { username, password, displayName, roleCode, status });
            showToast('用户创建成功', 'success');
            showModal(false);
            await loadUsers();
        } catch (e) {
            document.getElementById('usernameError').textContent = e.message;
        }
    }

    async function handleUpdate(userId) {
        const displayName = document.getElementById('displayName').value.trim();
        const roleCode = document.getElementById('roleCode').value;
        const status = parseInt(document.getElementById('status').value, 10);
        const newPassword = document.getElementById('password').value;

        const body = { displayName, roleCode, status };
        if (newPassword) {
            if (newPassword.length < 12 || newPassword.length > 128) {
                document.getElementById('passwordError').textContent = '密码须为 12–128 位';
                return;
            }
            body.newPassword = newPassword;
        }

        try {
            await API.put(`/api/users/${userId}`, body);
            showToast('用户更新成功', 'success');
            showModal(false);
            await loadUsers();
        } catch (e) {
            showToast(e.message, 'error');
        }
    }

    async function handleDelete(userId) {
        const user = state.users.find(u => u.id === Number(userId));
        if (!user) return;
        if (!confirm(`确定删除用户「${user.username}」？此操作不可恢复。`)) return;

        try {
            await API.del(`/api/users/${userId}`);
            showToast('用户已删除', 'success');
            await loadUsers();
        } catch (e) {
            showToast(e.message, 'error');
        }
    }

    async function handleReset(userId) {
        const user = state.users.find(u => u.id === Number(userId));
        const newPassword = prompt(`为「${user.username}」设置新密码（12–128 位）：`);
        if (!newPassword) return;
        if (newPassword.length < 12 || newPassword.length > 128) {
            showToast('密码须为 12–128 位', 'error');
            return;
        }

        try {
            await API.put(`/api/users/${userId}`, { newPassword });
            showToast('密码已重置', 'success');
        } catch (e) {
            showToast(e.message, 'error');
        }
    }

    // ============================
    // 工具
    // ============================
    function showToast(message, type = 'info') {
        const toast = document.createElement('div');
        toast.className = 'error-toast';
        toast.style.background = type === 'error'
            ? 'linear-gradient(135deg, var(--accent-danger), #dc2626)'
            : 'linear-gradient(135deg, var(--accent-success), #059669)';
        toast.textContent = message;
        document.body.appendChild(toast);
        setTimeout(() => {
            toast.style.opacity = '0';
            toast.style.transition = 'opacity 0.3s ease';
            setTimeout(() => toast.remove(), 300);
        }, 2500);
    }

    function escapeHtml(text) {
        const div = document.createElement('div');
        div.textContent = text;
        return div.innerHTML;
    }
})();
