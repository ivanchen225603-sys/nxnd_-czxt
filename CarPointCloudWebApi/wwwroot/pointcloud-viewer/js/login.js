/**
 * 登录页面脚本
 * 处理表单验证、密码可见性切换、登录态存储
 */

// ============================
// 配置
// ============================
const LOGIN_CONFIG = {
    // 登录接口地址。
    // 默认相对路径（同源部署，通过 http://localhost:5031 访问时自动正确）。
    // 如需跨域/子路径部署，可在页面加载前设置 window.__API_BASE__ 覆盖后端根地址，
    // 例如: window.__API_BASE__ = 'http://192.168.1.100:5031'
    get loginUrl() {
        const base = (typeof window.__API_BASE__ === 'string' && window.__API_BASE__.trim())
            ? window.__API_BASE__.replace(/\/+$/, '')
            : '';
        return base + '/api/auth/login';
    },
    // 登录成功后跳转地址
    redirectUrl: 'live.html',
    // 登录态在 sessionStorage 中的 key
    authKey: 'car_pointcloud_auth',
    // 记住我信息 key
    rememberKey: 'car_pointcloud_remember'
};

// ============================
// DOM 元素
// ============================
const form = document.getElementById('loginForm');
const usernameInput = document.getElementById('username');
const passwordInput = document.getElementById('password');
const usernameError = document.getElementById('usernameError');
const passwordError = document.getElementById('passwordError');
const togglePasswordBtn = document.getElementById('togglePassword');
const rememberMeCheckbox = document.getElementById('rememberMe');
const loginBtn = document.getElementById('loginBtn');
const forgotLink = document.getElementById('forgotLink');
const loginButtonContent = loginBtn.innerHTML;

// ============================
// 工具函数
// ============================

/**
 * 显示字段错误
 */
function showFieldError(input, errorEl, message) {
    input.classList.add('error');
    input.setAttribute('aria-invalid', 'true');
    errorEl.textContent = message;
}

/**
 * 清除字段错误
 */
function clearFieldError(input, errorEl) {
    input.classList.remove('error');
    input.removeAttribute('aria-invalid');
    errorEl.textContent = '';
}

/**
 * 清除所有错误
 */
function clearAllErrors() {
    clearFieldError(usernameInput, usernameError);
    clearFieldError(passwordInput, passwordError);
}

/**
 * 设置登录按钮加载状态
 */
function setLoading(loading) {
    loginBtn.disabled = loading;
    form.setAttribute('aria-busy', String(loading));
    if (loading) {
        loginBtn.innerHTML = `<span class="btn-icon loading-spinner" aria-hidden="true"></span> 登录中...`;
    } else {
        loginBtn.innerHTML = loginButtonContent;
    }
}

/**
 * 显示临时提示
 */
function showToast(message, type = 'info') {
    const existing = document.querySelector('.login-toast');
    if (existing) existing.remove();

    const toast = document.createElement('div');
    toast.className = type === 'error' ? 'login-toast login-toast-error' : 'login-toast';
    toast.setAttribute('role', type === 'error' ? 'alert' : 'status');
    toast.textContent = message;
    document.querySelector('.login-card').appendChild(toast);

    requestAnimationFrame(() => toast.classList.add('show'));

    setTimeout(() => {
        toast.classList.remove('show');
        setTimeout(() => toast.remove(), 300);
    }, 3000);
}

/**
 * 保存登录态
 * @param {string} username 用户名
 * @param {string} token JWT 令牌
 * @param {boolean} remember 是否记住账号
 * @param {Object|null} user 完整用户信息（含 roleCode/roleName/status 等，用于前端权限判断）
 */
function saveAuth(username, token, remember, user) {
    const auth = {
        username,
        token,
        user: user || null,
        loggedInAt: new Date().toISOString()
    };
    sessionStorage.setItem(LOGIN_CONFIG.authKey, JSON.stringify(auth));

    if (remember) {
        localStorage.setItem(LOGIN_CONFIG.rememberKey, JSON.stringify({ username }));
    } else {
        localStorage.removeItem(LOGIN_CONFIG.rememberKey);
    }
}

/**
 * 执行登录验证（调用后端 /api/auth/login）
 */
async function performLogin(username, password) {
    try {
        const response = await fetch(LOGIN_CONFIG.loginUrl, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ username, password })
        });

        let result;
        try {
            result = await response.json();
        } catch (e) {
            // 响应不是 JSON：通常是请求未到达后端（预览/静态方式打开页面，或后端未启动）
            const rawText = await response.text().catch(() => '');
            const isHtml = /^\s*</.test(rawText) || /<!DOCTYPE/i.test(rawText) || /<html/i.test(rawText);
            const expected = (typeof window.__API_BASE__ === 'string' && window.__API_BASE__.trim())
                ? window.__API_BASE__ + '/pointcloud-viewer/login.html'
                : 'http://localhost:5031/pointcloud-viewer/login.html';

            if (isHtml) {
                return {
                    success: false,
                    message: `无法连接后端服务（返回了页面而非接口数据）。请通过 ${expected} 访问系统登录页。`
                };
            }
            return { success: false, message: `服务器响应格式错误：${rawText.slice(0, 80) || '响应为空'}` };
        }

        if (!response.ok || !result.success) {
            return {
                success: false,
                message: result.message || `登录失败 (HTTP ${response.status})`
            };
        }

        return {
            success: true,
            token: result.data?.token || '',
            user: result.data?.user || null
        };
    } catch (error) {
        console.error('登录请求异常:', error);
        const expected = (typeof window.__API_BASE__ === 'string' && window.__API_BASE__.trim())
            ? window.__API_BASE__ + '/pointcloud-viewer/login.html'
            : 'http://localhost:5031/pointcloud-viewer/login.html';
        return { success: false, message: `无法连接服务器，请确认后端已启动并通过 ${expected} 访问系统` };
    }
}

// ============================
// 事件处理
// ============================

/**
 * 切换密码可见性
 */
togglePasswordBtn.addEventListener('click', () => {
    const isPassword = passwordInput.type === 'password';
    passwordInput.type = isPassword ? 'text' : 'password';
    togglePasswordBtn.setAttribute('aria-pressed', String(isPassword));
    togglePasswordBtn.setAttribute('aria-label', isPassword ? '隐藏密码' : '显示密码');
});

/**
 * 忘记密码
 */
forgotLink.addEventListener('click', (e) => {
    e.preventDefault();
    showToast('请联系管理员重置密码', 'info');
});

/**
 * 输入时清除错误
 */
usernameInput.addEventListener('input', () => clearFieldError(usernameInput, usernameError));
passwordInput.addEventListener('input', () => clearFieldError(passwordInput, passwordError));

/**
 * 表单提交
 */
form.addEventListener('submit', async (e) => {
    e.preventDefault();
    if (loginBtn.disabled) return;
    clearAllErrors();

    const username = usernameInput.value.trim();
    const password = passwordInput.value;
    const remember = rememberMeCheckbox.checked;

    let hasError = false;

    if (!username) {
        showFieldError(usernameInput, usernameError, '请输入用户名');
        hasError = true;
    }

    if (!password) {
        showFieldError(passwordInput, passwordError, '请输入密码');
        hasError = true;
    }

    if (hasError) return;

    setLoading(true);

    try {
        const result = await performLogin(username, password);
        if (result.success) {
            saveAuth(username, result.token, remember, result.user);
            showToast('登录成功，正在跳转...', 'info');
            setTimeout(() => {
                window.location.href = LOGIN_CONFIG.redirectUrl;
            }, 800);
        } else {
            showFieldError(passwordInput, passwordError, result.message || '登录失败');
            setLoading(false);
        }
    } catch (error) {
        console.error('登录异常:', error);
        showToast('登录请求异常，请稍后重试', 'error');
        setLoading(false);
    }
});

// ============================
// 初始化
// ============================

document.addEventListener('DOMContentLoaded', () => {
    // 如果已登录，直接跳转到首页
    if (sessionStorage.getItem(LOGIN_CONFIG.authKey)) {
        window.location.href = LOGIN_CONFIG.redirectUrl;
        return;
    }

    // 读取记住的账号
    const remembered = localStorage.getItem(LOGIN_CONFIG.rememberKey);
    if (remembered) {
        try {
            const { username } = JSON.parse(remembered);
            if (username) {
                usernameInput.value = username;
                rememberMeCheckbox.checked = true;
                passwordInput.focus();
            }
        } catch (error) {
            console.error('读取记住账号失败:', error);
        }
    } else {
        usernameInput.focus();
    }
});
