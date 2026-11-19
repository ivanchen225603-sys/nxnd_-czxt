/**
 * 后端 API 通用客户端
 * 统一处理：认证令牌注入、标准响应解析（{ code, message, data, success }）、401 跳转登录
 */

const API = (function () {
    'use strict';

    const AUTH_KEY = 'car_pointcloud_auth';
    const LOGIN_URL = 'login.html';

    /**
     * 获取登录态（sessionStorage）
     */
    function getAuth() {
        try {
            const raw = sessionStorage.getItem(AUTH_KEY);
            return raw ? JSON.parse(raw) : null;
        } catch (e) {
            return null;
        }
    }

    /**
     * 保存登录态
     */
    function setAuth(username, token, user) {
        const auth = {
            username,
            token,
            user: user || null,
            loggedInAt: new Date().toISOString()
        };
        sessionStorage.setItem(AUTH_KEY, JSON.stringify(auth));
        return auth;
    }

    /**
     * 清除登录态
     */
    function clearAuth() {
        sessionStorage.removeItem(AUTH_KEY);
    }

    /**
     * 获取当前登录用户信息
     */
    function getCurrentUser() {
        const auth = getAuth();
        return auth ? (auth.user || { username: auth.username, token: auth.token }) : null;
    }

    /**
     * 获取后端 API 基础地址（支持 window.__API_BASE__ 覆盖，默认同源相对路径）
     */
    function getBase() {
        const base = (typeof window.__API_BASE__ === 'string' && window.__API_BASE__.trim())
            ? window.__API_BASE__.replace(/\/+$/, '')
            : '';
        return base;
    }

    /**
     * 请求后端接口（自动注入 Bearer Token，解析标准响应）
     * @param {string} url 接口路径（如 /api/users）
     * @param {Object} options fetch 选项
     * @returns {Promise<{code:number, message:string, data:any, success:boolean}>}
     */
    async function request(url, options = {}) {
        const headers = Object.assign({}, options.headers || {});
        const auth = getAuth();

        if (auth && auth.token) {
            headers['Authorization'] = 'Bearer ' + auth.token;
        }

        if (options.body && typeof options.body !== 'string' && !(options.body instanceof FormData)) {
            headers['Content-Type'] = 'application/json';
            options.body = JSON.stringify(options.body);
        }

        const fullUrl = getBase() + url;
        let response;
        try {
            response = await fetch(fullUrl, Object.assign({}, options, { headers }));
        } catch (e) {
            throw { code: 0, message: '无法连接服务器，请确认后端服务已启动' };
        }

        let result;
        try {
            result = await response.json();
        } catch (e) {
            throw { code: response.status, message: `服务器响应异常 (HTTP ${response.status})` };
        }

        // 401：登录失效，清理本地状态并跳转登录页
        if (response.status === 401 || result.code === 401) {
            clearAuth();
            if (!window.location.pathname.endsWith(LOGIN_URL)) {
                window.location.href = LOGIN_URL;
            }
            throw { code: 401, message: result.message || '登录已失效，请重新登录' };
        }

        if (!response.ok || result.code !== 200) {
            throw { code: result.code || response.status, message: result.message || '请求失败' };
        }

        return result;
    }

    /**
     * 便捷方法
     */
    function get(url) { return request(url); }
    function post(url, body) { return request(url, { method: 'POST', body }); }
    function put(url, body) { return request(url, { method: 'PUT', body }); }
    function del(url) { return request(url, { method: 'DELETE' }); }
    function upload(url, formData) {
        return request(url, { method: 'POST', body: formData });
    }

    /**
     * 下载文件（带认证，通过 fetch + blob）
     */
    async function download(url, filename) {
        const auth = getAuth();
        const headers = {};
        if (auth && auth.token) {
            headers['Authorization'] = 'Bearer ' + auth.token;
        }

        const response = await fetch(getBase() + url, { headers });
        if (!response.ok) {
            let msg = `下载失败 (HTTP ${response.status})`;
            try {
                const err = await response.json();
                if (err.message) msg = err.message;
            } catch (e) { /* ignore */ }
            throw { code: response.status, message: msg };
        }

        const blob = await response.blob();
        const objectUrl = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = objectUrl;
        a.download = filename || 'download';
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(objectUrl);
    }

    /** 读取需要 JWT 的二进制内容，供 PCD 查看器直接解析。 */
    async function arrayBuffer(url) {
        const auth = getAuth();
        const headers = {};
        if (auth && auth.token) {
            headers['Authorization'] = 'Bearer ' + auth.token;
        }

        const response = await fetch(getBase() + url, { headers });
        if (response.status === 401) {
            clearAuth();
            window.location.href = LOGIN_URL;
            throw { code: 401, message: '登录已失效，请重新登录' };
        }
        if (!response.ok) {
            let message = `点云读取失败 (HTTP ${response.status})`;
            try {
                const body = await response.json();
                message = body.message || message;
            } catch (e) { /* binary/non-json error body */ }
            throw { code: response.status, message };
        }
        return response.arrayBuffer();
    }

    return {
        AUTH_KEY,
        getAuth,
        setAuth,
        clearAuth,
        getCurrentUser,
        request,
        get,
        post,
        put,
        del,
        upload,
        download,
        arrayBuffer
    };
})();
