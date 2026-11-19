/* 独立通道点云：沿用原看板的固定 ROI、按高度着色和冻结语义。 */
const LoadRadarCloud = (function () {
    'use strict';

    function create(canvas) {
        const ctx = canvas.getContext('2d');
        if (!ctx) throw new Error('当前浏览器无法创建点云画布');
        let points = [], latest = [], bounds = null, frozen = false, pending = null, disposed = false;
        const initialYaw = -1.5, initialPitch = 1.1;
        let yaw = initialYaw, pitch = initialPitch, zoom = 1, drag = null;
        const listeners = [];
        const dpr = Math.min(window.devicePixelRatio || 1, 2);

        function listen(target, name, handler, options) {
            target.addEventListener(name, handler, options);
            listeners.push([target, name, handler, options]);
        }
        function schedule() {
            if (disposed || pending !== null) return;
            // 远程桌面或内嵌窗口可能停止动画帧，定时合并绘制仍能刷新。
            pending = setTimeout(() => { pending = null; draw(); }, 50);
        }
        function setPoints(value) {
            const rows = Array.isArray(value) ? value : [];
            const step = Math.max(1, Math.ceil(rows.length / 5000));
            const next = [];
            let changed = false;
            for (let i = 0; i < rows.length; i += step) {
                const p = rows[i];
                if (!Array.isArray(p) || p.length < 3 || !Number.isFinite(p[0]) || !Number.isFinite(p[1]) || !Number.isFinite(p[2])) continue;
                const previous = latest[next.length];
                if (previous && previous[0] === p[0] && previous[1] === p[1] && previous[2] === p[2]) next.push(previous);
                else { next.push(p.slice(0, 3)); changed = true; }
            }
            if (!changed && next.length === latest.length) return;
            latest = next;
            // 冻结时只保留最新数据，解除冻结再显示，避免重复投影和排序。
            if (frozen) return;
            points = latest;
            schedule();
        }
        function setBounds(value) {
            const next = Array.isArray(value) && value.length === 2 && value.every(v => Array.isArray(v) && v.length >= 3 && v.slice(0, 3).every(Number.isFinite))
                && value[0].slice(0, 3).every((v, i) => v < value[1][i])
                ? value.map(v => v.slice(0, 3)) : null;
            if (next === bounds || next && bounds && next.every((row, i) => row.every((v, j) => v === bounds[i][j]))) return;
            bounds = next;
            schedule();
        }
        function setFrozen(value) {
            if (frozen === !!value) return;
            frozen = !!value;
            if (!frozen) points = latest;
            schedule();
        }
        function clear() {
            if (!points.length && !latest.length && !frozen) return;
            points = []; latest = []; frozen = false;
            schedule();
        }
        function reset() { yaw = initialYaw; pitch = initialPitch; zoom = 1; schedule(); }
        function draw() {
            if (disposed) return;
            const rect = canvas.getBoundingClientRect();
            // 单通道视图会隐藏其余卡片；切回时由 fit 触发绘制最新数据。
            if (rect.width <= 0 || rect.height <= 0) return;
            canvas.width = Math.max(1, Math.round(rect.width * dpr));
            canvas.height = Math.max(1, Math.round(rect.height * dpr));
            const w = canvas.width, h = canvas.height;
            ctx.fillStyle = '#0e1626'; ctx.fillRect(0, 0, w, h);
            if (!points.length) {
                ctx.fillStyle = '#91a8c8'; ctx.font = (13 * dpr) + 'px sans-serif'; ctx.textAlign = 'center';
                ctx.fillText('没有可用点云', w / 2, h / 2); return;
            }
            let lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
            if (bounds) { lo = bounds[0]; hi = bounds[1]; }
            else for (const p of points) for (let i = 0; i < 3; i++) { lo[i] = Math.min(lo[i], p[i]); hi[i] = Math.max(hi[i], p[i]); }
            const center = lo.map((v, i) => (v + hi[i]) / 2);
            const cy = Math.cos(yaw), sy = Math.sin(yaw), cp = Math.cos(pitch), sp = Math.sin(pitch);
            // 用固定 ROI 的投影范围适配宽/窄画布，取景不会跟随单个点跳动。
            const span = hi.map((value, i) => value - lo[i]);
            const projectedWidth = span[0] * Math.abs(cy) + span[1] * Math.abs(sy);
            const projectedHeight = span[0] * Math.abs(sy * cp) + span[1] * Math.abs(cy * cp) + span[2] * Math.abs(sp);
            const scale = Math.min(w / Math.max(projectedWidth, 1), h / Math.max(projectedHeight, 1)) * 0.78 * zoom;
            const spanZ = Math.max(hi[2] - lo[2], 0.001);
            const projected = points.map(p => {
                const x = p[0] - center[0], y = p[1] - center[1], z = p[2] - center[2];
                const rx = x * cy - y * sy, ry = x * sy + y * cy;
                return [w / 2 + rx * scale, h / 2 - (ry * cp - z * sp) * scale, ry * sp + z * cp, Math.max(0, Math.min(1, (p[2] - lo[2]) / spanZ))];
            }).sort((a, b) => a[2] - b[2]);
            const size = Math.max(1.6 * dpr, 1.2);
            for (const p of projected) {
                ctx.fillStyle = 'rgb(' + Math.round(60 + p[3] * 195) + ',' + Math.round(140 + p[3] * 40) + ',' + Math.round(235 - p[3] * 180) + ')';
                ctx.fillRect(p[0] - size / 2, p[1] - size / 2, size, size);
            }
        }
        listen(canvas, 'pointerdown', e => { drag = [e.clientX, e.clientY]; });
        listen(window, 'pointerup', () => { drag = null; });
        listen(window, 'pointercancel', () => { drag = null; });
        listen(window, 'pointermove', e => {
            if (!drag) return;
            yaw += (e.clientX - drag[0]) * 0.008;
            pitch = Math.max(-1.45, Math.min(1.45, pitch + (e.clientY - drag[1]) * 0.006));
            drag = [e.clientX, e.clientY]; schedule();
        });
        listen(canvas, 'wheel', e => { e.preventDefault(); zoom = Math.max(0.2, Math.min(8, zoom * (e.deltaY < 0 ? 1.1 : 0.9))); schedule(); }, { passive: false });
        listen(canvas, 'dblclick', reset);
        listen(window, 'resize', schedule);
        schedule();
        return {
            setPoints, setBounds, setFrozen, clear, reset, fit: schedule,
            get frozen() { return frozen; },
            get pointCount() { return points.length; },
            dispose() {
                disposed = true;
                if (pending !== null) clearTimeout(pending);
                for (const [target, name, handler, options] of listeners) target.removeEventListener(name, handler, options);
                points = []; latest = [];
            }
        };
    }
    return { create };
})();
