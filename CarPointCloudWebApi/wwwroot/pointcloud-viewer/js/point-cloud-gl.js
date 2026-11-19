/**
 * 轻量 WebGL 点云渲染器（磅房实时页面专用）
 *
 * 为什么手写而不是引入 three.js：目标机在隔离内网，没有 CDN，也不希望为一个
 * 只需要"画点 + 转视角"的页面引入并随包分发一个通用 3D 引擎。这里只实现真正
 * 用得到的部分：两段点云、轨道相机、正交式预设视角、地面网格与坐标轴。
 *
 * 坐标约定：画面直接使用基准雷达 A（CAM_004）的坐标系，B 已在服务端用正式
 * B_to_A 外参变换过。因此"俯视/侧视/正视"是相对 A 的机体轴定义的，
 * 与地磅的实际朝向可能有夹角——需要时用自由旋转对准即可。
 */
const PointCloudGL = (function () {
    'use strict';

    const VERTEX_SHADER = [
        'attribute vec3 aPosition;',
        'uniform mat4 uViewProjection;',
        'uniform float uPointSize;',
        'void main() {',
        '  gl_Position = uViewProjection * vec4(aPosition, 1.0);',
        '  gl_PointSize = uPointSize;',
        '}'
    ].join('\n');

    const FRAGMENT_SHADER = [
        'precision mediump float;',
        'uniform vec4 uColor;',
        'void main() {',
        '  gl_FragColor = uColor;',
        '}'
    ].join('\n');

    function compile(gl, type, source) {
        const shader = gl.createShader(type);
        gl.shaderSource(shader, source);
        gl.compileShader(shader);
        if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
            const log = gl.getShaderInfoLog(shader);
            gl.deleteShader(shader);
            throw new Error('着色器编译失败: ' + log);
        }
        return shader;
    }

    function createProgram(gl) {
        const program = gl.createProgram();
        gl.attachShader(program, compile(gl, gl.VERTEX_SHADER, VERTEX_SHADER));
        gl.attachShader(program, compile(gl, gl.FRAGMENT_SHADER, FRAGMENT_SHADER));
        gl.linkProgram(program);
        if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
            throw new Error('着色器链接失败: ' + gl.getProgramInfoLog(program));
        }
        return program;
    }

    /** 列主序 4x4 矩阵相乘（out = a * b）。 */
    function multiply(out, a, b) {
        for (let col = 0; col < 4; col++) {
            for (let row = 0; row < 4; row++) {
                let sum = 0;
                for (let k = 0; k < 4; k++) {
                    sum += a[k * 4 + row] * b[col * 4 + k];
                }
                out[col * 4 + row] = sum;
            }
        }
        return out;
    }

    function perspective(fovyRadians, aspect, near, far) {
        const f = 1 / Math.tan(fovyRadians / 2);
        const nf = 1 / (near - far);
        return new Float32Array([
            f / aspect, 0, 0, 0,
            0, f, 0, 0,
            0, 0, (far + near) * nf, -1,
            0, 0, 2 * far * near * nf, 0
        ]);
    }

    function lookAt(eye, target, up) {
        const zx = eye[0] - target[0];
        const zy = eye[1] - target[1];
        const zz = eye[2] - target[2];
        let zLen = Math.hypot(zx, zy, zz) || 1;
        const z = [zx / zLen, zy / zLen, zz / zLen];

        let x = [
            up[1] * z[2] - up[2] * z[1],
            up[2] * z[0] - up[0] * z[2],
            up[0] * z[1] - up[1] * z[0]
        ];
        let xLen = Math.hypot(x[0], x[1], x[2]);
        if (xLen < 1e-6) {
            // 视线与 up 共线：换一根参考轴，避免矩阵退化导致画面整块消失。
            const fallback = Math.abs(z[2]) < 0.9 ? [0, 0, 1] : [0, 1, 0];
            x = [
                fallback[1] * z[2] - fallback[2] * z[1],
                fallback[2] * z[0] - fallback[0] * z[2],
                fallback[0] * z[1] - fallback[1] * z[0]
            ];
            xLen = Math.hypot(x[0], x[1], x[2]) || 1;
        }
        x = [x[0] / xLen, x[1] / xLen, x[2] / xLen];

        const y = [
            z[1] * x[2] - z[2] * x[1],
            z[2] * x[0] - z[0] * x[2],
            z[0] * x[1] - z[1] * x[0]
        ];

        return new Float32Array([
            x[0], y[0], z[0], 0,
            x[1], y[1], z[1], 0,
            x[2], y[2], z[2], 0,
            -(x[0] * eye[0] + x[1] * eye[1] + x[2] * eye[2]),
            -(y[0] * eye[0] + y[1] * eye[1] + y[2] * eye[2]),
            -(z[0] * eye[0] + z[1] * eye[1] + z[2] * eye[2]),
            1
        ]);
    }

    /**
     * 创建渲染器。
     * @param {HTMLCanvasElement} canvas 目标画布。
     * @returns {Object} 渲染器实例；WebGL 不可用时抛出异常。
     */
    function create(canvas) {
        const gl = canvas.getContext('webgl', { antialias: true, alpha: false });
        if (!gl) {
            throw new Error('当前浏览器不支持 WebGL，无法显示点云');
        }

        const program = createProgram(gl);
        const aPosition = gl.getAttribLocation(program, 'aPosition');
        const uViewProjection = gl.getUniformLocation(program, 'uViewProjection');
        const uPointSize = gl.getUniformLocation(program, 'uPointSize');
        const uColor = gl.getUniformLocation(program, 'uColor');

        const pointBuffer = gl.createBuffer();
        const surfaceBuffer = gl.createBuffer();
        const gridBuffer = gl.createBuffer();
        let dirty = true;

        const state = {
            pointCountA: 0,
            pointCountB: 0,
            pointSize: 2,
            azimuth: -Math.PI / 4,
            elevation: 0.5,
            distance: 30,
            target: [0, 0, 0],
            gridVertexCount: 0,
            bounds: null
        };

        function resize() {
            const ratio = window.devicePixelRatio || 1;
            const width = Math.max(1, Math.floor(canvas.clientWidth * ratio));
            const height = Math.max(1, Math.floor(canvas.clientHeight * ratio));
            if (canvas.width !== width || canvas.height !== height) {
                canvas.width = width;
                canvas.height = height;
                dirty = true;
            }
        }

        /** 依据当前点云范围重建地面网格与坐标轴，作为尺度参照。 */
        function rebuildGrid(bounds) {
            const spanX = Math.max(1, bounds.max[0] - bounds.min[0]);
            const spanY = Math.max(1, bounds.max[1] - bounds.min[1]);
            const half = Math.ceil(Math.max(spanX, spanY) / 2) + 2;
            const z = bounds.min[2];
            const cx = (bounds.min[0] + bounds.max[0]) / 2;
            const cy = (bounds.min[1] + bounds.max[1]) / 2;

            // 网格线间隔按跨度自适应，避免大车时网格密到糊成一片。
            const step = half > 30 ? 5 : 1;
            const vertices = [];
            for (let offset = -half; offset <= half; offset += step) {
                vertices.push(cx - half, cy + offset, z, cx + half, cy + offset, z);
                vertices.push(cx + offset, cy - half, z, cx + offset, cy + half, z);
            }

            gl.bindBuffer(gl.ARRAY_BUFFER, gridBuffer);
            gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(vertices), gl.STATIC_DRAW);
            state.gridVertexCount = vertices.length / 3;
        }

        /**
         * 上传一帧融合点云。
         * @param {Float32Array} coordinates 交错的 xyz 坐标（A 在前、B 在后）。
         * @param {number} pointCountA A 点位的点数。
         * @param {number} pointCountB B 点位的点数。
         * @param {boolean} autoFit 是否根据点云范围重置相机。
         */
        function setPoints(coordinates, pointCountA, pointCountB, autoFit, syntheticPointCount = 0, surface = null) {
            dirty = true;
            // 与点云整帧替换：暂停、断线、切换模式及历史文件不得留下上一辆车的面。
            state.surfaceVertexCount = surface ? surface.length / 3 : 0;
            gl.bindBuffer(gl.ARRAY_BUFFER, surfaceBuffer);
            gl.bufferData(gl.ARRAY_BUFFER, surface || new Float32Array(0), gl.DYNAMIC_DRAW);
            gl.bindBuffer(gl.ARRAY_BUFFER, pointBuffer);
            gl.bufferData(gl.ARRAY_BUFFER, coordinates, gl.DYNAMIC_DRAW);
            state.pointCountA = pointCountA;
            state.pointCountB = pointCountB;
            state.syntheticPointCount = syntheticPointCount;

            const min = [Infinity, Infinity, Infinity];
            const max = [-Infinity, -Infinity, -Infinity];
            for (let i = 0; i < coordinates.length; i += 3) {
                for (let axis = 0; axis < 3; axis++) {
                    const value = coordinates[i + axis];
                    if (value < min[axis]) min[axis] = value;
                    if (value > max[axis]) max[axis] = value;
                }
            }
            if (!isFinite(min[0])) {
                state.bounds = null;
                state.gridVertexCount = 0;
                return;
            }

            state.bounds = { min, max };
            rebuildGrid(state.bounds);
            if (autoFit) {
                fit();
            }
        }

        /** 把相机拉到刚好看全当前点云的位置。 */
        function fit() {
            if (!state.bounds) {
                return;
            }
            const { min, max } = state.bounds;
            dirty = true;
            state.target = [
                (min[0] + max[0]) / 2,
                (min[1] + max[1]) / 2,
                (min[2] + max[2]) / 2
            ];
            const diagonal = Math.hypot(max[0] - min[0], max[1] - min[1], max[2] - min[2]);
            state.distance = Math.max(2, diagonal * 1.1);
        }

        /** 切换到预设视角。 */
        function setView(name) {
            dirty = true;
            if (name === 'top') {
                state.azimuth = -Math.PI / 2;
                state.elevation = Math.PI / 2 - 0.001;
            } else if (name === 'side') {
                state.azimuth = 0;
                state.elevation = 0;
            } else if (name === 'front') {
                state.azimuth = -Math.PI / 2;
                state.elevation = 0;
            } else {
                state.azimuth = -Math.PI / 4;
                state.elevation = 0.5;
                fit();
            }
        }

        function render() {
            resize();
            // 实时帧或相机变化才重绘；冻结画面无需每个动画帧重复提交 GPU。
            if (!dirty) return;
            dirty = false;
            gl.viewport(0, 0, canvas.width, canvas.height);
            gl.clearColor(0.043, 0.063, 0.094, 1);
            gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
            gl.enable(gl.DEPTH_TEST);

            const cosE = Math.cos(state.elevation);
            const eye = [
                state.target[0] + state.distance * cosE * Math.cos(state.azimuth),
                state.target[1] + state.distance * cosE * Math.sin(state.azimuth),
                state.target[2] + state.distance * Math.sin(state.elevation)
            ];
            const aspect = canvas.width / Math.max(1, canvas.height);
            const projection = perspective(Math.PI / 4, aspect, Math.max(0.05, state.distance / 1000), state.distance * 10);
            const view = lookAt(eye, state.target, [0, 0, 1]);
            const viewProjection = multiply(new Float32Array(16), projection, view);

            gl.useProgram(program);
            gl.uniformMatrix4fv(uViewProjection, false, viewProjection);
            gl.enableVertexAttribArray(aPosition);

            if (state.gridVertexCount > 0) {
                gl.bindBuffer(gl.ARRAY_BUFFER, gridBuffer);
                gl.vertexAttribPointer(aPosition, 3, gl.FLOAT, false, 0, 0);
                gl.uniform1f(uPointSize, 1);
                gl.uniform4f(uColor, 0.16, 0.22, 0.31, 1);
                gl.drawArrays(gl.LINES, 0, state.gridVertexCount);
            }

            const total = state.pointCountA + state.pointCountB + (state.syntheticPointCount || 0);
            if (state.surfaceVertexCount > 0) {
                gl.bindBuffer(gl.ARRAY_BUFFER, surfaceBuffer);
                gl.vertexAttribPointer(aPosition, 3, gl.FLOAT, false, 0, 0);
                // 双面、不封顶；深紫色连续面是推断的展示副本，不是雷达回波。
                gl.uniform4f(uColor, 0.34, 0.14, 0.43, 1);
                gl.enable(gl.POLYGON_OFFSET_FILL);
                gl.polygonOffset(1, 1);
                gl.drawArrays(gl.TRIANGLES, 0, state.surfaceVertexCount);
                gl.disable(gl.POLYGON_OFFSET_FILL);
            }
            if (total > 0) {
                gl.bindBuffer(gl.ARRAY_BUFFER, pointBuffer);
                gl.vertexAttribPointer(aPosition, 3, gl.FLOAT, false, 0, 0);
                gl.uniform1f(uPointSize, state.pointSize);

                // 两次绘制而不是逐点颜色缓冲：来源只有两类且天然连续，
                // 省下一整个与点云等长的颜色数组。
                gl.uniform4f(uColor, 0.302, 0.639, 1.0, 1);
                gl.drawArrays(gl.POINTS, 0, state.pointCountA);

                gl.uniform4f(uColor, 1.0, 0.624, 0.263, 1);
                gl.drawArrays(gl.POINTS, state.pointCountA, state.pointCountB);
                // 合成壁面独立用紫色，不能与任一雷达实测点混淆。
                gl.uniform4f(uColor, 0.85, 0.36, 1.0, 1);
                gl.drawArrays(gl.POINTS, state.pointCountA + state.pointCountB, state.syntheticPointCount || 0);
            }
        }

        // ---- 交互：左键旋转、右键平移、滚轮缩放 ----
        let dragging = null;
        let lastX = 0;
        let lastY = 0;

        canvas.addEventListener('contextmenu', (e) => e.preventDefault());
        canvas.addEventListener('mousedown', (e) => {
            dragging = e.button === 2 ? 'pan' : 'orbit';
            lastX = e.clientX;
            lastY = e.clientY;
        });
        window.addEventListener('mouseup', () => { dragging = null; });
        window.addEventListener('mousemove', (e) => {
            if (!dragging) return;
            const dx = e.clientX - lastX;
            const dy = e.clientY - lastY;
            if (!dx && !dy) return;
            dirty = true;
            lastX = e.clientX;
            lastY = e.clientY;

            if (dragging === 'orbit') {
                state.azimuth -= dx * 0.008;
                state.elevation = Math.max(-1.5, Math.min(1.5, state.elevation + dy * 0.008));
            } else {
                const scale = state.distance * 0.0015;
                const right = [-Math.sin(state.azimuth), Math.cos(state.azimuth), 0];
                state.target[0] -= right[0] * dx * scale;
                state.target[1] -= right[1] * dx * scale;
                state.target[2] += dy * scale;
            }
        });
        canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            dirty = true;
            state.distance = Math.max(0.5, state.distance * (e.deltaY > 0 ? 1.1 : 0.9));
        }, { passive: false });

        return {
            setPoints,
            setView,
            fit,
            render,
            setPointSize(size) { if (state.pointSize !== size) { state.pointSize = size; dirty = true; } },
            get bounds() { return state.bounds; }
        };
    }

    return { create };
})();
