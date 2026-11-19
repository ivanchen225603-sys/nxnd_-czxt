/**
 * 点云3D查看器核心类
 * 负责Three.js场景初始化、渲染和点云显示
 */

class PointCloudViewer {
    constructor(containerId) {
        this.container = document.getElementById(containerId);
        if (!this.container) {
            throw new Error(`找不到容器元素: ${containerId}`);
        }

        // Three.js核心对象
        this.scene = null;
        this.camera = null;
        this.renderer = null;
        this.controls = null;

        // 点云对象
        this.pointCloud = null;
        this.pointGeometry = null;
        this.pointMaterial = null;

        // 点云容器组（用于横向/纵向旋转，不影响camera.up和OrbitControls）
        this.pointCloudGroup = null;

        // 辅助元素
        this.axesHelper = null;
        this.gridHelper = null;

        // 状态
        this.isInitialized = false;
        this.currentColorMode = 'uniform';
        this.currentOrientation = 'auto'; // 'auto', 'landscape', 'portrait'
        this.originalPositions = null;
        this.originalIntensities = null;

        // FPS计数
        this.frameCount = 0;
        this.lastFPSTime = performance.now();
        this.currentFPS = 0;

        this.init();
    }

    /**
     * 初始化Three.js场景
     */
    init() {
        // 1. 创建场景 - 不设置背景色，让CSS背景透出
        this.scene = new THREE.Scene();

        // 2. 设置相机
        const width = this.container.clientWidth;
        const height = this.container.clientHeight;
        this.camera = new THREE.PerspectiveCamera(75, width / height, 0.01, 1000000);
        this.camera.position.set(0, 0, 10);

        // 3. 配置渲染器 - 开启alpha让CSS背景透出
        this.renderer = new THREE.WebGLRenderer({
            antialias: true,
            alpha: true
        });
        // 先设置pixelRatio，再setSize，确保内部分辨率正确
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.renderer.setSize(width, height);
        // 强制设置canvas CSS尺寸匹配容器，不被pixelRatio影响
        this.renderer.domElement.style.width = width + 'px';
        this.renderer.domElement.style.height = height + 'px';
        // 修正camera.aspect为canvas实际像素比例（而非CSS比例）
        this.camera.aspect = this.renderer.domElement.width / this.renderer.domElement.height;
        this.camera.updateProjectionMatrix();
        this.renderer.setClearColor(0x000000, 0); // 完全透明清除色
        this.container.appendChild(this.renderer.domElement);

        // 4. 添加光源（虽然Points不需要，但为后续扩展准备）
        const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
        this.scene.add(ambientLight);

        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(10, 10, 10);
        this.scene.add(directionalLight);

        // 5. 添加坐标轴辅助 - 增大尺寸并加粗
        this.axesHelper = new THREE.AxesHelper(10);
        this.scene.add(this.axesHelper);

        // 6. 添加网格辅助（默认隐藏）
        this.gridHelper = new THREE.GridHelper(20, 20, 0x666666, 0x444444);
        this.gridHelper.position.y = -0.01;
        this.gridHelper.visible = false;
        this.scene.add(this.gridHelper);

        // 7. 创建点云容器组（横向/纵向旋转用，不修改camera.up）
        this.pointCloudGroup = new THREE.Group();
        this.scene.add(this.pointCloudGroup);

        // 8. 初始化OrbitControls
        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.05;
        this.controls.screenSpacePanning = false;
        this.controls.minDistance = 0.1;
        this.controls.maxDistance = 1000000; // 足够大，不限制缩放距离
        // 允许360度全角度旋转（穿过极点）
        this.controls.minPolarAngle = 0;
        this.controls.maxPolarAngle = Math.PI * 2;

        // 9. 处理窗口大小变化
        window.addEventListener('resize', () => this.onWindowResize());

        // 10. 启动渲染循环
        this.animate();

        this.isInitialized = true;
        updateStatus('场景初始化完成');
    }

    /**
     * 加载点云数据
     * @param {Object} pcdData - PCD解析后的数据
     */
    loadPointCloud(pcdData) {
        if (!this.isInitialized) {
            throw new Error('查看器未初始化');
        }

        // 清除旧点云和边界框
        if (this.pointCloud) {
            this.pointCloudGroup.remove(this.pointCloud);
            this.pointGeometry.dispose();
            this.pointMaterial.dispose();
        }
        if (this.bboxHelper) {
            this.pointCloudGroup.remove(this.bboxHelper);
        }

        // 重置容器组旋转
        this.pointCloudGroup.rotation.set(0, 0, 0);

        // 保存原始数据用于着色切换
        this.originalPositions = pcdData.positions;
        this.originalIntensities = pcdData.intensities;

        // 1. 创建BufferGeometry
        this.pointGeometry = new THREE.BufferGeometry();
        this.pointGeometry.setAttribute('position', new THREE.BufferAttribute(pcdData.positions, 3));

        // 2. 根据着色模式设置颜色
        let colors = null;
        if (pcdData.colors) {
            colors = pcdData.colors;
        } else if (this.currentColorMode === 'height') {
            colors = this.applyHeightColoring(pcdData.positions);
        } else if (this.currentColorMode === 'intensity' && pcdData.intensities) {
            colors = this.applyIntensityColoring(pcdData.intensities);
        }

        if (colors) {
            this.pointGeometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
        }

        // 3. 创建PointsMaterial - 使用更亮的颜色
        this.pointMaterial = new THREE.PointsMaterial({
            size: 0.5, // 固定默认点大小
            vertexColors: colors ? THREE.VertexColors : false,
            color: colors ? 0xffffff : 0x7bbcff,
            sizeAttenuation: false, // 禁用衰减，确保所有点大小一致
            transparent: true,
            opacity: 0.9
        });

        // 4. 创建Points对象，添加到容器组
        this.pointCloud = new THREE.Points(this.pointGeometry, this.pointMaterial);
        this.pointCloudGroup.add(this.pointCloud);

        // 5. 创建边界框，添加到容器组
        this.createBoundingBox(pcdData.positions);

        // 6. 自动调整相机视角
        this.fitCameraToObject(pcdData.positions);

        // 7. 更新UI
        updatePointCount(pcdData.pointCount);
        updateStatus(`加载成功: ${pcdData.pointCount.toLocaleString()} 个点`);

        // 8. 更新点云信息面板
        this.updatePointInfo(pcdData);

        // 9. 更新边界框面板
        this.updateBBoxPanel(pcdData.positions);
    }

    /**
     * 创建点云边界框
     * @param {Float32Array} positions - 位置数组
     */
    createBoundingBox(positions) {
        const parser = new PCDParser();
        const bbox = parser.getBoundingBox(positions);

        const width = bbox.maxX - bbox.minX;
        const height = bbox.maxY - bbox.minY;
        const depth = bbox.maxZ - bbox.minZ;

        const geometry = new THREE.BoxGeometry(width, height, depth);
        const edges = new THREE.EdgesGeometry(geometry);
        const material = new THREE.LineBasicMaterial({
            color: 0x00ffff,
            transparent: true,
            opacity: 0.6
        });

        this.bboxHelper = new THREE.LineSegments(edges, material);
        this.bboxHelper.position.set(
            (bbox.minX + bbox.maxX) / 2,
            (bbox.minY + bbox.maxY) / 2,
            (bbox.minZ + bbox.maxZ) / 2
        );
        this.pointCloudGroup.add(this.bboxHelper);
    }

    /**
     * 更新边界框面板
     * @param {Float32Array} positions - 位置数组
     */
    updateBBoxPanel(positions) {
        const parser = new PCDParser();
        const bbox = parser.getBoundingBox(positions);
        const center = parser.getCenter(positions);

        const bboxPanel = document.getElementById('bbox-panel');
        if (bboxPanel) {
            bboxPanel.style.display = 'block';

            document.getElementById('bbox-x').textContent =
                `${formatNumber(bbox.minX)} ~ ${formatNumber(bbox.maxX)}`;
            document.getElementById('bbox-y').textContent =
                `${formatNumber(bbox.minY)} ~ ${formatNumber(bbox.maxY)}`;
            document.getElementById('bbox-z').textContent =
                `${formatNumber(bbox.minZ)} ~ ${formatNumber(bbox.maxZ)}`;
            document.getElementById('bbox-center').textContent =
                `(${formatNumber(center.x)}, ${formatNumber(center.y)}, ${formatNumber(center.z)})`;
        }
    }

    /**
     * 自动调整相机以包含所有点
     * 智能选择最佳观察方向，确保点云完整显示在画布内
     * 横向/纵向通过旋转pointCloudGroup实现，不修改camera.up
     * @param {Float32Array} positions - 位置数组
     * @param {Object} options - 可选参数 { orientation: 'auto'|'landscape'|'portrait' }
     */
    fitCameraToObject(positions, options = {}) {
        const orientation = options.orientation || this.currentOrientation || 'auto';
        const parser = new PCDParser();
        const center = parser.getCenter(positions);
        const bbox = parser.getBoundingBox(positions);

        // camera.up 始终保持标准值，不修改（保证OrbitControls鼠标操作方向一致）
        this.camera.up.set(0, 1, 0);

        // 使用canvas的实际像素尺寸（而非CSS尺寸）来计算相机距离
        const canvasWidth = this.renderer.domElement.width;
        const canvasHeight = this.renderer.domElement.height;
        const canvasAspect = canvasWidth / canvasHeight;

        // 计算三个轴向的跨度
        const spanX = bbox.maxX - bbox.minX;
        const spanY = bbox.maxY - bbox.minY;
        const spanZ = bbox.maxZ - bbox.minZ;

        // 定义三个观察方向
        // axis: 相机放置的轴方向
        // spanA: 画布水平方向对应的跨度
        // spanB: 画布垂直方向(FOV)对应的跨度
        const spans = [
            { axis: 'Y', spanA: spanX, spanB: spanZ, depth: spanY },
            { axis: 'X', spanA: spanY, spanB: spanZ, depth: spanX },
            { axis: 'Z', spanA: spanX, spanB: spanY, depth: spanZ },
        ];

        // 选择最佳观察方向（展开面积最大）
        let bestView = spans[0];
        let bestArea = spans[0].spanA * spans[0].spanB;
        for (let i = 1; i < spans.length; i++) {
            const area = spans[i].spanA * spans[i].spanB;
            if (area > bestArea) {
                bestArea = area;
                bestView = spans[i];
            }
        }

        console.log(`最佳观察方向: ${bestView.axis}轴, 展开面积: ${bestArea}, orientation: ${orientation}`);

        // 计算相机距离
        const fov = this.camera.fov * (Math.PI / 180);
        const halfFov = fov / 2;

        // 根据 orientation 决定 spanA/spanB 的映射
        let displaySpanA = bestView.spanA; // 水平方向需要覆盖的范围
        let displaySpanB = bestView.spanB; // 垂直方向(FOV)需要覆盖的范围
        let needRotateGroup = false;

        if (orientation === 'landscape') {
            // 横向优先：让较长的轴位于水平方向
            if (bestView.spanB > bestView.spanA) {
                needRotateGroup = true;
                displaySpanA = bestView.spanB;
                displaySpanB = bestView.spanA;
            }
        } else if (orientation === 'portrait') {
            // 纵向优先：让较长的轴位于垂直方向
            if (bestView.spanA > bestView.spanB) {
                needRotateGroup = true;
                displaySpanA = bestView.spanB;
                displaySpanB = bestView.spanA;
            }
        }

        // 垂直方向（FOV方向）所需距离
        const distForHeight = (displaySpanB / 2) / Math.tan(halfFov);

        // 水平方向所需距离（考虑画布宽高比）
        const halfHorizontalFov = Math.atan(Math.tan(halfFov) * canvasAspect);
        const distForWidth = (displaySpanA / 2) / Math.tan(halfHorizontalFov);

        // 最近一面仍须留出完整视野，深度的一半不能算作可用观察距离。
        const cameraDistance = Math.max(distForHeight, distForWidth) * 1.15 + bestView.depth / 2;

        // 根据最佳观察方向设置相机位置
        let camX = center.x, camY = center.y, camZ = center.z;
        switch (bestView.axis) {
            case 'Y':
                camY = center.y + cameraDistance;
                break;
            case 'X':
                camX = center.x + cameraDistance;
                break;
            case 'Z':
                camZ = center.z + cameraDistance;
                break;
        }

        this.camera.position.set(camX, camY, camZ);
        this.camera.lookAt(center.x, center.y, center.z);

        // 旋转点云容器组（而非camera.up），保持OrbitControls操作方向一致
        this._applyGroupRotation(bestView.axis, needRotateGroup);
        // 方向切换绕云图中心旋转，避免非原点档案旋转后移出画布。
        const pivot = new THREE.Vector3(center.x, center.y, center.z);
        this.pointCloudGroup.position.copy(pivot).sub(pivot.clone().applyEuler(this.pointCloudGroup.rotation));

        // 更新控制器目标点
        if (this.controls) {
            this.controls.target.set(center.x, center.y, center.z);
            this.controls.update();
        }

        // 调整网格大小
        this.scene.remove(this.gridHelper);
        const maxDim = Math.max(spanX, spanY, spanZ);
        const gridSize = Math.ceil(maxDim * 2);
        this.gridHelper = new THREE.GridHelper(gridSize, Math.min(gridSize / 10, 100), 0x888888, 0x555555);
        this.gridHelper.position.set(center.x, bbox.minY - 0.01, center.z);
        this.gridHelper.visible = false;
        this.scene.add(this.gridHelper);
    }

    /**
     * 旋转点云容器组实现横向/纵向切换
     * 不修改camera.up，保证OrbitControls鼠标操作方向始终与视觉一致
     * @param {string} viewAxis - 当前观察轴 'X'|'Y'|'Z'
     * @param {boolean} needRotate - 是否需要旋转90度
     */
    _applyGroupRotation(viewAxis, needRotate) {
        // 重置容器组旋转
        this.pointCloudGroup.rotation.set(0, 0, 0);

        if (!needRotate) return;

        // 根据观察方向，绕观察轴旋转90度
        // 这样画面上的点云会旋转90度，但camera.up始终是(0,1,0)
        switch (viewAxis) {
            case 'Y':
                // 从Y轴方向看，绕Y轴旋转-90度
                this.pointCloudGroup.rotation.y = -Math.PI / 2;
                break;
            case 'X':
                // 从X轴方向看，绕X轴旋转90度
                this.pointCloudGroup.rotation.x = Math.PI / 2;
                break;
            case 'Z':
                // 从Z轴方向看，绕Z轴旋转-90度
                this.pointCloudGroup.rotation.z = -Math.PI / 2;
                break;
        }
    }

    /**
     * 应用高度着色
     * @param {Float32Array} positions - 位置数组
     * @returns {Float32Array} 颜色数组
     */
    applyHeightColoring(positions) {
        const colors = new Float32Array(positions.length);

        // 找到Z轴范围
        let minZ = Infinity, maxZ = -Infinity;
        for (let i = 2; i < positions.length; i += 3) {
            if (positions[i] < minZ) minZ = positions[i];
            if (positions[i] > maxZ) maxZ = positions[i];
        }

        const rangeZ = maxZ - minZ;

        // 为每个点分配颜色
        for (let i = 0; i < positions.length; i += 3) {
            const t = rangeZ > 0 ? (positions[i + 2] - minZ) / rangeZ : 0.5;
            const [r, g, b] = getColorFromHeight(t);

            colors[i] = r;
            colors[i + 1] = g;
            colors[i + 2] = b;
        }

        return colors;
    }

    /**
     * 应用强度着色
     * @param {Float32Array} intensities - 强度数组
     * @returns {Float32Array} 颜色数组
     */
    applyIntensityColoring(intensities) {
        const colors = new Float32Array(intensities.length * 3);

        // 找到强度范围
        let maxIntensity = -Infinity;
        for (let i = 0; i < intensities.length; i++) {
            if (intensities[i] > maxIntensity) maxIntensity = intensities[i];
        }

        // 为每个点分配颜色（灰度）
        for (let i = 0; i < intensities.length; i++) {
            const t = maxIntensity > 0 ? intensities[i] / maxIntensity : 0.5;
            colors[i * 3] = t;
            colors[i * 3 + 1] = t;
            colors[i * 3 + 2] = t;
        }

        return colors;
    }

    /**
     * 设置点大小
     * @param {number} size - 点大小
     */
    setPointSize(size) {
        console.log('[Viewer] setPointSize called, size:', size);
        console.log('[Viewer] pointMaterial:', this.pointMaterial);
        
        if (this.pointMaterial) {
            console.log('[Viewer] 修改前 pointMaterial.size:', this.pointMaterial.size);
            this.pointMaterial.size = size;
            console.log('[Viewer] 修改后 pointMaterial.size:', this.pointMaterial.size);
            this.pointMaterial.needsUpdate = true; // 强制更新材质
        } else {
            console.warn('[Viewer] pointMaterial 不存在，无法设置点大小');
        }
    }

    /**
     * 设置着色模式
     * @param {string} mode - 着色模式: 'uniform', 'height', 'intensity'
     */
    setColorMode(mode) {
        if (!this.pointCloud || !this.originalPositions) return;

        this.currentColorMode = mode;

        // 移除旧的颜色属性
        this.pointGeometry.removeAttribute('color');

        let colors = null;

        switch (mode) {
            case 'uniform':
                this.pointMaterial.vertexColors = false;
                this.pointMaterial.color = new THREE.Color(0x7bbcff);
                break;

            case 'height':
                colors = this.applyHeightColoring(this.originalPositions);
                this.pointGeometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
                this.pointMaterial.vertexColors = true;
                this.pointMaterial.color = new THREE.Color(0xffffff);
                break;

            case 'intensity':
                if (this.originalIntensities) {
                    colors = this.applyIntensityColoring(this.originalIntensities);
                    this.pointGeometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
                    this.pointMaterial.vertexColors = true;
                    this.pointMaterial.color = new THREE.Color(0xffffff);
                } else {
                    console.warn('没有强度数据，无法应用强度着色');
                }
                break;
        }

        this.pointMaterial.needsUpdate = true;
    }

    /**
     * 显示/隐藏坐标轴
     * @param {boolean} show - 是否显示
     */
    showAxes(show) {
        if (this.axesHelper) {
            this.axesHelper.visible = show;
        }
    }

    /**
     * 显示/隐藏网格
     * @param {boolean} show - 是否显示
     */
    showGrid(show) {
        if (this.gridHelper) {
            this.gridHelper.visible = show;
        }
    }

    /**
     * 重置视角
     */
    resetView() {
        if (this.originalPositions) {
            this.fitCameraToObject(this.originalPositions);
            updateStatus('视角已重置');
        }
    }

    /**
     * 截图并下载
     */
    takeScreenshot() {
        this.renderer.render(this.scene, this.camera);
        const dataURL = this.renderer.domElement.toDataURL('image/png');

        // 创建下载链接
        const link = document.createElement('a');
        link.download = `pointcloud_${Date.now()}.png`;
        link.href = dataURL;
        link.click();

        updateStatus('截图已保存');
    }

    /**
     * 更新点云信息面板
     * @param {Object} pcdData - PCD数据
     */
    updatePointInfo(pcdData) {
        const parser = new PCDParser();
        const bbox = parser.getBoundingBox(pcdData.positions);
        const center = parser.getCenter(pcdData.positions);

        const infoHtml = `
            <div><strong>点数:</strong> ${pcdData.pointCount.toLocaleString()}</div>
            <div><strong>字段:</strong> ${pcdData.fields.join(', ')}</div>
            <div style="margin-top: 8px;"><strong>包围盒:</strong></div>
            <div>X: ${formatNumber(bbox.minX)} ~ ${formatNumber(bbox.maxX)}</div>
            <div>Y: ${formatNumber(bbox.minY)} ~ ${formatNumber(bbox.maxY)}</div>
            <div>Z: ${formatNumber(bbox.minZ)} ~ ${formatNumber(bbox.maxZ)}</div>
            <div style="margin-top: 8px;"><strong>中心点:</strong></div>
            <div>(${formatNumber(center.x)}, ${formatNumber(center.y)}, ${formatNumber(center.z)})</div>
        `;

        document.getElementById('pointInfo').innerHTML = infoHtml;
    }

    /**
     * 处理窗口大小变化
     */
    onWindowResize() {
        const width = this.container.clientWidth;
        const height = this.container.clientHeight;

        if (width === 0 || height === 0) return;

        this.renderer.setSize(width, height);
        // 强制设置canvas CSS尺寸匹配容器
        this.renderer.domElement.style.width = width + 'px';
        this.renderer.domElement.style.height = height + 'px';
        // 使用canvas实际像素比例更新cldcyv.aspect
        this.camera.aspect = this.renderer.domElement.width / this.renderer.domElement.height;
        this.camera.updateProjectionMatrix();

        // 如果有点云数据，重新适配相机
        if (this.originalPositions) {
            this.fitCameraToObject(this.originalPositions);
        }
    }

    /**
     * 动画循环
     */
    animate() {
        requestAnimationFrame(() => this.animate());

        // 更新控制器
        if (this.controls) {
            this.controls.update();
        }

        // 渲染场景
        this.renderer.render(this.scene, this.camera);

        // 计算FPS
        this.frameCount++;
        const currentTime = performance.now();
        if (currentTime - this.lastFPSTime >= 1000) {
            this.currentFPS = Math.round(this.frameCount * 1000 / (currentTime - this.lastFPSTime));
            updateFPS(this.currentFPS);
            this.frameCount = 0;
            this.lastFPSTime = currentTime;
        }
    }

    /**
     * 获取Raycaster用于拾取
     * @returns {THREE.Raycaster} Raycaster对象
     */
    getRaycaster() {
        return new THREE.Raycaster();
    }

    /**
     * 销毁查看器
     */
    dispose() {
        if (this.pointCloud) {
            this.pointCloudGroup.remove(this.pointCloud);
            this.pointGeometry.dispose();
            this.pointMaterial.dispose();
        }

        this.renderer.dispose();
        this.isInitialized = false;
    }
}
