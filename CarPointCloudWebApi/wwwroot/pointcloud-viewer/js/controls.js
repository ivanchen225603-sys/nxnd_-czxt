/**
 * 交互控制器
 * 封装OrbitControls并提供高级交互功能
 */

class InteractionController {
    constructor(viewer) {
        this.viewer = viewer;
        this.controls = viewer.controls;
        
        // 绑定事件
        this.bindEvents();
    }

    /**
     * 绑定UI控件事件
     */
    bindEvents() {
        // 点大小滑块
        const pointSizeSlider = document.getElementById('pointSize');
        if (pointSizeSlider) {
            pointSizeSlider.addEventListener('input', (e) => {
                const size = parseFloat(e.target.value);
                console.log('[Controls] 点大小滑块变化:', size);
                this.viewer.setPointSize(size);
                document.getElementById('pointSizeValue').textContent = size.toFixed(2);
            });
        }
        
        // 着色模式选择
        const colorModeSelect = document.getElementById('colorMode');
        if (colorModeSelect) {
            colorModeSelect.addEventListener('change', (e) => {
                this.viewer.setColorMode(e.target.value);
            });
        }
        
        // 显示坐标轴复选框
        const showAxesCheckbox = document.getElementById('showAxes');
        if (showAxesCheckbox) {
            showAxesCheckbox.addEventListener('change', (e) => {
                this.viewer.showAxes(e.target.checked);
            });
        }
        
        // 显示网格复选框
        const showGridCheckbox = document.getElementById('showGrid');
        if (showGridCheckbox) {
            showGridCheckbox.addEventListener('change', (e) => {
                this.viewer.showGrid(e.target.checked);
            });
        }
        
        // 重置视角按钮
        const resetBtn = document.getElementById('resetBtn');
        if (resetBtn) {
            resetBtn.addEventListener('click', () => {
                this.viewer.resetView();
            });
        }
        
        // 截图按钮
        const screenshotBtn = document.getElementById('screenshotBtn');
        if (screenshotBtn) {
            screenshotBtn.addEventListener('click', () => {
                this.viewer.takeScreenshot();
            });
        }
        
        // 视角方向选择
        const viewOrientationSelect = document.getElementById('viewOrientation');
        if (viewOrientationSelect) {
            viewOrientationSelect.addEventListener('change', (e) => {
                const orientation = e.target.value;
                this.viewer.currentOrientation = orientation;
                if (this.viewer.originalPositions) {
                    this.viewer.fitCameraToObject(this.viewer.originalPositions, { orientation });
                    updateStatus(`视角方向已切换: ${orientation === 'auto' ? '自动适配' : orientation === 'landscape' ? '横向显示' : '纵向显示'}`);
                }
            });
        }
    }

    /**
     * 获取当前模式
     * @returns {string} 当前模式: 'normal'
     */
    getCurrentMode() {
        return 'normal';
    }
}
