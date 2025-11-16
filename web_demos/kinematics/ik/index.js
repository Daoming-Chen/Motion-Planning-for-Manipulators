/* IK (Inverse Kinematics) Demo - End Effector Control */
import * as THREE from 'three';
import { registerURDFViewer, setupViewer, getSortedJoints } from '../shared/viewer-setup.js';
import { initURDFOptions } from '../shared/urdf-loader.js';
import { solveIK, getEndEffectorPose, getControllableJoints } from './ik-solver.js';

// Register custom element
registerURDFViewer();

const viewer = document.querySelector('urdf-viewer');
const positionSlidersList = document.getElementById('position-sliders');
const orientationSlidersList = document.getElementById('orientation-sliders');
const jointList = document.getElementById('joint-list');
const ikErrorDisplay = document.getElementById('ik-error');
const ikIterationsDisplay = document.getElementById('ik-iterations');
const ikStatus = document.getElementById('ik-status');

const DEG2RAD = Math.PI / 180;
const RAD2DEG = 1 / DEG2RAD;

// 目标位姿
let targetPose = {
    position: [0, 0, 0],
    orientation: [0, 0, 0]  // 欧拉角（弧度）
};

// 末端执行器控制范围
const positionLimits = {
    x: { min: -1.5, max: 1.5, default: 0.5 },
    y: { min: -1.5, max: 1.5, default: 0.3 },
    z: { min: -0.5, max: 2.0, default: 0.5 }
};

const orientationLimits = {
    rx: { min: -180, max: 180, default: 0 },
    ry: { min: -180, max: 180, default: 0 },
    rz: { min: -180, max: 180, default: 0 }
};

let controllableJoints = [];
let isInitialized = false;
let updateInProgress = false;

/**
 * 创建位置控制滑块
 */
function createPositionSliders() {
    positionSlidersList.innerHTML = '';
    
    ['x', 'y', 'z'].forEach((axis, index) => {
        const limits = positionLimits[axis];
        const li = document.createElement('li');
        li.innerHTML = `
            <span class="label">${axis.toUpperCase()}:</span>
            <input type="range" 
                   id="slider-${axis}" 
                   min="${limits.min}" 
                   max="${limits.max}" 
                   step="0.01" 
                   value="${limits.default}">
            <span class="value" id="value-${axis}">${limits.default.toFixed(3)}m</span>
        `;
        positionSlidersList.appendChild(li);
        
        const slider = li.querySelector('input[type="range"]');
        const valueDisplay = li.querySelector('.value');
        
        slider.addEventListener('input', () => {
            const value = parseFloat(slider.value);
            targetPose.position[index] = value;
            valueDisplay.textContent = `${value.toFixed(3)}m`;
            updateIK();
        });
    });
}

/**
 * 创建姿态控制滑块
 */
function createOrientationSliders() {
    orientationSlidersList.innerHTML = '';
    
    ['rx', 'ry', 'rz'].forEach((axis, index) => {
        const limits = orientationLimits[axis];
        const li = document.createElement('li');
        li.innerHTML = `
            <span class="label">${axis.toUpperCase()}:</span>
            <input type="range" 
                   id="slider-${axis}" 
                   min="${limits.min}" 
                   max="${limits.max}" 
                   step="1" 
                   value="${limits.default}">
            <span class="value" id="value-${axis}">${limits.default}°</span>
        `;
        orientationSlidersList.appendChild(li);
        
        const slider = li.querySelector('input[type="range"]');
        const valueDisplay = li.querySelector('.value');
        
        slider.addEventListener('input', () => {
            const valueDeg = parseFloat(slider.value);
            targetPose.orientation[index] = valueDeg * DEG2RAD;
            valueDisplay.textContent = `${valueDeg}°`;
            updateIK();
        });
    });
}

/**
 * 更新关节角度显示
 */
function updateJointDisplay() {
    if (!viewer.robot) return;
    
    jointList.innerHTML = '';
    
    controllableJoints.forEach(jointName => {
        const joint = viewer.robot.joints[jointName];
        if (!joint) return;
        
        const li = document.createElement('li');
        const angleDeg = (joint.angle * RAD2DEG).toFixed(1);
        li.innerHTML = `
            <span class="joint-name">${jointName}</span>
            <span class="joint-value">${angleDeg}°</span>
        `;
        jointList.appendChild(li);
    });
}

/**
 * 更新IK求解
 */
function updateIK() {
    if (!viewer.robot || !isInitialized || updateInProgress) return;
    
    updateInProgress = true;
    
    // 使用requestAnimationFrame以避免阻塞UI
    requestAnimationFrame(() => {
        const result = solveIK(
            viewer.robot,
            controllableJoints,
            targetPose.position,
            targetPose.orientation,
            {
                maxIterations: 50,
                positionTolerance: 0.001,
                orientationTolerance: 0.01,
                stepSize: 0.3,
                positionWeight: 1.0,
                orientationWeight: 0.5
            }
        );
        
        // 更新显示
        ikErrorDisplay.textContent = `误差: ${result.error.toFixed(4)}`;
        ikIterationsDisplay.textContent = `迭代: ${result.iterations}`;
        
        // 更新状态样式
        ikStatus.classList.remove('error', 'success');
        if (result.success) {
            ikStatus.classList.add('success');
        } else if (result.error > 0.1) {
            ikStatus.classList.add('error');
        }
        
        // 更新关节显示
        updateJointDisplay();
        
        updateInProgress = false;
    });
}

/**
 * 从当前机器人姿态初始化目标位姿
 */
function initializeFromCurrentPose() {
    if (!viewer.robot) return;
    
    const currentPose = getEndEffectorPose(viewer.robot);
    
    // 更新目标位姿
    targetPose.position = currentPose.position.slice();
    targetPose.orientation = currentPose.orientation.slice();
    
    // 更新滑块
    ['x', 'y', 'z'].forEach((axis, index) => {
        const slider = document.getElementById(`slider-${axis}`);
        const valueDisplay = document.getElementById(`value-${axis}`);
        if (slider && valueDisplay) {
            const value = currentPose.position[index];
            slider.value = value;
            valueDisplay.textContent = `${value.toFixed(3)}m`;
        }
    });
    
    ['rx', 'ry', 'rz'].forEach((axis, index) => {
        const slider = document.getElementById(`slider-${axis}`);
        const valueDisplay = document.getElementById(`value-${axis}`);
        if (slider && valueDisplay) {
            const valueDeg = currentPose.orientation[index] * RAD2DEG;
            slider.value = valueDeg;
            valueDisplay.textContent = `${valueDeg.toFixed(1)}°`;
        }
    });
    
    isInitialized = true;
    updateJointDisplay();
}

// URDF加载完成后的处理
viewer.addEventListener('urdf-processed', () => {
    // 获取可控关节
    controllableJoints = getControllableJoints(viewer.robot);
    
    // 创建滑块
    createPositionSliders();
    createOrientationSliders();
    
    // 等待机器人渲染后初始化
    setTimeout(() => {
        initializeFromCurrentPose();
    }, 100);
});

// 监听关节变化（用于外部操作时更新显示）
viewer.addEventListener('angle-change', () => {
    if (isInitialized && !updateInProgress) {
        updateJointDisplay();
    }
});

// 初始化viewer和加载URDF
document.addEventListener('WebComponentsReady', () => {
    setupViewer(viewer, {
        cameraPosition: [1.5, 1.5, 2.0],
        cameraLookAt: [0.5, 0, 0.5],
        robotPosition: [0, 0, 0],
        noAutoRecenter: true
    });
    
    // 初始化URDF选项
    initURDFOptions(viewer, () => {
        // URDF加载完成的回调
        isInitialized = false;
    });
    
    // 加载默认URDF
    const defaultURDF = document.querySelector('li[urdf]');
    if (defaultURDF) {
        defaultURDF.dispatchEvent(new Event('click'));
    }
});

// 监听URDF变化
viewer.addEventListener('urdf-change', () => {
    isInitialized = false;
    jointList.innerHTML = '';
    ikErrorDisplay.textContent = '误差: 0.000';
    ikIterationsDisplay.textContent = '迭代: 0';
    ikStatus.classList.remove('error', 'success');
});

