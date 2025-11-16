/* IK (Inverse Kinematics) Demo - End Effector Control */
import * as THREE from 'three';
import { registerURDFViewer, setupViewer, getSortedJoints } from '../shared/viewer-setup.js';
import { initURDFOptions } from '../shared/urdf-loader.js';
import { solveIK, getEndEffectorPose, getControllableJoints } from './ik-solver.js';

console.log('=== IK Demo 页面加载开始 ===');

// Register custom element
registerURDFViewer();
console.log('✓ URDF Viewer 注册完成');

const viewer = document.querySelector('urdf-viewer');
const positionSlidersList = document.getElementById('position-sliders');
const orientationSlidersList = document.getElementById('orientation-sliders');
const jointList = document.getElementById('joint-list');
const ikErrorDisplay = document.getElementById('ik-error');
const ikIterationsDisplay = document.getElementById('ik-iterations');
const ikStatus = document.getElementById('ik-status');

const DEG2RAD = Math.PI / 180;
const RAD2DEG = 1 / DEG2RAD;

// 目标位姿（四元数表示，避免欧拉角跳变）
let targetPose = {
    position: [0, 0, 0],
    quaternion: new THREE.Quaternion(0, 0, 0, 1)  // 使用四元数表示姿态
};

// 用于UI显示的累计旋转角度（相对于初始姿态）
let cumulativeRotation = {
    rx: 0,  // 度
    ry: 0,
    rz: 0
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
let animationFrameId = null;
let positionSpeed = 0.2; // 位置速度：米/秒
let orientationSpeed = 25; // 姿态速度：度/秒
let activeIntervals = {}; // 存储活动的interval定时器
let currentPositionValues = { x: 0, y: 0, z: 0 }; // 存储当前位置值

/**
 * 创建位置控制（带按钮）
 */
function createPositionSliders() {
    positionSlidersList.innerHTML = '';

    ['x', 'y', 'z'].forEach((axis, index) => {
        const limits = positionLimits[axis];
        currentPositionValues[axis] = limits.default;
        const li = document.createElement('li');
        li.innerHTML = `
            <span class="label">${axis.toUpperCase()}:</span>
            <button class="slider-btn decrease" data-axis="${axis}" data-type="position">-</button>
            <span class="value" id="value-${axis}">${limits.default.toFixed(3)}m</span>
            <button class="slider-btn increase" data-axis="${axis}" data-type="position">+</button>
        `;
        positionSlidersList.appendChild(li);

        const valueDisplay = li.querySelector('.value');
        const decreaseBtn = li.querySelector('.decrease');
        const increaseBtn = li.querySelector('.increase');

        // 按钮控制
        setupButtonControl(decreaseBtn, increaseBtn, valueDisplay,
            (value) => {
                targetPose.position[index] = value;
                currentPositionValues[axis] = value;
                valueDisplay.textContent = `${value.toFixed(3)}m`;
                updateIK();
            },
            limits.min, limits.max, 0.01, positionSpeed
        );
    });
}

/**
 * 设置按钮控制逻辑
 */
function setupButtonControl(decreaseBtn, increaseBtn, valueDisplay, onUpdate, min, max, step, speed) {
    const axisKey = `${decreaseBtn.dataset.axis}-${decreaseBtn.dataset.type}`;
    const axis = decreaseBtn.dataset.axis;

    // 减少按钮 - 鼠标事件
    decreaseBtn.addEventListener('mousedown', (e) => {
        e.preventDefault();
        startContinuousChange(axisKey, -1, axis, onUpdate, min, max, step, speed);
    });
    decreaseBtn.addEventListener('mouseup', () => stopContinuousChange(axisKey));
    decreaseBtn.addEventListener('mouseleave', () => stopContinuousChange(axisKey));

    // 减少按钮 - 触摸事件（移动设备支持）
    decreaseBtn.addEventListener('touchstart', (e) => {
        e.preventDefault();
        startContinuousChange(axisKey, -1, axis, onUpdate, min, max, step, speed);
    });
    decreaseBtn.addEventListener('touchend', () => stopContinuousChange(axisKey));
    decreaseBtn.addEventListener('touchcancel', () => stopContinuousChange(axisKey));

    // 增加按钮 - 鼠标事件
    increaseBtn.addEventListener('mousedown', (e) => {
        e.preventDefault();
        startContinuousChange(axisKey, 1, axis, onUpdate, min, max, step, speed);
    });
    increaseBtn.addEventListener('mouseup', () => stopContinuousChange(axisKey));
    increaseBtn.addEventListener('mouseleave', () => stopContinuousChange(axisKey));

    // 增加按钮 - 触摸事件（移动设备支持）
    increaseBtn.addEventListener('touchstart', (e) => {
        e.preventDefault();
        startContinuousChange(axisKey, 1, axis, onUpdate, min, max, step, speed);
    });
    increaseBtn.addEventListener('touchend', () => stopContinuousChange(axisKey));
    increaseBtn.addEventListener('touchcancel', () => stopContinuousChange(axisKey));
}

/**
 * 开始连续改变值
 */
function startContinuousChange(key, direction, axis, onUpdate, min, max, step, speed) {
    // 如果已经有活动的interval，先清除
    if (activeIntervals[key]) {
        clearInterval(activeIntervals[key]);
    }

    // 立即执行一次
    changeValue(direction, axis, onUpdate, min, max, step);

    // 根据speed计算interval间隔（毫秒）
    // speed是单位/秒，我们需要计算每次改变step需要多少毫秒
    const intervalMs = (step / speed) * 1000;

    // 设置连续改变
    activeIntervals[key] = setInterval(() => {
        changeValue(direction, axis, onUpdate, min, max, step);
    }, intervalMs);
}

/**
 * 停止连续改变值
 */
function stopContinuousChange(key) {
    if (activeIntervals[key]) {
        clearInterval(activeIntervals[key]);
        delete activeIntervals[key];
    }
}

/**
 * 改变值
 */
function changeValue(direction, axis, onUpdate, min, max, step) {
    let currentValue = currentPositionValues[axis];
    let newValue = currentValue + (direction * step);

    // 限制在范围内
    newValue = Math.max(min, Math.min(max, newValue));

    // 四舍五入到step精度
    newValue = Math.round(newValue / step) * step;

    onUpdate(newValue);
}

/**
 * 姿态按钮控制 - 增量式旋转
 */
function setupOrientationButtonControl(decreaseBtn, increaseBtn, valueDisplay, axis, axisIndex) {
    const axisKey = `${axis}-orientation`;

    // 减少按钮 - 鼠标事件
    decreaseBtn.addEventListener('mousedown', (e) => {
        e.preventDefault();
        startIncrementalRotation(axisKey, -1, axis, axisIndex, valueDisplay);
    });
    decreaseBtn.addEventListener('mouseup', () => stopIncrementalRotation(axisKey));
    decreaseBtn.addEventListener('mouseleave', () => stopIncrementalRotation(axisKey));

    // 减少按钮 - 触摸事件
    decreaseBtn.addEventListener('touchstart', (e) => {
        e.preventDefault();
        startIncrementalRotation(axisKey, -1, axis, axisIndex, valueDisplay);
    });
    decreaseBtn.addEventListener('touchend', () => stopIncrementalRotation(axisKey));
    decreaseBtn.addEventListener('touchcancel', () => stopIncrementalRotation(axisKey));

    // 增加按钮 - 鼠标事件
    increaseBtn.addEventListener('mousedown', (e) => {
        e.preventDefault();
        startIncrementalRotation(axisKey, 1, axis, axisIndex, valueDisplay);
    });
    increaseBtn.addEventListener('mouseup', () => stopIncrementalRotation(axisKey));
    increaseBtn.addEventListener('mouseleave', () => stopIncrementalRotation(axisKey));

    // 增加按钮 - 触摸事件
    increaseBtn.addEventListener('touchstart', (e) => {
        e.preventDefault();
        startIncrementalRotation(axisKey, 1, axis, axisIndex, valueDisplay);
    });
    increaseBtn.addEventListener('touchend', () => stopIncrementalRotation(axisKey));
    increaseBtn.addEventListener('touchcancel', () => stopIncrementalRotation(axisKey));
}

/**
 * 开始增量式旋转
 */
function startIncrementalRotation(key, direction, axis, axisIndex, valueDisplay) {
    // 如果已经有活动的interval，先清除
    if (activeIntervals[key]) {
        clearInterval(activeIntervals[key]);
    }

    // 立即执行一次
    applyIncrementalRotation(direction, axis, axisIndex, valueDisplay);

    // 计算interval间隔（orientationSpeed是度/秒，步长为1度）
    const intervalMs = (1 / orientationSpeed) * 1000;

    // 设置连续旋转
    activeIntervals[key] = setInterval(() => {
        applyIncrementalRotation(direction, axis, axisIndex, valueDisplay);
    }, intervalMs);
}

/**
 * 停止增量式旋转
 */
function stopIncrementalRotation(key) {
    if (activeIntervals[key]) {
        clearInterval(activeIntervals[key]);
        delete activeIntervals[key];
    }
}

/**
 * 应用增量式旋转 - 关键函数
 * 使用四元数在当前姿态基础上施加旋转增量
 */
function applyIncrementalRotation(direction, axis, axisIndex, valueDisplay) {
    if (!viewer.robot || !isInitialized) return;

    // 旋转增量（度）
    const deltaAngleDeg = direction * 1;  // 每次旋转1度
    const deltaAngleRad = deltaAngleDeg * DEG2RAD;

    // 更新累计旋转角度（用于UI显示）
    const axisName = axis.substring(1); // 'rx' -> 'x'
    cumulativeRotation[axis] += deltaAngleDeg;

    // 限制累计角度在[-180, 180]范围内（用于显示）
    const limits = orientationLimits[axis];
    cumulativeRotation[axis] = Math.max(limits.min, Math.min(limits.max, cumulativeRotation[axis]));

    // 更新UI显示
    valueDisplay.textContent = `${cumulativeRotation[axis].toFixed(0)}°`;

    // 创建旋转轴（世界坐标系）
    const rotationAxis = new THREE.Vector3(
        axis === 'rx' ? 1 : 0,
        axis === 'ry' ? 1 : 0,
        axis === 'rz' ? 1 : 0
    );

    // 创建增量旋转四元数（轴角表示转四元数）
    const deltaQuat = new THREE.Quaternion();
    deltaQuat.setFromAxisAngle(rotationAxis, deltaAngleRad);

    // 在当前姿态基础上施加增量旋转
    // 新姿态 = 增量旋转 × 当前姿态
    targetPose.quaternion.premultiply(deltaQuat);
    targetPose.quaternion.normalize();  // 归一化四元数

    // 触发IK求解
    updateIK();
}

/**
 * 创建姿态控制（带按钮） - 增量式控制
 */
function createOrientationSliders() {
    orientationSlidersList.innerHTML = '';

    ['rx', 'ry', 'rz'].forEach((axis, index) => {
        const limits = orientationLimits[axis];
        const li = document.createElement('li');
        li.innerHTML = `
            <span class="label">${axis.toUpperCase()}:</span>
            <button class="slider-btn decrease" data-axis="${axis}" data-type="orientation">-</button>
            <span class="value" id="value-${axis}">${limits.default}°</span>
            <button class="slider-btn increase" data-axis="${axis}" data-type="orientation">+</button>
        `;
        orientationSlidersList.appendChild(li);

        const valueDisplay = li.querySelector('.value');
        const decreaseBtn = li.querySelector('.decrease');
        const increaseBtn = li.querySelector('.increase');

        // 按钮控制 - 增量式旋转
        setupOrientationButtonControl(decreaseBtn, increaseBtn, valueDisplay, axis, index);
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
 * 更新IK求解 - 实时模式（jog mode）
 */
function updateIK() {
    console.log('[updateIK] 开始执行');
    console.log('[updateIK] viewer.robot:', viewer.robot);
    console.log('[updateIK] isInitialized:', isInitialized);

    if (!viewer.robot || !isInitialized) {
        console.log('[updateIK] ❌ 未初始化或机器人未加载，跳过');
        return;
    }

    // 将四元数转换为欧拉角（用于IK求解器）
    const euler = new THREE.Euler();
    euler.setFromQuaternion(targetPose.quaternion, 'XYZ');
    const targetOrientation = [euler.x, euler.y, euler.z];

    console.log('[updateIK] 可控关节:', controllableJoints);
    console.log('[updateIK] 目标位置:', targetPose.position);
    console.log('[updateIK] 目标姿态 (四元数):', [
        targetPose.quaternion.x.toFixed(3),
        targetPose.quaternion.y.toFixed(3),
        targetPose.quaternion.z.toFixed(3),
        targetPose.quaternion.w.toFixed(3)
    ]);
    console.log('[updateIK] 目标姿态 (欧拉角-弧度):', targetOrientation.map(a => a.toFixed(3)));

    // 取消之前的动画帧请求（如果有）
    if (animationFrameId !== null) {
        cancelAnimationFrame(animationFrameId);
    }

    // 获取机器人当前的关节角度作为参考起点
    const currentJointAngles = controllableJoints.map(name => viewer.robot.joints[name].angle);
    console.log('[updateIK] 当前关节角度 (reference):', currentJointAngles.map(a => (a * RAD2DEG).toFixed(1) + '°'));

    // 立即执行IK求解，不等待下一帧
    console.log('[updateIK] 调用 solveIK...');
    const result = solveIK(
        viewer.robot,
        controllableJoints,
        targetPose.position,
        targetOrientation,  // 使用从四元数转换的欧拉角
        {
            maxIterations: 100,  // 增加最大迭代次数以提高收敛率
            positionTolerance: 0.001,  // 提高精度要求
            orientationTolerance: 0.01,
            stepSize: 0.3,  // 减小步长以避免发散，更稳定
            positionWeight: 1.0,
            orientationWeight: 0.3,
            dampingFactor: 0.05  // 添加阻尼因子
        },
        viewer,  // 传入viewer以确保可视化同步
        currentJointAngles  // 传入当前关节角度作为参考起点
    );

    console.log('[updateIK] ✅ IK求解完成:', result);

    // 使用requestAnimationFrame更新UI显示，避免阻塞
    animationFrameId = requestAnimationFrame(() => {
        // 更新显示（安全检查）
        if (ikErrorDisplay) {
            ikErrorDisplay.textContent = `误差: ${result.error.toFixed(4)}`;
        }
        if (ikIterationsDisplay) {
            ikIterationsDisplay.textContent = `迭代: ${result.iterations}`;
        }

        // 更新状态样式
        if (ikStatus) {
            ikStatus.classList.remove('error', 'success');
            if (result.success) {
                ikStatus.textContent = '状态: 成功 ✓';
                ikStatus.classList.add('success');
            } else if (result.error > 0.1) {
                ikStatus.textContent = '状态: 误差过大 ⚠';
                ikStatus.classList.add('error');
            } else {
                ikStatus.textContent = '状态: 部分收敛';
            }
        }

        // 更新关节显示
        updateJointDisplay();

        animationFrameId = null;
    });
}

/**
 * 从当前机器人姿态初始化目标位姿
 */
function initializeFromCurrentPose() {
    console.log('[初始化] 开始从当前位姿初始化');

    if (!viewer.robot) {
        console.log('[初始化] ❌ 机器人未加载');
        return;
    }

    const currentPose = getEndEffectorPose(viewer.robot);
    console.log('[初始化] 当前末端位姿:', currentPose);

    // 更新目标位置
    targetPose.position = currentPose.position.slice();

    // 将当前欧拉角转换为四元数
    const euler = new THREE.Euler(
        currentPose.orientation[0],
        currentPose.orientation[1],
        currentPose.orientation[2],
        'XYZ'
    );
    targetPose.quaternion.setFromEuler(euler);

    console.log('[初始化] 设置目标位置:', targetPose.position);
    console.log('[初始化] 设置目标姿态 (四元数):', [
        targetPose.quaternion.x.toFixed(3),
        targetPose.quaternion.y.toFixed(3),
        targetPose.quaternion.z.toFixed(3),
        targetPose.quaternion.w.toFixed(3)
    ]);

    // 重置累计旋转角度
    cumulativeRotation.rx = 0;
    cumulativeRotation.ry = 0;
    cumulativeRotation.rz = 0;

    // 更新位置显示
    ['x', 'y', 'z'].forEach((axis, index) => {
        const valueDisplay = document.getElementById(`value-${axis}`);
        if (valueDisplay) {
            const value = currentPose.position[index];
            currentPositionValues[axis] = value;
            valueDisplay.textContent = `${value.toFixed(3)}m`;
        }
    });

    // 更新姿态显示（显示为0，因为是相对增量）
    ['rx', 'ry', 'rz'].forEach((axis) => {
        const valueDisplay = document.getElementById(`value-${axis}`);
        if (valueDisplay) {
            valueDisplay.textContent = '0°';
        }
    });

    isInitialized = true;
    console.log('[初始化] ✅ 初始化完成, isInitialized =', isInitialized);
    updateJointDisplay();
}

// 初始关节值（度）
const initialJointValues = {
    'shoulder_pan_joint': 0,
    'shoulder_lift_joint': -91.9,
    'elbow_joint': 88.1,
    'wrist_1_joint': -91.9,
    'wrist_2_joint': -97.4,
    'wrist_3_joint': 0
};

// URDF加载完成后的处理
viewer.addEventListener('urdf-processed', () => {
    console.log('[事件] urdf-processed 触发');

    // 设置初始关节值
    Object.keys(initialJointValues).forEach(jointName => {
        if (viewer.robot.joints[jointName]) {
            const valueInRadians = initialJointValues[jointName] * DEG2RAD;
            viewer.setJointValue(jointName, valueInRadians);
            console.log(`[初始化] 设置关节 ${jointName} = ${initialJointValues[jointName]}° (${valueInRadians.toFixed(4)} rad)`);
        }
    });

    // 获取可控关节
    controllableJoints = getControllableJoints(viewer.robot);
    console.log('[事件] 可控关节列表:', controllableJoints);

    // 创建滑块
    createPositionSliders();
    createOrientationSliders();
    console.log('[事件] 滑块创建完成');

    // 等待机器人渲染后初始化
    setTimeout(() => {
        initializeFromCurrentPose();
    }, 100);

    // 速度已固定为 0.5 单位/秒，无需设置速度控制
});

// 监听关节变化（用于外部操作时更新显示）
viewer.addEventListener('angle-change', () => {
    if (isInitialized) {
        updateJointDisplay();
    }
});

// 初始化viewer和加载URDF
document.addEventListener('WebComponentsReady', () => {
    console.log('=== [页面] WebComponentsReady 事件触发 ===');

    setupViewer(viewer, {
        cameraPosition: [1.5, 1.5, 2.0],
        cameraLookAt: [0.5, 0, 0.5],
        robotPosition: [0, 0, 0],
        noAutoRecenter: true,
        showWorldFrame: true,
        worldFrameSize: 0.3  // 世界坐标系大小
    });
    console.log('[页面] Viewer 设置完成');

    // 初始化URDF选项
    initURDFOptions(viewer, () => {
        // URDF加载完成的回调
        console.log('[页面] URDF 选项初始化回调');
        isInitialized = false;
    });

    // 加载默认URDF
    const defaultURDF = document.querySelector('li[urdf]');
    if (defaultURDF) {
        console.log('[页面] 加载默认 URDF:', defaultURDF.getAttribute('urdf'));
        defaultURDF.dispatchEvent(new Event('click'));
    }
});

// 监听URDF变化
viewer.addEventListener('urdf-change', () => {
    console.log('[事件] urdf-change 触发 - 重置状态');
    isInitialized = false;

    // 重置目标姿态
    targetPose.quaternion.set(0, 0, 0, 1);
    cumulativeRotation.rx = 0;
    cumulativeRotation.ry = 0;
    cumulativeRotation.rz = 0;

    if (jointList) {
        jointList.innerHTML = '';
    }
    if (ikErrorDisplay) {
        ikErrorDisplay.textContent = '误差: 0.000';
    }
    if (ikIterationsDisplay) {
        ikIterationsDisplay.textContent = '迭代: 0';
    }
    if (ikStatus) {
        ikStatus.textContent = '状态: 就绪';
        ikStatus.classList.remove('error', 'success');
    }
});

