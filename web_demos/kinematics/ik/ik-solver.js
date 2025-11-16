/* IK Solver using Jacobian-based optimization */
import * as THREE from 'three';

/**
 * 计算机器人的雅可比矩阵
 * @param {Object} robot - URDF机器人对象
 * @param {Array} jointNames - 需要控制的关节名称列表
 * @returns {Array} 6xN的雅可比矩阵 (dx, dy, dz, drx, dry, drz)
 */
export function computeJacobian(robot, jointNames) {
    const n = jointNames.length;
    const jacobian = Array(6).fill(0).map(() => Array(n).fill(0));

    // 获取末端执行器的世界位置和姿态
    const endEffector = getEndEffector(robot);
    if (!endEffector) {
        console.warn('[computeJacobian] ⚠️ 未找到末端执行器');
        return jacobian;
    }

    const eePosition = new THREE.Vector3();
    endEffector.getWorldPosition(eePosition);

    // 对每个关节计算其对末端执行器的影响
    jointNames.forEach((jointName, i) => {
        const joint = robot.joints[jointName];
        if (!joint) return;

        // 获取关节的世界位置
        const jointPosition = new THREE.Vector3();
        joint.getWorldPosition(jointPosition);

        // 获取关节的旋转轴
        const axis = new THREE.Vector3(
            joint.axis.x,
            joint.axis.y,
            joint.axis.z
        );

        // 将轴转换到世界坐标系
        const worldAxis = axis.clone().applyQuaternion(joint.getWorldQuaternion(new THREE.Quaternion()));
        worldAxis.normalize();

        if (joint.jointType === 'revolute' || joint.jointType === 'continuous') {
            // 旋转关节
            // 线速度分量: v = ω × r
            const r = eePosition.clone().sub(jointPosition);
            const linearVel = worldAxis.clone().cross(r);

            jacobian[0][i] = linearVel.x;
            jacobian[1][i] = linearVel.y;
            jacobian[2][i] = linearVel.z;

            // 角速度分量: ω
            jacobian[3][i] = worldAxis.x;
            jacobian[4][i] = worldAxis.y;
            jacobian[5][i] = worldAxis.z;
        } else if (joint.jointType === 'prismatic') {
            // 平移关节
            jacobian[0][i] = worldAxis.x;
            jacobian[1][i] = worldAxis.y;
            jacobian[2][i] = worldAxis.z;
            // 角速度为0
            jacobian[3][i] = 0;
            jacobian[4][i] = 0;
            jacobian[5][i] = 0;
        }
    });

    return jacobian;
}

/**
 * 获取末端执行器
 * @param {Object} robot - URDF机器人对象
 * @returns {Object} 末端执行器对象
 */
function getEndEffector(robot) {
    // 查找最后一个非固定关节后的link
    const joints = Object.values(robot.joints);
    let endEffector = null;
    let maxDepth = -1;

    // 遍历所有links，找到层级最深的
    robot.traverse((child) => {
        if (child.isURDFLink) {
            let depth = 0;
            let current = child;
            while (current.parent) {
                depth++;
                current = current.parent;
            }
            if (depth > maxDepth) {
                maxDepth = depth;
                endEffector = child;
            }
        }
    });

    return endEffector;
}

/**
 * 获取末端执行器的当前位姿
 * @param {Object} robot - URDF机器人对象
 * @returns {Object} {position: [x,y,z], orientation: [rx,ry,rz]} 欧拉角为弧度
 */
export function getEndEffectorPose(robot) {
    const endEffector = getEndEffector(robot);
    if (!endEffector) {
        return { position: [0, 0, 0], orientation: [0, 0, 0] };
    }

    const position = new THREE.Vector3();
    endEffector.getWorldPosition(position);

    const quaternion = new THREE.Quaternion();
    endEffector.getWorldQuaternion(quaternion);

    const euler = new THREE.Euler();
    euler.setFromQuaternion(quaternion, 'XYZ');

    return {
        position: [position.x, position.y, position.z],
        orientation: [euler.x, euler.y, euler.z]
    };
}

/**
 * 矩阵转置
 */
function transpose(matrix) {
    const rows = matrix.length;
    const cols = matrix[0].length;
    const result = Array(cols).fill(0).map(() => Array(rows).fill(0));

    for (let i = 0; i < rows; i++) {
        for (let j = 0; j < cols; j++) {
            result[j][i] = matrix[i][j];
        }
    }

    return result;
}

/**
 * 矩阵乘法
 */
function matrixMultiply(a, b) {
    const aRows = a.length;
    const aCols = a[0].length;
    const bCols = b[0].length;
    const result = Array(aRows).fill(0).map(() => Array(bCols).fill(0));

    for (let i = 0; i < aRows; i++) {
        for (let j = 0; j < bCols; j++) {
            for (let k = 0; k < aCols; k++) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }

    return result;
}

/**
 * 矩阵与向量相乘
 */
function matrixVectorMultiply(matrix, vector) {
    const rows = matrix.length;
    const result = Array(rows).fill(0);

    for (let i = 0; i < rows; i++) {
        for (let j = 0; j < vector.length; j++) {
            result[i] += matrix[i][j] * vector[j];
        }
    }

    return result;
}

/**
 * 计算伪逆矩阵 (使用阻尼最小二乘法 - Damped Least Squares)
 * J^+ = J^T (J J^T + λ^2 I)^-1
 * @param {Array} J - 雅可比矩阵
 * @param {number} lambda - 阻尼因子
 * @returns {Array} 伪逆矩阵
 */
function pseudoInverse(J, lambda = 0.01) {
    const JT = transpose(J);
    const m = J.length;    // 任务空间维度 (6)
    const n = J[0].length; // 关节空间维度

    // 计算 J * J^T
    const JJT = matrixMultiply(J, JT);

    // 添加阻尼项: J * J^T + λ^2 * I
    for (let i = 0; i < m; i++) {
        JJT[i][i] += lambda * lambda;
    }

    // 使用高斯消元法求解 (J * J^T + λ^2 * I)^-1
    const JJT_inv = invertMatrix(JJT);

    // 计算 J^+ = J^T * (J * J^T + λ^2 * I)^-1
    const J_pseudo = matrixMultiply(JT, JJT_inv);

    return J_pseudo;
}

/**
 * 矩阵求逆 (使用高斯-约旦消元法)
 * @param {Array} matrix - 方阵
 * @returns {Array} 逆矩阵
 */
function invertMatrix(matrix) {
    const n = matrix.length;
    const augmented = matrix.map((row, i) => {
        const identityRow = Array(n).fill(0);
        identityRow[i] = 1;
        return [...row, ...identityRow];
    });

    // 前向消元
    for (let i = 0; i < n; i++) {
        // 寻找主元
        let maxRow = i;
        for (let k = i + 1; k < n; k++) {
            if (Math.abs(augmented[k][i]) > Math.abs(augmented[maxRow][i])) {
                maxRow = k;
            }
        }

        // 交换行
        [augmented[i], augmented[maxRow]] = [augmented[maxRow], augmented[i]];

        // 归一化当前行
        const pivot = augmented[i][i];
        if (Math.abs(pivot) < 1e-10) {
            // 矩阵奇异，返回单位矩阵的倍数作为近似
            console.warn('[invertMatrix] 矩阵接近奇异，使用近似值');
            return Array(n).fill(0).map((_, i) =>
                Array(n).fill(0).map((_, j) => i === j ? 1 : 0)
            );
        }

        for (let j = 0; j < 2 * n; j++) {
            augmented[i][j] /= pivot;
        }

        // 消元
        for (let k = 0; k < n; k++) {
            if (k !== i) {
                const factor = augmented[k][i];
                for (let j = 0; j < 2 * n; j++) {
                    augmented[k][j] -= factor * augmented[i][j];
                }
            }
        }
    }

    // 提取逆矩阵
    return augmented.map(row => row.slice(n));
}

/**
 * IK求解器 - 使用雅可比矩阵的迭代优化方法
 * @param {Object} robot - URDF机器人对象
 * @param {Array} jointNames - 需要控制的关节名称列表
 * @param {Array} targetPosition - 目标位置 [x, y, z]
 * @param {Array} targetOrientation - 目标姿态欧拉角 [rx, ry, rz] (弧度)
 * @param {Object} options - 配置选项
 * @param {HTMLElement} viewer - URDF viewer元素（用于更新可视化）
 * @param {Array} referenceAngles - 参考关节角度（起始点），如果未提供则使用机器人当前角度
 * @returns {Object} {success: bool, jointAngles: Array, error: number, iterations: number}
 */
export function solveIK(robot, jointNames, targetPosition, targetOrientation, options = {}, viewer = null, referenceAngles = null) {
    console.log('[solveIK] 开始求解');
    console.log('[solveIK] 关节名称:', jointNames);
    console.log('[solveIK] 目标位置:', targetPosition);
    console.log('[solveIK] 目标姿态:', targetOrientation);

    const {
        maxIterations = 100,
        positionTolerance = 0.001,  // 1mm
        orientationTolerance = 0.01, // ~0.57度
        stepSize = 0.5,
        positionWeight = 1.0,
        orientationWeight = 0.3,
        dampingFactor = 0.05  // 阻尼因子
    } = options;

    let iteration = 0;
    let error = Infinity;
    let prevError = Infinity;

    // 获取当前关节角度 - 使用提供的参考角度或机器人当前角度
    const currentAngles = referenceAngles
        ? referenceAngles.slice()  // 复制参考角度
        : jointNames.map(name => robot.joints[name].angle);  // 从机器人获取当前角度
    console.log('[solveIK] 初始关节角度 (reference):', currentAngles.map(a => (a * 180 / Math.PI).toFixed(1) + '°'));

    // 自适应步长
    let adaptiveStepSize = stepSize;

    while (iteration < maxIterations) {
        // 1. 计算当前末端执行器位姿
        const currentPose = getEndEffectorPose(robot);

        if (iteration === 0 || iteration % 10 === 0) {
            console.log(`[solveIK] 迭代 ${iteration}: 当前位姿`, currentPose);
        }

        // 2. 计算位姿误差
        const posError = [
            targetPosition[0] - currentPose.position[0],
            targetPosition[1] - currentPose.position[1],
            targetPosition[2] - currentPose.position[2]
        ];

        const oriError = [
            targetOrientation[0] - currentPose.orientation[0],
            targetOrientation[1] - currentPose.orientation[1],
            targetOrientation[2] - currentPose.orientation[2]
        ];

        // 归一化角度误差到 [-π, π]
        for (let i = 0; i < 3; i++) {
            while (oriError[i] > Math.PI) oriError[i] -= 2 * Math.PI;
            while (oriError[i] < -Math.PI) oriError[i] += 2 * Math.PI;
        }

        // 综合误差（加权）
        const fullError = [
            ...posError.map(e => e * positionWeight),
            ...oriError.map(e => e * orientationWeight)
        ];

        error = Math.sqrt(fullError.reduce((sum, e) => sum + e * e, 0));

        // 3. 检查收敛
        const posErrorNorm = Math.sqrt(posError.reduce((sum, e) => sum + e * e, 0));
        const oriErrorNorm = Math.sqrt(oriError.reduce((sum, e) => sum + e * e, 0));

        if (iteration === 0 || iteration % 10 === 0) {
            console.log(`[solveIK] 迭代 ${iteration}: 位置误差 ${posErrorNorm.toFixed(4)}, 姿态误差 ${oriErrorNorm.toFixed(4)}`);
        }

        if (posErrorNorm < positionTolerance && oriErrorNorm < orientationTolerance) {
            console.log(`[solveIK] ✅ 收敛! 迭代次数: ${iteration}, 最终误差: ${error.toFixed(6)}`);
            return {
                success: true,
                jointAngles: currentAngles.slice(),
                error,
                iterations: iteration,
                positionError: posErrorNorm,
                orientationError: oriErrorNorm
            };
        }

        // 4. 计算雅可比矩阵
        const J = computeJacobian(robot, jointNames);

        // 5. 计算伪逆（使用阻尼最小二乘法）
        const J_pseudo = pseudoInverse(J, dampingFactor);

        // 6. 计算关节角度变化 dq = α * J^+ * e
        const dq = matrixVectorMultiply(J_pseudo, fullError);

        // 自适应步长：如果误差增加，减小步长
        if (error > prevError && iteration > 0) {
            adaptiveStepSize *= 0.5;  // 减半步长
            if (iteration % 10 === 0) {
                console.log(`[solveIK] 误差增加，减小步长到 ${adaptiveStepSize.toFixed(3)}`);
            }
        } else if (error < prevError * 0.9) {
            // 如果误差快速下降，可以适当增加步长
            adaptiveStepSize = Math.min(stepSize, adaptiveStepSize * 1.1);
        }

        prevError = error;

        // 7. 更新关节角度
        for (let i = 0; i < jointNames.length; i++) {
            const oldAngle = currentAngles[i];
            currentAngles[i] += adaptiveStepSize * dq[i];

            // 限制关节范围
            const joint = robot.joints[jointNames[i]];
            if (joint.jointType === 'revolute' && joint.limit) {
                currentAngles[i] = Math.max(
                    joint.limit.lower,
                    Math.min(joint.limit.upper, currentAngles[i])
                );
            }

            // 应用到机器人 - 通过viewer更新以确保Three.js可视化同步
            if (viewer) {
                viewer.setJointValue(jointNames[i], currentAngles[i]);
            } else {
                // 如果没有viewer，回退到直接设置（可能不会触发可视化更新）
                robot.joints[jointNames[i]].setJointValue(currentAngles[i]);
            }

            if (iteration === 0 || iteration % 10 === 0) {
                console.log(`[solveIK] 关节 ${jointNames[i]}: ${(oldAngle * 180 / Math.PI).toFixed(1)}° -> ${(currentAngles[i] * 180 / Math.PI).toFixed(1)}° (Δ=${((currentAngles[i] - oldAngle) * 180 / Math.PI).toFixed(1)}°)`);
            }
        }

        iteration++;
    }

    console.log(`[solveIK] ⚠️ 达到最大迭代次数 ${maxIterations}，未完全收敛`);

    // 达到最大迭代次数
    return {
        success: false,
        jointAngles: currentAngles.slice(),
        error,
        iterations: iteration,
        positionError: Math.sqrt(
            (targetPosition[0] - getEndEffectorPose(robot).position[0]) ** 2 +
            (targetPosition[1] - getEndEffectorPose(robot).position[1]) ** 2 +
            (targetPosition[2] - getEndEffectorPose(robot).position[2]) ** 2
        ),
        orientationError: 0
    };
}

/**
 * 获取机器人的可控关节列表
 * @param {Object} robot - URDF机器人对象
 * @returns {Array} 可控关节名称列表
 */
export function getControllableJoints(robot) {
    return Object.keys(robot.joints).filter(name => {
        const joint = robot.joints[name];
        return joint.jointType === 'revolute' ||
            joint.jointType === 'continuous' ||
            joint.jointType === 'prismatic';
    });
}

