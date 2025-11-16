/* Viewer Setup - Shared Module */
import URDFManipulator from 'urdf-loader/src/urdf-manipulator-element.js';
import { createMeshLoader } from './urdf-loader.js';

/**
 * 初始化URDF Viewer
 * @param {HTMLElement} viewer - URDF viewer元素
 * @param {Object} config - 配置选项
 */
export function setupViewer(viewer, config = {}) {
    const {
        cameraPosition = [1, 1.2, 2.5],
        cameraLookAt = [1.5, -1, 0],
        robotPosition = [1, -1, 0],
        noAutoRecenter = true
    } = config;

    // 设置自动居中
    viewer.noAutoRecenter = noAutoRecenter;

    // 设置mesh加载函数
    viewer.loadMeshFunc = createMeshLoader();

    // 设置相机和机器人位置
    viewer.addEventListener('urdf-processed', () => {
        if (robotPosition) {
            viewer.robot.position.set(...robotPosition);
        }
    });

    // 设置相机位置
    if (cameraPosition) {
        viewer.camera.position.set(...cameraPosition);
    }
    if (cameraLookAt) {
        viewer.camera.lookAt(...cameraLookAt);
    }
}

/**
 * 注册自定义元素
 */
export function registerURDFViewer() {
    if (!customElements.get('urdf-viewer')) {
        customElements.define('urdf-viewer', URDFManipulator);
    }
}

/**
 * 获取机器人的关节信息
 * @param {Object} robot - URDF机器人对象
 * @returns {Array} 排序后的关节数组
 */
export function getSortedJoints(robot) {
    return Object.keys(robot.joints)
        .sort((a, b) => {
            const da = a.split(/[^\d]+/g).filter(v => !!v).pop();
            const db = b.split(/[^\d]+/g).filter(v => !!v).pop();

            if (da !== undefined && db !== undefined) {
                const delta = parseFloat(da) - parseFloat(db);
                if (delta !== 0) return delta;
            }

            if (a > b) return 1;
            if (b > a) return -1;
            return 0;
        })
        .map(key => robot.joints[key]);
}

/**
 * 设置关节值
 * @param {HTMLElement} viewer - URDF viewer元素
 * @param {string} jointName - 关节名称
 * @param {number} value - 关节值
 */
export function setJointValue(viewer, jointName, value) {
    viewer.setJointValue(jointName, value);
}

/**
 * 获取所有关节的当前值
 * @param {Object} robot - URDF机器人对象
 * @returns {Object} 关节名称到值的映射
 */
export function getJointValues(robot) {
    const values = {};
    Object.keys(robot.joints).forEach(name => {
        values[name] = robot.joints[name].angle;
    });
    return values;
}

/**
 * 设置所有关节的值
 * @param {HTMLElement} viewer - URDF viewer元素
 * @param {Object} values - 关节名称到值的映射
 */
export function setAllJointValues(viewer, values) {
    Object.keys(values).forEach(name => {
        viewer.setJointValue(name, values[name]);
    });
}

