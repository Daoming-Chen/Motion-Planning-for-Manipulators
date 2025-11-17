/* FK (Forward Kinematics) Demo - Joint Angle Control */
import * as THREE from 'three';
import { registerURDFViewer, setupViewer, getSortedJoints, setJointValue } from '../shared/viewer-setup.js';
import { initURDFOptions } from '../shared/urdf-loader.js';

// Register custom element
registerURDFViewer();

const viewer = document.querySelector('urdf-viewer');
const limitsToggle = document.getElementById('ignore-joint-limits');
const collisionToggle = document.getElementById('collision-toggle');
const radiansToggle = document.getElementById('radians-toggle');
const autocenterToggle = document.getElementById('autocenter-toggle');
const upSelect = document.getElementById('up-select');
const sliderList = document.querySelector('#controls ul');
const controlsel = document.getElementById('controls');
const controlsToggle = document.getElementById('toggle-controls');
const animToggle = document.getElementById('do-animate');
const hideFixedToggle = document.getElementById('hide-fixed');

const DEG2RAD = Math.PI / 180;
const RAD2DEG = 1 / DEG2RAD;
let sliders = {};

// Toggle event handlers
limitsToggle.addEventListener('click', () => {
    limitsToggle.classList.toggle('checked');
    viewer.ignoreLimits = limitsToggle.classList.contains('checked');
});

radiansToggle.addEventListener('click', () => {
    radiansToggle.classList.toggle('checked');
    Object.values(sliders).forEach(sl => sl.update());
});

collisionToggle.addEventListener('click', () => {
    collisionToggle.classList.toggle('checked');
    viewer.showCollision = collisionToggle.classList.contains('checked');
});

autocenterToggle.addEventListener('click', () => {
    autocenterToggle.classList.toggle('checked');
    viewer.noAutoRecenter = !autocenterToggle.classList.contains('checked');
});

hideFixedToggle.addEventListener('click', () => {
    hideFixedToggle.classList.toggle('checked');
    const hideFixed = hideFixedToggle.classList.contains('checked');
    if (hideFixed) controlsel.classList.add('hide-fixed');
    else controlsel.classList.remove('hide-fixed');
});

// Initialize hide fixed joints on load
if (hideFixedToggle.classList.contains('checked')) {
    controlsel.classList.add('hide-fixed');
}

upSelect.addEventListener('change', () => viewer.up = upSelect.value);
controlsToggle.addEventListener('click', () => controlsel.classList.toggle('hidden'));

// URDF change events
viewer.addEventListener('urdf-change', () => {
    Object.values(sliders).forEach(sl => sl.remove());
    sliders = {};
});

viewer.addEventListener('ignore-limits-change', () => {
    Object.values(sliders).forEach(sl => sl.update());
});

viewer.addEventListener('angle-change', e => {
    if (sliders[e.detail]) sliders[e.detail].update();
});

viewer.addEventListener('joint-mouseover', e => {
    const j = document.querySelector(`li[joint-name="${e.detail}"]`);
    if (j) j.setAttribute('robot-hovered', true);
});

viewer.addEventListener('joint-mouseout', e => {
    const j = document.querySelector(`li[joint-name="${e.detail}"]`);
    if (j) j.removeAttribute('robot-hovered');
});

let originalNoAutoRecenter;
viewer.addEventListener('manipulate-start', e => {
    const j = document.querySelector(`li[joint-name="${e.detail}"]`);
    if (j) {
        j.scrollIntoView({ block: 'nearest' });
        window.scrollTo(0, 0);
    }
    originalNoAutoRecenter = viewer.noAutoRecenter;
    viewer.noAutoRecenter = true;
});

viewer.addEventListener('manipulate-end', e => {
    viewer.noAutoRecenter = originalNoAutoRecenter;
});

// 初始关节值（度）
const initialJointValues = {
    'shoulder_pan_joint': 0,
    'shoulder_lift_joint': -91.9,
    'elbow_joint': 88.1,
    'wrist_1_joint': -91.9,
    'wrist_2_joint': -97.4,
    'wrist_3_joint': 0
};

// Create joint sliders
viewer.addEventListener('urdf-processed', () => {
    const r = viewer.robot;
    const joints = getSortedJoints(r);

    // 设置初始关节值
    Object.keys(initialJointValues).forEach(jointName => {
        if (r.joints[jointName]) {
            const valueInRadians = initialJointValues[jointName] * DEG2RAD;
            setJointValue(viewer, jointName, valueInRadians);
        }
    });

    joints.forEach(joint => {
        const li = document.createElement('li');
        li.innerHTML = `
            <span title="${joint.name}">${joint.name}</span>
            <input type="range" value="0" step="0.0001"/>
            <input type="number" step="0.0001" />
        `;
        li.setAttribute('joint-type', joint.jointType);
        li.setAttribute('joint-name', joint.name);

        sliderList.appendChild(li);

        const slider = li.querySelector('input[type="range"]');
        const input = li.querySelector('input[type="number"]');

        li.update = () => {
            const degMultiplier = radiansToggle.classList.contains('checked') ? 1.0 : RAD2DEG;
            let angle = joint.angle;

            if (joint.jointType === 'revolute' || joint.jointType === 'continuous') {
                angle *= degMultiplier;
            }

            if (Math.abs(angle) > 1) {
                angle = angle.toFixed(1);
            } else {
                angle = angle.toPrecision(2);
            }

            input.value = parseFloat(angle);
            slider.value = joint.angle;

            if (viewer.ignoreLimits || joint.jointType === 'continuous') {
                slider.min = -6.28;
                slider.max = 6.28;
                input.min = -6.28 * degMultiplier;
                input.max = 6.28 * degMultiplier;
            } else {
                slider.min = joint.limit.lower;
                slider.max = joint.limit.upper;
                input.min = joint.limit.lower * degMultiplier;
                input.max = joint.limit.upper * degMultiplier;
            }
        };

        switch (joint.jointType) {
            case 'continuous':
            case 'prismatic':
            case 'revolute':
                break;
            default:
                li.update = () => { };
                input.remove();
                slider.remove();
        }

        slider.addEventListener('input', () => {
            setJointValue(viewer, joint.name, slider.value);
            li.update();
        });

        input.addEventListener('change', () => {
            const degMultiplier = radiansToggle.classList.contains('checked') ? 1.0 : DEG2RAD;
            setJointValue(viewer, joint.name, input.value * degMultiplier);
            li.update();
        });

        li.update();
        sliders[joint.name] = li;
    });
});

// Initialize viewer and load URDF
document.addEventListener('WebComponentsReady', () => {
    setupViewer(viewer, {
        robotPosition: [0, 0, 0],  // 机器人位置与 world frame 对齐
        showWorldFrame: true,
        worldFrameSize: 0.3  // 世界坐标系大小
    });

    // Initialize URDF options
    initURDFOptions(viewer, () => {
        animToggle.classList.add('checked');
    });

    // Load default URDF
    document.querySelector('li[urdf]').dispatchEvent(new Event('click'));
});

// Animation toggle
document.addEventListener('WebComponentsReady', () => {
    animToggle.addEventListener('click', () => animToggle.classList.toggle('checked'));
    viewer.addEventListener('manipulate-start', e => animToggle.classList.remove('checked'));
});
