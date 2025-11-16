/* URDF Loader - Shared Module */
import * as THREE from 'three';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { ColladaLoader } from 'three/examples/jsm/loaders/ColladaLoader.js';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';

/**
 * 创建mesh加载函数
 * 用于加载不同格式的3D模型文件
 */
export function createMeshLoader() {
    return (path, manager, done) => {
        const ext = path.split(/\./g).pop().toLowerCase();
        switch (ext) {
            case 'gltf':
            case 'glb':
                new GLTFLoader(manager).load(
                    path,
                    result => done(result.scene),
                    null,
                    err => done(null, err),
                );
                break;
            case 'obj':
                new OBJLoader(manager).load(
                    path,
                    result => done(result),
                    null,
                    err => done(null, err),
                );
                break;
            case 'dae':
                new ColladaLoader(manager).load(
                    path,
                    result => done(result.scene),
                    null,
                    err => done(null, err),
                );
                break;
            case 'stl':
                new STLLoader(manager).load(
                    path,
                    result => {
                        const material = new THREE.MeshPhongMaterial();
                        const mesh = new THREE.Mesh(result, material);
                        done(mesh);
                    },
                    null,
                    err => done(null, err),
                );
                break;
        }
    };
}

/**
 * 加载URDF模型
 * @param {HTMLElement} viewer - URDF viewer元素
 * @param {string} urdfPath - URDF文件路径
 * @param {string} color - 背景颜色
 */
export function loadURDF(viewer, urdfPath, color) {
    viewer.up = '+Z';
    viewer.urdf = urdfPath;

    // 设置背景颜色和高亮颜色
    document.body.style.backgroundColor = color;
    viewer.highlightColor = '#' + (new THREE.Color(0xffffff))
        .lerp(new THREE.Color(color), 0.35)
        .getHexString();
}

/**
 * 初始化URDF选项列表
 * @param {HTMLElement} viewer - URDF viewer元素
 * @param {Function} onLoad - 加载完成后的回调函数
 */
export function initURDFOptions(viewer, onLoad) {
    document.querySelectorAll('#urdf-options li[urdf]').forEach(el => {
        el.addEventListener('click', e => {
            const urdf = e.target.getAttribute('urdf');
            const color = e.target.getAttribute('color');

            loadURDF(viewer, urdf, color);

            if (onLoad) {
                onLoad();
            }
        });
    });
}

