import { defineConfig } from 'vite';
import { resolve } from 'path';

export default defineConfig({
  root: '.',
  base: './',
  build: {
    outDir: 'dist',
    emptyOutDir: true,
    rollupOptions: {
      input: {
        'intro/dijkstra': resolve(__dirname, 'web_demos/intro/dijkstra.html'),
        'kinematics/c_space': resolve(__dirname, 'web_demos/kinematics/c_space.html'),
        'kinematics/fk': resolve(__dirname, 'web_demos/kinematics/fk/index.html'),
        'kinematics/ik': resolve(__dirname, 'web_demos/kinematics/ik/index.html'),
      },
      output: {
        entryFileNames: (chunkInfo) => {
          // 为每个入口配置不同的输出路径
          const name = chunkInfo.name;
          if (name === 'kinematics/fk') {
            return 'web_demos/kinematics/fk/fk.js';
          } else if (name === 'kinematics/ik') {
            return 'web_demos/kinematics/ik/ik.js';
          } else if (name === 'intro/dijkstra') {
            return 'web_demos/intro/dijkstra.js';
          } else if (name === 'kinematics/c_space') {
            return 'web_demos/kinematics/c_space.js';
          }
          return 'web_demos/[name]/[name].js';
        },
        chunkFileNames: 'web_demos/chunks/[name]-[hash].js',
        assetFileNames: (assetInfo) => {
          const name = assetInfo.name || '';

          // CSS 文件根据文件名判断输出路径
          if (name.endsWith('.css')) {
            // 根据文件名或源路径判断来源
            const source = assetInfo.source || '';
            if (name.includes('fk') || source.includes('kinematics/fk')) {
              return 'web_demos/kinematics/fk/fk.css';
            } else if (name.includes('ik') || source.includes('kinematics/ik')) {
              return 'web_demos/kinematics/ik/ik.css';
            } else if (name.includes('shared') || source.includes('kinematics/shared')) {
              return 'web_demos/kinematics/shared/styles.css';
            }
            // 默认输出到对应目录
            return 'web_demos/[name]/[name].css';
          }

          // URDF 和模型文件 - Vite 会自动处理，保持相对路径
          if (name.endsWith('.urdf') || name.endsWith('.obj') ||
            name.endsWith('.stl') || name.endsWith('.dae') ||
            name.endsWith('.gltf') || name.endsWith('.glb')) {
            // 使用默认路径结构，Vite 会保持相对路径
            return 'web_demos/[name]-[hash].[ext]';
          }

          // 其他资源文件
          return 'web_demos/assets/[name]-[hash].[ext]';
        }
      }
    },
    // Ensure proper asset handling
    assetsInlineLimit: 0,
  },
  server: {
    open: '/web_demos/kinematics/fk/index.html'
  },
  assetsInclude: ['**/*.urdf', '**/*.obj', '**/*.stl', '**/*.dae', '**/*.gltf', '**/*.glb']
});