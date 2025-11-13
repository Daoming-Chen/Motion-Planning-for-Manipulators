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
        fk: resolve(__dirname, 'web_demos/kinematics/fk/index.html'),
      },
      output: {
        entryFileNames: 'web_demos/kinematics/fk/fk.js',
        chunkFileNames: 'web_demos/kinematics/fk/chunks/[name]-[hash].js',
        assetFileNames: (assetInfo) => {
          if (assetInfo.name && assetInfo.name.endsWith('.css')) {
            return 'web_demos/kinematics/fk/fk.css';
          }
          return 'web_demos/kinematics/fk/assets/[name]-[hash].[ext]';
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