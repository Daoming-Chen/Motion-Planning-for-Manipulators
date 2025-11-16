# Kinematics Demos

这个目录包含了机器人运动学的交互式演示，包括正向运动学(FK)和逆向运动学(IK)。

## 📁 目录结构

```
kinematics/
├── shared/              # 共享模块
│   ├── urdf-loader.js  # URDF加载和mesh处理
│   ├── viewer-setup.js # 3D viewer初始化
│   └── styles.css      # 共享样式
├── fk/                  # 正向运动学 (Forward Kinematics)
│   ├── index.html      # FK演示页面
│   ├── index.js        # FK控制逻辑
│   └── styles.css      # FK特定样式
├── ik/                  # 逆向运动学 (Inverse Kinematics)
│   ├── index.html      # IK演示页面
│   ├── index.js        # IK控制逻辑
│   ├── ik-solver.js    # IK求解器（雅可比矩阵优化）
│   └── styles.css      # IK特定样式
└── c_space.html        # C-Space可视化
```

## 🎯 功能特性

### FK Demo (正向运动学)
- **控制方式**: 通过滑块控制各个关节的角度
- **输入**: 关节角度 (θ₁, θ₂, ..., θₙ)
- **输出**: 机器人末端执行器的位置和姿态
- **特点**: 
  - 实时3D可视化
  - 支持角度/弧度切换
  - 显示关节限制

### IK Demo (逆向运动学)
- **控制方式**: 通过滑块控制末端执行器的位置和姿态
- **输入**: 末端位置 (x, y, z) 和姿态 (rx, ry, rz)
- **输出**: 达到目标位姿所需的关节角度
- **算法**: 基于雅可比矩阵的迭代优化方法
- **特点**:
  - 实时IK求解
  - 显示求解误差和迭代次数
  - 自动更新机器人姿态
  - 显示计算得到的关节角度

## 🔧 技术实现

### 共享模块

#### urdf-loader.js
- 提供URDF文件加载功能
- 支持多种3D模型格式 (STL, OBJ, GLTF, Collada)
- 统一的模型加载接口

#### viewer-setup.js
- 初始化3D viewer
- 设置相机和场景
- 提供关节操作的辅助函数

#### styles.css
- 统一的UI样式
- 滑块和控制面板样式
- 响应式设计

### IK求解器

#### 算法原理
使用**基于雅可比矩阵的数值优化方法**:

1. **雅可比矩阵计算**: 
   ```
   J = ∂x/∂q
   ```
   其中 x 是末端执行器位姿，q 是关节角度

2. **迭代更新**:
   ```
   Δq = α · J^T · e
   ```
   其中:
   - α 是步长因子
   - J^T 是雅可比矩阵的转置（阻尼最小二乘近似）
   - e 是位姿误差向量

3. **收敛条件**:
   - 位置误差 < 1mm
   - 姿态误差 < 0.57°
   - 或达到最大迭代次数

#### 关键参数
- `maxIterations`: 最大迭代次数 (默认: 50)
- `positionTolerance`: 位置容差 (默认: 0.001m)
- `orientationTolerance`: 姿态容差 (默认: 0.01 rad)
- `stepSize`: 步长因子 (默认: 0.3)
- `positionWeight`: 位置权重 (默认: 1.0)
- `orientationWeight`: 姿态权重 (默认: 0.5)

## 🚀 使用方法

### 本地开发
```bash
# 启动开发服务器
npm run dev

# 访问演示
# FK: http://localhost:5173/web_demos/kinematics/fk/
# IK: http://localhost:5173/web_demos/kinematics/ik/
```

### 构建部署
```bash
# 构建
npm run build

# 部署后访问
# FK: /web_demos/kinematics/fk/
# IK: /web_demos/kinematics/ik/
```

## 📝 代码重构说明

### 为什么重构？
1. **代码复用**: FK和IK共享URDF加载和3D可视化逻辑
2. **可维护性**: 模块化设计便于后续功能扩展
3. **关注点分离**: 共享逻辑与特定逻辑分离

### 重构内容
- ✅ 提取共享的URDF加载逻辑
- ✅ 提取共享的3D viewer设置
- ✅ 统一样式管理
- ✅ 重构FK使用共享模块
- ✅ 实现IK的末端位置控制
- ✅ 实现基于雅可比矩阵的IK求解器

## 🔍 未来改进

### IK求解器优化
- [ ] 实现更精确的伪逆计算（SVD分解）
- [ ] 添加奇异性检测和处理
- [ ] 支持多解选择（肘部上/下配置）
- [ ] 添加碰撞检测
- [ ] 支持关节权重自定义

### 功能扩展
- [ ] 添加轨迹规划演示
- [ ] 集成动力学模拟
- [ ] 支持更多机器人模型
- [ ] 添加性能分析工具

## 📚 参考资料

- [URDF Loader](https://github.com/gkjohnson/urdf-loaders)
- [Three.js](https://threejs.org/)
- [Robotics: Modelling, Planning and Control](https://www.springer.com/gp/book/9781846286414)

