# Motion-Planning-for-Manipulators

本项目是一个面向“机械臂运动规划”主题的 Jupyter Notebook 教程，系统梳理了从基础理论到前沿方法、从经典采样到优化与学习增强的全流程知识。适合机器人学、自动化、人工智能等方向的研究者、工程师与学生自学或教学使用。

## 项目特色

- **结构化系列教程**：覆盖配置空间、采样式规划、轨迹优化、TAMP、学习增强、地图与碰撞检测等全链路内容。
- **可运行 Notebook**：每篇配有可复现代码、可视化与动手练习，便于理解与实践。
- **权威资源引用**：精选 OMPL、TrajOpt、CHOMP、STOMP、VLA/𝜋₀ 等核心论文与开源工具，附带延伸阅读。
- **工程与研究兼顾**：既适合理论学习，也可作为工程实现和项目开发的参考。

## 教程目录（建议学习顺序）

1. **序章**：系列总览与环境准备
2. **配置空间与碰撞基础**
3. **采样式规划（PRM/RRT 及变体）**
4. **最优采样与改进方法（RRT*/PRM*/BIT*/FMT*）**
5. **轨迹优化入门与三大方法（CHOMP/STOMP/TrajOpt）**
6. **连续碰撞与可微距离场**
7. **搜索式与动力学约束规划**
8. **学习增强与经验重用（MPNet/VLA/𝜋₀）**
9. **TAMP（任务与运动规划）综述**
10. **环境地图、碰撞检测与凸分解**
11. **性能评测与基准**
12. **未来趋势与开源工具清单**

详细章节与大纲见 [docs/](docs/) 及 notebooks 目录。

## 在线阅读

推荐直接访问 [在线 Jupyter Book](https://daoming-chen.github.io/Motion-Planning-for-Manipulators/notebooks/00_intro.html) ，以“电子书”形式顺序阅读并运行章节示例。若希望进一步探索配套代码与交互式 Notebook，建议 `git clone` 本仓库在本地环境中体验。

## 快速开始

1. **环境准备**  
	推荐使用 Ubuntu + ROS/ROS2 + OMPL/TrajOpt/FCL/PyBullet/Drake。  
	可用 `environment.yml` 或 `requirements.txt` 配置 Python 环境：

	```bash
	conda env create -f environment.yml
	# 或
	pip install -r requirements.txt
	```

2. **运行教程**  
	启动 Jupyter Lab/Notebook，依次打开 `notebooks/` 下的各章节 notebook，按提示运行与练习。

3. **可视化与实验**  
	每篇 notebook 提供了关键算法的可视化、动画与代码片段，便于动手实践与理解。

## 关键资源与参考文献

- [OMPL: The Open Motion Planning Library](https://ompl.kavrakilab.org/?utm_source=chatgpt.com) —— 采样式规划算法权威库
- [TrajOpt: Motion Planning with Sequential Convex Optimization](https://rll.berkeley.edu/trajopt/ijrr/2013-IJRR-TRAJOPT.pdf?utm_source=chatgpt.com) —— 优化类轨迹规划代表
- [CHOMP: Covariant Hamiltonian Optimization for Motion Planning](https://www.ri.cmu.edu/pub_files/2013/5/CHOMP_IJRR.pdf?utm_source=chatgpt.com)
- [STOMP: Stochastic Trajectory Optimization for Motion Planning](https://ros.fei.edu.br/roswiki/attachments/Papers%282f%29ICRA2011_Kalakrishnan/kalakrishnan_icra2011.pdf?utm_source=chatgpt.com)
- [Sampling-Based Motion Planning: A Comparative Review](https://www.kavrakilab.org/publications/orthey2024-review-sampling.pdf?utm_source=chatgpt.com)
- [𝜋₀: A Vision-Language-Action Flow Model for General Robot Policies](https://arxiv.org/html/2410.24164v1?utm_source=chatgpt.com)
- [OctoMap: An Efficient Probabilistic 3D Mapping Framework](https://courses.cs.washington.edu/courses/cse571/16au/slides/hornung13auro.pdf?utm_source=chatgpt.com)
- [FCL: A General Purpose Library for Collision and Proximity](https://gamma.cs.unc.edu/FCL/fcl_docs/webpage/pdfs/fcl_icra2012.pdf?utm_source=chatgpt.com)
- [Unity-Technologies/VHACD: The V-HACD library](https://github.com/Unity-Technologies/VHACD?utm_source=chatgpt.com)
- [A Survey of Optimization-based Task and Motion Planning](https://arxiv.org/abs/2404.02817?utm_source=chatgpt.com)
- [OctoMap - 3D occupancy mapping](https://octomap.github.io/?utm_source=chatgpt.com)

更多详细引用与延伸阅读见各 notebook 末尾。

## 贡献与反馈

欢迎提交 issue、PR 或建议，完善教程内容与代码实现。详见 [CONTRIBUTING.md](CONTRIBUTING.md)。

---

如需详细写作大纲、代码示例或特定章节的 markdown 模板，请在 issue 区留言或联系作者。

---

（本 README 参考了系列教程的结构与关键文献，便于后续扩展与引用。）
