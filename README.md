# Robotics Tutorial

[![License: CC BY 4.0](https://img.shields.io/badge/License-CC%20BY%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by/4.0/)

面向机器人与人工智能交叉方向的系统化教学文档，覆盖从数学基础到具身智能的完整知识体系。

**Author:** Pengfei Guo  
**Affiliation:** 达妙科技 (DAMIAO Technology)  
**License:** [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) — 自由使用与商用，需注明出处  
**AI Collaboration:** 部分内容由 Claude 与 Codex 辅助编写与审核

---

## Project Structure

```
Robotics_Tutorial/
├── 01_数学/                 # 数学基础（微分几何、李群、优化、控制、概率、刚体动力学、RL数学）
├── 02_C++基础与进阶/        # C++ 工程（语言核心、并发、软件工程、通用库、ROS2、规控工程基础）
├── 03_SLAM/                 # 同时定位与建图
├── 04_移动机器人规控/        # 移动机器人规划与控制（无人机、横切专题）
├── 05_运动控制/              # 运动控制（公共基础/足式/机械臂/复合/仿真）
│   ├── 00_公共基础/          #   运动控制公共能力层
│   ├── 10_足式/              #   足式机器人（24章完整教学序列）
│   ├── 20_机械臂/            #   机械臂
│   ├── 30_复合/              #   复合方向（移动操作）
│   └── 40_仿真/              #   仿真环境与工具
└── 06_具身智能/              # 具身智能（RL运控、VLA、世界模型）
```

## Content Overview

| Direction | Files | Lines | Status |
|-----------|------:|------:|--------|
| 01 数学基础 | 90 | 124,596 | 6/10 sub-directions complete (QA'd) |
| 02 C++基础与进阶 | 58 | 115,782 | 4/6 sub-directions complete |
| 03 SLAM | 15 | 16,191 | In Progress |
| 04 移动机器人规控 | 76 | 15,613 | Skeleton |
| 05 运动控制 | 134 | 190,726 | **Complete** |
| 06 具身智能 | 33 | 70,892 | In Progress |
| **Total** | **406** | **533,800** | |

### 05 运动控制 Breakdown

| Sub-direction | Files | Lines | Status |
|---------------|------:|------:|--------|
| 公共基础 (Common Foundation) | 1 | 406 | **Complete** |
| 足式 (Legged) | 27 | 44,985 | **Complete** |
| 机械臂 (Manipulator) | 47 | 67,085 | **Complete** |
| 复合 (Loco-manipulation) | 48 | 56,073 | **Complete** |
| 仿真 (Simulation) | 10 | 20,005 | **Complete** |
| 总大纲 (Overview) | 1 | 2,172 | **Complete** |

## Legged Robot Chapters (足式)

Complete 24-chapter teaching sequence covering:

| Part | Chapters | Topic |
|------|----------|-------|
| Foundation | Ch47-50 | Pinocchio, CppAD, spatial algebra, QP/NLP |
| Models & Contact | Ch51-52 | Simplified models, contact mechanics |
| Control | Ch53-55 | WBC/TSID, DDP/Crocoddyl, OCS2 MPC |
| Planning | Ch56-60 | Gait, state estimation, foothold planning |
| Engineering | Ch61-62 | Real-time C++, hardware stack |
| RL & Perception | Ch63-67 | RL training/deploy, hybrid RL+MPC, perception |
| Capstone | Ch68-70 | Code reading, project, research directions |

## Quality Standards

All teaching chapters follow a v5.0 internal spec with four document types (theory / algorithm-engineering / paper-reading / engineering-practice):

- Minimum 2,000 lines per chapter (1,500 for engineering-practice)
- 15 rules across 3 layers (content / expression / cognition)
- 5-gate quality system (G1 structure → G2 accuracy → G3 consistency → G4 completeness → G5 polish)
- Cognitive tools: ≥3 analogies, ≥3 counterfactuals, ≥2 essence insights per chapter
- Full derivations with step-by-step reasoning, no "obvious" or "trivially"
- Fault diagnosis tables, cross-chapter bridges, cumulative projects

## Usage

```bash
git clone https://github.com/Michael-Jetson/Robotics_Tutorial.git
```

Documents are standalone Markdown files. Recommended reading order follows chapter numbering within each direction.

## Citation

If you use this material in your work, please cite:

```
Robotics Tutorial by Pengfei Guo, DAMIAO Technology
https://github.com/Michael-Jetson/Robotics_Tutorial
Licensed under CC BY 4.0
```

## License

This work is licensed under a [Creative Commons Attribution 4.0 International License](https://creativecommons.org/licenses/by/4.0/).

You are free to:
- **Share** — copy and redistribute in any medium or format
- **Adapt** — remix, transform, and build upon for any purpose, including commercial

Under the following terms:
- **Attribution** — You must give appropriate credit, provide a link to the license, and indicate if changes were made.

## Star History

<a href="https://www.star-history.com/?repos=Michael-Jetson%2FRobotics_Tutorial&type=date&legend=top-left">
 <picture>
   <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/chart?repos=Michael-Jetson/Robotics_Tutorial&type=date&theme=dark&legend=top-left" />
   <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/chart?repos=Michael-Jetson/Robotics_Tutorial&type=date&legend=top-left" />
   <img alt="Star History Chart" src="https://api.star-history.com/chart?repos=Michael-Jetson/Robotics_Tutorial&type=date&legend=top-left" />
 </picture>
</a>
