# Robotics Tutorial

[![License: CC BY 4.0](https://img.shields.io/badge/License-CC%20BY%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by/4.0/)

面向机器人学研究者与工程师的系统化教学文档，覆盖从数学基础到具身智能的完整知识体系。

**Author:** Pengfei Guo  
**Affiliation:** 达妙科技 (DAMIAO Technology)  
**License:** [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) — 自由使用与商用，需注明出处  
**AI Collaboration:** 部分内容由 Claude (Anthropic) 辅助编写

---

## Project Structure

```
Robotics_Tutorial/
├── 01_数学/                 # 数学基础（流形、李群、凸优化）
├── 02_基础/                 # 编程基础（C++ 进阶、核心库剖析）
├── 03_SLAM/                 # 同时定位与建图
├── 04_移动机器人规控/        # 移动机器人规划与控制
├── 05_运动控制/              # 运动控制（足式/机械臂/复合/仿真）
│   ├── 足式/                #   足式机器人 — 24 章 + 2 序章 + 大纲
│   ├── 机械臂/              #   机械臂 — 37 章 + 大纲
│   ├── 复合/                #   复合方向（移动操作）
│   └── 仿真/                #   仿真环境与工具
└── 06_具身智能/              # 具身智能与大模型
```

## Content Overview

| Direction | Files | Lines | Status |
|-----------|------:|------:|--------|
| 01 数学基础 | 93 | 55,374 | In Progress |
| 02 编程基础 | 52 | 14,209 | In Progress |
| 03 SLAM | 25 | 35,291 | In Progress |
| 04 移动机器人规控 | 76 | 15,558 | In Progress |
| 05 运动控制 | 138 | 123,951 | **足式/机械臂 Complete** |
| 06 具身智能 | 5 | 2,444 | Early Stage |
| **Total** | **389** | **246,827** | |

### 05 运动控制 Breakdown

| Sub-direction | Files | Lines | Status |
|---------------|------:|------:|--------|
| 足式 (Legged) | 27 | 41,914 | **Complete** |
| 机械臂 (Manipulator) | 47 | 63,053 | **Complete** |
| 复合 (Loco-manipulation) | 54 | 10,686 | In Progress |
| 仿真 (Simulation) | 9 | 6,305 | In Progress |

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

All teaching chapters follow [v3.2 spec](00_项目导航/教学文档编写规范.md):

- Minimum 1,000 lines, target 1,500-2,000 per chapter
- Cognitive tools: ≥3 analogies, ≥3 counterfactuals, ≥2 essence insights
- Fault diagnosis tables for common pitfalls
- Cross-chapter forward/backward bridges
- 6:4 text-to-code ratio with full derivations
- Reviewed via Codex gpt-5.5 + internet-verified fixes

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
