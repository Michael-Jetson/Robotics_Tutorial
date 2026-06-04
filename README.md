# Robotics Tutorial

[![License: CC BY 4.0](https://img.shields.io/badge/License-CC%20BY%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by/4.0/)

面向机器人与人工智能交叉方向的系统化教学文档，覆盖从数学基础到具身智能的完整知识体系。

> **声明：** 本项目大量使用 AI（Claude、Codex 等）结合互联网调研来撰写内容。尽管所有文档均经过多轮独立审核与事实核查，但仍然无法完全避免错误。如果您在阅读过程中发现任何事实性错误、论文引用偏差或技术描述不当，欢迎通过 [Issue](https://github.com/Michael-Jetson/Robotics_Tutorial/issues) 反馈，我们会及时修正。

**Author:** Pengfei Guo  
**Affiliation:** 达妙科技 (DAMIAO Technology)  
**License:** [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) — 自由使用与商用，需注明出处  
**AI Collaboration:** 内容由 Claude 与 Codex 辅助编写，经多轮双盲审核与基于互联网调研的事实核查

---

## 项目结构

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

## 内容概览

| 方向 | 文件数 | 总行数 | 状态 |
|------|------:|------:|------|
| 01 数学基础 | 90 | 159,718 | **完工（全 10 个子方向审核闭环）** |
| 02 C++基础与进阶 | 58 | 115,733 | **完工（41 篇教学正文双审闭环）** |
| 03 SLAM | 14 | 14,642 | **完工（5 篇教学正文审核闭环）** |
| 04 移动机器人规控 | 84 | 145,154 | **完工（综述+采样 MPC 审核闭环）** |
| 05 运动控制 | 152 | 231,320 | **完工（全部 67 篇教学正文双审闭环）** |
| 06 具身智能 | 30 | 69,453 | **完工（审核闭环）** |
| **合计** | **428** | **736,020** | **全方向完工** |

### 05 运动控制 明细

| 子方向 | 文件数 | 总行数 | 状态 |
|--------|------:|------:|------|
| 公共基础 | 1 | 406 | **完工** |
| 足式（腿足机器人） | 27 | 52,204 | **完工（12 篇教学正文双审闭环）** |
| 机械臂 | 51 | 79,963 | **完工（37 篇教学正文双审闭环）** |
| 复合（移动操作） | 48 | 67,302 | **完工（17 篇教学正文双审闭环）** |
| 仿真 | 10 | 20,346 | **完工（审核闭环）** |
| 总大纲 | 1 | 2,172 | **完工** |

## 足式机器人章节序列

完整 24 章教学序列：

| 篇章 | 章节 | 主题 |
|------|------|------|
| 基础篇 | Ch47-50 | Pinocchio、CppAD、空间代数、QP/NLP |
| 模型与接触 | Ch51-52 | 简化模型、接触力学 |
| 控制篇 | Ch53-55 | WBC/TSID、DDP/Crocoddyl、OCS2 MPC |
| 规划篇 | Ch56-60 | 步态、状态估计、落脚规划 |
| 工程篇 | Ch61-62 | 实时 C++、硬件栈 |
| RL 与感知 | Ch63-67 | RL 训练/部署、RL+MPC 混合、感知 |
| 综合篇 | Ch68-70 | 代码精读、综合项目、研究方向 |

## 质量标准

全部 193 篇教学正文遵循 v5.0 内部编写规范，涵盖四种文档类型（理论教学/算法工程教学/论文解读/工程实践）。每篇均经过至少两轮独立审核，审核意见归档于 `00_项目导航/审核意见/`。

- 每章最低 2,000 行（工程实践类 1,500 行）
- 15 条规则，横跨 3 个层次（内容层/表达层/认知层）
- 5 层质量门禁（G1 结构 → G2 准确 → G3 一致 → G4 完整 → G5 润色）
- 双轮独立审核流水线：R1 审核 → 修复 → R2 验证 → PASS
- 认知工具：每章 ≥3 个类比、≥3 个反事实推理、≥2 个本质洞察
- 完整推导，逐步推理，禁止"显然""容易看出"等跳步表述
- 故障排查手册、跨章桥接、累积项目
- 基于互联网调研的事实验证（论文引用、API 名称、数值声明）

## 使用方法

```bash
git clone https://github.com/Michael-Jetson/Robotics_Tutorial.git
```

所有文档为独立 Markdown 文件，建议按各方向内的章节编号顺序阅读。

## 引用

```
Robotics Tutorial by Pengfei Guo, DAMIAO Technology
https://github.com/Michael-Jetson/Robotics_Tutorial
Licensed under CC BY 4.0
```

## 许可协议

本项目采用 [CC BY 4.0 国际许可协议](https://creativecommons.org/licenses/by/4.0/)。

你可以自由地：
- **分享** — 以任何媒介或格式复制、转载
- **改编** — 以任何目的（包括商业目的）进行混合、转换、再创作

但须遵守以下条件：
- **署名** — 必须注明出处、提供许可协议链接，并说明是否进行了修改。

## Star History

<a href="https://www.star-history.com/?repos=Michael-Jetson%2FRobotics_Tutorial&type=date&legend=top-left">
 <picture>
   <source media="(prefers-color-scheme: dark)" srcset="https://api.star-history.com/chart?repos=Michael-Jetson/Robotics_Tutorial&type=date&theme=dark&legend=top-left" />
   <source media="(prefers-color-scheme: light)" srcset="https://api.star-history.com/chart?repos=Michael-Jetson/Robotics_Tutorial&type=date&legend=top-left" />
   <img alt="Star History Chart" src="https://api.star-history.com/chart?repos=Michael-Jetson/Robotics_Tutorial&type=date&legend=top-left" />
 </picture>
</a>
