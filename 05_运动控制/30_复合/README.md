# 复合方向（轮足·移动操作·足式操作·人形）

> **适用对象**：完成腿足基础、MPC/WBC、RL 部署与移动操作基础后，继续学习轮足、四足臂、人形与多机协作的工程师。
> **核心技术栈**：Pinocchio · OCS2 · IsaacLab · ros2_control · MuJoCo · ROS 2
> **目录状态**：编号文件 `10_` 到 `300_` 对应第 71-100 章，按当前目录中的实际 Markdown 文件维护。

## 阅读路径

1. **公共基础**：先读 10-50，建立复合机器人统一动力学、MPC、RL 与操作技能接口。
2. **D1 轮足方向**：继续读 60-110，重点是轮式非完整约束、轮足混合 MPC、端到端 RL 与硬件部署。
3. **D2 移动操作方向**：继续读 120-150，重点是底盘臂联合规划、OCS2 mobile manipulator、VLA 与数据采集。
4. **D3a 四足臂方向**：继续读 160-210，重点是四足+臂动力学、OCS2 NMPC、WBC、视觉全身控制与混合 MPC+RL。
5. **D3b 人形方向**：继续读 220-250，重点是经典人形 WBC、全身 RL、Sim-to-Real 与力敏感行走操作。
6. **跨方向收束**：最后读 260-300，连接基础模型、统一 Sim-to-Real、多机协作、统一闭环与博士规划。

## 章节导航

| 文件 | 章节 | 主题 |
|---|---:|---|
| [10_复合机器人全景.md](10_复合机器人全景.md) | 71 | 复合机器人全景 |
| [20_浮动基座臂统一动力学.md](20_浮动基座臂统一动力学.md) | 72 | 浮动基座+臂统一动力学 |
| [30_多模态MPC.md](30_多模态MPC.md) | 73 | 多模态 MPC |
| [40_RL全身控制基础.md](40_RL全身控制基础.md) | 74 | RL 全身控制基础 |
| [50_操作技能接口.md](50_操作技能接口.md) | 75 | 操作技能接口 |
| [60_轮式运动学与Pfaffian.md](60_轮式运动学与Pfaffian.md) | 76 | 轮式运动学与 Pfaffian 约束 |
| [70_轮足混合MPC.md](70_轮足混合MPC.md) | 77 | 轮足混合 MPC |
| [80_Wheel_Legged_Gym_RL.md](80_Wheel_Legged_Gym_RL.md) | 78 | Wheel-Legged-Gym 端到端 RL |
| [90_Swiss_Mile商业化.md](90_Swiss_Mile商业化.md) | 79 | Swiss-Mile 与 RIVR 商业化路线 |
| [100_模式切换.md](100_模式切换.md) | 80 | 轮 / 足 / 混合模式切换 |
| [110_轮足SimToReal与硬件.md](110_轮足SimToReal与硬件.md) | 81 | 轮足 Sim-to-Real 与硬件集成 |
| [120_底盘臂联合规划.md](120_底盘臂联合规划.md) | 82 | 底盘 + 臂联合规划 |
| [130_OCS2_mobile_manipulator.md](130_OCS2_mobile_manipulator.md) | 83 | OCS2 mobile_manipulator 精读 |
| [140_VLA移动操作.md](140_VLA移动操作.md) | 84 | VLA 驱动的移动操作 |
| [150_Mobile_ALOHA与UMI.md](150_Mobile_ALOHA与UMI.md) | 85 | Mobile ALOHA 与 UMI 数据采集 |
| [160_四足臂动力学概览.md](160_四足臂动力学概览.md) | 86 | 四足+臂动力学与控制概览 |
| [170_qm_control精读.md](170_qm_control精读.md) | 87 | qm_control 源码精读 |
| [180_Deep_WBC精读.md](180_Deep_WBC精读.md) | 88 | Deep Whole-Body Control |
| [190_Visual_WBC精读.md](190_Visual_WBC精读.md) | 89 | Visual Whole-Body Control |
| [200_UMI_on_Legs精读.md](200_UMI_on_Legs精读.md) | 90 | UMI on Legs |
| [210_RAMBO混合MPC_RL.md](210_RAMBO混合MPC_RL.md) | 91 | RAMBO 与混合 MPC+RL |
| [220_经典人形全身控制.md](220_经典人形全身控制.md) | 92 | 经典人形全身控制 |
| [230_人形全身RL.md](230_人形全身RL.md) | 93 | 人形全身 RL |
| [240_ASAP_SimToReal.md](240_ASAP_SimToReal.md) | 94 | ASAP 与人形 Sim-to-Real |
| [250_力敏感人形LocoMani.md](250_力敏感人形LocoMani.md) | 95 | 力敏感人形 Loco-Manipulation |
| [260_VLA_Foundation_Model.md](260_VLA_Foundation_Model.md) | 96 | VLA / Foundation Model |
| [270_SimToReal统一方法论.md](270_SimToReal统一方法论.md) | 97 | SimToReal 统一方法论 |
| [280_多机协作LocoMani.md](280_多机协作LocoMani.md) | 98 | 多机协作 Loco-Manipulation |
| [290_感知操作运动统一闭环.md](290_感知操作运动统一闭环.md) | 99 | 感知-操作-运动统一闭环 |
| [300_研究方向与博士规划.md](300_研究方向与博士规划.md) | 100 | 研究方向地图与博士规划 |

## 专题与调研

| 路径 | 类型 | 内容 |
|---|---|---|
| [动作模仿理论.md](动作模仿理论.md) | 专题补充 | 参考动作追踪、对抗运动先验、技能潜空间与动作重定向 |
| [调研/](调研/) | 调研资料 | 轮足、移动操作、四足臂、人形与课程规划的深度调研 |

## 收束关系

第 98-100 章是复合方向的收束段：`280_多机协作LocoMani.md` 处理多机任务分解、分布式协调和局部自治；`290_感知操作运动统一闭环.md` 把 SLAM 不确定性、操作策略、MPC/WBC 与安全过滤接成统一闭环；`300_研究方向与博士规划.md` 将前面章节转化为博士阶段的方向评估、五年路线和最小闭环计划。
