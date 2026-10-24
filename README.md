# AV Lane Change Simulator

面向自动驾驶换道能力理论研究的二维科研仿真软件，以 MATLAB 为主开发环境。

## 当前范围：Phase 0 / P0.2.1

当前仅建立项目目录、`avlc` package、最小函数占位和基础结构检查脚本。
尚未实现车辆数据模型、Scenario、动力学、数值积分、仿真循环、间隙计算、
可视化或数据读写。本任务不推进至 P0.2.2，不引入外部依赖。

Phase 0 固定研究两条直线车道上的四辆车：

| 角色 | 含义 |
| --- | --- |
| EGO | 自车 |
| FO | 原车道前车 |
| RO | 原车道后车 |
| RT | 目标车道后车 |

后续统一采用 SI 单位，以车辆几何中心为位置参考点，六状态接口为
`x, y, vx, vy, ax, ay`。这些数据模型尚未在本步骤实现。

## 包结构与职责

```text
AVLaneChangeSimulator/
├── README.md
├── src/
│   └── +avlc/
│       ├── +core/             # 仿真引擎（当前仅入口占位）
│       ├── +model/            # 车辆与状态数据模型（后续实现）
│       ├── +dynamics/         # 动力学（后续实现）
│       ├── +scenario/         # 场景描述与验证（后续实现）
│       │   └── presets/      # 场景预设预留目录
│       ├── +analysis/        # 分析与理论接口（当前仅能力接口占位）
│       ├── +result/          # SimulationResult（后续实现）
│       ├── +visualization/   # 结果绘图与回放（后续实现）
│       └── +io/              # 保存与加载（后续实现）
├── app/                     # GUI 预留目录
├── scenarios/
├── tests/
├── examples/
│   └── check_project_structure.m
├── data/
└── docs/
    ├── requirements/
    └── architecture/
```

所有 MATLAB 包统一使用命名空间 `avlc`。后续处理流程遵循：

```text
Scenario → Dynamics → Simulation Engine → Analysis → SimulationResult
                                                        ↓
                                                 Visualization / IO
```

各层保持单向依赖，避免循环依赖。数值核心独立于 GUI；可视化只读取结果，
不推进车辆状态；分析不得修改历史结果。GUI 位于系统最外层，
不属于当前开发步骤，`app/` 仅为空目录。

CapabilityEvaluator 是独立的未来理论模块，当前通过
`avlc.analysis.evaluateCapability` 预留接口，**尚未实现**。
当前不定义或计算 `K_T`、`K_O_abort`、commitment boundary 或 capability margin，
也不允许该模块反向控制基础仿真架构。

## MATLAB 路径与验证

在 MATLAB 中将当前目录切换至 `AVLaneChangeSimulator`，执行：

```matlab
addpath("src")
run("examples/check_project_structure.m")
```

只需将 `src` 加入 MATLAB path，无须递归加入 `+avlc` 或其子包目录。
检查脚本也支持从其他当前目录运行：它根据脚本位置定位项目根目录，
临时添加 `src`，并在检查结束时恢复调用前的 path。

脚本检查目录、必要文件、两个函数的解析路径及预期错误标识。
全部通过时输出 `P0.2.1 project structure verification passed.`；
任何检查失败都会抛出错误。此检查只验证项目骨架，不验证仿真能力。

后续核心调用形式为：

```matlab
result = avlc.core.runSimulation(scenario, config);
```

当前 `runSimulation` 和 `evaluateCapability` 均无计算逻辑，调用时分别
抛出 `AVLC:NotImplemented` 和 `AVLC:CapabilityNotImplemented`，以明确提示功能尚未实现。
