# AV Lane Change Simulator

面向自动驾驶换道能力理论研究的二维科研仿真软件，以 MATLAB 为主开发环境。

## 当前范围：Phase 0 / P0.2.7

已实现基于普通 MATLAB struct 的 VehicleState、Vehicle、Road 和
SimulationConfig，提供创建器、验证器和 MATLAB 单元测试。
理论到软件映射已冻结，现已实现 TheoryModelParameters 契约和带跨对象验证的
四车 Scenario 契约；结构有效不表示交通安全或理论可行。
现已实现 Control 数据契约、精确匀速单步原语和 Ego 双积分器单步原语，
后者检查冻结的加速度控制边界。
现已实现最小固定步长四车 Simulation Engine，支持恒定 Ego 控制、同步推进和
确定性的轨迹结果记录（N 个控制区间、N+1 个状态样本）。
现已实现 Theory State Builder，将当前四车物理状态映射为八维约化理论状态
`(y,w,gR,cR,gF,cF,gT,cT)`，包括几何和安全净距到理论间隙的映射。
该映射不判断安全性或能力，允许负间隙和负横向速度。
现已实现冻结 Parts I–III 的当前约化撤销/完成执行能力评估，以及有限机会分支的
联合能力几何。`dualCapable` 只表示当前两个执行选项共存。
尚未实现正时间双选项保持、储备动态、共享控制兼容性、fixed-T 求解、
机动自由时域、安全分析、轨迹规划、事件执行、轨迹级理论评估、可视化、GUI
或数据读写。本任务停在 P0.2.7，不引入外部依赖。

Phase 0 固定研究两条直线车道上的四辆车：

| 角色 | 含义 |
| --- | --- |
| EGO | 自车 |
| FO | 原车道前车 |
| RO | 原车道后车 |
| RT | 目标车道后车 |

后续统一采用 SI 单位，以车辆几何中心为位置参考点，六状态接口为
`x, y, vx, vy, ax, ay`。这些字段由本阶段的数据模型提供。

## 包结构与职责

```text
AVLaneChangeSimulator/
├── README.md
├── src/
│   └── +avlc/
│       ├── +core/             # 最小固定步长四车仿真引擎（已实现）
│       ├── +model/            # 基础数据模型、理论参数及其创建器和验证器（已实现）
│       ├── +dynamics/         # 匀速与 Ego 双积分器单步原语（已实现）
│       ├── +scenario/         # 四车场景描述与结构验证（已实现）
│       │   └── presets/      # 场景预设预留目录
│       ├── +analysis/        # 单状态理论映射与当前约化执行能力评估（已实现）
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

CapabilityEvaluator 通过 `avlc.analysis.evaluateCapability(scenario, z)`
评估当前约化撤销、完成及双选项执行能力，独立于仿真引擎。
其定义域要求 `y >= 0, w >= 0`，负间隙只禁用相应分支。
有限几何中的 `Gamma` 只选择前向边界，不判断动态选项消失顺序。
该模块不反向控制基础仿真架构，也不计算机动自由时域。

## MATLAB 路径与验证

在 MATLAB 中将当前目录切换至 `AVLaneChangeSimulator`，执行：

```matlab
addpath("src")
run("examples/check_project_structure.m")
results = runtests("tests");
assert(~isempty(results) && all([results.Passed]), "Data-model tests did not all pass.")
```

只需将 `src` 加入 MATLAB path，无须递归加入 `+avlc` 或其子包目录。
检查脚本也支持从其他当前目录运行：它根据脚本位置定位项目根目录，
临时添加 `src`，并在检查结束时恢复调用前的 path。

脚本检查目录、必要文件、包路径、最小引擎运行及当前能力评估接口。
全部通过时输出 `Project structure verification passed (P0.2.7).`；
任何检查失败都会抛出错误。此检查验证结构和最小运行，数值正确性由单元测试验证。

核心调用形式为：

```matlab
result = avlc.core.runSimulation(scenario, simulationConfig, egoControl);
```

`runSimulation` 返回 time、四车 states、EGO controls 和确定性 metadata。
网格要求 endTime/dt 在容差内为正整数，不使用缩短末步。
能力评估通过独立接口显式调用，尚未自动接入 `runSimulation`。

单个时刻的理论状态映射接口为：

```matlab
z = avlc.analysis.buildTheoryState(scenario, egoState, foState, roState, rtState);
```

当前状态按参数顺序绑定四车角色，不要求等于 Scenario 初态。加速度快照及
周车横向状态不参与映射；`W` 仍由 Road 派生，不存入 `z`。

```matlab
cap = avlc.analysis.evaluateCapability(scenario, z);
```

结果包含 `abort`、`completion` 和 `joint`。无限机会使用 `Inf`；当前分支未定义的
有限边界/窗口使用 `NaN`，具体约定见 P0.2.7 契约。

## Design documentation

以下版本化文档是后续开发的规范依据，以仓库中的版本为准：

- [Phase 0 Software Requirements v0.1](docs/requirements/phase0_requirements_v0.1.md)
- [P0.2 Software Architecture v0.1](docs/architecture/p0.2_architecture_v0.1.md)
- [P0.2.2 Data Model Contract v0.1](docs/architecture/p0.2.2_data_model_contract_v0.1.md)
- [P0.2.2b Theory-to-Software Contract v0.1](docs/architecture/p0.2.2b_theory_to_software_contract_v0.1.md)
- [P0.2.3 Scenario Contract v0.1](docs/architecture/p0.2.3_scenario_contract_v0.1.md)
- [P0.2.4 Dynamics Contract v0.1](docs/architecture/p0.2.4_dynamics_contract_v0.1.md)
- [P0.2.5 Simulation Engine Contract v0.1](docs/architecture/p0.2.5_simulation_engine_contract_v0.1.md)
- [P0.2.6 Theory State Builder Contract v0.1](docs/architecture/p0.2.6_theory_state_builder_contract_v0.1.md)
- [P0.2.7 Current Capability Contract v0.1](docs/architecture/p0.2.7_current_capability_contract_v0.1.md)
