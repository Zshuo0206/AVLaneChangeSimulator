# AV Lane Change Simulator

面向自动驾驶换道能力理论研究的二维科研仿真软件，以 MATLAB 为主开发环境。

## 当前基线：P0.2.1–P0.3.1 / Research Visualization V0

P0.2.1–P0.2.9、Research Visualization V0 为 FROZEN / PASS；
P0.3.0 mapping 为 REVIEWED / PASS；P0.3.1 Exact Global Foundations
为 FROZEN / PASS after focused independent audit。

P0.3.2a supplied longitudinal witness verifier 为
FROZEN / PASS after focused independent audit。

P0.3.2b longitudinal inner witness searcher 为
FROZEN / PASS after focused independent audit。

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
现已增加独立的 signed reserve 与 active-set 诊断：保留 R/F/G ties，
分别报告当前完成时间分支和有限上边界机制，继承未定义有限量的 NaN 语义。
现已增加薄轨迹分析 adapter 和普通 MATLAB research summary figure，
沿全部 N+1 个状态样本展示车辆运动、当前能力、有限边界、signed reserves
及最终机制，不改变冻结理论。
尚未实现全局双选项保持搜索、储备动态、fixed-T 求解、
机动自由时域、安全分析、轨迹规划、事件执行、动画、GUI 或数据读写。
P0.2.9 只新增局部兼容性后端，不扩展 Research Visualization V0，不引入外部依赖。

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
│       ├── +analysis/        # 理论映射、当前能力、诊断及薄轨迹分析 adapter
│       ├── +result/          # SimulationResult（后续实现）
│       ├── +visualization/   # 静态 research summary figure（已实现）
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
全部通过时输出 `Project structure verification passed (Research Visualization V0).`；
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

在同一状态下消费已有结果，不重复评估能力：

```matlab
diagnostics = avlc.analysis.evaluateCapabilityDiagnostics(z.y, cap);
```

`diagnostics.reserves` 提供 signed `rO/lT/rT`；`abort` 保留所有有限 active
mechanisms，`completion` 分开报告当前时间分支与上边界 Regime I/II。
这些信息不判断共享控制兼容性或正时间双选项保持。P0.2.7 输出保持不变。

## Exact Global Foundations — P0.3.1

```matlab
membership = avlc.analysis.evaluateGlobalPreservationMembership(scenario,z);
reach = avlc.analysis.evaluateLateralEndpointReachability(scenario,z0,T,rho);
```

这两个函数是 exact analytical foundation evaluators，分别返回当前 closed
global preservation-set membership 和给定 `(T,rho)` 的 exact lateral
reachable interval；它们不是 fixed-horizon optimizer 或 maneuver-freedom
horizon solver。集合/分配区间使用闭边界，不依赖有限 frontier 或 reserve。
输入错误与负状态、超界 rho 的定义域结果分开，详见 P0.3.1 契约。

## Longitudinal Safe-Prefix Witness Verification — P0.3.2a

```matlab
witness = struct('times',[0,0.5,1.5,2],'ax',[-2,2,0]);
certificate = avlc.analysis.verifyLongitudinalSafePrefixWitness(scenario,z0,2,witness);
```

它解析检查给定分段恒定纵向控制的整个 prefix，包括段内最小 gap。
成功 certificate 证明该 supplied control whole-prefix safe；失败 candidate
不证明不存在其他安全纵向控制。控制值须符合当前 Scenario 的 `[-B,C]`；
本函数不搜索控制，也不判断 fixed-T 共享能力保持或机动自由时域。

## Longitudinal Inner Witness Search — P0.3.2b

```matlab
searchResult = avlc.analysis.searchLongitudinalSafePrefixWitness(scenario,z0,T, ...
    struct('numSegments',8,'maxCutIterations',20));
```

正时间搜索依赖 Optimization Toolbox 的 `linprog`，只生成固定 N 的 uniform
piecewise-constant candidates，再由冻结 P0.3.2a verifier 决定是否安全。
`WITNESS_FOUND` 必须附带 valid certificate；`UNRESOLVED` 不证明不存在其他
安全控制，也不判断 fixed-T 双选项保持。缺少 solver 时返回
`UNRESOLVED / SOLVER_UNAVAILABLE`，
不使用 fallback optimizer；T=0 直接调用 verifier。

## Research Visualization V0

```matlab
result = avlc.core.runSimulation(scenario, simulationConfig, egoControl);
analysis = avlc.analysis.analyzeTrajectory(scenario, result);
fig = avlc.visualization.plotResearchSummary(scenario, result, analysis);
```

一条命令运行完整示例：

```matlab
run("examples/run_research_visualization_v0.m")
```

左侧显示真实四车轨迹和最终车框；右侧显示能力布尔历史、y 与有限边界、
三个 signed reserves。NaN 直接断线，负 reserve 保留。所有历史为 N+1 列向量，
与初态到末态逐点对应；轨迹须处于冻结评估器的 `y >= 0, w >= 0` 定义域。
示例输出 `researchScenario/researchResult/researchAnalysis/researchFigure`，
不自动保存文件。分析和绘图分层，绘图函数不重新评估理论。

## Design documentation

P0.2.9 提供独立的、定理定义域内的局部共享控制兼容性评估，
状态为 `FROZEN / PASS after focused independent audit`：

```matlab
compatibility = avlc.analysis.evaluateLocalCompatibility(scenario, z, cap, diagnostics);
```

它不是 controller 或换道决策策略；`UNDEFINED` 与已证明不兼容严格区分。
详见 [P0.2.9 contract](docs/architecture/p0.2.9_local_compatibility_contract_v0.1.md)。

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
- [P0.2.8 Reserve Diagnostics Contract v0.1](docs/architecture/p0.2.8_reserve_diagnostics_contract_v0.1.md)
- [Research Visualization V0 Contract](docs/architecture/research_visualization_v0_contract.md)
- [P0.2.9 Local Compatibility Contract v0.1](docs/architecture/p0.2.9_local_compatibility_contract_v0.1.md)
- [P0.3.0 Global Preservation / Fixed-T Interface Extraction v0.1](docs/architecture/p0.3.0_global_preservation_fixed_t_interface_extraction_v0.1.md) — REVIEWED / PASS，设计报告。
- [P0.3.1 Exact Global Foundations Contract v0.1](docs/architecture/p0.3.1_exact_global_foundations_contract_v0.1.md) — FROZEN / PASS after focused independent audit。
- [P0.3.2a Longitudinal Safe-Prefix Witness Contract v0.1](docs/architecture/p0.3.2a_longitudinal_safe_prefix_witness_contract_v0.1.md) — FROZEN / PASS after focused independent audit。
- [P0.3.2b Longitudinal Inner Witness Search Contract v0.1](docs/architecture/p0.3.2b_longitudinal_inner_witness_search_contract_v0.1.md) — FROZEN / PASS after focused independent audit。
