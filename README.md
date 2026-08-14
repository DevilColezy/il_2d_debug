# il_2d_multiscale_debug

完全二维、确定性、可逐步推进的 **5Hz 特权宏观专家 + 30Hz 局部避障专家** 协同行为调试包。

这是一个**纯 2D、独立、轻量**的专家行为调试器：**不调用 Flightmare / Unity**，**不写训练数据**，只用于在固定步长二维仿真中观察、步进和调试两层专家的协作、状态机、左右绕行决策与信息边界。

> **状态：本包仅为代码与配置，尚未编译、尚未运行、尚未测试。**
> 作者没有执行任何构建 / 运行 / 测试命令，也不保证一次通过编译。请自行在 catkin 工作空间中编译后再使用。

---

## 1. 包的目的

在真实(三维、带深度传感器、Flightmare)系统里，5Hz 宏观专家与 30Hz 局部专家之间的时序、信息边界、状态机切换和“左右绕行决策”很难单独观察。本包把整个系统压到二维平面上，用确定性仿真还原同样的层级结构：

```
Navigation goal
      ↓
5Hz Macro Expert（必要时使用全局特权 ESDF，输出连续变化的局部目标）
      ↓ 5Hz ZOH
Current local target
      ↓
30Hz Local Expert（只能使用当前状态、局部观测、当前局部目标）
      ↓
body velocity + yaw rate
      ↓
2D deterministic simulator（dt = 1/30 s，swept 碰撞检查）
```

所有随机过程由显式 `uint64_t seed` 驱动：**同一个 seed 产生完全相同的场景、起终点与初始偏航**。

## 2. 5Hz / 30Hz 架构

- **30Hz 局部专家**（`LocalPlanner30Hz`）：每个 30Hz tick 重新规划短时候选轨迹，跟踪“当前局部目标”。
- **5Hz 宏观专家**（`MacroExpert5Hz`）：仅在 `tick % 6 == 0` 时精确执行一次（**没有两个互相漂移的 wall timer**，只有一个 30Hz 驱动定时器，5Hz 逻辑在 tick 计数内完成）。
- **零阶保持（ZOH）**：5Hz 每 6 个 30Hz tick 更新一次局部目标，两个更新之间目标保持不变。
- **边界内即时入宏观**：若连续局部失败恰好在某个 5Hz 边界达到阈值，则**同一个边界内**立即完成选边与宏观路线生成（宏观专家每个边界最多执行一次），不会多等一个 5Hz 周期。
- **tick 语义**：`tick` = 已完成 30Hz 步数（仿真时间 = tick/30）；`processing_tick` = 本次快照所描述的正在/刚被处理的 tick。两者同时在 `DebugSnapshot.msg` 与 GUI 中发布。
- **Step To Next 5Hz Tick** 以 `macro_tick_event` 的**严格递增**为停止条件（至少先走一个 30Hz tick），保证停在“该次 5Hz 决策已经执行完成”之后，而不是停在边界决策执行之前。
- **单步是原子暂停操作**：`StepSimulation` 服务进入后先停止自动推进（mutex 内 wall timer 无法插入），执行完指定步数后**无条件保持暂停**（不恢复调用前的 Run 状态）；`steps==0 && step_to_next_5hz==false` 返回失败“nothing requested”。
- **层次状态机**（`HierarchicalExpertFsm`）在两者之上统一调度。

## 3. 信息边界（信息泄漏检查的核心）

### 30Hz 局部专家只能收到：

- `VehicleState2D`（位置、yaw、世界系速度、yaw rate）
- `LocalObservation`（世界系栅格：FREE / OCCUPIED / UNKNOWN，带参数化短时历史）
- `LocalTarget`（仅：局部目标位置 + `update_event`（数值更新事件，仅用于日志/可视化）+ `mission_revision`（**只有该值变化才重置规划器记忆**，见 §14）+ `is_macro_guide`（**只用于日志/可视化/上层管理，v7 起不影响任何 30 Hz 运动策略**））

### 30Hz 局部专家**收不到**：

- 全局 ESDF、障碍物真值、完整全局路径、未观测区域、左右路线总代价、未来多个局部目标、任何特权字段。

接口层面强制隔离：

```cpp
PlannerResult LocalPlanner30Hz::plan(
    const VehicleState2D& state,
    const LocalObservation& obs,
    const LocalTarget& target);
```

该类的 `plan` / `previewPlan` 接口根本没有全局 ESDF / 场景 / 障碍物的形参，**结构上无法访问**。审计字段 `used_truth_by_local_planner`、`used_global_esdf_by_local_planner`、`used_global_path_by_local_planner` 恒为 `false`，并在 GUI 中显示，用于人工检查泄漏。

**瞬时帧 vs 历史地图（信息边界）**：每个 30Hz tick 的 `FovRaycaster2D` 输出称为 **current_patch**（本 tick 的瞬时 FOV 帧，合并前），`ObservedGrid2D` 合并后的持久地图称为 **local_history_map**。FSM 的 `FsmInput` **同时携带两者**：
- 5Hz 宏观专家的 `identifyBlocker` 与 `selectSideFromVisibleEvidence` **只能读取 current_patch**（证据只反映本帧可见内容，绝不混入历史合并信息）；
- 30Hz 局部规划器（`plan` / `previewPlan` / `canBrakeSafely`）**只能读取 local_history_map**；
- 审计字段 `side_selected_using_current_patch`（每当选边计算运行时为 true）证明选边使用的是 current_patch；GUI 单独订阅并叠加显示瞬时 patch（**橙色**）与历史地图（绿/红/灰），语义清晰可辨。

### 5Hz 宏观专家的特权：

5Hz 专家可以读取全局 ESDF、障碍物真值和最终导航目标，用于：
1. 生成/持续刷新选定侧的完整安全路线；
2. 左右路线的可视化（人类调试）。

但**左右选择本身只能由“当前局部观测”的可见前缀证据决定**（见 §6）。特权信息不得用于偷偷切换“全局更短的一侧”。

### 观测合成说明：

局部观测由 `FovRaycaster2D` 用**二维射线投射**对真实圆柱表面求交生成：射线命中第一个障碍物表面后，后方区域保持 UNKNOWN；FOV 外保持 UNKNOWN；**不**允许因为知道圆心半径就把整个圆填入局部地图。真值在这里只用于“模拟传感器测量过程”，并非把特权信息交给 30Hz 规划器。碰撞真值仅供仿真执行器、安全裁判、5Hz 专家和人类可视化使用，绝不反馈给 30Hz 规划器。

**统一全局栅格基准（起步死锁根因）**：瞬时 patch 与历史地图使用**同一个全局栅格**——patch 原点被吸附到 `scene.min_bounds + n*resolution` 的整数栅格边界（`worldToGrid`：`ix = floor((world − min_bounds)/resolution)`；`gridCellCenter`：`min_bounds + (ix+0.5)*res`，全包唯一约定，放在 `types.hpp` 供 FovRaycaster2D / ObservedGrid2D / LocalObservation 共用）。合并时根据两个对齐原点一次性计算整数格 offset，随后直接使用 `global_index = offset + patch_index`，不再逐格执行“cell center 浮点坐标→floor”而重新引入边界舍入。旧实现用 `state.position − range` 作 patch 原点导致 1 格错位（起点 (-13.25,-9.55) 查询格 (67,104) 却写入 (68,105)）；修复后 patch 与历史地图严格写入同一全局格。patch 覆盖范围至少包含车辆为中心半径 `obs_range_m` 的圆盘；raycast 完成后**显式把传感器原点所在格标记为 FREE**（只标记那一格，不碰邻域、不穿透遮挡；碰撞仍由真值 referee 处理）。

## 4. 状态机

| 状态 | 含义 |
|---|---|
| `DIRECT_LOCAL` | 默认：30Hz 专家直接以最终导航目标为当前局部目标 |
| `TURN_TO_TARGET` | 目标方向超出 FOV，停止平移、只转 yaw（带滞回） |
| `LOCAL_BLOCKED_PENDING` | 已观测到真实阻挡，连续失败计时中（0.3~0.5s） |
| `MACRO_SELECT_SIDE` | 5Hz 边界上进行左右选择（可见证据） |
| `MACRO_GUIDANCE` | 宏观引导中：锁定侧路线 + 2.5~4.0m 名义 lookahead；实际局部目标截到“当前位置直连仍满足 handoff 净空”的最远路线点，避免目标落在 blocker 后方而直线切障碍 |
| `MACRO_EXIT_PENDING` | 满足退出条件计数中（连续 N 个 5Hz tick） |
| `GOAL_REACHED` / `TASK_INVALID` / `COLLISION` / `TIMEOUT` | 终态 |

每次状态切换都会保存并发布：previous state、current state、transition reason、simulation tick、30Hz 失败计数、macro stable-exit 计数、selected side、blocker id、local target update event。`previous_fsm_state` / `transition_tick` / `transition_reason` 三者**取自同一条 TransitionRecord**（最近一次真实切换），即使在两次切换之间也保持内部一致。`TransitionRecord.local_target_update_event` 记录的是该次切换最终对应的**有效 local target event**（FSM 自身计数器，提交切换时已定稿），**不是**尚未填充的输出默认值；同一 tick 内多次转换时最终发布最近一次真实转换记录。**终态也完整填充** side / evidence / 左右与锁定路线 / 特权 blocker / 局部 blocker / 关联状态 / latched 进度 / 失败与宏观计数 / 审计，GUI 在 `TASK_INVALID` 后仍能看到导致失败的选边与局部/特权 blocker。

**FOV 滞回（TURN_TO_TARGET）**：目标方位角 `|bearing| > 42°`（`turn_enter_deg`）时进入转向模式；**退出必须同时满足** `|bearing| ≤ 8°`（`turn_exit_deg`）**且** `|state.yaw_rate| ≤ 0.15 rad/s`（`turn_exit_max_yaw_rate`）——剩余角速度必须衰减后才能恢复平移，避免航向过冲。转向命令仍受 `max_yaw_rate` / `max_yaw_accel` 约束（由共享运动学限制）。**近目标放宽**：距离目标 ≤ `near_goal_heading_relax_distance`（默认 1.0 m）时，进入阈值放宽到 `near_goal_turn_enter_deg`（默认 75°）；进入 `goal_tolerance` 后禁止重新进入原地转向，改由终点控制器同时收敛平移速度与角速度。

**宏观模式进入条件**（全部满足才进入）：
- 目标已在 30Hz FOV 内（非转向模式）；
- 失败原因是“已观测到的真实阻挡”`BLOCKED_BY_OBSERVED_OBSTACLE`（UNKNOWN 或 FOV 原因绝不触发宏观）。v5：**“存在安全候选但全部没有有效进展 + 存在当前可见障碍证据”** 也归类为 `BLOCKED_BY_OBSERVED_OBSTACLE`（计入连续失败预算），而 **UNKNOWN-only / 无可见 blocker 的停滞** 归类为 `STALLED_WITHOUT_PROGRESS`——绝不伪装成观测阻挡、绝不偷偷用全局信息选边；
- 30Hz 局部规划连续失败达到 `macro.local_failure_duration_s`；
- 当前速度能够安全制动**且制动终点保持 `handoff_clearance`**（`canBrakeSafely()` 与 `spaceToStop()` 与新的宏观交接基础净空一致；由于动态包络会更早报告真实阻塞，`local_failure_duration_s` 的 0.4s 暂不直接缩短）；
- 不是单次候选采样失败；
- 宏观重入滞回（`macro_reentry_guard_ticks`，单位为 **30Hz tick**）**在每个非终止 30Hz step 上饱和递减一次**（不只在 5Hz 边界，且不会变成负值），30 ≈ 1s 已过。

**宏观模式退出条件**（连续 `macro_exit_stable_ticks` 个 5Hz tick 全部满足）：
- 最终导航目标重新进入带裕度的 FOV；
- 30Hz 专家用当前局部观测预检查（`previewPlan`）能独立生成**安全且具有有效进展**的轨迹；进入最终目标收敛区后，安全的 `TERMINAL_SETTLING` 也允许接管，以免宏观 guide 绕过终点控制器。v5 的 `PreviewResult` 扩展了 `has_progressing_trajectory` / `executable_progress_m` / `safe_prefix_duration_s` / `selected_output_speed_mps` / `failure_reason` / `planner_status`；**普通 SAFE_HOLD/远距离安全停止仍不足以退出宏观**；
- 已越过当前阻挡物（**`blocker_passed_latched`**）：以**宏观进入时固定的参考路线**（entry reference route）判定——车辆沿该路线的进度用**窗口化单调投影**跟踪（连续线段投影保留；只在上次匹配段附近的相邻段 / 合理弧长窗口内搜索；单 tick 前向增量 ≤ `max_speed × 5Hz 间隔 + progress_forward_tolerance_m`；侧向投影 > `blocker_projection_max_dist_m`（保守默认 **2.0 m**，与局部路线走廊宽度一致）时保持旧进度，**不允许**全路线最近点 + `max()` 突然跳到远端相近线段）。一旦进度超过 blocker 中心在该参考路线上的弧长 + `blocker_clear_dist_m`，`blocker_passed_latched` 置位且**此后不回退**（连续目标变化不清除它）。latched 后宏观改用**普通全局 ESDF 路线**继续引导，不再强制经过原 blocker 的 gateway，也**不再因固定侧路线不可行而 TASK_INVALID**；
- 当前计划不再依赖远处宏观路线；
- 当前轨迹不是紧急制动或恢复转向。

**退出条件计数**：满足全部条件后 `macro_stable_exit_count` 连续累计 `macro_exit_stable_ticks` 个 5Hz tick 才真正退出宏观。

退出后：取消宏观局部目标、局部目标恢复为最终目标、**保留动力学状态和局部地图历史、不重置速度**，并用滞回防止反复进出。

## 5. 30Hz 局部专家行为

1. 目标方向不在 FOV → `TURN_TO_TARGET`（带滞回）。
2. 正常模式每个 tick 都重新规划：速度 × 侧向 × yaw-rate 采样，生成多条候选轨迹并**真的比较代价**（不是势场控制）。
3. **横向采样左右对称**（默认 `lateral_ratio_samples: [-0.5,-0.25,0,0.25,0.5]`；机体系 +Y=左/CCW，正值=LEFT、负值=RIGHT）。
4. **候选预测与执行使用同一个共享运动学积分函数**（`kinematics.hpp::integrateKinematicStep`，由 `PlanarVehicleSimulator` 和 `LocalPlanner30Hz` 共用）：每一步都施加线性加速度限制、`max_speed` 径向限幅、yaw 加速度限制与 `max_yaw_rate` 限幅，预测与执行永不背离。
5. 候选轨迹安全性按**整条扫掠路径**检查，而非离散节点：每两个轨迹点之间的线段按 ≤ `obs_resolution/2` 连续采样，所有样本都必须：
   - 位于当前 FOV（可执行前缀）；
   - 是当前已知 FREE（UNKNOWN 不可作为安全自由空间）；
   - 与已观测障碍物满足**硬净空**（距离 ≥ `min_clearance`，默认 0.5 m）与**速度相关动态净空**（见下面第 5.1 节）；
   - 对当前局部目标不**明显后退**（`max_allowed_regress_m`，默认 0.05 m；轻微零进展候选允许进入候选集合，不再用 0.05 m 硬门槛立即删除“停止”候选，停止↔横移逐帧切换由第 5.4 节停滞检测器统一处理）；
   - 代价包含：进展、**连续软净空**、**真实控制变化/jerk/曲率近似**的平滑项（不是行驶距离）、速度变化、**yaw-rate 变化（控制变化，不是绝对转向量）**、**目标航向误差、速度方向对齐误差、横向偏差（局部参考线）**。

   **5.1 连续软净空代价（修复“净空代价失效”）**：旧公式 `cost = 1 - cl/min_clearance` 只在 0.5 m 半径内搜索，一旦 `cl ≥ 0.5` 代价恒为 0，局部规划器没有提前避障梯度。新公式使用严格圆形半径的有界OCCUPIED栅格遍历（半径 ≥ `soft_clearance_radius_m`，且当动态包络需要更大半径时用 `max(soft_clearance_radius_m, dynamic_required_clearance)`）得到**实际**的最近已观测OCCUPIED栅格中心距离：
   ```
   hard_clearance = lp_min_clearance (0.5 m)
   soft_radius    = lp_soft_clearance_radius_m (2.0 m)
   未搜索到障碍物        → cost_clearance = 0
   否则                  → cost_clearance =
       clamp((soft_radius - min_clear) / max(eps, soft_radius - hard_clearance), 0, 1)
   ```
   净空接近 2.0 m 时代价≈0，越靠近障碍物代价连续增大，到 0.5 m 附近代价≈1——可行候选的 clearance cost 不再全部为 0，规划器会在 0.5 m 硬门槛触发前就持续偏向远离障碍物的候选。`selected_soft_min_clearance_m` 输出**实际搜索到的距离**（仅当搜索半径内确实没有障碍物时才为 inf/NaN），不再是 0.5 m 小半径搜索产生的假 inf。

   **5.2 速度相关动态安全/交接包络（修复“进入 0.5~0.6 m 才触发 5Hz 专家”）**：局部规划器不能只检查固定 0.5 m 净空。静态配置推导的**宏观交接基础净空**（允许 30Hz 规划器使用，非运行时特权信息）：
   ```
   handoff_clearance = scene_safety_clearance (0.5)
                     + macro_route_clearance_margin (0.1)
                     + clearance_discretization_margin_m (0.05)
                     ≈ 0.65 m   (0.05 m 用于 0.1 m 栅格离散误差)
   ```
   对候选轨迹每个采样点：用相邻轨迹点估算该点世界速度 → 查询最近已观测 OCCUPIED 栅格 → 取“轨迹点→障碍格中心”单位向量 → 只计算**朝障碍方向的闭合速度**：
   ```
   closing_speed = max(0, velocity_world · direction_to_obstacle)
   required_clearance = handoff_clearance
                      + closing_speed * obstacle_reaction_time_s (0.20)
                      + closing_speed^2 / (2 * max_accel)
   ```
   规则：对搜索圆内每个已观测OCCUPIED栅格分别计算闭合速度和动态要求，不能只检查最近栅格——最近栅格可能位于切向，而稍远栅格可能正位于速度方向。任意栅格满足`cl < min_clearance`即为**硬净空违规**（`OBSERVED_CLEARANCE_TOO_SMALL`）；满足`cl < required_clearance`即为**制动/宏观交接净空不足**（`INSUFFICIENT_BRAKING_CLEARANCE`）。两者都是真实的已观测障碍物阻塞，**整条候选直接拒绝**；UNKNOWN/FOV边缘仍按安全前缀截断规则处理。

   **5.3 候选语义 v5：rollout intent 与 executable output 分离（修复“单周期投影把长期意图压缩成停止”）**：YAML 采样集合（`speed_samples` / `lateral_ratio_samples` / `yaw_rate_samples`）是**长期控制意图（rollout intent）**。每个候选明确拆成两类控制量：
   - **intent**（`desired_vx_body` / `desired_vy_body` / `desired_yaw_rate`）：长期期望控制意图（原始 YAML 采样或反馈律），**只用于 2.5s 前向预测**——每个 `lp_dt` 都调用共享的 `integrateKinematicStep()`，由运动学逐步施加最大加速度、最大速度、最大 yaw 加速度与最大 yaw rate（从静止向 2.5 m/s 意图自然加速）；
   - **executable output**（`vx_body` / `vy_body` / `yaw_rate`）：**当前一个 30Hz 控制周期内实际可达的命令**，由共享函数 `reachableCommand()` 把意图投影到单周期可达窗口：
   ```
   current_v_body = R(yaw)^T · velocity_world
   vx_out = clamp(intent_vx, current_vx_body - max_accel*control_period_s,
                              current_vx_body + max_accel*control_period_s)
   vy_out = clamp(intent_vy, ...)
   yaw_rate_out = clamp(intent_yaw_rate,
                        current_yaw_rate - max_yaw_accel*control_period_s,
                        current_yaw_rate + max_yaw_accel*control_period_s)
   （之后统一限制到 max_speed / max_yaw_rate）
   ```
   **只有 executable output 会下发给模拟器 / 记录进专家标签**；**绝不允许**把“单周期投影后的微小命令”当作完整 2.5s 恒定 rollout 命令。所有模式（普通局部候选、TURN_TO_TARGET、terminal controller、emergency brake）使用**同一种输出语义**——例如 TURN 的 yaw rate 意图也先经过单周期 yaw 加速度窗口，不能普通模式输出可达 yaw rate、TURN 模式却直接输出 2 rad/s。
   **去重规则（v5）**：按“**长期意图 + 可达首命令**”的联合键确定性去重（1e-4 量化键 + 有序映射，与枚举顺序无关、左右严格对称）。不同长期意图即使投影成同一个可达首命令也**必须保留**（它们的 rollout 不同），因此从静止出发时 `speed_samples=0.6/1.2/1.8/2.5` 不会被压缩成一个候选。`stable_index` 对左右镜像意图对共享（`(vx,+vy,+yr)/(vx,-vy,-yr)`），`tie_hash` 只编码意图数值、从不编码固定左绕/右绕偏好。`dynamic_window_candidate_count` 现在 = **真正参与预测的长期意图候选数量**。

   **5.4 停滞/控制振荡检测器（v7，面向连续 LocalTarget）**：轻量、确定性 C++ 状态（属于 `LocalPlanner30Hz`，`previewPlan` 不污染）。维护最近 `limit_cycle_window_ticks`（默认15个30Hz规划样本）的窗口，记录：mission revision、距目标距离、选中 **executable output**、是否 observed/dynamic-clearance 阻塞、**实际车辆位置与目标位置**。窗口只在 `reset()`、**mission revision 变化**（新任务 / 最终目标变更 / 进入或退出宏观）、**目标不连续重置**以及 `TURN_TO_TARGET`期间清空——**普通 update_event 数值变化（5Hz 连续目标滚动）不清窗，且不再读取 `is_macro_guide`**（v7 移除）。停滞判据使用**窗口内车辆实际位移**（对移动目标依然有效：目标正常移动时也能发现“车辆长期原地不动”），同时检查**沿每周期目标方向的投影进展**；两项都小于 `limit_cycle_net_progress_m`（0.10m）且满足以下任一条件时判定局部规划失败：
   - **A**：窗口内持续存在真实观测阻塞（≥ `limit_cycle_min_blocked_ticks`，默认 8 tick）；
   - **B**：窗口内存在真实观测障碍证据，并多次出现“近零命令—明显横移命令—近零命令”（或横向命令符号反转 ≥ `limit_cycle_lateral_flip_count`，默认2次）。没有障碍证据的纯跟踪振荡不会误触发5Hz特权专家。
   直接模式检测到后输出**安全制动**（同样经过单周期可达窗口）、`failure_reason = BLOCKED_BY_OBSERVED_OBSTACLE`、置诊断标志 `local_limit_cycle_detected`，让现有 0.4 s 宏观失败预算正常累计。宏观引导激活后，若30Hz已找到安全且有进展的候选，该候选就是脱困动作，限环状态仅保留为诊断、不会再把动作覆盖为零，从而避免“无位移→限环→刹停→继续无位移”的闭锁。`dynamic_clearance_blocked` 也只描述最终规划结果确实因动态包络无可执行进展，不再因任意一个采样候选被拒就置位。

   **5.5 v5：progress 资格、分层选择与新 success 判据（修复“长期 success+零速度”）**：
   - **progress 归一化**：`cost_progress = 1 − clamp(executable_progress / achievable, −1, 1) + short_prefix_penalty`，其中 `achievable_progress = max_speed × scoring_horizon_s`（所有候选统一的归一化时间尺度），**不再除以当前到目标的总距离**；不足统一评分时域的安全前缀保留明确、连续的截断惩罚。
   - **progress 资格（`progress_qualified`）**：远离目标时，候选必须同时满足：executable output 速度 ≥ `min_progress_speed_mps`（默认 0.03，低于静止起步的单周期可达首命令 ≈0.067）、前缀内目标进展 ≥ `min_progress_epsilon_m`（默认 0.01m）、`dist_current > task_goal_tolerance`。停止候选永远不满足。
   - **分层（字典序）选择，不再只靠加权代价**：①硬安全约束（FOV / known-FREE / 硬净空 / 动态制动包络）；②是否具有有效进展；③动态净空与可执行前缀；④同一类别内再比较 tracking / clearance / smoothness 等代价。远离目标时**只要存在安全且具有进展的候选，停止候选不能击败它**；零速度候选只允许用于：已进入终点收敛区域（`dist ≤ terminal_control_distance`，状态 `TERMINAL_SETTLING`）、TURN_TO_TARGET、emergency brake、所有运动候选都不安全。
   - **planner status（`PlannerStatus`）**：`SAFE_PROGRESSING` / `SAFE_HOLD` / `TERMINAL_SETTLING` / `TURNING` / `EMERGENCY_BRAKE` / `BLOCKED_BY_OBSERVED_OBSTACLE` / `NO_SAFE_CANDIDATE` / `STALLED_WITHOUT_PROGRESS` / `NO_TARGET`。**所有候选都没有进展时不能返回 success=1、failure=NONE**：若存在可见障碍证据 → `BLOCKED_BY_OBSERVED_OBSTACLE`（计入宏观失败预算）；否则 → `STALLED_WITHOUT_PROGRESS`（30Hz 内部回归，绝不靠 5Hz 特权专家掩盖；空旷环境、目标在 FOV 内、距离很远时禁止出现长期 success+零速度）。
   - **消除重复惩罚**：单周期加速度窗口是硬约束；`smoothness` 只惩罚 executable output 之间的突变/加速度变化/jerk；`speed_change` 只比较相邻周期 **executable output** 的变化，不惩罚长期 intent 的速度大小；“速度越小”绝不隐式等于“越平滑”。
   - **名义时域 vs 已认证前缀**：`nominal rollout`（2.5s，意图驱动，评估趋势）、`certified executable prefix`（当前 FOV + known-FREE + 硬净空 + 动态制动净空连续成立的前缀，≥ `min_executable_prefix_s`）、`actual output`（前缀第一个控制周期可达命令）。UNKNOWN 不当作 FREE；短于统一评分时域的前缀带截断惩罚，停在已知安全起点的 stationary 轨迹不能因此获得“完整 2.5s 安全轨迹”的不公平优势。诊断字段 `safe_prefix_duration_s` / `nominal_progress_m` / `executable_progress_m` 贯通 CSV v5 与 GUI。

   **5.6 v7：连续提前避障风险（修复“避障启动过晚 / 临近才猛避”）**：旧实现只在候选轨迹低于硬/动态净空后“整条拒绝”，规划器在障碍物进入 2.5s rollout 视野前毫无反应，直到净空 0.7~0.9 m 才突然选择大横移。v7 在每个候选的代价里加入**连续、归一化的 obstacle-risk 项**（`cost_w_obstacle_risk`，默认 3.0），它成为“什么时候开始避障”的主要驱动——**0.5 m / 0.65 m 只做安全验证与路径质量参考，绝不作为避障触发距离**。对候选的名义 rollout 与当前局部观测中的每个 OCCUPIED 格子计算：
   ```
   dir_cand   = 候选长期 body-frame intent 旋转到世界系（静止时回退 rollout/当前航向）
   lon        = (cell - pos)·dir_cand        # 沿候选方向的纵向距离
   lat        = |cross(dir_cand, cell - pos)|  # 横向距离
   d_path     = 格子到候选 rollout 折线的最短距离（预测最近接近）
   ttc        = rollout 首次进入 nominal_clearance 包络的时间（从未进入则 +inf）
   lon_gate   = clamp((distance_horizon + shoulder - lon)/shoulder, 0, 1)
   g_corridor = clamp(1 - (lat - corridor_half_width)/shoulder, 0, 1)   # 走廊成员
   g_traj     = clamp(1 - (d_path - hard)/trajectory_radius, 0, 1)      # 轨迹扫掠成员
   membership = max(g_corridor, g_traj)      # 二者满足其一才产生避让
   dist_factor= clamp((distance_horizon - lon)/(distance_horizon - hard), 0, 1)
   clear_factor= clamp((distance_horizon - d_path)/(distance_horizon - hard), 0, 1)
   ttc_factor = clamp((ttc_horizon - ttc)/ttc_horizon, 0, 1)
   risk_i     = lon_gate · membership · max(dist_factor, clear_factor, ttc_factor)
   risk       = max_i(risk_i)
   ```
   参数：`risk_corridor_half_width`（1.0）、`risk_distance_horizon_m`（5.0）、`risk_ttc_horizon_s`（2.5）、`risk_trajectory_radius_m`（1.0）、`avoidance_active_threshold`（0.10）。要点：
   - 障碍物在走廊前方 4 m 时 risk 已明显非零（几何距离因子 + TTC 因子），规划器在**少量周期内**就选择小幅减速/横移/转向，风险随接近**连续增长**、无新二值跳变；
   - **不阻挡走廊且远离轨迹扫掠的侧面障碍物 risk ≈ 0**（不无意义绕行）；
   - 风险沿“候选方向”与“轨迹最近接近”计算，因此**向左/向右偏航或减速的候选风险更低**——方向性自然涌现，不写死左/右偏好；风险方向仍可由 `lat`/`lon` 分解诊断；
   - 只用当前局部观测，不向 30 Hz 规划器泄漏全局 ESDF/路径；5 Hz 特权信息不受影响。
   新增诊断（贯通 msg/GUI/CSV v7）：`local_corridor_blocked`、`first_blocking_obstacle_distance`、`predicted_closest_clearance`、`time_to_collision`、`obstacle_risk_cost`、`avoidance_strength`、`avoidance_active`、`local_target_distance`、`selected_cost_obstacle_risk`。**“走廊受阻”判定**：沿车辆→LocalTarget 线段以 ≤ res/2 步长推进，若某步在 `risk_corridor_half_width` 半径内发现 OCCUPIED 格，则走廊受阻，`first_blocking_obstacle_distance` = 该步纵向距离（否则 NaN）。

   **5.7 v7：净空语义与动态制动包络重定义**：
   - `hard clearance = lp_min_clearance`（0.5 m）：无人机中心到障碍物表面的最低硬安全距离，**不因障碍物半径小而降低**；只做安全验证。
   - `nominal_clearance_m`（0.65 m）：期望路径质量参考（软净空代价刻度 / 文档中的 handoff 基值），**不是避障触发距离**。
   - 动态净空：`requiredClearance(closing)` 对**分离/恢复运动（closing ≤ 0）返回硬净空 0.5**，对闭合运动才用 `handoff + closing·reaction + closing²/2a`。因此**当前净空位于 0.5~0.65 m 时，单调增大的低速恢复轨迹可以被选中**（不会因“起点低于期望净空”把所有平移候选拒绝）。
   - 动态包络逐预测点使用候选轨迹**该点的实际速度**（`v_seg`），绝不用当前高速的完整停止距离去拒绝整条恒速候选；更早减速与渐进横移自然降低动态净空需求。
   - 硬净空整条拒绝规则对所有目标来源**统一**（v7 删除了 is_macro_guide 例外分支）；5 Hz 引导点经特权 chord-clearance 裁剪且具有 ≥ `guide_min_distance_m` 前视后，安全前缀不会被误拒。
   - 小半径障碍物：表面净空仍 ≥ 0.5 m，但绕行几何宽度由实际表面几何决定（更紧凑），不对所有障碍物生成同样宽度的大绕行。

   **5.8 v7：统一目标语义（最终目标 / 5 Hz 引导点 / 未来目标网络完全一致）**：30 Hz 规划器**不再按 `is_macro_guide` 分支运动策略**。已删除的分支：①终点反馈控制器对 macro guide 的跳过（现在对任何 LocalTarget 进入 `terminal_control_distance` 都统一启用）；②终点收敛区 `terminal_region` 对 macro guide 的排除；③候选整条拒绝对 macro guide 的例外；④停滞检测触发制动对 macro guide 的豁免（停滞窗口清空也只按 mission revision / 目标不连续，不再读 `is_macro_guide`）。`is_macro_guide` 仅保留用于：日志、可视化、上层目标管理与任务完成判定。相同局部观测 + 相同状态 + 相同 LocalTarget ⇒ 相同专家动作（训练语义一致）；5 Hz 特权信息只能通过替换 LocalTarget 间接影响 30 Hz 输出。采样也更密（v7）：`lateral_ratio_samples` 增加 ±0.05/±0.15，`yaw_rate_samples` 增加 ±0.15，`speed_samples` 增加 0.3，保证从直行到绕行有连续的小幅候选（纯减速 / 小/中/大横移 / 小/中转向 / 组合 / 制动）。

   **5.9 v7：5 Hz 滚动引导点（carrot）生成与无进展处理（修复“脚边引导点永久零输出”）**：`MacroExpert5Hz::guidanceTarget` 现在按“车辆在锁定路线上的连续弧长投影 + 自适应前视”选择引导点：
   - **车辆投影**：连续线段投影（非最近 waypoint），得到 `progress`（弧长）与侧向距离；
   - **自适应前视**：`lookahead = clamp(min + speed·guide_lookahead_time_gap_s, min, max) × curvature_factor`，曲率大的路段收缩前视（不切弯）；`macro_guide_lookahead` 记录实际值；
   - **硬最小距离**：引导点距车辆 < `guide_min_distance_m`（1.5m）时沿路径前推直到满足（或退化为指向目标的 `fallback_goal_near_route_end`），**绝不生成 0.2~0.3 m 脚边目标**；
   - **FOV 夹取**：引导点越出带裕度 FOV 时沿路径回拉至 FOV 边界（不低于最小距离；极紧弯道整体在 FOV 外时保留并让 30 Hz 进入 TURN）；
   - **特权 chord 裁剪**（仅 5 Hz）：车辆→引导点直线段在特权 ESDF 上保持 ≥ handoff 净空（带“从 0.5~0.65 带单调爬升”的恢复模式），不要求 30 Hz 切 blocker 的角；
   - **世界系滞回**：健康新引导点（≥ 最小距离）移动 < `guide_hysteresis_m` 时保持上一引导点，滚动路线不会让 5 Hz 目标左右跳变；`macro_guide_update_reason` 记录 `normal_advance / min_distance_advance / fov_clamp / chord_clip / hysteresis_hold / fallback_goal* / no_progress_advance`。
   - **无进展处理（FSM）**：宏观期间按每 5 Hz 间隔位移统计 `macro_no_progress_duration`；连续超过 `no_progress_duration_threshold_s`（1.0s）时丢弃引导点滞回并强制重新前推 / 重规划路线（`no_progress_advance`），车辆不动也不会永久停在同一个近距离引导点。任何恢复轨迹仍必须通过 30 Hz 的 FOV / known-FREE / 硬净空 / 动态净空验证——不绕过碰撞检查。
   - **退出接管**：保持 v5/v6 语义——最终目标重新进入带裕度 FOV，且 30 Hz preview 连续 `macro_exit_stable_ticks` 个 5 Hz tick 能独立产生安全且有进展的轨迹时退出宏观（找到一个安全停止候选不足以退出）。
6. 正在执行的轨迹被新观测障碍物截断 → 当前 tick 立即重新避障（`immediate_avoidance_event`）；只检查当前车辆位置之后的**可执行后缀**，不重复检查已走过的旧轨迹段。`currentTrajectoryBlocked()`与候选一致地对有界搜索圆内的每个已观测障碍栅格计算闭合速度和动态安全包络，一旦任意障碍使当前轨迹违反包络就置`immediate_avoidance=true`，而不是进入0.5m才反应；若停止距离不足则立即安全制动。
7. **制动方向与交接净空**：`spaceToStop()`在速度大于阈值时沿归一化`velocity_world`检查无人机中心的完整制动轨迹，速度接近0时才退回yaw前向。`scene_safety_clearance`和`handoff_clearance`已经定义为“无人机中心到障碍物表面”的距离，因此这里不再额外横移`drone_radius`，避免重复膨胀。当前中心位置即使已经静止也必须是known-FREE且满足`handoff_clearance`，之后每个中心轨迹采样点也必须满足同一条件；该结果继续作为5Hz宏观进入的安全门控。
8. 目标由上层 5Hz ZOH 更新：**目标改变时不会重置速度、历史或规划器**。
9. 宏观触发只允许：目标已进入 FOV、存在已观测 blocker、`BLOCKED_BY_OBSERVED_OBSTACLE` 持续达到阈值、且能在已知自由空间内安全制动（制动终点保持 `handoff_clearance`）；UNKNOWN、目标在 FOV 外、单次候选失败都不能触发宏观。**失败分类规则（v4）**：当所有候选失败时，只要至少有一个候选因 `OBSERVED_CLEARANCE_TOO_SMALL` 或 `INSUFFICIENT_BRAKING_CLEARANCE` 被拒绝，就分类为 `BLOCKED_BY_OBSERVED_OBSTACLE`（不再只依赖“目标方向中心线 0.5 m 内是否有障碍物”的 `corridorBlockedByObserved()`，否则动态包络提前拒绝时会被错误标为 `NO_SAFE_CANDIDATE` 导致 5Hz 专家无法触发）；`corridorBlockedByObserved()` 仅作为补充判断保留。
10. **可执行安全前缀（Executable Safe Prefix）**：候选轨迹按**整条扫掠路径**验证；预测后缀首次到达当前 FOV 边界或 UNKNOWN 时，可以保留并执行其完全位于当前 FOV + known-FREE + 合法净空内的**安全前缀**。候选被接受时截断为安全前缀（≥ `local_planner.min_executable_prefix_s`，默认 0.2s）作为输出轨迹与代价/进展依据。短于 `scoring_horizon_s`（默认 0.8s）的前缀会按缺失时长增加 progress cost，仍可在其为唯一可行运动时被选中，但不能再利用“预测太短、jerk/对齐误差尚未发展”的评分漏洞。若预测首次失败原因是**已观察障碍物净空不足**，则整条候选立即拒绝。
11. **候选拒绝原因诊断**：每个被拒候选只累计其**第一个决定性原因**，计数包括 `reject_not_known_free` / `reject_outside_current_fov` / `reject_observed_clearance_too_small` / `reject_no_progress` / `reject_other` / `reject_insufficient_braking_clearance`（v4 新增），六者之和 = `rejected_candidate_count`（每 tick 重新计算；preview 不污染正式状态；转向模式不生成候选时全为 0）。计数进入 SimSnapshot / DebugSnapshot.msg / Qt 面板 / CSV v4 日志。
12. **LOCAL_UNKNOWN_RECOVERY 诊断**：连续 `NO_SAFE_CANDIDATE`（UNKNOWN 驱动、**非**真实观测阻挡）tick 数由 FSM 跟踪并在超过 `macro.unknown_recovery_threshold_ticks`（默认 60 tick ≈ 2s）时置 `unknown_recovery_active`；`NO_SAFE_CANDIDATE` 仍**不触发 5Hz 特权专家**、仍**不**被误标为真实阻塞（`blocked_observed` 恒 false）。按“不扩大成本、不破坏 TURN_TO_TARGET 滞回”的原则，本轮采用显式诊断而非新增旋转状态。
13. **目标跟踪代价（消除无障碍弓形轨迹）**：候选安全可行后新增三项对齐代价，参考方向/参考线**只由当前车辆位置与当前 LocalTarget 构造**（不是全局路径、不含特权信息），LocalTarget 连续变化时每 tick 自动重建（宏观模式下即 5Hz 局部目标）。全部归一化到 ~[0,1]：
    - `terminal_heading`：候选安全前缀末端 yaw 与“当前车辆指向当前 LocalTarget”的局部参考方向之差 `|wrap(target_dir − yaw_end)| / π`，权重 1.5；不再因候选前缀提前进入 `goal_tolerance` 而突变为零；
    - `velocity_alignment`：候选末端世界速度方向与同一局部参考方向的夹角 / π，权重 1.2；低速/近零速度时回退到航向对齐；参考线每 tick 随 LocalTarget 重建，兼容连续变化目标且不使用全局路径；
    - `cross_track`：候选安全前缀各点相对“车辆→LocalTarget”局部参考线的平均横向距离，除以 `cross_track_normalize_m`（默认 2.0）后 clamp 到 [0,1.5]，权重 1.0。
    无障碍直线路径上，对齐目标的候选（含小 yaw_rate 持续修正）在航向/速度方向/横向偏差三项上占优，规划器近似直线追踪 LocalTarget；不再因“绝对转向量惩罚”而长期保持错误航向。
14. **smoothness / yaw-rate-change 语义**：`smoothness` 用连续三段速度形成两段加速度，计算 `mean(|Δacceleration|)/(2*max_accel)`，不再把加速度误写成 jerk；`yaw_rate_change` = `|命令 yaw_rate − 当前 yaw_rate| / max_yaw_rate + 前缀内 mean(|yaw_acceleration|) / max_yaw_accel`（yaw 差分先 wrap，避免跨越 ±π 产生假跳变；总项 clamp [0,1.5]，权重 0.3）。它们约束控制变化而不惩罚朝目标修正航向本身；默认 yaw-rate 采样包含对称的 `±0.25/±0.5 rad/s` 精细候选，避免退出原地转向后只能在 0 与至少 1 rad/s 之间跳变。
15. **确定性并列规则与控制连续性**：最佳候选采用带容差（`cost_tie_tolerance`，默认 1e-6）的比较；并列末级还比较上一帧线速度命令变化。`speed_change` 使用身体坐标系速度向量而不是仅比较标量速度，`smoothness` 包含候选首段与上一命令形成的加速度跳变，因此 2～3 点短前缀不再免费获得零平滑代价。目标 update event 变化时不沿用旧目标的命令偏好，保证连续变化目标可以立即生效。
16. **目标跟踪诊断字段（每 tick，贯通 msg/GUI/CSV v4）**：`target_bearing_error_deg`、`selected_terminal_heading_error_deg`、`selected_velocity_alignment_error_deg`、`selected_cross_track_error_m`、`selected_cost_total` 以及 progress/clearance/smoothness/speed_change/yaw_rate_change/terminal_heading/velocity_alignment/cross_track 各项成本。preview 不污染这些诊断。v4 追加软净空/动态包络诊断：`selected_soft_min_clearance_m`（实际测得净空）、`selected_dynamic_required_clearance_m`、`selected_closing_speed_mps`、`handoff_clearance_m`、`dynamic_clearance_blocked`、`local_limit_cycle_detected`、`dynamic_window_candidate_count`、`reject_insufficient_braking_clearance`、`start_clearance_recovery_used`。未选中候选时软净空/动态字段为 **NaN**（明确“无测量”，避免把 0 误读为真实测量）。
17. **终点连续制动（v7 统一应用于所有目标来源）**：任意 LocalTarget（最终目标、5 Hz 宏观引导点、未来目标网络输出）进入 `terminal_control_distance`（默认 1.2m）后，优先滚动预测一个目标指向反馈控制器；速度由 `terminal_speed_gain`、制动距离和 `terminal_max_speed` 共同限制，在目标容差圈的内半径连续降到零。该轨迹与普通候选经过完全相同的当前 FOV、known-FREE、硬净空和**动态净空**验证；验证失败才回退普通避障采样。v7 **不再因 `is_macro_guide` 跳过终点控制**——宏观引导点保证 ≥ `guide_min_distance_m`（1.5m）前视，车辆接近引导点时按普通目标规则减速，5 Hz 层在其停车前沿路径前推引导点；“近目标航向放宽”同样只按目标距离统一应用。进入 goal tolerance 后不再追逐不稳定 bearing。`GOAL_REACHED` 同时要求位置、平移速度和角速度达标，且**宏观引导点到达不会宣告任务完成**（任务完成判定留在任务管理层）。终点反馈律输出同样通过 `reachableCommand()` 单周期可达窗口（`terminalIntent` 是意图、输出是可达命令），与其他模式语义一致。
18. **训练接口契约（v5）**：最终结构保持
```
最终导航目标 → 5Hz目标专家 → 连续LocalTarget → 30Hz局部运动专家 → 安全过滤 → executable command
```
30Hz 网络的训练输入必须能明确对应：**局部当前/历史观测**、**车辆状态**、**当前 LocalTarget**、必要时 **LocalTarget 的相对方向/距离/是否 macro guide**——**不含全局 ESDF 或全局路径**（接口层面 `plan(state, obs, target)` 结构上无特权形参）。**30Hz 专家标签是 executable output（单周期可达命令），不是全局路径、也不是 5Hz 路线**；5Hz 专家的训练目标是 **LocalTarget 序列**，5Hz 特权路径绝不混入 30Hz 控制标签。`DebugSnapshot`/CSV v5 的 `selected_intent_*`（长期意图）与 `selected_output_*`（可达输出）把两者在日志里分开记录。

## 6. 左右选择因果规则

**统一二维左右定义**：`axis = goal - start`，`cross(axis, point - start) > 0` 表示 LEFT，`< 0` 表示 RIGHT（gateway、A* 侧约束、可见证据与 GUI 颜色全部一致）。

1. 用**当前瞬时 FOV patch（current_patch）**分别计算 LEFT / RIGHT 的“可见前缀证据”：只在**左右均位于 FOV 内的成对对称射线**上采样，取**平均可见自由距离**（`SideEvidence.left_score/right_score` 保存的就是用于比较的平均值，**不是**未归一化总和）；有效射线对不足 `min_evidence_ray_pairs` 时判为 ambiguous。
2. 视野明确显示某一侧更容易通过 → 选该侧；**即使全局真值知道该侧后续路线更长也仍然选该侧**。
3. 视野无法可靠区分（含射线对不足、FOV 外）→ 固定选 **RIGHT**（`AMBIGUOUS_DEFAULT_RIGHT`）。
4. 选中后**锁定该物理侧**，直到越过当前阻挡物。**固定物理同伦参考（HomotopyReference）**在宏观进入时保存：entry 位置→entry 最终目标的单位轴 `entry_axis`、`entry_blocker_center`、选中侧与带符号侧法向。此后 LEFT/RIGHT 的物理含义**只相对该固定参考解释**——车辆移动、连续导航目标变化都**不会重新定义左右**；LeftRightRoutePlanner 的 gateway、A* 侧偏代价与同伦检查**接收该固定参考**，而不是在内部用新的 current_start→goal 轴解释 `SideSelection`。
5. 全局特权信息只用于生成/刷新选定侧的完整安全路线。
6. LEFT/RIGHT 路线是**真正的侧约束路线**：必须经过 blocker 对应侧的切线 gateway。gateway 以**固定参考的 blocker 中心 + entry 轴**为参考（`cross(entry_axis, p - entry_blocker_center) > 0`=LEFT；两侧分别计算两个切点并选位于目标全局侧的那个，绝不对 start/goal 用同一旋转符号）；A* 侧偏势进入累计 g-cost；最后用**连续线段**做同伦校验（不是离散 waypoint）。若 LEFT/RIGHT 落到同一侧/同一同伦路线，则该侧 `valid=false`，**绝不把两条相同路线显示成左右两条**。shortcut 与平滑按**分段**进行，强制 gateway 不会被 shortcut 跨越，平滑后重新做连续侧别与 ESDF 安全验证。

**局部证据 vs 特权几何（严格分离）**：5Hz 先只用 **current_patch** 提取**局部阻挡证据** `LocalBlockerEvidence`（可见 OCCUPIED 簇的质心 / 包围半径 / 簇号 / 格子数 / **格子世界坐标**，**不含任何真值 id**）——触发、选边与审计只使用它。簇选择**优先与阻挡走廊真正相交**（横向跨度 ≥ 走廊半宽的一半）且**前向距离最近**的簇（不再仅按欧氏距离最近）。随后 `resolvePrivilegedBlocker` 对证据的每个 OCCUPIED 格子计算其对各真值圆柱**表面/内部匹配支持**（残差 ≤ `blocker_match_surface_tol_m`），按 支持数 ↓ / 几何残差 ↑ / 前向距离 ↑ / id ↑ **确定性排序**，返回显式状态：
- **MATCHED**：唯一主导圆柱，`BlockerInfo` 直接采用该圆柱中心/半径——**绝不把彼此有可通行表面间距的多个圆柱合并成大圆**（否则会虚构封闭障碍物）；
- **NO_MATCH**（支持不足 `blocker_match_min_cells` 或无圆柱支持）；
- **AMBIGUOUS_MATCH**（次优支持 ≥ `ambiguity_ratio × 最优支持`）。

NO_MATCH / AMBIGUOUS_MATCH **不得**按 `blocker.found=false` 静默执行普通 A*——FSM 明确进入 `TASK_INVALID_BLOCKER_ASSOCIATION`，snapshot 与 GUI 显示关联状态与原因。另外，切线 gateway 点恰好落在 `esdf == clearance`（严格 `>` 判定会失败），路由生成会把 gateway 投影到 `esdf > clearance + ½×单元对角线(+ε)` 的**合法格子**内再作为 A* 端点（`projectToLegalCell`，有界螺旋搜索）。
7. 若按可见规则选中的一侧**全局不可行**（且 blocker 尚未被越过）：不偷偷换侧，而是把任务标记为 `TASK_INVALID`（原因 `TASK_INVALID_FOR_CAUSAL_RULE`），在 GUI 中清楚显示拒绝原因。**一旦 `blocker_passed_latched=true`，新目标导致旧固定侧路线不可行也不会 TASK_INVALID**（改用普通全局路线继续引导）。
8. 任务资格审查尽量保证：视野无法区分时 RIGHT 全局可行；视野偏向某侧时该侧可行；**优先接受 LEFT/RIGHT 两条同伦路线都真正可行的任务**，便于观察左右决策。

## 7. 场景与任务

- 区域默认 `[-20,20]×[-20,20] m`，ESDF 分辨率 `0.1 m`。
- 障碍物：二维圆（无限高圆柱的水平投影），数量随机 `0~20`，半径随机 `0.1~4.0 m`（log-uniform 覆盖不同尺度；数量均匀覆盖稀疏/中等/稠密）。
- 任意两圆表面间距 ≥ `2*safety_clearance + passage_margin`（默认 `2*0.5+0.2 = 1.2 m`），到区域边界留出 `boundary_margin + safety_clearance`。
- **场景生成显式成功/失败**：只有随机抽到 0 个障碍物才产生合法空场景；非零障碍物放置失败时返回 `SceneGenerationResult{success=false, reason, requested/placed count}`，由上层换 seed 或重新请求，**绝不静默降级为空场景**。
- **scene counter 只在成功提交后递增**：`newScene` 先以当前计数为候选 scene_id 在临时状态生成，ESDF/连通域/任务全部成功并 commit 后才 `++scene_counter_`；任一步失败都不递增计数、不改动当前有效场景，服务返回真实失败。
- **启动失败语义**：节点首次启动若用户指定 seed 生成失败，**不会静默尝试 seed+1…seed+8**，而是 `ROS_ERROR` 报告 seed 与原因并把仿真置为未初始化（`simulation_initialized=false`）；此时不发布伪造的 snapshot/ESDF，step/reset/new task/set goal 等服务返回 `success=false, reason="simulation not initialized"`；`/generate_scene` 仍可显式提交另一个 seed，成功后恢复初始化。
- `DebugSimulation::newScene()` 是**事务化**的：先在临时 scene/ESDF/connectivity/task 中生成，全部成功后才替换当前有效状态；失败时**保留原场景、任务与仿真状态**（`initialized_` 不会被清空，已提交的有效状态不受影响），服务返回 `success=false` 和真实原因，用户换 seed 后可直接重试。
- **首个任务 seed 与历史计数器无关**：场景提交后的第一个任务 seed 恒为 `deriveTaskSeed(scene_seed, kSceneFirstTaskTag)`（固定标签，不依赖 `task_counter_`），因此相同 scene seed 在不同任务历史下永远产生**相同**的初始任务（确定性不随“做过几次 New Task”漂移）。
- 可选空间严格采用 `esdf > safety_clearance`（不混用 `>=`/`>`）；8 邻域连通域分析、任务 A* 与宏观 A* 都禁止**对角线穿墙角**（对角移动要求两个正交邻格同样合法）。
- 起终点从**主连通域**采样，满足最小距离、ESDF 安全距离，并用**全局 A\*** 做静态连通确认。**A\* 只用于场景/任务资格审查和 5Hz 特权专家，禁止交给 30Hz 专家。**
- **因果任务资格审查**：若起终点直线路径存在主要阻挡物，则预生成 LEFT/RIGHT 约束路线；`require_both_sides_feasible=true`（推荐）时只接受两侧都真正可行的任务，保证运行时可见证据决不会选到全局不可行的一侧；清晰无阻挡与零障碍物任务仍允许存在。
- **局部 blocker → 真值圆柱关联是显式三态**：`MATCHED` / `NO_MATCH` / `AMBIGUOUS_MATCH`。宏观触发后若关联失败，任务进入 `TASK_INVALID_BLOCKER_ASSOCIATION`（snapshot 的 `transition_reason` 与 `blocker_association_name`、GUI 均显示原因），**绝不**静默退回普通 A*。
- 初始偏航 = 起点→终点方向 + 随机偏航偏置（幅度 `20°~160°`，正负随机）。

## 8. 可视化颜色含义

| 颜色 | 含义 |
|---|---|
| 深色底 | 背景 |
| ESDF 热力图（viridis 色带） | 全局真值 ESDF（PRIVILEGED/TRUTH，可开关） |
| 深红实心圆 | 真值圆柱障碍物（PRIVILEGED/TRUTH） |
| 绿色 / 金色圆点 | 起点 / 最终导航目标 |
| 半透明白色扇形 | 当前 FOV 扇区及量程 |
| 绿色 / 红色 / 灰色半透明 | 历史局部地图 FREE / OCCUPIED / UNKNOWN |
| **橙色半透明叠加 + 标签** | **瞬时 FOV patch（current_patch）**：FREE 淡橙、OCCUPIED 亮橙、UNKNOWN 透明；与历史地图语义区分显示 |
| 亮蓝折线 | 30Hz 选中的局部轨迹 |
| 淡灰折线 | 被拒绝的候选轨迹（可开关） |
| 青色细线 | 已执行轨迹 |
| 绿色虚线 / 红色虚线 | LEFT / RIGHT 全局特权路线（PRIVILEGED/TRUTH，可开关） |
| 黄色粗线 | 当前锁定路线（latched 后为普通全局路线） |
| 橙色虚线圆 | 阻挡物（`MATCHED` 用特权圆柱几何） |
| **红色虚线圆 + 红色标签** | 阻挡关联失败（`NO_MATCH` / `AMBIGUOUS_MATCH`，显示局部证据簇） |
| 黄色三角 | 无人机（yaw 朝向） |
| 品红箭头 | 当前速度向量 |
| 青色圆环 | 当前局部目标 |
| 红色文字 | `PRIVILEGED / TRUTH` 标记 |

**仿真与可视化说明**：

- FOV 扇形以 **`TRIANGLE_LIST`**（每片 中心-圆弧点i-圆弧点i+1）发布（ROS 没有 `TRIANGLE_FAN`）；Qt GUI 与 RViz 都按此绘制。
- 速度向量以 **ARROW 两点形式**（`points[0]`=车辆位置，`points[1]`=速度末端）发布。
- 所有 MarkerArray 每次先发 `DELETEALL`，避免对象数量减少或状态消失时残留旧障碍物 / blocker / 文字。
- 仿真器对**圆柱**（含 `drone_radius`）与**矩形区域边界**（含 `drone_radius`，车辆中心不得越界）都做 swept 检查。
- 新任务初始化时 `tick_=0` → reset FSM → **立即构建合法初始 snapshot**：GUI 启动后无需先单步即可看到正确的状态、目标与审计字段。
- 起点/终点 Marker 在 Reset / New Task / 修改导航目标时同步更新（不仅 scene 变化时）。

## 9. GUI 按钮与操作

| 控件 | 作用 |
|---|---|
| Run / Pause | 开始 / 暂停 30Hz 定时器推进 |
| Single 30Hz Step | **只推进一个 1/30 s tick**，完成后**必定保持暂停**（原子暂停操作） |
| Step To Next 5Hz Tick | 推进到下一次宏观专家更新边界并完成该次 5Hz 决策，然后**必定保持暂停** |
| Reset Current Task | **重启当前完全相同任务**：同场景 / 起点 / 终点 / 初始偏航 / task_id，tick 清零，车辆 / 局部观测历史 / 已执行路径 / FSM 全部重置；不递增 task counter，也不使用 seed 字段 |
| New Task In Same Scene | 同场景**采样新任务**（新的起点 / 终点 / 初始偏航；从场景 seed 派生确定性任务 seed，task counter 递增） |
| New Scene | 用 seed 输入框的值生成全新场景 |
| Export Current Task Log | 导出当前任务从 tick 0 到当前时刻的完整 30Hz CSV 日志；路径显示在状态面板最后一次服务结果中 |
| seed 输入框 | 指定场景 / 任务 seed |
| simulation speed | Run 模式的实时倍速（逻辑仍精确按 tick 推进） |
| ESDF / truth paths / local obs / rejected candidates | 显示开关 |
| 点击画布自由空间 | 把点击位置投影到最近合法可选格子 → 更新最终导航目标（**不重置无人机动力学**，通知 5Hz 层重新评估，30Hz 层只看到更新后的当前局部目标） |

状态信息面板显示：**权威暂停状态（以 snapshot.paused 为准，服务失败不猜测）**、scene id / task id / seed、simulation tick / time、30Hz FSM state、macro active、selected side、side reason（`VISIBLE_LEFT_BETTER` / `VISIBLE_RIGHT_BETTER` / `AMBIGUOUS_DEFAULT_RIGHT`）、**左右平均可见自由距离**、**blocker association（MATCHED / NO_MATCH / AMBIGUOUS_MATCH）**、**blocker_passed_latched**、**固定路线车辆进度 / blocker 进度 / 投影距离**、planner success/failure、consecutive failure count、local target update event、macro tick event、**pending goal set / revision / position**、minimum observed clearance、truth minimum clearance、transition reason、task invalid / collision / goal reached，以及全部审计字段（含 `side_selected_using_current_patch`）。

**权威暂停状态**：`DebugSnapshot.paused` 由仿真节点发布真实状态（`launch paused:=false` 时 GUI 初始即显示 RUNNING）；GUI 标签**以 snapshot.paused 为准**——service 调用或 `response.success` 失败时**不**把标签改成请求值（等待权威 snapshot）；`StepSimulation` 成功后才显示 PAUSED，失败时也不自行猜测。

**GUI 只能通过 ROS 话题和服务控制仿真，不直接持有仿真核心裸指针。**

## 10. 使用方式

```bash
# 唯一入口（一次性启动两个节点 + 可选 RViz）
roslaunch il_2d_multiscale_debug expert_debug.launch

# 常用参数
roslaunch il_2d_multiscale_debug expert_debug.launch \
    seed:=42 paused:=true use_rviz:=true \
    config_file:=$(rospack find il_2d_multiscale_debug)/config/default.yaml \
    show_truth:=true
```

启动后默认暂停，用 GUI 按钮逐步推进观察。

## 11. 通过 seed 复现场景

- `seed:=...` 是 launch 参数，**以字符串传入**（`type="str"`），节点用 `readUint64Param` 解析（支持字符串或非负 int，检查溢出/非法；**seed 0 是合法 seed**，不会被隐式当作默认值）。
- **指定 seed 严格对应实际场景**：生成失败不会被自动替换为其它 seed（没有 seed+1…seed+8 的静默回退）；必须通过 `/generate_scene` 显式提交另一个 seed。
- `New Scene` 使用 seed 输入框的值（`use_default_seed=false`，0 合法）；
- 同一个 seed → 完全相同的障碍物布局、起终点、初始偏航（确定性由单一 `std::mt19937_64` 驱动）。
- `New Task In Same Scene` 使用 `use_default_seed=true`，从场景 seed 派生确定性任务 seed；若通过 `ros service call` 显式传 `use_default_seed=false` + `seed`，则**相同 scene seed + task seed 必复现相同任务**。
- `Reset Current Task` **不使用任何 seed**：它重启当前完全相同任务（同 scene/start/goal/yaw/task_id），`ResetTask.srv` 里的 `seed`/`use_default_seed` 仅为接口兼容保留、被节点忽略；task_counter 不递增。
- **首个任务 seed 与计数器无关**：场景提交后的第一个任务 seed 恒为固定标签 `kSceneFirstTaskTag` 派生的值（不依赖 `task_counter_`），相同 scene seed 在不同任务历史下始终产生相同的初始任务。

## 12. 点击修改导航目标

在画布上左键点击自由空间 → 点击位置被投影到最近的合法可选格子（`esdf > safety_clearance` 且在主连通域内）→ 先保存为 **pending navigation goal**（GUI 画布显示半透明橙色“PENDING GOAL”标记），在**下一个 5Hz 边界**由上层正式接纳（`acceptNewGoal`，`accepted_goal_event` 递增，正式 final goal 标记更新）并重新评估：

- **每次成功的 `setNavigationGoal` 都递增 `pending_goal_revision`**——即使 pending 状态已经是 true，只要位置变化，pending 标记也会**重新发布**（`publishAll` 以 revision 变化为条件重发 scene 标记，DELETEALL 后按新位置绘制）；
- **正式接纳后 pending 标记被 DELETEALL 清除**，final goal 标记同一 tick 更新；`DebugSnapshot` 发布 `pending_goal_set` / `pending_goal_revision` / `pending_goal_position`；
- **终态下 `setNavigationGoal` 返回 `success=false, reason="episode terminal; reset or generate a new task"`**——不接收一个永远无法处理的 pending goal；
- 宏观中：**保留当前锁定侧、当前 blocker、固定物理同伦参考与 `blocker_passed_latched`（全部不因目标变化而清除）**；未越过 blocker 时用固定参考 + 当前 blocker + 新目标重建锁定侧路线，若锁定侧不可行则 `TASK_INVALID`（previous/current transition record 正确，不偷偷换侧）；已越过（latched）则改用普通全局路线继续引导，**不再 TASK_INVALID**；
- **不重置无人机速度 / 局部地图历史 / 规划器状态**。可用来调试“上层目标规划网络连续改变目标”的情况。

## 13. ROS 接口

**话题**（frame 均为 `world`；`show_truth:=false` 时不发布 truth ESDF / selectable mask / 障碍物真值 / LEFT / RIGHT / locked 特权路线，通用 debug marker 中的 blocker 也降级为瞬时局部证据簇，不泄漏匹配后的真值圆柱；GUI 的 truth 开关同时隐藏真值障碍物、起终点和全部特权路线，仍显示局部观测、局部轨迹与执行轨迹）：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/truth_esdf` | `nav_msgs/OccupancyGrid` | 全局真值 ESDF，场景/任务变化时 latch 发布（PRIVILEGED） |
| `/selectable_mask` | `nav_msgs/OccupancyGrid` | 可选空间掩码，latch |
| `/local_observation` | `nav_msgs/OccupancyGrid` | 当前局部观测（**合并后的历史地图**） |
| `/instantaneous_patch` | `nav_msgs/OccupancyGrid` | **本 tick 瞬时 FOV patch**（current_patch，合并前；GUI 橙色叠加显示，与历史地图语义区分） |
| `/executed_path` | `nav_msgs/Path` | 已执行轨迹 |
| `/local_plan` | `nav_msgs/Path` | 30Hz 选中局部轨迹 |
| `/left_privileged_path` / `/right_privileged_path` | `nav_msgs/Path` | LEFT / RIGHT 全局特权路线（PRIVILEGED/TRUTH） |
| `/locked_route` | `nav_msgs/Path` | 当前锁定路线 |
| `/rejected_candidates` | `visualization_msgs/MarkerArray` | 被拒绝候选轨迹（含 DELETEALL） |
| `/debug_markers` | `visualization_msgs/MarkerArray` | 车辆 / FOV(TRIANGLE_LIST) / 局部目标 / 阻挡簇 / 速度(ARROW 两点) / 文字标签（含 DELETEALL） |
| `/scene_obstacles` | `visualization_msgs/MarkerArray` | 障碍物 / 起点 / 目标 / 边界（PRIVILEGED/TRUTH，含 DELETEALL，任务变化时也更新） |
| `/debug_snapshot` | `il_2d_multiscale_debug/DebugSnapshot` | 轻量状态消息（含全部审计字段、tick 语义与转换信息） |

**服务**（全部返回 `success / reason / tick / scene_id / task_id`，`success` 为显式布尔，不再用 `reason=="ok"` 推断）：

`/set_paused`、`/step_simulation`（`steps` 或 `step_to_next_5hz`；原子暂停；`steps==0 && step_to_next_5hz==false` 返回失败）、`/reset_task`（`use_default_seed` + `seed`）、`/generate_scene`（`use_default_seed` + `seed`，成功返回 scene_id/seed 信息）、`/generate_task`（`use_default_seed` + `seed`）、`/set_navigation_goal`（成功即“pending”语义：`success=true, reason="goal pending; will be accepted at next 5Hz boundary"`）、`/set_sim_speed`、`/export_flight_log`。

`Export Current Task Log` 默认写入 `logging/output_directory`（`~/.ros/il_2d_multiscale_debug_logs/`；服务请求 `output_directory` 为空时也使用该参数，不再另写默认路径）。日志在核心中逐个真实 30Hz tick 记录，因此 `Step To Next 5Hz Tick` 内部经过的中间 tick 不会丢失；Reset / New Task / New Scene 会开始新的当前任务日志。CSV 文件头包含场景/任务 seed、边界、全部圆柱参数，并区分 `original_sampled_goal` / `initial_accepted_goal` / `final_goal_at_export`。v5/v6 的 intent/output/status 与 v6 附加列顺序保持不变；**v7 仅在末尾追加**：`selected_cost_obstacle_risk`、`local_corridor_blocked`、`first_blocking_obstacle_distance`、`predicted_closest_clearance`、`time_to_collision`、`obstacle_risk_cost`、`avoidance_strength`、`avoidance_active`、`local_target_distance`、`macro_route_progress`、`macro_guide_lookahead`、`macro_guide_update_reason`、`macro_no_progress_duration`。原 `progress_qualified` 保留并严格等于最终输出进展；原 `min_observed_clearance` / `truth_min_clearance` 继续表示回合累计最小值。从日志可直接区分：障碍物首次被观测、首次阻塞预测走廊（`local_corridor_blocked` / `first_blocking_obstacle_distance`）、专家首次产生避让动作（`avoidance_active` / `obstacle_risk_cost`）、避让是减速/横移/转向（intent/output 分量）、为什么进入宏观（failure_reason + planner_status + FSM）、为什么更新宏观引导点（`macro_guide_update_reason` / `macro_guide_lookahead`）、为什么出现零输出（`stationary_selection_reason` / `planner_status`）。导出不改变暂停/FSM状态。**格式版本为 v7**（`# format,il_2d_multiscale_debug_flight_log_v7`）。

**启动时清理历史日志**：`debug_simulation_node` 启动时（仅该节点，GUI 节点不清理）删除 `logging/output_directory` **第一层**中满足 `logging/filename_prefix`（默认 `flight_log_`）开头且扩展名严格为 `.csv` 的**普通文件**；**非递归**，不删除目录/符号链接/其他文件；若目录为空、为文件系统根、为 HOME 本身、或为不安全路径则 `ROS_WARN` 并拒绝清理；目录不存在时安全创建；每个删除失败 `ROS_WARN` 但不会崩溃；最后输出清理目录与删除数量。`logging/clear_on_start: false` 时完全不删除。只清理程序自己的导出目录，不触碰用户手工复制到别处的日志。

**GUI 判断**同时检查 ROS 调用成功与 `response.success`，失败时显示 `response.reason`。

## 14. 关键审计字段

`used_truth_by_local_planner`、`used_global_esdf_by_local_planner`、`used_global_path_by_local_planner`（恒 false）、`macro_used_privileged_esdf`、`side_selected_from_visible_evidence`、`side_selected_using_current_patch`、`side_ambiguous_defaulted_right`、`local_target_update_event`、`macro_enter_event`、`macro_exit_event`、`obstacle_first_observed_event`、`immediate_avoidance_event`、`emergency_brake_event`。全部在 `DebugSnapshot.msg` 与 GUI 状态面板中显示，供人工检查信息泄漏与专家时序。

**候选拒绝诊断**（每 tick）：`reject_not_known_free` / `reject_outside_current_fov` / `reject_observed_clearance_too_small` / `reject_no_progress` / `reject_other` / `reject_insufficient_braking_clearance`（之和 = `rejected_candidate_count`）。**LOCAL_UNKNOWN_RECOVERY**：`unknown_recovery_ticks` / `unknown_recovery_active` / `unknown_recovery_episode_count`。两者均进入 DebugSnapshot.msg、Qt 面板与飞行日志 CSV。**v4 软净空/动态包络诊断**（见 §5.1~5.4）：`selected_soft_min_clearance_m`、`selected_dynamic_required_clearance_m`、`selected_closing_speed_mps`、`handoff_clearance_m`、`dynamic_clearance_blocked`、`local_limit_cycle_detected`、`dynamic_window_candidate_count`、`start_clearance_recovery_used`。**v5 intent/output/进展/状态诊断**：`selected_intent_vx_body` / `selected_intent_vy_body` / `selected_intent_yaw_rate`、`selected_output_vx_body` / `selected_output_vy_body` / `selected_output_yaw_rate` / `selected_output_speed_mps`、`nominal_progress_m`、`executable_progress_m`、`safe_prefix_duration_s`、`progress_qualified`、`planner_status` / `planner_status_name`、`stationary_candidate_selected` / `stationary_selection_reason`、`target_discontinuity_reset`、`target_mission_revision`。**v7 提前避障风险/滚动引导点诊断**：`selected_cost_obstacle_risk`、`local_corridor_blocked`、`first_blocking_obstacle_distance`、`predicted_closest_clearance`、`time_to_collision`、`obstacle_risk_cost`、`avoidance_strength`、`avoidance_active`、`local_target_distance`、`macro_route_progress`、`macro_guide_lookahead`、`macro_guide_update_reason`、`macro_no_progress_duration`。GUI 显示当前实际净空、动态要求净空与差值、风险/避让强度、宏观引导前视与更新原因，日志能区分：硬 0.5 m 违规（`reject_observed_clearance_too_small`）、动态制动净空违规（`reject_insufficient_braking_clearance` / `dynamic_clearance_blocked`）、UNKNOWN/FOV 无候选（`NO_SAFE_CANDIDATE` + unknown recovery）、控制振荡/停滞（`local_limit_cycle_detected`）、**停滞无进展（`STALLED_WITHOUT_PROGRESS`，30Hz 内部回归，不触发宏观）**、**走廊受阻与提前避让（`local_corridor_blocked` / `avoidance_active` / `obstacle_risk_cost`）**、5Hz 宏观接管（FSM 状态）。

**宏观 A\* 起点恢复（§9，防御性）**：宏观 A\* 起点所在栅格因 0.1 m 离散误差略低于 macro/local handoff clearance（0.65 m），但连续起点仍高于 `scene_safety_clearance`（0.5 m）时：
1. 不立即把任务判为 `TASK_INVALID`；
2. 在有限半径（`macro/start_recovery_max_radius_m`，默认 0.5 m）内做确定性有界螺旋搜索，找最近 route-clear 合法栅格（有限节点数）；
3. 构造连续起点 → 合法栅格的短恢复连接段，且该段连续采样满足 `scene_safety_clearance`（不穿过障碍物、不低于基础净空的格子）；
4. 进入合法栅格后，其余 A* 路线仍严格满足 `scene_safety_clearance + route_clearance_margin + clearance_discretization_margin_m`；
5. 恢复搜索**不允许静默切换 LEFT/RIGHT 侧**（搜索侧中立，A* 侧约束/同伦仍用固定参考）；
6. **不允许**“直接忽略起点净空检查”式修复；
7. `Route2D` 用显式 C++ 字段表达恢复前缀（`recovery_prefix_length` / `start_clearance_recovery_used`），`routeSafe()` 对前 `recovery_prefix_length` 米按基础净空、其余按宏观路线净空校验；`buildSideRoute` 与 `planPlainRoute` 都采用该机制，并在日志/GUI 中记录 `start_clearance_recovery_used`。

**局部目标事件语义（VALUE-CHANGE 规则）**：`effective_local_target_event` 是**唯一、单调递增**的事件号，**只在 30Hz 实际看到的局部目标值变化超过 `macro/local_target_event_tolerance_m`（默认 0.05m）时递增一次**——包括：5Hz 宏观专家发布新局部目标、进入宏观模式、退出宏观模式并恢复最终目标、5Hz 层正式接纳新的最终导航目标。若重算后的目标与之前数值一致则不增加事件。同一个 5Hz 边界内“接纳目标 + 刷新宏观路线 + 发布宏观目标”只会产生**一个**事件（不会重复计数）。事件号在宏观退出时**不会回退**。

**mission revision（v5，规划器记忆重置的唯二来源之一）**：reset/新任务会直接重置 FSM 与局部规划器；运行中 `LocalTarget.mission_revision` 在**每次正式接纳新的最终导航目标 revision**、进入宏观引导、退出宏观引导时递增。30Hz 规划器的**命令连续性与停滞窗口只在该值变化、或目标跳变超过 `local_planner/target_discontinuity_reset_m` 时重置**；**普通 update_event 数值变化（5Hz 连续目标网络滚动）绝不会清空平滑连续性或停滞检测**。相邻 tick 的 executable output 始终参与 jerk/smoothing 计算，即使目标点小幅连续移动。宏观退出预检使用退出后的下一 mission revision，避免继承宏观 guide 的命令连续性代价。`target_mission_revision` / `target_discontinuity_reset` 进入 CSV v5 与 GUI。

**blocker passed 判定（latched）**：进入宏观时保存**固定参考路线** + blocker 中心在该路线上的弧长；此后用**窗口化单调投影**跟踪车辆进度（相邻段/弧长窗口、单 tick 增量上限、侧向投影 > `blocker_projection_max_dist_m` 保持旧进度）。一旦 `progress > blocker_progress + blocker_clear_dist_m`，`blocker_passed_latched` 置位且**不回退**；latched 后宏观改用**普通全局 ESDF 路线**继续引导，不再强制原 blocker 的 gateway，也不再因固定侧路线不可行而 TASK_INVALID。

固定侧路线的 A* 偏置、gateway 和连续同伦检查共同使用宏观进入时锁定的 `entry_axis + entry_blocker_center`；分段路径的起点不会重新平移左右分界。相关阈值均在 YAML 中配置：`macro/route_side_bias`、`macro/homotopy_side_tolerance_m`、`macro/gateway_projection_radius_m`。任务拒绝采样总预算由 `task_sampling/max_sampling_attempts` 配置。

`DebugSnapshot.msg` 额外发布：`simulation_initialized`、**`paused`（权威）**、`processing_tick`、`previous_fsm_state` / `previous_fsm_state_name`、`transition_tick`、`accepted_goal_event`、`macro_reentry_guard_ticks`（剩余 guard，30Hz tick）、`local_target_updated_this_tick`、`macro_tick_ran_this_tick`、**`blocker_association` / `blocker_association_name`、`blocker_passed_latched`、`start_clearance_recovery_used`、`entry_vehicle_progress`、`entry_blocker_progress`、`entry_projection_dist`、`entry_segment_index`、`entry_progress_delta` / `entry_progress_max_delta`、`pending_goal_set` / `pending_goal_revision` / `pending_goal_position`、`side_left_score` / `side_right_score`、`side_selected_using_current_patch`**，以及软净空、动态包络、intent/output/status诊断。v6 追加 `candidate_progress_qualified` / `output_progress_qualified` 与瞬时 `current_observed_clearance` / `current_truth_clearance`，避免把监督前候选进展误读成实际输出进展、或把回合累计最小净空误读成当前位置净空。v7 追加**提前避障风险**字段（`local_corridor_blocked` / `first_blocking_obstacle_distance` / `predicted_closest_clearance` / `time_to_collision` / `obstacle_risk_cost` / `avoidance_strength` / `avoidance_active` / `local_target_distance` / `selected_cost_obstacle_risk`）与**滚动宏观引导点**字段（`macro_route_progress` / `macro_guide_lookahead` / `macro_guide_update_reason` / `macro_no_progress_duration`）。

## 15. 参数配置

所有行为参数都在 `config/default.yaml`（由 launch 载入两个节点的私有命名空间，C++ 按同名 key 读取）：区域/ESDF、障碍物密度与尺度、安全通道、场景与任务采样预算、因果资格审查、FOV/历史地图、30Hz 候选采样与运动约束、转向滞回、终点连续制动、目标跟踪代价权重与并列规则、5Hz blocker 走廊/侧证据/显式关联、固定同伦路线、越障进度窗口、退出与重入、车辆静止阈值、日志以及 GUI 开关。本轮末端收敛相关参数为 `terminal_control_distance`、`terminal_speed_gain`、`terminal_max_speed`、`terminal_max_yaw_rate` 和 `scoring_horizon_s`。**v4 新增参数**（均有默认值、读取与合法性检查）：`local_planner/soft_clearance_radius_m`（2.0）、`clearance_discretization_margin_m`（0.05）、`obstacle_reaction_time_s`（0.20）、`control_period_s`（1/30）、`max_allowed_regress_m`（0.05）、`limit_cycle_window_ticks`（15）、`limit_cycle_net_progress_m`（0.10）、`limit_cycle_min_blocked_ticks`（8）、`limit_cycle_lateral_flip_count`（2）、`macro/start_recovery_max_radius_m`（0.5）。**v5 新增参数**：`local_planner/min_progress_speed_mps`（0.03）、`min_progress_epsilon_m`（0.01）、`target_discontinuity_reset_m`（1.5）。**v7 新增参数**：`local_planner/nominal_clearance_m`（0.65，期望净空，仅路径质量参考）、`risk_corridor_half_width`（1.0）、`risk_distance_horizon_m`（5.0）、`risk_ttc_horizon_s`（2.5）、`risk_trajectory_radius_m`（1.0）、`avoidance_active_threshold`（0.10）、`cost_weights/obstacle_risk`（3.0）；`macro/guide_min_distance_m`（1.5）、`guide_lookahead_time_gap_s`（1.0）、`guide_hysteresis_m`（0.3）、`guide_fov_margin_deg`（10.0）、`no_progress_threshold_m`（0.05）、`no_progress_duration_threshold_s`（1.0）。采样集 v7 加密：`speed_samples`（加 0.3）、`lateral_ratio_samples`（加 ±0.05/±0.15）、`yaw_rate_samples`（加 ±0.15）。

## 16. 文件结构（摘要）

```
il_2d_multiscale_debug/
  CMakeLists.txt  package.xml  README.md
  config/default.yaml
  launch/expert_debug.launch
  rviz/expert_debug.rviz
  msg/DebugSnapshot.msg  msg/PlannerCandidate.msg
  srv/SetPaused.srv StepSimulation.srv ResetTask.srv
      GenerateScene.srv GenerateTask.srv SetNavigationGoal.srv SetSimSpeed.srv
  include/il_2d_multiscale_debug/
    types.hpp params_io.hpp truth_esdf_2d.hpp scene_generator.hpp
    connectivity_analyzer.hpp task_sampler.hpp fov_raycaster_2d.hpp
    observed_grid_2d.hpp local_planner_30hz.hpp left_right_route_planner.hpp
    macro_expert_5hz.hpp hierarchical_expert_fsm.hpp planar_vehicle_simulator.hpp
    debug_simulation.hpp debug_panel.hpp
  src/（与上对应的 .cpp）+ debug_simulation_node.cpp + debug_panel_node.cpp
```

## 17. 说明与限制

- 本包只做二维调试，**不写训练数据**；如需数据请在三维数据集包中另行实现。
- 依赖：`roscpp std_msgs geometry_msgs nav_msgs visualization_msgs`、`message_generation/message_runtime`、Qt5 Widgets（Debian 包 `qtbase5-dev`）、`rviz`（可选）。
- 该包**独立编译运行**，与 `il_dataset`、`il_multiscale_dataset`、`flightmare` 均无依赖关系，也没有修改任何现有包。
- **尚未编译、尚未运行、尚未测试。**
