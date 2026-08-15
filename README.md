# il_2d_multiscale_debug

完全二维、确定性、可逐步推进的 **5Hz 可见性目标修正器 + 30Hz 局部避障专家** 协同行为调试包。

这是一个**纯 2D、独立、轻量**的专家行为调试器：**不调用 Flightmare / Unity**，**不写训练数据**，只用于在固定步长二维仿真中观察、步进和调试两层专家的协作、状态机与信息边界。

> **状态：本包仅为代码与配置，尚未编译、尚未运行、尚未测试。**
> 作者没有执行任何构建 / 运行 / 测试命令，也不保证一次通过编译。请自行在 catkin 工作空间中编译后再使用。
>
> **v9（本版）**：5Hz 专家已从“5Hz 宏观绕行规划器”彻底改造为 **“局部可观测性判断器 + 目标修正器”**（`VisibilityTargetCorrector`），并新增 **`EffectiveTargetAdapter`** 作为 30Hz 专家与未来 30Hz 学生共享的唯一信息瓶颈（机体系单位方向 + 归一化距离 + 世界目标点）。当前收尾版同时为30Hz候选库补充了对称的低速FOV边缘逃逸原语；场景/任务生成、碰撞裁判与全局 ESDF 的离线预检未改动。

---

## 1. 包的目的

在真实(三维、带深度传感器、Flightmare)系统里，5Hz 专家与 30Hz 局部专家之间的时序、信息边界与“何时该介入”很难单独观察。本包把整个系统压到二维平面上，用确定性仿真还原同样的层级结构：

```
original_goal
      ↓
5Hz VisibilityTargetCorrector（局部可观测性判断器 + 目标修正器，
         只在 tick % 6 == 0 运行，零阶保持）
      ├─ PASS_THROUGH        （FOV 已足够，原样输出原始目标）
      ├─ NORMAL_CORRECTION   （锁定侧的量化观察 frontier）
      ├─ TURN_LEFT / TURN_RIGHT（世界系锁存的有限角度步进，距离1纯旋转）
      ↓ 5Hz ZOH
EffectiveTargetAdapter（每个真实 30Hz tick 运行）
      ↓
LocalTarget 世界点（供当前 C++ 30Hz 专家）
      + 机体系方向单位向量 + 归一化距离（供未来 30Hz 学生）
      ↓
30Hz Local Expert（只能使用当前状态、局部观测、当前局部目标）
      ↓
body velocity + yaw rate
      ↓
2D deterministic simulator（dt = 1/30 s，swept 碰撞检查）
```

所有随机过程由显式 `uint64_t seed` 驱动：**同一个 seed 产生完全相同的场景、起终点与初始偏航**。

## 2. 5Hz / 30Hz 架构与职责边界

### 30Hz 专家（`LocalPlanner30Hz`，本轮未修改）

- 始终只跟踪一个普通的有效目标（`LocalTarget`）；
- 目标方向明显偏离 FOV 时按现有逻辑**原地转向**（TURN_TO_TARGET 带滞回）；
- 障碍物已被当前 FOV 观测、局部绕行出口可见时，由 30Hz **自行采样和选择避障轨迹**；
- 30Hz 自行负责：轨迹 known-free、当前 FOV、硬净空、动态制动净空、提前风险代价、平滑性和进展；
- **30Hz 不知道当前目标是原始目标还是 5Hz 修正目标**；不接收 side、correction mode、blocker track、5Hz 状态；不允许为 5Hz 目标设置特殊避障规则。

### 5Hz 专家（`VisibilityTargetCorrector`，v9 新职责）

“**局部可观测性判断器 + 目标修正器**”。它不规划完整绕行路线、不持续生成绕障轨迹、不判断 30Hz 是否失败，只判断：

> “当前FOV与由历史FOV融合得到的衰减局部地图，是否已经足够让30Hz自行完成局部避障？”

- **足够且当前未处于修正 episode** → `PASS_THROUGH`：原样输出原始目标语义，不介入 30Hz 避障；已处于修正 episode 时还必须等待原始目标回到安全交接角；
- **不足** → 根据当前局部观测选择左/右并锁定（`locked_side`），临时修正30Hz跟踪目标；必要时输出由5Hz逐步复核的有限左转/右转指令改变视野（`TURN_LEFT` / `TURN_RIGHT`）；
- 当前 FOV 首次出现可用绕行出口后，若存在安全普通 frontier，5Hz 必须先保持同一绕行侧并由 `TURN_LEFT/RIGHT` 切换为 FOV 内的 `NORMAL_CORRECTION`，吸收残余角速度并引导 30Hz 产生绕行平移；只有“局部避障已可观测”“原始目标已回到普通方向安全锥”且“实际角速度已降至30Hz转向退出阈值以下”同时成立时，才**立即释放修正、恢复原始目标**。

**严格边界**：“看到障碍物”不等于“5Hz 接管”。只有“看到阻挡，但看不清如何绕，30Hz 缺少局部可观测信息（遮挡 / UNKNOWN / FOV 边界截断）”才允许 5Hz 修正目标。

### 严格禁止 5Hz 读取 30Hz 结果

5Hz 决策运行时只允许使用：

- `VehicleState2D`（位姿、速度、yaw、yaw_rate）；
- `original final goal`；
- `current_patch`（当前瞬时 FOV 帧）；
- `local_history_map`（仅由历史FOV融合、带超时衰减的局部地图）；
- 当前 tick；
- 5Hz 自身内部记忆：`correction_active`、`locked_side`、当前 direction token、已观测局部 blocker 轮廓、可观测性稳定计数、上一个修正结果。

5Hz **禁止**读取或接收：`PlannerResult`、`PreviewResult`、`FailureReason`、`PlannerStatus`、`previewPlan()`、`turn_mode`、`emergency_brake`、`blocked_observed`、30Hz 候选轨迹、30Hz 输出 vx/vy/yaw_rate、30Hz 连续失败次数、30Hz limit cycle 结果、全局 ESDF、`Scene2D`、真值障碍物 id/圆心/半径、全局 A* 或全局左右路线。

已**删除/停止使用**：`canEnterMacro()`（基于 30Hz 失败触发）、`BLOCKED_BY_OBSERVED_OBSTACLE` 持续时间触发 5Hz、`last_macro_turn_mode_`、5Hz 用 `previewPlan` 决定退出、5Hz 依据 30Hz no-progress 推进 guide、5Hz 持续规划完整绕行 guide、宏观进入/退出依据 30Hz 状态。`FsmInput` 不再携带 scene/esdf（v8 起已如此，v9 保持）。

## 3. 信息边界（信息泄漏检查的核心）

### 30Hz 局部专家只能收到：

- `VehicleState2D`（位置、yaw、世界系速度、yaw rate）
- `LocalObservation`（世界系栅格：FREE / OCCUPIED / UNKNOWN，带参数化短时历史）
- `LocalTarget`（仅：局部目标位置 + `update_event`（directive 语义更新事件，仅用于日志/可视化）+ `mission_revision`（**只有该值变化才重置规划器记忆**）；兼容字段 `is_macro_guide` 在 v9 恒为 false，不向 30Hz 泄露修正模式）

接口层面强制隔离：

```cpp
PlannerResult LocalPlanner30Hz::plan(
    const VehicleState2D& state,
    const LocalObservation& obs,
    const LocalTarget& target);
```

### 5Hz `VisibilityTargetCorrector` 可以读取：

1. `VehicleState2D`；
2. 最终导航目标（original goal）；
3. **current_patch**（本 tick 瞬时 FOV 帧，用于新证据和首次选边）；
4. **local_history_map**（只由已经观测过的局部FOV融合，按年龄退化为UNKNOWN）；
5. 当前 tick；
6. 5Hz 自身的长期记忆（correction_active / locked_side / 稳定性计数 / 重入滞回 / 事件计数 / 上一 directive）。

**接口层面强制隔离（v9）**：`VisibilityTargetCorrector` 的任何运行时方法都**不接收** `Scene2D` / `TruthEsdf2D` / `PlannerResult` / `PreviewResult` / 30Hz 状态（结构上无法访问）；`FsmInput` 不携带 scene / esdf。

> **因果性保证**：相同局部观测历史 + 车辆状态 + 原始目标 + 5Hz 内部状态 ⇒ 完全相同的 directive；改变**从未被观测过**的 FOV 外障碍物**不可能**改变当前 5Hz 输出。

**瞬时帧 vs 历史地图**：每个 30Hz tick 的 `FovRaycaster2D` 输出称为 **current_patch**（合并前瞬时帧），`ObservedGrid2D` 合并后的持久地图称为 **local_history_map**。FSM 同时携带两者：5Hz用current_patch约束首次接管和选边，用local_history_map保持已观测旁路的连续性；30Hz规划器也只使用local_history_map。两者都不能访问全局真值。

### 观测合成说明

局部观测由 `FovRaycaster2D` 用**二维射线投射**对真实圆柱表面求交生成：射线命中第一个障碍物表面后，后方区域保持 UNKNOWN；FOV 外保持 UNKNOWN；**不**允许因为知道圆心半径就把整个圆填入局部地图。真值只用于“模拟传感器测量过程”与裁判/可视化，绝不反馈给 30Hz 规划器，也不进入 v9 5Hz 运行时调用图。

**统一全局栅格基准**：瞬时 patch 与历史地图使用同一个全局栅格（`worldToGrid`：`ix = floor((world − min_bounds)/resolution)`；`gridCellCenter`：`min_bounds + (ix+0.5)*res`，全包唯一约定，放在 `types.hpp`）。raycast 完成后显式把传感器原点所在格标记为 FREE。

## 4. 状态机（v9）

| 状态 | 含义 |
|---|---|
| `DIRECT_LOCAL` | 默认：30Hz 专家直接跟踪 `EffectiveTargetAdapter` 给出的有效目标（PASS_THROUGH 时即原始目标方向 + 截断距离） |
| `TURN_TO_TARGET` | 目标方向超出 FOV，停止平移、只转 yaw（带滞回） |
| `GOAL_REACHED` / `TASK_INVALID` / `COLLISION` / `TIMEOUT` | 终态 |

v9 **删除**了旧宏观状态（`LOCAL_BLOCKED_PENDING` / `MACRO_SELECT_SIDE` / `MACRO_GUIDANCE` / `MACRO_EXIT_PENDING`）：5Hz 修正不再由 30Hz 失败触发，也不再需要“宏观引导”状态。

每次状态切换都会保存并发布：previous/current state、transition reason、simulation tick、30Hz 失败计数、side（v9 = 5Hz locked side）、blocker id（v9 恒 -1，legacy）、local target update event。`previous_fsm_state` / `transition_tick` / `transition_reason` 三者取自同一条 `TransitionRecord`。**终态也完整填充** v9 修正/编码/可观测性/审计字段。

**FOV 滞回（TURN_TO_TARGET，30Hz 本体）**：目标方位角 `|bearing| > turn_enter_deg`（42°）时进入转向；退出必须同时满足 `|bearing| ≤ turn_exit_deg`（8°）且 `|yaw_rate| ≤ turn_exit_max_yaw_rate`。近目标放宽：距离 ≤ `near_goal_heading_relax_distance` 时进入阈值放宽到 `near_goal_turn_enter_deg`（75°）。

### 5Hz 修正进入 / 退出（只由 5Hz 自己的可观测性判断决定）

**进入修正**（连续 `correction_enter_stable_ticks` 个真实 5Hz 周期满足）：

- 车辆→original goal 的局部走廊被 current_patch 中的 OCCUPIED 阻挡；
- 当前 FOV **无法提供任何足够明确的局部绕行出口**（原因：遮挡 / UNKNOWN / FOV 边界截断）；
- 目标本身在 FOV 外**不会**触发 5Hz（30Hz 按原设计自行转向）。

**退出修正**（第一次在真实5Hz边界同时满足以下条件时立即执行，**不调用30Hz previewPlan、不读取30Hz状态**）：

- 当前目标走廊已经清楚，或当前FOV内至少存在一侧满足净空、known-free连通、正向进展且未被UNKNOWN/FOV边界截断的局部绕行出口；
- original goal 的机体系方位位于普通方向安全锥内，即 `abs(goal_bearing) <= FOV/2 - turn_ray_margin`；
- 车辆实际角速度满足 `abs(state.yaw_rate) <= local_planner/turn_exit_max_yaw_rate`（默认 `0.15 rad/s`）；该值直接来自允许的 `VehicleState2D`，不读取30Hz规划结果；
- 若绕行出口已经可见但 original goal 尚未进入安全锥，则不 `PASS_THROUGH`：保持本 episode 的 `locked_side`，优先输出该侧 `NORMAL_CORRECTION` 让30Hz向绕行出口平移；只有该侧暂时找不到可直接跟踪的安全普通 frontier 时才继续 `TURN_LEFT/RIGHT`。
- 若上一条指令仍是 `TURN_LEFT/RIGHT`，且当前可见绕行出口对应安全普通 frontier，则本5Hz周期强制输出一次 `NORMAL_CORRECTION` 作为交接桥；即使其它退出条件已经满足，也不能在同一周期直接 `TURN→PASS_THROUGH`。
- 不使用“等待稳定退出”计数；上述交接条件首次同时成立就释放，边界闪烁由退出后的重入保护处理。

退出后：directive 立即切换为 `PASS_THROUGH`；effective target 恢复 original goal；清除 locked side 与修正状态；防边界闪烁只由普通重入滞回（`macro_reentry_guard_ticks`，30Hz tick单位）承担；**不修改 mission_revision、不重置30Hz规划器**。

## 5. 目标编码协议（v9）

未来 30Hz 学生的输入始终只有：

- 机体系方向单位向量：`(direction_x_body, direction_y_body)`
- 一个归一化距离：`normalized_distance`

不向 30Hz 学生增加 mode/token/side 等额外输入。

$$
R = \text{observation/range\_m},\qquad reserve = \text{target\_encoding/normal\_distance\_reserve\_m (0.5)}
$$

**普通目标编码**：

$$
normal\_distance = \frac{\min(real\_target\_distance,\ R - reserve)}{R}
$$

普通目标的最大归一化距离**严格小于 1**：

$$
normal\_distance\_max = \frac{R - reserve}{R} = \frac{6-0.5}{6} \approx 0.9167
$$

**纯旋转编码**：

- `TURN_LEFT` → 发布时位于FOV外左侧、随后锁存在世界系的方向，`normalized_distance = 1.0`；
- `TURN_RIGHT` → 发布时位于FOV外右侧、随后锁存在世界系的方向，`normalized_distance = 1.0`。

方向在每个30Hz tick根据实时姿态重新转换回机体系，因此会随旋转逐渐进入FOV；距离1始终禁止平移。到达FOV后，下一次5Hz边界基于更新后的局部地图决定发布新的有限转向步、普通修正目标或退出修正。

**到达终点**：`normalized_distance = 0`，方向使用固定规范占位值 `(1,0)`（30Hz 终点控制以距离为准，不能让接近零距离时的随机方向影响标签）。

**关键规则**：

- 不能用神经网络浮点输出是否“刚好等于 1”判断旋转；旋转由 5Hz 方向分类的**特殊类别**决定；
- 解码器选择 TURN_LEFT/RIGHT 后，**强制**给 30Hz 写入精确的 `distance = 1`；
- 普通方向类别的距离**强制 clamp** 到 `normal_distance_max`；
- TURN 类别下 5Hz 的距离回归输出被忽略。

### 方向类别（学生分类）

```
class 0           = TURN_LEFT
class 1 .. N      = 从左到右连续排列的普通 FOV 内方向 bin（N = direction_bin_count，奇数）
class N + 1       = TURN_RIGHT
```

普通 bin 对称覆盖 $[-FOV/2 + margin,\ +FOV/2 - margin]$，**必须包含 0° 方向**，使用确定性量化规则（最近邻，`lround`，并列远离零舍入，左右无偏）。`EffectiveTargetAdapter::quantizeBearing` / `decodeDirectionToken` 是专家与学生共享的**同一套**量化/解码。

这些 token 只存在于 5Hz 输出和适配器内部，**绝不作为额外输入传给 30Hz**。

## 6. 5Hz 局部可观测性判断（确定性、局部、因果）

`VisibilityTargetCorrector::assessObservability`使用仅由历史FOV融合并按年龄衰减的`local_history_map`；首次接管和左右选边仍要求`current_patch`中的新鲜阻挡证据：

1. **目标方向在 FOV 外** → 记为 `GOAL_OUTSIDE_FOV`，不能宣称当前目标走廊已经可观测；但5Hz不因此进入修正，仍输出`PASS_THROUGH`，由30Hz按原设计自行转向原始目标。
2. **检查车辆→original goal 的局部走廊**（只到感知距离）：只把已经局部观测到且尚未过期的OCCUPIED当作阻挡；**UNKNOWN不能当作FREE**。
3. **目标方向位于FOV内且走廊未被观测障碍阻挡** → `local_avoidance_observable = true` → `PASS_THROUGH`。目标位于FOV外时，单纯“没有检测到走廊障碍”不能算作已观测清楚。
4. **走廊被阻挡**：
   - 从局部历史地图提取最近前向blocker的已观测局部簇；
   - 在局部历史地图内建立按无人机净空膨胀的known-free网格，并从车辆位置对known-FREE且满足膨胀净空的格子做8连通flood-fill（仅允许车辆附近的有界恢复前缀）；
   - 分别检查 blocker 左、右是否存在：与车辆在共享静态交接净空（默认0.65m）膨胀后的 known-free 网格中连通；位于当前 FOV（带 margin）；**不是只停在 UNKNOWN/FOV 截断边缘**（端点之后仍需 ≥ `observable_unknown_margin_cells` 格的 known-FREE 延续）；能获得相对 original goal 的正向进展（≥ `observable_frontier_min_progress_m`）；绕行方向与 original-goal 方向夹角不超过 `FOV/2`；且**严格位于 blocker 最近表面之后**（真正的绕行出口）。
5. **只要当前 FOV 中至少有一侧局部绕行出口已经清楚可见** → `local_avoidance_observable = true`。未处于修正 episode 时仍保持 `PASS_THROUGH`；已处于修正 episode 时，还要检查 original goal 是否回到普通方向安全锥：未回到则继续用锁定侧的 `NORMAL_CORRECTION` 引导平移，回到后才 `PASS_THROUGH`。
6. **只有**“目标走廊被阻挡 **且** 当前 FOV 无任何明确绕行出口”才修正。

进入修正时按 §7 锁定 LEFT/RIGHT（无法区分 → 固定 RIGHT，episode 内不换侧）。

## 7. 5Hz 修正策略与编码/解码

**A. NORMAL_CORRECTION**（锁定侧存在当前 FOV 内、known-free、适合作为观察方向的普通 frontier）：

1. 专家选择连续方向/距离（确定性候选采样：目标方位角向该侧 FOV 边缘、`observable_frontier_min_distance_m` ~ R-margin）；
2. **必须量化到学生普通方向 bin**（`quantizeBearing`）；
3. 使用 **bin 中心方向** 和 **clamp 后的距离**（≤ `normal_distance_max`）重建 `corrected_target_world`；
4. `corrected_target_world` 在该 5Hz 周期内**锁存在世界坐标系**；
5. 每个 30Hz tick 由适配器用实时 `current_position/current_yaw` 重新计算车辆到该世界点的方向/距离，转成机体系单位方向与归一化距离——**不能只保持 5Hz 生成时的机体系方向**（车辆旋转后方向必须实时变化）；
6. 交给 30Hz 专家的 `LocalTarget` 沿“当前位置→锁存点”的实时方向按同一 `R-reserve` 上限截断；因此即使车辆暂时远离锁存点，C++ 专家也不会获得比学生距离编码更多的信息。

**B. TURN_LEFT / TURN_RIGHT**（当前 FOV 完全遮挡 / 无安全普通观察 frontier）：

- 5Hz发布时先在指定侧构造`±(FOV/2+margin)`的机体系方向，再立即转换并锁存为世界系方向；
- 每个30Hz tick使用实时yaw把该世界方向重新转换回机体系，所以随着车辆旋转，目标方位误差逐渐减小并进入FOV；
- `normalized_distance = 1.0`是30Hz输入契约中保留的纯旋转值：C++专家和未来学生都只能转向，不能产生主动平移；
- 若上一有限转向目标仍在FOV外，下一次5Hz保持同一世界方向；进入FOV后，5Hz根据最新瞬时帧和局部历史地图决定发布新的有限转向步、切换普通修正目标或结束接管；
- 因此一次5Hz指令不再能够驱动车辆无限旋转。

**编码/解码闭环（专家 = 学生）**：

```
连续专家结果 → quantizeBearing → direction token
             → decodeDirectionToken（bin 中心 / TURN 固定射线）
             → clampNormalizedDistance（普通类 ≤ normal_distance_max，TURN = 1.0）
             → EffectiveTargetAdapter::encode → 30Hz LocalTarget
```

**禁止**：专家用连续高精度目标驱动 30Hz、日志只保存量化后的学生标签（否则训练轨迹不是学生动作能够复现的）。

## 8. `EffectiveTargetAdapter`（每个真实 30Hz tick）

把 5Hz 零阶保持的 directive 转换成：

1. 当前 C++ `LocalPlanner30Hz` 使用的 `LocalTarget` 世界坐标；
2. 未来 30Hz 学生使用的机体系单位方向和归一化距离；
3. 对应诊断字段。

- **PASS_THROUGH**：`delta = original_goal - pos`、`d = |delta|`、`d_clip = min(d, R - reserve)`、`effective_target_world = pos + normalize(delta)·d_clip`。真正的 goal reached 仍由 FSM 用 original_goal 判断，不能用截断点代替任务终点。
- **NORMAL_CORRECTION**：用锁存的 `corrected_target_world`，每个 30Hz tick 从实时位姿重算方向/距离并按 `R-reserve` 截断；`effective_target_world` 与归一化距离表达同一个截断目标。
- **TURN_LEFT/RIGHT**：锁存有限转向步的世界方向，每tick用实时姿态转回机体系（见 §7B），`normalized_distance`严格为1且禁止平移。
- **到达终点**：`normalized_distance = 0`、方向 `(1,0)`（规范占位）。

## 9. mission revision 与目标事件（v9）

- 新任务、新场景、**正式接受新的最终导航目标**：`mission_revision` 可以递增；
- 5Hz 进入修正 / NORMAL_CORRECTION 更新 / TURN_LEFT/RIGHT / 5Hz 释放修正恢复原始目标：**均不递增 mission_revision**；
- 5Hz 目标变化只更新 `target/directive update event`（`correction_enter_event` / `correction_exit_event` / `correction_update_event`）；
- 30Hz不应因为5Hz介入或退出而清空内部记忆；有限TURN步和普通修正目标都按世界系锁存，只有真实5Hz指令变化才更新目标事件。

## 10. 30Hz 局部专家行为

除原有对称速度/横移/yaw-rate格点外，候选库额外加入两档低速、位于 `±0.75·FOV/2` 与 `±0.90·FOV/2` 的对称逃逸原语，并为每个方向同时生成零yaw和同侧小yaw版本。这些原语只依赖当前位姿、配置和局部观测，仍通过完全相同的FOV、known-free、硬净空、动态制动净空及目标进展检查；其作用是填补“浅对角线不够绕开、下一档横移又立即出FOV”的采样空洞，避免近障碍静止后只剩STOP候选。

1. 普通目标方向不在FOV → `TURN_TO_TARGET`（带滞回）；`normalized_distance=1`无论方向是否已经进入FOV都保持零平移，只执行转向/角速度收敛并等待下一次5Hz决策。
2. 每个 tick 重新规划：速度 × 侧向 × yaw-rate 采样，多候选轨迹并比较代价。
3. 横向采样左右对称；候选预测与执行使用同一个共享运动学积分函数（`kinematics.hpp`）。
4. 候选安全性按**整条扫掠路径**检查：当前 FOV（可执行前缀）/ known-FREE / 硬净空（`min_clearance` 0.5m）/ 速度相关动态净空（`handoff_clearance + closing·reaction + closing²/2a`）。
5. 连续软净空代价、动态窗口、停滞/控制振荡检测器（限环检测）、progress 资格、分层选择、可执行安全前缀、连续提前避障风险、目标跟踪代价（terminal heading / velocity alignment / cross track）、终点连续制动等 v5-v8 机制**全部保持原样**。
6. 正在执行的轨迹被新观测障碍物截断 → 立即重新避障（`immediate_avoidance_event`）；停止距离不足 → 紧急制动。
7. 目标由上层（FSM + 适配器）每 tick 提供；目标改变不会重置速度、历史或规划器（mission_revision 除外）。

## 11. 左右定义与侧选择（v9）

- **统一二维左右定义**：`axis = goal - vehicle_position`，`cross(axis, p - pos) > 0` 为 LEFT、`< 0` 为 RIGHT。
- 进入修正时用**当前瞬时 FOV patch** 的成对对称射线（平均可见自由距离）判断哪侧更容易扩大可观测空间；能明确判断 → 选该侧；无法区分 → **固定 RIGHT**（`AMBIGUOUS_DEFAULT_RIGHT`）。
- correction episode 内**锁定 side，不能来回切换**。
- 不能读取全局路线长度或 FOV 外障碍物。

## 12. 场景与任务（v9 未修改）

- 区域默认 `[-20,20]×[-20,20] m`，ESDF 分辨率 `0.1 m`。
- 障碍物：二维圆，数量随机 `0~12`，半径随机 `0.1~8.0 m`（log-uniform）；大半径障碍物可形成长时间绕行任务。
- 任意两圆表面间距 ≥ `2*safety_clearance + passage_margin`；到区域边界留出 `boundary_margin + safety_clearance`。
- 场景生成显式成功/失败；`newScene` 事务化；失败保留原有效状态。
- 首个任务 seed 与历史计数器无关（`deriveTaskSeed(scene_seed, kSceneFirstTaskTag)`）。
- 起终点从主连通域采样，满足最小距离、ESDF 安全距离，并用全局 A* 做静态连通确认（**仅离线资格审查，禁止交给 30Hz 与 v9 5Hz 运行时**）。
- `episode_timeout_s: 0.0` 禁用时间上限（保持）。

## 13. 日志、消息与 GUI（v9）

### DebugSnapshot.msg / SimSnapshot / CSV v9 新增字段

- `target_correction_type` / `target_correction_type_name`
- `target_correction_active`
- `target_direction_token`
- `target_direction_x_body` / `target_direction_y_body`
- `target_distance_normalized`
- `effective_target_x` / `effective_target_y` / `effective_target_world_valid`
- `original_goal[2]`
- `observability_goal_inside_fov` / `observability_direct_corridor_blocked` / `observability_blocker_observed`
- `observability_left_bypass_visible` / `observability_right_bypass_visible`
- `observability_local_avoidance_observable`
- `observability_fov_boundary_truncated` / `observability_unknown_occluded`
- `observability_reason`
- `observability_left_score` / `observability_right_score`
- `correction_enter_event` / `correction_exit_event` / `correction_update_event`

CSV 格式版本递增为 **v9**（`# format,il_2d_multiscale_debug_flight_log_v9`），**只在旧列末尾追加新列**，不改变既有列顺序。

30Hz 数据每个真实 30Hz tick 记录适配器实际提供的：`target_direction_x_body`、`target_direction_y_body`、`target_distance_normalized`、`effective_target_world_x/y`、`target_correction_type`。5Hz 训练标签每个真实 5Hz 边界记录：`target_direction_token`、`target_distance_normalized`、correction/pass-through、side、可观测性诊断（CSV 中按 30Hz 行重复记录零阶保持值）。

### 旧字段处理（v9 LEGACY）

`left/right/locked_route`、`blocker*`、`blocker_passed_latched`、`start_clearance_recovery_used`、`entry_*`、`macro_route_progress`、`macro_guide_*`、`macro_no_progress_duration`、`macro_stable_exit_count`、`local_blocker_track_*`、`local_leave_progress_m`、`local_macro_route_valid`、`macro_enter_event`、`macro_exit_event`、`macro_active` 等旧字段**保留但标记为 LEGACY**：运行时不参与任何决策、恒为无效/空值；README 说明已弃用。

### GUI（debug_panel）

状态面板新增显示：

- 当前原始目标（`original_goal`）与当前 effective target（`local_target` 标记，即适配器输出）；
- 当前 30Hz 实际输入单位方向和归一化距离；
- `PASS_THROUGH` / `NORMAL_CORRECTION` / `TURN_LEFT` / `TURN_RIGHT`；
- LEFT/RIGHT latch；
- 为什么 5Hz 认为当前 FOV 不可观测（`observability_reason`）；
- 当前左右绕行出口是否可见；
- correction enter/exit 原因与事件计数。

**TURN_LEFT/RIGHT 可视化**：从无人机中心绘制世界系锁存方向在当前机体系中的实时射线（`view_rotation`，虚线箭头，标签 **“VIEW ROTATION ONLY”**）；它会随姿态变化逐渐进入FOV，但不是可平移的世界航点。**普通修正目标**：显示量化后实际执行的世界目标点（即 `local_target` 标记）。

## 14. GUI 按钮与操作（v9 未修改）

| 控件 | 作用 |
|---|---|
| Run / Pause | 开始 / 暂停 30Hz 定时器推进 |
| Single 30Hz Step | 只推进一个 1/30 s tick，完成后必定保持暂停（原子暂停操作） |
| Step To Next 5Hz Tick | 推进到下一次 5Hz 修正边界并完成该次 5Hz 决策（以 `macro_tick_event` 严格递增为停止条件），完成后必定保持暂停 |
| Reset Current Task | 重启当前完全相同任务（同场景/起点/终点/初始偏航/task_id），tick 清零，车辆/局部观测历史/已执行路径/FSM 全部重置 |
| New Task In Same Scene | 同场景采样新任务（task counter 递增） |
| New Scene | 用 seed 输入框的值生成全新场景 |
| Export Current Task Log | 导出当前任务从 tick 0 到当前时刻的完整 30Hz CSV 日志（v9 格式） |
| seed 输入框 | 指定场景/任务 seed |
| simulation speed | Run 模式的实时倍速（逻辑仍精确按 tick 推进） |
| ESDF / truth paths / local obs / rejected candidates | 显示开关 |
| 点击画布自由空间 | 更新最终导航目标（不重置无人机动力学；5Hz 层重新评估，30Hz 层只看到更新后的有效目标） |

**权威暂停状态**：`DebugSnapshot.paused` 由仿真节点发布真实状态；GUI 标签以 snapshot.paused 为准。

## 15. 使用方式

```bash
# 唯一入口（一次性启动两个节点 + 可选 RViz）
roslaunch il_2d_multiscale_debug expert_debug.launch

# 常用参数
#  seed := 场景/任务随机种子（默认 42）
#  paused := 是否启动即暂停（默认 true）
```

## 16. 新增参数（v9）

| 参数 | 默认值 | 含义 |
|---|---|---|
| `target_encoding/direction_bin_count` | 11 | 普通 FOV 内方向 bin 数量（不含 TURN_LEFT/RIGHT；必须奇数 ≥3，保证含 0°） |
| `target_encoding/normal_distance_reserve_m` | 0.5 | 普通目标归一化距离保留量（m） |
| `target_encoding/turn_ray_margin_deg` | 10.0 | TURN 机体系固定射线相对 FOV 边缘的余量（deg），兼作普通 bin 覆盖边界 margin |
| `macro/correction_enter_stable_ticks` | 1 | 进入修正前需连续满足条件的真实 5Hz 周期数 |
| `macro/observable_frontier_min_distance_m` | 1.5 | 可观测性/观察 frontier 候选最小距离（m） |
| `macro/observable_frontier_min_progress_m` | 0.5 | 绕行出口/观察 frontier 相对 original goal 的最小正向进展（m） |
| `macro/observable_unknown_margin_cells` | 3 | 已认证 frontier 点之后仍需可见的 known-FREE 格数（防 FOV/UNKNOWN 截断误判） |

所有新参数均存在于 `Params2D` 默认值、`params_io.hpp` 读取与有限性/范围校验、`config/default.yaml` 与本 README。

## 17. 已知风险 / 需实际运行观察

- 本轮**未编译、未运行、未测试**；只做了静态一致性检查。
- 修正退出在“局部避障重新可观测 + 原始目标进入普通方向安全锥 + 实际角速度不超过 `turn_exit_max_yaw_rate`”首次同时成立时立即发生；若刚从TURN看到绕行出口且存在安全普通frontier，先执行至少一个5Hz周期的TURN→NORMAL交接。边界闪烁由 `macro_reentry_guard_ticks` 抑制。
- 正常绕行交接应优先经历 TURN→NORMAL_CORRECTION→PASS_THROUGH，避免从 FOV 外旋转射线直接跳回反侧原始目标造成周期性反转；若当前局部观测无法形成安全普通 frontier，TURN 会继续保持到可形成 frontier 为止。`mission_revision` 不变、`update_event` 不因每 tick 虚拟点运动变化。
- `observable_frontier_*` 阈值与 `lp_min_clearance` 膨胀半径共同决定“绕行出口可见”的严格程度，需实机标定。
