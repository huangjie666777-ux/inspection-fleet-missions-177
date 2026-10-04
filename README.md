# inspection-fleet-routing

厂内多巡检车无碰撞路径规划后端。基于 CBS（Conflict-Based Search）在矩形网格上做多车联合最优协调，纯 HTTP API，无前端。

## 构建与启动

```bash
go build -o bin/server .
PORT=8080 ./bin/server   # PORT 环境变量配置本机端口，默认 8080
```

## 模型假设（点车辆近似）

- 车辆抽象为占据单个格子的点，不考虑车体尺寸、转向半径与速度连续变化。
- 时间离散为步。所有车在时刻 0 同时出发；每步可向上/下/左/右移动一格或原地等待，不得进入障碍格或越界。
- 同一时刻两车不得同格；连续两时刻不得迎面交换位置（边冲突）；进入他车刚离开的格子允许。
- 车辆首次到达终点后永久停留；起点即终点的车从时刻 0 起停留。到达后的停留占格同样参与冲突检查。
- 代价 = 各车到达时刻之和（等待计入到达时刻，到达后停留不计）。CBS 按总代价最优优先展开，求步数上限内的最小总代价联合解，非固定车序贪心。

## API

### POST /plan（旧接口，保持兼容）

请求体：

```json
{
  "width": 3,                // 网格宽 1..20
  "height": 2,               // 网格高 1..20
  "obstacles": [],
  "max_steps": 16,           // 步数上限 1..64
  "node_budget": 20000,      // CBS 高层节点展开预算 1..20000
  "vehicles": [              // 1..5 辆，id 唯一，起终点互不相同
    {"id": "a", "start": {"x": 0, "y": 0}, "goal": {"x": 2, "y": 0}},
    {"id": "b", "start": {"x": 2, "y": 0}, "goal": {"x": 0, "y": 0}}
  ]
}
```

旧接口行为保持不变；修复：起点即终点的车从时刻 0 直接永久停留，不再驶离再返回。

坐标为 0 起始整数，`x` 为列、`y` 为行。校验拒绝：越界或落在障碍上的起终点/障碍、重复车辆 ID、重复起点或终点、以及超出范围的数值（HTTP 400 + `{"error": ...}`）。

响应（HTTP 200）：

```json
{
  "status": "success",
  "total_cost": 6,
  "vehicles": [
    {"id": "a", "arrival_time": 4,
     "path": [{"x":0,"y":0},{"x":0,"y":1},{"x":1,"y":1},{"x":2,"y":1},{"x":2,"y":0}]},
    {"id": "b", "arrival_time": 2,
     "path": [{"x":2,"y":0},{"x":1,"y":0},{"x":0,"y":0}]}
  ]
}
```

（旧版文档曾给出一个障碍在 `(1,1)` 的路径示例，该路径穿过障碍格本身，属错误示例，已删除；带障碍的单车绕障请参考测试 `TestDetourAroundObstacle`。）

`path` 为时刻 0 到到达时刻的逐步坐标（长度 = `arrival_time + 1`）。

### POST /inspect（巡检任务分配与驻留作业）

在与 `/plan` 相同的网格/步数/预算校验之上，要求车辆数 1..3，且任务数等于车辆数，求解“一车一任务”的合法一一分配（bijection）。请求体：

```json
{
  "width": 5, "height": 3,
  "obstacles": [{"x": 2, "y": 1}],
  "max_steps": 40,            // 1..64，与 /plan 相同
  "node_budget": 20000,        // 1..20000，所有分配共享同一预算
  "vehicles": [                // 1..3 辆，校验同 /plan
    {"id": "a", "start": {"x": 0, "y": 0}, "goal": {"x": 4, "y": 0}},
    {"id": "b", "start": {"x": 0, "y": 2}, "goal": {"x": 4, "y": 2}}
  ],
  "tasks": [
    {"id": "t1", "work": {"x": 2, "y": 0},
     "allowed_vehicles": ["a", "b"],
     "earliest_start": 2, "latest_finish": 12, "dwell_steps": 2},
    {"id": "t2", "work": {"x": 2, "y": 2},
     "allowed_vehicles": ["a", "b"],
     "earliest_start": 0, "latest_finish": 20, "dwell_steps": 1}
  ]
}
```

任务字段语义：

- `id`：非空且任务内唯一；`work`：作业坐标，必须在界内且非障碍。
- `allowed_vehicles`：非空可执行车 ID 列表，ID 必须存在且不重复；分配必须是任务与车的一一对应。
- `earliest_start`、`latest_finish`：0..64 整数且不倒置；驻留开始不早于下界、完成不晚于上界。
- `dwell_steps`：1..8，到达作业点后可等待，但驻留必须在该格连续原地完成、不可中断。

时间线：车从时刻 0 出发 → 可在作业点等待到窗口下界 → 在 `work` 连续驻留 `dwell_steps` 步（驻留占用时刻 `service_start..service_finish`）→ 之后前往本车终点；作业前经过终点不算停车，作业后首次到达终点即永久停留。车与作业点允许重合（含他车的作业点），冲突只按两车同格/迎面交换判定。若窗口本身容不下驻留（或步数上限内无法按时到达），对应分配不可行；所有合法分配都穷尽才报 `no_solution`。

响应（HTTP 200）：

```json
{
  "status": "success",
  "total_cost": 10,
  "assignments": [
    {"vehicle_id": "a", "task_id": "t1"},
    {"vehicle_id": "b", "task_id": "t2"}
  ],
  "vehicles": [
    {"id": "a", "task_id": "t1",
     "service_start": 2, "service_finish": 3, "arrival_time": 5,
     "path": [{"x":0,"y":0},{"x":1,"y":0},{"x":2,"y":0},{"x":2,"y":0},{"x":3,"y":0},{"x":4,"y":0}]},
    {"id": "b", "task_id": "t2",
     "service_start": 2, "service_finish": 2, "arrival_time": 5,
     "path": [{"x":0,"y":2},{"x":1,"y":2},{"x":2,"y":2},{"x":2,"y":2},{"x":3,"y":2},{"x":4,"y":2}]}
  ]
}
```

`total_cost` 为各车停车时刻之和。`no_solution` 与 `undecided` 不输出 `assignments`/`vehicles` 等任何部分方案。

### 状态语义

- `success`：找到并证明总代价最优的联合解。`/inspect` 会枚举全部合法一一分配（不做距离贪心预选），所有分配的 CBS 节点在同一优先队列按总代价展开、共享同一预算，首个无冲突节点即跨分配全局最优。
- `no_solution`：搜索空间已穷尽（`/inspect` 包含所有合法分配），证明该步数上限内无解。不输出任何路线或分配。
- `undecided`：CBS 展开预算耗尽（或请求被取消），既未证明最优也未证明无解。不输出部分或非最优方案。

### 取消与并发

客户端断开（HTTP 请求取消）会中止该次 CBS 计算（状态为 `undecided`）。没有“停止搜索”接口；每个请求的约束集与节点预算完全独立，并发请求互不影响。

## 示例

```bash
# 协调路径：两车对向换边，一车绕行，总代价 6
curl -s localhost:8080/plan -d @examples/coordinated.json

# 无解：1 宽走廊内互换，搜索穷尽
curl -s localhost:8080/plan -d @examples/no_solution.json

# 巡检：枚举任务-车辆一一分配，含等待窗口与连续驻留
curl -s localhost:8080/inspect -d @examples/inspect.json
```

## 代码结构

- `model.go`：请求/响应模型、网格与常量限制
- `validate.go`：共享地图/步数/预算校验、车辆校验与巡检任务校验
- `lowlevel.go`：带顶点/边约束的单车 (位置, 时间) A* 寻路（含起点即终点直接停留）
- `inspect_lowlevel.go`：作业前等待 → 连续驻留 → 终点永久停留的分阶段单车寻路
- `cbs.go`：`/plan` 的 CBS 高层冲突检测、分支约束与按总代价的联合搜索
- `inspect_cbs.go`：`/inspect` 的合法一一分配枚举与跨分配共享预算的 CBS
- `server.go` / `main.go`：Chi 路由、HTTP 处理与可配置端口启动

## 测试

```bash
go test ./...
```

覆盖参数校验、单车寻路、边冲突禁止、到达后占格、起终点相同不再驶离、协调换边、无解穷尽、预算未决与请求取消；巡检侧覆盖任务校验、窗口与驻留不可中断、强制/最优一一分配、作业点可重合、跨分配无解穷尽与预算未决、HTTP 200/400 响应。
