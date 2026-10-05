# inspection-fleet-routing

厂内多巡检车无碰撞路径规划后端。基于 CBS（Conflict-Based Search）在矩形网格上做多车联合最优协调，纯 HTTP API，无前端。提供两个接口：`/plan`（点到点联合寻路）与 `/inspect`（巡检任务分配 + 驻留作业 + 联合寻路）。

## 构建与启动

```bash
go build -o bin/server .
PORT=8080 ./bin/server   # PORT 环境变量配置本机端口，默认 8080
```

## 模型假设（点车辆近似）

- 车辆抽象为占据单个格子的点，不考虑车体尺寸、转向半径与速度连续变化。
- 时间离散为步。所有车在时刻 0 同时出发；每步可向上/下/左/右移动一格或原地等待，不得进入障碍格或越界。
- 同一时刻两车不得同格；连续两时刻不得迎面交换位置（边冲突）；进入他车刚离开的格子允许。
- 车辆首次到达终点后永久停留；起点即终点的车从时刻 0 起停留，不会驶离再返回。到达后的停留占格同样参与冲突检查。
- 代价 = 各车到达时刻之和（等待计入到达时刻，到达后停留不计）。CBS 按总代价最优优先展开，求步数上限内的最小总代价联合解，非固定车序贪心。

## API

### POST /plan

请求体：

```json
{
  "width": 3,                // 网格宽 1..20
  "height": 2,               // 网格高 1..20
  "obstacles": [],           // 障碍格列表，均需在界内
  "max_steps": 16,           // 步数上限 1..64
  "node_budget": 20000,      // CBS 高层节点展开预算 1..20000
  "vehicles": [              // 1..5 辆，id 唯一，起终点互不相同
    {"id": "a", "start": {"x": 0, "y": 0}, "goal": {"x": 2, "y": 0}},
    {"id": "b", "start": {"x": 2, "y": 0}, "goal": {"x": 0, "y": 0}}
  ]
}
```

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

`path` 为时刻 0 到到达时刻的逐步坐标（长度 = `arrival_time + 1`）。

### 状态语义

- `success`：找到并证明总代价最优的联合解（CBS 按总代价出队，首个无冲突节点即最优）。
- `no_solution`：搜索空间已穷尽，证明该步数上限内无解。不输出任何车辆路线。
- `undecided`：CBS 展开预算耗尽（或请求被取消），既未找到解也未证明无解。不输出部分路线。

### POST /inspect

巡检任务分配与驻留作业。在 `/plan` 的地图、车辆、步数与预算校验之上增加任务列表：限 1 至 3 车，任务数必须等于车数，一车一任务（一一分配）。

请求体（与 `/plan` 相同的字段不再注释）：

```json
{
  "width": 6, "height": 2, "max_steps": 16, "node_budget": 20000,
  "vehicles": [
    {"id": "a", "start": {"x": 0, "y": 0}, "goal": {"x": 5, "y": 0}},
    {"id": "b", "start": {"x": 1, "y": 1}, "goal": {"x": 0, "y": 1}}
  ],
  "tasks": [
    {"id": "t1",                      // 任务 ID，非空且唯一
     "cell": {"x": 1, "y": 0},        // 作业坐标，须界内且非障碍
     "vehicles": ["a", "b"],          // 可执行车 ID 集合，非空，均须已知
     "earliest_start": 0,             // 最早开始时刻 0..64
     "latest_finish": 12,             // 最晚完成时刻 0..64，不得早于 earliest_start
     "dwell": 1}                      // 驻留步数 1..8
    {"id": "t2", "cell": {"x": 4, "y": 0}, "vehicles": ["a", "b"],
     "earliest_start": 0, "latest_finish": 12, "dwell": 1}
  ]
}
```

校验拒绝（HTTP 400）：非法数值、未知车辆、越界或障碍上的作业点、空可执行集合、倒置窗口等。窗口长度容不下驻留步数不算非法请求，而是按无解处理（`no_solution`）。

执行语义：

- 每车从时刻 0 出发前往其任务作业点，可提前到达并原地等待；驻留在 `[earliest_start, latest_finish]` 窗口内开始，连续原地驻留 `dwell` 步，不可中断；驻留完成时刻 = 开始时刻 + dwell，不得晚于 `latest_finish`。
- 驻留完成后前往本车终点，首次到达即永久停车；作业前经过终点不算停车。车辆与作业点可重合，按上述阶段判定。
- 移动、等待与驻留同样遵守同格与迎面交换限制。
- 求解器枚举全部合法一一分配（不预先固定距离贪心结果），所有分配的 CBS 树共享一个按总代价排序的优先队列与同一个展开预算；弹出的首个无冲突节点即为跨分配的全局最优（最小化各车停车时刻之和）。仅证明最优后报 `success`，全部分配搜索穷尽才报 `no_solution`，预算耗尽未证明时报 `undecided`，均不输出部分或非最优方案。

响应（HTTP 200）：

```json
{
  "status": "success",
  "total_cost": 11,
  "assignments": [
    {"vehicle_id": "a", "task_id": "t2", "job_start": 4, "job_finish": 5,
     "parking_time": 6,
     "path": [{"x":0,"y":0},{"x":1,"y":0},{"x":2,"y":0},{"x":3,"y":0},{"x":4,"y":0},{"x":4,"y":0},{"x":5,"y":0}]},
    {"vehicle_id": "b", "task_id": "t1", "job_start": 2, "job_finish": 3,
     "parking_time": 5,
     "path": [{"x":1,"y":1},{"x":1,"y":1},{"x":1,"y":0},{"x":1,"y":0},{"x":0,"y":0},{"x":0,"y":1}]}
  ]
}
```

`path` 为时刻 0 到停车时刻的逐步坐标（长度 = `parking_time + 1`）；`job_start`/`job_finish` 为驻留起止时刻；`total_cost` 为各车停车时刻之和。

### 取消与并发

客户端断开（HTTP 请求取消）会中止该次 CBS 计算。每个请求的约束集与节点预算完全独立，并发请求互不影响。

## 实际执行核验（/executions）

在巡检规划之上提供执行核验：先由求解器生成方案并冻结，再分批上报实测位置，最后查询或封存核验报告。纯后端，无状态外依赖。

### POST /executions

请求体与 POST /inspect 完全相同。求解成功（`success`）才创建执行并返回 HTTP 201：

```json
{"status": "success", "execution_id": "exec-...", "plan": { ...同 /inspect 响应... }}
```

`no_solution`/`undecided` 时不创建执行、不返回 ID（HTTP 200 + `{"status": ...}`）。创建后地图、任务、分配与路线全部冻结，核验只接受该方案，不接受自造方案。核验时间范围为 0..max_steps；车辆计划停车后按终点位置延展到 max_steps。

### POST /executions/{id}/observations

分批上报实测位置，支持乱序与重复：

```json
{"observations": [
  {"vehicle_id": "a", "time": 2, "cell": {"x": 2, "y": 0}},
  {"vehicle_id": "a", "time": 0, "cell": {"x": 0, "y": 0}}
]}
```

整批校验，任一失败则整批拒绝且不落任何数据：

- 未知车辆、越界时刻（0..max_steps 之外）、越界坐标：HTTP 400。
- 同车同时刻异位置（批内或与已存记录冲突）：HTTP 409。
- 同车同时刻同位置为幂等重复，计入 `duplicates`。
- 落在障碍格或构成违规移动的观测**照常保存**为证据，由核验报告标记，不丢弃。

成功响应：`{"status": "accepted", "added": N, "duplicates": M}`。

### GET /executions/{id}

返回一致快照：冻结的请求与方案、全部已存观测、以及按当前观测即时计算的核验报告（已封存的执行返回封存时冻结的报告）。

### POST /executions/{id}/seal

封存执行：计算并冻结报告，随记录持久化。封存后上报观测返回 HTTP 409；重复封存返回原报告，不重新计算。查询与封存返回同一快照结构。

### 核验报告格式

```json
{
  "execution_id": "exec-...", "sealed": false,
  "status": "passed | failed | unknown",
  "deviations": [{"vehicle_id": "a", "time": 5, "planned": {"x":4,"y":0}, "observed": {"x":0,"y":1}}],
  "violations": [{"type": "illegal_move", "vehicles": ["a"], "time": 6, "cell": {"x":0,"y":1}, "detail": "..."}],
  "missing": [{"vehicle_id": "a", "from": 3, "to": 4}],
  "tasks": [{"task_id": "t2", "vehicle_id": "a", "status": "verified", "dwell_start": 4, "dwell_finish": 5}]
}
```

- `deviations`：实测与冻结方案不同的位置（仅报告，不直接判违规）。
- `violations`：违反运动/冲突规则，`type` 为 `start_mismatch`（时刻 0 不在起点）、`obstacle`（位于障碍格）、`illegal_move`（相邻完整观测间非四邻移动或等待）、`same_cell`（同刻同格）、`head_on_swap`（相邻时刻迎面交换）。均指出车辆、时刻与位置。
- `missing`：缺测区间（闭区间）。缺测**不插值、不照抄计划、不推断缺口内安全**；相邻观测校验只在两侧都有观测的相邻时刻之间进行。
- `tasks`：按冻结分配核对。窗口 `[earliest_start, latest_finish]` 内连续 `dwell+1` 个时刻都在作业格，才证明驻留 `dwell` 步，返回最早有效起止（`dwell_start`/`dwell_finish`）。该车观测完整仍不满足为 `incomplete`，否则为 `unknown`。
- 整体 `status`：存在已知违规或 `incomplete` 任务为 `failed`；无这些问题但有缺测为 `unknown`；全量观测且合规则 `passed`。

### 持久化、并发与限额

- 记录以 JSON 存于 `EXEC_DATA_DIR` 目录（每执行一个文件，临时文件 + fsync + rename 原子写入；写失败不留部分状态，内存状态也不变）。重启后自动加载，封存状态与报告保持有效。未设置 `EXEC_DATA_DIR` 时为纯内存模式。
- 创建、上报、封存由单锁串行提交，并发请求按到达顺序生效。
- 限额：执行数量 `EXEC_MAX_EXECUTIONS`（默认 128，超限 HTTP 429）、单批观测数 `EXEC_MAX_BATCH`（默认 2048）、请求体 1 MiB。

## 示例

```bash
# 协调路径：两车对向换边，一车绕行，总代价 6
curl -s localhost:8080/plan -d @examples/coordinated.json

# 无解：1 宽走廊内互换，搜索穷尽
curl -s localhost:8080/plan -d @examples/no_solution.json

# 巡检任务分配：两车两任务，距离贪心分配总代价 16，
# 联合最优交换任务后总代价 11
curl -s localhost:8080/inspect -d @examples/inspect.json

# 执行核验：创建执行 -> 上报观测 -> 查询 -> 封存
ID=$(curl -s -X POST localhost:8080/executions -d @examples/inspect.json | jq -r .execution_id)
curl -s -X POST localhost:8080/executions/$ID/observations -d @examples/observations.json
curl -s localhost:8080/executions/$ID            # 即时核验快照
curl -s -X POST localhost:8080/executions/$ID/seal   # 封存并冻结报告
```

## 代码结构

- `model.go`：请求/响应模型、网格与常量限制
- `validate.go`：地图、车辆与任务参数校验
- `lowlevel.go`：带顶点/边约束的单车 (位置, 时间) A* 寻路
- `cbs.go`：CBS 高层冲突检测、分支约束与按总代价的联合搜索
- `task_lowlevel.go`：巡检阶段化（赴作业点/驻留/赴终点）单车 A* 寻路
- `task_cbs.go`：任务分配枚举与共享预算的跨分配 CBS 联合优化
- `execution.go`：执行记录、观测存储与原子持久化
- `verify.go`：实测对方案的偏离、运动/冲突违规与任务驻留核验
- `execution_server.go`：执行创建、观测上报、查询与封存的 HTTP 处理
- `server.go` / `main.go`：Chi 路由、HTTP 处理与可配置端口启动

## 测试

```bash
go test ./...
```

覆盖参数校验、单车寻路、边冲突禁止、到达后占格、协调换边、无解穷尽、预算未决、请求取消、起点即终点停车、任务校验、驻留窗口、分配最优性与强制分配；执行核验覆盖创建门槛、整批拒绝、幂等与冲突、缺测区间、违规检测、驻留证明、封存幂等与重启持久化。
