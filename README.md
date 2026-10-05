# inspection-fleet-routing

厂内多巡检车无碰撞路径规划后端。基于 CBS（Conflict-Based Search）在矩形网格上做多车联合最优协调，纯 HTTP API，无前端。提供两个接口：`/plan`（点到点联合寻路）与 `/inspect`（巡检任务分配 + 驻留作业 + 联合寻路）。

## 构建与启动

```bash
go build -o bin/server .
PORT=8080 DATA_DIR=./data ./bin/server
# PORT：监听端口，默认 8080
# DATA_DIR：执行记录目录；留空时仅保存在进程内存中
# MAX_EXECUTIONS：最多保留执行数，默认 1000
# MAX_INFLIGHT_PLANS：最多并发求解数，默认 8
# MAX_REQUEST_BODY_BYTES：请求体上限，默认 1048576
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
     "dwell": 1},                     // 驻留步数 1..8
    {"id": "t2",
     "cell": {"x": 4, "y": 0},
     "vehicles": ["a", "b"],
     "earliest_start": 0,
     "latest_finish": 12,
     "dwell": 1}
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

## 实际执行核验

`/inspect` 仍然只返回规划结果，不创建执行。执行接口接收相同的巡检请求体，并再次调用现有求解器；只有 HTTP 体内求解状态为 `success` 时才创建执行 ID。客户端不能提交自造的地图、任务、分配或路线。

核验时刻范围固定为 `0..max_steps`。方案中的路线只保存到停车时刻；停车后的计划位置按终点坐标延展，直到 `max_steps`。

### POST /inspect/executions

请求体与 `POST /inspect` 完全相同。成功时返回 201：

```json
{
  "execution_id": "exec_...",
  "status": "success",
  "plan": {"max_steps": 16, "vehicles": [
    {"vehicle_id": "a", "task_id": "t2", "job_start": 4, "job_finish": 5,
     "parking_time": 6, "path": []}
  ]}
}
```

`no_solution` 与 `undecided` 不创建执行 ID，返回 200 和对应状态；达到执行数量上限返回 429。

### POST /inspect/executions/{id}/observations

请求体：

```json
{"observations": [
  {"vehicle_id": "a", "time": 0, "cell": {"x": 0, "y": 0}},
  {"vehicle_id": "b", "time": 0, "cell": {"x": 1, "y": 1}}
]}
```

- 批次内及已保存数据均按 `(vehicle_id,time)` 去重；同位置重复为幂等，异位置冲突，冲突返回 409。
- 未知车辆、时刻不在 `0..max_steps`、坐标越界都会整批拒绝，不保存该批任何记录。
- 障碍位置是合法上报坐标，会被保存并生成 `obstacle` 证据；同格、迎面交换、非法移动、错误起点和偏离计划也保留为证据。
- 相邻完整观测仅允许曼哈顿距离为 0 或 1。中间有缺测时只列缺测区间，不做插值、不照抄计划、不断言缺口内安全。
- 观测可以乱序、分批提交；响应携带当前核验报告，但未封存前不冻结。

### GET /inspect/executions/{id}

返回一致的执行快照：固定计划、已按车辆和时刻排序的观测，以及当前报告。

### POST /inspect/executions/{id}/seal

封存时计算并持久化最终报告，之后上报返回 409。重复封存不重新计算，返回第一次封存的同一报告。上报、查询和封存通过同一存储互斥串行提交，避免封存与写入交错。

记录按执行 ID 保存为 `DATA_DIR/<execution_id>.json`：先写临时文件再原子改名。若持久化写失败，内存状态不变，也不会产生部分执行记录；配置 `DATA_DIR` 后重启可恢复观测、报告和封存状态。

### 报告语义

顶层 `status`：

- `passed`：`0..max_steps` 对每辆车均有完整观测，任务均证明完成，且没有任何证据。
- `failed`：存在已知违规证据，或某车辆有完整观测但仍未完成其固定分配任务。
- `unknown`：没有已知失败证据，但存在缺测区间；此时任务可能为 `unknown`，缺测不能推断为安全。

驻留证明要求在分配车辆的作业格上，于窗口内连续观测 `dwell+1` 个时刻（开始时刻到开始+dwell），表示实际驻留 `dwell` 步。任务结果返回找到的最早有效起止；完整观测但不存在有效区间为 `incomplete`，观测不全且未找到证据为 `unknown`。

证据会明确车辆或车辆对、时刻与位置，类型包括：`start_mismatch`、`plan_deviation`、`obstacle`、`movement_violation`、`vertex_conflict`、`edge_swap`。

### curl 演示

```bash
# 1. 由服务端求解并创建执行
ID=$(curl -s localhost:8080/inspect/executions \
  -H 'Content-Type: application/json' \
  -d @examples/inspect.json \
  | sed -n 's/.*"execution_id": *"\([^"]*\)".*/\1/p')

# 2. 乱序补报；障碍/违规坐标会作为证据保存，坐标越界才整批拒绝
curl -s localhost:8080/inspect/executions/$ID/observations \
  -H 'Content-Type: application/json' \
  -d '{"observations":[
    {"vehicle_id":"b","time":0,"cell":{"x":1,"y":1}},
    {"vehicle_id":"a","time":0,"cell":{"x":0,"y":0}}
  ]}'

# 3. 查询当前快照和核验报告（缺测时 status=unknown）
curl -s localhost:8080/inspect/executions/$ID

# 4. 封存最终报告；重复执行仍返回同一封存报告
curl -s -X POST localhost:8080/inspect/executions/$ID/seal
```

## 示例

```bash
# 协调路径：两车对向换边，一车绕行，总代价 6
curl -s localhost:8080/plan -d @examples/coordinated.json

# 无解：1 宽走廊内互换，搜索穷尽
curl -s localhost:8080/plan -d @examples/no_solution.json

# 巡检任务分配：两车两任务，距离贪心分配总代价 16，
# 联合最优交换任务后总代价 11
curl -s localhost:8080/inspect -d @examples/inspect.json
```

## 代码结构

- `model.go`：请求/响应模型、网格与常量限制
- `validate.go`：地图、车辆与任务参数校验
- `lowlevel.go`：带顶点/边约束的单车 (位置, 时间) A* 寻路
- `cbs.go`：CBS 高层冲突检测、分支约束与按总代价的联合搜索
- `task_lowlevel.go`：巡检阶段化（赴作业点/驻留/赴终点）单车 A* 寻路
- `task_cbs.go`：任务分配枚举与共享预算的跨分配 CBS 联合优化
- `execution.go` / `verifier.go`：执行快照、观测核验、缺测区间、任务证明与报告状态
- `store.go`：执行互斥、原子持久化、重启恢复与执行数量限制
- `server.go` / `main.go`：Chi 路由、HTTP 处理、容量限制与可配置端口/数据目录启动

## 测试

```bash
go test ./...
```

覆盖参数校验、单车寻路、边冲突禁止、到达后占格、协调换边、无解穷尽、预算未决、请求取消、起点即终点停车、任务校验、驻留窗口、分配最优性、强制分配、乱序观测、整批拒绝、幂等/冲突、证据保留、缺测未知、封存拒绝写入、重复封存与持久化恢复。
