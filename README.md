# Robot Trajectory Audit Service

机器人标定平台的轨迹审计服务：在关节程序下发前，对**分段三次 Bézier 轨迹**做
**整段连续曲线**（非抽样）的行程、速度、加速度合规裁决。所有计算使用精确
有理数算术，十进制等值写法（`0.5` / `0.50` / `5E-1`）不会改变峰值、越限
排序或放行结论。

## API

### `POST /api/trajectories/audit`

请求体：

```json
{
  "joints": [
    {
      "id": "j1",
      "travel": {"min": "-2", "max": "2"},
      "velocity_limit": "3",
      "acceleration_limit": "20"
    }
  ],
  "segments": [
    {"duration": "0.5", "control_positions": {"j1": ["0", "0.2", "0.6", "1.0"]}},
    {"duration": "0.5", "control_positions": {"j1": ["1.0", "1.4", "1.6", "1.6"]}}
  ]
}
```

约束：

- `joints`：1–8 个关节，`id` 唯一；`travel` 为闭合行程区间（`min <= max`）；
  `velocity_limit` / `acceleration_limit` 非负。
- `segments`：1–200 段；`duration` 为**正**十进制；`control_positions` 必须
  恰好覆盖每个关节，每关节**恰好 4 个**控制位置（三次 Bézier）。
- 数字可写为 JSON 字符串或 JSON 数值；字符串须为规范十进制（允许指数记法），
  拒绝 `NaN`/`Infinity`/分数/十六进制等。
- 相邻段必须**精确**连续：段 `k` 的末位置等于段 `k+1` 的首位置，且端点速度
  `3*(c3-c2)/T_k` 等于 `3*(c1'-c0')/T_{k+1}`（精确相等，非容差）。

### 200 响应

```json
{
  "approved": true,
  "joints": [
    {"joint_index": 0, "joint": "j1", "peak_velocity": "2.4", "peak_acceleration": "4.8"}
  ],
  "violations": []
}
```

- `approved`：无任何越限时为 `true`。
- `joints[].peak_velocity` / `peak_acceleration`：该关节在**整条连续曲线**上的
  精确峰值（速度是二次 Bézier，峰值在端点或加速度过零点；加速度是线性的，
  峰值必在端点）。值为精确字符串：有限小数直接输出（`"2.4"`），否则输出
  最简分数（`"1/6"`），保证不丢精度。
- `violations`：全部越限，按 **段号 → 关节号 → 约束类型**（`travel` →
  `velocity` → `acceleration`）稳定排序；段号/关节号为从 0 开始的索引。
  越限判定为严格大于上限——**等于上限视为合格**（行程为闭区间同理）。
  - `travel`：控制位置越出闭合行程；`bound` 为 `lower`/`upper`，
    `control_point_index` 指向最严重越界控制点，`value` 为其位置。
  - `velocity` / `acceleration`：`value` 为该段精确峰值，`limit` 为上限。

### 422 响应

格式错误、非正时长、结构错误（关节数/段数越界、控制点不是 4 个、关节缺失
或未知、id 重复等）以及连续性错误均返回 422，`detail` 中每项含可定位的
`loc` / `msg` / `type`，例如：

```json
{
  "detail": [
    {
      "loc": ["body", "segments", 1, "control_positions", "j1", 0],
      "msg": "position discontinuity between segments 0 and 1 for joint 'j1': ...",
      "type": "continuity.position"
    }
  ]
}
```

连续性错误的 `type` 为 `continuity.position` / `continuity.velocity`。

### `POST /api/trajectories/retime`

整周期重定时：**路径形状与段间 C0/C1 连续性完全不变**，把每段时长统一乘以
一个最小正整数倍率 `scale`（新时长 `d'_k = scale * d_k`），使

1. 每段新时长都是控制周期 `cycle_duration` 的**正整数倍**，可直接下发只接受
   整周期段时长的控制器；
2. 整条连续曲线的速度、加速度均不超限（速度随 `1/scale` 降低，加速度随
   `1/scale²` 降低）。

请求体在 /audit 的 `joints` / `segments` 契约之上新增两个字段：

| 字段 | 类型 | 约束 |
| --- | --- | --- |
| `cycle_duration` | 正十进制（字符串或 JSON 数值） | `> 0`，控制器周期 |
| `max_scale` | 整数 | `1 <= max_scale <= 1000000`，严格整数（拒绝 `1.5`、`"5"`、布尔） |

```json
{
  "joints": [
    {"id": "j1", "travel": {"min": "-2", "max": "2"},
     "velocity_limit": "3", "acceleration_limit": "20"}
  ],
  "segments": [
    {"duration": "0.5", "control_positions": {"j1": ["0", "0.2", "0.6", "1.0"]}},
    {"duration": "0.5", "control_positions": {"j1": ["1.0", "1.4", "1.6", "1.6"]}}
  ],
  "cycle_duration": "0.3",
  "max_scale": 100
}
```

### 200 响应

```json
{
  "scale": 3,
  "cycle_duration": "0.3",
  "segments": [
    {"segment_index": 0, "duration": "1.5", "cycles": 5},
    {"segment_index": 1, "duration": "1.5", "cycles": 5}
  ],
  "joints": [
    {"joint_index": 0, "joint": "j1",
     "peak_velocity": "0.8", "peak_acceleration": "8/15"}
  ]
}
```

- `scale`：满足全部条件的**最小正整数统一倍率**。
- `segments[].duration`：该段精确新时长；`cycles` 为其包含的整数控制周期数
  （`duration == cycles * cycle_duration`，恒为正整数）。
- `joints[]`：重定时后整条曲线的精确峰值（格式与 /audit 相同：有限小数或
  最简分数）。

#### 倍率如何精确求得（无浮点容差、无抽样）

设周期为 `C`，段 `s` 原时长 `T_s`、该段某关节原峰值速度 `V_s`、峰值加速度
`A_s`（均为 /audit 同一套精确解析结果：速度峰值在端点或加速度唯一过零点，
加速度峰值在端点）。统一倍率 `n` 可行当且仅当：

- 整周期：把 `T_s/C` 约为最简分数 `p_s/q_s`，因 `p_s` 与 `q_s` 互质，
  要求 `n` 是每个 `q_s` 的倍数，即 `n` 为 `L = lcm_s(q_s)` 的倍数；
- 速度：`n >= ⌈V_s / vlim⌉`（对每个正速度上限）；
- 加速度：`n² >= A_s / alim`，即
  `n >= ⌈√(A_s/alim)⌉`（整数平方根精确计算，对每个正加速度上限）。

令 `B` 为速度/加速度各整数下界中的最大值，则最小可行倍率是
`scale = L · ⌈B/L⌉`——不低于 `B` 的最小 `L` 倍数（`⌈·⌉` 全部用整数运算）。
**任何更小的正整数要么破坏整周期整除性，要么违反某条限幅**，因此不存在更小
的合格统一倍率；边界恰好等于限制视为合格。十进制等值写法
（`0.3` / `0.30` / `3E-1`）经过完全相同的有理数运算，倍率、时长与峰值结果
逐字节一致。

### 409 响应（结构与连续性合法，但无法编排）

`{"reason": <稳定原因码>, "msg": <人类可读说明>}`，原因码：

| `reason` | 触发条件 |
| --- | --- |
| `travel_out_of_bounds` | 控制位置越出闭合行程（放慢不改变路径，无法修复） |
| `zero_limit_with_motion` | 速度/加速度上限为 0 而对应峰值非零（再慢也无法满足） |
| `scale_exceeds_max` | 满足整周期与全部限幅所需的最小统一倍率超过 `max_scale` |

### 422 响应（/retime）

与 /audit 一致：格式错误（非法十进制记法等）、非正 `cycle_duration` 或
非正段时长、`max_scale` 越界或非整数、结构错误、连续性错误均返回 422，
`detail` 中每项含 `loc` / `msg` / `type`。

### 健康检查

`GET ${API_HEALTH_PATH:-/api/health}` → `{"status": "ok"}`。

## 运行

```bash
# 本地
pip install -r requirements.txt
uvicorn app.main:app --port 8000

# Docker（宿主机端口可配置）
API_HOST_PORT=9090 docker compose up --build api
```

配置项（环境变量或 `.env`，见 `.env.example`）：

| 变量 | 默认 | 说明 |
| --- | --- | --- |
| `API_HOST_PORT` | `8080` | 宿主机发布端口 |
| `APP_PORT` | `8000` | 容器内监听端口 |
| `API_HEALTH_PATH` | `/api/health` | 健康检查路径（服务与探针共用） |

## 验证（一次性 verify 服务）

```bash
docker compose up --build --exit-code-from verify
echo $?   # 0 = 全部通过
```

`verify` 等待 `api` 健康后依次执行：代码测试（pytest）、构建检查
（字节码编译、应用导入、OpenAPI 生成）、以及对活服务的 API 冒烟
（/audit：可放行轨迹、越限轨迹、十进制等值不变性、422 行为；
/retime：整周期最小时长计划与在线最小性证明、409 稳定原因码、
十进制等值不变性、422 行为），并以退出码报告结果。

## 本地测试

```bash
pytest -q
```
