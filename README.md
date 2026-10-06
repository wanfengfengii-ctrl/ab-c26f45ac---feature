# Robot Trajectory Audit Service

机器人标定平台的轨迹审计服务：在关节程序下发前，对**分段三次 Bézier 轨迹**做
**整段连续曲线**（非抽样）的行程、速度、加速度合规裁决；并提供**整周期重定时**
接口，为合法轨迹选取最小整数统一倍率，使每段时长都是控制周期的整数倍且速度、
加速度不超限。所有计算使用精确有理数算术，十进制等值写法（`0.5` / `0.50` /
`5E-1`）不会改变峰值、倍率选择、越限排序或放行结论。

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

机器人控制器只接受**整周期**段时长。本接口在不改变路径形状与段间连续性
（C0/C1）的前提下，为结构与连续性合法的轨迹选择**最小正整数统一倍率**
`k ≤ max_scale`，使每段新时长 `k * duration` 都是控制周期的整数倍，且整
条连续曲线的速度、加速度峰值均不超限（等于上限视为合格）。统一放慢
`k` 倍后，速度峰值变为 `1/k`、加速度峰值变为 `1/k²`，行程与连续性不变。

请求体在 audit 字段（`joints` / `segments`）之外增加：

```json
{
  "joints": [ ... ],
  "segments": [ ... ],
  "cycle_duration": "0.02",
  "max_scale": 1000
}
```

- `cycle_duration`：正十进制控制周期（写法等值结果相同）。
- `max_scale`：允许的**最大统一倍率**，整数 `1 .. 1000000`。

#### 200 响应

```json
{
  "scale": 2,
  "cycle_duration": "0.2",
  "segments": [
    {"segment_index": 0, "duration": "1", "cycles": 5},
    {"segment_index": 1, "duration": "1", "cycles": 5}
  ],
  "total_duration": "2",
  "total_cycles": 10,
  "joints": [
    {"joint_index": 0, "joint": "j1", "peak_velocity": "1.2", "peak_acceleration": "1.2"}
  ],
  "proof": {"cycle_multiple": 2, "limit_min_scale": 1, "minimal": true}
}
```

- `segments[].cycles` / `total_cycles`：精确的整数周期数，时长计划可直接
  下发整周期控制器。
- `joints[]`：重定时后整条连续曲线的精确峰值（与 audit 同样的精确
  字符串格式）。
- `proof`：最小性证明。合格倍率恰好是 `cycle_multiple` 的整数倍且
  `>= limit_min_scale`；返回的 `scale` 是其中最小者，因此不存在更小的
  合格统一倍率。全部判定使用精确有理数算术，不依赖浮点容差或采样。

#### 409 响应

轨迹结构/连续性合法但无可行倍率时返回 409，`detail.reason` 为稳定原因码
（按此顺序判定）：

| `reason` | 含义 |
| --- | --- |
| `travel_out_of_bounds` | 行程越界；放慢无法改变路径（`detail.violations` 列出越界点） |
| `zero_velocity_limit` | 速度上限为 0 但峰值非零（`detail.joints` 列出关节与峰值） |
| `zero_acceleration_limit` | 加速度上限为 0 但峰值非零（同上） |
| `scale_exceeds_max` | 最小合格倍率超过 `max_scale`（`detail.required_scale` 为精确所需倍率） |

#### 422 响应

格式错误（非法十进制、`max_scale` 越界或非整数）、非正 `cycle_duration`、
非正段时长、结构错误与连续性错误与 audit 一致，均返回 422。

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
（字节码编译、应用导入、OpenAPI 生成）、以及对活服务的新旧接口冒烟
（audit 的可放行轨迹、越限轨迹、十进制等值不变性、422 行为；retime 的
最小倍率与整周期计划、409 原因码、422 行为），并以退出码报告结果。

## 本地测试

```bash
pytest -q
```
