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
（可放行轨迹、越限轨迹、十进制等值不变性、422 行为），并以退出码报告结果。

## 本地测试

```bash
pytest -q
```
