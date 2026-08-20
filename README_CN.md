# BehAVExplor · CARLA 迁移说明（中文）

BehAVExplor（ISSTA '23）原本跑在 LGSVL 上，这个分支把它迁到 CARLA，同时完整保留原生变异算法。真 CARLA 还没启动，所以先用 mock 顶替仿真器；mock 模拟的是 CARLA 对象层的真实接口，因此现在写的适配代码将来能直接对接真 CARLA。

## 一、怎么测试

```bash
# 建虚拟环境 + 装依赖（只需一次）
python3 -m venv .venv
.venv/bin/pip install numpy scipy shapely scikit-learn loguru pyyaml pandas tsfresh

# 跑 mock 冒烟测试（自动跑满 60 秒后退出）
.venv/bin/python main.py --config configs/mock_town01.yaml
```

- 一次运行是一整轮 fuzzing 闭环（采样种子 → 变异 → 仿真 → 行为反馈），不是单场景。
- 60 秒里约执行一百多次场景（4 个初始随机种子 + 一百来轮变异）。
- 输出在 `outputs/mock_town01-at-<时间戳>/`，日志每场景打印帧数和事件。

## 二、改动

三层结构，mock 只顶替最底层的数据生产者，中间适配层和上层核心接真 CARLA 时不动：

```
Fuzzing 核心 (frame.py / corpus / 变异算法)   ← 零改动
        ▲  读 LGSVL 形状的 state
CarlaStateAdapter (carla_adapter.py)          ← 真实迁移代码，CARLA→LGSVL 坐标换算
        ▲  调 carla.Actor / carla.World 接口
CARLA 对象层                                   ← 真 CARLA 或 MockCarlaWorld（现在）
```

新增文件：

- `common/carla_adapter.py`：真实迁移代码，把 CARLA 的 Transform/Velocity/BoundingBox 换算成 LGSVL 形状（Y 轴翻转遵循 `carla_bridge/actor/traffic_participant.py:78`）。
- `common/mock_carla_world.py`：模拟 `carla.World`/`carla.Actor`/碰撞传感器接口，用脚本轨迹回放代替物理引擎。
- `common/mock_carla.py`：`MockSimulator`，`Simulator` 的 drop-in 替身，建世界 → tick → 经 adapter 读状态 → 喂 `frame.py`。
- `tools/gen_town01_routes.py`：从 `carla_bridge/map.json`（Town01 Apollo HD 地图）生成 `data/town01/` 路网，保持原 `lane_details/route_details/routes` 格式。
- `configs/mock_town01.yaml`：mock 测试配置。

改动的既有文件：

- `main.py`：按 `sim_mode` 分流（`mock` → `MockSimulator`，其它 → 真 `Simulator`），并删掉顶层 lgsvl 导入。

mock 行为：

- ego 沿起始车道中心线从起点匀速开到终点。
- NPC 跟随变异产出的路点（每点 `[offset, speed]`，offset 是沿路线累积弧长）逐帧插值运动。
- 碰撞走一个与真 CARLA 同形状的传感器接缝，但 mock 没物理引擎，传感器永不触发，碰撞标志恒 False。

已知限制（非 bug，等真 CARLA 才消除）：

- ego 到终点速度一步归零，必然触发一次 `hard_brake`（脚本假象，与 NPC 无关）。
- 无物理引擎，NPC 轨迹是脚本插值。
- 碰撞恒 False，下游逻辑接好了但未被真实触发验证。

## 三、对接真实模拟器时的待办

- **替换对象层**：`carla.Client` 连接 + `load_world('Town01')` + 同步模式；`world.spawn_actor` 生成真车；`spawn_actor(find('sensor.other.collision'), attach_to=ego)` 挂真碰撞传感器。tick、时间戳、`carla_adapter` 都不动。
- **补齐 CARLA 环节**：连 Apollo bridge 让 ego 由 Apollo 驱动；routes.json 的 `lane_id+offset` 转世界坐标；读 `environment` 设天气/信号灯；场景结束销毁 actor/sensor。
- 🔴 **决策点：NPC 怎么开**：CARLA 无 `follow_waypoints` 等价 API，需自己实现。推荐逐帧摆位置 + 改 bridge 用相邻帧位置差算速度，否则 Apollo 读到的 NPC 速度恒为 0。需先确认能否改 bridge 源码。
- **核对事件语义**：确认 CARLA/Apollo 的碰撞/压线/超速与 `frame.py` 事件码一致。压线、超速本就是 `frame.py` 用 ego 状态 + 地图几何算的，喂对数据即可，无需改；真正需要外部信号的只有碰撞。

不用动：`carla_adapter.py`、`frame.py`、`behavexplor/`（corpus + BehaviorMiner）、`scenario.py` 变异算法。接真 CARLA 时 `main.py` 只需把 `sim_mode` 从 `mock` 改成真实模式即可自动切换。
