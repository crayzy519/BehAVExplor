# BehAVExplor · CARLA 迁移说明（中文）

将BehAVExplor迁移到CARLA，同时完整保留原生变异算法。场景变异生成objects.json文件传给仿真器，读取仿真器输出的traces.json文件，因此算法侧和仿真器彻底解耦。
适配CARLA时，只需将carla_adapter.py:50换成真实CARLA Bridge即可。

## 一、测试方法

```bash
python3 -m venv .venv
.venv/bin/pip install numpy scipy shapely scikit-learn loguru pyyaml pandas tsfresh

# 在town01地图跑冒烟测试（限制60秒）
.venv/bin/python main.py --config configs/mock_town01.yaml
```

- 输出在 `outputs/mock_town01-at-<时间戳>/simulation/records_apollo/scenario_<id>/`，每个场景一份 `objects.json`，从同位置读取`trace.json`；日志每场景打印帧数和事件。

## 二、架构

```
Scenario (变异算法, common/scenario.py)         ← 零改动
        │  get_lgsvl_input()
        ▼
common/objects_writer.py  →  objects.json       ← 世界坐标系，schema 对齐 carla_bridge/config/follow/objects.json
        │
        ▼
common/mock_bridge.py     →  trace.json         ← 这一步将来换成真 CARLA + carla_bridge
        │
        ▼
common/trace_adapter.py   →  LGSVL 形状 state/bbox
        │
        ▼
common/frame.py (行为 oracle)
```

新增文件：

- `common/objects_writer.py`：把 `Scenario.get_lgsvl_input()` 转成 `objects.json`
- `common/mock_bridge.py`：读 `objects.json`，无物理引擎、按段等速插值出 `trace.json`（适配真实CARLA时删除）
- `common/trace_adapter.py`：把 `trace.json` 的一帧 actor dict 转成 `frame.py` 认识的 LGSVL 形状 `state`/`bbox`（注意：这里没有碰撞信息）
- `common/carla_adapter.py`：`CarlaSimulatorAdapter` - 每次run生成`objects.json`，读取`trace.json`并转换
- `tools/gen_town01_routes.py`：从 `carla_bridge/map.json`（Town01 Apollo HD 地图）生成 `data/town01/` 路网，保持原 `lane_details/route_details/routes` 格式。
- `configs/mock_town01.yaml`：mock 测试配置。

## 三、对接真实模拟器时的待办

- **替换 `mock_bridge.run()`**
- **替换 `detect_violation()`**：换成真实的违规检测逻辑

不用动：`behavexplor/`（corpus + BehaviorMiner）、`scenario.py` 变异算法、`carla_bridge/`（仅供参考的真实 bridge）。`frame.py` 本次做了一处改动：`compute_offline_event()` 现在无论是否命中违规都会计算真实的 NPC 最小距离（不再在违规时把 `min_distance2NPCs` 直接置0），保证 fitness 计算始终使用真实距离。接真 CARLA 时 `main.py` 只需把 `sim_mode` 从 `mock` 改成真实模式即可自动切换。
