# BehAVExplor · CARLA 迁移说明（中文）

将BehAVExplor迁移到CARLA，同时完整保留原生变异算法。场景变异生成objects.json文件传给仿真器，读取仿真器输出的traces.json文件，因此算法侧和仿真器彻底解耦。
适配CARLA时，只需将carla_adapter.py:70换成真实CARLA Bridge即可。

## 8.24更新：真实交规场景转换

`traffic_rule_BehAVExplor/`下是47个人工设计、针对具体交规的真实场景，每个场景固定了ego和NPC的真实世界坐标起止点（`objects.json`）以及CARLA连接配置（`settings.yaml`）。`tools/convert_traffic_rule_scenario.py`把这些世界坐标转换成BehAVExplor原生认识的`lane_id`+`offset`格式，尽量还原真实场景，作为fuzzing的初始种子。

转换后的场景文件写在 `data/<town>/LawXX_basic_info.json`，配置文件在`configs/mock_<town>_<lawXX>.yaml`（比如`configs/mock_town05_law44.yaml`）。

运行：

```bash
# 转换单个/多个场景
.venv/bin/python tools/convert_traffic_rule_scenario.py Law1 Law2 Law27
# 转换全部（自动跳过无法转换的场景并报告原因）
.venv/bin/python tools/convert_traffic_rule_scenario.py --all
# 校验转换出的坐标误差
.venv/bin/python tools/verify_traffic_rule_conversion.py --all
```

**当前结果：47个场景，42个转换成功（Town01 3个 / Town04 2个 / Town05 37个），5个跳过。**

**转换精度**：`tools/verify_traffic_rule_conversion.py`把转换后重新生成的ego/NPC坐标和真实数据逐点比对（42个场景共148个点位：每个场景的ego起点+终点，加上每个NPC的起点+终点）。误差中位数0.23米、平均0.32米，96%的点位（142/148）误差在1米以内，都是"最近车道点匹配"本身带来的贴合误差，不是转换逻辑出错。个别偏大的点位（最多到Law43的3.95米）是原始数据点本身离车道中心线较远（可能靠近路肩）。

**NPC变道（Law4 / Law28 / Law44）**：这3个场景的NPC起止点分别落在同一条路的不同车道上，需要变道才能连通。`scenario.py`原生的`_uniform_mutation`/`_gauss_mutation`只认"successor"（车道首尾相连）关系，每次换车道都会把上一条车道的完整长度加进偏移量——这个假设对变道（横向挪几米）不成立，如果直接把多条车道拼成一条链塞给NPC，会导致后续每次变异都把NPC位置算错。

workaround：把"车道A走一段 + 横移变道 + 车道B走一段"这整条真实轨迹，改写成**一条独立的合成车道**（写进`basic_info.json`的`synthetic_lanes`字段，运行时由`Scenario.add_synthetic_lanes()`合并进`self.lanes`，只作用于存在变道的场景）。这样NPC路线在`scenario.py`看来就是普通单车道路线。代价是**变道发生的具体位置被固定死**——后续变异只能调整NPC沿这条路径的路点，无法把"变道位置"纳入探索空间。

跳过的场景及原因：

| 场景 | 原因 |
|---|---|
| Law13 / Law14 / Law15 / Law23 | NPC是行人（`walker.pedestrian.*`）。BehAVExplor的变异算法从设计上就只有车辆NPC模型（`common/scenario.py`里`NPC_AGENT_TYPES`只有车辆类型，路线也是沿车道lane_id走的），没有行人的路径/尺寸表示。 |
| Law39 | 源数据`objects.json`是空文件 |

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
- `common/carla_adapter.py`：`CarlaSimulatorAdapter` - 每次run生成`objects.json`（连带把场景自带的`settings.yaml`原样复制到同目录），读取`trace.json`并转换
- `tools/gen_town01_routes.py`：从Apollo HD地图JSON生成`data/<town>/`路网（`lane_details`/`route_details`/`routes`），支持`--map-json`/`--town`/`--out-dir`参数，可以对任意城镇重复使用（目前已生成Town01/Town04/Town05）。
- `tools/convert_traffic_rule_scenario.py`：把`traffic_rule_BehAVExplor/`里人工设计的真实场景（世界坐标ego/NPC起止点）转换成`data/<town>/LawXX_basic_info.json`种子，NPC需要变道时会额外生成一个`synthetic_lanes`合成车道，详见上面"8.24更新"一节。
- `tools/verify_traffic_rule_conversion.py`：对比转换出的坐标和真实数据的坐标误差
- `configs/mock_town01.yaml`：mock 测试配置；`configs/mock_<town>_<lawXX>.yaml`：42个转换出的场景各自对应的配置。

## 三、对接真实模拟器时的待办

- **替换 `mock_bridge.run()`**
- **替换 `detect_violation()`**：换成真实的违规检测逻辑

不用动：`behavexplor/`（corpus + BehaviorMiner）、`scenario.py` 变异算法、`carla_bridge/`（仅供参考的真实 bridge）。`frame.py` 本次做了一处改动：`compute_offline_event()` 现在无论是否命中违规都会计算真实的 NPC 最小距离（不再在违规时把 `min_distance2NPCs` 直接置0），保证 fitness 计算始终使用真实距离。接真 CARLA 时 `main.py` 只需把 `sim_mode` 从 `mock` 改成真实模式即可自动切换。
