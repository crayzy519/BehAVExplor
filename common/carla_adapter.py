"""
Feed objects.json into CARLA, read traces.json back, and convert to LGSVL-shaped state/bbox objects frame.py already knows how to consume.
    Scenario -> lgsvl_input -> objects.json   (common/objects_writer.py)
                                    |
                             common/mock_bridge.py   <- stand-in for
                                    |                    real CARLA + carla_bridge
                                trace.json
                                    |
                          common/trace_adapter.py -> LGSVL-shaped state/bbox
                                    |
                             common/frame.py (unchanged fuzzing core)

Swapping in the real simulator later means replacing mock_bridge.run() with "send objects.json to CARLA/carla_bridge, wait, read the trace.json it
writes" - objects_writer.py and trace_adapter.py (the two real seams) do not change.
"""
import json
import os
import shutil

from loguru import logger

from common.frame import CaseRecorder, FrameElement
from common import objects_writer, mock_bridge, trace_adapter


def _lines_to_xz(lines):
    """[[{x,y}, ...]] -> [[(x, z), ...]]; z = world y (see trace_adapter docstring)."""
    return [[(p["x"], p["y"]) for p in line] for line in lines]


def detect_violation(frame, case_dir):
    """Per-frame violation check fed into CaseRecorder.add_frame(collision=...).

    Mock has no physics/Apollo stack, so there is nothing to detect: always
    False. Swap this out for the real Apollo/CARLA violation-detection logic
    when adapting the real bridge - it only needs to keep returning a plain
    bool per frame. frame.py's termination condition and is_fail()'s
    CaseFaultType.COLLISION check already key off that single boolean, so
    nothing downstream needs to change.
    """
    return False


class CarlaSimulatorAdapter(object):

    def __init__(self, max_sim_time, lgsvl_map=None, apollo_map=None, sim_mode="mock", dt=0.1):
        self.max_sim_time = max_sim_time
        self.dt = dt
        self.settings_yaml = None
        logger.info("[CarlaSimulatorAdapter] init: objects.json/trace.json mock loop. map=%s mode=%s"
                    % (lgsvl_map, sim_mode))

    def set_settings_yaml(self, settings_yaml_path):
        """Real bridge's scenario_executor.py reads settings.yaml + objects.json from
        the same --scene_dir, so we copy the scenario's settings.yaml as-is into every
        case_dir alongside the objects.json we generate."""
        self.settings_yaml = settings_yaml_path

    def run(self, scenario_obj, scenario_id, record_apollo_path):
        lgsvl_input = scenario_obj.get_lgsvl_input()
        objects_json = objects_writer.build_objects_json(lgsvl_input)

        case_dir = os.path.join(record_apollo_path, scenario_id)
        os.makedirs(case_dir, exist_ok=True)
        with open(os.path.join(case_dir, "objects.json"), "w") as f:
            json.dump(objects_json, f, indent=2)
        if self.settings_yaml:
            shutil.copy(self.settings_yaml, os.path.join(case_dir, "settings.yaml"))

        # 此处改为真实bridge
        mock_bridge.run(objects_json, self.max_sim_time, case_dir, self.dt)
        with open(os.path.join(case_dir, "trace.json")) as f:
            trace = json.load(f)

        ego_cfg = next(o for o in objects_json["ego"] if "dest_point" in o)
        destination = trace_adapter.world_point_to_destination(
            ego_cfg["dest_point"]["x"], ego_cfg["dest_point"]["y"])

        case_recorder = CaseRecorder(scenario_id)
        case_recorder.set_destination(destination)

        yellow = _lines_to_xz(lgsvl_input["yellow_lines"])
        edge = _lines_to_xz(lgsvl_input["edge_lines"])
        cross = _lines_to_xz(lgsvl_input["cross_lines"])

        for frame in trace:
            ego_state = trace_adapter.state_from_trace(frame["EGO"])
            ego_bbox = trace_adapter.bbox_from_trace(frame["EGO"])
            frame_npc_info = [
                {"npc_id": i, "npc_bbox": trace_adapter.bbox_from_trace(npc),
                 "npc_state": trace_adapter.state_from_trace(npc)}
                for i, npc in enumerate(frame["NPCs"])
            ]

            frame_element = FrameElement(
                frame["Sequence"], frame["TimeStamp"], destination, ego_bbox, ego_state,
                frame_npc_info, yellow, edge, cross,
            )
            violation = detect_violation(frame, case_dir)
            _, end = case_recorder.add_frame(frame_element, collision=violation)
            if end:
                break

        case_recorder.offline_analyze()
        logger.info("[CarlaSimulatorAdapter] %s: %d frames, events=[%s]"
                    % (scenario_id, len(case_recorder.frames),
                       case_recorder.obtain_case_event_str()))
        return case_recorder

    def close(self):
        logger.info("[CarlaSimulatorAdapter] closed.")
