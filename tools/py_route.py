"""Run the original Python planner's exhaustive B3 route search on one fixture; print a JSON line.

Usage:  python py_route.py <algo_dir> <fixture.json>

This is `optimise_executable_b3_route` with alternatives disabled: it enumerates every obstacle
order (n!), so only small layouts are practical. Only the search call is timed.
"""
import json
import math
import sys
import time


def main() -> None:
    algo_dir, fixture_path = sys.argv[1], sys.argv[2]
    sys.path.insert(0, algo_dir)
    import executable_route_optimizer as ero
    from config import DEFAULT_PLANNER_CONFIG
    from models import Obstacle, Pose
    from motion_primitives import normalize_heading

    fx = json.load(open(fixture_path, encoding="utf-8"))
    start = Pose(float(fx["start"]["x"]), float(fx["start"]["y"]),
                 normalize_heading(math.radians(float(fx["start"]["heading_deg"]))))
    obstacles = tuple(Obstacle(o["id"], float(o["x"]), float(o["y"]), o["face"]) for o in fx["obstacles"])

    t0 = time.perf_counter()
    result = ero.optimise_executable_b3_route(start, obstacles, DEFAULT_PLANNER_CONFIG,
                                              allow_alternative_viewing=False)
    elapsed_us = round((time.perf_counter() - t0) * 1e6)

    cmds = [c.stm_command for leg in result.legs for c in leg.command_plan.commands]
    print(json.dumps({
        "valid": bool(result.is_valid), "complete": bool(result.is_complete),
        "order": list(result.obstacle_order), "cost": result.total_cost,
        "time_us": elapsed_us, "cmds": cmds,
    }))


if __name__ == "__main__":
    main()
