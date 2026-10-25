"""Run ONE leg through the original Python MDP planner (the reference) and print a JSON line.

Usage:  python py_leg.py <algo_dir> <sx> <sy> <sh> <tx> <ty> <th> <obstacles_json>

The leg is planned exactly as the planner's own benchmark does it: state-lattice search, then
command compression, via executable_route_optimizer.plan_executable_leg with the live
DEFAULT_PLANNER_CONFIG. Only the planning call is timed (imports are excluded).
"""
import json
import sys
import time


def main() -> None:
    algo_dir = sys.argv[1]
    sx, sy, sh, tx, ty, th = (float(v) for v in sys.argv[2:8])
    obstacles_json = json.loads(sys.argv[8])

    sys.path.insert(0, algo_dir)
    import executable_route_optimizer as ero
    from config import DEFAULT_PLANNER_CONFIG
    from models import Obstacle, Pose

    obstacles = tuple(Obstacle(o["id"], float(o["x"]), float(o["y"]), o["face"]) for o in obstacles_json)
    start, target = Pose(sx, sy, sh), Pose(tx, ty, th)

    t0 = time.perf_counter()
    plan = ero.plan_executable_leg(start, target, obstacles, DEFAULT_PLANNER_CONFIG)
    elapsed_us = round((time.perf_counter() - t0) * 1e6)

    final = plan.final_pose
    print(json.dumps({
        "valid": bool(plan.is_valid),
        "reason": plan.reason or "",
        "cost": plan.total_cost,
        "distance": plan.total_distance_cm,
        "expansions": plan.expansions,
        "final": [final.x, final.y, final.heading] if final is not None else [0.0, 0.0, 0.0],
        "time_us": elapsed_us,
        "cmds": [c.stm_command for c in plan.commands],
    }))


if __name__ == "__main__":
    main()
