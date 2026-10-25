"""Route-level comparison: C++ branch-and-bound + Held-Karp vs the Python exhaustive B3 search.

For each (small) fixture both planners search for the cheapest complete route over all obstacle
orders. The comparison checks that the C++ search finds a route of the same optimal cost, and
records how long each took.

    python tools/compare_routes.py --algo <SC2079_MDP_T7/algo> --cli <mdp_plan_cli> A_spread_01 ...
"""
import argparse
import json
import math
import subprocess
import sys
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--algo", required=True, type=Path)
    ap.add_argument("--cli", required=True, type=Path)
    ap.add_argument("--timeout", type=float, default=900.0)
    ap.add_argument("--cpp-limit-ms", type=int, default=70000)
    ap.add_argument("--out", type=Path, default=Path("docs/route_results.json"))
    ap.add_argument("fixtures", nargs="+")
    args = ap.parse_args()

    sys.path.insert(0, str(args.algo))
    from motion_primitives import normalize_heading

    results = []
    for fid in args.fixtures:
        path = args.algo / "benchmarks" / "fixtures" / f"{fid}.json"
        fx = json.loads(path.read_text(encoding="utf-8"))

        lines = [f"case {fid}",
                 f"start {float(fx['start']['x'])!r} {float(fx['start']['y'])!r} "
                 f"{normalize_heading(math.radians(float(fx['start']['heading_deg'])))!r}"]
        lines += [f"obs {o['id']} {float(o['x'])!r} {float(o['y'])!r} {o['face']}" for o in fx["obstacles"]]
        lines.append(f"route {args.cpp_limit_ms}")
        cases = args.out.parent / f"route_{fid}.txt"
        cases.write_text("\n".join(lines) + "\n", encoding="utf-8")

        cpp = json.loads(subprocess.run([str(args.cli), str(cases)], capture_output=True, text=True,
                                        check=True).stdout.strip().splitlines()[-1])
        try:
            out = subprocess.run([sys.executable, "-I", str(Path(__file__).with_name("py_route.py")),
                                  str(args.algo), str(path)], capture_output=True, text=True,
                                 timeout=args.timeout)
            py = json.loads(out.stdout.strip().splitlines()[-1]) if out.returncode == 0 else {"error": out.stderr[-300:]}
        except subprocess.TimeoutExpired:
            py = {"timeout": True}

        row = {"fixture": fid, "n": len(fx["obstacles"]), "cpp": cpp, "py": py}
        if "cost" in py and py.get("valid") and cpp.get("valid"):
            row["same_cost"] = abs(py["cost"] - cpp["cost"]) < 1e-6
            row["same_order"] = py["order"] == cpp["order"]
            row["same_cmds"] = [c.replace("MOV,", "") for c in py["cmds"]] == cpp["cmds"]
            row["speedup"] = py["time_us"] / max(cpp["time_us"], 1)
        results.append(row)
        print(json.dumps({k: v for k, v in row.items() if k not in ("cpp", "py")}) +
              f"  py_us={py.get('time_us', 'n/a')} cpp_us={cpp['time_us']}", flush=True)

    args.out.write_text(json.dumps(results, indent=1))


if __name__ == "__main__":
    main()
