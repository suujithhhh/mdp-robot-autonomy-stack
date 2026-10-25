"""Parity + speed comparison: C++ planner (this repo) vs the original Python MDP planner.

For every benchmark fixture from the MDP repo it plans the same legs the planner's own
`legs` benchmark mode uses (start -> each primary viewing pose, then pose i -> pose i+1),
with both planners, from identical exact start/goal poses and the same obstacle set, and
compares: validity, total cost, expansions, command list, final pose and wall-clock time.

    python tools/compare.py --algo <path to SC2079_MDP_T7/algo> --cli <path to mdp_plan_cli>
"""
import argparse
import json
import math
import statistics
import subprocess
import sys
from pathlib import Path


def load_fixtures(algo: Path):
    fixture_dir = algo / "benchmarks" / "fixtures"
    return [json.loads(p.read_text(encoding="utf-8")) for p in sorted(fixture_dir.glob("*.json"))]


def build_cases(algo: Path, fixtures):
    """Cases with exact start pose, obstacles and the planner's own `legs` set."""
    sys.path.insert(0, str(algo))
    from config import DEFAULT_PLANNER_CONFIG
    from models import Obstacle, Pose
    from motion_primitives import normalize_heading
    from targets import viewing_target

    cases = []
    for fx in fixtures:
        start = Pose(float(fx["start"]["x"]), float(fx["start"]["y"]),
                     normalize_heading(math.radians(float(fx["start"]["heading_deg"]))))
        obstacles = [Obstacle(o["id"], float(o["x"]), float(o["y"]), o["face"]) for o in fx["obstacles"]]
        targets = [viewing_target(o, DEFAULT_PLANNER_CONFIG) for o in obstacles]
        legs = [(start, t) for t in targets]
        if len(targets) > 1:
            legs += [(targets[i], targets[(i + 1) % len(targets)]) for i in range(len(targets))]
        cases.append({"name": fx["fixture_id"], "start": start, "obstacles": obstacles,
                      "targets": targets, "legs": legs})
    return cases


def write_cases(path: Path, cases, route_limit_ms=None):
    lines = []
    for c in cases:
        lines.append(f"case {c['name']}")
        s = c["start"]
        lines.append(f"start {s.x!r} {s.y!r} {s.heading!r}")
        for o in c["obstacles"]:
            lines.append(f"obs {o.obstacle_id} {o.x!r} {o.y!r} {o.image_face}")
        lines.append("targets")
        for a, b in c["legs"]:
            lines.append(f"leg {a.x!r} {a.y!r} {a.heading!r} {b.x!r} {b.y!r} {b.heading!r}")
        if route_limit_ms is not None:
            lines.append(f"route {route_limit_ms}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_cpp(cli: Path, cases_file: Path):
    out = subprocess.run([str(cli), str(cases_file)], capture_output=True, text=True, check=True)
    return [json.loads(line) for line in out.stdout.splitlines() if line.strip()]


def run_python_leg(algo: Path, a, b, obstacles, timeout_s):
    here = Path(__file__).with_name("py_leg.py")
    obs = json.dumps([{"id": o.obstacle_id, "x": o.x, "y": o.y, "face": o.image_face} for o in obstacles])
    args = [sys.executable, "-I", str(here), str(algo),
            repr(a.x), repr(a.y), repr(a.heading), repr(b.x), repr(b.y), repr(b.heading), obs]
    try:
        out = subprocess.run(args, capture_output=True, text=True, timeout=timeout_s)
    except subprocess.TimeoutExpired:
        return {"timeout": True}
    if out.returncode != 0:
        return {"error": out.stderr.strip()[-300:]}
    return json.loads(out.stdout.strip().splitlines()[-1])


def strip_mov(cmds):
    return [c.replace("MOV,", "") for c in cmds]


def same_float(x, y, tol):
    return abs(x - y) <= tol


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--algo", required=True, type=Path)
    ap.add_argument("--cli", required=True, type=Path)
    ap.add_argument("--out", type=Path, default=Path("docs/benchmark_results.json"))
    ap.add_argument("--timeout", type=float, default=180.0, help="per-leg Python timeout (s)")
    ap.add_argument("--only", nargs="*", help="restrict to these fixture ids")
    ap.add_argument("--repeat", type=int, default=3, help="C++ repetitions for stable timing")
    args = ap.parse_args()

    fixtures = load_fixtures(args.algo)
    if args.only:
        fixtures = [f for f in fixtures if f["fixture_id"] in args.only]
    cases = build_cases(args.algo, fixtures)

    work = args.out.parent
    work.mkdir(parents=True, exist_ok=True)
    cases_file = work / f"cases_{args.out.stem}.txt"
    write_cases(cases_file, cases)

    # C++: run several times; keep the fastest time per leg (least scheduler noise).
    cpp_runs = [run_cpp(args.cli, cases_file) for _ in range(args.repeat)]
    cpp = {}
    cpp_targets = {}
    for run in cpp_runs:
        for rec in run:
            if rec["kind"] == "targets":
                cpp_targets[rec["case"]] = rec["targets"]
                continue
            key = (rec["case"], rec["idx"])
            if key not in cpp or rec["time_us"] < cpp[key]["time_us"]:
                cpp[key] = rec

    rows = []
    target_mismatch = 0
    for c in cases:
        py_targets = [[t.x, t.y, t.heading] for t in c["targets"]]
        if cpp_targets.get(c["name"]) != py_targets:
            target_mismatch += 1
        for idx, (a, b) in enumerate(c["legs"]):
            py = run_python_leg(args.algo, a, b, c["obstacles"], args.timeout)
            cc = cpp[(c["name"], idx)]
            row = {"case": c["name"], "idx": idx, "n_obstacles": len(c["obstacles"]), "cpp": cc, "py": py}
            if "valid" in py:
                row["valid_match"] = py["valid"] == cc["valid"]
                if py["valid"] and cc["valid"]:
                    row["cost_diff"] = abs(py["cost"] - cc["cost"])
                    row["cmds_match"] = strip_mov(py["cmds"]) == cc["cmds"]
                    row["expansions_match"] = py["expansions"] == cc["expansions"]
                    row["final_diff"] = max(abs(py["final"][i] - cc["final"][i]) for i in range(2))
                    row["speedup"] = py["time_us"] / max(cc["time_us"], 1)
                else:
                    row["reason_match"] = py["reason"] == cc["reason"]
            rows.append(row)
            tag = "ok" if row.get("valid_match") else ("TIMEOUT" if py.get("timeout") else "DIFF")
            print(f"{c['name']:>22} leg {idx:2d}: {tag:7s} py={py.get('time_us', '-')}us cpp={cc['time_us']}us",
                  file=sys.stderr, flush=True)

    args.out.write_text(json.dumps({"rows": rows, "target_mismatch_cases": target_mismatch}, indent=1))
    summarise(rows, len(cases), target_mismatch)


def summarise(rows, n_cases, target_mismatch):
    done = [r for r in rows if "valid_match" in r]
    timeouts = [r for r in rows if r["py"].get("timeout")]
    errors = [r for r in rows if "error" in r["py"]]
    both_valid = [r for r in done if r["py"]["valid"] and r["cpp"]["valid"]]
    print("\n==== SUMMARY ====")
    print(f"fixtures (layouts): {n_cases}   legs: {len(rows)}   python timeouts: {len(timeouts)}   errors: {len(errors)}")
    print(f"viewing-target mismatches: {target_mismatch} fixtures")
    print(f"validity agrees: {sum(r['valid_match'] for r in done)}/{len(done)}")
    print(f"both valid: {len(both_valid)}")
    if both_valid:
        print(f"  identical command lists : {sum(r['cmds_match'] for r in both_valid)}/{len(both_valid)}")
        print(f"  identical expansions    : {sum(r['expansions_match'] for r in both_valid)}/{len(both_valid)}")
        print(f"  max |cost diff|         : {max(r['cost_diff'] for r in both_valid):.3e}")
        print(f"  max |final pose diff|   : {max(r['final_diff'] for r in both_valid):.3e}")
        sp = [r["speedup"] for r in both_valid]
        print(f"  speedup (per leg) median {statistics.median(sp):.1f}x  mean {statistics.fmean(sp):.1f}x  "
              f"min {min(sp):.1f}x  max {max(sp):.1f}x")
        tp = sum(r["py"]["time_us"] for r in both_valid)
        tc = sum(r["cpp"]["time_us"] for r in both_valid)
        print(f"  total time: python {tp/1e6:.2f}s  c++ {tc/1e6:.3f}s  -> {tp/tc:.1f}x overall")
    bad = [r for r in done if not r["valid_match"]]
    for r in bad[:10]:
        print("  validity DIFF:", r["case"], r["idx"], r["py"].get("reason"), "vs", r["cpp"].get("reason"))


if __name__ == "__main__":
    main()
