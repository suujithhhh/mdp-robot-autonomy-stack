"""Merge the per-group result files written by parallel runs of compare.py.

    python tools/merge_results.py docs/parts/*.json --out docs/benchmark_results.json
"""
import argparse
import json
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("parts", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, default=Path("docs/benchmark_results.json"))
    args = ap.parse_args()

    rows, mismatches = [], 0
    for part in args.parts:
        data = json.loads(part.read_text())
        rows += data["rows"]
        mismatches += data["target_mismatch_cases"]
    rows.sort(key=lambda r: (r["case"], r["idx"]))
    args.out.write_text(json.dumps({"rows": rows, "target_mismatch_cases": mismatches}, indent=1))
    print(f"merged {len(rows)} legs from {len(args.parts)} parts; target mismatches: {mismatches}")


if __name__ == "__main__":
    main()
