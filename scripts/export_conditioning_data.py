"""Export the trajectory spread / conditioning / RMSE points used by the
paper's Fig. 2 (drawn natively in pgfplots from the .dat file this writes),
extracted from the run_supervised.../conditioning-only slice of the shared
renderer used across both papers in the original combined repository."""

from __future__ import annotations

import math
from pathlib import Path

import render_lcss_sv_validation as conditioning

ROOT = Path(__file__).resolve().parents[1]


def export_conditioning_pgfplots_data() -> None:
    points = [point for point in conditioning.load_points() if point[0] != "stationary"]
    points.sort(key=lambda point: point[1])
    out = ROOT / "figures" / "conditioning_points.dat"
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="\n") as handle:
        handle.write("idx lsv lkappa rmse\n")
        for index, (name, spread, kappa, rmse) in enumerate(points):
            handle.write(
                f"{index + 1} {math.log10(spread):.4f} {math.log10(kappa):.4f} {rmse:.5f}\n"
            )
    print(out)


if __name__ == "__main__":
    export_conditioning_pgfplots_data()
