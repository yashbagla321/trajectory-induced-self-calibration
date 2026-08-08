"""Export the trajectory spread / conditioning / RMSE points used by the
paper's Fig. 2 (drawn natively in pgfplots from the .dat file this writes),
extracted from the run_supervised.../conditioning-only slice of the shared
renderer used across both papers in the original combined repository.

Reads (indirectly, via ``render_conditioning_summary.load_points()``):
    - results/information_conditioning.csv
    - results/trajectory_sweep.csv
    - results/near_degenerate_trajectory_sweep.csv

Writes:
    - figures/conditioning_points.dat

``load_points()`` (see render_conditioning_summary.py) joins each trajectory's
"trajectory_spread" and "condition_number" columns from
information_conditioning.csv with its target-position RMSE from the
trajectory sweep CSVs. NOTE: although this value is labeled "S_v" / "lsv" in
load_points()'s own docstring and in this script's output header, the actual
column consumed is "trajectory_spread", not "smallest_singular_value" (see
the more detailed note in load_points() in render_conditioning_summary.py) --
documented here structurally rather than assuming which name is authoritative.
This module excludes the degenerate "stationary" trajectory (spread = 0,
unobservable) and writes the remaining points, sorted by trajectory spread,
as a whitespace-delimited pgfplots data table so the paper's figure can be
typeset natively in LaTeX (rather than embedding a rasterized plot). The
columns are log10 of that spread/S_v value ("lsv"), log10 of the condition
number ("lkappa"), and the raw target RMSE, illustrating that
better-conditioned (larger spread, smaller kappa) self-calibration
geometries yield lower target localization error.
"""

from __future__ import annotations

import math
from pathlib import Path

import render_conditioning_summary as conditioning

ROOT = Path(__file__).resolve().parents[1]


def export_conditioning_pgfplots_data() -> None:
    """Write figures/conditioning_points.dat, a pgfplots-ready data table of
    (index, log10 trajectory-spread, log10 condition-number, target RMSE)
    for every non-stationary trajectory instance.

    Points come from ``render_conditioning_summary.load_points()``, which joins
    results/information_conditioning.csv (trajectory_spread, condition_number)
    with the target RMSE reported in results/trajectory_sweep.csv and
    results/near_degenerate_trajectory_sweep.csv. The degenerate "stationary"
    trajectory (spread = 0, rank-deficient) is dropped since its log10 is
    undefined and it is reported separately in the manuscript's tables. Points
    are sorted by trajectory spread (ascending) so the resulting pgfplots
    curve/scatter reads left-to-right from least to most excited trajectory.

    Takes no parameters and returns nothing; prints the output path as a
    side effect for convenience when run from the command line.
    """
    # Drop the stationary trajectory: spread = 0 there, so log10(spread) below
    # would be undefined (it also appears separately in the paper's tables).
    points = [point for point in conditioning.load_points() if point[0] != "stationary"]
    # point[1] is trajectory_spread; sorting by it orders the exported rows
    # from least- to most-excited trajectory for a monotonic-looking plot.
    points.sort(key=lambda point: point[1])
    out = ROOT / "figures" / "conditioning_points.dat"
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="\n") as handle:
        # Header names match the pgfplots column keys expected by the .tex source.
        handle.write("idx lsv lkappa rmse\n")
        for index, (name, spread, kappa, rmse) in enumerate(points):
            # Smallest singular value and condition number are log-scaled since
            # they span several orders of magnitude across trajectories.
            handle.write(
                f"{index + 1} {math.log10(spread):.4f} {math.log10(kappa):.4f} {rmse:.5f}\n"
            )
    print(out)


if __name__ == "__main__":
    export_conditioning_pgfplots_data()
