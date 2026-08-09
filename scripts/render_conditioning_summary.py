"""Render the paper's "conditioning summary" figure, a two-panel
scatter plot (plus shared legend) validating that the paper's observability
conditioning metrics track both each other and the downstream target
localization error.

Reads:
    - results/information_conditioning.csv (per-trajectory
      trajectory_spread and condition_number)
    - results/trajectory_sweep.csv (per-trajectory target_rmse, for
      trajectories not present in the near-degenerate sweep below)
    - results/near_degenerate_trajectory_sweep.csv (per-trajectory
      target_rmse; preferred over trajectory_sweep.csv where both exist,
      since it is the more targeted sweep for the near-degenerate cases)

Writes:
    - figures/conditioning_summary.png

This script is also imported as a module (by scripts/export_conditioning_data.py)
purely to reuse its ``load_points()`` CSV-joining helper; only ``main()`` and
the drawing helpers are specific to the PNG rendered here.

The output image has two log-log scatter panels sharing a numbered legend of
trajectory names: (1) "Conditioning tracks trajectory spread" -- condition
number vs. trajectory spread, showing the two are correlated -- and (2)
"Target error falls as trajectory spread grows" -- target RMSE vs.
trajectory spread, showing that more-excited trajectories (wider viewing
geometry variation) yield lower self-calibration error. The plot renders
everything with Pillow (no matplotlib dependency) so it can run in a
minimal Python environment; numbered circular markers plus a legend are
used instead of per-point labels because several trajectories have nearly
identical coordinates (see ``declutter``).

NOTE on naming: ``load_points()`` below and this module's own historical
docstring describe the second element of each returned tuple as "S_v" (the
smallest singular value), and the exported pgfplots columns are named
accordingly ("lsv"). However the code actually reads the CSV's
"trajectory_spread" column, not "smallest_singular_value". This is
documented here structurally, without assuming which name is authoritative,
since correcting it would be a logic change out of scope for this pass.
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

from plot_fonts import load_font

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results"
OUT = ROOT / "figures" / "conditioning_summary.png"

# Marker palette, matched to the paper's Fig. 2 colors: the two weakly
# excited trajectories (see WEAK below) are drawn in the paper's red and
# every well-excited trajectory in the paper's blue.
MARKER_FILL = "#2563eb"
MARKER_OUTLINE = "#1e3a8a"
WEAK_MARKER_FILL = "#e11d48"
WEAK_MARKER_OUTLINE = "#881337"
MARKER_TEXT = "#ffffff"

# Display name for the shared legend, keyed by the trajectory field in the CSVs.
DISPLAY_NAME = {
    "short_line": "short-line",
    "repeated_viewpoints": "repeated-viewpoints",
    "low_curvature_arc": "low-curvature arc",
    "collinear_pass": "collinear pass",
    "line": "line",
    "figure_eight": "figure-eight",
    "excited_figure_eight": "excited figure-eight",
    "excited": "excited",
    "circle": "circle",
}
# The two weakly excited trajectories (full rank but poorly conditioned) are
# drawn with red markers in both panels and called out in bold in the shared
# legend.
WEAK = {"short_line", "repeated_viewpoints"}


def font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    """Load a TrueType font at the given pixel size. Thin wrapper around
    plot_fonts.load_font() (shared with the other rendering scripts in
    this repo), kept under this module's original name since it is called
    throughout the rest of this file.
    """
    return load_font(size, bold)


def rows(path: Path) -> list[dict[str, str]]:
    """Read a CSV file at ``path`` and return its rows as a list of
    header-keyed dicts (via ``csv.DictReader``); all values remain strings.
    """
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def load_points() -> list[tuple[str, float, float, float]]:
    """Return (trajectory, S_v, condition_number, target_rmse) for every trajectory instance."""
    info = {row["trajectory"]: row for row in rows(RESULTS / "information_conditioning.csv")}
    # near_degenerate_trajectory_sweep.csv is preferred where available; the remaining
    # trajectory names (circle, figure_eight, excited) come only from trajectory_sweep.csv.
    rmse_by_name: dict[str, float] = {}
    for row in rows(RESULTS / "trajectory_sweep.csv"):
        rmse_by_name[row["trajectory"]] = float(row["target_rmse"])
    # Overwrite with near_degenerate_trajectory_sweep.csv's values for any trajectory
    # names that also appear there, since that sweep targets the near-degenerate
    # regime more precisely; trajectories unique to trajectory_sweep.csv keep the
    # values set above.
    for row in rows(RESULTS / "near_degenerate_trajectory_sweep.csv"):
        rmse_by_name[row["trajectory"]] = float(row["target_rmse"])

    points = []
    for name, row in info.items():
        # Skip any trajectory present in information_conditioning.csv but missing
        # an RMSE counterpart in either sweep CSV -- the join is otherwise inner.
        if name not in rmse_by_name:
            continue
        points.append(
            (
                name,
                # NOTE: despite being referred to as "S_v" elsewhere in this file,
                # this is the "trajectory_spread" column, not "smallest_singular_value".
                float(row["trajectory_spread"]),
                float(row["condition_number"]),
                rmse_by_name[name],
            )
        )
    return points


def nice_ticks(lo: float, hi: float, count: int = 4) -> list[float]:
    """Return ``count`` evenly spaced tick values spanning [lo, hi] inclusive
    (simple linear interpolation, not "nice round number" snapping despite
    the name). If ``hi <= lo`` the range is degenerate and a single tick at
    ``lo`` is returned to avoid a division by zero.
    """
    if hi <= lo:
        return [lo]
    return [lo + i * (hi - lo) / (count - 1) for i in range(count)]


def declutter(positions: list[list[float]], min_dist: float, iterations: int = 60) -> None:
    """Nudge pixel-space marker centers apart in place so nearly-coincident
    data points (e.g. collinear_pass and excited_figure_eight, which have
    almost identical S_v and kappa) don't fully hide one another.

    This is a simple relaxation/force-directed pass: on each iteration, every
    pair of points closer than ``min_dist`` pixels apart is pushed apart
    along their connecting line by half the overlap each, repeated up to
    ``iterations`` times (stopping early once a full pass makes no moves).

    Parameters:
        positions: list of mutable [x, y] pixel coordinates, modified in
            place.
        min_dist: minimum allowed pixel distance between any two marker
            centers.
        iterations: maximum number of relaxation passes to run.

    Returns: None (mutates ``positions`` in place).
    """
    n = len(positions)
    for _ in range(iterations):
        moved = False
        for i in range(n):
            for j in range(i + 1, n):
                dx = positions[j][0] - positions[i][0]
                dy = positions[j][1] - positions[i][1]
                dist = math.hypot(dx, dy)
                if dist < min_dist:
                    moved = True
                    if dist < 1e-6:
                        # Points are (numerically) coincident, so the direction
                        # between them is undefined; pick an arbitrary axis
                        # (+x) to push them apart along.
                        dx, dy, dist = 1.0, 0.0, 1.0
                    push = (min_dist - dist) / 2.0
                    ux, uy = dx / dist, dy / dist
                    positions[i][0] -= ux * push
                    positions[i][1] -= uy * push
                    positions[j][0] += ux * push
                    positions[j][1] += uy * push
        if not moved:
            break


def draw_numbered_marker(draw: ImageDraw.ImageDraw, px: float, py: float, number: int, f_num: ImageFont.ImageFont, weak: bool = False) -> None:
    """Draw a single filled circular marker (radius 11px) at pixel
    coordinates (px, py) with ``number`` centered inside it in white text.

    Parameters:
        draw: the Pillow drawing context to render onto.
        px, py: pixel-space center coordinates of the marker.
        number: the integer label to render inside the marker (matches the
            trajectory's index in the shared legend).
        f_num: font used to render the label text.
        weak: True renders the weakly-excited red marker colors instead of
            the default blue.

    Returns: None (draws directly onto ``draw``'s underlying image).
    """
    r = 11
    draw.ellipse((px - r, py - r, px + r, py + r),
                 fill=WEAK_MARKER_FILL if weak else MARKER_FILL,
                 outline=WEAK_MARKER_OUTLINE if weak else MARKER_OUTLINE, width=2)
    text = str(number)
    # Measure the text's bounding box so it can be centered exactly within
    # the circle rather than merely anchored at a corner.
    bbox = draw.textbbox((0, 0), text, font=f_num)
    tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
    draw.text((px - tw / 2 - bbox[0], py - th / 2 - bbox[1]), text, fill=MARKER_TEXT, font=f_num)


def draw_scatter(
    draw: ImageDraw.ImageDraw,
    rect: tuple[int, int, int, int],
    points: list[tuple[float, float, int]],
    title: str,
    x_label: str,
    y_label: str,
    x_fmt: str,
    y_fmt: str,
) -> None:
    """Draw one titled, axis-labeled scatter panel with numbered markers
    inside a bordered card.

    Lays out a card at ``rect`` containing a title, a plotted axes box with
    gridlines/ticks, and one numbered circular marker per point (using
    ``declutter`` to separate near-coincident points before drawing), plus
    axis labels along the left and bottom edges.

    Parameters:
        draw: Pillow drawing context to render onto.
        rect: (left, top, right, bottom) pixel bounds of the panel card.
        points: list of (x, y, number, weak) in data space, where ``number``
            is the marker's integer legend index and ``weak`` selects the
            red weakly-excited marker colors.
        title: panel title text.
        x_label, y_label: axis label text.
        x_fmt, y_fmt: ``str.format``-style format strings (e.g. "{:.1f}")
            used to render tick label values.

    Returns: None (draws directly onto ``draw``'s underlying image).
    """
    left, top, right, bottom = rect
    f_title = font(22, True)
    f_label = font(15, True)
    f_tick = font(13)
    f_num = font(13, True)
    draw.rectangle(rect, fill="#fffdf7", outline="#cbd5e1", width=2)
    draw.text((left + 16, top + 10), title, fill="#111827", font=f_title)

    plot_left, plot_top = left + 78, top + 46
    plot_right, plot_bottom = right - 30, bottom - 54
    draw.rectangle((plot_left, plot_top, plot_right, plot_bottom), outline="#94a3b8", width=1)

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    # Pad the data range so markers near the extremes aren't drawn flush
    # against the axes box; y gets more padding (14% vs 8%) to leave room
    # for markers that get pushed vertically by declutter() below. Falls
    # back to a unit range when all points share the same coordinate so the
    # division by (x_max - x_min) in to_px() below can't divide by zero.
    x_pad = 0.08 * (x_max - x_min if x_max > x_min else 1.0)
    y_pad = 0.14 * (y_max - y_min if y_max > y_min else 1.0)
    x_min, x_max = x_min - x_pad, x_max + x_pad
    y_min, y_max = y_min - y_pad, y_max + y_pad

    def to_px(x: float, y: float) -> tuple[float, float]:
        # Linearly map data-space (x, y) to pixel-space, flipping the y axis
        # (image y grows downward, but plot_top should correspond to y_max).
        px = plot_left + (x - x_min) / (x_max - x_min) * (plot_right - plot_left)
        py = plot_bottom - (y - y_min) / (y_max - y_min) * (plot_bottom - plot_top)
        return px, py

    for value in nice_ticks(y_min, y_max, 4):
        _, gy = to_px(x_min, value)
        draw.line((plot_left, gy, plot_right, gy), fill="#e5e7eb", width=1)
        draw.text((left + 8, gy - 7), y_fmt.format(value), fill="#898781", font=f_tick)

    for value in nice_ticks(x_min, x_max, 4):
        gx, _ = to_px(value, y_min)
        draw.line((gx, plot_bottom, gx, plot_bottom + 4), fill="#94a3b8", width=1)
        # getlength() isn't available on the bitmap default font Pillow falls
        # back to when no TrueType font is found; use a fixed width guess then.
        w = f_tick.getlength(x_fmt.format(value)) if hasattr(f_tick, "getlength") else 20
        draw.text((gx - w / 2, plot_bottom + 7), x_fmt.format(value), fill="#898781", font=f_tick)

    pixel_positions = [list(to_px(x, y)) for x, y, _, _ in points]
    # Separate markers that would otherwise overlap (some trajectories have
    # nearly identical S_v/kappa values) before drawing them.
    declutter(pixel_positions, min_dist=26.0)
    r = 11
    # Clamp decluttered positions back inside the axes box, in case the
    # push-apart nudging moved a marker past the plot border.
    for pos in pixel_positions:
        pos[0] = min(max(pos[0], plot_left + r), plot_right - r)
        pos[1] = min(max(pos[1], plot_top + r), plot_bottom - r)
    for (px, py), (_, _, number, weak) in zip(pixel_positions, points):
        draw_numbered_marker(draw, px, py, number, f_num, weak)

    draw.text((left + 12, top + (bottom - top) / 2 - 8), y_label, fill="#334155", font=f_label)
    draw.text(((plot_left + plot_right) / 2 - 60, bottom - 16), x_label, fill="#334155", font=f_label)


def draw_legend(draw: ImageDraw.ImageDraw, rect: tuple[int, int, int, int], ordered_names: list[str]) -> None:
    """Draw the shared legend card mapping each numbered marker to its
    trajectory's display name, laid out in a fixed 3-column grid.

    Weakly-excited trajectories (full observability rank but poorly
    conditioned, per ``WEAK``) are rendered bold with a "(weak)" suffix to
    call them out visually.

    Parameters:
        draw: Pillow drawing context to render onto.
        rect: (left, top, right, bottom) pixel bounds of the legend card.
        ordered_names: trajectory names in the same order as their marker
            numbers (i.e. ``ordered_names[i]`` is marker number ``i + 1``).

    Returns: None (draws directly onto ``draw``'s underlying image).
    """
    left, top, right, bottom = rect
    draw.rectangle(rect, fill="#fffdf7", outline="#cbd5e1", width=2)
    f_head = font(15, True)
    f_entry = font(14)
    f_entry_bold = font(14, True)
    f_num = font(12, True)
    draw.text((left + 16, top + 8), "Trajectory key (shared by both panels)", fill="#111827", font=f_head)

    cols, col_w = 3, (right - left - 32) / 3
    row_h = 26
    for i, name in enumerate(ordered_names):
        # Lay entries out row-major across a fixed 3-column grid.
        col, row = divmod(i, 3)
        cx = left + 20 + col * col_w
        cy = top + 40 + row * row_h
        r = 9
        is_weak = name in WEAK
        draw.ellipse((cx - r, cy - r, cx + r, cy + r),
                     fill=WEAK_MARKER_FILL if is_weak else MARKER_FILL,
                     outline=WEAK_MARKER_OUTLINE if is_weak else MARKER_OUTLINE, width=2)
        text = str(i + 1)
        bbox = draw.textbbox((0, 0), text, font=f_num)
        tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
        draw.text((cx - tw / 2 - bbox[0], cy - th / 2 - bbox[1]), text, fill=MARKER_TEXT, font=f_num)
        label = DISPLAY_NAME[name] + ("  (weak)" if is_weak else "")
        draw.text((cx + 16, cy - 9), label, fill="#111827" if is_weak else "#334155",
                   font=f_entry_bold if is_weak else f_entry)


def main() -> None:
    """Build and save the conditioning-summary PNG.

    Loads all trajectory points via ``load_points()``, drops the degenerate
    "stationary" trajectory, sorts the rest by trajectory spread to assign
    stable legend numbers, then renders the two log-log scatter panels
    ("conditioning vs. spread" and "RMSE vs. spread") plus the shared
    trajectory-key legend onto a single 1100x800 canvas, and writes it to
    ``OUT`` (figures/conditioning_summary.png). Also prints a per-trajectory
    summary line to stdout for quick inspection when run from the command
    line.

    Takes no parameters and returns nothing.
    """
    points = load_points()
    # The stationary trajectory has S_v = 0 (rank-deficient, unobservable) and is excluded
    # from the log-log panels; it already appears in Tables I and III of the manuscript.
    points = [p for p in points if p[0] != "stationary"]
    points.sort(key=lambda p: p[1])
    # Marker numbers are assigned by ascending trajectory spread so the legend
    # and both scatter panels agree on which number identifies which trajectory.
    number_by_name = {name: i + 1 for i, (name, _, _, _) in enumerate(points)}
    ordered_names = [name for name, _, _, _ in points]

    # Both quantities span orders of magnitude across trajectories, so plot
    # them in log10 space; RMSE (already in meters, a narrower range) is left linear.
    conditioning_points = [
        (math.log10(sv), math.log10(kappa), number_by_name[name], name in WEAK) for name, sv, kappa, _ in points
    ]
    rmse_points = [(math.log10(sv), rmse, number_by_name[name], name in WEAK) for name, sv, _, rmse in points]

    img = Image.new("RGB", (1100, 800), "#f8fafc")
    draw = ImageDraw.Draw(img)
    draw_scatter(
        draw,
        (30, 28, 1070, 320),
        conditioning_points,
        "Conditioning tracks trajectory spread",
        "log10 S_v",
        "log10 kappa",
        "{:.1f}",
        "{:.1f}",
    )
    draw_scatter(
        draw,
        (30, 348, 1070, 640),
        rmse_points,
        "Target error falls as trajectory spread grows",
        "log10 S_v",
        "RMSE (m)",
        "{:.1f}",
        "{:.3f}",
    )
    draw_legend(draw, (30, 668, 1070, 776), ordered_names)

    OUT.parent.mkdir(parents=True, exist_ok=True)
    img.save(OUT)
    print(OUT)
    for name, sv, kappa, rmse in points:
        print(f"{number_by_name[name]}: {name:22s} S_v={sv:10.3f} log10Sv={math.log10(sv):6.3f} kappa={kappa:12.3f} rmse={rmse:8.5f}")


if __name__ == "__main__":
    main()
