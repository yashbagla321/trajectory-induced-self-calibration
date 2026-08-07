from __future__ import annotations

import csv
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results"
OUT = ROOT / "docs" / "lcss_acc2027" / "versions" / "3_minor_revision_polish" / "source" / "conditioning_summary.png"

MARKER_FILL = "#2a78d6"
MARKER_OUTLINE = "#12335c"
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
# called out in bold in the shared legend.
WEAK = {"short_line", "repeated_viewpoints"}


def font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    candidates = [
        Path("C:/Windows/Fonts/arialbd.ttf" if bold else "C:/Windows/Fonts/arial.ttf"),
        Path("C:/Windows/Fonts/calibrib.ttf" if bold else "C:/Windows/Fonts/calibri.ttf"),
    ]
    for candidate in candidates:
        if candidate.exists():
            return ImageFont.truetype(str(candidate), size)
    return ImageFont.load_default()


def rows(path: Path) -> list[dict[str, str]]:
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
    for row in rows(RESULTS / "near_degenerate_trajectory_sweep.csv"):
        rmse_by_name[row["trajectory"]] = float(row["target_rmse"])

    points = []
    for name, row in info.items():
        if name not in rmse_by_name:
            continue
        points.append(
            (
                name,
                float(row["trajectory_spread"]),
                float(row["condition_number"]),
                rmse_by_name[name],
            )
        )
    return points


def nice_ticks(lo: float, hi: float, count: int = 4) -> list[float]:
    if hi <= lo:
        return [lo]
    return [lo + i * (hi - lo) / (count - 1) for i in range(count)]


def declutter(positions: list[list[float]], min_dist: float, iterations: int = 60) -> None:
    """Nudge pixel-space marker centers apart in place so nearly-coincident
    data points (e.g. collinear_pass and excited_figure_eight, which have
    almost identical S_v and kappa) don't fully hide one another."""
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
                        dx, dy, dist = 1.0, 0.0, 1.0
                    push = (min_dist - dist) / 2.0
                    ux, uy = dx / dist, dy / dist
                    positions[i][0] -= ux * push
                    positions[i][1] -= uy * push
                    positions[j][0] += ux * push
                    positions[j][1] += uy * push
        if not moved:
            break


def draw_numbered_marker(draw: ImageDraw.ImageDraw, px: float, py: float, number: int, f_num: ImageFont.ImageFont) -> None:
    r = 11
    draw.ellipse((px - r, py - r, px + r, py + r), fill=MARKER_FILL, outline=MARKER_OUTLINE, width=2)
    text = str(number)
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
    x_pad = 0.08 * (x_max - x_min if x_max > x_min else 1.0)
    y_pad = 0.14 * (y_max - y_min if y_max > y_min else 1.0)
    x_min, x_max = x_min - x_pad, x_max + x_pad
    y_min, y_max = y_min - y_pad, y_max + y_pad

    def to_px(x: float, y: float) -> tuple[float, float]:
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
        w = f_tick.getlength(x_fmt.format(value)) if hasattr(f_tick, "getlength") else 20
        draw.text((gx - w / 2, plot_bottom + 7), x_fmt.format(value), fill="#898781", font=f_tick)

    pixel_positions = [list(to_px(x, y)) for x, y, _ in points]
    declutter(pixel_positions, min_dist=26.0)
    r = 11
    for pos in pixel_positions:
        pos[0] = min(max(pos[0], plot_left + r), plot_right - r)
        pos[1] = min(max(pos[1], plot_top + r), plot_bottom - r)
    for (px, py), (_, _, number) in zip(pixel_positions, points):
        draw_numbered_marker(draw, px, py, number, f_num)

    draw.text((left + 12, top + (bottom - top) / 2 - 8), y_label, fill="#334155", font=f_label)
    draw.text(((plot_left + plot_right) / 2 - 60, bottom - 16), x_label, fill="#334155", font=f_label)


def draw_legend(draw: ImageDraw.ImageDraw, rect: tuple[int, int, int, int], ordered_names: list[str]) -> None:
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
        col, row = divmod(i, 3)
        cx = left + 20 + col * col_w
        cy = top + 40 + row * row_h
        r = 9
        draw.ellipse((cx - r, cy - r, cx + r, cy + r), fill=MARKER_FILL, outline=MARKER_OUTLINE, width=2)
        text = str(i + 1)
        bbox = draw.textbbox((0, 0), text, font=f_num)
        tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
        draw.text((cx - tw / 2 - bbox[0], cy - th / 2 - bbox[1]), text, fill=MARKER_TEXT, font=f_num)
        is_weak = name in WEAK
        label = DISPLAY_NAME[name] + ("  (weak)" if is_weak else "")
        draw.text((cx + 16, cy - 9), label, fill="#111827" if is_weak else "#334155",
                   font=f_entry_bold if is_weak else f_entry)


def main() -> None:
    points = load_points()
    # The stationary trajectory has S_v = 0 (rank-deficient, unobservable) and is excluded
    # from the log-log panels; it already appears in Tables I and III of the manuscript.
    points = [p for p in points if p[0] != "stationary"]
    points.sort(key=lambda p: p[1])
    number_by_name = {name: i + 1 for i, (name, _, _, _) in enumerate(points)}
    ordered_names = [name for name, _, _, _ in points]

    conditioning_points = [(math.log10(sv), math.log10(kappa), number_by_name[name]) for name, sv, kappa, _ in points]
    rmse_points = [(math.log10(sv), rmse, number_by_name[name]) for name, sv, _, rmse in points]

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
