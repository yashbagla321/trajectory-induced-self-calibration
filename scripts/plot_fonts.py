"""Shared cross-platform TrueType font loader for this repo's Pillow-based
figure-rendering scripts (currently just render_conditioning_summary.py).

Tries a handful of common sans-serif fonts across Windows, Linux, and macOS,
in order, before falling back to Pillow's built-in bitmap default font. The
bitmap default renders at a fixed, small size, so a figure regenerated on a
machine that hits it will look visibly different (blockier, unscaled text)
from the ones checked into this repo -- callers get a one-time stderr
warning so that difference isn't silent.
"""

from __future__ import annotations

import sys
from pathlib import Path

from PIL import ImageFont

# (regular, bold) path candidates, tried in order, spanning the common
# sans-serif fonts available out of the box on Windows, Linux, and macOS.
_CANDIDATES: list[tuple[str, str]] = [
    ("C:/Windows/Fonts/arial.ttf", "C:/Windows/Fonts/arialbd.ttf"),
    ("C:/Windows/Fonts/calibri.ttf", "C:/Windows/Fonts/calibrib.ttf"),
    ("C:/Windows/Fonts/segoeui.ttf", "C:/Windows/Fonts/segoeuib.ttf"),
    (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    ),
    (
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    ),
    (
        "/usr/share/fonts/truetype/msttcorefonts/Arial.ttf",
        "/usr/share/fonts/truetype/msttcorefonts/Arial_Bold.ttf",
    ),
    (
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/System/Library/Fonts/Supplemental/Arial Bold.ttf",
    ),
    ("/Library/Fonts/Arial.ttf", "/Library/Fonts/Arial Bold.ttf"),
]

_warned_fallback = False


def load_font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    """Loads a TrueType font at ``size`` pixels.

    Tries, in order, the Windows/Linux/macOS candidates in ``_CANDIDATES``
    above (regular or bold per ``bold``), and falls back to Pillow's
    built-in bitmap default font (with a one-time stderr warning) if none
    of them are installed on the current machine.

    Parameters:
        size: point/pixel size to render the font at.
        bold: whether to prefer the bold variant of each candidate font.
    Returns: a Pillow ``ImageFont`` instance usable with ``ImageDraw``.
    """
    global _warned_fallback
    for regular, bold_path in _CANDIDATES:
        candidate = Path(bold_path if bold else regular)
        if candidate.exists():
            return ImageFont.truetype(str(candidate), size)
    if not _warned_fallback:
        print(
            "warning: no TrueType font found among the known Windows/Linux/macOS "
            "candidates; falling back to Pillow's bitmap default font. Figures "
            "rendered here will look visibly different (smaller, unscaled text) "
            "from the ones checked into this repo. Install one of Arial, Calibri, "
            "Segoe UI, DejaVu Sans, or Liberation Sans to match.",
            file=sys.stderr,
        )
        _warned_fallback = True
    return ImageFont.load_default()
