#!/usr/bin/env python3
"""
Generate a macOS .icns app icon.

Styles:
- rose (default): 8-petal rose curve like the app's target path.
- spiral: the previous spiral/path motif.

Exports a square PNG at multiple sizes and then builds .icns via iconutil, if available.

Usage:
    python scripts/generate_icon.py --out assets/appicon --style rose
This will create assets/appicon.iconset and assets/appicon.icns

On macOS, 'iconutil' is used to build the .icns. If not available, you'll still get the PNGs.
"""
import argparse
import os
import subprocess
from pathlib import Path

from PIL import Image, ImageDraw
import math

SIZES = [16, 32, 64, 128, 256, 512, 1024]


def _base_canvas(size: int):
    img = Image.new("RGBA", (size, size), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)
    return img, draw


def _background_circle(draw: ImageDraw.ImageDraw, size: int, color=(14, 88, 186, 255)):
    draw.ellipse((0, 0, size - 1, size - 1), fill=color)


def draw_rose_icon(size: int) -> Image.Image:
    """8-petal rose curve r = a * cos(4θ) rendered on a circular background."""
    img, draw = _base_canvas(size)
    _background_circle(draw, size)

    cx, cy = size / 2, size / 2
    max_r = size * 0.42
    a = max_r
    k = 4  # 2k petals when k is even => 8 petals
    steps = 1200
    points = []
    for i in range(steps + 1):
        theta = (i / steps) * 2 * math.pi
        r = a * math.cos(k * theta)
        x = cx + r * math.cos(theta)
        y = cy + r * math.sin(theta)
        points.append((x, y))

    line_w = max(2, int(size * 0.02))
    line_color = (194, 231, 194, 255)  # soft green similar to the plot
    draw.line(points, fill=line_color, width=line_w, joint="curve")

    # Optional subtle inner highlight ring
    highlight_color = (255, 255, 255, 22)
    inset = int(size * 0.06)
    draw.ellipse((inset, inset, size - 1 - inset, size - 1 - inset), outline=highlight_color, width=max(1, int(size * 0.01)))

    return img


def draw_spiral_icon(size: int) -> Image.Image:
    """Previous spiral icon retained as an alternative style."""
    img, draw = _base_canvas(size)
    _background_circle(draw, size)
    cx, cy = size / 2, size / 2
    max_r = size * 0.40
    turns = 2.25
    steps = 480
    points = []
    for i in range(steps + 1):
        t = i / steps
        theta = t * turns * 2 * math.pi
        r = t * max_r
        x = cx + r * math.cos(theta)
        y = cy + r * math.sin(theta)
        points.append((x, y))
    line_w = max(2, int(size * 0.06))
    draw.line(points, fill=(255, 255, 255, 255), width=line_w, joint="curve")
    ex, ey = points[-1]
    dot_r = max(2, int(size * 0.08))
    draw.ellipse((ex - dot_r, ey - dot_r, ex + dot_r, ey + dot_r), fill=(255, 255, 255, 255))
    highlight_color = (255, 255, 255, 22)
    inset = int(size * 0.06)
    draw.ellipse((inset, inset, size - 1 - inset, size - 1 - inset), outline=highlight_color, width=max(1, int(size * 0.01)))
    return img


def build_iconset(out_base: Path, style: str = "rose"):
    iconset_dir = out_base.with_suffix(".iconset")
    iconset_dir.mkdir(parents=True, exist_ok=True)

    # Render PNGs for each size + @2x
    for sz in SIZES:
        if style == "rose":
            img = draw_rose_icon(sz)
            img2x = draw_rose_icon(sz * 2)
        else:
            img = draw_spiral_icon(sz)
            img2x = draw_spiral_icon(sz * 2)
        img.save(iconset_dir / f"icon_{sz}x{sz}.png")
        img2x.save(iconset_dir / f"icon_{sz}x{sz}@2x.png")

    # Try to build .icns using iconutil on macOS
    icns_path = out_base.with_suffix(".icns")
    try:
        subprocess.run(["iconutil", "-c", "icns", str(iconset_dir), "-o", str(icns_path)], check=True)
        print(f"Wrote {icns_path}")
    except Exception as e:
        print(f"Skipping .icns build (iconutil not available?): {e}")
        print(f"Icon set available at: {iconset_dir}")

    # Also write a Windows .ico from multiple PNG sizes
    try:
        ico_path = out_base.with_suffix('.ico')
        # Use rose or spiral based on style
        imgs = []
        for sz in [16, 24, 32, 48, 64, 128, 256]:
            img = draw_rose_icon(sz) if style == 'rose' else draw_spiral_icon(sz)
            imgs.append(img)
        # PIL saves ICO from the first image; include sizes
        imgs[0].save(ico_path, format='ICO', sizes=[im.size for im in imgs])
        print(f"Wrote {ico_path}")
    except Exception as e:
        print(f"Skipping .ico build: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="assets/appicon", help="Output base path (without extension)")
    parser.add_argument("--style", default="rose", choices=["rose", "spiral"], help="Icon style to render")
    args = parser.parse_args()

    out_base = Path(args.out)
    out_base.parent.mkdir(parents=True, exist_ok=True)
    build_iconset(out_base, style=args.style)
