#!/usr/bin/env python3
"""
Generate a simple DMG background PNG with an instruction arrow.

Output: assets/dmg_background.png (default 720x420)
"""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

W, H = 720, 420
OUT = Path("assets/dmg_background.png")


def load_font(size: int) -> ImageFont.FreeTypeFont:
    # Try a few common fonts; fall back to default
    candidates = [
        "/System/Library/Fonts/SFNS.ttf",
        "/System/Library/Fonts/SFNSDisplay.ttf",
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/System/Library/Fonts/Supplemental/Helvetica.ttf",
    ]
    for p in candidates:
        try:
            return ImageFont.truetype(p, size)
        except Exception:
            pass
    return ImageFont.load_default()


def main():
    img = Image.new("RGBA", (W, H), (245, 247, 252, 255))
    d = ImageDraw.Draw(img)

    # Header text
    title_font = load_font(28)
    info_font = load_font(16)
    title = "Install MPPT-App"
    info = "Drag MPPT-App.app to Applications →"
    tb = d.textbbox((0, 0), title, font=title_font)
    tw, th = tb[2] - tb[0], tb[3] - tb[1]
    d.text(((W - tw) / 2, 24), title, fill=(30, 45, 80, 255), font=title_font)

    # Draw arrow from left-center to right-center
    # Points roughly align with icon coordinates in build_dmg.sh
    x1, y1 = 220, 220
    x2, y2 = 520, 220
    d.line([(x1, y1), (x2, y2)], fill=(90, 110, 160, 220), width=10)
    # Arrow head
    head = [(x2, y2), (x2 - 24, y2 - 14), (x2 - 24, y2 + 14)]
    d.polygon(head, fill=(90, 110, 160, 220))

    ib = d.textbbox((0, 0), info, font=info_font)
    iw, ih = ib[2] - ib[0], ib[3] - ib[1]
    d.text(((W - iw) / 2, H - ih - 28), info, fill=(40, 55, 90, 255), font=info_font)

    OUT.parent.mkdir(parents=True, exist_ok=True)
    img.save(OUT)
    print(f"Wrote {OUT}")


if __name__ == "__main__":
    main()
