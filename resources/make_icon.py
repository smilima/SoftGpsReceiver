#!/usr/bin/env python3
"""Generates SoftGpsReceiver.ico. Requires Pillow and numpy.

    python resources/make_icon.py

The icon is the sky plot from the "Satellites & orbits" tab: the satellites sit
where they really were in the sample recording, and the lines are the
pseudoranges meeting at the position fix. The artwork is redrawn for each size
class rather than scaled, because the rings and rays of the large version turn
to mush at 16 px. The smallest class keeps three satellites and moves them onto
the rim so they stay readable.
"""
import math
import struct
from io import BytesIO
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

SIZES = (16, 20, 24, 32, 40, 48, 64, 96, 128, 256)
UNITS = 256  # all artwork coordinates are in a 256 x 256 design space
CENTER = UNITS / 2

TILE_TOP = (28, 96, 178)
TILE_BOTTOM = (9, 34, 84)
GRID = (150, 224, 255)
WHITE = (255, 255, 255)
GREEN = (52, 211, 120)
AMBER = (255, 190, 60)
PINK = (240, 80, 150)
VIOLET = (170, 135, 255)
TEAL = (64, 224, 208)
FIX = (255, 122, 48)

# Azimuth and elevation (deg) at epoch 0 of the sample recording
PRN_29 = (41.6, 19.7, GREEN)
PRN_3 = (309.9, 22.6, AMBER)
PRN_9 = (119.1, 17.5, PINK)
PRN_6 = (174.0, 14.7, VIOLET)
PRN_22 = (268.7, 46.3, TEAL)

# Size classes, largest first: (minimum size, artwork)
STYLES = (
    (64, dict(corner=50, rim=2.5, sky=100, grid=3, rings=((100 / 3, 3.5), (200 / 3, 3.5)), outer=7,
              sats=(PRN_29, PRN_3, PRN_9, PRN_6, PRN_22), sat=15, halo=4.5, ray=4.5,
              fix_halo=24, fix_ring=(18.5, 6.5), fix_dot=8)),
    (32, dict(corner=46, rim=0, sky=104, grid=0, rings=((58, 7),), outer=12,
              sats=(PRN_29, PRN_3, PRN_9, PRN_6), sat=21, halo=5, ray=9,
              fix_halo=0, fix_ring=None, fix_dot=19)),
    (0, dict(corner=44, rim=0, sky=106, grid=0, rings=(), outer=16,
             sats=((41.6, 19, GREEN), (174.0, 19, PINK), (268.7, 19, AMBER)), sat=25, halo=0, ray=11,
             fix_halo=0, fix_ring=None, fix_dot=21)),
)


def project(az, el, radius):
    """Same projection as SkyPlotWidget."""
    r = radius * (90 - el) / 90
    a = math.radians(az)
    return CENTER + r * math.sin(a), CENTER - r * math.cos(a)


class Canvas:
    """Supersampled canvas; every shape is a mask painted through a fill."""

    def __init__(self, size):
        self.size = size
        self.px = size * (8 if size <= 64 else 4)
        self.scale = self.px / UNITS
        self.image = Image.new("RGBA", (self.px, self.px), (0, 0, 0, 0))

    def paint(self, fill, shape, alpha=1.0):
        mask = Image.new("L", (self.px, self.px), 0)
        shape(ImageDraw.Draw(mask))
        if alpha < 1.0:
            mask = mask.point(lambda v: round(v * alpha))
        layer = fill.copy() if isinstance(fill, Image.Image) else Image.new("RGBA", mask.size, fill)
        layer.putalpha(mask)
        self.image.alpha_composite(layer)

    def box(self, cx, cy, r):
        s = self.scale
        return [(cx - r) * s, (cy - r) * s, (cx + r) * s, (cy + r) * s]

    def disc(self, cx, cy, r):
        return lambda d: d.ellipse(self.box(cx, cy, r), fill=255)

    def ring(self, cx, cy, r, width):
        def shape(d):
            d.ellipse(self.box(cx, cy, r + width / 2), fill=255)
            d.ellipse(self.box(cx, cy, r - width / 2), fill=0)
        return shape

    def line(self, x0, y0, x1, y1, width):
        s = self.scale

        def shape(d):
            d.line([x0 * s, y0 * s, x1 * s, y1 * s], fill=255, width=max(1, round(width * s)))
            d.ellipse(self.box(x0, y0, width / 2), fill=255)
            d.ellipse(self.box(x1, y1, width / 2), fill=255)
        return shape

    def tile(self, inset, corner, width=0):
        s = self.scale
        box = [inset * s, inset * s, (UNITS - inset) * s, (UNITS - inset) * s]
        if width:
            return lambda d: d.rounded_rectangle(box, radius=corner * s, outline=255, width=round(width * s))
        return lambda d: d.rounded_rectangle(box, radius=corner * s, fill=255)

    def tile_fill(self):
        """Vertical gradient with a soft glow behind the sky plot."""
        t = np.linspace(0, 1, self.px)[:, None, None]
        rgb = np.array(TILE_TOP, float) + (np.array(TILE_BOTTOM, float) - TILE_TOP) * t
        rgb = np.broadcast_to(rgb, (self.px, self.px, 3))
        y, x = np.mgrid[0:self.px, 0:self.px] / (self.px - 1) - 0.5
        glow = np.clip(1 - np.hypot(x, y) / 0.7, 0, 1)[..., None] ** 2 * 0.16
        rgb = rgb + (255 - rgb) * glow * (0.45, 0.7, 1.0)
        alpha = np.full((self.px, self.px, 1), 255.0)
        return Image.fromarray(np.concatenate([rgb, alpha], axis=2).astype(np.uint8))


def render(size):
    style = next(st for minimum, st in STYLES if size >= minimum)
    c = Canvas(size)
    sky = style["sky"]
    inset = (size // 32) * UNITS / size  # a whole number of pixels, so the tile edge stays sharp

    c.paint(c.tile_fill(), c.tile(inset, style["corner"]))
    if style["rim"]:  # keeps the edge of the tile visible on a dark taskbar
        c.paint(WHITE + (255,), c.tile(inset, style["corner"], style["rim"]), 0.18)

    if style["grid"]:
        lo, hi = CENTER - sky, CENTER + sky
        c.paint(GRID + (255,), c.line(CENTER, lo, CENTER, hi, style["grid"]), 0.28)
        c.paint(GRID + (255,), c.line(lo, CENTER, hi, CENTER, style["grid"]), 0.28)
    for r, width in style["rings"]:
        c.paint(GRID + (255,), c.ring(CENTER, CENTER, r, width), 0.45)
    c.paint(GRID + (255,), c.ring(CENTER, CENTER, sky, style["outer"]))

    sats = [(*project(az, el, sky), colour + (255,)) for az, el, colour in style["sats"]]
    for x, y, colour in sats:
        c.paint(colour, c.line(x, y, CENTER, CENTER, style["ray"]))
    for x, y, colour in sats:
        if style["halo"]:
            c.paint(TILE_BOTTOM + (255,), c.disc(x, y, style["sat"] + style["halo"]))
        c.paint(colour, c.disc(x, y, style["sat"]))

    if style["fix_halo"]:
        c.paint(TILE_BOTTOM + (255,), c.disc(CENTER, CENTER, style["fix_halo"]))
    if style["fix_ring"]:
        c.paint(FIX + (255,), c.ring(CENTER, CENTER, *style["fix_ring"]))
    c.paint(WHITE + (255,), c.disc(CENTER, CENTER, style["fix_dot"]))

    return c.image.resize((size, size), Image.BOX)


def dib(image):
    """Uncompressed 32-bit icon frame: BGRA pixels, bottom-up, then the 1-bit AND mask."""
    w, h = image.size
    pixels = np.asarray(image)[::-1]
    header = struct.pack("<IiiHHIIiiII", 40, w, h * 2, 1, 32, 0, 0, 0, 0, 0, 0)
    stride = (w + 31) // 32 * 4
    mask = np.packbits(pixels[..., 3] < 128, axis=1)
    mask = np.pad(mask, ((0, 0), (0, stride - mask.shape[1])))
    return header + pixels[..., [2, 1, 0, 3]].tobytes() + mask.tobytes()


def png(image):
    out = BytesIO()
    image.save(out, "PNG", optimize=True)
    return out.getvalue()


def write_ico(path, images):
    """Small frames stay uncompressed for compatibility; only 256 px is PNG."""
    frames = [png(im) if im.width >= 256 else dib(im) for im in images]
    offset = 6 + 16 * len(frames)
    out = struct.pack("<HHH", 0, 1, len(frames))
    for im, frame in zip(images, frames):
        side = im.width % 256  # 0 means 256
        out += struct.pack("<BBBBHHII", side, side, 0, 0, 1, 32, len(frame), offset)
        offset += len(frame)
    path.write_bytes(out + b"".join(frames))


if __name__ == "__main__":
    target = Path(__file__).with_name("SoftGpsReceiver.ico")
    write_ico(target, [render(size) for size in SIZES])
    print(f"wrote {target} ({target.stat().st_size} bytes)")
