"""Build and export. ``uv run python -m holder`` writes ``out/``."""

from pathlib import Path

from build123d import Box, Part

from holder.export import export_part
from holder.preview import preview_part


def placeholder_box() -> Part:
    box = Box(10, 20, 30)
    box.label = "placeholder"
    return box


def main() -> None:
    out = Path("out")
    part = placeholder_box()
    export_part(part, out / part.label)
    preview_part(part, out / f"{part.label}.png")


if __name__ == "__main__":
    main()
