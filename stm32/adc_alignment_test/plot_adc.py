#!/usr/bin/env python3
"""Read 1024 ADC samples from a file and save a plot image."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

try:
    import matplotlib.pyplot as plt
except ImportError as exc:
    raise SystemExit(
        "matplotlib is required to run this script. Install it with: pip install matplotlib"
    ) from exc


def parse_samples(source: str) -> list[int]:
    values: list[int] = []
    for line in source.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        for token in line.replace(",", " ").split():
            try:
                values.append(int(token))
            except ValueError:
                raise ValueError(f"Invalid ADC value: {token!r}")
    return values


def read_samples(path: Path | None) -> list[int]:
    if path is None:
        text = sys.stdin.read()
    else:
        text = path.read_text(encoding="utf-8")
    return parse_samples(text)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read ADC sample values from a text file and plot them to an image."
    )
    parser.add_argument(
        "input",
        nargs="?",
        default="adc_samples.txt",
        help="Input file containing one ADC sample per line (default: adc_samples.txt). Use '-' for stdin.",
    )
    parser.add_argument(
        "-o",
        "--output",
        default="adc_plot.png",
        help="Output image path (default: adc_plot.png).",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display the plot window after saving the image.",
    )
    parser.add_argument(
        "--title",
        default="ADC Sample Plot",
        help="Custom title for the plot.",
    )
    args = parser.parse_args()

    input_path = None if args.input == "-" else Path(args.input)
    if input_path is not None and not input_path.exists():
        raise SystemExit(f"Input file not found: {input_path}")

    samples = read_samples(input_path)
    if not samples:
        raise SystemExit("No ADC samples found in input.")

    if len(samples) != 1024:
        print(
            f"Warning: expected 1024 samples but read {len(samples)}. Plotting available values.",
            file=sys.stderr,
        )

    indices = list(range(len(samples)))

    plt.figure(figsize=(12, 5))
    plt.plot(indices, samples, color="tab:blue", linewidth=1)
    plt.scatter(indices, samples, s=5, color="tab:blue")
    plt.title(args.title)
    plt.xlabel("Sample index")
    plt.ylabel("ADC reading")
    plt.grid(True, linestyle="--", alpha=0.4)

    minimum = min(samples)
    maximum = max(samples)
    mean = sum(samples) / len(samples)
    stats = f"count={len(samples)}, min={minimum}, max={maximum}, mean={mean:.2f}"
    plt.annotate(
        stats,
        xy=(0.99, 0.01),
        xycoords="axes fraction",
        ha="right",
        va="bottom",
        fontsize=9,
        bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8),
    )

    plt.tight_layout()
    plt.savefig(args.output, dpi=150)
    print(f"Saved plot image to {args.output}")

    if args.show:
        plt.show()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
