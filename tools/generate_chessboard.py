from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate a printable chessboard calibration image")
    parser.add_argument("--columns", type=int, default=9, help="Inner corner columns")
    parser.add_argument("--rows", type=int, default=6, help="Inner corner rows")
    parser.add_argument("--square-px", type=int, default=160)
    parser.add_argument("--margin-px", type=int, default=160)
    parser.add_argument("--output", default="data/calibration/chessboard_9x6.png")
    args = parser.parse_args()

    square_columns, square_rows = args.columns + 1, args.rows + 1
    board = np.full((square_rows * args.square_px, square_columns * args.square_px), 255, np.uint8)
    for row in range(square_rows):
        for column in range(square_columns):
            if (row + column) % 2 == 0:
                y1, y2 = row * args.square_px, (row + 1) * args.square_px
                x1, x2 = column * args.square_px, (column + 1) * args.square_px
                board[y1:y2, x1:x2] = 0
    output = cv2.copyMakeBorder(board, args.margin_px, args.margin_px,
                                args.margin_px, args.margin_px, cv2.BORDER_CONSTANT, value=255)
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), output):
        raise RuntimeError(f"Could not write {path}")
    print(path.resolve())
    print("Print at 100% scale and measure the physical square size before calibration.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
