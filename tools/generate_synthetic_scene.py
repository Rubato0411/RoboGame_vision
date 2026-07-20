from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "data" / "synthetic_blocks.jpg"


def main() -> None:
    image = np.full((720, 1280, 3), (125, 125, 125), dtype=np.uint8)
    cv2.rectangle(image, (80, 580), (1200, 625), (25, 25, 25), -1)

    # Orange and purple blocks with small brightness differences.
    cv2.rectangle(image, (180, 300), (350, 475), (0, 135, 255), -1)
    cv2.rectangle(image, (430, 330), (600, 505), (0, 105, 220), -1)
    cv2.rectangle(image, (790, 285), (960, 460), (185, 55, 150), -1)

    # Add small colored noise that should be removed by area/morphology filters.
    cv2.circle(image, (1040, 150), 4, (0, 140, 255), -1)
    cv2.circle(image, (1120, 220), 5, (185, 55, 150), -1)

    # Mild illumination gradient approximates uneven lighting.
    gradient = np.linspace(0.82, 1.08, image.shape[1], dtype=np.float32)[None, :, None]
    image = np.clip(image.astype(np.float32) * gradient, 0, 255).astype(np.uint8)

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(OUTPUT), image)
    print(OUTPUT)


if __name__ == "__main__":
    main()
