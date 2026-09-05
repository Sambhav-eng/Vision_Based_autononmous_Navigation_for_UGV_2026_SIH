import cv2
import numpy as np


def detect_free_space(frame):
    """
    Detect the approximate drivable/free area
    in the lower part of the camera image.
    """

    height, width = frame.shape[:2]

    # Convert image to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # --------------------------------------------------
    # Define the road/ground region
    # --------------------------------------------------

    # We initially assume the ground is relatively
    # low-saturation and reasonably bright.
    lower = np.array([0, 0, 40])
    upper = np.array([180, 100, 255])

    mask = cv2.inRange(hsv, lower, upper)

    # --------------------------------------------------
    # Only consider the lower portion of the image
    # because that is where the drivable ground is.
    # --------------------------------------------------

    roi = np.zeros_like(mask)

    roi[int(height * 0.45):height, :] = 255

    mask = cv2.bitwise_and(mask, roi)

    # --------------------------------------------------
    # Remove small noise
    # --------------------------------------------------

    kernel = np.ones((7, 7), np.uint8)

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel
    )

    # --------------------------------------------------
    # Create visualization
    # --------------------------------------------------

    free_space = frame.copy()

    # Green overlay on detected free space
    free_space[mask > 0] = (
        0.5 * free_space[mask > 0] +
        0.5 * np.array([0, 255, 0])
    ).astype(np.uint8)

    # --------------------------------------------------
    # Draw center line
    # --------------------------------------------------

    center_x = width // 2

    cv2.line(
        free_space,
        (center_x, height),
        (center_x, int(height * 0.45)),
        (255, 0, 0),
        2
    )

    return free_space, mask