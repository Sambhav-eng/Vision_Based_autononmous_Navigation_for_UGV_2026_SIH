import cv2
import numpy as np


def detect_obstacles(frame):

    hsv = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2HSV
    )

    # Red range 1
    lower_red_1 = np.array([0, 100, 80])
    upper_red_1 = np.array([10, 255, 255])

    # Red range 2
    lower_red_2 = np.array([170, 100, 80])
    upper_red_2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(
        hsv,
        lower_red_1,
        upper_red_1
    )

    mask2 = cv2.inRange(
        hsv,
        lower_red_2,
        upper_red_2
    )

    # Combine masks
    mask = mask1 | mask2

    # Remove noise
    kernel = np.ones(
        (5, 5),
        np.uint8
    )

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

    # Find obstacles
    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    obstacle_count = 0

    # Store obstacle information
    obstacles = []

    output = frame.copy()

    for contour in contours:

        area = cv2.contourArea(contour)

        if area < 300:
            continue

        x, y, w, h = cv2.boundingRect(contour)

        obstacle_count += 1

        # Center of obstacle
        center_x = x + w // 2
        center_y = y + h // 2

        # Bottom-center of obstacle
        # This is useful later for mapping
        bottom_x = x + w // 2
        bottom_y = y + h

        # Store obstacle information
        obstacles.append({
            "bbox": (x, y, w, h),
            "center": (center_x, center_y),
            "bottom_center": (bottom_x, bottom_y),
            "area": area
        })

        # Draw bounding box
        cv2.rectangle(
            output,
            (x, y),
            (x + w, y + h),
            (0, 255, 0),
            2
        )

        # Draw center
        cv2.circle(
            output,
            (center_x, center_y),
            5,
            (255, 0, 0),
            -1
        )

        # Draw bottom-center
        cv2.circle(
            output,
            (bottom_x, bottom_y),
            5,
            (0, 0, 255),
            -1
        )

        # Label
        cv2.putText(
            output,
            "OBSTACLE",
            (x, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

    # Display obstacle count
    cv2.putText(
        output,
        f"Obstacles: {obstacle_count}",
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2
    )

    return output, mask, obstacles