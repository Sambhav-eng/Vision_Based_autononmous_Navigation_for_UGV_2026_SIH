# Is typically the file responsible for estimating how the UGV is moving using camera images.

import cv2
import numpy as np


class VisualOdometry:

    def __init__(self):

        # ORB feature detector
        self.orb = cv2.ORB_create(
            nfeatures=1000
        )

        # Previous frame
        self.previous_frame = None

        # Previous keypoints and descriptors
        self.previous_keypoints = None
        self.previous_descriptors = None

        # Estimated position
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0

        # ORB matcher
        self.matcher = cv2.BFMatcher(
            cv2.NORM_HAMMING,
            crossCheck=True
        )

    def update(self, frame):

        # Convert to grayscale
        gray = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2GRAY
        )

        # Detect features
        keypoints, descriptors = self.orb.detectAndCompute(
            gray,
            None
        )

        # First frame
        if self.previous_frame is None:

            self.previous_frame = gray
            self.previous_keypoints = keypoints
            self.previous_descriptors = descriptors

            return self.x, self.y, self.heading

        # Not enough features
        if (
            self.previous_descriptors is None
            or descriptors is None
            or len(keypoints) < 5
        ):

            return self.x, self.y, self.heading

        # Match features
        matches = self.matcher.match(
            self.previous_descriptors,
            descriptors
        )

        # Sort by distance
        matches = sorted(
            matches,
            key=lambda match: match.distance
        )

        # Keep good matches
        good_matches = matches[:50]

        if len(good_matches) >= 8:

            previous_points = np.float32([
                self.previous_keypoints[m.queryIdx].pt
                for m in good_matches
            ])

            current_points = np.float32([
                keypoints[m.trainIdx].pt
                for m in good_matches
            ])

            # Estimate transformation
            transformation, mask = cv2.estimateAffinePartial2D(
                previous_points,
                current_points
            )

            if transformation is not None:

                dx = transformation[0, 2]
                dy = transformation[1, 2]

                rotation = np.arctan2(
                    transformation[1, 0],
                    transformation[0, 0]
                )

                # Accumulate movement
                self.x += dx
                self.y += dy
                self.heading += rotation

        # Save current frame
        self.previous_frame = gray
        self.previous_keypoints = keypoints
        self.previous_descriptors = descriptors

        return self.x, self.y, self.heading