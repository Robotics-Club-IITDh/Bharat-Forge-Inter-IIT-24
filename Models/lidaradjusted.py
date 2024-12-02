import numpy as np

def inverseTruncateLiDAR(lidarRanges, C, max_range=None, collisionThreshold=0.0):
    """
    Transforms LiDAR ranges inversely and truncates to a maximum value.

    Parameters:
        lidarRanges (np.array): Array of LiDAR ranges (distances).
        C (float): Maximum truncated value.
        max_range (float, optional): Maximum range of the LiDAR sensor.
        collisionThreshold (float, optional): Collision threshold distance.

    Returns:
        np.array: Transformed and truncated distances.
    """
    # Handle infinite distances or out-of-range values
    if max_range is not None:
        lidarRanges = setMaxRangeToLargeConstantLiDAR(lidarRanges, max_range, collisionThreshold)

    # Compute inverse truncated values
    inverseTruncated = np.minimum(1.0 / np.clip(lidarRanges, collisionThreshold, None), C)
    return inverseTruncated


def setMaxRangeToLargeConstantLiDAR(lidarRanges, max_range, collisionThreshold=0.0):
    """
    Adjusts LiDAR ranges to handle out-of-range values by replacing them with a large constant.

    Parameters:
        lidarRanges (np.array): Array of LiDAR ranges (distances).
        max_range (float): Maximum range of the LiDAR sensor.
        collisionThreshold (float, optional): Collision threshold distance.

    Returns:
        np.array: Adjusted LiDAR ranges.
    """
    tol = 1e-3  # Tolerance for max range comparison
    largeConstant = 1e5  # Large constant for out-of-range beams

    # Adjust distances based on collision threshold
    lidarAdjusted = lidarRanges - collisionThreshold

    # Replace distances beyond max range with the large constant
    lidarAdjusted[np.isinf(lidarRanges) | (lidarRanges > max_range - tol)] = largeConstant
    return lidarAdjusted


lidarRanges = np.array([0.5, 1.0, np.inf, 10.0, 15.0])  # Example LiDAR ranges
max_range = 10.0  # Maximum LiDAR range
collisionThreshold = 0.5  # Collision threshold
C = 10  # Truncation limit


# Step 1: Adjust ranges for out-of-range and infinite values
adjustedRanges = setMaxRangeToLargeConstantLiDAR(lidarRanges, max_range, collisionThreshold)
print("Adjusted Ranges:", adjustedRanges)

# Step 2: Compute inverse truncated values
transformedRanges = inverseTruncateLiDAR(lidarRanges, C, max_range, collisionThreshold)
print("Transformed Ranges:", transformedRanges)

import matplotlib.pyplot as plt

# Visualize original vs. transformed data
plt.plot(lidarRanges, label="Original Ranges", marker='o')
plt.plot(adjustedRanges, label="Adjusted Ranges", marker='x')
plt.plot(transformedRanges, label="Transformed (Inverse Truncated)", marker='s')
plt.xlabel("Beam Index")
plt.ylabel("Range / Transformed Value")
plt.legend()
plt.show()
