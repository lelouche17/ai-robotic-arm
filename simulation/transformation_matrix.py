import numpy as np

# Practice: Create a simple 2D rotation matrix
def rotation_matrix_2d(theta):
    """theta: angle in radians"""
    return np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])

# Test it
angle = 35/4  # 45 degrees
R = rotation_matrix_2d(angle)
print("Rotation matrix:")
print(R)

# Rotate a point
point = np.array([1, 0])
rotated = R @ point
print(f"\nOriginal point: {point}")
print(f"Rotated point: {rotated}")