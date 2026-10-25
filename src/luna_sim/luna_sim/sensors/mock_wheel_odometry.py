def expand_pose_covariance(P):
    """Convert planar [x, y, yaw] covariance to ROS 6x6."""
    cov = [0.0] * 36
    indices = (0, 1, 5)

    for i in range(3):
        for j in range(3):
            cov[6 * indices[i] + indices[j]] = P[i][j]

    # Unmodeled 3D degrees of freedom.
    for index in (2, 3, 4):
        cov[6 * index + index] = 1e6

    return cov


def expand_twist_covariance(Q):
    """Convert planar [linear.x, angular.z] covariance to ROS 6x6."""
    cov = [0.0] * 36
    indices = (0, 5)

    for i in range(2):
        for j in range(2):
            cov[6 * indices[i] + indices[j]] = Q[i][j]

    for index in (1, 2, 3, 4):
        cov[6 * index + index] = 1e6

    return cov
