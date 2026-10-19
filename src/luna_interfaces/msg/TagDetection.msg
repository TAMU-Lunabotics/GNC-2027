# One detected fiducial tag.
int32 id
geometry_msgs/PoseWithCovariance pose  # pose of the TAG frame in the camera OPTICAL frame
                                       # tag frame: origin at tag center, x right, y up,
                                       # z out of the tag face (pointing toward the viewer)