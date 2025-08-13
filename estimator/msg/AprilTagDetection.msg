# Tag ID(s). If a standalone tag, it is a vector of size 1. If a tag bundle,it is a vector containing the IDs of each tag in the bundle.
int32[] id

# Tag size(s). If a standalone tag, it is a vector of size 1. If a tag bundle,
# it is a vector containing the sizes of each tag in the bundle, in the same order as the IDs above.
float64[] size

# 相机系下的位姿矩阵, 由单应变换得到. 
# 对于单独一个standalone tag,单应矩阵来自于tag的四个角. 对于tag bundle束, 单应矩阵至少来自一个成员标签的四个角，最多来自所有成员标签的四个角
geometry_msgs/PoseWithCovarianceStamped pose