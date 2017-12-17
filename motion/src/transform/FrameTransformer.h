#ifndef MOTION_FRAMETRANSFORMER_H
#define MOTION_FRAMETRANSFORMER_H


#include <geometry_msgs/PointStamped.h>

class FrameTransformer {
public:
    void transformPoint(const std::string& target_frame, const geometry_msgs::PointStamped& stamped_in, geometry_msgs::PointStamped& stamped_out) const;
};


#endif //MOTION_FRAMETRANSFORMER_H
