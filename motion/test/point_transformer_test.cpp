#include <gtest/gtest.h>
#include <tf/transform_listener.h>
#include <point_transformer.h>

TEST(PointTransformerTest, TestCopyToPose) {
    tf::Quaternion rotation;
    rotation.setX(11.4);
    rotation.setY(12.3);
    rotation.setZ(13.2);
    rotation.setW(14.3);
    tf::Vector3 origin;
    origin.setX(2.2);
    origin.setY(1.5);
    origin.setZ(4.1);
    tf::StampedTransform in;
    in.setOrigin(origin);
    in.setRotation(rotation);

    geometry_msgs::Pose result;

    PointTransformer::copyStampedTransformToPose(in, result);

    ASSERT_EQ(result.orientation.x, 11.4);
    ASSERT_EQ(result.orientation.y, 12.3);
    ASSERT_EQ(result.orientation.z, 13.2);
    ASSERT_EQ(result.orientation.w, 14.3);
    ASSERT_EQ(result.position.x, 2.2);
    ASSERT_EQ(result.position.y, 1.5);
    ASSERT_EQ(result.position.z, 4.1);
}