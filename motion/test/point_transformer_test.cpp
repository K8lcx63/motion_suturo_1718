#include <gtest/gtest.h>
#include <tf/transform_listener.h>
#include <point_transformer.h>

TEST(PointTransformerTest, TestCopyToPose) {
    tf::Quaternion rotation;
    rotation.setX(0.3);
    rotation.setY(0.4);
    rotation.setZ(0.5);
    rotation.setW(0.6);
    tf::Vector3 origin;
    origin.setX(2.2);
    origin.setY(1.5);
    origin.setZ(4.1);
    tf::StampedTransform in;
    in.setOrigin(origin);
    in.setRotation(rotation);

    geometry_msgs::Pose result;
    PointTransformer pointTrans;
    pointTrans.copyStampedTransformToPose(in, result);

    EXPECT_FLOAT_EQ(result.orientation.x, in.getRotation().getX());
    EXPECT_FLOAT_EQ(result.orientation.y, in.getRotation().getY());
    EXPECT_FLOAT_EQ(result.orientation.z, in.getRotation().getZ());
    EXPECT_FLOAT_EQ(result.orientation.w, in.getRotation().getW());
    EXPECT_FLOAT_EQ(result.position.x, 2.2);
    EXPECT_FLOAT_EQ(result.position.y, 1.5);
    EXPECT_FLOAT_EQ(result.position.z, 4.1);
}