#include <gtest/gtest.h>
#include <visualization_marker.h>
#include <ros/ros.h>

TEST(VisualizationMarkerTest, TestGeneratePoseMarker) {
    geometry_msgs::Pose poseIn;
    poseIn.position.x = 0.2;
    poseIn.position.y = 0.3;
    poseIn.position.z = 0.4;
    poseIn.orientation.x = 0.5;
    poseIn.orientation.y = 0.6;
    poseIn.orientation.z = 0.7;
    poseIn.orientation.w = 1;

    visualization_msgs::Marker result = VisualizationMarker::generatePoseMarker(poseIn, "somepath", 2, "frameid");

    ASSERT_EQ(result.header.frame_id, "frameid");
    ASSERT_EQ(result.ns, "gripper_meshes");
    ASSERT_EQ(result.id, 2);
    ASSERT_EQ(result.type, visualization_msgs::Marker::MESH_RESOURCE);
    ASSERT_EQ(result.action, visualization_msgs::Marker::ADD);
    ASSERT_EQ(result.pose.position.x, 0.2);
    ASSERT_EQ(result.pose.position.y, 0.3);
    ASSERT_EQ(result.pose.position.z, 0.4);
    ASSERT_EQ(result.pose.orientation.x, 0.5);
    ASSERT_EQ(result.pose.orientation.y, 0.6);
    ASSERT_EQ(result.pose.orientation.z, 0.7);
    ASSERT_EQ(result.pose.orientation.w, 1);
    ASSERT_EQ(result.scale.x, 0.8);
    ASSERT_EQ(result.scale.y, 0.8);
    ASSERT_EQ(result.scale.z, 0.8);
    ASSERT_EQ(result.color.a, 1.0);
    ASSERT_EQ(result.color.r, 0.5);
    ASSERT_EQ(result.color.g, 0.5);
    ASSERT_EQ(result.color.b, 0.5);
    ASSERT_EQ(result.mesh_resource, "somepath");
}

TEST(VisualizationMarkerTest, TestGenerateDeleteMarker) {
    visualization_msgs::Marker result = VisualizationMarker::generateDeleteMarker();

    ASSERT_EQ(result.ns, "gripper_meshes");
    ASSERT_EQ(result.type, visualization_msgs::Marker::MESH_RESOURCE);
    ASSERT_EQ(result.action, visualization_msgs::Marker::DELETE);
}
