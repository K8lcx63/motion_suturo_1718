// Bring in my package's API, which is what I'm testing
#include <Gripper.h>
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(Gripper, test1) {
    Gripper gripper("right");
    gripper::close();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}

class GripperClientMock : public GripperClient {
    GripperClientMock(attr1, attr2) : GripperClient(attr1, attr2) {}

    MOCK_METHOD1(waitForServer, bool(ros::Duration));

    /**
     * TODO
     */
    MOCK_METHOD1(sendGoal, );

    MOCK_METHOD0(waitForResult, bool());

};