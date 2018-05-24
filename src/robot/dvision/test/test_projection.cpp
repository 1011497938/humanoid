#include "dvision/projection.hpp"
#include <gtest/gtest.h>

using namespace dvision;
using namespace cv;
using namespace std;

// TODO(MWX):
// 1. calibrate / optimize initial camera location and orientation
// 2. project field to image and see result

TEST(Projection, main)
{
    ros::NodeHandle nh;

    Projection p;
    p.init(&nh);

    //    while (ros::ok()) {
    //    }
}

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "capture");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
