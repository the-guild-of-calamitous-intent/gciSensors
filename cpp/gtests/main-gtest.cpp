#include <gtest/gtest.h>
#include <gciSensors.hpp>

using namespace gci::sensors;

TEST(gcisensors, dummy) {
  EXPECT_TRUE(true);
}

// // Demonstrate some basic assertions.
// TEST(gciSensors, quaternion_float) {
//   gcisensors::quaternion_t<float> q;
//   EXPECT_FLOAT_EQ(q.w, 1.0f);
//   EXPECT_FLOAT_EQ(q.x, 0.0f);
//   EXPECT_FLOAT_EQ(q.y, 0.0f);
//   EXPECT_FLOAT_EQ(q.z, 0.0f);

//   q = quaternionf_t::from_euler(0,0,0);
//   EXPECT_FLOAT_EQ(q.w, 1.0f);
//   EXPECT_FLOAT_EQ(q.x, 0.0f);
//   EXPECT_FLOAT_EQ(q.y, 0.0f);
//   EXPECT_FLOAT_EQ(q.z, 0.0f);
// }


// TEST(gciSensors, quaternion_double) {
//   gcisensors::quaternion_t<double> q;
//   EXPECT_DOUBLE_EQ(q.w, 1.0);
//   EXPECT_DOUBLE_EQ(q.x, 0.0);
//   EXPECT_DOUBLE_EQ(q.y, 0.0);
//   EXPECT_DOUBLE_EQ(q.z, 0.0);

//   q = quaterniond_t::from_euler(0,0,0);
//   EXPECT_DOUBLE_EQ(q.w, 1.0);
//   EXPECT_DOUBLE_EQ(q.x, 0.0);
//   EXPECT_DOUBLE_EQ(q.y, 0.0);
//   EXPECT_DOUBLE_EQ(q.z, 0.0);
// }