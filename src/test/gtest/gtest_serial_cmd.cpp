#include "../../SerialCommand_Lx16a.h"
#include <gtest/gtest.h>



TEST(helper, checksum)
{
  std::vector<uint8_t> data;
  ASSERT_EQ(francor::servo::SerialCommand_Lx16a::compute_checksum(data), 0);
}

// TEST(Serial_cmd, serial_str)
// {
//   ASSERT_EQ()
// }