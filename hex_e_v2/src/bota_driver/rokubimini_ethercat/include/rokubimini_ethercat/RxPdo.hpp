#pragma once

#include <cstdint>

namespace rokubimini
{
namespace ethercat
{

// OnRobot HEX-E v2 / OptoForce EtherCAT RxPDO, exactly 4 bytes.
// Layout confirmed using SOEM slaveinfo -map.
struct RxPdo
{
  uint16_t filter_ = 0;  // 0x7000:01 FILTER
  uint16_t zero_ = 0;    // 0x7000:02 ZERO
} __attribute__((packed));

static_assert(sizeof(RxPdo) == 4, "RxPdo must be exactly 4 bytes");

}  // namespace ethercat
}  // namespace rokubimini
