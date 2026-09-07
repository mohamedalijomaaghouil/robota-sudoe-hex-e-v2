#pragma once

#include <cstdint>

namespace rokubimini
{
namespace ethercat
{

// OnRobot HEX-E v2 / OptoForce EtherCAT TxPDO, exactly 56 bytes.
// Layout confirmed using SOEM slaveinfo -map.
struct TxPdoA
{
  // 0x6000:01-06, offsets 0x00-0x17
  float forceX_;
  float forceY_;
  float forceZ_;
  float torqueX_;
  float torqueY_;
  float torqueZ_;

  // 0x6010:01-06, offsets 0x1c-0x33
  int32_t forceX_raw_;
  int32_t forceY_raw_;
  int32_t forceZ_raw_;
  int32_t torqueX_raw_;
  int32_t torqueY_raw_;
  int32_t torqueZ_raw_;

  // Offsets 0x34-0x3b
  int16_t divider_;
  uint16_t sampleCounter_;
  uint32_t status_;
} __attribute__((packed));

static_assert(sizeof(TxPdoA) == 56, "TxPdoA must be exactly 56 bytes");

}  // namespace ethercat
}  // namespace rokubimini
