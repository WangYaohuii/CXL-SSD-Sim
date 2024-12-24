#ifndef SIMPLESSD_STANDALONE_CXL_SSD_H
#define SIMPLESSD_STANDALONE_CXL_SSD_H

#include <base/addr_range.hh>
#include <base/bitfield.hh>
#include <mem/packet.hh>

#include <cassert>
#include <dev/storage/simplessd/hil/hil.hh>
#include "dev/storage/simplessd/sim/simulator.hh"
#include <string>

namespace gem5 {

#define CXL_SSD_PAGE_LEFT_BITS (12)                     // 12 bit
#define CXL_SSD_CAPACITY (1 << 30)                      // 1G capacity
#define CXL_SSD_CACHE_CAPACITY (1 << 20)                // 1M capacity
#define CXL_SSD_PAGE_SIZE (1 << CXL_SSD_PAGE_LEFT_BITS) // 4K capacity


struct Page {
  uint64_t tag_;
  uint8_t flag_;

  static const uint64_t valid_offset = 0x0;
  static const uint64_t dirty_offset = 0x1;

  bool IsValid() { return gem5::bits(flag_, valid_offset); }
  bool IsDirty() { return gem5::bits(flag_, dirty_offset); }

  void SetValid() { gem5::set_bit(flag_, valid_offset); }
  void SetDirty() { gem5::set_bit(flag_, dirty_offset); }

  void ClearValid() { gem5::clear_bit(flag_, valid_offset); }
  void ClearDirty() { gem5::clear_bit(flag_, dirty_offset); }

  void SetTag(uint64_t tag) { tag_ = tag; }

  bool CacheHit(Addr addr) {
    return ((addr >> CXL_SSD_PAGE_LEFT_BITS) ==
            (tag_ >> CXL_SSD_PAGE_LEFT_BITS));
  }
};

typedef uint64_t Tick;

typedef size_t frame_id;

class EvictStrategy {
public:
  frame_id hello;
};

class CxlSSD {
private:
  AddrRange range_{0, 1 << 30}; // cpu allocate addr range for cxlssd device
  //  Memory mem_;
  Tick latency_;
  Tick cxl_mem_latency_;

  SimpleSSD::HIL::HIL *pHIL{nullptr};

  FILE *data_fp_{nullptr};
  uint64_t instruction_id{0};
  uint32_t logical_page_size_{CXL_SSD_PAGE_SIZE};
  // uint32_t logical_page_size_{1 << 10};

  uint64_t capacity{CXL_SSD_CAPACITY};
  uint64_t cache_capacity{CXL_SSD_CACHE_CAPACITY};

  uint64_t pages_counts{CXL_SSD_CACHE_CAPACITY / CXL_SSD_PAGE_SIZE};

  Addr physicalAddrToSSDAddr(Addr addr) { return addr - range_.start(); };

  Page *pages{nullptr};

public:
  virtual Tick read(PacketPtr pkt);
  virtual Tick write(PacketPtr pkt);

  bool AddrCheck(PacketPtr &pkt);

  Tick resolve_cxl_mem(PacketPtr ptk);

  CxlSSD(SimpleSSD::ConfigReader &config);
  ~CxlSSD();
};

} // namespace gem5
#endif // SIMPLESSD_STANDALONE_CXL_SSD_H
