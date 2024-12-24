#include "base/addr_range.hh"
#include "base/bitfield.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/CxlMemory.hh"
#include "debug/CxlMemoryCacheHit.hh"
#include "debug/CxlMemoryCoherency.hh"
#include "dev/pci/device.hh"
#include "dev/storage/simplessd/hil/hil.hh"
#include "dev/storage/simplessd/util/simplessd.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/CxlMemory.hh"
#include <fcntl.h>
#include <set>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <unordered_map>

namespace gem5 {

// #define CXL_SSD_PAGE_LEFT_BITS (12)                       // 12 bit //那这里没进行粒度转换吗？
#define CXL_SSD_PAGE_LEFT_BITS (10)                       // 10 bit
#define CXL_SSD_CAPACITY (1LL << 32)                      // 4G capacity
#define CXL_SSD_CACHE_CAPACITY (1LL << 24)                // 32M capacity
#define CXL_SSD_PAGE_SIZE (1LL << CXL_SSD_PAGE_LEFT_BITS) // 4K capacity
#define CXL_SSD_CACHE_HIT_STAT (1 << 16)                  // 10 0000 counts
// #define CXL_MEMORY_ENABLE 1
// #define CXL_SSD_NO_CACHE 1

typedef uint64_t Tick;
typedef size_t frame_id;

/**
 * eventengine override the simulator of simplessd
 *
 */
class Engine : public SimpleSSD::Simulator {
private:
  uint64_t simTick;
  SimpleSSD::Event counter;
  std::unordered_map<SimpleSSD::Event, SimpleSSD::EventFunction> eventList;
  std::list<std::pair<SimpleSSD::Event, uint64_t>> eventQueue;

  uint64_t eventHandled;

  bool insertEvent(SimpleSSD::Event, uint64_t, uint64_t * = nullptr);
  bool removeEvent(SimpleSSD::Event);
  bool isEventExist(SimpleSSD::Event, uint64_t * = nullptr);

public:
  Engine();
  ~Engine();

  uint64_t getCurrentTick() override;

  SimpleSSD::Event allocateEvent(SimpleSSD::EventFunction) override;
  void scheduleEvent(SimpleSSD::Event, uint64_t) override;
  void descheduleEvent(SimpleSSD::Event) override;
  bool isScheduled(SimpleSSD::Event, uint64_t * = nullptr) override;
  void deallocateEvent(SimpleSSD::Event) override;
};

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

  void SetTag(uint64_t tag) { tag_ = tag & (~(CXL_SSD_PAGE_SIZE - 1)); }

  bool CacheHit(Addr addr) {
    return ((addr >> CXL_SSD_PAGE_LEFT_BITS) ==
            (tag_ >> CXL_SSD_PAGE_LEFT_BITS));
  }
};

extern Engine engine;
extern SimpleSSD::ConfigReader ssdConfig;

enum class EvictStrategyMode { Direct, LRU, FIFO, TwoQ, LFRU };

class LRUCache {
  struct Node {
    // frame means aligned to 4k
    uint64_t logical_frame;
    uint64_t frame_id;
  };

public:
  std::unordered_map<int, std::list<Node>::iterator> mem_;
  std::list<Node> table_;
  size_t capacity_;

  uint64_t allocated_id{0};
  uint64_t total_pages{0};

  // 4k multiples
  LRUCache(size_t capacity) : capacity_(capacity) {
    total_pages = capacity_ / CXL_SSD_PAGE_SIZE;
  }

  uint64_t access(uint64_t key) {
    if (auto it = mem_.find(key); it != mem_.end()) {
      auto frame_id = it->second->frame_id;
      table_.splice(table_.end(), table_, it->second);
      return frame_id;
    }
    if (table_.size() < total_pages) {
      uint64_t value = allocated_id++;
      table_.push_back({key, value});
      mem_.insert({key, --table_.end()});
      return value;
    }
    auto [delete_logical, frame_id] = table_.front();
    mem_.erase(delete_logical);
    table_.erase(table_.begin());
    table_.push_back({key, frame_id});
    mem_.insert({key, --table_.end()});
    return frame_id;
  }
};

class FIFOCache {
public:
  struct Node {
    // frame means aligned to 4k
    uint64_t logical_frame;
    uint64_t frame_id;
  };

public:
  std::unordered_map<uint64_t, std::list<Node>::iterator> mem_;
  std::list<Node> table_;
  size_t capacity_;

  uint64_t allocated_id{0};
  uint64_t total_pages{0};

  // 4k multiples
  FIFOCache(size_t capacity) : capacity_(capacity) {
    total_pages = capacity_ / CXL_SSD_PAGE_SIZE;
  }

  bool Exist(uint64_t key) { return mem_.find(key) != mem_.end(); }
  // FIFO strategy
  uint64_t access(uint64_t key) {
    if (auto it = mem_.find(key); it != mem_.end()) {
      return it->second->frame_id;
    }
    if (table_.size() < total_pages) {
      uint64_t value = allocated_id++;
      table_.push_back({key, value});
      mem_.insert({key, --table_.end()});
      return value;
    }
    auto [delete_logical, frame_id] = table_.front();
    mem_.erase(delete_logical);
    table_.erase(table_.begin());
    table_.push_back({key, frame_id});
    mem_.insert({key, --table_.end()});
    return frame_id;
  }
};

class LFRUCache {
private:
  struct LFRUNode {
    int cnt;  // frequency for lfu
    int time; // for lru
    uint64_t key;
    uint64_t value;
    LFRUNode(int _cnt, int _time, int _key, int _value)
        : cnt(_cnt), time(_time), key(_key), value(_value) {}

    bool operator<(const LFRUNode item) const {
      return cnt == item.cnt ? time < item.time : cnt < item.cnt;
    }
  };

  uint64_t capacity;
  uint64_t time_;
  std::unordered_map<uint64_t, std::set<LFRUNode>::iterator> map_;
  std::set<LFRUNode> mem_;

  uint64_t total_pages_;
  uint64_t allocated_id_{0};

public:
  LFRUCache(int _capacity) : capacity(_capacity), time_(0) {
    total_pages_ = capacity / CXL_SSD_PAGE_SIZE;
  }

  uint64_t access(uint64_t key) {
    if (auto it = map_.find(key); it != map_.end()) {
      auto iterator = it->second;
      auto node = mem_.extract(iterator);
      assert(bool(node));
      node.value().cnt += 1;
      node.value().time = ++time_;
      auto res_value = node.value().value;
      mem_.insert(std::move(node));
      return res_value;
    }
    if (mem_.size() < total_pages_) {
      uint64_t value = allocated_id_++;
      auto insert_pair = mem_.insert(LFRUNode(1, ++time_, key, value));
      assert(insert_pair.second);
      map_.insert({key, insert_pair.first});
      return value;
    }
    assert(mem_.size() == total_pages_);
    auto delete_node = mem_.extract(mem_.begin());
    assert(bool(delete_node));
    map_.erase(delete_node.value().key);
    auto value = delete_node.value().value;
    delete_node.value().cnt = 1;
    delete_node.value().time = ++time_;
    delete_node.value().key = key;
    auto insert_pair = mem_.insert(std::move(delete_node));
    assert(insert_pair.inserted);
    map_.insert({key, insert_pair.position});
    return value;
  }
};

class EvictStrategy {
public:
  virtual uint64_t access(uint64_t logical_frame) = 0;
  virtual ~EvictStrategy() = default;
};

class DirectEvictStrategy : public EvictStrategy {
public:
  uint64_t page_counts;
  DirectEvictStrategy(uint64_t page_counts) : page_counts(page_counts) {}
  uint64_t access(uint64_t logical_frame) override {
    uint64_t tag = logical_frame / CXL_SSD_PAGE_SIZE;
    uint64_t index = tag & (page_counts - 1);
    return index;
  }
};

class LRUEvictStrategy : public EvictStrategy {
public:
  LRUCache cache;
  LRUEvictStrategy(uint64_t capacity) : cache(capacity) {}
  uint64_t access(uint64_t logical_frame) override {
    return cache.access(logical_frame);
  }
};
class FIFOEvictStrategy : public EvictStrategy {
public:
  FIFOCache cache;
  FIFOEvictStrategy(uint64_t capacity) : cache(capacity) {}
  uint64_t access(uint64_t logical_frame) override {
    return cache.access(logical_frame);
  }
};

class LFRUEviceStrategy : public EvictStrategy {
public:
  LFRUCache cache;
  LFRUEviceStrategy(uint64_t capacity) : cache(capacity) {}
  uint64_t access(uint64_t logical_frame) override {
    return cache.access(logical_frame);
  }
};

class TwoQEvictStrategy : public EvictStrategy {
public:
  struct Node {
    // frame means aligned to 4k
    uint64_t logical_frame;
    uint64_t frame_id;
  };

public:
  std::unordered_map<uint64_t, std::list<Node>::iterator> fifo_map_;
  std::list<Node> fifo_list_;

  std::unordered_map<uint64_t, std::list<Node>::iterator> lru_map_;
  std::list<Node> lru_list_;

  size_t capacity_;
  size_t queue_pages_;

  uint64_t allocated_id_{0};
  uint64_t total_pages_{0};

  std::list<uint64_t> free_frame_id_;

  explicit TwoQEvictStrategy(size_t capacity) : capacity_(capacity) {
    total_pages_ = capacity_ / CXL_SSD_PAGE_SIZE;
    queue_pages_ = total_pages_ / 2;
  }

  uint64_t fifoQueueAccess(uint64_t key) {
    if (auto it = fifo_map_.find(key); it != fifo_map_.end()) {
      return it->second->frame_id;
    }

    if (fifo_list_.size() == queue_pages_) {
      auto [delete_logical, frame_id] = fifo_list_.front();
      fifo_map_.erase(delete_logical);
      fifo_list_.erase(fifo_list_.begin());
      fifo_list_.push_back({key, frame_id});
      fifo_map_.insert({key, --fifo_list_.end()});
      return frame_id;
    }
    assert(fifo_list_.size() < queue_pages_);
    uint64_t value = -1;
    if (!free_frame_id_.empty()) {
      value = free_frame_id_.back();
      free_frame_id_.pop_back();
    } else {
      value = allocated_id_++;
    }
    fifo_list_.push_back({key, value});
    fifo_map_.insert({key, --fifo_list_.end()});
    return value;
  }

  uint64_t lruQueueAccess(uint64_t key) {
    if (auto it = lru_map_.find(key); it != lru_map_.end()) {
      auto frame_id = it->second->frame_id;
      lru_list_.splice(lru_list_.end(), lru_list_, it->second);
      return frame_id;
    }
    if (lru_list_.size() == queue_pages_) {
      auto [delete_logical, frame_id] = lru_list_.front();
      lru_map_.erase(delete_logical);
      lru_list_.erase(lru_list_.begin());
      if (!free_frame_id_.empty()) {
        auto last_frame_id = free_frame_id_.back();
        free_frame_id_.pop_back();
        free_frame_id_.push_back(frame_id);
        lru_list_.push_back({key, last_frame_id});
        lru_map_.insert({key, --lru_list_.end()});
        return last_frame_id;
      } else {
        lru_list_.push_back({key, frame_id});
        lru_map_.insert({key, --lru_list_.end()});
        return frame_id;
      }
    }
    assert(lru_list_.size() < queue_pages_);
    uint64_t value = -1;
    if (!free_frame_id_.empty()) {
      value = free_frame_id_.back();
      free_frame_id_.pop_back();
    } else {
      value = allocated_id_++;
    }
    lru_list_.push_back({key, value});
    lru_map_.insert({key, --lru_list_.end()});
    return value;
  }

  bool fifoExist(uint64_t key) {
    return fifo_map_.find(key) != fifo_map_.end();
  }

  bool lruExist(uint64_t key) { return lru_map_.find(key) != lru_map_.end(); }

  uint64_t access(uint64_t logical_frame) override {
    if (lruExist(logical_frame)) {
      return lruQueueAccess(logical_frame);
    }
    if (auto it = fifo_map_.find(logical_frame); it != fifo_map_.end()) {
      auto [delete_logical, frame_id] = *it->second;
      assert(logical_frame == delete_logical);
      fifo_map_.erase(delete_logical);
      fifo_list_.erase(it->second);
      free_frame_id_.push_back(frame_id);
      return lruQueueAccess(logical_frame);
    }
    return fifoQueueAccess(logical_frame);
  }
};

EvictStrategy *Worker(EvictStrategyMode mode, uint64_t capacity);

class CxlMemory : public PciDevice {
private:
  AddrRange range_{0, 1 << 30}; // cpu allocate addr range for cxlssd device
  //  Memory mem_;
  Tick latency_;
  Tick cxl_mem_latency_;

  SimpleSSD::HIL::HIL *pHIL{nullptr};

  // CXL_SSD_CAPACITY mapped region
  int data_fd_{-1};
  uint64_t cache_hit_counts_{0};
  uint64_t access_counts_{0};
  char *mapped_cache_{nullptr};

  uint64_t instruction_id{0};
  uint32_t logical_page_size_{CXL_SSD_PAGE_SIZE};
  // uint32_t logical_page_size_{1 << 10};

  uint64_t capacity{CXL_SSD_CAPACITY};
  uint64_t cache_capacity{CXL_SSD_CACHE_CAPACITY};

  uint64_t pages_counts{CXL_SSD_CACHE_CAPACITY / CXL_SSD_PAGE_SIZE};

  Addr physicalAddrToSSDAddr(Addr addr) { return addr - range_.start(); };

  Page *pages{nullptr};
  EvictStrategy *evict_strategy{nullptr};

public:
  virtual Tick read(PacketPtr pkt) override;
  virtual Tick write(PacketPtr pkt) override;

  void access(PacketPtr pkt);
  virtual AddrRangeList getAddrRanges() const override;

  Tick resolve_cxl_mem(PacketPtr ptk);

  // for cxl-ssd
  uint8_t *toHostAddr(Addr addr);
  bool ssdAddrCheck(PacketPtr &ptk);
  Tick ssdRead(PacketPtr pkt);
  Tick ssdWrite(PacketPtr pkt);
  uint64_t GetPagesIndex(Addr ssd_start) const;

  using Param = CxlMemoryParams;
  CxlMemory(const Param &p);
  ~CxlMemory();
};

} // namespace gem5