#include "dev/storage/cxl_memory.hh"
#include "base/trace.hh"
#include "debug/CxlMemory.hh"
#include "debug/CxlMemoryCoherency.hh"
#include <cstdio>

namespace gem5 {

/**
 * eventEngine global variable
 */
Engine engine;

std::ostream *pDebugLog = nullptr;
SimpleSSD::ConfigReader ssdConfig =
    initSimpleSSDEngine(&engine, pDebugLog, pDebugLog,
                        "./src/dev/storage/simplessd/config/sample.cfg");
/**
 * eventengine for simplessd
 */
Engine::Engine()
    : SimpleSSD::Simulator(), simTick(0), counter(0), eventHandled(0) {}

Engine::~Engine() {}

bool Engine::insertEvent(SimpleSSD::Event eid, uint64_t tick,
                         uint64_t *pOldTick) {
  bool found = false;
  bool flag = false;
  auto old = eventQueue.begin();
  auto insert = eventQueue.end();

  for (auto iter = eventQueue.begin(); iter != eventQueue.end(); iter++) {
    if (iter->first == eid) {
      found = true;
      old = iter;

      if (pOldTick) {
        *pOldTick = iter->second;
      }
    }

    if (iter->second > tick && !flag) {
      insert = iter;
      flag = true;
    }
  }

  if (found && pOldTick) {
    if (*pOldTick == tick) {
      // Rescheduling to same tick. Ignore.
      return false;
    }
  }

  // Iterator will not invalidated on insert
  // Do insert first
  eventQueue.insert(insert, {eid, tick});

  if (found) {
    eventQueue.erase(old);
  }

  return found;
}

bool Engine::removeEvent(SimpleSSD::Event eid) {
  bool found = false;

  for (auto iter = eventQueue.begin(); iter != eventQueue.end(); iter++) {
    if (iter->first == eid) {
      eventQueue.erase(iter);
      found = true;

      break;
    }
  }

  return found;
}

bool Engine::isEventExist(SimpleSSD::Event eid, uint64_t *pTick) {
  for (auto &iter : eventQueue) {
    if (iter.first == eid) {
      if (pTick) {
        *pTick = iter.second;
      }

      return true;
    }
  }

  return false;
}

uint64_t Engine::getCurrentTick() { return simTick; }

SimpleSSD::Event Engine::allocateEvent(SimpleSSD::EventFunction func) {
  auto iter = eventList.insert({++counter, func});

  if (!iter.second) {
    SimpleSSD::ssd_panic("Fail to allocate event");
  }

  return counter;
}

void Engine::scheduleEvent(SimpleSSD::Event eid, uint64_t tick) {
  auto iter = eventList.find(eid);

  if (iter != eventList.end()) {
    uint64_t tickCopy;

    tickCopy = simTick;

    if (tick < tickCopy) {
      SimpleSSD::ssd_warn("Tried to schedule %" PRIu64
                          " < simTick to event %" PRIu64
                          ". Set tick as simTick.",
                          tick, eid);

      tick = tickCopy;
    }

    uint64_t oldTick;

    if (insertEvent(eid, tick, &oldTick)) {
      SimpleSSD::ssd_warn("Event %" PRIu64 " rescheduled from %" PRIu64
                          " to %" PRIu64,
                          eid, oldTick, tick);
    }
  } else {
    SimpleSSD::ssd_panic("Event %" PRIu64 " does not exists", eid);
  }
}

void Engine::descheduleEvent(SimpleSSD::Event eid) {
  auto iter = eventList.find(eid);

  if (iter != eventList.end()) {
    removeEvent(eid);
  } else {
    SimpleSSD::ssd_panic("Event %" PRIu64 " does not exists", eid);
  }
}

bool Engine::isScheduled(SimpleSSD::Event eid, uint64_t *pTick) {
  bool ret = false;
  auto iter = eventList.find(eid);

  if (iter != eventList.end()) {
    ret = isEventExist(eid, pTick);
  } else {
    SimpleSSD::ssd_panic("Event %" PRIu64 " does not exists", eid);
  }

  return ret;
}

void Engine::deallocateEvent(SimpleSSD::Event eid) {
  auto iter = eventList.find(eid);

  if (iter != eventList.end()) {
    removeEvent(eid);
    eventList.erase(iter);
  } else {
    SimpleSSD::ssd_panic("Event %" PRIu64 " does not exists", eid);
  }
}

EvictStrategy *Worker(EvictStrategyMode mode, uint64_t capacity) {
  if (mode == EvictStrategyMode::Direct) {
    return new DirectEvictStrategy(capacity / CXL_SSD_PAGE_SIZE);
  } else if (mode == EvictStrategyMode::LRU) {
    return new LRUEvictStrategy(capacity);
  } else if (mode == EvictStrategyMode::FIFO) {
    return new FIFOEvictStrategy(capacity);
  } else if (mode == EvictStrategyMode::TwoQ) {
    return new TwoQEvictStrategy(capacity);
  } else if (mode == EvictStrategyMode::LFRU) {
    return new LFRUEviceStrategy(capacity);
  }
  assert(0);
}

/**
 * CxlMemory
 */

CxlMemory::CxlMemory(const Param &p)
    : PciDevice(p), latency_(p.latency), cxl_mem_latency_(p.cxl_mem_latency),
      pHIL(new SimpleSSD::HIL::HIL(ssdConfig)) {
  data_fd_ = open("./CxlSSD.img", O_RDWR | O_CREAT | O_TRUNC, 0666);
  if (data_fd_ == -1) {
    perror("Error opening file");
    assert(0);
  }
  if (ftruncate(data_fd_, CXL_SSD_CAPACITY) == -1) {
    perror("Error setting file size");
    assert(0);
  }

  mapped_cache_ = (char *)mmap(NULL, CXL_SSD_CAPACITY, PROT_READ | PROT_WRITE,
                               MAP_SHARED, data_fd_, 0);

  if (mapped_cache_ == MAP_FAILED) {
    perror("Error mmap CxlSSD.img");
    assert(0);
  }
  pages = new Page[pages_counts];
  evict_strategy = Worker(EvictStrategyMode::FIFO, cache_capacity);
}

CxlMemory::~CxlMemory() {
  delete pHIL;
  delete[] pages;
  delete evict_strategy;

  if (munmap(mapped_cache_, CXL_SSD_CAPACITY) == -1) {
    perror("Error unmapping file from memory");
  }
  close(data_fd_);
}

uint8_t *CxlMemory::toHostAddr(Addr addr) {
  Addr ssd_start = physicalAddrToSSDAddr(addr);
  return (uint8_t *)mapped_cache_ + ssd_start;
}

Tick CxlMemory::read(PacketPtr pkt) {
  DPRINTF(CxlMemory, "read address : (%lx, %lx)\n", pkt->getAddr(),
          pkt->getSize());
  DPRINTF(CxlMemoryCoherency, "read packet: MemCmd %s, address %lx\n",
          pkt->cmdString(), pkt->getAddr());
  access_counts_ += 1;

  // statistics cache hit
  // if (access_counts_ % CXL_SSD_CACHE_HIT_STAT == 0) {
  //   DPRINTF(CxlMemoryCacheHit,
  //           "CxlSSD cache hit statistics: hit counts: %ld, access counts: "
  //           "%ld, cache hit rate: %.4lf\n",
  //           cache_hit_counts_, access_counts_,
  //           (double)cache_hit_counts_ / access_counts_);
  // }

  access(pkt); // storage may be dram or SSD
  Tick cxl_latency = resolve_cxl_mem(pkt);

#ifdef CXL_MEMORY_ENABLE
  Tick storage_latency = latency_;
#else
  Tick storage_latency = ssdRead(pkt);
#endif

  DPRINTF(CxlMemory, "cxl_latency: %ld, read_latency: %ld\n", cxl_latency,
          storage_latency);

  return cxl_latency + storage_latency;
}

Tick CxlMemory::write(PacketPtr pkt) {
  DPRINTF(CxlMemory, "write address : (%lx, %lx)\n", pkt->getAddr(),
          pkt->getSize());
  DPRINTF(CxlMemoryCoherency,
          "write packet: MemCmd %s, address: %lx, req->isClean: %d, "
          "req->isInvalidate: %d\n",
          pkt->cmdString(), pkt->getAddr(), pkt->req->isCacheClean(),
          pkt->req->isCacheInvalidate());
  access_counts_ += 1;

  // // statistics cache hit
  // if (access_counts_ % CXL_SSD_CACHE_HIT_STAT == 0) {
  //   DPRINTF(CxlMemoryCacheHit,
  //           "CxlSSD cache hit statistics: hit counts: %ld, access counts: "
  //           "%ld, cache hit rate: %.4lf\n",
  //           cache_hit_counts_, access_counts_,
  //           (double)cache_hit_counts_ / access_counts_);
  // }

  access(pkt);
  Tick cxl_latency = resolve_cxl_mem(pkt);

#ifdef CXL_MEMORY_ENABLE
  Tick storage_latency = latency_;
#else
  Tick storage_latency = ssdWrite(pkt);
#endif

  DPRINTF(CxlMemory, "cxl_latency: %ld, write_latency: %ld\n", cxl_latency,
          storage_latency);
  return cxl_latency + storage_latency;
}

AddrRangeList CxlMemory::getAddrRanges() const {
  return PciDevice::getAddrRanges();
}

Tick CxlMemory::resolve_cxl_mem(PacketPtr pkt) {
  if (pkt->cmd == MemCmd::ReadReq) {
    assert(pkt->isRead());
    assert(pkt->needsResponse());
  } else if (pkt->cmd == MemCmd::WriteReq) {
    assert(pkt->isWrite());
    assert(pkt->needsResponse());
  }
  return cxl_mem_latency_;
}

// process data
void CxlMemory::access(PacketPtr pkt) {
  // get the bar address when linux wirte the corresponding registers
  range_ = AddrRange(BARs[0]->addr(), BARs[0]->addr() + BARs[0]->size());
  // DPRINTF(CxlMemory, "range_ addr_ : %lx, size: %lx\n", range_.start(),
  // range_.size());
  if (pkt->cacheResponding()) {
    DPRINTF(CxlMemory, "Cache responding to %#llx: not responding\n",
            pkt->getAddr());
    return;
  }

  if (pkt->cmd == MemCmd::CleanEvict || pkt->cmd == MemCmd::WritebackClean) {
    DPRINTF(CxlMemory, "CleanEvict  on 0x%x: not responding\n", pkt->getAddr());
    return;
  }

  assert(pkt->getAddrRange().isSubset(range_));

  uint8_t *host_addr = toHostAddr(pkt->getAddr());

  if (pkt->cmd == MemCmd::SwapReq) {
    if (pkt->isAtomicOp()) {
      if (mapped_cache_) {
        pkt->setData(host_addr);
        (*(pkt->getAtomicOp()))(host_addr);
      }
    } else {
      std::vector<uint8_t> overwrite_val(pkt->getSize());
      uint64_t condition_val64;
      uint32_t condition_val32;

      if (!mapped_cache_) {
        panic("Swap only works if there is real memory "
              "(i.e. null=False)");
      }

      bool overwrite_mem = true;
      // keep a copy of our possible write value, and copy what is at the
      // memory address into the packet
      pkt->writeData(&overwrite_val[0]);
      pkt->setData(host_addr);

      if (pkt->req->isCondSwap()) {
        if (pkt->getSize() == sizeof(uint64_t)) {
          condition_val64 = pkt->req->getExtraData();
          overwrite_mem =
              !std::memcmp(&condition_val64, host_addr, sizeof(uint64_t));
        } else if (pkt->getSize() == sizeof(uint32_t)) {
          condition_val32 = (uint32_t)pkt->req->getExtraData();
          overwrite_mem =
              !std::memcmp(&condition_val32, host_addr, sizeof(uint32_t));
        } else
          panic("Invalid size for conditional read/write\n");
      }

      if (overwrite_mem)
        std::memcpy(host_addr, &overwrite_val[0], pkt->getSize());

      assert(!pkt->req->isInstFetch());
    }
  } else if (pkt->isRead()) {
    assert(!pkt->isWrite());
    if (mapped_cache_) {
      pkt->setData(host_addr);
    }
  } else if (pkt->isWrite()) {
    if (mapped_cache_) {
      pkt->writeData(host_addr);
    }
    assert(!pkt->req->isInstFetch());
  } else {
    panic("Unexpected packet %s", pkt->print());
  }

  if (pkt->needsResponse()) {
    pkt->makeResponse();
  }
}

bool CxlMemory::ssdAddrCheck(PacketPtr &pkt) {
  Addr pktStart = pkt->getAddr();
  Addr pktend = pkt->getAddr() + pkt->getSize();
  return range_.start() <= pktStart &&
         pktend <= (range_.start() + range_.size());
}

uint64_t CxlMemory::GetPagesIndex(Addr ssd_start) const {
  uint64_t tag = ssd_start / logical_page_size_;
  uint64_t index = tag & (pages_counts - 1);
  return index;
}

Tick CxlMemory::ssdRead(PacketPtr pkt) {
  if (!ssdAddrCheck(pkt)) {
    assert(0);
  }
  Tick storage_latency = 0; // record the latency for simplessd

  uint64_t ssd_start = physicalAddrToSSDAddr(pkt->getAddr());
#ifndef CXL_SSD_NO_CACHE
  uint64_t logical_frame = ssd_start & (~(logical_page_size_ - 1));

  uint64_t index = evict_strategy->access(logical_frame);

  DPRINTF(CxlMemory, "ssd_read ssd_start: %lx, page_index: %lx\n", ssd_start,
          index);

  auto &page = pages[index];

  if (page.IsValid() && page.CacheHit(ssd_start)) {
    cache_hit_counts_ += 1;
    return latency_;
  }

  if (page.IsDirty()) {
    uint64_t dirty_addr_start = page.tag_;
    uint64_t dirty_page_size = logical_page_size_;
    uint64_t write_latency = 0;

    SimpleSSD::HIL::Request write_back_request(&write_latency);
    write_back_request.reqID = ++instruction_id;
    write_back_request.range.slpn = dirty_addr_start / logical_page_size_;
    write_back_request.range.nlp = dirty_page_size / logical_page_size_;
    write_back_request.offset = dirty_addr_start % logical_page_size_;
    write_back_request.length = dirty_page_size;
    write_back_request.function = [](uint64_t, void *) {};
    write_back_request.context = (void *)instruction_id;
    pHIL->write(write_back_request);

    storage_latency += 35250000; // TODO: need fix this patch, get real latency
    page.ClearDirty();
  }
  page.SetValid();
  page.SetTag(ssd_start);

#endif
  uint64_t read_latency = 0;
  SimpleSSD::HIL::Request request(&read_latency);
  request.reqID = ++instruction_id;
  request.range.slpn = ssd_start / logical_page_size_;
  request.range.nlp = pkt->getSize() / logical_page_size_;
  request.offset = ssd_start % logical_page_size_;
  request.length = pkt->getSize();
  request.function = [](uint64_t, void *) {};
  request.context = (void *)instruction_id;
  pHIL->read(request);

  storage_latency += read_latency * 10;

  DPRINTF(CxlMemory, "ssdread latency %ld, engine current tick %ld\n",
          storage_latency, engine.getCurrentTick());

  return storage_latency + latency_;
}

Tick CxlMemory::ssdWrite(PacketPtr pkt) {
  if (!ssdAddrCheck(pkt)) {
    assert(0);
  }
  Tick storage_latency = 0;

  uint64_t ssd_start = physicalAddrToSSDAddr(pkt->getAddr());
#ifndef CXL_SSD_NO_CACHE
  uint64_t logical_frame = ssd_start & (~(logical_page_size_ - 1));

  uint64_t index = evict_strategy->access(logical_frame);

  DPRINTF(CxlMemory, "ssd_write ssd_start: %lx, page_index: %lx\n", ssd_start,
          index);

  auto &page = pages[index];
  if (page.IsValid() && page.CacheHit(ssd_start)) {
    cache_hit_counts_ += 1;
    return latency_;
  }

  if (page.IsDirty()) {
    uint64_t dirty_addr_start = page.tag_;
    uint64_t dirty_page_size = logical_page_size_;
    uint64_t write_latency = 0;

    SimpleSSD::HIL::Request write_back_request(&write_latency);
    write_back_request.reqID = ++instruction_id;
    write_back_request.range.slpn = dirty_addr_start / logical_page_size_;
    write_back_request.range.nlp = dirty_page_size / logical_page_size_;
    write_back_request.offset = dirty_addr_start % logical_page_size_;
    write_back_request.length = dirty_page_size;
    write_back_request.function = [](uint64_t, void *) {};
    write_back_request.context = (void *)instruction_id;
    pHIL->write(write_back_request);

    storage_latency += 35250000; // TODO: need fix this patch, get real latency
    page.ClearDirty();
  }

  page.SetValid();
  page.SetTag(ssd_start);
  page.SetDirty();

  uint64_t read_latency = 0;
  SimpleSSD::HIL::Request request(&read_latency);
  request.reqID = ++instruction_id;
  request.range.slpn = ssd_start / logical_page_size_;
  request.range.nlp = pkt->getSize() / logical_page_size_;
  request.offset = ssd_start % logical_page_size_;
  request.length = pkt->getSize();
  request.function = [](uint64_t, void *) {};
  request.context = (void *)instruction_id;
  pHIL->read(request);

  storage_latency += read_latency * 10;

  DPRINTF(CxlMemory, "ssdwrite latency %ld, engine current tick %ld\n",
          storage_latency, engine.getCurrentTick());
#else  // define CXL_SSD_NO_CACHE

  uint64_t write_latency = 0;

  SimpleSSD::HIL::Request write_back_request(&write_latency);
  write_back_request.reqID = ++instruction_id;
  write_back_request.range.slpn = ssd_start / logical_page_size_;
  write_back_request.range.nlp = 1;
  write_back_request.offset = ssd_start % logical_page_size_;
  write_back_request.length = logical_page_size_;
  write_back_request.function = [](uint64_t, void *) {};
  write_back_request.context = (void *)instruction_id;
  pHIL->write(write_back_request);

  // storage_latency += 35250000; // TODO: need fix this patch, get real latency
  storage_latency += write_latency * 10;
#endif // CXL_SSD_NO_CACHE
  return storage_latency + latency_;
}

} // namespace gem5