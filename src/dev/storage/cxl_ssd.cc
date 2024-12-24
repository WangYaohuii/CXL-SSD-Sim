#include <dev/storage/cxl_ssd.hh>

#include <cassert>
#include <cstdio>

namespace gem5 {

CxlSSD::CxlSSD(SimpleSSD::ConfigReader &config)
    : latency_(50), cxl_mem_latency_(10),
      pHIL(new SimpleSSD::HIL::HIL(config)) {
  data_fp_ = fopen("./CxlSSD.img", "w+");
  assert(data_fp_);
  pages = new Page[pages_counts];
}

CxlSSD::~CxlSSD() {
  fflush(data_fp_);
  fclose(data_fp_);
  delete pHIL;
  delete[] pages;
}

bool CxlSSD::AddrCheck(PacketPtr &pkt) {
  Addr pktStart = pkt->start;
  Addr pktend = pkt->start + pkt->length;
  return range_.start() <= pktStart &&
         pktend <= (range_.start() + range_.size());
}
Tick CxlSSD::read(PacketPtr pkt) {
  if (!AddrCheck(pkt)) {
    assert(0);
  }
  Tick storage_latency = 0; // record the latency for simplessd

  uint64_t ssd_start = physicalAddrToSSDAddr(pkt->start);
  uint64_t page_entry_id = ssd_start / logical_page_size_;
  if (pkt->memcmd == packetMode::read) {
    auto &page = pages[page_entry_id];

    if (page.IsValid() && page.CacheHit(ssd_start)) {
      printf("cxl hit latency %ld \n", cxl_mem_latency_);
      return cxl_mem_latency_;
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

      storage_latency += write_latency;
      page.ClearDirty();
    }

    page.SetValid();
    page.SetTag(ssd_start);

    uint64_t read_latency = 0;
    SimpleSSD::HIL::Request request(&read_latency);
    request.reqID = ++instruction_id;
    request.range.slpn = ssd_start / logical_page_size_;
    request.range.nlp = pkt->length / logical_page_size_;
    request.offset = ssd_start % logical_page_size_;
    request.length = pkt->length;
    request.function = [](uint64_t, void *) {};
    request.context = (void *)instruction_id;
    pHIL->read(request);

    storage_latency += read_latency;
  } else {
    assert(0);
  }

  printf("cxl hit miss %ld \n", storage_latency + cxl_mem_latency_);
  return storage_latency + cxl_mem_latency_;
}

Tick CxlSSD::write(PacketPtr pkt) {
  if (!AddrCheck(pkt)) {
    assert(0);
  }
  Tick storage_latency = 0;
  if (pkt->memcmd == packetMode::write) {
    uint64_t ssd_start = physicalAddrToSSDAddr(pkt->start);
    uint64_t page_entry_id = ssd_start / logical_page_size_;

    auto &page = pages[page_entry_id];
    if (page.IsValid() && page.CacheHit(ssd_start)) {
      printf("cxl hit latency %ld \n", cxl_mem_latency_);
      return cxl_mem_latency_;
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

      storage_latency += write_latency;
      page.ClearDirty();
    }

    page.SetValid();
    page.SetTag(ssd_start);
    page.SetDirty();

    uint64_t read_latency = 0;
    SimpleSSD::HIL::Request request(&read_latency);
    request.reqID = ++instruction_id;
    Addr simplessd_start_addr = physicalAddrToSSDAddr(pkt->start);
    request.range.slpn = simplessd_start_addr / logical_page_size_;
    request.range.nlp = pkt->length / logical_page_size_;
    request.offset = simplessd_start_addr % logical_page_size_;
    request.length = pkt->length;
    request.function = [](uint64_t, void *) {};
    request.context = (void *)instruction_id;
    pHIL->write(request);

    storage_latency += read_latency;
  } else {
    assert(0);
  }
  printf("cxl hit miss %ld \n", storage_latency + cxl_mem_latency_);
  return storage_latency + cxl_mem_latency_;
}

} // namespace gem5
