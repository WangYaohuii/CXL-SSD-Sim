from m5.SimObject import SimObject
from m5.params import *
from m5.objects.PciDevice import *


class CxlMemory(PciDevice):
    type = 'CxlMemory'
    cxx_header = "dev/storage/cxl_memory.hh"
    cxx_class = 'gem5::CxlMemory'
    latency = Param.Latency('50ns', "DRAM latency for cxl-SSD")
    cxl_mem_latency = Param.Latency('25ns', "cxl.mem protocol processing's latency for device")
    # evict_strategy = Param.String("TwoQ", "cxl cache evict strategy, Direct LRU FIFO TwoQ LFRU")

    VendorID = 0x8086
    DeviceID = 0x7890
    Command = 0x0
    Status = 0x280
    Revision = 0x0
    ClassCode = 0x01
    SubClassCode = 0x01
    ProgIF = 0x85
    InterruptLine = 0x1f
    InterruptPin = 0x01

    # Primary
    BAR0 = PciMemBar(size='4GiB')
    BAR1 = PciMemUpperBar()
