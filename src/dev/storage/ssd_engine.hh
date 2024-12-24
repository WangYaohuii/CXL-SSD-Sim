#ifndef __GEM5__DEV_STORAGE_SSD_ENGINE_HH__
#define __GEM5__DEV_STORAGE_SSD_ENGINE_HH__

#include "dev/storage/simplessd/sim/simulator.hh"
#include <list>

namespace gem5 {

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

extern Engine engine;

} // namespace gem5

#endif // __GEM5__DEV_STORAGE_SSD_ENGINE_HH__