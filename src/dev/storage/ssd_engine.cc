#include "dev/storage/ssd_engine.hh"

namespace gem5 {

Engine engine;

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
    panic("Fail to allocate event");
  }

  return counter;
}

void Engine::scheduleEvent(SimpleSSD::Event eid, uint64_t tick) {
  auto iter = eventList.find(eid);

  if (iter != eventList.end()) {
    uint64_t tickCopy;

    tickCopy = simTick;

    if (tick < tickCopy) {
      warn("Tried to schedule %" PRIu64 " < simTick to event %" PRIu64
           ". Set tick as simTick.",
           tick, eid);

      tick = tickCopy;
    }

    uint64_t oldTick;

    if (insertEvent(eid, tick, &oldTick)) {
      warn("Event %" PRIu64 " rescheduled from %" PRIu64 " to %" PRIu64, eid,
           oldTick, tick);
    }
  } else {
    panic("Event %" PRIu64 " does not exists", eid);
  }
}

void Engine::descheduleEvent(SimpleSSD::Event eid) {
  auto iter = eventList.find(eid);

  if (iter != eventList.end()) {
    removeEvent(eid);
  } else {
    panic("Event %" PRIu64 " does not exists", eid);
  }
}

bool Engine::isScheduled(SimpleSSD::Event eid, uint64_t *pTick) {
  bool ret = false;
  auto iter = eventList.find(eid);

  if (iter != eventList.end()) {
    ret = isEventExist(eid, pTick);
  } else {
    panic("Event %" PRIu64 " does not exists", eid);
  }

  return ret;
}

void Engine::deallocateEvent(SimpleSSD::Event eid) {
  auto iter = eventList.find(eid);

  if (iter != eventList.end()) {
    removeEvent(eid);
    eventList.erase(iter);
  } else {
    panic("Event %" PRIu64 " does not exists", eid);
  }
}

} // namespace gem5