#pragma once

#include <mujoco/mujoco.h>

namespace ros2_sensors {

// The simulator owns model/data. All sample/reset calls run under its mutex.
// Implementations own their worker threads and must stop before model unload.
class Sensor {
 public:
  virtual ~Sensor() = default;
  virtual void sample(const mjModel* model, const mjData* data) = 0;
  virtual void reset() = 0;
  virtual void stop() = 0;
};

}  // namespace ros2_sensors
