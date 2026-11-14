#pragma once

#include <memory>
#include <string>

#include <mujoco/mujoco.h>

namespace ros2_sensors {

class SensorManager {
 public:
  explicit SensorManager(const std::string& config_file, int unitree_domain_id);
  ~SensorManager();
  SensorManager(const SensorManager&) = delete;
  SensorManager& operator=(const SensorManager&) = delete;

  void load_model(const mjModel* model, const mjData* data);
  void update(const mjModel* model, const mjData* data);
  void reset();
  void unload_model();

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ros2_sensors
