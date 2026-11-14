// Adapted from mujoco_ros2_control_plugins/src/camera_plugin.cpp (Apache-2.0).
// Copyright (c) 2026, United States Government, as represented by the
// Administrator of the National Aeronautics and Space Administration.
// Modified for a standalone, thread-owned EGL context. See ../LICENSE.
#pragma once

#include <EGL/egl.h>

namespace ros2_sensors {

class OffscreenContext {
 public:
  OffscreenContext();
  ~OffscreenContext();
  OffscreenContext(const OffscreenContext&) = delete;
  OffscreenContext& operator=(const OffscreenContext&) = delete;
  // Call at manager shutdown, after every render worker has been joined.
  // The shared display stays initialized across model/context reloads.
  static void release_displays();

 private:
  void close();
  EGLDisplay display_ = EGL_NO_DISPLAY;
  EGLContext context_ = EGL_NO_CONTEXT;
  EGLSurface surface_ = EGL_NO_SURFACE;
};

}  // namespace ros2_sensors
