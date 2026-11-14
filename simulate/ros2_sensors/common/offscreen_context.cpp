// Adapted from mujoco_ros2_control_plugins/src/camera_plugin.cpp (Apache-2.0).
// Copyright (c) 2026, United States Government, as represented by the
// Administrator of the National Aeronautics and Space Administration.
// Modified: use EGL independently of the simulator's GLFW window; RAII cleanup.
// Licensed under the Apache License, Version 2.0. See ../LICENSE.

#include "common/offscreen_context.hpp"

#include <EGL/eglext.h>
#include <algorithm>
#include <mutex>
#include <stdexcept>
#include <vector>

namespace ros2_sensors {
namespace {
std::mutex display_mutex;
std::vector<EGLDisplay> displays;

bool initialize_display(EGLDisplay display) {
  if (display == EGL_NO_DISPLAY) return false;
  if (std::find(displays.begin(), displays.end(), display) != displays.end()) return true;
  EGLint major, minor;
  if (!eglInitialize(display, &major, &minor)) return false;
  displays.push_back(display);
  return true;
}
}  // namespace

OffscreenContext::OffscreenContext() {
  try {
    {
      std::lock_guard<std::mutex> lock(display_mutex);
      display_ = eglGetPlatformDisplay(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr);
      if (!initialize_display(display_)) {
        display_ = eglGetDisplay(EGL_DEFAULT_DISPLAY);
        if (!initialize_display(display_)) {
          throw std::runtime_error("Camera: cannot initialize an EGL display");
        }
      }
    }
    const EGLint attributes[] = {
      EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
      EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_DEPTH_SIZE, 24, EGL_NONE
    };
    EGLConfig config;
    EGLint count = 0;
    if (!eglChooseConfig(display_, attributes, &config, 1, &count) || count == 0 ||
        !eglBindAPI(EGL_OPENGL_API)) {
      throw std::runtime_error("Camera: cannot select an EGL OpenGL configuration");
    }
    context_ = eglCreateContext(display_, config, EGL_NO_CONTEXT, nullptr);
    const EGLint pbuffer[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
    surface_ = eglCreatePbufferSurface(display_, config, pbuffer);
    if (context_ == EGL_NO_CONTEXT || surface_ == EGL_NO_SURFACE ||
        !eglMakeCurrent(display_, surface_, surface_, context_)) {
      throw std::runtime_error("Camera: cannot create/make current an EGL context");
    }
  } catch (...) {
    close();
    throw;
  }
}

OffscreenContext::~OffscreenContext() { close(); }

void OffscreenContext::close() {
  if (display_ == EGL_NO_DISPLAY) return;
  eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
  if (surface_ != EGL_NO_SURFACE) eglDestroySurface(display_, surface_);
  if (context_ != EGL_NO_CONTEXT) eglDestroyContext(display_, context_);
  // Display termination is deferred until SensorManager has joined all workers.
  eglReleaseThread();
  display_ = EGL_NO_DISPLAY;
  context_ = EGL_NO_CONTEXT;
  surface_ = EGL_NO_SURFACE;
}

void OffscreenContext::release_displays() {
  std::lock_guard<std::mutex> lock(display_mutex);
  for (EGLDisplay display : displays) eglTerminate(display);
  displays.clear();
  eglReleaseThread();
}

}  // namespace ros2_sensors
