/*
 * Copyright (C) 2016 Simon Fels <morphis@gravedo.de>
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 3, as published
 * by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranties of
 * MERCHANTABILITY, SATISFACTORY QUALITY, or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "anbox/platform/kmsdrm/window.h"
#include "anbox/graphics/emugl/DispatchTables.h"
#include "anbox/graphics/emugl/Renderer.h"
#include "anbox/logger.h"

namespace anbox {
namespace platform {
namespace kmsdrm {
std::shared_ptr<Window> Window::create(const std::shared_ptr<Renderer> &renderer,
                                       gbm_surface *surface,
                                       const graphics::Rect &frame) {
  return std::shared_ptr<Window>(new Window(renderer, surface, frame));
}

Window::Window(const std::shared_ptr<Renderer> &renderer,
               gbm_surface *surface,
               const graphics::Rect &frame)
    : wm::Window(renderer, 0, frame, ""),
      surface_{surface} {

  egl_surface_ = s_egl.eglCreateWindowSurface(renderer->getEglDisplay(),
                                              renderer->getEglConfig(),
                                              reinterpret_cast<EGLNativeWindowType>(surface_),
                                              nullptr);
}

Window::~Window() {
  if (surface_)
    gbm_surface_destroy(surface_);
}

EGLNativeWindowType Window::native_handle() const {
  return reinterpret_cast<EGLNativeWindowType>(surface_);
}
} // namespace kmsdrm
} // namespace platforms
} // namespace anbox
