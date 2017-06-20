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

#ifndef ANBOX_PLATFORM_KMSDRM_WINDOW_H_
#define ANBOX_PLATFORM_KMSDRM_WINDOW_H_

#include "anbox/wm/window.h"

#include <EGL/egl.h>

#include <gbm.h>

class Renderer;

namespace anbox {
namespace platform {
namespace kmsdrm {
class Window : public wm::Window,
               public std::enable_shared_from_this<Window> {
 public:
  typedef std::int32_t Id;
  static Id Invalid;

  class Observer {
   public:
    virtual ~Observer() {}
    virtual void on_swap_buffers_needed(EGLDisplay display, std::shared_ptr<Window> &window) = 0;
  };

  static std::shared_ptr<Window> create(const std::shared_ptr<Renderer> &renderer,
                                        const std::weak_ptr<Observer> &observer,
                                        gbm_surface *surface,
                                        const graphics::Rect &frame);

  ~Window();

  EGLNativeWindowType native_handle() const override;
  EGLSurface egl_surface() const override;

  void swap_buffers(EGLDisplay display) override;

 private:
  Window(const std::shared_ptr<Renderer> &renderer,
         const std::weak_ptr<Observer> &observer,
         gbm_surface *surface,
         const graphics::Rect &frame);

  std::weak_ptr<Observer> observer_;
  gbm_surface *surface_ = nullptr;
};
} // namespace kmsdrm
} // namespace platforms
} // namespace anbox

#endif
