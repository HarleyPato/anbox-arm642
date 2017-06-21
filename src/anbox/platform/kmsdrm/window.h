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
  static std::shared_ptr<Window> create(const std::shared_ptr<Renderer> &renderer,
                                        gbm_surface *surface,
                                        const graphics::Rect &frame);

  ~Window();

  EGLNativeWindowType native_handle() const override;

 private:
  Window(const std::shared_ptr<Renderer> &renderer,
         gbm_surface *surface,
         const graphics::Rect &frame);

  gbm_surface *surface_ = nullptr;
};
} // namespace kmsdrm
} // namespace platforms
} // namespace anbox

#endif
