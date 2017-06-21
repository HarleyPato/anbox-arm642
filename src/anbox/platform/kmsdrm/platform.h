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

#ifndef ANBOX_PLATFORM_KMSDRM_PLATFORM_H_
#define ANBOX_PLATFORM_KMSDRM_PLATFORM_H_

#include "anbox/platform/kmsdrm/window.h"
#include "anbox/platform/base_platform.h"
#include "anbox/graphics/emugl/DisplayManager.h"
#include "anbox/common/fd.h"
#include "anbox/runtime.h"

#include <unordered_map>
#include <thread>

#include <gbm.h>

#include <xf86drm.h>
#include <xf86drmMode.h>

struct libinput;
struct udev;

class Renderer;

namespace anbox {
namespace input {
class Device;
class Manager;
}  // namespace input
namespace platform {
namespace kmsdrm {
class Platform : public std::enable_shared_from_this<Platform>,
                 public BasePlatform {
 public:
  Platform(const std::shared_ptr<Runtime> &runtime,
           const std::shared_ptr<input::Manager> &input_manager);
  ~Platform();

  std::shared_ptr<wm::Window> create_window(
      const anbox::wm::Task::Id &task,
      const anbox::graphics::Rect &frame,
      const std::string &title) override;

  void set_clipboard_data(const ClipboardData &data) { }
  ClipboardData get_clipboard_data() { }

  void set_renderer(const std::shared_ptr<Renderer> &renderer) override;
  void set_window_manager(const std::shared_ptr<wm::Manager> &window_manager) override {}

  std::shared_ptr<audio::Sink> create_audio_sink() override { return nullptr; }
  std::shared_ptr<audio::Source> create_audio_source() override { return nullptr; }

  EGLNativeDisplayType native_display() const override;

  bool choose_config(EGLDisplay display, EGLConfig *config) override;

  EGLSurface create_offscreen_surface(EGLDisplay display, EGLConfig config, unsigned int width, unsigned int height) override;
  void destroy_offscreen_surface(EGLDisplay display, EGLSurface) override;
  void swap_buffers(EGLDisplay display, EGLSurface surface) override;

  bool supports_multi_window() const override;
  bool supports_cursor() const override;

 private:
  void setup();
  bool setup_kms();
  bool setup_tty();

  struct FrameBuffer {
    FrameBuffer() : id(0) {}
    std::uint32_t id;
  };
  FrameBuffer* frame_buffer_from_bo(gbm_bo *bo);

  bool wait_for_page_flip(int timeout);

  void read_input_events();

  static void on_gbm_bo_destroyed(gbm_bo *bo, void *data);
  static void on_page_flip(int fd,
                           unsigned int sequence,
                           unsigned int tv_sec,
                           unsigned int tv_usec,
                           void *user_data);

  std::shared_ptr<Runtime> runtime_;
  std::shared_ptr<input::Manager> input_manager_;
  std::shared_ptr<input::Device> pointer_;
  std::shared_ptr<input::Device> keyboard_;
  std::shared_ptr<Renderer> renderer_;
  Fd fd_;
  gbm_device *gbm_ = nullptr;
  drmModeConnector *connector_;
  std::uint32_t saved_conn_id_ = 0;
  drmModeEncoder *encoder_;
  drmModeModeInfo mode_;
  std::uint32_t crtc_;
  drmModeCrtcPtr original_crtc_ = nullptr;
  std::shared_ptr<Window> window_;
  std::uint32_t fb_id_;
  bool page_flip_pending_ = false;
  gbm_bo *current_bo_ = nullptr;
  gbm_bo *next_bo_ = nullptr;
  udev *udev_;
  libinput *libin_;
  std::shared_ptr<boost::asio::posix::stream_descriptor> libin_descriptor_;
  graphics::Rect pointer_pos_;
  int prev_tty_mode_ = 0;
  int prev_kd_mode_ = 0;
  std::unordered_map<EGLSurface,gbm_surface*> offscreen_surfaces_;
};
} // namespace kmsdrm
} // namespace platform
} // namespace anbox

#endif
