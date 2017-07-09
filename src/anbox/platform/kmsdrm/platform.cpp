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

#include "anbox/platform/kmsdrm/platform.h"
#include "anbox/platform/kmsdrm/window.h"
#include "anbox/graphics/emugl/Renderer.h"
#include "anbox/input/device.h"
#include "anbox/input/manager.h"
#include "anbox/logger.h"

#include "external/android-emugl/host/include/OpenGLESDispatch/EGLDispatch.h"

#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <math.h>

#include <sys/ioctl.h>

#include <linux/vt.h>
#include <linux/kd.h>

#include <libudev.h>
#include <libinput.h>

#include <boost/asio.hpp>

namespace ba = boost::asio;

namespace {
constexpr const char *dri_card_path{"/dev/dri/card0"};

static int libin_open(const char *path, int flags, void *data) {
  (void) data;
  const auto fd = open(path, flags);
  return fd < 0 ? -errno : fd;
}

static void libin_close(int fd, void *data) {
  (void) data;
  close(fd);
}

const libinput_interface libin_iface {
  libin_open,
  libin_close,
};
}

namespace anbox {
namespace platform {
namespace kmsdrm {
Platform::Platform(const std::shared_ptr<Runtime> &runtime,
                   const std::shared_ptr<input::Manager> &input_manager) :
  runtime_(runtime),
  input_manager_(input_manager) {
  setup();
}

Platform::~Platform() {
  if (libin_)
    libinput_unref(libin_);

  if (udev_)
    udev_unref(udev_);

  if (original_crtc_) {
    if (drmModeSetCrtc(fd_,
                       original_crtc_->crtc_id,
                       original_crtc_->buffer_id,
                       original_crtc_->x,
                       original_crtc_->y,
                       &connector_->connector_id, 1,
                       &original_crtc_->mode))
      WARNING("Cannot restore original CRTC");

    drmModeFreeCrtc(original_crtc_);
  }

  if (gbm_)
    gbm_device_destroy(gbm_);

  Fd tty_fd{open("/dev/tty0",  O_RDONLY | O_NDELAY)};
  ioctl(tty_fd, KDSETMODE, prev_kd_mode_);
  ioctl(tty_fd, KDSKBMODE, prev_tty_mode_);
}

void Platform::setup() {
  fd_ = Fd{::open(dri_card_path, O_RDWR)};
  if (fd_ < 0)
    throw std::runtime_error("Cannot connect to DRI card device");

  if (!setup_tty())
    throw std::runtime_error("Failed to setup TTY");

  if (!setup_kms())
    throw std::runtime_error("Failed to setup KMS");

  gbm_ = gbm_create_device(fd_);
  if (!gbm_)
    throw std::runtime_error("Failed to create GBM device");

  DEBUG("Using GBM backend: %s", gbm_device_get_backend_name(gbm_));

  pointer_ = input_manager_->create_device();
  pointer_->set_name("anbox-pointer");
  pointer_->set_driver_version(1);
  pointer_->set_input_id({BUS_VIRTUAL, 2, 2, 2});
  pointer_->set_physical_location("none");
  pointer_->set_key_bit(BTN_MOUSE);
  // NOTE: We don't use REL_X/REL_Y in reality but have to specify them here
  // to allow InputFlinger to detect we're a cursor device.
  pointer_->set_rel_bit(REL_X);
  pointer_->set_rel_bit(REL_Y);
  pointer_->set_rel_bit(REL_HWHEEL);
  pointer_->set_rel_bit(REL_WHEEL);
  pointer_->set_prop_bit(INPUT_PROP_POINTER);

  keyboard_ = input_manager_->create_device();
  keyboard_->set_name("anbox-keyboard");
  keyboard_->set_driver_version(1);
  keyboard_->set_input_id({BUS_VIRTUAL, 3, 3, 3});
  keyboard_->set_physical_location("none");
  keyboard_->set_key_bit(BTN_MISC);
  keyboard_->set_key_bit(KEY_OK);

  udev_ = udev_new();
  if (!udev_)
    throw std::runtime_error("Failed to create udev listener");

  libin_ = libinput_udev_create_context(&libin_iface, nullptr, udev_);
  if (!libin_)
    throw std::runtime_error("Failed to create libinput context");

  if (libinput_udev_assign_seat(libin_, "seat0") != 0)
    throw std::runtime_error("Failed to assign seat to libinput context");

  pointer_pos_ = graphics::Rect::Empty;

  libin_descriptor_ = std::make_shared<ba::posix::stream_descriptor>(runtime_->service());
  libin_descriptor_->assign(libinput_get_fd(libin_));
  read_input_events();
}

bool Platform::setup_tty() {
  Fd tty_fd{open("/dev/tty0",  O_RDONLY | O_NDELAY)};
  if (tty_fd < 0) {
    ERROR("Failed to open TTY");
    return false;
  }

  if (ioctl(tty_fd, KDGETMODE, &prev_kd_mode_) < 0) {
    DEBUG("Failed to get current VT mode");
    return false;
  }

  if (ioctl(tty_fd, KDGKBMODE, &prev_tty_mode_) < 0) {
    DEBUG("Failed to get current TTY mode");
    return false;
  }

  if (ioctl(tty_fd, KDSKBMODE, K_OFF) < 0) {
    DEBUG("Failed to mute keyboard");
    return false;
  }

  if (ioctl(tty_fd, KDSETMODE, KD_GRAPHICS) < 0) {
    DEBUG("Failed to switch VT mode");
    return false;
  }

  return true;
}

bool Platform::setup_kms() {
  if (fd_ < 0)
    return false;

  auto resources = drmModeGetResources(fd_);
  if (!resources) {
    ERROR("Failed to retrieve DRM resources");
    return false;
  }

  drmModeConnector *connector = nullptr;
  int i = 0;
  for (i = 0; i < resources->count_connectors; i++) {
    connector = drmModeGetConnector(fd_, resources->connectors[i]);
    if (connector == NULL)
      continue;

    if (connector->connection == DRM_MODE_CONNECTED &&
        connector->count_modes > 0) {
      saved_conn_id_ = connector->connector_id;
      break;
    }

    drmModeFreeConnector(connector);
  }

  if (i == resources->count_connectors) {
    ERROR("No currently active connector found");
    return false;
  }

  drmModeEncoder *encoder = nullptr;
  for (i = 0; i < resources->count_encoders; i++) {
    encoder = drmModeGetEncoder(fd_, resources->encoders[i]);

    if (encoder == NULL)
      continue;

    if (encoder->encoder_id == connector->encoder_id)
      break;

    drmModeFreeEncoder(encoder);
  }

  std::uint32_t crtc = 0;
  for (int j = 0; j < resources->count_crtcs; j++) {
    if ((encoder->possible_crtcs & (1 << j))) {
      crtc = resources->crtcs[j];
      break;
    }
  }

  if (crtc == 0) {
    WARNING("No suitable CRTC available");
    return false;
  }

  connector_ = connector;
  encoder_ = encoder;
  mode_ = connector->modes[0];
  crtc_ = crtc;

  original_crtc_ = drmModeGetCrtc(fd_, encoder_->crtc_id);

  drmModeFreeResources(resources);

  return true;
}

void Platform::read_input_events() {
  libin_descriptor_->async_read_some(ba::null_buffers(),
                                     [this](const boost::system::error_code& err,
                                            std::size_t bytes_read) {
    (void) bytes_read;

    if (err) {
      WARNING("Failed to read from libinput file descriptor: %s", err.message());
      return;
    }

    if (libinput_dispatch(libin_) != 0) {
      WARNING("Failed to dispatch pending libinput events");
      return;
    }

    std::vector<input::Event> mouse_events;
    std::vector<input::Event> keyboard_events;

    libinput_event *ev;
    while ((ev = libinput_get_event(libin_)) != nullptr) {
      const auto type = libinput_event_get_type(ev);
      switch (type) {
      case LIBINPUT_EVENT_POINTER_BUTTON: {
        auto e = libinput_event_get_pointer_event(ev);
        const uint32_t btn = libinput_event_pointer_get_button(e);
        const bool pressed = libinput_event_pointer_get_button_state(e) == LIBINPUT_BUTTON_STATE_PRESSED;
        mouse_events.push_back({EV_KEY, static_cast<uint16_t>(btn), pressed ? 1 : 0});
        mouse_events.push_back({EV_SYN, SYN_REPORT, 0});
        break;
      }
      case LIBINPUT_EVENT_POINTER_MOTION_ABSOLUTE: {
        auto e = libinput_event_get_pointer_event(ev);

        graphics::Rect screen_frame = window_->frame();

        const double x = libinput_event_pointer_get_absolute_x_transformed(e, screen_frame.width());
        const double y = libinput_event_pointer_get_absolute_y_transformed(e, screen_frame.height());

        const auto dx = pointer_pos_.left() + x;
        const auto dy = pointer_pos_.top() + y;

        pointer_pos_ = graphics::Rect(x, y, 0, 0);

        // NOTE: Sending relative move events doesn't really work and we have
        // changes in libinputflinger to take ABS_X/ABS_Y instead for absolute
        // position events.
        mouse_events.push_back({EV_ABS, ABS_X, static_cast<int32_t>(x)});
        mouse_events.push_back({EV_ABS, ABS_Y, static_cast<int32_t>(y)});

        // We're sending relative position updates here too but they will be only
        // used by the Android side EventHub/InputReader to determine if the cursor
        // was moved. They are not used to find out the exact position.
        mouse_events.push_back({EV_REL, REL_X, static_cast<int32_t>(dx)});
        mouse_events.push_back({EV_REL, REL_Y, static_cast<int32_t>(dy)});
        mouse_events.push_back({EV_SYN, SYN_REPORT, 0});
        break;
      }
      case LIBINPUT_EVENT_POINTER_MOTION: {
        DEBUG("pointer motion");
        auto e = libinput_event_get_pointer_event(ev);
        const double dx = libinput_event_pointer_get_dx(e);
        const double dy = libinput_event_pointer_get_dy(e);

        if (!window_)
          break;

        graphics::Rect screen_frame = window_->frame();

        auto x = pointer_pos_.left() + dx;
        if (x < screen_frame.width())
          x = screen_frame.width();
        if (x > screen_frame.width())
          x = screen_frame.width();

        auto y = pointer_pos_.top() + dy;
        if (x < screen_frame.height())
          x = screen_frame.height();
        if (x > screen_frame.height())
          x = screen_frame.height();

        pointer_pos_ = graphics::Rect(x, y, 0, 0);

        DEBUG("Pointer motion: abs [%s] rel [%d,%d]", pointer_pos_, dx, dy);

        // NOTE: Sending relative move events doesn't really work and we have
        // changes in libinputflinger to take ABS_X/ABS_Y instead for absolute
        // position events.
        mouse_events.push_back({EV_ABS, ABS_X, static_cast<int32_t>(x)});
        mouse_events.push_back({EV_ABS, ABS_Y, static_cast<int32_t>(y)});

        // We're sending relative position updates here too but they will be only
        // used by the Android side EventHub/InputReader to determine if the cursor
        // was moved. They are not used to find out the exact position.
        mouse_events.push_back({EV_REL, REL_X, static_cast<int32_t>(dx)});
        mouse_events.push_back({EV_REL, REL_Y, static_cast<int32_t>(dy)});
        mouse_events.push_back({EV_SYN, SYN_REPORT, 0});

        break;
      }
      case LIBINPUT_EVENT_POINTER_AXIS:
        break;
      case LIBINPUT_EVENT_KEYBOARD_KEY:
        break;
      default:
        break;
      }
    }

    if (mouse_events.size() > 0)
      pointer_->send_events(mouse_events);

    if (keyboard_events.size() > 0)
      keyboard_->send_events(keyboard_events);

    read_input_events();
  });
}

void Platform::set_renderer(const std::shared_ptr<Renderer> &renderer) {
  renderer_ = renderer;
}

std::shared_ptr<wm::Window> Platform::create_window(
    const anbox::wm::Task::Id &task,
    const anbox::graphics::Rect &frame,
    const std::string &title) {
  (void) task;
  (void) frame;
  (void) title;

  if (!renderer_) {
    ERROR("Cannot create window without a renderer assigned");
    return nullptr;
  }

  auto width = 1024;
  auto height = 768;
  if (original_crtc_) {
    width = original_crtc_->mode.hdisplay;
    height = original_crtc_->mode.vdisplay;
  }

  anbox::graphics::Rect window_frame{0, 0, width, height};

  graphics::emugl::DisplayInfo::get()->set_resolution(window_frame.width(), window_frame.height());

  DEBUG("width %d height %d", window_frame.width(), window_frame.height());

  if (!window_) {
    auto gs = gbm_surface_create(gbm_,
             window_frame.width(), window_frame.height(),
             GBM_BO_FORMAT_XRGB8888,
             GBM_BO_USE_SCANOUT | GBM_BO_USE_RENDERING);

    window_ = Window::create(renderer_, gs, window_frame);
  }

  return window_;
}

void Platform::on_gbm_bo_destroyed(gbm_bo *bo, void *data) {
  auto fb = static_cast<FrameBuffer*>(data);
  if (fb->id) {
    gbm_device *device = gbm_bo_get_device(bo);
    drmModeRmFB(gbm_device_get_fd(device), fb->id);
  }
  delete fb;
}

Platform::FrameBuffer* Platform::frame_buffer_from_bo(gbm_bo *bo) {
  auto fb = static_cast<FrameBuffer*>(gbm_bo_get_user_data(bo));
  if (fb)
    return fb;

  const auto width = gbm_bo_get_width(bo);
  const auto height = gbm_bo_get_height(bo);
  const auto stride = gbm_bo_get_stride(bo);
  const auto handle = gbm_bo_get_handle(bo).u32;

  fb = new FrameBuffer;

  if (drmModeAddFB(fd_, width, height, 24, 32,
                   stride, handle, &fb->id)) {
    WARNING("Failed to create KMS frame buffer");
    delete fb;
    return nullptr;
  }

  gbm_bo_set_user_data(bo, fb, on_gbm_bo_destroyed);

  return fb;
}

void Platform::on_page_flip(int fd,
                          unsigned int sequence,
                          unsigned int tv_sec,
                          unsigned int tv_usec,
                          void *user_data) {
  (void) fd;
  (void) sequence;
  (void) tv_sec;
  (void) tv_usec;

  auto value = static_cast<bool*>(user_data);
  *value = false;
}

bool Platform::wait_for_page_flip(int timeout) {
  pollfd drm_pollfd;
  drm_pollfd.events = POLLIN;
  drm_pollfd.fd = fd_;

  while (page_flip_pending_) {
    drm_pollfd.revents = 0;
    if (poll(&drm_pollfd, 1, timeout) < 0) {
      WARNING("Error while waiting for page flip to happen");
      return false;
    }

    if (drm_pollfd.revents & (POLLHUP | POLLERR)) {
      WARNING("Received error or hup event while waiting for page flip");
      return false;
    }

    if (drm_pollfd.revents & POLLIN) {
      drmEventContext drm_event = { DRM_EVENT_CONTEXT_VERSION, nullptr, on_page_flip };
      drmHandleEvent(fd_, &drm_event);
    } else {
      WARNING("Timed out while waiting for page flip");
      return false;
    }
  }

  return true;
}

EGLNativeDisplayType Platform::native_display() const {
  return reinterpret_cast<EGLNativeDisplayType>(gbm_);
}

EGLDisplay Platform::create_display() {
  return s_egl.eglGetDisplay(native_display());
}

bool Platform::choose_config(EGLDisplay display, EGLConfig *config) {
  const EGLint attribs[] = {
    EGL_RED_SIZE, 1,
    EGL_GREEN_SIZE, 1,
    EGL_BLUE_SIZE, 1,
    EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
    EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
    EGL_NONE
  };

  int n;
  if (!s_egl.eglChooseConfig(display, attribs, config, 1, &n))
    return false;

  return true;
}

EGLSurface Platform::create_offscreen_surface(EGLDisplay display, EGLConfig config,
                                              unsigned int width, unsigned int height) {
  auto gs = gbm_surface_create(gbm_, width, height,
                               GBM_BO_FORMAT_XRGB8888,
                               GBM_BO_USE_SCANOUT | GBM_BO_USE_RENDERING);
  if (!gs)
    return EGL_NO_SURFACE;

  auto surface = s_egl.eglCreateWindowSurface(display, config, reinterpret_cast<EGLNativeWindowType>(gs), nullptr);
  if (surface == EGL_NO_SURFACE) {
    gbm_surface_destroy(gs);
    return EGL_NO_SURFACE;
  }

  offscreen_surfaces_.insert({surface, gs});

  return surface;
}

void Platform::destroy_offscreen_surface(EGLDisplay display, EGLSurface surface) {
  s_egl.eglDestroySurface(display, surface);

  auto iter = offscreen_surfaces_.find(surface);
  if (iter == offscreen_surfaces_.end()) {
    WARNING("Surface without attached GBM surface!?");
    return;
  }

  gbm_surface_destroy(iter->second);
  offscreen_surfaces_.erase(iter);
}

void Platform::swap_buffers(EGLDisplay display, EGLSurface surface) {
  if (window_->egl_surface() != surface)
    return;

  auto gs = reinterpret_cast<gbm_surface*>(window_->native_handle());
  if (!gs) {
    WARNING("Cannot swap buffers for window without a surface");
    return;
  }

  if (!s_egl.eglSwapBuffers(display, surface)) {
    WARNING("Cannot swap buffers");
    return;
  }

  next_bo_ = gbm_surface_lock_front_buffer(gs);
  if (!next_bo_) {
    WARNING("Failed to lock front buffer of surface");
    return;
  }

  auto fb = frame_buffer_from_bo(next_bo_);
  if (!fb) {
    WARNING("Cannot create frame buffer from GBM buffer");
    return;
  }

  if (drmModePageFlip(fd_, encoder_->crtc_id, fb->id,
                      DRM_MODE_PAGE_FLIP_EVENT, &page_flip_pending_)) {

    // If the page flip failed we use the fallback blitting with via
    // setting the buffer directly to the crtc. This may tear.
    if (drmModeSetCrtc(fd_, encoder_->crtc_id, fb->id, 0, 0,
                       &saved_conn_id_, 1, &mode_)) {
      WARNING("Cannot queue DRM page flip: %s", strerror(errno));
      gbm_surface_release_buffer(gs, next_bo_);
      next_bo_ = nullptr;
      return;
    }
  } else {
    // FIXME bind to eglSwapInterval
    int timeout = -1;
    if (!wait_for_page_flip(timeout))
      return;
  }

  if (current_bo_)
    gbm_surface_release_buffer(gs, current_bo_);

  current_bo_ = next_bo_;
  next_bo_ = nullptr;
}

bool Platform::supports_multi_window() const {
  return false;
}

bool Platform::supports_cursor() const {
  return false;
}
} // namespace kmsdrm
} // namespace platform
} // namespace anbox
