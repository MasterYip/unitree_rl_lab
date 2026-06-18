// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <array>
#include <string>
#include <cmath>
#include <algorithm>
#include <spdlog/spdlog.h>
#include "unitree/dds_wrapper/common/unitree_joystick.hpp"

/// @brief Reads a standard gamepad from /dev/input/jsX and overrides
///        a UnitreeJoystick state. Uses the Linux joystick API with
///        non-blocking I/O so it is safe to call inside the 1 ms FSM loop.
///
/// Standard gamepad mapping (matches DEFAULT_AXIS_NAMES / DEFAULT_BUTTON_NAMES):
///
///   Axes:
///     0 = left_x   → lx     [-1, 1]
///     1 = left_y   → ly     [-1, 1]  (inverted on most gamepads)
///     2 = l2       → LT     [0, 1]   (auto-calibrated trigger)
///     3 = right_x  → rx     [-1, 1]
///     4 = right_y  → ry     [-1, 1]  (inverted on most gamepads)
///     5 = r2       → RT     [0, 1]   (auto-calibrated trigger)
///     6 = dpad_x   → left/right      (-1=left, 0=neutral, +1=right)
///     7 = dpad_y   → up/down         (-1=up,  0=neutral, +1=down)
///
///   Buttons:
///     0 = A, 1 = B, 2 = X, 3 = Y
///     4 = L1 (→LB),  5 = R1 (→RB)
///     6 = Select (→back), 7 = Start (→start)
///     8 = Mode (unmapped)
///     9 = L3 (→LS), 10 = R3 (→RS)
///     (11-14 = dpad buttons, also checked as fallback)
class CustomJoystick
{
public:
    CustomJoystick(const std::string& device = "/dev/input/js0")
    {
        fd_ = open(device.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd_ < 0) {
            spdlog::warn("CustomJoystick: failed to open '{}' ({}). "
                         "Is the gamepad connected?", device, strerror(errno));
        } else {
            spdlog::info("CustomJoystick: reading from '{}'", device);
        }
    }

    ~CustomJoystick()
    {
        if (fd_ >= 0) {
            close(fd_);
        }
    }

    bool is_connected() const { return fd_ >= 0; }

    /// Read all pending js_event structs. Safe to call at >1 kHz.
    void poll()
    {
        if (fd_ < 0) return;

        struct js_event ev;
        ssize_t n;
        while ((n = read(fd_, &ev, sizeof(ev))) == sizeof(ev)) {
            switch (ev.type & ~JS_EVENT_INIT) {
            case JS_EVENT_BUTTON:
                if (ev.number < static_cast<int>(buttons_.size())) {
                    buttons_[ev.number] = ev.value;
                }
                break;
            case JS_EVENT_AXIS:
                if (ev.number < static_cast<int>(axis_raw_.size())) {
                    axis_raw_[ev.number] = ev.value;

                    // Normalise joystick axes (0,1,3,4) to [-1, 1]
                    if (ev.number == 0 || ev.number == 1 ||
                        ev.number == 3 || ev.number == 4) {
                        axes_[ev.number] = std::clamp(
                            ev.value / 32767.0f, -1.0f, 1.0f);
                    }
                    // Auto-calibrate trigger axes (2=l2, 5=r2)
                    if (ev.number == 2) {
                        l2_min_ = std::min(l2_min_, static_cast<float>(ev.value));
                        l2_max_ = std::max(l2_max_, static_cast<float>(ev.value));
                    }
                    if (ev.number == 5) {
                        r2_min_ = std::min(r2_min_, static_cast<float>(ev.value));
                        r2_max_ = std::max(r2_max_, static_cast<float>(ev.value));
                    }
                    // D-pad axes (6=dpad_x, 7=dpad_y) — discrete: -32767/0/+32767
                    if (ev.number == 6) {
                        dpad_x_ = (ev.value > 16384) ? 1 :
                                  (ev.value < -16384) ? -1 : 0;
                    }
                    if (ev.number == 7) {
                        dpad_y_ = (ev.value > 16384) ? 1 :
                                  (ev.value < -16384) ? -1 : 0;
                    }
                }
                break;
            default:
                break;
            }
        }

        // Compute normalised trigger values [0, 1] from calibrated range
        auto norm_trigger = [](float raw, float lo, float hi) -> float {
            if (hi - lo < 1.0f) return 0.0f;
            return std::clamp((raw - lo) / (hi - lo), 0.0f, 1.0f);
        };
        axes_[2] = norm_trigger(axis_raw_[2], l2_min_, l2_max_);
        axes_[5] = norm_trigger(axis_raw_[5], r2_min_, r2_max_);
    }

    /// Override every button / axis on `joystick` with the latest js0 state.
    /// The Unitree "extract"-style operator() methods handle edge detection
    /// (on_pressed / on_released) and axis deadband / smoothing internally.
    void apply_to(unitree::common::UnitreeJoystick& joystick)
    {
        if (fd_ < 0) return;

        // --- axes (ly/ry are inverted on most gamepads) ---
        joystick.lx(axes_[0]);
        joystick.ly(-axes_[1]);
        joystick.LT(axes_[2]);
        joystick.rx(axes_[3]);
        joystick.ry(-axes_[4]);
        joystick.RT(axes_[5]);

        // --- d-pad: primary source is axes 6/7, fallback to buttons 11-14 ---
        bool up    = (dpad_y_ == -1) || (buttons_[11] != 0);
        bool down  = (dpad_y_ ==  1) || (buttons_[12] != 0);
        bool left  = (dpad_x_ == -1) || (buttons_[13] != 0);
        bool right = (dpad_x_ ==  1) || (buttons_[14] != 0);
        joystick.up(up ? 1 : 0);
        joystick.down(down ? 1 : 0);
        joystick.left(left ? 1 : 0);
        joystick.right(right ? 1 : 0);

        // --- face buttons ---
        joystick.A(buttons_[0]);
        joystick.B(buttons_[1]);
        joystick.X(buttons_[2]);
        joystick.Y(buttons_[3]);

        // --- bumpers (L1=LB, R1=RB) ---
        joystick.LB(buttons_[4]);
        joystick.RB(buttons_[5]);

        // --- menu buttons (Select=back) ---
        joystick.back(buttons_[6]);
        joystick.start(buttons_[7]);

        // --- stick clicks (L3=LS, R3=RS) ---
        joystick.LS(buttons_[9]);
        joystick.RS(buttons_[10]);
    }

private:
    int fd_ = -1;

    /// Normalised joystick axis values (0=lx, 1=ly, 2=LT, 3=rx, 4=ry, 5=RT, 6/7 unused)
    std::array<float, 8> axes_{};

    /// Raw axis values for trigger auto-calibration
    std::array<float, 8> axis_raw_{};

    /// Button state (0 = released, 1 = pressed)
    std::array<int, 16> buttons_{};

    /// D-pad discrete values from axes 6 (dpad_x) and 7 (dpad_y): -1, 0, or +1
    int dpad_x_ = 0;
    int dpad_y_ = 0;

    /// Auto-calibration range for l2 (axis 2)
    float l2_min_{0.0f}, l2_max_{32767.0f};

    /// Auto-calibration range for r2 (axis 5)
    float r2_min_{0.0f}, r2_max_{32767.0f};
};
