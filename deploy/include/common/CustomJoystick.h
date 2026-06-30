// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <string>
#include <cmath>
#include <algorithm>
#include <spdlog/spdlog.h>
#include "unitree/dds_wrapper/common/unitree_joystick.hpp"
#include "common/joystick_parser.hpp"

/// @brief Reads a standard gamepad from /dev/input/jsX and overrides a
///        unitree::common::UnitreeJoystick state with the captured input.
///
/// Uses joystick::JoystickParser (a standalone header-only Linux js* reader)
/// for device I/O and logical-name resolution.  The adapter maps logical
/// axis / button names (e.g. "left_x", "south") onto the Unitree joystick API.
///
/// Standard Xbox mapping (built into JoystickParser):
///
///   Axes:
///     left_x   → lx     [-1, 1]
///     left_y   → ly     [-1, 1]  (inverted — up is positive in Unitree)
///     l2       → LT     [0, 1]   (auto-calibrated trigger)
///     right_x  → rx     [-1, 1]
///     right_y  → ry     [-1, 1]  (inverted)
///     r2       → RT     [0, 1]   (auto-calibrated trigger)
///     dpad_x   → left / right    (-1 / 0 / +1)
///     dpad_y   → up / down       (-1 / 0 / +1)
///
///   Buttons:
///     south→A, east→B, west→X, north→Y
///     l1→LB, r1→RB
///     select→back, start→start
///     l3→LS, r3→RS
class CustomJoystick
{
public:
    /// @param device  Path to the joystick device, e.g. "/dev/input/js0".
    /// @param mapping Optional mapping config pointer.  If nullptr, the
    ///                built-in Xbox mapping is used.
    explicit CustomJoystick(const std::string& device = "/dev/input/js0",
                            const joystick::JoystickMappingConfig* mapping = nullptr)
        : parser_(device, mapping ? mapping : &joystick::get_mapping("xbox"))
    {
        if (!parser_.is_connected()) {
            spdlog::warn("CustomJoystick: failed to open '{}'. "
                         "Is the gamepad connected?", device);
        } else {
            spdlog::info("CustomJoystick: reading from '{}'", device);
        }
    }

    bool is_connected() const { return parser_.is_connected(); }

    /// Read all pending js_event structs. Non-blocking — safe to call inside
    /// the 1 ms FSM loop.
    void poll() { parser_.poll(); }

    /// Override every button / axis on @p joystick with the latest gamepad
    /// state.  The Unitree "extract"-style operator() methods handle edge
    /// detection (on_pressed / on_released) and axis deadband / smoothing
    /// internally.
    void apply_to(unitree::common::UnitreeJoystick& joystick)
    {
        if (!parser_.is_connected()) return;

        // -- stick axes (normalised to [-1, 1], Y-inverted) --
        joystick.lx( get_stick("left_x") );
        joystick.ly(-get_stick("left_y") );
        joystick.rx( get_stick("right_x") );
        joystick.ry(-get_stick("right_y") );

        // -- triggers (auto-calibrated to [0, 1]) --
        joystick.LT( get_trigger("l2", l2_min_, l2_max_) );
        joystick.RT( get_trigger("r2", r2_min_, r2_max_) );

        // -- d-pad (primary: dpad_x / dpad_y axes; fallback: buttons 11-14) --
        int dpad_x = parser_.get_axis_raw("dpad_x");
        int dpad_y = parser_.get_axis_raw("dpad_y");
        bool up    = (dpad_y < 0) || parser_.get_button_by_number(11);
        bool down  = (dpad_y > 0) || parser_.get_button_by_number(12);
        bool left  = (dpad_x < 0) || parser_.get_button_by_number(13);
        bool right = (dpad_x > 0) || parser_.get_button_by_number(14);
        joystick.up(up ? 1 : 0);
        joystick.down(down ? 1 : 0);
        joystick.left(left ? 1 : 0);
        joystick.right(right ? 1 : 0);

        // -- face buttons --
        joystick.A(parser_.get_button("south") ? 1 : 0);
        joystick.B(parser_.get_button("east")  ? 1 : 0);
        joystick.X(parser_.get_button("west")  ? 1 : 0);
        joystick.Y(parser_.get_button("north") ? 1 : 0);

        // -- bumpers (L1→LB, R1→RB) --
        joystick.LB(parser_.get_button("l1") ? 1 : 0);
        joystick.RB(parser_.get_button("r1") ? 1 : 0);

        // -- menu buttons --
        joystick.back(parser_.get_button("select") ? 1 : 0);
        joystick.start(parser_.get_button("start") ? 1 : 0);

        // -- stick clicks --
        joystick.LS(parser_.get_button("l3") ? 1 : 0);
        joystick.RS(parser_.get_button("r3") ? 1 : 0);
    }

private:
    /// Normalise a stick axis to [-1, 1] using the standard signed-16-bit
    /// joystick range.
    static float get_stick(int raw) {
        return std::clamp(raw / 32767.0f, -1.0f, 1.0f);
    }
    float get_stick(const char* logical) {
        return get_stick(parser_.get_axis_raw(logical));
    }

    /// Auto-calibrate and normalise a trigger axis to [0, 1].
    float get_trigger(const char* logical, float& lo, float& hi) {
        float raw = static_cast<float>(parser_.get_axis_raw(logical));
        lo = std::min(lo, raw);
        hi = std::max(hi, raw);
        if (hi - lo < 1.0f) return 0.0f;
        return std::clamp((raw - lo) / (hi - lo), 0.0f, 1.0f);
    }

    joystick::JoystickParser parser_;

    /// Auto-calibration range for l2 (left trigger)
    float l2_min_{0.0f}, l2_max_{32767.0f};
    /// Auto-calibration range for r2 (right trigger)
    float r2_min_{0.0f}, r2_max_{32767.0f};
};
