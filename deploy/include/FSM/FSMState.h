#pragma once

#include "Types.h"
#include "param.h"
#include "FSM/BaseState.h"
#include "isaaclab/devices/keyboard/keyboard.h"
#include "unitree_joystick_dsl.hpp"
#include "common/CustomJoystick.h"

namespace fsm_debug {

struct JoystickSnapshot {
    bool lt_pressed;
    bool lt_on_pressed;
    bool lt_on_released;
    bool rb_pressed;
    bool rb_on_pressed;
    bool rb_on_released;
    bool x_pressed;
    bool x_on_pressed;
    bool x_on_released;
    bool up_pressed;
    bool up_on_pressed;
    bool up_on_released;
    bool down_pressed;
    bool left_pressed;
    bool right_pressed;
    float lx;
    float ly;
    float rx;
    float ry;
    float lt;
    float rt;
};

inline JoystickSnapshot capture_snapshot(unitree::common::UnitreeJoystick& joystick)
{
    return {
        joystick.LT.pressed,
        joystick.LT.on_pressed,
        joystick.LT.on_released,
        joystick.RB.pressed,
        joystick.RB.on_pressed,
        joystick.RB.on_released,
        joystick.X.pressed,
        joystick.X.on_pressed,
        joystick.X.on_released,
        joystick.up.pressed,
        joystick.up.on_pressed,
        joystick.up.on_released,
        joystick.down.pressed,
        joystick.left.pressed,
        joystick.right.pressed,
        joystick.lx(),
        joystick.ly(),
        joystick.rx(),
        joystick.ry(),
        joystick.LT(),
        joystick.RT(),
    };
}

inline bool should_log(const JoystickSnapshot& current, const JoystickSnapshot& previous)
{
    constexpr float kAxisThreshold = 0.05f;
    auto axis_changed = [](float current_axis, float previous_axis) {
        return std::fabs(current_axis - previous_axis) > kAxisThreshold;
    };

    return current.lt_pressed != previous.lt_pressed
        || current.lt_on_pressed
        || current.lt_on_released
        || current.rb_pressed != previous.rb_pressed
        || current.rb_on_pressed
        || current.rb_on_released
        || current.x_pressed != previous.x_pressed
        || current.x_on_pressed
        || current.x_on_released
        || current.up_pressed != previous.up_pressed
        || current.up_on_pressed
        || current.up_on_released
        || current.down_pressed != previous.down_pressed
        || current.left_pressed != previous.left_pressed
        || current.right_pressed != previous.right_pressed
        || axis_changed(current.lx, previous.lx)
        || axis_changed(current.ly, previous.ly)
        || axis_changed(current.rx, previous.rx)
        || axis_changed(current.ry, previous.ry)
        || axis_changed(current.lt, previous.lt)
        || axis_changed(current.rt, previous.rt);
}

inline void log_joystick_state(unitree::common::UnitreeJoystick& joystick)
{
    static bool has_previous_snapshot = false;
    static JoystickSnapshot previous_snapshot{};

    const JoystickSnapshot current_snapshot = capture_snapshot(joystick);
    if (has_previous_snapshot && !should_log(current_snapshot, previous_snapshot)) {
        return;
    }

    spdlog::info(
        "Gamepad rx: LT[p={} op={} or={} hold={:.2f}s trig={:.2f}] up[p={} op={} or={}] RB[p={} op={} or={}] X[p={} op={} or={}] dpad[down={} left={} right={}] axes[lx={:.2f} ly={:.2f} rx={:.2f} ry={:.2f} rt={:.2f}]",
        joystick.LT.pressed,
        joystick.LT.on_pressed,
        joystick.LT.on_released,
        joystick.LT.pressed_time,
        current_snapshot.lt,
        joystick.up.pressed,
        joystick.up.on_pressed,
        joystick.up.on_released,
        joystick.RB.pressed,
        joystick.RB.on_pressed,
        joystick.RB.on_released,
        joystick.X.pressed,
        joystick.X.on_pressed,
        joystick.X.on_released,
        joystick.down.pressed,
        joystick.left.pressed,
        joystick.right.pressed,
        current_snapshot.lx,
        current_snapshot.ly,
        current_snapshot.rx,
        current_snapshot.ry,
        current_snapshot.rt
    );

    previous_snapshot = current_snapshot;
    has_previous_snapshot = true;
}

} // namespace fsm_debug

class FSMState : public BaseState
{
public:
    FSMState(int state, std::string state_string) 
    : BaseState(state, state_string) 
    {
        spdlog::info("Initializing State_{} ...", state_string);

        auto transitions = param::config["FSM"][state_string]["transitions"];

        if(transitions)
        {
            auto transition_map = transitions.as<std::map<std::string, std::string>>();

            for(auto it = transition_map.begin(); it != transition_map.end(); ++it)
            {
                std::string target_fsm = it->first;
                if(!FSMStringMap.right.count(target_fsm))
                {
                    spdlog::warn("FSM State_'{}' not found in FSMStringMap!", target_fsm);
                    continue;
                }

                int fsm_id = FSMStringMap.right.at(target_fsm);

                std::string condition = it->second;
                unitree::common::dsl::Parser p(condition);
                auto ast = p.Parse();
                auto func = unitree::common::dsl::Compile(*ast);
                registered_checks.emplace_back(
                    std::make_pair(
                        [func]()->bool{ return func(FSMState::lowstate->joystick); },
                        fsm_id
                    )
                );
            }
        }

        // register for all states
        registered_checks.emplace_back(
            std::make_pair(
                []()->bool{ return lowstate->isTimeout(); },
                FSMStringMap.right.at("Passive")
            )
        );
    }

    void pre_run()
    {
        lowstate->update();
        // Override joystick state with custom device if enabled
        if (!param::custom_joystick.empty()) {
            static CustomJoystick custom_joy(param::custom_joystick);
            custom_joy.poll();
            custom_joy.apply_to(lowstate->joystick);
        }
        // fsm_debug::log_joystick_state(lowstate->joystick);
        if(keyboard) keyboard->update();
    }

    void post_run()
    {
        lowcmd->unlockAndPublish();
    }

    static std::unique_ptr<LowCmd_t> lowcmd;
    static std::shared_ptr<LowState_t> lowstate;
    static std::shared_ptr<Keyboard> keyboard;
};