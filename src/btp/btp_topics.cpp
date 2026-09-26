// The robot's telemetry. Topic ids and names are the ones TraceView layouts
// already know (0x0001..0x0005). The fills run on BtpDevice's comms task,
// only at each topic's subscribed rate: they copy snapshots the robot keeps
// (ROBOT::readSensors/readFlags), never touch hardware themselves.
#include "btp/btp_robot.h"

#include <cstring>
#include <string>

#include <BallyRobot.h>

namespace {

constexpr std::uint16_t kProtocolTestTopicId = 0x0001U;
constexpr std::uint16_t kRobotStateTopicId = 0x0002U;
constexpr std::uint16_t kSystemMonitorTopicId = 0x0003U;
constexpr std::uint16_t kSensorsTopicId = 0x0004U;
constexpr std::uint16_t kFlagsTopicId = 0x0005U;

constexpr const char* kArrayFields[ArraySensor::MAX_LEN] = {
    "array_0", "array_1", "array_2", "array_3", "array_4", "array_5", "array_6", "array_7",
};
static_assert(ArraySensor::MAX_LEN == 8U, "one array_N field per ArraySensor channel");

}  // namespace

void robot_topics(btpdev::Topics& t) {
    ROBOT& robot = ROBOT::getInstance();

    // A link check: a counter and a float whose IEEE-754 bits (3f 0d 0a 00,
    // on the wire 00 0a 0d 3f) prove NUL, LF and CR travel as data.
    t.topic("protocol.test", btpdev::Hz(50))
        .id(kProtocolTestTopicId)
        .u32("counter", "1")
        .f32("value", "1")
        .fill([&robot](btpdev::Sample& s) {
            constexpr uint32_t kEdgeFloatBits = 0x3F0D0A00U;
            float edge_value = 0.0f;
            std::memcpy(&edge_value, &kEdgeFloatBits, sizeof(edge_value));
            s.put("counter", robot.nextProtocolTestCounter()).put("value", edge_value);
        });

    // One sample per state transition (ROBOT::runStateMachine()).
    t.topic("robot.state", btpdev::OnEvent)
        .id(kRobotStateTopicId)
        .u8("state", "1")
        .fill([&robot](btpdev::Sample& s) { s.put("state", robot.currentState()); });

    // TraceView's Text Board: the task table and memory, once a second at most.
    t.text("system.monitor", btpdev::Hz(1),
           [&robot](btpdev::Text& out) {
               const std::string report = robot.systemReport(btpdev::Text::kCapacity);
               out.append(report.c_str());
           })
        .id(kSystemMonitorTopicId);

    btpdev::TopicDecl& sensors = t.topic("robot.sensors", btpdev::Hz(50))
                                     .id(kSensorsTopicId)
                                     .f32("linear_speed", "m/s")
                                     .f32("angular_speed", "rad/s")
                                     .f32("gyro_z", "rad/s")
                                     .f32("accel_x", "m/s^2")
                                     .f32("accel_y", "m/s^2")
                                     // Raw ADC counts, not amps: the DRV8251A
                                     // current-sense scale is not calibrated yet.
                                     .u16("current_a", "counts").range(0, 4095)
                                     .u16("current_b", "counts").range(0, 4095);
    // Raw ADC >> 4, uncalibrated, plotting only.
    for (const char* name : kArrayFields) sensors.u8(name, "counts");
    sensors.fill([&robot](btpdev::Sample& s) {
        ROBOT::SensorsSample v;
        robot.readSensors(v);
        s.put("linear_speed", v.linear_speed)
            .put("angular_speed", v.angular_speed)
            .put("gyro_z", v.gyro_z)
            .put("accel_x", v.accel_x)
            .put("accel_y", v.accel_y)
            .put("current_a", v.current_a)
            .put("current_b", v.current_b);
        for (std::size_t i = 0U; i < ArraySensor::MAX_LEN; ++i) s.put(kArrayFields[i], v.array[i]);
    });

    t.topic("robot.flags", btpdev::Hz(50))
        .id(kFlagsTopicId)
        .u8("buttons", "1")
        .u8("side_sensors", "1")
        .u8("leds", "1")
        // -100..100 is the PWM duty-cycle range, narrower than int8's.
        .i8("pwm_left", "%").range(-100, 100)
        .i8("pwm_right", "%").range(-100, 100)
        .fill([&robot](btpdev::Sample& s) {
            ROBOT::FlagsSample v;
            robot.readFlags(v);
            s.put("buttons", v.buttons)
                .put("side_sensors", v.side_sensors)
                .put("leds", v.leds)
                .put("pwm_left", v.pwm_left)
                .put("pwm_right", v.pwm_right);
        });
}
