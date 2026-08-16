// pybind11 bindings over core/WooDriveSdk.h.
//
// Exposes a Pythonic subset of the WooDrive protocol engine plus a native,
// platform-appropriate serial transport (PosixSerialTransport on
// Linux/macOS, WindowsSerialTransport on Windows) so `import woodrive` works
// out of the box without needing pyserial. This is a starting scaffold, not
// full coverage of every core/ getter/setter -- extend as needed.

#include <pybind11/pybind11.h>

#include "WooDriveSdk.h"

#if defined(_WIN32)
#include "WindowsSerialTransport.h"
#else
// core/WooDriveSdk.h only declares PosixSerialTransport/StdClock when
// __linux__ or __APPLE__ is defined -- same condition as here.
#endif

namespace py = pybind11;

namespace {

#if defined(_WIN32)
using NativeTransport = WindowsSerialTransport;
using NativeClock = WindowsClock;
#else
using NativeTransport = PosixSerialTransport;
using NativeClock = StdClock;
#endif

}  // namespace

PYBIND11_MODULE(_woodrive, m)
{
    m.doc() = "Native bindings for the WooDrive motor controller SDK";

    // --- Transport / clock -------------------------------------------------
    // ITransport/IClock are registered too (with no constructor exposed,
    // since they're abstract) purely so pybind11 knows SerialTransport/Clock
    // derive from them and can convert a Python instance into the
    // ITransport&/IClock& that WooDrive's constructor expects below.
    py::class_<ITransport>(m, "_ITransport");
    py::class_<IClock>(m, "_IClock");

    // Named "SerialTransport" / "Clock" in Python regardless of platform;
    // the actual native implementation is chosen at compile time above.
    py::class_<NativeTransport, ITransport>(m, "SerialTransport")
        .def(py::init<const char*, int>(), py::arg("port"), py::arg("baudrate") = 9600,
             "Open a serial port (e.g. \"COM5\" on Windows, \"/dev/ttyUSB0\" on Linux).")
        .def("is_open", &NativeTransport::isOpen)
        .def("baudrate", &NativeTransport::baudrate);

    py::class_<NativeClock, IClock>(m, "Clock").def(py::init<>());

    // --- Protocol enums ------------------------------------------------------
    py::enum_<DirectionPhase>(m, "DirectionPhase",
                               "Protocol V1.1.0, address 0x13 (direction and phase).")
        .value("NORMAL", DirectionPhase::Normal)
        .value("DIRECTION_INVERTED", DirectionPhase::DirectionInverted)
        .value("PHASE_SWAPPED", DirectionPhase::PhaseSwapped)
        .value("DIRECTION_INVERTED_AND_PHASE_SWAPPED", DirectionPhase::DirectionInvertedAndPhaseSwapped);

    // --- Status snapshot -------------------------------------------------
    py::class_<WooDrive::MotorStatus>(m, "MotorStatus")
        .def(py::init<>())
        .def_readonly("total_time_ms", &WooDrive::MotorStatus::totalTime)
        .def_readonly("elapsed_time_ms", &WooDrive::MotorStatus::elapsedTime)
        .def_readonly("remain_time_ms", &WooDrive::MotorStatus::remainTime)
        .def_readonly("position_deg", &WooDrive::MotorStatus::position)
        .def_readonly("velocity_rpm", &WooDrive::MotorStatus::velocity)
        .def_readonly("iq_current_a", &WooDrive::MotorStatus::iqCurrent)
        .def_readonly("id_current_a", &WooDrive::MotorStatus::idCurrent)
        .def_readonly("bus_current_a", &WooDrive::MotorStatus::busCurrent)
        .def_readonly("vq_voltage_v", &WooDrive::MotorStatus::vqVoltage)
        .def_readonly("vd_voltage_v", &WooDrive::MotorStatus::vdVoltage)
        .def_readonly("bus_voltage_v", &WooDrive::MotorStatus::busVoltage)
        .def_readonly("temperature_c", &WooDrive::MotorStatus::temperature)
        .def_readonly("pulse_count", &WooDrive::MotorStatus::pulseCount)
        .def_readonly("u_phase_current_a", &WooDrive::MotorStatus::uPhaseCurrent)
        .def_readonly("w_phase_current_a", &WooDrive::MotorStatus::wPhaseCurrent);

    // --- Core driver -----------------------------------------------------
    py::class_<WooDrive>(m, "WooDrive")
        .def(py::init<ITransport&, IClock&>(), py::arg("transport"), py::arg("clock"),
             // Keep the transport/clock Python objects alive at least as
             // long as the WooDrive instance holds references to them.
             py::keep_alive<1, 2>(), py::keep_alive<1, 3>())
        .def("set_timeout", &WooDrive::setTimeout, py::arg("timeout_ms"))
        .def("timeout", &WooDrive::timeout)

        // Board config
        .def("get_id",
             [](WooDrive& self, uint8_t id) {
                 uint8_t out = 0;
                 bool ok = self.getId(id, out);
                 return py::make_tuple(ok, out);
             },
             py::arg("id"), "Returns (ok, value).")
        .def("get_fault",
             [](WooDrive& self, uint8_t id) {
                 uint8_t out = 0;
                 bool ok = self.getFault(id, out);
                 return py::make_tuple(ok, out);
             },
             py::arg("id"), "Returns (ok, fault_code).")

        // Direction / phase (Protocol V1.1.0)
        .def("set_direction_phase", &WooDrive::setDirectionPhase, py::arg("id"), py::arg("value"))
        .def("get_direction_phase",
             [](WooDrive& self, uint8_t id) {
                 DirectionPhase out = DirectionPhase::Normal;
                 bool ok = self.getDirectionPhase(id, out);
                 return py::make_tuple(ok, out);
             },
             py::arg("id"), "Returns (ok, DirectionPhase).")

        // Motor enable / brake
        .def("set_motor_enable", &WooDrive::setMotorEnable, py::arg("id"), py::arg("value"))
        .def("set_motor_brake", &WooDrive::setMotorBrake, py::arg("id"), py::arg("value"))

        // High-level motion command (see core/WooDriveSdk.h for motion mode values)
        .def("set_motor_motion_all", &WooDrive::setMotorMotionAll, py::arg("id"), py::arg("accel_ms"),
             py::arg("decel_ms"), py::arg("motion_mode"), py::arg("sub_target"), py::arg("main_target"),
             py::arg("direction"))

        // Status
        .def("get_position_velocity",
             [](WooDrive& self, uint8_t id) {
                 float position = 0.0f, velocity = 0.0f;
                 bool ok = self.getPositionVelocity(id, position, velocity);
                 return py::make_tuple(ok, position, velocity);
             },
             py::arg("id"), "Returns (ok, position_deg, velocity_rpm).")
        .def("get_motor_status_all",
             [](WooDrive& self, uint8_t id) {
                 WooDrive::MotorStatus status{};
                 bool ok = self.getMotorStatusAll(id, status);
                 return py::make_tuple(ok, status);
             },
             py::arg("id"), "Returns (ok, MotorStatus).");
}
