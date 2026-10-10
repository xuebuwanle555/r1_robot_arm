// ---------------------------------------------------------------------------
// r1_control_binding.cpp  —  pybind11 bridge: the C++ ControlService drives the
// existing Python MuJoCo SimArm.
//
// WHY THIS DIRECTION (not linking MuJoCo into C++)
//   MuJoCo already works cleanly as a Python dependency, and the safety
//   authority is meant to stay in C++ (see README "Python does not own the
//   real-time loop").  So we keep the *plant* in Python (SimArm) and put a thin
//   C++ IArmBackend adapter in front of it; the production ControlService -
//   identical to the one exercised against MockJointBus in the unit tests - then
//   owns this backend and runs the SAME gating / watchdog / fault-latch logic
//   against real physics.
//
// Threading / GIL
//   Every method here calls back into Python, so it MUST run with the GIL held.
//   The binding therefore exposes the synchronous runCycle() step and lets
//   Python schedule it (the ordinary, safe pattern).  The C++ internal periodic
//   thread (start()) is intentionally NOT exposed: driving Python from a
//   detached std::thread needs explicit gil_scoped_acquire and is a separate,
//   hardware-side concern.
// ---------------------------------------------------------------------------
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "r1_robot/backend.hpp"
#include "r1_robot/control_service.hpp"
#include "r1_robot/limits.hpp"
#include "r1_robot/types.hpp"

namespace py = pybind11;
using namespace r1_robot;

namespace {

// -- C++ ArmCommand  ->  Python r1_robot.ArmCommand -------------------------
py::object to_python_command(const ArmCommand& cmd) {
  py::object rr = py::module_::import("r1_robot");
  py::object arm_cmd = rr.attr("ArmCommand")();  // defaults to a stop frame

  py::array_t<double> q(kNumJoints), dq(kNumJoints), tau(kNumJoints),
      kp(kNumJoints), kd(kNumJoints);
  py::array_t<std::int64_t> mode(kNumJoints);
  double* pq = static_cast<double*>(q.request().ptr);
  double* pdq = static_cast<double*>(dq.request().ptr);
  double* ptau = static_cast<double*>(tau.request().ptr);
  double* pkp = static_cast<double*>(kp.request().ptr);
  double* pkd = static_cast<double*>(kd.request().ptr);
  std::int64_t* pmode = static_cast<std::int64_t*>(mode.request().ptr);
  for (int i = 0; i < kNumJoints; ++i) {
    const JointCommand& j = cmd.joints[static_cast<std::size_t>(i)];
    pq[i] = j.q;
    pdq[i] = j.dq;
    ptau[i] = j.tau;
    pkp[i] = j.kp;
    pkd[i] = j.kd;
    pmode[i] = (j.mode == ControlMode::kFoc) ? 1 : 0;
  }
  arm_cmd.attr("mode") = mode;
  arm_cmd.attr("q") = q;
  arm_cmd.attr("dq") = dq;
  arm_cmd.attr("tau") = tau;
  arm_cmd.attr("kp") = kp;
  arm_cmd.attr("kd") = kd;
  arm_cmd.attr("timed_out") = cmd.timed_out;
  return arm_cmd;
}

template <typename T>
void copy_array(const py::object& attr, T* dst) {
  py::array_t<T> arr = py::cast<py::array_t<T>>(attr);
  auto r = arr.request();
  const T* src = static_cast<const T*>(r.ptr);
  for (int i = 0; i < kNumJoints; ++i) {
    dst[i] = src[i];
  }
}

// -- Python r1_robot.RobotState  ->  C++ RobotState -------------------------
RobotState from_python_state(const py::object& st) {
  RobotState out;
  out.connected = st.attr("connected").cast<bool>();
  out.stamp_ns = st.attr("stamp_ns").cast<std::uint64_t>();
  copy_array<double>(st.attr("q"), out.q.data());
  copy_array<double>(st.attr("dq"), out.dq.data());
  copy_array<double>(st.attr("tau"), out.tau.data());
  copy_array<double>(st.attr("voltage"), out.voltage.data());
  copy_array<double>(st.attr("temperature_c"), out.temperature_c.data());

  py::array_t<std::int64_t> mode =
      py::cast<py::array_t<std::int64_t>>(st.attr("mode"));
  {
    const std::int64_t* p = static_cast<const std::int64_t*>(mode.request().ptr);
    for (int i = 0; i < kNumJoints; ++i) {
      out.mode[static_cast<std::size_t>(i)] =
          (p[i] == 1) ? ControlMode::kFoc : ControlMode::kStop;
    }
  }
  py::array_t<std::uint32_t> err =
      py::cast<py::array_t<std::uint32_t>>(st.attr("error_code"));
  {
    const std::uint32_t* p = static_cast<const std::uint32_t*>(err.request().ptr);
    for (int i = 0; i < kNumJoints; ++i) {
      out.error_code[static_cast<std::size_t>(i)] = p[i];
    }
  }
  py::array_t<bool> valid = py::cast<py::array_t<bool>>(st.attr("valid"));
  {
    const bool* p = static_cast<const bool*>(valid.request().ptr);
    for (int i = 0; i < kNumJoints; ++i) {
      out.valid[static_cast<std::size_t>(i)] = p[i];
    }
  }
  return out;
}

// -- PythonArmBackend: an IArmBackend that delegates to a Python SimArm ------
// isRealHardware() == false, but every actuating frame still flows through the
// production MotionGate/ControlService, exactly as with the hardware backends.
class PythonArmBackend final : public IArmBackend {
 public:
  explicit PythonArmBackend(py::object sim) : sim_(std::move(sim)) {}

  const char* name() const override { return "sim-bridge"; }
  bool isRealHardware() const override { return false; }

  bool connect(std::string* error) override {
    try {
      py::gil_scoped_acquire gil;
      sim_.attr("connect")();
      return true;
    } catch (const py::error_already_set& e) {
      if (error) {
        *error = e.what();
      }
      return false;
    }
  }
  void disconnect() override {
    py::gil_scoped_acquire gil;
    sim_.attr("disconnect")();
  }
  bool isConnected() const override {
    py::gil_scoped_acquire gil;
    return sim_.attr("is_connected").cast<bool>();
  }
  bool hasValidCalibration(std::string* detail) const override {
    py::gil_scoped_acquire gil;
    const bool ok = sim_.attr("has_valid_calibration")().cast<bool>();
    if (detail) {
      *detail = ok ? "sim calibration" : "sim reports no calibration";
    }
    return ok;
  }
  RobotLimits limits() const override { return r1DefaultLimits(); }

  RobotState readState() const override {
    py::gil_scoped_acquire gil;
    return from_python_state(sim_.attr("read_state")());
  }
  bool writeCommand(const ArmCommand& cmd) override {
    try {
      py::gil_scoped_acquire gil;
      sim_.attr("write_command")(to_python_command(cmd));
      return true;
    } catch (const py::error_already_set&) {
      return false;
    }
  }
  void stop() override {
    py::gil_scoped_acquire gil;
    sim_.attr("stop")();
  }

 private:
  py::object sim_;
};

ControlServiceConfig config_from_python(const py::object& c) {
  ControlServiceConfig cfg;
  cfg.period_ns = c.attr("period_ns").cast<std::uint64_t>();
  cfg.command_timeout_ns = c.attr("command_timeout_ns").cast<std::uint64_t>();
  cfg.stop_hold_ns = c.attr("stop_hold_ns").cast<std::uint64_t>();
  cfg.stop_vel_tol = c.attr("stop_vel_tol").cast<double>();
  cfg.hold_kp = c.attr("hold_kp").cast<double>();
  cfg.hold_kd = c.attr("hold_kd").cast<double>();
  // Anti-step bound (command vs. measured).  Its real value is a hardware-tuning
  // concern (HARDWARE_UNVERIFIED); a lagging plant needs a looser bound than the
  // conservative default, so it is configurable here.
  cfg.safety.max_joint_step_rad = c.attr("max_joint_step_rad").cast<double>();
  return cfg;
}

// -- Python-facing trajectory: a sequence of (t_ns, q[7]) items --------------
std::unique_ptr<ControlService> make_service(py::object sim, py::object config) {
  auto backend = std::make_shared<PythonArmBackend>(std::move(sim));
  return std::make_unique<ControlService>(backend, config_from_python(config));
}

}  // namespace

PYBIND11_MODULE(_control_binding, m) {
  m.doc() = "C++ ControlService driving the Python MuJoCo SimArm (opt-in bridge)";

  py::enum_<ServiceState>(m, "ServiceState")
      .value("UNCONNECTED", ServiceState::kUnconnected)
      .value("STANDBY", ServiceState::kStandby)
      .value("ENABLING", ServiceState::kEnabling)
      .value("TRACKING_TRAJECTORY", ServiceState::kTrackingTrajectory)
      .value("SOFTWARE_HOLD", ServiceState::kSoftwareHold)
      .value("CONTROLLED_STOP", ServiceState::kControlledStop)
      .value("MOTOR_DISABLED", ServiceState::kMotorDisabled)
      .export_values();

  py::class_<ControlServiceConfig>(m, "ControlServiceConfig")
      .def(py::init<>())
      .def_readwrite("period_ns", &ControlServiceConfig::period_ns)
      .def_readwrite("command_timeout_ns", &ControlServiceConfig::command_timeout_ns)
      .def_readwrite("stop_hold_ns", &ControlServiceConfig::stop_hold_ns)
      .def_readwrite("stop_vel_tol", &ControlServiceConfig::stop_vel_tol)
      .def_readwrite("hold_kp", &ControlServiceConfig::hold_kp)
      .def_readwrite("hold_kd", &ControlServiceConfig::hold_kd)
      .def_property("max_joint_step_rad",
                    [](const ControlServiceConfig& c) {
                      return c.safety.max_joint_step_rad;
                    },
                    [](ControlServiceConfig& c, double v) {
                      c.safety.max_joint_step_rad = v;
                    });

  py::class_<EnableResult>(m, "EnableResult")
      .def_readonly("ok", &EnableResult::ok)
      .def_readonly("error", &EnableResult::error);

  py::class_<ControlService>(m, "ControlService")
      .def("connect",
           [](ControlService& s) {
             std::string err;
             bool ok = s.connect(&err);
             return std::make_pair(ok, err);
           })
      .def("disconnect", &ControlService::disconnect)
      .def("enable_motion", &ControlService::enableMotion, py::arg("human_confirmed"))
      .def("disable_motion", &ControlService::disableMotion)
      .def("heartbeat", &ControlService::heartbeat)
      .def("submit_trajectory",
           [](ControlService& s, py::sequence wps) {
             std::vector<TrajWaypoint> out;
             for (const py::handle& item : wps) {
               py::sequence pair = py::cast<py::sequence>(item);
               TrajWaypoint t;
               t.t_ns = py::cast<std::uint64_t>(pair[py::int_(0)]);
               py::array_t<double> q =
                   py::cast<py::array_t<double>>(pair[py::int_(1)]);
               const double* p = static_cast<const double*>(q.request().ptr);
               for (int i = 0; i < kNumJoints; ++i) {
                 t.q[static_cast<std::size_t>(i)] = p[i];
               }
               out.push_back(t);
             }
             s.submitTrajectory(out);
           },
           py::arg("waypoints"))
      .def("request_controlled_stop", &ControlService::requestControlledStop)
      .def("run_cycle", &ControlService::runCycle)
      .def("clear_fault",
           [](ControlService& s) {
             std::string err;
             bool ok = s.clearFault(&err);
             return std::make_pair(ok, err);
           })
      .def("state", &ControlService::state)
      .def("fault_latched", &ControlService::faultLatched)
      .def("last_fault_code", &ControlService::lastFaultCode)
      .def("latest_state", &ControlService::latestState);

  m.def("make_control_service", &make_service,
        py::arg("sim_backend"), py::arg("config"),
        "Wrap a Python SimArm backend in the production C++ ControlService.");
}
