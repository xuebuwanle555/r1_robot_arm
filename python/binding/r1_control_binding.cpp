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
// Threading / GIL - and the honest limit of this design
//   Every method here calls back into Python, so it MUST run with the GIL held.
//   Two shapes are exposed, and they are not interchangeable:
//
//     1. run_cycle(): Python schedules the C++ control cycle. This is the
//        normal, safe pattern - Python owns scheduling, C++ owns every safety
//        decision made inside the cycle.
//     2. start()/stop(): the C++ service runs its own periodic thread, exactly
//        as it would against hardware. Exposed only so the command-timeout
//        watchdog can be tested with Python NOT calling run_cycle(); every
//        cycle then re-acquires the GIL to enter Python. If the main interpreter
//        is stuck WHILE HOLDING the GIL, that cycle blocks - the C++ watchdog
//        cannot fire. This is structurally why the sim bridge must never be
//        wired to a real arm: a hardware service needs a backend that does not
//        depend on the Python interpreter being able to give up the GIL.
//
//   The bridge backend also reports isRealHardware() == false, so none of the
//   real-hardware gates (allow_real_hardware, calibration) can be satisfied here.
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

// Validate a numpy array BEFORE anything reads it.  `py::cast<py::array_t<T>>`
// alone will happily accept a 3-element, non-contiguous or wrong-dtype array and
// then read past the end of the buffer, so shape, element count, contiguity and
// dtype are all checked explicitly and a violation raises into Python.
py::array checked_array(const py::object& attr, const char* what,
                        const py::dtype& want) {
  if (!py::isinstance<py::array>(attr)) {
    throw py::type_error(std::string(what) + " must be a numpy array");
  }
  const py::array arr = py::reinterpret_borrow<py::array>(attr);
  if (arr.ndim() != 1) {
    throw py::value_error(std::string(what) + " must be 1-dimensional, got ndim=" +
                          std::to_string(arr.ndim()));
  }
  if (arr.size() != static_cast<py::ssize_t>(kNumJoints)) {
    throw py::value_error(std::string(what) + " must have " +
                          std::to_string(kNumJoints) + " elements, got " +
                          std::to_string(arr.size()));
  }
  if (!(arr.flags() & py::array::c_style)) {
    throw py::value_error(std::string(what) + " must be C-contiguous");
  }
  const py::dtype dt = arr.dtype();
  if (dt.kind() != want.kind() || dt.itemsize() != want.itemsize()) {
    throw py::type_error(std::string(what) + " has an unexpected dtype");
  }
  return arr;
}

template <typename T>
void copy_array(const py::object& attr, const char* what, T* dst) {
  const py::array arr = checked_array(attr, what, py::dtype::of<T>());
  const T* src = static_cast<const T*>(arr.data());
  for (int i = 0; i < kNumJoints; ++i) {
    dst[i] = src[i];
  }
}

/// 7 joint values from either a numpy array or a plain sequence.
JointVector joint_vector_from_python(const py::handle& item, const char* what) {
  JointVector out{};
  if (py::isinstance<py::array>(item)) {
    const py::array arr = py::reinterpret_borrow<py::array>(item);
    copy_array<double>(py::reinterpret_borrow<py::object>(item), what, out.data());
    return out;
  }
  if (!py::isinstance<py::sequence>(item)) {
    throw py::type_error(std::string(what) +
                         " must be a numpy array or a 7-element sequence");
  }
  const py::sequence seq = py::reinterpret_borrow<py::sequence>(item);
  if (static_cast<std::size_t>(seq.size()) != kNumJoints) {
    throw py::value_error(std::string(what) + " must have " +
                          std::to_string(kNumJoints) + " elements, got " +
                          std::to_string(seq.size()));
  }
  for (int i = 0; i < kNumJoints; ++i) {
    out[static_cast<std::size_t>(i)] = py::cast<double>(seq[i]);
  }
  return out;
}

// -- Python r1_robot.RobotState  ->  C++ RobotState -------------------------
RobotState from_python_state(const py::object& st) {
  RobotState out;
  out.connected = st.attr("connected").cast<bool>();
  out.stamp_ns = st.attr("stamp_ns").cast<std::uint64_t>();
  copy_array<double>(st.attr("q"), "RobotState.q", out.q.data());
  copy_array<double>(st.attr("dq"), "RobotState.dq", out.dq.data());
  copy_array<double>(st.attr("tau"), "RobotState.tau", out.tau.data());
  copy_array<double>(st.attr("voltage"), "RobotState.voltage", out.voltage.data());
  copy_array<double>(st.attr("temperature_c"), "RobotState.temperature_c",
                    out.temperature_c.data());

  std::int64_t mode[kNumJoints] = {0};
  copy_array<std::int64_t>(st.attr("mode"), "RobotState.mode", mode);
  for (int i = 0; i < kNumJoints; ++i) {
    out.mode[static_cast<std::size_t>(i)] =
        (mode[i] == 1) ? ControlMode::kFoc : ControlMode::kStop;
  }

  std::uint32_t err[kNumJoints] = {0};
  copy_array<std::uint32_t>(st.attr("error_code"), "RobotState.error_code", err);
  for (int i = 0; i < kNumJoints; ++i) {
    out.error_code[static_cast<std::size_t>(i)] = err[i];
  }

  bool valid[kNumJoints] = {false};
  copy_array<bool>(st.attr("valid"), "RobotState.valid", valid);
  for (int i = 0; i < kNumJoints; ++i) {
    out.valid[static_cast<std::size_t>(i)] = valid[i];
  }
  return out;
}

// -- Python r1_robot.ArmCommand  ->  C++ ArmCommand -------------------------
ArmCommand from_python_command(const py::object& cmd) {
  ArmCommand out;
  out.timed_out = cmd.attr("timed_out").cast<bool>();
  double q[kNumJoints], dq[kNumJoints], tau[kNumJoints], kp[kNumJoints],
      kd[kNumJoints];
  copy_array<double>(cmd.attr("q"), "ArmCommand.q", q);
  copy_array<double>(cmd.attr("dq"), "ArmCommand.dq", dq);
  copy_array<double>(cmd.attr("tau"), "ArmCommand.tau", tau);
  copy_array<double>(cmd.attr("kp"), "ArmCommand.kp", kp);
  copy_array<double>(cmd.attr("kd"), "ArmCommand.kd", kd);
  std::int64_t mode[kNumJoints] = {0};
  copy_array<std::int64_t>(cmd.attr("mode"), "ArmCommand.mode", mode);
  for (int i = 0; i < kNumJoints; ++i) {
    JointCommand& j = out.joints[static_cast<std::size_t>(i)];
    j.q = q[i];
    j.dq = dq[i];
    j.tau = tau[i];
    j.kp = kp[i];
    j.kd = kd[i];
    j.mode = (mode[i] == 1) ? ControlMode::kFoc : ControlMode::kStop;
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
  cfg.stop_settle_ns = c.attr("stop_settle_ns").cast<std::uint64_t>();
  cfg.stop_vel_tol = c.attr("stop_vel_tol").cast<double>();
  cfg.hold_kp = c.attr("hold_kp").cast<double>();
  cfg.hold_kd = c.attr("hold_kd").cast<double>();
  // Anti-step bound (command vs. measured).  Its real value is a hardware-tuning
  // concern (HARDWARE_UNVERIFIED); a lagging plant needs a looser bound than the
  // conservative default, so it is configurable here.
  cfg.safety.max_joint_step_rad = c.attr("max_joint_step_rad").cast<double>();
  cfg.safety.allow_real_hardware = c.attr("allow_real_hardware").cast<bool>();
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

  py::enum_<StopReason>(m, "StopReason")
      .value("NONE", StopReason::kNone)
      .value("OPERATOR_STOP", StopReason::kOperatorStop)
      .value("UPPER_LAYER_TIMEOUT", StopReason::kUpperLayerTimeout)
      .value("FEEDBACK_LOST", StopReason::kFeedbackLost)
      .value("ACTUATOR_FAULT", StopReason::kActuatorFault)
      .value("UNSAFE_STATE", StopReason::kUnsafeState)
      .export_values();

  py::class_<ControlServiceConfig>(m, "ControlServiceConfig")
      .def(py::init<>())
      .def_readwrite("period_ns", &ControlServiceConfig::period_ns)
      .def_readwrite("command_timeout_ns", &ControlServiceConfig::command_timeout_ns)
      .def_readwrite("stop_settle_ns", &ControlServiceConfig::stop_settle_ns)
      .def_readwrite("stop_vel_tol", &ControlServiceConfig::stop_vel_tol)
      .def_readwrite("hold_kp", &ControlServiceConfig::hold_kp)
      .def_readwrite("hold_kd", &ControlServiceConfig::hold_kd)
      .def_property("allow_real_hardware",
                    [](const ControlServiceConfig& c) {
                      return c.safety.allow_real_hardware;
                    },
                    [](ControlServiceConfig& c, bool v) {
                      c.safety.allow_real_hardware = v;
                    })
      .def_property("max_joint_step_rad",
                    [](const ControlServiceConfig& c) {
                      return c.safety.max_joint_step_rad;
                    },
                    [](ControlServiceConfig& c, double v) {
                      c.safety.max_joint_step_rad = v;
                    });

  py::enum_<SubmitStatus>(m, "SubmitStatus")
      .value("ACCEPTED", SubmitStatus::kAccepted)
      .value("REJECTED", SubmitStatus::kRejected)
      .export_values();

  py::class_<SubmitResult>(m, "SubmitResult")
      .def_property_readonly("ok", [](const SubmitResult& r) { return r.ok(); })
      .def_property_readonly("status",
                             [](const SubmitResult& r) { return r.status; })
      .def_readonly("error", &SubmitResult::error)
      .def_readonly("waypoint_index", &SubmitResult::waypoint_index)
      .def("__bool__", [](const SubmitResult& r) { return r.ok(); });

  py::class_<ControlLoopMetrics>(m, "ControlLoopMetrics")
      .def_readonly("cycles", &ControlLoopMetrics::cycles)
      .def_readonly("period_ns", &ControlLoopMetrics::period_ns)
      .def_readonly("overruns", &ControlLoopMetrics::overruns)
      .def_readonly("consecutive_overruns", &ControlLoopMetrics::consecutive_overruns)
      .def_readonly("send_failures", &ControlLoopMetrics::send_failures)
      .def_readonly("total_exec_ns", &ControlLoopMetrics::total_exec_ns)
      .def_readonly("max_exec_ns", &ControlLoopMetrics::max_exec_ns)
      .def_readonly("max_jitter_ns", &ControlLoopMetrics::max_jitter_ns)
      .def_readonly("measured_rate_hz", &ControlLoopMetrics::measured_rate_hz);

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
           [](ControlService& s, const py::object& wps) {
             if (!py::isinstance<py::sequence>(wps)) {
               throw py::type_error(
                   "waypoints must be a sequence of (t_ns, q[7]) pairs");
             }
             const py::sequence seq = py::reinterpret_borrow<py::sequence>(wps);
             std::vector<TrajWaypoint> out;
             out.reserve(static_cast<std::size_t>(seq.size()));
             for (std::size_t i = 0; i < static_cast<std::size_t>(seq.size()); ++i) {
               const py::handle item = seq[py::int_(static_cast<Py_ssize_t>(i))];
               if (!py::isinstance<py::sequence>(item) ||
                   py::reinterpret_borrow<py::sequence>(item).size() != 2) {
                 throw py::value_error(
                     "waypoint " + std::to_string(i) +
                     " must be a (t_ns, q) pair with exactly 2 fields");
               }
               const py::sequence pair =
                   py::reinterpret_borrow<py::sequence>(item);
               TrajWaypoint t;
               const py::object t_handle = pair[py::int_(0)];
               if (!py::isinstance<py::int_>(t_handle)) {
                 throw py::type_error("waypoint " + std::to_string(i) +
                                      " t_ns must be an integer, not a " +
                                      std::string(py::str(t_handle.ptr()->ob_type->tp_name)));
               }
               const long long raw_t = py::cast<long long>(t_handle);
               if (raw_t < 0) {
                 throw py::value_error("waypoint " + std::to_string(i) +
                                       " has a negative t_ns");
               }
               t.t_ns = static_cast<std::uint64_t>(raw_t);
               t.q = joint_vector_from_python(
                   pair[py::int_(1)],
                   ("waypoint " + std::to_string(i) + " q").c_str());
               out.push_back(t);
             }
             return s.submitTrajectory(out);
           },
           py::arg("waypoints"),
           "Validate and buffer a joint trajectory. Returns a SubmitResult: an "
           "unsafe plan is refused with a reason and NOTHING is executed.")
      .def("submit_command",
           [](ControlService& s, const py::object& cmd) {
             return s.submitCommand(from_python_command(cmd));
           },
           py::arg("command"),
           "Submit one setpoint frame. Returns a SubmitResult.")
      .def("request_controlled_stop", &ControlService::requestControlledStop)
      // Operator-only drive removal.  Exposed as a deliberate call because the
      // service will never take this step by itself.
      .def("confirm_motor_disable",
           [](ControlService& s) {
             std::string err;
             bool ok = s.confirmMotorDisable(&err);
             return std::make_pair(ok, err);
           })
      .def("stop_reason", &ControlService::stopReason)
      .def("stop_hold_verified", &ControlService::stopHoldVerified)
      // The synchronous, Python-scheduled step.  Provided so the C++ safety
      // stack can be exercised against real physics in a deterministic test;
      // it is NOT how a real arm is driven.
      .def("run_cycle", &ControlService::runCycle,
           "Run one control cycle synchronously (simulation bridge only).")
      // The service's own periodic thread.  Exposed so the watchdog can be
      // tested WITHOUT Python cooperating: once started, nothing may call
      // run_cycle() and the service must still notice the silence.  Each cycle
      // re-enters Python and therefore needs the GIL, which is exactly why this
      // path is for simulation and must never be wired to hardware.
      .def("start", &ControlService::start)
      .def("stop", &ControlService::stop)
      .def("metrics", &ControlService::metrics)
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
