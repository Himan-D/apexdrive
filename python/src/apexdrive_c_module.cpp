#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include "../../include/apexdrive/core/types.hpp"
#include "../../include/apexdrive/control/foc_core.hpp"
#include "../../include/apexdrive/control/mtpa_optimizer.hpp"
#include "../../include/apexdrive/sim/isaac_mujoco_bridge.hpp"

using namespace apexdrive;

// Fast C++ FOC Computation for Python/RL environments
static PyObject* py_compute_foc(PyObject* /*self*/, PyObject* args) {
    float ia, ib, ic, theta_e, target_iq, v_bus;
    if (!PyArg_ParseTuple(args, "ffffff", &ia, &ib, &ic, &theta_e, &target_iq, &v_bus)) {
        return NULL;
    }

    MotorParameters params;
    FocEngine engine(params);
    FocEngine::FocInputs in{
        .i_phase_a = ia,
        .i_phase_b = ib,
        .i_phase_c = ic,
        .electrical_angle_rad = theta_e,
        .electrical_speed_rad_s = 0.0f,
        .target_id_a = 0.0f,
        .target_iq_a = target_iq,
        .v_bus = v_bus
    };

    auto out = engine.Step(in, 0.00004f);
    return Py_BuildValue("{s:f,s:f,s:f,s:f,s:f,s:f,s:f}",
        "duty_u", out.duty_u,
        "duty_v", out.duty_v,
        "duty_w", out.duty_w,
        "v_d_sat", out.v_d_sat,
        "v_q_sat", out.v_q_sat,
        "i_d", out.measured_id_a,
        "i_q", out.measured_iq_a
    );
}

// Fast Analytical MTPA Setpoint Generator for Python
static PyObject* py_compute_mtpa(PyObject* /*self*/, PyObject* args) {
    float torque_nm, v_bus_v, elec_speed_rad_s;
    if (!PyArg_ParseTuple(args, "fff", &torque_nm, &v_bus_v, &elec_speed_rad_s)) {
        return NULL;
    }

    MotorParameters params;
    MtpaOptimizer mtpa(params);
    auto opt = mtpa.ComputeOptimalCurrents(torque_nm, v_bus_v, elec_speed_rad_s, 0.001f);

    return Py_BuildValue("{s:f,s:f}",
        "target_id_a", opt.target_id_a,
        "target_iq_a", opt.target_iq_a
    );
}

// High-Speed Isaac / MuJoCo Joint Dynamics Physics Step
static PyObject* py_step_joint_physics(PyObject* /*self*/, PyObject* args) {
    float pos_rad, vel_rad_s, target_pos, target_vel, kp, kd, tau_ff, dt;
    if (!PyArg_ParseTuple(args, "ffffffff", &pos_rad, &vel_rad_s, &target_pos, &target_vel, &kp, &kd, &tau_ff, &dt)) {
        return NULL;
    }

    apexdrive::sim::IsaacMujocoJointBridge joint(0x10);
    joint.SetCommand(ImpedanceCommand{
        .target_pos_rad = target_pos,
        .target_vel_rad_s = target_vel,
        .stiffness_kp = kp,
        .damping_kd = kd,
        .feedforward_torque_nm = tau_ff
    });

    float torque_em = joint.StepJointDynamics(pos_rad, vel_rad_s, dt);
    auto state = joint.GetJointState();

    return Py_BuildValue("{s:f,s:f,s:f}",
        "applied_torque_nm", torque_em,
        "current_iq_a", state.current_iq_a,
        "current_id_a", state.current_id_a
    );
}

// Method Definitions Table
static PyMethodDef ApexDriveMethods[] = {
    {"compute_foc", py_compute_foc, METH_VARARGS, "Execute C++ FOC Engine step"},
    {"compute_mtpa", py_compute_mtpa, METH_VARARGS, "Compute optimal MTPA/FW setpoints"},
    {"step_joint_physics", py_step_joint_physics, METH_VARARGS, "Step high-speed Isaac/MuJoCo joint physics"},
    {NULL, NULL, 0, NULL}
};

// Module Definition Struct
static struct PyModuleDef apexdrive_c_module = {
    PyModuleDef_HEAD_INIT,
    "_apexdrive_c",
    "ApexDrive C++ Hardware-Accelerated Core Extension for Python & RL",
    -1,
    ApexDriveMethods
};

// Module Initialization Function
PyMODINIT_FUNC PyInit__apexdrive_c(void) {
    return PyModule_Create(&apexdrive_c_module);
}
