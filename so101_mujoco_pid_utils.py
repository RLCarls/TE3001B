
# so101_mujoco_pid_utils.py (best per-joint gains + minimum-jerk + profiles)
from __future__ import annotations

import time
from typing import Protocol, Optional
import numpy as np
import mujoco

# --- JOINTS primero (para que RunLogger pueda usarlos como default) ---
DEFAULT_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

# --- Logging util ---
import csv, os
from datetime import datetime

class RunLogger:
    """
    Escribe una fila por paso: t, q[i], qd[i], qref[i], tau_pid[i], tau_dist[i], tau_total[i]
    """
    def __init__(self, path: str, joint_names: list[str] | None = None):
        if joint_names is None:
            joint_names = DEFAULT_JOINTS
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.joint_names = list(joint_names)
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        header = (["t"] +
                  [f"q[{jn}]" for jn in self.joint_names] +
                  [f"qd[{jn}]" for jn in self.joint_names] +
                  [f"qref[{jn}]" for jn in self.joint_names] +
                  [f"tau_pid[{jn}]" for jn in self.joint_names] +
                  [f"tau_dist[{jn}]" for jn in self.joint_names] +
                  [f"tau_total[{jn}]" for jn in self.joint_names])
        self.w.writerow(header)

    def row(self, t, q, qd, qref, tau_pid, tau_dist, tau_total):
        def vec(dct): return [float(dct[jn]) for jn in self.joint_names]
        self.w.writerow(
            [float(t)] + vec(q) + vec(qd) + vec(qref) + vec(tau_pid) + vec(tau_dist) + vec(tau_total)
        )

    def close(self):
        self.f.close()

# --- Tipado del plotter (opcional) ---
class _PlotterProto(Protocol):
    def sample(self, m, d, now: float | None = None) -> None: ...

# --- Control / Perturbaciones / Helpers desde so101_control ---
from so101_control import (
    JointPID, PIDGains,
    PerturbationModel, PerturbationConfig,
    get_q_qd_dict,
    apply_joint_torques_qfrc,
)

# --- Utilidades de ganancias ---
def set_uniform_gains(pid: JointPID, kp: float = 0.0, ki: float = 0.0, kd: float = 0.0):
    for jn, g in pid.gains.items():
        g.kp, g.ki, g.kd = float(kp), float(ki), float(kd)

# --- Helpers de movimiento ---
def lerp_pose(p0: dict[str, float], p1: dict[str, float], s: float) -> dict[str, float]:
    s = float(np.clip(s, 0.0, 1.0))
    out = {}
    for k in p0.keys():
        out[k] = (1.0 - s) * p0[k] + s * p1[k]
    return out

# Suavizado temporal mínimo-jerk (mejor arranque/parada)
def smoothstep_jerk(s: float) -> float:
    s = float(np.clip(s, 0.0, 1.0))
    return 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5

# === MEJORES GANANCIAS PER-JOINT (de tu compañero) ===
def build_default_pid(joint_names=DEFAULT_JOINTS) -> JointPID:
    gains = {
        "shoulder_pan":  PIDGains(kp=70.0, ki=0.7, kd=2.7, i_limit=2.0, tau_limit=8.0),
        "shoulder_lift": PIDGains(kp=40.0, ki=0.7, kd=1.4, i_limit=2.0, tau_limit=18.0),
        "elbow_flex":    PIDGains(kp=31.0, ki=0.7, kd=0.7, i_limit=2.0, tau_limit=15.0),
        "wrist_flex":    PIDGains(kp=20.0, ki=0.7, kd=0.7, i_limit=2.0, tau_limit=8.0),
        "wrist_roll":    PIDGains(kp=13.0, ki=0.7, kd=1.1, i_limit=2.0, tau_limit=8.0),
    }
    for jn in joint_names:
        if jn not in gains:
            gains[jn] = PIDGains(kp=25.0, ki=0.3, kd=1.0, i_limit=2.0, tau_limit=6.0)
    return JointPID(joint_names, gains)

# Perfiles de perturbación (opcional)
def build_default_perturbations(
    joint_names=DEFAULT_JOINTS,
    profile: str = "off",
    scale: float = 1.0,
    seed: int = 7,
) -> PerturbationModel:
    scale = float(np.clip(scale, 0.0, 10.0))

    if profile == "off":
        cfg = PerturbationConfig(
            sinus_amp=0.0, sinus_freq_hz=0.0,
            noise_std=0.0, noise_tau=0.25,
            impulse_prob_per_s=0.0, impulse_mag=0.0, impulse_dur=0.0,
            meas_q_std=0.0, meas_qd_std=0.0, seed=seed
        )
        return PerturbationModel(joint_names, cfg)

    if profile == "smooth":
        cfg = PerturbationConfig(
            sinus_amp=0.20 * scale,
            sinus_freq_hz=0.40,
            noise_std=0.06 * scale,
            noise_tau=0.35,
            impulse_prob_per_s=0.0,
            impulse_mag=0.0,
            impulse_dur=0.0,
            meas_q_std=0.0,
            meas_qd_std=0.0,
            seed=seed,
        )
        return PerturbationModel(joint_names, cfg)

    if profile == "realistic":
        cfg = PerturbationConfig(
            sinus_amp=0.50 * scale,
            sinus_freq_hz=0.50,
            noise_std=0.15 * scale,
            noise_tau=0.25,
            impulse_prob_per_s=0.04 * scale,
            impulse_mag=0.8 * scale,
            impulse_dur=0.03,
            meas_q_std=0.0,
            meas_qd_std=0.0,
            seed=seed,
        )
        return PerturbationModel(joint_names, cfg)

    if profile == "stress":
        cfg = PerturbationConfig(
            sinus_amp=0.90 * scale,
            sinus_freq_hz=0.60,
            noise_std=0.25 * scale,
            noise_tau=0.20,
            impulse_prob_per_s=0.12 * scale,
            impulse_mag=2.0 * scale,
            impulse_dur=0.05,
            meas_q_std=0.0,
            meas_qd_std=0.0,
            seed=seed,
        )
        return PerturbationModel(joint_names, cfg)

    raise ValueError(f"Unknown profile='{profile}'. Use off/smooth/realistic/stress.")


def step_sim(m, d, viewer, realtime: bool, plotter: Optional[_PlotterProto] = None):
    """Un paso de MuJoCo + (plot opcional) + (viewer opcional) + (pacing opcional)."""
    mujoco.mj_step(m, d)
    if plotter is not None:
        plotter.sample(m, d)
    if viewer is not None:
        viewer.sync()
    if realtime:
        time.sleep(m.opt.timestep)


def move_to_pose_pid(
    m, d, viewer,
    target_pose_deg: dict[str, float],
    duration: float = 2.0,
    realtime: bool = True,
    joint_names=DEFAULT_JOINTS,
    pid: JointPID | None = None,
    perturb: PerturbationModel | None = None,
    plotter: Optional[_PlotterProto] = None,
    logger: RunLogger | None = None,
    # nuevos (opcionales)
    disturb_profile: str | None = None,
    disturb_scale: float | None = None,
    use_minimum_jerk: bool = True,
):
    """PID en torque + perturbaciones opcionales. Interpola de q0 -> qT en 'duration'."""
    if pid is None:
        pid = build_default_pid(joint_names)
    if perturb is None:
        prof = disturb_profile if disturb_profile is not None else "off"
        sc   = disturb_scale if disturb_scale is not None else 1.0
        perturb = build_default_perturbations(joint_names, profile=prof, scale=sc)

    pid.reset()
    q0, _ = get_q_qd_dict(m, d, joint_names)
    qT = {jn: np.deg2rad(target_pose_deg[jn]) for jn in joint_names}

    steps = int(max(1, duration / m.opt.timestep))
    t0 = float(d.time)

    for _ in range(steps):
        t = float(d.time)
        s = (t - t0) / max(duration, 1e-9)
        s = float(np.clip(s, 0.0, 1.0))
        s_ref = smoothstep_jerk(s) if use_minimum_jerk else s

        q_des = lerp_pose(q0, qT, s_ref)

        q, qd = get_q_qd_dict(m, d, joint_names)
        q_meas, qd_meas = perturb.noisy_measurement(q, qd)

        tau_pid = pid.compute(q_meas, qd_meas, q_des, m.opt.timestep)
        tau_dist = perturb.apply_joint_torques(t=t, dt=m.opt.timestep)
        tau_total = {jn: tau_pid[jn] + tau_dist[jn] for jn in joint_names}

        apply_joint_torques_qfrc(m, d, joint_names, tau_total)

        if logger is not None:
            logger.row(t=t, q=q, qd=qd, qref=q_des, tau_pid=tau_pid, tau_dist=tau_dist, tau_total=tau_total)

        step_sim(m, d, viewer, realtime=realtime, plotter=plotter)


def hold_position_pid(
    m, d, viewer,
    hold_pose_deg: dict[str, float],
    duration: float = 2.0,
    realtime: bool = True,
    joint_names=DEFAULT_JOINTS,
    pid: JointPID | None = None,
    perturb: PerturbationModel | None = None,
    plotter: Optional[_PlotterProto] = None,
    logger: RunLogger | None = None,
    disturb_profile: str | None = None,
    disturb_scale: float | None = None,
):
    """Mantiene una postura fija con PID inyectando perturbaciones."""
    if pid is None:
        pid = build_default_pid(joint_names)
    if perturb is None:
        prof = disturb_profile if disturb_profile is not None else "off"
        sc   = disturb_scale if disturb_scale is not None else 1.0
        perturb = build_default_perturbations(joint_names, profile=prof, scale=sc)

    pid.reset()
    q_des = {jn: np.deg2rad(hold_pose_deg[jn]) for jn in joint_names}
    steps = int(max(1, duration / m.opt.timestep))

    for _ in range(steps):
        t = float(d.time)
        q, qd = get_q_qd_dict(m, d, joint_names)
        q_meas, qd_meas = perturb.noisy_measurement(q, qd)

        tau_pid = pid.compute(q_meas, qd_meas, q_des, m.opt.timestep)
        tau_dist = perturb.apply_joint_torques(t=t, dt=m.opt.timestep)
        tau_total = {jn: tau_pid[jn] + tau_dist[jn] for jn in joint_names}

        apply_joint_torques_qfrc(m, d, joint_names, tau_total)

        if logger is not None:
            logger.row(t=t, q=q, qd=qd, qref=q_des, tau_pid=tau_pid, tau_dist=tau_dist, tau_total=tau_total)

        step_sim(m, d, viewer, realtime=realtime, plotter=plotter)