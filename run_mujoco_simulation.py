
# run_mujoco_simulation.py (usa per-joint gains por defecto + flags opcionales)
import os
import argparse
import mujoco

try:
    import mujoco.viewer
except Exception:
    mujoco = mujoco

from so101_mujoco_utils2 import set_initial_pose, RealtimeJointPlotter
from so101_mujoco_pid_utils import (
    move_to_pose_pid, hold_position_pid,
    build_default_pid, set_uniform_gains,
    RunLogger,
)

MODEL_PATH = "/home/carly/Desktop/Control/simulation_code/model/scene_urdf.xml"

def parse_args():
    p = argparse.ArgumentParser(description="SO101 MuJoCo PID study (per-joint ready)")
    p.add_argument("--family", choices=["P", "PD", "PI", "PID"], default="PID")
    p.add_argument("--kp", type=float, default=25.0)
    p.add_argument("--kd", type=float, default=0.5)
    p.add_argument("--ki", type=float, default=0.3)
    p.add_argument("--use_perjoint_gains", action="store_true",
                   help="Usa las ganancias definidas per-joint en build_default_pid() y NO sobrescribe con --kp/--kd/--ki")
    p.add_argument("--disturb_profile", choices=["off","smooth","realistic","stress"], default="off")
    p.add_argument("--disturb_scale", type=float, default=1.0)
    p.add_argument("--duration_move", type=float, default=2.0)
    p.add_argument("--duration_hold", type=float, default=2.0)
    p.add_argument("--outdir", type=str, default="data/runs")
    p.add_argument("--combo_id", type=str, default="best")
    p.add_argument("--no_plot", action="store_true", help="Disable Dash realtime plot")
    p.add_argument("--headless", action="store_true", help="Run without viewer (use MUJOCO_GL=osmesa)")
    p.add_argument("--no_minjerk", action="store_true", help="Desactiva suavizado mínimo-jerk")
    return p.parse_args()

def main():
    args = parse_args()

    m = mujoco.MjModel.from_xml_path(MODEL_PATH)
    d = mujoco.MjData(m)

    starting_position = {
        "shoulder_pan":  -4.4003158666,
        "shoulder_lift": -92.2462050161,
        "elbow_flex":     89.9543738355,
        "wrist_flex":     55.1185398916,
        "wrist_roll":      0.0,
        "gripper":         0.0,
    }
    desired_zero = {
        "shoulder_pan":  0.0,
        "shoulder_lift": 0.0,
        "elbow_flex":    0.0,
        "wrist_flex":    0.0,
        "wrist_roll":    0.0,
        "gripper":       0.0,
    }

    set_initial_pose(m, d, starting_position)

    pid = build_default_pid()
    if not args.use_perjoint_gains:
        # Mantén compatibilidad con barridos uniformes
        kp, kd, ki = args.kp, args.kd, args.ki
        if args.family == "P":
            kd = 0.0; ki = 0.0
        elif args.family == "PD":
            ki = 0.0
        elif args.family == "PI":
            kd = 0.0
        set_uniform_gains(pid, kp=kp, ki=ki, kd=kd)

    plotter = None
    if not args.no_plot:
        plotter = RealtimeJointPlotter(max_points=4000)
        plotter.start(host="127.0.0.1", port=8050, update_ms=100)

    os.makedirs(args.outdir, exist_ok=True)
    csv_path = os.path.join(args.outdir, f"{args.family}_{args.combo_id}.csv")
    logger = RunLogger(csv_path)

    def do_seq(viewer):
        move_to_pose_pid(m, d, viewer, desired_zero, duration=args.duration_move,  realtime=True,
                         plotter=plotter, logger=logger,
                         disturb_profile=args.disturb_profile, disturb_scale=args.disturb_scale,
                         use_minimum_jerk=not args.no_minjerk)
        hold_position_pid(m, d, viewer, desired_zero,  duration=args.duration_hold,  realtime=True,
                          plotter=plotter, logger=logger,
                          disturb_profile=args.disturb_profile, disturb_scale=args.disturb_scale)
        move_to_pose_pid(m, d, viewer, starting_position, duration=args.duration_move, realtime=True,
                         plotter=plotter, logger=logger,
                         disturb_profile=args.disturb_profile, disturb_scale=args.disturb_scale,
                         use_minimum_jerk=not args.no_minjerk)
        hold_position_pid(m, d, viewer, starting_position, duration=args.duration_hold, realtime=True,
                          plotter=plotter, logger=logger,
                          disturb_profile=args.disturb_profile, disturb_scale=args.disturb_scale)

    if args.headless:
        do_seq(viewer=None)
    else:
        with mujoco.viewer.launch_passive(m, d) as viewer:
            do_seq(viewer)

    logger.close()
    print(f"[OK] Guardado: {csv_path}")

if __name__ == "__main__":
    main()