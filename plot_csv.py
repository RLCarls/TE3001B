#!/usr/bin/env python3
"""
plot_csv.py

Lee un CSV de data/runs generado por run_mujoco_simulation.py y guarda un PNG
con q vs qref, qd y tau_total por articulación.

Uso:
  python plot_csv.py data/runs/P_kp10.csv
  # o indicando salida
  python plot_csv.py data/runs/P_kp10.csv figures/P_kp10.png
"""
import sys, os
import pandas as pd
import matplotlib.pyplot as plt


def infer_joint_names(columns):
    return [c[2:-1] for c in columns if c.startswith("q[") and c.endswith("]")]

def plot_run(csv_path, out_png):
    df = pd.read_csv(csv_path)
    t = df["t"].values

    jns = infer_joint_names(df.columns)

    fig, axs = plt.subplots(3, 1, figsize=(11, 10), sharex=True)

    # q vs qref
    for jn in jns:
        axs[0].plot(t, df[f"q[{jn}]"], label=f"q[{jn}]")
        axs[0].plot(t, df[f"qref[{jn}]"], "--", label=f"qref[{jn}]")
    axs[0].set_ylabel("Posición [rad]")
    axs[0].legend(ncol=3, fontsize=8)

    # qd
    for jn in jns:
        axs[1].plot(t, df[f"qd[{jn}]"], label=f"qd[{jn}]")
    axs[1].set_ylabel("Vel. [rad/s]")
    axs[1].legend(ncol=3, fontsize=8)

    # tau total
    for jn in jns:
        axs[2].plot(t, df[f"tau_total[{jn}]"], label=f"tau[{jn}]")
    axs[2].set_ylabel("Torque [Nm]")
    axs[2].set_xlabel("Tiempo [s]")
    axs[2].legend(ncol=3, fontsize=8)

    fig.tight_layout()
    os.makedirs(os.path.dirname(out_png) or ".", exist_ok=True)
    fig.savefig(out_png, dpi=200)
    print("Saved", out_png)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Uso: python plot_csv.py <input.csv> [output.png]")
        raise SystemExit(2)
    csv = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(csv)[0] + ".png"
    plot_run(csv, out)