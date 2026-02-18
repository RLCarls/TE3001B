#!/usr/bin/env python3
"""
run_all_experiments.py

Ejecuta automáticamente las 4 familias (P, PD, PI, PID) con >=5 combinaciones
cada una, guarda los CSV en outdir, genera PNGs con plot_csv.py y crea un
CSV maestro con todos los resultados.

Requisitos:
  - Estar en la raíz del proyecto (donde está run_mujoco_simulation.py)
  - Tener pandas y matplotlib instalados (pip install pandas matplotlib)
  - MuJoCo ya instalado y el modelo accesible en model/scene_urdf.xml

Uso típico:
  export MUJOCO_GL=osmesa
  python run_all_experiments.py --outdir data/runs --headless --no-plot \
      --duration-move 2.0 --duration-hold 2.0
"""

import argparse
import os
import shlex
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List

PYTHON = sys.executable


def ensure_model_path():
    """Garantiza que model/scene_urdf.xml exista copiándolo desde assets si hace falta."""
    scene = Path("model/scene_urdf.xml")
    assets_scene = Path("model/assets/scene_urdf.xml")
    if not scene.exists() and assets_scene.exists():
        scene.write_text(assets_scene.read_text(encoding="utf-8"), encoding="utf-8")
        print("[INFO] Copiado model/assets/scene_urdf.xml -> model/scene_urdf.xml")
    if not scene.exists():
        raise SystemExit("[ERROR] No se encontró model/scene_urdf.xml (ni en assets).")


def run_cmd(cmd: List[str], env: Dict[str, str] | None = None) -> int:
    print("\n[RUN]", " ".join(shlex.quote(c) for c in cmd))
    return subprocess.call(cmd, env=env)


def plot_csv(csv_path: Path, out_png: Path):
    if not Path("plot_csv.py").exists():
        print("[WARN] plot_csv.py no existe, me salto el ploteo de", csv_path)
        return
    cmd = [PYTHON, "plot_csv.py", str(csv_path), str(out_png)]
    rc = run_cmd(cmd)
    if rc != 0:
        print("[WARN] Falló plot_csv.py para", csv_path)


def merge_all(outdir: Path, manifest_rows: List[Dict[str, str]]):
    """Funde todos los CSV en un maestro con metadatos de familia/gains/combo_id."""
    import pandas as pd

    rows = []
    for r in manifest_rows:
        csv_path = Path(r["csv_path"])
        if not csv_path.exists():
            continue
        try:
            df = pd.read_csv(csv_path)
        except Exception as e:
            print("[WARN] No pude leer", csv_path, e)
            continue
        # agrega columnas meta
        df.insert(0, "family", r["family"])
        df.insert(1, "combo_id", r["combo_id"])
        df.insert(2, "kp", r.get("kp"))
        df.insert(3, "kd", r.get("kd"))
        df.insert(4, "ki", r.get("ki"))
        rows.append(df)

    if not rows:
        print("[WARN] No hay CSV para fusionar.")
        return

    big = pd.concat(rows, ignore_index=True)
    out_master = outdir / "all_results.csv"
    big.to_csv(out_master, index=False)
    print("[OK] CSV maestro:", out_master)


def build_arg_parser():
    p = argparse.ArgumentParser(description="Ejecuta P/PD/PI/PID automáticamente")
    p.add_argument("--outdir", default="data/runs", help="Carpeta de salida para CSV/PNG")
    p.add_argument("--duration-move", type=float, default=2.0)
    p.add_argument("--duration-hold", type=float, default=2.0)
    p.add_argument("--headless", action="store_true", help="Llama a run_mujoco_simulation con --headless")
    p.add_argument("--no-plot", action="store_true", help="Llama a run_mujoco_simulation con --no_plot")
    p.add_argument("--pause", type=float, default=0.2, help="Pausa entre corridas (s)")
    p.add_argument("--gl", default=os.environ.get("MUJOCO_GL", "osmesa"), help="Backend GL (osmesa|egl|glfw)")
    return p


def main():
    args = build_arg_parser().parse_args()
    ensure_model_path()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    # Define combinaciones por familia (ajusta si quieres)
    P = [{"kp": k} for k in [5, 10, 20, 35, 50]]
    PD = [
        {"kp": 15, "kd": 0.2},
        {"kp": 25, "kd": 0.4},
        {"kp": 35, "kd": 0.6},
        {"kp": 45, "kd": 0.8},
        {"kp": 55, "kd": 1.0},
    ]
    PI = [
        {"kp": 5, "ki": 0.2},
        {"kp": 10, "ki": 0.4},
        {"kp": 15, "ki": 0.6},
        {"kp": 20, "ki": 0.8},
        {"kp": 25, "ki": 1.0},
    ]
    PID = [
        {"kp": 20, "kd": 0.3, "ki": 0.5},
        {"kp": 25, "kd": 0.4, "ki": 0.5},
        {"kp": 30, "kd": 0.5, "ki": 0.8},
        {"kp": 35, "kd": 0.6, "ki": 1.0},
        {"kp": 40, "kd": 0.7, "ki": 1.2},
    ]

    schedule = [("P", P), ("PD", PD), ("PI", PI), ("PID", PID)]

    env = os.environ.copy()
    env["MUJOCO_GL"] = args.gl

    manifest: List[Dict[str, str]] = []

    for family, combos in schedule:
        for i, gains in enumerate(combos, start=1):
            if family == "P":
                combo_id = f"kp{gains['kp']}"
            elif family == "PD":
                combo_id = f"kp{gains['kp']}_kd{gains['kd']}"
            elif family == "PI":
                combo_id = f"kp{gains['kp']}_ki{gains['ki']}"
            else:
                combo_id = str(i)

            csv_name = f"{family}_{combo_id}.csv"
            png_name = f"{family}_{combo_id}.png"
            csv_path = outdir / csv_name
            png_path = outdir / png_name

            cmd = [
                PYTHON, "run_mujoco_simulation.py",
                "--family", family,
                "--duration_move", str(args.duration_move),
                "--duration_hold", str(args.duration_hold),
                "--outdir", str(outdir),
                "--combo_id", combo_id,
            ]
            if "kp" in gains:
                cmd += ["--kp", str(gains["kp"])]
            if "kd" in gains:
                cmd += ["--kd", str(gains["kd"])]
            if "ki" in gains:
                cmd += ["--ki", str(gains["ki"])]
            if args.headless:
                cmd.append("--headless")
            if args.no_plot:
                cmd.append("--no_plot")

            t0 = time.time()
            rc = run_cmd(cmd, env=env)
            dt = time.time() - t0
            if rc != 0:
                print(f"[ERROR] Corrida fallida {family} {combo_id} (rc={rc})")
            else:
                print(f"[OK] {family} {combo_id} en {dt:.1f}s -> {csv_path}")
                manifest.append({
                    "family": family,
                    "combo_id": combo_id,
                    "kp": gains.get("kp"),
                    "kd": gains.get("kd"),
                    "ki": gains.get("ki"),
                    "csv_path": str(csv_path),
                })
                plot_csv(csv_path, png_path)

            time.sleep(max(0.0, args.pause))

    # Manifest y CSV maestro
    man_path = outdir / "manifest.tsv"
    with man_path.open("w", encoding="utf-8") as f:
        f.write("family\tcombo_id\tkp\tkd\tki\tcsv_path\n")
        for r in manifest:
            f.write(f"{r['family']}\t{r['combo_id']}\t{r.get('kp')}\t{r.get('kd')}\t{r.get('ki')}\t{r['csv_path']}\n")
    print("[OK] Manifest:", man_path)

    try:
        merge_all(outdir, manifest)
    except Exception as e:
        print("[WARN] No se pudo crear all_results.csv:", e)


if __name__ == "__main__":
    main()