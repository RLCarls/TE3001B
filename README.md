# SO101 Robot Simulation

Simulación y control del brazo robótico SO101 con MuJoCo y comparativa con Gazebo.

## 📁 Estructura del Proyecto

```
simulation_code/
├── model/                              # Modelos del robot
│   ├── robot_from_urdf.xml            # Modelo MuJoCo (sin escena)
│   ├── scene_urdf.xml                 # Escena completa MuJoCo
│   ├── robot_from_urdf.xml            # Alternativa URDF
│   └── assets/                         # Meshes STL del robot
│       ├── base_so101_v2.stl
│       ├── upper_arm_so101_v1.stl
│       ├── under_arm_so101_v1.stl
│       ├── sts3215_03a_v1.stl         # Servos
│       ├── wrist_roll_pitch_so101_v2.stl
│       ├── moving_jaw_so101_v1.stl    # Pinza
│       └── ... (más componentes)
│
├── so101/                              # Modelo Gazebo
│   ├── so101_gazebo.sdf               # Descripción SDF para Gazebo
│   └── assets/                         # Mismo directorio de meshes
│
├── data/                               # Datos y resultados
│   └── runs/                           # CSV generados de simulaciones
│       ├── PID_best.csv
│       ├── P_kp*.csv
│       ├── PD_kp*_kd*.csv
│       ├── PI_kp*_ki*.csv
│       └── all_results.csv            # Resumen maestro
│
├── 📄 Simulación Principal
│   ├── mujoco_simulation.py           # Simulación básica MuJoCo
│   ├── run_mujoco_simulation.py       # Simulación con grabación CSV
│   ├── run_mujoco_simulation2.py      # Versión alternativa
│   └── run_mujoco_simulation_startingpose.py  # Con pose inicial custom
│
├── 📊 Control y Análisis
│   ├── run_all_experiments.py         # Ejecuta P, PD, PI, PID automáticamente
│   ├── plot_csv.py                    # Grafica resultados CSV
│   ├── export_reference_from_csv.py   # Extrae referencias de CSV
│   └── so101_control.py               # Controlador PID
│
├── 🔧 Utilidades
│   ├── so101_mujoco_utils.py          # Funciones auxiliares MuJoCo
│   ├── so101_mujoco_utils2.py         # Plotter en tiempo real (Dash)
│   ├── so101_mujoco_pid_utils.py      # Herramientas PID
│   └── gazebo_replay.py               # Reproduce simulación en Gazebo
│
├── 🌐 Gazebo
│   ├── gazebo_replay.py               # Replay de datos en Gazebo
│   └── so101_world.world.save         # Mundo guardado de Gazebo
│
└── .venv/                             # Entorno virtual Python
```

## ⚙️ Configuración Inicial

### Instalar Dependencias

```bash
# MuJoCo (versión 3.1.0+)
pip install mujoco

# Herramientas de visualización y análisis
pip install pandas matplotlib plotly dash

# Para ROS 2 (Gazebo)
sudo apt install ros-humble-ros-gz-sim
```

### Estructura del Espacio de Trabajo

```
Control/
├── simulation_code/          # Este directorio
├── data/
└── ...
```

## 🚀 Uso

### 1. Simulación Básica en MuJoCo

```bash
cd simulation_code

# Vista del robot sin control
python mujoco_simulation.py

# Con grabación de datos
python run_mujoco_simulation.py \
    --joint target_position \
    --duration 10.0 \
    --outdir data/runs
```

### 2. Ejecutar Todos los Experimentos (P, PD, PI, PID)

```bash
python run_all_experiments.py \
    --outdir data/runs \
    --duration-move 2.0 \
    --duration-hold 2.0 \
    --headless
```

Esto genera automáticamente:
- 5+ combinaciones de cada familia de control (P, PD, PI, PID)
- CSV con trayectorias (angular, velocidad, torque)
- Gráficos PNG comparativos
- `all_results.csv` con resumen de todos

### 3. Analizar Resultados

```bash
# Graficar un CSV
python plot_csv.py data/runs/PID_best.csv

# Extraer referencia de un CSV
python export_reference_from_csv.py data/runs/PID_best.csv
```

### 4. Simulación en Gazebo (Opcional)

```bash
# Terminal 1: Lanzar Gazebo con el modelo
ros2 run ros_gz_sim create -file simulation_code/so101/so101_gazebo.sdf -name so101

# Terminal 2: Reproducir grabación
python gazebo_replay.py data/runs/PID_best.csv
```

## 📊 Archivos de Salida (CSV)

Los archivos generados contienen:

| Columna | Descripción |
|---------|-------------|
| `time` | Tiempo en segundos |
| `joint_0_ref` a `joint_5_ref` | Posiciones de referencia (6 joints) |
| `joint_0_pos` a `joint_5_pos` | Posiciones reales |
| `joint_0_vel` a `joint_5_vel` | Velocidades |
| `joint_0_torque` - `joint_5_torque` | Torques aplicados |

**Joints:**
0. `shoulder_pan` - Giro base (Z)
1. `shoulder_lift` - Levanta brazo (Y)
2. `elbow_flex` - Codo (Z)
3. `wrist_flex` - Muñeca (Z)
4. `wrist_roll` - Rotación muñeca (Z)
5. `gripper` - Pinza

## 🤖 Arquitectura del SO101

- **Base**: Base fija con motor servo
- **Shoulder Pan**: Giro horizontal (Z)
- **Shoulder Lift**: Elevación del brazo
- **Upper Arm**: Primer segmento del brazo
- **Lower Arm**: Segundo segmento (antebrazo)
- **Wrist**: Muñeca con rotación
- **Gripper**: Pinza con movimiento

### Límites de Joints

| Joint | Min (rad) | Max (rad) | Min (°) | Max (°) |
|-------|-----------|-----------|---------|---------|
| shoulder_pan | -1.920 | 1.920 | -110 | 110 |
| shoulder_lift | -1.745 | 1.745 | -100 | 100 |
| elbow_flex | -1.690 | 1.690 | -97 | 97 |
| wrist_flex | -1.658 | 1.658 | -95 | 95 |
| wrist_roll | -2.744 | 2.841 | -157 | 163 |
| gripper | -0.175 | 1.745 | -10 | 100 |

## 🎮 Scripts Principales

### `run_mujoco_simulation.py`
- Simula el robot con control PID
- Graba datos en CSV
- Permite customizar duración, joint target y parámetros

Ejemplo:
```bash
python run_mujoco_simulation.py \
    --joint 1 \
    --target-position 0.5 \
    --duration 5.0 \
    --kp 50 --ki 0.1 --kd 1.0 \
    --outdir data/runs
```

### `run_all_experiments.py`
- Automatiza ejecución de múltiples controladores
- Compara P, PD, PI, PID
- Genera análisis comparativo

### `plot_csv.py`
- Visualiza trayectorias, velocidades, errores y torques
- Exporta a PNG
- Usa matplotlib

### `gazebo_replay.py`
- Lee CSV y reproduce en Gazebo
- Permite validar en otro simulador

## 🔧 Controladores Disponibles

### P (Proporcional)
- Solo `Kp` (ganancia proporcional)
- Control básico: `u = Kp * error`

### PD (Proporcional-Derivativo)
- `Kp` y `Kd` (ganancia derivativa)
- Añade amortiguamiento: `u = Kp * error + Kd * d_error`

### PI (Proporcional-Integral)
- `Kp` y `Ki` (ganancia integral)
- Elimina offset: `u = Kp * error + Ki * integral(error)`

### PID (Proporcional-Integral-Derivativo)
- `Kp`, `Ki`, `Kd`
- Control completo: `u = Kp * error + Ki * ∫error + Kd * d_error`

## 📈 Ejemplo de Flujo Típico

```bash
# 1. Ejecutar todos los experimentos
python run_all_experiments.py --outdir data/runs --headless

# 2. Ver el mejor resultado
python plot_csv.py data/runs/PID_best.csv

# 3. Reproducir en Gazebo (opcional)
# Terminal A
ros2 run ros_gz_sim create -file so101/so101_gazebo.sdf -name so101

# Terminal B
python gazebo_replay.py data/runs/PID_best.csv
```

## 🐛 Troubleshooting

### MuJoCo no encuentra el modelo
```bash
# Asegúrate de estar en simulation_code/
pwd
# Debería mostrar: .../Control/simulation_code

# Verifica que model/scene_urdf.xml existe
ls model/scene_urdf.xml
```

### Gazebo no carga el modelo
```bash
# Verifica rutas relativas en SDF
# Deben usar model://so101/assets/...

# Re-crea el modelo
rm -rf ~/.cache/ignition
ros2 run ros_gz_sim create -file so101/so101_gazebo.sdf -name so101
```

### ROS 2 no funciona
```bash
# Activa el workspace ROS
source /opt/ros/humble/setup.bash

# O desde tu workspace personalizado
source ~/ros_ws/install/setup.bash
```

## 📝 Notas Importantes

- **MuJoCo vs Gazebo**: MuJoCo es más rápido y estable; Gazebo permite visualización mejor
- **Meshes**: Los STL están compartidos entre ambos simuladores en `assets/`
- **Datos**: Todos los CSV se guardan en `data/runs/`
- **Permisos**: Algunos scripts pueden requerir `chmod +x` para ejecutar directamente

## 📚 Recursos Útiles

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [Gazebo Documentation](https://gazebosim.org/docs/)
- [ROS 2 Humble](https://docs.ros.org/en/humble/)

## 👤 Autor

Proyecto SO101 - Robot Control Laboratory

---

**Última actualización**: Febrero 2026
