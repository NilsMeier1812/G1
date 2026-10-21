# Fenster-Icons

Eigene Icons für die G1-Fenster in Dock/Taskleiste (statt Zahnrad):

| Datei | Fenster | WM_CLASS |
|---|---|---|
| `g1_start` (grün, Play) | Start-GUI `g1_gui.py` (Tk, Host) | `g1-start` / `G1-start` |
| `g1_demo_sim` (blau, Roboter) | Demo-GUI in der Sim | `g1-demo-sim` |
| `g1_demo_real` (rot, Roboter) | Demo-GUI am echten Roboter | `g1-demo-real` |
| `g1_streamdeck` (lila, Tastenfeld) | Streamdeck `ui_interface` | `g1-streamdeck` |
| `g1_mujoco` (türkis, Würfel) | MuJoCo-Viewer | `g1-mujoco` / `MuJoCo` |

RViz bringt sein eigenes Icon mit.

Die SVGs sind die Quelle, die PNGs (256×256) sind daraus gerendert. Der
MuJoCo-Container sieht nur `unitree_mujoco/`, darum liegt `g1_mujoco.png`
zusätzlich in `unitree_mujoco/simulate_python/icons/` (Test
`test_window_icons.py` prüft, dass beide gleich sind).

Gesetzt werden die Icons in `g1pilot/utils/window_style.py`
(`set_app_identity`, Qt), `g1_gui.py` (`iconphoto`, Tk) und
`unitree_mujoco/simulate_python/window_icon.py` (`_NET_WM_ICON` per X11).
