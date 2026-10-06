#!/usr/bin/env bash
# ════════════════════════════════════════════════════════════════════════
#  check_gpu.sh — Warnt VOR dem Start, wenn die Sim-Container keine GPU
#  bekommen und MuJoCo-Viewer/RViz deshalb auf der CPU rendern (llvmpipe).
#
#  Aufgerufen von start.sh und Makefile (sim/sim-bg). Bricht NIE ab
#  (Exit-Code immer 0) -- die Sim laeuft auch ohne GPU, nur langsam.
#
#  Erkannte Faelle:
#   1. NVIDIA-GPU vorhanden, aber NVIDIA Container Toolkit fehlt
#      -> docker-compose.nvidia.yml wird uebersprungen; /dev/dri allein hilft
#         beim proprietaeren NVIDIA-Treiber NICHT (Mesa hat dafuer keinen
#         Treiber und faellt auf llvmpipe zurueck).
#   2. Toolkit installiert, aber Docker kennt die nvidia-Runtime nicht
#      (nvidia-ctk runtime configure vergessen) -> nur Hinweis.
#   3. Weder NVIDIA noch /dev/dri (z.B. WSL2 ohne GPU-Weitergabe)
#      -> reines CPU-Rendering.
#
#  Abschalten: G1_SKIP_GPU_CHECK=1
# ════════════════════════════════════════════════════════════════════════

[ "${G1_SKIP_GPU_CHECK:-0}" = "1" ] && exit 0

if [ -t 1 ]; then
  B="\033[1m"; Y="\033[33m"; RED="\033[31m"; G="\033[32m"; R="\033[0m"
else
  B=""; Y=""; RED=""; G=""; R=""
fi

has_nvidia_gpu=0
if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1; then
  has_nvidia_gpu=1
fi
has_toolkit=0
command -v nvidia-container-runtime-hook >/dev/null 2>&1 && has_toolkit=1

if [ "$has_nvidia_gpu" = "1" ] && [ "$has_toolkit" = "0" ]; then
  gpu_name=$(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -n1)
  echo -e "${RED}${B}"
  echo "╔══════════════════════════════════════════════════════════════════════╗"
  echo "║  WARNUNG: NVIDIA-GPU gefunden, aber NVIDIA Container Toolkit fehlt!  ║"
  echo "╚══════════════════════════════════════════════════════════════════════╝${R}"
  echo -e "${Y}  GPU: ${gpu_name:-unbekannt}"
  echo "  Die Sim-Container bekommen die GPU NICHT. MuJoCo-Viewer und RViz"
  echo "  rendern auf der CPU (llvmpipe) -> sehr niedrige FPS, und die CPU fehlt"
  echo "  Physik + Reglern (Roboter kann beim Laufen kippen)."
  echo ""
  echo "  Beheben (einmalig, siehe g1pilot/docs/01_installation.md):"
  echo "    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg"
  echo "    curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list"
  echo "    sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit"
  echo "    sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker"
  echo -e "  (Warnung abschalten: G1_SKIP_GPU_CHECK=1)${R}"
  echo ""
elif [ "$has_nvidia_gpu" = "1" ] && [ "$has_toolkit" = "1" ]; then
  if ! docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q nvidia; then
    echo -e "${Y}[gpu] Hinweis: NVIDIA Container Toolkit ist installiert, aber Docker"
    echo -e "      kennt die nvidia-Runtime nicht. Falls die GPU im Container fehlt:"
    echo -e "      sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker${R}"
  else
    echo -e "${G}[gpu] NVIDIA-GPU + Container Toolkit gefunden -> GPU-Rendering.${R}"
  fi
elif [ ! -e /dev/dri ]; then
  echo -e "${Y}${B}[gpu] WARNUNG: Keine GPU fuer die Container gefunden (weder NVIDIA noch"
  echo -e "      /dev/dri). MuJoCo-Viewer und RViz rendern auf der CPU -> niedrige FPS.${R}"
fi

exit 0
