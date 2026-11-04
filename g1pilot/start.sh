#!/usr/bin/env bash
# ════════════════════════════════════════════════════════════════════════
#  start.sh — Start-Menue fuer den G1-Stack (Sim UND Real).
#
#  Standard: OHNE Argumente oeffnet sich das grafische Startmenue (g1_gui.py,
#  Tkinter) — Sim/Real/Umgebungen komplett per Klick, kein Terminal noetig.
#  Die GUI ruft am Ende genau dieses Skript mit --yes + Env-Vars auf. Fehlt
#  Tkinter/Display, kommt automatisch das Text-Menue unten. Erzwingen mit
#  --menu (Text) oder G1_NO_GUI=1.
#
#  Text-Menue — gleicher Aufbau und gleiche Defaults wie die GUI.
#  Allererste Frage: SIMULATION oder ECHTER ROBOTER.
#
#  SIM  : MuJoCo + loco_sim (Whole-Body-Policy, Lockstep immer an).
#         Umgebung -> Inspire-Haende -> Lauf-Policy
#         -> Navigation -> RViz -> Rebuild.
#  REAL : Unitree-Loco-Controller (loco_client) + Arm-Manipulation +
#         Inspire-Haende via Modbus TCP. Netzwerk-Interface (Auto-
#         Erkennung) -> Haende + IPs -> RViz ->
#         Rebuild, plus SICHERHEITS-BESTAETIGUNG (der Roboter bewegt sich!).
#
#  Bedienoberflaeche ist immer die Demo-GUI (Handsteuerung eingebaut).
#
#  Alles hat sinnvolle Defaults — einfach ENTER druecken nimmt den Default.
#  Nicht-interaktiv nutzbar via Env, z.B.:
#    USE_RVIZ=true ./start.sh --yes                       # Sim
#    G1_MODE=real ROBOT_INTERFACE=enp3s0 G1_REAL_CONFIRM=1 ./start.sh --yes
#  (Im --yes-Modus startet REAL nur mit G1_REAL_CONFIRM=1.)
# ════════════════════════════════════════════════════════════════════════
set -e
cd "$(dirname "$0")"

# ── Farben (nur wenn Terminal) ──────────────────────────────────────────
if [ -t 1 ]; then
  B="\033[1m"; DIM="\033[2m"; G="\033[32m"; Y="\033[33m"; C="\033[36m"; R="\033[0m"
else
  B=""; DIM=""; G=""; Y=""; C=""; R=""
fi

ASSUME_YES=0
FORCE_MENU=0
PASSTHRU=()
for a in "$@"; do
  case "$a" in
    -y|--yes) ASSUME_YES=1 ;;        # keine Rueckfragen, Defaults/Env nehmen
    --menu|--no-gui|--tui) FORCE_MENU=1 ;;  # klassisches Text-Menue erzwingen
    *) PASSTHRU+=("$a") ;;           # Rest an `docker compose up` (z.B. --build)
  esac
done

# ── Grafisches Startmenue (Standard) ────────────────────────────────────
#  Ohne Argumente und mit erreichbarem Tkinter startet die GUI (g1_gui.py).
#  Sie sammelt alle Optionen und ruft dann ihrerseits 'start.sh --yes' auf —
#  daher wird die GUI nur im interaktiven Ur-Aufruf gezoegert. Fehlt Tkinter
#  oder ein Display, faellt es lautlos auf das Text-Menue unten zurueck.
#  Erzwingen: --menu (Text) bzw. das Setzen von G1_NO_GUI=1.
if [ "$ASSUME_YES" != "1" ] && [ "$FORCE_MENU" != "1" ] && \
   [ "${G1_NO_GUI:-0}" != "1" ] && [ "${#PASSTHRU[@]}" -eq 0 ]; then
  if command -v python3 >/dev/null 2>&1 && \
     python3 -c 'import tkinter' >/dev/null 2>&1; then
    exec python3 "$(dirname "$0")/g1_gui.py"
  fi
fi

# ── Helfer: Einzelauswahl-Menue. $1=Frage, $2=Default-Index(1-basiert),
#    $3=aktueller Env-Wert (hat im --yes-Modus Vorrang; "" = keiner),
#    danach Paare "Label|WERT". Setzt globale Variable REPLY_VALUE. ───────
ask_menu() {
  local prompt="$1"; shift
  local def="$1"; shift
  local current="$1"; shift
  local labels=() values=()
  while [ "$#" -ge 1 ]; do labels+=("${1%%|*}"); values+=("${1##*|}"); shift; done
  # Nicht-interaktiv: vorgegebener Env-Wert gewinnt, sonst Default-Index.
  if [ "$ASSUME_YES" = "1" ]; then
    if [ -n "$current" ]; then REPLY_VALUE="$current"; else REPLY_VALUE="${values[$((def-1))]}"; fi
    return
  fi
  echo -e "${B}${prompt}${R}"
  local i
  for i in "${!labels[@]}"; do
    local n=$((i+1)); local mark="  "
    [ "$n" = "$def" ] && mark="${G}*${R} "
    echo -e "   ${mark}${C}${n})${R} ${labels[$i]}"
  done
  local choice
  read -rp "$(echo -e "   ${DIM}Auswahl [${def}]:${R} ")" choice
  choice="${choice:-$def}"
  if ! [[ "$choice" =~ ^[0-9]+$ ]] || [ "$choice" -lt 1 ] || [ "$choice" -gt "${#values[@]}" ]; then
    echo -e "   ${Y}Ungueltig -> Default ${def}.${R}"; choice="$def"
  fi
  REPLY_VALUE="${values[$((choice-1))]}"
  echo
}

# ── Gemeinsame Fragen (Sim + Real) ──────────────────────────────────────
ask_rebuild() {
  ask_menu "$1) Docker-Images vor dem Start neu bauen?" 1 "" \
    "Nein — vorhandene Images nutzen (schnell)|0" \
    "Ja   — --build (nach Aenderungen an Dockerfiles/Abhaengigkeiten)|1"
  if [ "$REPLY_VALUE" = "1" ]; then PASSTHRU+=("--build"); fi
}

clear 2>/dev/null || true
echo -e "${B}╔══════════════════════════════════════════════╗${R}"
echo -e "${B}║      G1 — Start-Menue (Sim / Real)            ║${R}"
echo -e "${B}╚══════════════════════════════════════════════╝${R}"
echo -e "${DIM}ENTER = markierter Default (*).${R}\n"

# ── 0) Modus: Simulation oder echter Roboter ────────────────────────────
ask_menu "0) Was soll gestartet werden?" 1 "${G1_MODE:-}" \
  "SIMULATION — MuJoCo + Whole-Body-Policy|sim" \
  "ECHTER ROBOTER — G1 per LAN (Unitree-Loco + Arme + Haende)|real"
G1_MODE="$REPLY_VALUE"

# ════════════════════════════════════════════════════════════════════════
#  REAL-ZWEIG
# ════════════════════════════════════════════════════════════════════════
if [ "$G1_MODE" = "real" ]; then
  PROFILE=real

  # ── R1) Netzwerk-Interface zum G1 (Auto-Erkennung) ────────────────────
  #   Kandidaten: physische NICs (kein lo/docker/veth/br-). Das Interface mit
  #   einer 192.168.123.x-Adresse (Unitree-Roboter-LAN) wird als Default markiert.
  _nic_menu=() ; _def_idx=1 ; _i=0
  while read -r _nic; do
    [ -z "$_nic" ] && continue
    case "$_nic" in lo|docker*|veth*|br-*|virbr*) continue ;; esac
    _ip=$(ip -4 -o addr show "$_nic" 2>/dev/null | awk '{print $4}' | head -1)
    _i=$((_i+1))
    _nic_menu+=("${_nic}  ${_ip:-<keine IPv4>}|${_nic}")
    case "$_ip" in 192.168.123.*) _def_idx=$_i ;; esac
  done < <(ip -o link show 2>/dev/null | awk -F': ' '{print $2}' | cut -d@ -f1)
  _nic_menu+=("Anderes Interface (manuell eingeben)|__manual__")

  ask_menu "R1) Netzwerk-Interface zum G1? (Roboter-LAN = 192.168.123.x)" \
    "$_def_idx" "${ROBOT_INTERFACE:-}" "${_nic_menu[@]}"
  if [ "$REPLY_VALUE" = "__manual__" ]; then
    read -rp "$(echo -e "   ${DIM}Interface-Name:${R} ")" REPLY_VALUE
  fi
  export ROBOT_INTERFACE="$REPLY_VALUE"

  # ── R3) Inspire-FTP-Haende (Modbus TCP im Roboter-LAN) ────────────────
  ask_menu "R3) Inspire-Haende ansteuern? (Modbus TCP im Roboter-LAN)" 1 "${G1_INSPIRE_HANDS:-}" \
    "Ja   — Hand-Bridge mit Modbus-Backend|1" \
    "Nein — ohne Haende|0"
  export G1_INSPIRE_HANDS="$REPLY_VALUE"
  if [ "$G1_INSPIRE_HANDS" = "1" ] && [ "$ASSUME_YES" != "1" ]; then
    read -rp "$(echo -e "   ${DIM}IP linke Hand  [${G1_HAND_LEFT_HOST:-192.168.123.210}]:${R} ")" _l
    read -rp "$(echo -e "   ${DIM}IP rechte Hand [${G1_HAND_RIGHT_HOST:-192.168.123.211}]:${R} ")" _r
    G1_HAND_LEFT_HOST="${_l:-${G1_HAND_LEFT_HOST:-}}"
    G1_HAND_RIGHT_HOST="${_r:-${G1_HAND_RIGHT_HOST:-}}"
    echo
  fi
  export G1_HAND_LEFT_HOST="${G1_HAND_LEFT_HOST:-192.168.123.210}"
  export G1_HAND_RIGHT_HOST="${G1_HAND_RIGHT_HOST:-192.168.123.211}"

  # ── R3c) Wofuer der PS4-Controller? ───────────────────────────────────
  #   Gehen = bisheriger Weg (joystick -> loco_client, Deadman Button 8).
  #   Arme + Haende = ps4_arm_teleop, KEIN Laufen (docs/11-A-Controller.md).
  ask_menu "R3c) PS4-Controller steuert …" 1 "${G1_PS4_ARMS:-}" \
    "Gehen          — wie bisher (Deadman + Sticks -> loco_client)|0" \
    "Arme + Haende  — Steuerkreuz waehlt die Hand, KEIN Laufen|1"
  export G1_PS4_ARMS="$REPLY_VALUE"

  # ── R4) RViz ──────────────────────────────────────────────────────────
  ask_menu "R4) RViz mitstarten?" 1 "${USE_RVIZ:-}" \
    "Ja   — empfohlen: Arm-Marker + Zustand sichtbar|true" \
    "Nein — ohne RViz|false"
  export USE_RVIZ="$REPLY_VALUE"

  # ── R5) Rebuild ───────────────────────────────────────────────────────
  ask_rebuild R5

  export G1_SIM_MODE=false

  # ── Zusammenfassung + SICHERHEITS-GATE ────────────────────────────────
  echo -e "${B}\033[31m╔══════════════════════════════════════════════════════════╗${R}"
  echo -e "${B}\033[31m║  ACHTUNG: ECHTER ROBOTER — der G1 wird sich bewegen!      ║${R}"
  echo -e "${B}\033[31m╚══════════════════════════════════════════════════════════╝${R}"
  echo -e "   Interface      : ${G}ROBOT_INTERFACE=${ROBOT_INTERFACE}${R}"
  _hands_lbl=$( [ "${G1_INSPIRE_HANDS}" = "1" ] && echo "Modbus ${G1_HAND_LEFT_HOST} / ${G1_HAND_RIGHT_HOST}" || echo "aus" )
  echo -e "   Haende         : ${G}G1_INSPIRE_HANDS=${G1_INSPIRE_HANDS}${R} ${DIM}(${_hands_lbl})${R}"
  echo -e "   RViz           : ${G}USE_RVIZ=${USE_RVIZ}${R}"
  echo -e "   Geh-Limits     : ${G}vx=${G1_MAX_VX:-0.4} vy=${G1_MAX_VY:-0.3} vyaw=${G1_MAX_VYAW:-0.4}${R}"
  _ps4_lbl=$( [ "${G1_PS4_ARMS}" = "1" ] \
    && echo "Arme + Haende, kein Laufen; Tempo ${G1_PS4_SPEED:-Langsam}, max ${G1_PS4_MAX_SPEED:-Normal}" \
    || echo "Gehen mit Deadman" )
  echo -e "   PS4-Controller : ${G}G1_PS4_ARMS=${G1_PS4_ARMS}${R} ${DIM}(${_ps4_lbl})${R}"
  echo -e "   ${DIM}Kein Auto-Start: Der Roboter bewegt sich erst, wenn er in der${R}"
  echo -e "   ${DIM}Bedienoberflaeche gestartet wird (Demo-GUI: 'Start' in der Startleiste).${R}"
  echo -e "   ${Y}NOT-HALT / E-STOP: Arme schlaff, Beine bleiben stehen (kein Damp).${R}"
  echo -e "   ${Y}Voll-Notaus (Controller R1+L1+PS 2 s) = Damp = Roboter sackt in den Kran (sichern!).${R}"
  echo -e "   ${DIM}Checkliste vor dem ersten Lauf: g1pilot/docs/50-A-Echter-Roboter.md${R}"
  echo
  if [ "$ASSUME_YES" = "1" ]; then
    if [ "${G1_REAL_CONFIRM:-0}" != "1" ]; then
      echo -e "${Y}[start] REAL im --yes-Modus braucht G1_REAL_CONFIRM=1 — Abbruch.${R}"
      exit 1
    fi
  else
    read -rp "$(echo -e "   ${B}Zum Bestaetigen 'REAL' tippen (alles andere bricht ab):${R} ")" _confirm
    if [ "$_confirm" != "REAL" ]; then
      echo -e "${Y}[start] Abgebrochen — nichts gestartet.${R}"
      exit 1
    fi
  fi
  echo

# ════════════════════════════════════════════════════════════════════════
#  SIM-ZWEIG (bisheriger Ablauf)
# ════════════════════════════════════════════════════════════════════════
else
  PROFILE=sim

  # ── 1) Umgebung (G1 bleibt gleich, nur die Welt drumherum) ─────────────
  #   Umgebungen werden im scene_editor gebaut (scene_editor/scenes/*.xml).
  #   "Standard" = bisheriges Terrain (scene.xml). Die kombinierte Szene
  #   (G1 + Umgebung) wird weiter unten auf dem HOST erzeugt -- der Container
  #   kann im Repo-Mount nichts schreiben.
  _scenes_dir="../unitree_mujoco/scene_editor/scenes"
  _env_menu=("Standard (scene.xml)|__default__")
  if [ -d "$_scenes_dir" ]; then
    for _f in "$_scenes_dir"/*.xml; do
      [ -e "$_f" ] || continue
      _b=$(basename "$_f" .xml)
      _env_menu+=("$_b|$_b")
    done
  fi
  ask_menu "1) Welche Umgebung laden?" 1 "${G1_ENV:-}" "${_env_menu[@]}"
  if [ "$REPLY_VALUE" = "__default__" ]; then G1_ENV=""; else G1_ENV="$REPLY_VALUE"; fi

  # ── 3) Inspire-FTP-Haende ─────────────────────────────────────────────
  #   Master-Schalter: waehlt im MuJoCo das Inspire-Finger-Modell UND startet die
  #   Hand-Bridge (Finger steuerbar, Kraft/Taktil gemessen).
  ask_menu "3) Inspire-Haende? (Finger steuerbar + Kraftsensoren)" 1 "${G1_INSPIRE_HANDS:-}" \
    "Ja   — Inspire-Modell + Hand-Bridge|1" \
    "Nein — starre Haende (Rubber-Hand-Modell)|0"
  export G1_INSPIRE_HANDS="$REPLY_VALUE"

  # ── 3c) PS4-Controller fuer den Oberkoerper (Arme + Haende, kein Laufen) ─
  #   Siehe docs/11-A-Controller.md. Braucht /dev/input auf dem Host.
  ask_menu "3c) PS4-Controller fuer Arme + Haende? (kein Laufen)" 2 "${G1_PS4_ARMS:-}" \
    "Ja   — Controller steuert Haende (Steuerkreuz waehlt die Hand)|1" \
    "Nein — ohne Controller|0"
  export G1_PS4_ARMS="$REPLY_VALUE"

  # ── 4) Lauf-Policy von loco_sim (Ordner unter policies/) ──────────────
  #   Arme beim Laufen: g1_wholebody parkt sie in der Lauf-Pose, die beiden
  #   AGILE-Policies lassen sie frei (bringup_sim liest das aus der deploy.yaml).
  ask_menu "4) Lauf-Policy?" 1 "${G1_WALK_POLICY:-}" \
    "Alt (g1_wholebody)        — natuerlicher Gang, Arme beim Laufen geparkt|g1_wholebody" \
    "AGILE Velocity            — sehr robust, Gang mit Fangschritten, Arme frei|agile_velocity_g1" \
    "AGILE Velocity-Height     — gleichmaessiger Gang, Arme frei|agile_velocity_height_g1"
  export G1_WALK_POLICY="$REPLY_VALUE"

  # ── 5) Navigation ─────────────────────────────────────────────────────
  ask_menu "5) Navigation mitstarten? (Planer + Stationen / AUTO NAV)" 2 "${G1_ENABLE_NAV:-}" \
    "Ja   — Nav-Stack an (erzwingt RViz)|1" \
    "Nein — ohne Navigation|0"
  export G1_ENABLE_NAV="$REPLY_VALUE"

  # ── 6) RViz (bei Navigation Pflicht: Karte/Ziel-Werkzeug leben dort) ───
  if [ "$G1_ENABLE_NAV" = "1" ]; then
    export USE_RVIZ=true
  else
    ask_menu "6) RViz mitstarten? (MuJoCo-Fenster kommt immer)" 2 "${USE_RVIZ:-}" \
      "Ja   — Zusatzfenster mit TF/Markern|true" \
      "Nein — nur MuJoCo-Fenster|false"
    export USE_RVIZ="$REPLY_VALUE"
  fi

  # ── 7) Rebuild ────────────────────────────────────────────────────────
  ask_rebuild 7

  # ── Umgebung erzeugen (braucht G1_INSPIRE_HANDS, darum erst hier) ───────
  if [ -n "$G1_ENV" ]; then
    if ! python3 ../unitree_mujoco/scene_editor/build_env_scene.py \
           --env "$_scenes_dir/${G1_ENV}.xml" --inspire "${G1_INSPIRE_HANDS:-0}" >/dev/null; then
      echo -e "${Y}[start] Umgebung '${G1_ENV}' konnte nicht erzeugt werden -> Standard.${R}"
      G1_ENV=""
    fi
  fi
  export G1_ENV

  # ── Feste Sim-Voreinstellungen (Loco-Modus, Basis frei) ───────────────
  export HOLD_BASE_MODE=off            # Basis frei -> die Policy regelt den Koerper
  # Lockstep ist im Betrieb IMMER an (deterministische 50-Hz-Regelrate, PC-unabhaengig).
  export SIM_LOCKSTEP=1
  # Eine velocity-konditionierte Whole-Body-Policy macht Stehen UND Laufen: cmd=0
  # -> stehen, cmd!=0 -> laufen. Kein separater Balance-Regler mehr zu waehlen.
  # Lockstep ist auf Echtzeit gedeckelt; SIM_REALTIME_FACTOR (GUI: Sim-Tempo) setzt
  # diese Obergrenze (1.0 = Echtzeit, <1 = Zeitlupe).
  export SIM_REALTIME_FACTOR=${SIM_REALTIME_FACTOR:-1.0}
  export G1_SIM_MODE=true

  # ── Zusammenfassung ───────────────────────────────────────────────────
  echo -e "${B}Starte mit:${R}"
  _env_lbl=$( [ -n "${G1_ENV}" ] && echo "${G1_ENV} (scene_env_${G1_ENV}.xml)" || echo "Standard (scene.xml)" )
  echo -e "   Umgebung       : ${G}G1_ENV=${G1_ENV:-<default>}${R} ${DIM}(${_env_lbl})${R}"
  echo -e "   RViz           : ${G}USE_RVIZ=${USE_RVIZ}${R}"
  _hands_lbl=$( [ "${G1_INSPIRE_HANDS}" = "1" ] && echo "Inspire-FTP" || echo "Rubber-Hand" )
  echo -e "   Haende         : ${G}G1_INSPIRE_HANDS=${G1_INSPIRE_HANDS}${R} ${DIM}(${_hands_lbl})${R}"
  echo -e "   Lauf-Policy    : ${G}G1_WALK_POLICY=${G1_WALK_POLICY}${R}"
  echo -e "   Sim-Tempo      : ${G}SIM_REALTIME_FACTOR=${SIM_REALTIME_FACTOR}${R} ${DIM}(Lockstep, gedeckelt)${R}"
  _nav_lbl=$( [ "${G1_ENABLE_NAV}" = "1" ] && echo "an (dijkstra + nav2point + Sim-Glue)" || echo "aus" )
  echo -e "   Navigation     : ${G}G1_ENABLE_NAV=${G1_ENABLE_NAV}${R} ${DIM}(${_nav_lbl})${R}"
  _ps4_lbl=$( [ "${G1_PS4_ARMS}" = "1" ] && echo "an (Arme + Haende, kein Laufen)" || echo "aus" )
  echo -e "   PS4-Controller : ${G}G1_PS4_ARMS=${G1_PS4_ARMS}${R} ${DIM}(${_ps4_lbl})${R}"
  [ "${#PASSTHRU[@]}" -gt 0 ] && echo -e "   compose-Args   : ${G}${PASSTHRU[*]}${R}"
  echo
fi

# ── X11 freigeben (MuJoCo-Viewer + ggf. RViz brauchen den X-Server) ─────
xhost +local:root >/dev/null 2>&1 || \
  echo -e "${Y}[start] WARN: 'xhost' nicht verfuegbar - laeuft hier ein X-Server?${R}"

# ── GPU fuer MuJoCo-Viewer/RViz ─────────────────────────────────────────
# Sonst rendern die Container auf der CPU (llvmpipe) -> CPU fehlt Sim/Regler.
# NVIDIA (nur mit Container Toolkit) bevorzugt, sonst /dev/dri (Intel/AMD).
# Siehe docker-compose.nvidia.yml / docker-compose.gpu.yml.
if [ -z "${COMPOSE_FILE:-}" ]; then
  _cf="docker-compose.yml"
  [ -e /dev/dri ] && _cf="$_cf:docker-compose.gpu.yml"
  if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L >/dev/null 2>&1 \
     && command -v nvidia-container-runtime-hook >/dev/null 2>&1; then
    _cf="$_cf:docker-compose.nvidia.yml"
  fi
  export COMPOSE_FILE="$_cf"
  echo -e "${G}[start] Compose-Dateien: ${COMPOSE_FILE}${R}"
fi
# PS4-Controller (nur Sim): Container braucht /dev/input -> Zusatz-Datei.
if [ "$PROFILE" = "sim" ] && [ "${G1_PS4_ARMS:-0}" = "1" ]; then
  if [ -d /dev/input ]; then
    case ":$COMPOSE_FILE:" in
      *:docker-compose.ps4.yml:*) ;;
      *) export COMPOSE_FILE="$COMPOSE_FILE:docker-compose.ps4.yml" ;;
    esac
    echo -e "${G}[start] PS4-Controller: ${COMPOSE_FILE}${R}"
    ls /dev/input/by-id/*event-joystick >/dev/null 2>&1 || \
      echo -e "${Y}[start] Hinweis: noch kein Controller sichtbar -- jetzt koppeln/anstecken geht auch spaeter.${R}"
  else
    echo -e "${Y}[start] WARN: /dev/input fehlt (WSL?) -- PS4-Controller nicht verfuegbar.${R}"
  fi
fi
# Laut warnen, wenn die Sim-Container keine GPU bekommen (z.B. NVIDIA ohne
# Container Toolkit) -- sonst faellt das CPU-Rendering nur an den FPS auf.
if [ "$PROFILE" = "sim" ]; then
  bash docker/check_gpu.sh
fi
# PRIME Render Offload nur auf Hybrid-Systemen (Bildschirm an Intel/AMD).
# Treibt die NVIDIA selbst den Bildschirm, bleibt das MuJoCo-Fenster mit
# Offload schwarz. Siehe docker-compose.nvidia.yml.
if [ -z "${G1_NV_PRIME_OFFLOAD:-}" ]; then
  if nvidia-smi --query-gpu=display_active --format=csv,noheader 2>/dev/null \
       | grep -q Enabled; then
    export G1_NV_PRIME_OFFLOAD=0
  else
    export G1_NV_PRIME_OFFLOAD=1
  fi
fi

# ── Reste eines frueheren Laufs sauber entfernen ────────────────────────
docker compose --profile "$PROFILE" down --remove-orphans

# ── »Sim beenden« der Demo-GUI ──────────────────────────────────────────
# Host-Watcher: stoppt den Sim-Stack sofort, sobald die Demo-GUI die
# Trigger-Datei anlegt. $$ ist nach dem exec unten docker compose up -> der
# Watcher endet mit dem Stack. Siehe docker/sim_shutdown_watcher.sh.
if [ "$PROFILE" = "sim" ]; then
  bash docker/sim_shutdown_watcher.sh $$ &
else
  # »Real beenden« der Demo-GUI: Arme sind dann schon abgegeben; der Watcher
  # stoppt den Stack sanft (SIGTERM). Siehe docker/real_shutdown_watcher.sh.
  bash docker/real_shutdown_watcher.sh $$ "$PROFILE" &
fi

# ── Hochfahren ──────────────────────────────────────────────────────────
echo -e "${G}[start] docker compose --profile ${PROFILE} up ${PASSTHRU[*]}${R}"
exec docker compose --profile "$PROFILE" up "${PASSTHRU[@]}"
