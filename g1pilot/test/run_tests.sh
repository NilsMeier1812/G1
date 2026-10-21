#!/bin/bash
# Gezielte Testlaeufe je Bereich (Projektregel: nur passende Tests je Aenderung,
# volle Suite bei Sicherheitsrelevantem). Einteilung und Begruendung:
# /mnt/project-files/analysen/test_suite_relevanz.md
#
# Aufruf (aus g1pilot/, im Test-Container bzw. mit rclpy/pinocchio/PyQt6):
#   test/run_tests.sh <gruppe> [weitere pytest-Argumente]
# Gruppen:
#   schnell     alles ausser den drei langsamen Fake-G1-Dateien (~2 min)
#   sicherheit  Notaus, Totmann, Walk/Modus, Bodenkontakt, Arm-Controller real (~10 min)
#   voll        komplette Suite (~14 min)
#   arm | planer | loco | controller | gui | haende | sim | handy   je Bereich
# Uebersprungene Tests werden mit Grund angezeigt (-rs): fehlt z. B. pinocchio,
# skippen ganze Dateien still -- dann ist der Lauf KEIN Nachweis.
set -u
cd "$(dirname "$0")/.." || exit 2

SLOW="test_arm_controller_real test_jog_reference_real test_ik_servo_async"

case "${1:-}" in
  sicherheit) T="test_arm_controller_real test_loco_client_real test_loco_client_ps4_walk
                 test_ps4_arm_teleop test_ps4_arm_teleop_node test_ps4_walk test_ps4_walk_node
                 test_ps4_joystick test_xbox_layout test_joystick_legacy test_joy_mux
                 test_ground_contact test_status_line test_demo_gui_ground_precheck
                 test_demo_gui_temperature test_gui_external_estop test_demo_gui_real_quit
                 test_waist_hold test_arm_planner test_ik_servo_async test_inspire_modbus_backend
                 test_phone_gateway test_phone_demo_gui test_waist_yaw" ;;
  arm)        T="test_arm_controller_real test_ik_servo_async test_jog_reference_real
                 test_waist_hold test_waist_yaw test_demo_gui_jog_hold test_ps4_arm_teleop test_arm_mirror" ;;
  planer)     T="test_arm_planner test_arm_controller_real" ;;
  loco)       T="test_loco_client_real test_loco_client_ps4_walk test_ground_contact test_joy_mux test_waist_yaw
                 test_joystick_legacy test_status_line test_demo_gui_ground_precheck
                 test_demo_gui_temperature" ;;
  controller) T="test_ps4_arm_teleop test_ps4_arm_teleop_node test_ps4_joystick test_ps4_walk
                 test_ps4_walk_node test_xbox_layout test_shared_state test_loco_client_ps4_walk
                 test_demo_gui_controller_sync test_arm_mirror test_waist_yaw test_demo_gui_waist" ;;
  gui)        T="test_demo_gui_controller_sync test_demo_gui_fullscreen test_demo_gui_ground_precheck
                 test_demo_gui_jog_hold test_demo_gui_real_quit test_demo_gui_temperature
                 test_gui_external_estop test_status_line test_g1_temperature test_shared_state
                 test_window_icons test_phone_demo_gui test_demo_gui_waist" ;;
  haende)     T="test_inspire_modbus_backend" ;;
  sim)        T="test_walk_policy test_loco_sim_health test_push_charge test_sim_autostart" ;;
  handy)      T="test_phone_gateway test_phone_demo_gui test_phone_ble_dbus test_phone_protocol_js
                 test_phone_web_e2e" ;;
  schnell)    T=""
              for f in test/test_*.py; do
                n=$(basename "$f" .py)
                case " $SLOW " in *" $n "*) ;; *) T="$T $n" ;; esac
              done ;;
  voll)       T="" ;;
  *) sed -n '2,15p' "$0"; exit 2 ;;
esac
GROUP=$1
shift

FILES=()
if [ "${T// /}" = "" ]; then
  FILES=(test)
else
  for n in $T; do
    # Dateien, die es (noch) nicht gibt (z. B. Handy vor dem Merge), auslassen.
    [ -f "test/$n.py" ] && FILES+=("test/$n.py")
  done
  if [ ${#FILES[@]} -eq 0 ]; then
    echo "Keine Testdateien fuer Gruppe '$GROUP' in diesem Stand." >&2
    exit 5
  fi
fi
exec python3 -m pytest -q -rs "${FILES[@]}" "$@"
