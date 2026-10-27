# Hüftdrehung — Anleitung

Richtet sich an alle, die den G1 am Tisch bedienen. Technik (Topics,
Parameter, Sim/Real): [24-T-Hueftdrehung.md](24-T-Hueftdrehung.md).
Seit 2026-10-21, Bereich seit 2026-10-23 ±105°.

## Überblick

Der G1 kann den Oberkörper über das Taillengelenk `waist_yaw` nach links oder
rechts drehen, ohne die Füße zu bewegen. Gedreht wird mit dem Controller
(PS4/Xbox), die Demo-GUI und das Handy zeigen den Stand an, die Demo-GUI hat
zusätzlich den Knopf **Hüfte Mitte**.

## Bedienung am Controller

Steht der G1 schräg vor dem Tisch, dreht man den
Oberkörper ein Stück nach links oder rechts (nur die Taille, Gelenk
`waist_yaw`), statt neu anzulaufen.

- **R2 gedrückt halten** (im Modus GREIFEN) **+ rechter Stick links/rechts**:
  Hüfte dreht, proportional zum Ausschlag, höchstens **15 °/s**. Stick
  links = nach links drehen (aus Sicht des Roboters). LED weiß, Demo-GUI
  „HÜFTE – rechter Stick ◀/▶ dreht, R3 = Mitte“.
- **R2 loslassen** = Hüfte steht sofort (Totmann wie L2 beim Gehen; bleibt
  der Strom aus, z. B. Funkabriss, hält der arm_controller nach 0,3 s an).
- **R2 halten + R3** (rechten Stick drücken): Hüfte fährt sanft zur Mitte.
  Die Demo-GUI hat dafür den Knopf **Hüfte Mitte** und zeigt den Stand
  („Hüfte 12° links“).
- Solange R2 gehalten wird, bewegen Sticks, Steuerkreuz, Dreieck und L1/R1
  **nicht** die Arme (auch kein Spiegeln). Kreis/Viereck (Hände), Kreuz (Stopp, hält auch die
  Hüfte bis der Stick zurück ist), Options (Sperre) und PS wirken weiter.
- **Bereich:** Sim und echter G1 **±105°** um die Mitte
  (`G1_WAIST_YAW_LIMIT_DEG`). Mitte = Stellung, die der Onboard-Regler beim
  Übernehmen der Arme hielt (Sim: 0).
- **Die Arme drehen mit:** sie halten ihre Gelenkstellung, die Hände bleiben
  also vor der Brust. Eine laufende geplante Fahrt (Grundstellung, Pose,
  Spiegeln) endet beim Drehen; Hand-Ziele und neue Fahrten werden
  abgelehnt, solange die Hüfte dreht.
- **Danach joggt man im Oberkörper-Bezug:** „vor“ = Richtung Brust, also zum
  Tisch hin (Controller und Demo-GUI). Bei Hüfte in der Mitte ändert sich
  nichts. (`jog_torso_frame=false` = Becken wie früher.)
- Aus GEHEN heraus wechselt R2 zuerst nach GREIFEN; die Hüfte dreht erst,
  wenn die Arme übernommen sind (real nach der Gewichtsrampe).

**Automatisch zurück zur Mitte** (sanft, 15 °/s), bevor:

| Aktion | Verhalten |
|---|---|
| **GEHEN** (L2, Kachel, Streamdeck) | erst Mitte, dann `walk_ready`; Sim: `loco_sim` wartet darauf (auch mit AGILE), real: `loco_client` sendet `Move()` nur mit 0, bis die Hüfte in der Mitte ist |
| **Grundstellung** (Share 1 s) | erst Mitte, dann die geplante Fahrt |
| **Arme abgeben** (DISABLE, real) | erst Mitte, dann Rückfahrt + Gewicht 1→0 — sonst risse der Onboard-Regler die Taille selbst zurück |

Dauert das länger als 10 s (`waist_yaw_center_timeout_s`), geht es mit
Warnung trotzdem weiter.

**NOT-HALT:** die Hüfte bleibt stehen, wo sie ist (wie bisher gehalten),
nichts dreht von allein zurück — auch nicht nach START. Zur Mitte dann mit
R2 + R3 oder dem GUI-Knopf. **Voll-Notaus:** Taille schlaff (wie bisher);
nach START wird sie an der dann gemessenen Stellung gehalten.

Abschalten: `G1_WAIST_YAW=0` (real) bzw. Parameter `waist_yaw_control:=false`
(arm_controller) / `waist_yaw:=false` (ps4_arm_teleop).

## Anzeige in Demo-GUI und Handy

- **Demo-GUI** (Modus GREIFEN, Seite ARME): Zeile „Hüfte: …“ mit dem
  gemessenen Stand, z. B. „Hüfte 12° links (max. ±105°)“, „Hüfte Mitte“,
  „Hüfte 30° rechts – dreht“ oder „… – fährt zur Mitte vor GEHEN“. Daneben
  der Knopf **Hüfte Mitte** (grau, wenn die Hüfte schon in der Mitte steht
  oder keine Daten kommen). Bei gehaltenem R2 zeigt die Info-Zeile
  „HÜFTE – rechter Stick ◀/▶ dreht, R3 = Mitte“.
- **Handy:** Die Controller-Kachel zeigt „Hüfte“, solange R2 gehalten wird.
- **»Hand bewegen«** in der Demo-GUI joggt bei gedrehter Hüfte im
  Oberkörper-Bezug wie der Controller.

## Fehlerbehebung

| Symptom | Ursache / Abhilfe |
|---|---|
| R2 + Stick dreht nichts | Arme nicht übernommen (erst R2 kurz drücken bzw. GREIFEN), NOT-HALT aktiv, Controller gesperrt (Options) oder Hüftdrehung abgeschaltet (`G1_WAIST_YAW=0`). |
| Demo-GUI „Hüfte: keine Daten“ | arm_controller läuft nicht oder sendet `/g1pilot/waist/state` nicht. |
| GEHEN dauert nach dem Drehen einige Sekunden | Die Hüfte fährt erst zur Mitte (15 °/s, bei 105° also rund 7 s), danach startet GEHEN. |
| Hand-Ziele werden abgelehnt | Solange die Hüfte dreht, nimmt der arm_controller keine Hand-Ziele und keine geplanten Fahrten an. |
