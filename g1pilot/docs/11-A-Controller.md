# Controller (PS4/Xbox) — Anleitung

Ein PS4-Controller (DualShock 4) steuert die Hände des G1: Position und
Drehung beider Hände, Hände auf/zu, Grundstellung, Stopp und NOT-HALT.
**L2 drücken** schaltet in den Modus **GEHEN**, **R2 drücken** in den
Modus **GREIFEN** (wie die Kacheln der Demo-GUI). Gelaufen wird nur, solange
**L2 gehalten** wird (Totmann); losgelassen bleibt GEHEN, die Arme sind am
Controller dann gesperrt — siehe [GEHEN und GREIFEN](#gehen-und-greifen-l2--r2).
Am echten G1 schaltet der Controller die Beine nie selbst um (Damp,
Preparation, Walk bleiben in der GUI).

Statt des PS4-Controllers geht auch ein **Xbox-Controller** (Xbox One /
Series, z. B. „Shock Blue“) mit denselben Funktionen auf denselben
Positionen, siehe [Xbox-Controller](#xbox-controller). Er wird automatisch
erkannt.

Funktioniert in der **Simulation und am echten Roboter** (Unterschiede am
echten Roboter: siehe [Echter Roboter](#echter-roboter)). Ohne den Schalter
bleibt alles wie bisher — am echten Roboter steuert der Controller dann wie
gehabt das Gehen (Deadman, `loco_client`).

## Starten

| | Startmenü (GUI) | Text-Menü (`start.sh`) | ohne Menü |
|---|---|---|---|
| Sim | Simulation → Ausstattung → **PS4-Controller** | *3c) PS4-Controller für Arme + Hände?* → Ja | `G1_PS4_ARMS=1 ./start.sh --yes` |
| Real | Echter Roboter → Ausstattung → **PS4-Controller für Arme + Hände** | *R3c) PS4-Controller steuert …* → Arme + Hände | `G1_MODE=real G1_PS4_ARMS=1 ROBOT_INTERFACE=… G1_REAL_CONFIRM=1 ./start.sh --yes` |

Den Controller per Bluetooth koppeln oder per USB anstecken — vor oder nach
dem Start, auch ein kurzer Funkabriss ist kein Problem (er verbindet sich
selbst neu). In der Sim zuerst **R2** drücken: der Roboter steht und
die Arme gehören dem Controller (wie die Kachel GREIFEN in der Demo-GUI).
Am echten Roboter erst in der GUI starten, siehe unten.

## Belegung

![Belegung](images/ps4_belegung.svg)

| Taste | Wirkung |
|---|---|
| Steuerkreuz ← | **linke** Hand wählen (LED gelb; blau ist GEHEN) |
| Steuerkreuz → | **rechte** Hand wählen (LED grün, Standard) |
| Steuerkreuz ↑ | **beide** Hände, parallel (LED violett) — z. B. eine Kiste tragen |
| Steuerkreuz ↓ | **beide gespiegelt** (LED türkis): links/rechts und Drehungen spiegelbildlich, Bezug ist die rechte Hand — z. B. Hände auseinander/zueinander |
| Dreieck | Umschalten **VERSCHIEBEN ↔ DREHEN**. Beim Wechsel zu VERSCHIEBEN vibriert der Controller kurz, beim Wechsel zu DREHEN nicht — so ohne GUI unterscheidbar. Beim Drehen leuchtet die LED zusätzlich gedimmt |
| Linker Stick | VERSCHIEBEN: vor/zurück + links/rechts · DREHEN: kippen (Finger hoch/runter) + schwenken. Rastet auf die Achse ein (bis 20° daneben zählt nur die Hauptrichtung), erst echt schräg fahren beide Achsen |
| Rechter Stick | VERSCHIEBEN: hoch/runter (vertikal) · DREHEN: rollen (horizontal) |
| L1 / R1 | Tempo langsamer / schneller: Langsam 5 cm/s · Normal 15 cm/s · Schnell 30 cm/s (Drehen 20 / 45 / 90 °/s). Start: Sim **Normal**, Real **Langsam** (höchstens Normal). Im Modus GEHEN bzw. mit gehaltenem L2: **Geh-Tempo** (siehe [Gehen mit L2](#gehen-mit-l2)) |
| Kreis | gewählte Hand(e) **öffnen** |
| Viereck | gewählte Hand(e) **schließen** |
| Kreuz | **Stopp**: Hand hält sofort an, eine laufende geplante Bewegung (Grundstellung, Ablauf) wird abgebrochen |
| Options | **Controller sperren / entsperren** (zum Ablegen). Gesperrt lösen L2/R2, Sticks und alle Tasten nichts aus; nur **PS (NOT-HALT)** und **R1 + L1 + PS (Voll-Notaus)** wirken weiter. LED fast aus (dunkelgrau), lange Vibration beim Sperren, kurze beim Entsperren, gesperrt vibriert jede andere Taste kurz und leicht. Siehe [Controller sperren](#controller-sperren-options) |
| Share **1 s halten** | Grundstellung (Arme neben den Körper, geplant um Hindernisse herum) |
| PS | **NOT-HALT** (wie in den GUIs: Arme schlaff, Hände aus; real Taille gehalten und Beine nur StopMove, der G1 bleibt stehen; siehe [51-A-Notaus.md](51-A-Notaus.md)) |
| **Steuerkreuz ← / → + Kreuz 2 s halten** | **ARM SPIEGELN**: der Arm der gedrückten Seite fährt geplant (Planer + Kollisionsschutz) in die exakt gespiegelte Pose des anderen Arms — z. B. rechten Arm hinstellen, dann ← + Kreuz halten: der linke Arm spiegelt ihn. Controller vibriert während des Haltens, vorzeitig loslassen bricht ab. Siehe [Arm spiegeln](#arm-spiegeln-steuerkreuz--kreuz) |
| **R1 + L1 + PS 2 s halten** | **VOLL-NOTAUS**: ganzer G1 in Damp, auch die Beine. Controller vibriert während des Haltens, vorzeitig loslassen bricht ab. Siehe [Voll-Notaus](#voll-notaus-r1--l1--ps) |
| **L2 drücken** | Modus **GEHEN** (LED blau wie die GUI-Kachel GEHEN): Arm-Tasten und Sticks gesperrt, bis R2; L1/R1 = Geh-Tempo |
| **L2 halten** | **Gehen** (Totmann): Sticks, L1/R1 und Steuerkreuz ▲/▼ gehören dem Gehen, siehe [Gehen mit L2](#gehen-mit-l2) |
| **R2 drücken** | Modus **GREIFEN**: Arme übernehmen, Arm-Steuerung wieder frei. Schon im GREIFEN, Arme aber abgegeben (z. B. nach NOT-HALT): R2 übernimmt sie wieder (Sim: + stehen) |
| **R2 halten + rechter Stick ←/→** | **HÜFTE drehen** (Oberkörper links/rechts, LED weiß), Arme drehen mit. Siehe [Hüfte drehen](24-A-Hueftdrehung.md) |
| **R2 halten + R3** | Hüfte sanft zurück zur Mitte |
| **L3** (linken Stick drücken) | **Ablauf weiter**: wartet ein Ablauf „Schritt für Schritt“ der Demo-GUI auf Bestätigung, fährt er den nächsten Schritt an (wie *Weiter ▶*). Der Controller vibriert zweimal kurz, sobald ein Ablauf wartet, und kurz beim Weiter. Sonst ohne Wirkung. Lief beim Drücken ein Jog (Stick ausgelenkt), hält die Hand genau einmal an, danach kommt kein Hand-Ziel mehr. **Solange der Schritt fährt, sind beide Sticks gesperrt** (Anzeige „ABLAUF fährt (Sticks gesperrt, Kreuz = Stopp)“), sonst bräche ein versehentlich bewegter Stick die Fahrt mit „manueller Vorrang“ ab; Kreuz = Stopp geht immer. Wartet der Ablauf wieder, darf man nachkorrigieren. Nicht bei gehaltenem L2/R2 und nicht im GEHEN |
| Touchpad | frei (Reserve) |

Richtungen gelten **aus Sicht des Roboters** (vor = in Blickrichtung des
Roboters, links = Roboter-links). Steht man dem Roboter gegenüber, sind
links/rechts also vertauscht.

Loslassen = Hand steht. Die Hand läuft dem Stick höchstens ein kleines Stück
voraus (wie in der Demo-GUI); kommt der Arm nicht weiter (Gelenkgrenze,
Kollisions-Gate vor Körper/Tisch), wandert das Ziel nicht davon. Wie schnell
die Hand reagiert und wie weit sie nachläuft, hängt vom Regeltakt des
arm_controllers ab, siehe [Reaktionszeit](11-T-Controller.md#reaktionszeit).

**Rückmeldung**, welche Hand aktiv ist:
- Controller-LED in der Farbe der Auswahl (siehe Tabelle), rot bei NOT-HALT,
- kurze Vibration bei jedem Umschalten, lange bei NOT-HALT,
- in RViz eine farbige Kugel an der aktiven Hand, Display „PS4-Controller“
  (kein Text über dem Roboter, der stand im Weg; den vollen Status
  `PS4 · RECHTS · Verschieben · Normal` zeigt `ros2 topic echo /g1pilot/ps4/status`).

Hände öffnen/schließen wirkt mit **Inspire-Händen**; mit den starren Händen
passiert dabei nichts.

## GEHEN und GREIFEN (L2 / R2)

Seit 2026-10-19. Die beiden Trigger schalten den Modus wie die Kacheln
GEHEN und GREIFEN der Demo-GUI:

- **L2 drücken** → Modus **GEHEN** (`/g1pilot/start_walking`, einmal je
  Wechsel). L2 **gedrückt halten** = laufen (Totmann, siehe unten).
  **L2 loslassen** = sofort Stopp, der Modus **bleibt GEHEN**: Sticks,
  Steuerkreuz, Dreieck, L1/R1, Kreis/Viereck und Share wirken dann **nicht**
  auf die Arme (LED blau, Demo-GUI „PS4: GEHEN – L2 halten = laufen“).
  Es wirken nur Kreuz (Stopp), PS (NOT-HALT), der Voll-Notaus, L2 (laufen),
  L1/R1 (Geh-Tempo) und R2 (zurück zu GREIFEN).
- **R2 drücken** → Modus **GREIFEN**: Arme übernehmen und
  `/g1pilot/start_balancing` (Gehen aus, Roboter steht). Die Arm-Steuerung
  ist wieder frei.
- Wählt man GEHEN/GREIFEN in der Demo-GUI, folgt der
  Controller (Sperre an/aus).
- Die Trigger sind analog: gedrückt ab halbem Weg, losgelassen erst unter
  30 % (kein Flattern).
- **Echter G1:** Beide Wechsel senden ihr Beine-Topic nur, wenn der G1 laut
  `/g1pilot/onboard_state` **schon im Walk-Modus** steht (FSM 500/501).
  Sonst bleibt der Modus, wie er ist, L2 vibriert lang und das Log sagt
  „GEHEN gesperrt – G1 steht nicht im Walk-Modus“; R2 übernimmt dann nur
  die Arme. `start_walking` würde den G1 sonst aus Preparation aufstellen.
  Im Walk bleiben die Arme beim Wechsel nach GEHEN, wo sie sind (wie bei
  der Kachel GEHEN, siehe PR #10); in der Sim fahren sie je nach
  Lauf-Policy erst in die Lauf-Pose.
- Abschalten: `G1_PS4_WALK=0` (dann weder L2 noch R2).

## Controller und Demo-GUI gekoppelt

Seit 2026-10-20. Controller und Demo-GUI haben **einen gemeinsamen
Bedienzustand** (`/g1pilot/teleop_state`, `teleoperation/shared_state.py`):
was auf einer Seite gewählt wird, zeigt die andere sofort.

| Was | am Controller | in der Demo-GUI |
|---|---|---|
| Modus GEHEN / GREIFEN | L2 / R2 | Kacheln GEHEN / GREIFEN |
| Hand-Auswahl | Steuerkreuz ◀ ▶ ▲ ▼ | GREIFEN → Hand bewegen: Links / Rechts / Beide / **Gespiegelt** |
| Tempo »Hand bewegen« | L1 / R1 | Tempo Langsam / Normal / Schnell |
| Geh-Tempo | im GEHEN L1 / R1 (auch L2 halten + ▲ / ▼) | GEHEN → Tempo Langsam / Normal (Schnell nur Anzeige) |

- **Nur Anzeige und Einstellung, keine neuen Fahrbefehle.** Wer den Modus
  wechselt, sendet wie bisher `start_walking` / `start_balancing` /
  `arms/enabled`; die andere Seite zieht nur die Anzeige nach. Wechselt der
  Controller auf GEHEN, zeigt die Kachel »wechselt …«, bis die Arme in der
  Lauf-Pose sind (`arms/walk_ready`), wie beim Antippen der Kachel.
- **Sicherheit unverändert:** Totmann (L2), NOT-HALT, Voll-Notaus und
  „Real-GEHEN nur, wenn der G1 schon im Walk steht“ bleiben, wie sie waren.
  Lehnt der Controller L2 ab (real, G1 nicht im Walk), erfährt die GUI auch
  kein GEHEN. Vor »Start« (real) übernimmt die GUI keinen Modus vom
  Controller, sie zeigt nur einen Hinweis.
- **Tempo-Grenzen bleiben je Seite:** Schnell in der GUI → der Controller
  fährt real höchstens Normal (`ps4_max_speed`). Geh-Tempo Schnell gibt es nur am Controller; die GUI zeigt
  es an, geht selbst aber höchstens mit Normal (wie bisher gedeckelt).
- **Startwerte:** beim Start gelten die Parameter der Controller-Nodes
  (real: Hand-Tempo Langsam, Geh-Tempo Langsam) auch in der GUI. Die GUI
  real startet damit bei »Hand bewegen« mit Langsam statt Normal. Ohne
  Controller (`G1_PS4_ARMS=0`) bleibt alles wie bisher.
- **Controller in der GUI:** keine eigene Info-Zeile mehr (Modus, Hand und
  Tempo zeigt die GUI selbst). Solange ein Controller aktiv ist, steht oben
  rechts *🎮 Belegung* (Name im Tooltip, bei Xbox mit Tasten-Zuordnung).
- Zeitstempel je Feld, der neueste gewinnt. Ein alter Modus (vor dem Start
  des jeweiligen Programms gewählt) wird nicht übernommen.

## Hüfte drehen (R2 halten)

**R2 gedrückt halten + rechter Stick links/rechts** dreht den Oberkörper
(Gelenk `waist_yaw`), **R2 halten + R3** fährt die Hüfte sanft zur Mitte.
R2 loslassen = Hüfte steht sofort. Solange R2 gehalten wird, bewegen Sticks,
Steuerkreuz, Dreieck und L1/R1 nicht die Arme. Bereich, automatische
Rückkehr zur Mitte vor GEHEN/Grundstellung/Arme abgeben, Verhalten bei
NOT-HALT und Abschalten: [24-A-Hueftdrehung.md](24-A-Hueftdrehung.md).

## Gehen mit L2

| Mit **L2 gehalten** | Wirkung |
|---|---|
| Linker Stick | **vor/zurück** + **seitlich** gehen |
| Rechter Stick (horizontal) | **drehen** (links/rechts) |
| R1 / L1 oder Steuerkreuz ▲ / ▼ | Geh-Tempo schneller / langsamer (R1/L1 auch ohne L2, solange Modus GEHEN): Langsam 35 % · Normal 65 % · Schnell 100 % der Geh-Limits (`G1_MAX_VX` 0,4 m/s, `G1_MAX_VY` 0,3 m/s, `G1_MAX_VYAW` 0,4 rad/s). Start **Langsam** (kurze Vibration je Stufe) |
| L2 loslassen | **sofort Stopp** |

- L2 schaltet zugleich in den Modus GEHEN (siehe oben): die Arme halten
  ihre Lage und sind am Controller gesperrt, auch nach dem Loslassen.
- Gehen nur, wenn der G1 **schon im Walk-Modus** steht (Real: Startleiste
  »Walk« der Demo-GUI bzw. Handy-App; Sim: GEHEN/WALK). Sonst passiert
  nichts — L2 vibriert dann einmal lang und das Log sagt
  „G1 steht nicht im Walk-Modus“.
- Der Controller schaltet Damp/Preparation/Walk **nie** selbst um.
- Erst **scharf nach einmal Loslassen**: wird L2 beim Start, nach einem
  Funkabriss oder nach einem NOT-HALT noch gehalten, läuft der G1 nicht los.
- NOT-HALT (PS oder GUI) sperrt auch das Gehen, bis quittiert ist.
- Abriss: kommt 0,3 s kein Joy (Funkabriss, Stall-Erkennung), sendet
  `ps4_walk` einmal Stopp; `loco_client` stoppt zusätzlich selbst nach 0,5 s
  ohne Befehl, und jedes `Move()` gilt am Roboter nur 1 s.
- Abschalten: `G1_PS4_WALK=0` (L2 wieder ohne Wirkung).

## Controller sperren (Options)

Zum **Ablegen** des Controllers: **Options** (Xbox: **Menu**) drücken sperrt
ihn, nochmal Options entsperrt. Gedacht gegen versehentlich gedrückte
Trigger beim Hinlegen (L2 = GEHEN/laufen, R2 = GREIFEN/Hüfte).

- **Gesperrt wirkt nur noch:** PS (**NOT-HALT**), R1 + L1 + PS 2 s
  (**Voll-Notaus**) und Options (entsperren). L2/R2, Sticks, Steuerkreuz,
  Kreuz, Kreis/Viereck, Dreieck, L1/R1, L3 und Share tun nichts — auch das
  Gehen (`ps4_walk`) nicht. Jeder solche Tastendruck (auch L2/R2) vibriert
  kurz und leicht — so merkt man, dass der Controller gesperrt ist.
- Beim Sperren hält eine laufende Hand-Bewegung sofort an, Hüfte und Gehen
  stoppen. Eine geplante Fahrt aus der GUI (Pose, Ablauf) läuft weiter.
  Modus, Hand-Auswahl und Tempo bleiben, wie sie sind.
- **Anzeige:** LED fast aus (dunkelgrau), beim Sperren eine lange kräftige
  Vibration, beim Entsperren eine kurze leichte. Demo-GUI-Infozeile:
  „PS4: GESPERRT – Options entsperrt (PS-Notaus wirkt weiter)“. NOT-HALT
  hat Vorrang (LED rot).
- **Nach dem Entsperren** wirken ein noch ausgelenkter Stick erst, wenn er
  einmal in der Mitte war, und ein noch gedrückter L2 (Gehen) bzw. R2 (Hüfte)
  erst nach Loslassen und neuem Drücken.
- Technik: `ps4_arm_teleop` führt die Sperre und meldet sie im Feld
  `locked` auf `/g1pilot/ps4/status` (latched); `ps4_walk` folgt diesem Feld.
  Nach einem Neustart von `ps4_arm_teleop` ist der Controller entsperrt.

## NOT-HALT und Quittieren

PS löst den NOT-HALT aus — derselbe wie der Knopf in der Demo-GUI.
Die Demo-GUI zeigt ihn dann auch an (Modus aus, real erscheint wieder
»Start«). Umgekehrt
sperrt ein NOT-HALT aus der GUI auch den Controller (LED rot).

Danach ignoriert der Controller alles außer PS, bis quittiert ist: Sim in
der Demo-GUI einen Modus wählen, Real »Start«. Das Quittieren liegt absichtlich nicht auf dem Controller: in der Sim
setzt START den Roboter auch zurück, real schaltet es den Roboter in Standby.

## Voll-Notaus (R1 + L1 + PS)

Zusätzlich zum NOT-HALT (PS allein, Oberkörper) gibt es einen **kompletten
Notaus für den ganzen G1**: R1, L1 und PS gleichzeitig **2 s halten**.
Die Reihenfolge ist egal. Drückt man L1 oder R1 zuerst, verstellt das kurz
das Tempo; sobald PS dazukommt, wird dieser Tempo-Schritt zurückgenommen.
(Bis 2026-10-19 war es R1 + R2 + PS.)

1. Beim Drücken von PS greift sofort der normale NOT-HALT (Arme schlaff),
   der Controller beginnt durchgehend stark zu vibrieren und die Demo-GUI
   zeigt „PS4: VOLL-NOTAUS wird ausgelöst – loslassen bricht ab“.
2. **Vorzeitig losgelassen** (eine der drei Tasten, vor Ablauf der 2 s):
   Vibration stoppt, kein Voll-Notaus. Der PS-NOT-HALT bleibt bestehen.
   Bricht die Funkverbindung während des Haltens ab, zählt das ebenfalls als
   Abbruch; danach beginnen die 2 s von vorn.
3. **2 s gehalten**: `/g1pilot/full_emergency_stop` (danach zusätzlich
   `/g1pilot/emergency_stop`), lange Vibration (1,5 s), LED rot.
   - loco_client: **Damp (FSM 1) sofort**, unabhängig von
     `estop_damp_legs`, danach StopMove. Der G1 sackt in den Kran bzw. fällt.
   - arm_controller: Arme schlaff wie beim NOT-HALT und **Taille passiv**
     (auch mit `estop_waist_hold=true`). arm_sdk behält Gewicht 1, damit der
     Onboard-Regler die Arme nicht posiert, falls der Damp-RPC ausfällt.
   - Demo-GUI: „G1: Damp (FSM 1) · VOLL-NOTAUS – »Start« quittiert,
     dann Startleiste“.

**Zurück:** »Start« **quittiert nur** –
anders als nach einem normalen NOT-HALT schaltet der G1 danach **nicht** von
selbst nach Preparation, er bleibt in Damp. Aufstehen ausdrücklich über die
Startleiste der Demo-GUI: **Preparation** (am Kran), dann **Walk**; danach
Arme wieder mit R2/ENABLE übernehmen.

Abschalten: Parameter `full_estop_hold_s:=0` am `ps4_arm_teleop`.
In der Sim wirkt der Voll-Notaus wie der NOT-HALT (loco_sim dämpft dort ohnehin
den ganzen Roboter).

## Arm spiegeln (Steuerkreuz + Kreuz)

Seit 2026-10-21. **Steuerkreuz ← + Kreuz 2 s halten**: der **linke** Arm
fährt in die Spiegelpose des rechten. **Steuerkreuz → + Kreuz**: der
**rechte** Arm spiegelt den linken. Die Reihenfolge der beiden Tasten ist
egal (Xbox: Steuerkreuz + **A**).

1. Beim Drücken wirken die beiden Tasten zuerst wie gewohnt: das Steuerkreuz
   wählt die Seite (LED), Kreuz hält die Hände an. Dazu beginnt der
   Controller durchgehend zu vibrieren (wie beim Voll-Notaus), die Demo-GUI
   bzw. `/g1pilot/ps4/status` zeigt `mirror_arming`.
2. **Vorzeitig losgelassen** (eine der beiden Tasten, vor Ablauf der 2 s),
   Seite gewechselt oder Funkabriss: Vibration stoppt, es fährt nichts.
   Danach zählen die 2 s von vorn.
3. **2 s gehalten**: kurze Bestätigungs-Vibration, der Controller schickt
   `{"type": "mirror", "side": …}` an `/g1pilot/arm_command`. Der
   arm_controller plant die Fahrt wie *Pose anfahren* (Planer-Prozess,
   Selbst- und Umgebungskollision, Gelenklimits, sanfte Bahn ohne Ruckeln)
   und fährt sie ab. Weiter gehalten passiert nichts mehr.
4. **Stoppen** während der Fahrt: **Kreuz** (erneut drücken), **PS**
   (NOT-HALT) oder ein Stick-Ausschlag (Hand-Ziel = manueller Vorrang).
   Loslassen der Tasten nach dem Auslösen stoppt die Fahrt **nicht** — sie
   läuft wie eine Pose aus dem Positionsspeicher bis zum Ziel.
5. **Abgelehnt** (Arme nicht übernommen, NOT-HALT, WALK, Homing, Ziel
   kollidiert/Planung scheitert): lange, starke Vibration (0,6 s), Grund im
   Log und auf `/g1pilot/arm_command/status`. Am Ziel: kurze schwache
   Vibration.

**Was gespiegelt wird:** die **kommandierte** Stellung des anderen Arms (wie
der Jog-Bezug `command`, nicht die Messung — Durchhang/Schleppfehler
spiegeln nicht mit), an der Sagittalebene (Mitte des Oberkörpers, vorn/oben
bleibt, links/rechts tauscht). Gerechnet wird in Gelenkwinkeln: Pitch-Gelenke
und Ellbogen gleich, Roll- und Yaw-Gelenke mit umgekehrtem Vorzeichen. Das
ist **am URDF belegt gleichwertig** zur Spiegelung der Hand-Pose
(Position *x, −y, z*, Orientierung gespiegelt): Vorwärtskinematik über
1 600 Zufallsposen, Rest ≤ 0,01 mm (Rundung im URDF), Drehung exakt — und
zwar für die ganze Kette (Schulter, Ellbogen, Handgelenk, Greifpunkt), der
Ellbogen steht also auch spiegelbildlich (`test/test_arm_mirror.py`). Die
Gelenkgrenzen beider Arme sind ebenfalls gespiegelt, eine gültige Pose
bleibt gültig. Bei verdrehter Taille gilt die Spiegelung zum Oberkörper
(die Arme hängen am Torso); im Greifen-Modus hält die Taillen-Haltung die
Taille gerade. Die Finger (Inspire-Hände) werden nicht mitgespiegelt.

**Sicherheit:** gefahren wird nur, wenn der arm_controller die Arme schon
führt (R2/ENABLE) — das Spiegeln aktiviert sie nie selbst, auch nicht in
der Sim. Gesperrt im NOT-HALT, im Modus GEHEN und mit gehaltenem L2
(Steuerkreuz gehört dann dem Gehen). Der Voll-Notaus (R1 + L1 + PS) hat
Vorrang und bricht ein laufendes Halten ab.

Abschalten: Parameter `mirror_hold_s:=0` am `ps4_arm_teleop` (Steuerkreuz
und Kreuz wirken dann nur einzeln wie bisher).

## Xbox-Controller

Seit 2026-10-19. Ein Xbox-Wireless-Controller (Xbox One, Xbox Series X|S,
z. B. „Shock Blue“) kann alles, was der PS4-Controller kann — die Tasten
sind **nach Position** zugeordnet, man muss nichts Neues lernen. Er wird
automatisch erkannt (Name/Hersteller Microsoft); der Schalter im Startmenü
bleibt **PS4-Controller** (`G1_PS4_ARMS=1`). Ohne den Schalter (alter
`joystick`-Node, Gehen über `loco_client`) ist der Xbox-Controller nicht
vorgesehen.

![Belegung Xbox](images/xbox_belegung.svg)

| PS4 | Xbox | Wirkung (wie oben) |
|---|---|---|
| Kreuz | **A** | Stopp |
| Kreis | **B** | Hand öffnen |
| Viereck | **X** | Hand schließen |
| Dreieck | **Y** | Verschieben ↔ Drehen |
| L1 / R1 | **LB / RB** | Tempo langsamer / schneller |
| **L2 drücken / halten** | **LT drücken / halten** | Modus **GEHEN** / **Gehen** (Totmann) |
| **R2 drücken** | **RT drücken** | Modus **GREIFEN** |
| **R2 halten + rechter Stick ←/→** | **RT halten + rechter Stick ←/→** | **Hüfte drehen** |
| **R2 halten + R3** | **RT halten + R3** (rechten Stick drücken) | Hüfte zur Mitte |
| Share 1 s | **View** 1 s (linke kleine Taste) | Grundstellung |
| Options | **Menu** (rechte kleine Taste) | Controller sperren / entsperren |
| PS | **Xbox-Taste** | **NOT-HALT** (Oberkörper) |
| R1 + L1 + PS 2 s | **RB + LB + Xbox-Taste 2 s** | **VOLL-NOTAUS**, Controller vibriert beim Halten |
| Steuerkreuz ←/→ + Kreuz 2 s | **Steuerkreuz ←/→ + A 2 s** | **Arm spiegeln** (nicht Xbox-X: das ist Viereck = Hand schließen) |
| Sticks, Steuerkreuz | gleich | gleich |
| L3 | L3 (linken Stick drücken) | **Ablauf weiter** (Schritt für Schritt) |
| Touchpad | – | frei; die Share-Taste der Series-Controller (Mitte) ist frei |

**Was gleich ist:** Totmann (LT), GEHEN/GREIFEN mit LT/RT, NOT-HALT, Voll-Notaus mit 2 s Halten und
Vibration, Tempo-Stufen, Abriss-Erkennung über den Joy-Timeout, Vorrang
PS4 → Xbox: sind beide verbunden, gilt der PS4-Controller.
`G1_CONTROLLER=xbox` (oder `ps4`) erzwingt einen.

**Was anders ist:**

- **Keine LED-Farbe** (der Xbox-Controller hat keine RGB-LED). Welche Hand
  aktiv ist, zeigen RViz (Kugel an der Hand) und die Vibration.
- **Xbox-Taste:** sie kommt normalerweise an (USB: `BTN_MODE`, Bluetooth je
  nach Firmware `BTN_MODE` oder als `KEY_HOMEPAGE` auf dem Teilgerät
  „… Consumer Control“, das wird mitgelesen). Abgefangen wird sie, wenn auf
  dem Rechner **Steam** läuft (Big Picture) oder der Desktop sie belegt —
  dann den Test „Xbox-Taste = NOT-HALT“ unten unbedingt machen. Falls sie
  nicht ankommt: **`G1_XBOX_PS_BUTTON=view`** → **View** ist dann NOT-HALT
  (und mit RB + LB der Voll-Notaus), die Grundstellung wandert auf die
  **Share-Taste** in der Mitte (nur Series-Controller). Die Xbox-Taste hat
  dann keine Wirkung. Hinweis: Xbox-Taste **lange** (mehrere Sekunden) halten
  schaltet den Controller aus — für die 2 s des Voll-Notaus kein Problem; ausgeschaltet
  heißt Abriss, die Hand bzw. der G1 hält an.
- **Hängende Funkverbindung (Bluetooth):** der PS4 hat dafür das
  Sensor-Teilgerät (0,3 s). Der Xbox-Controller hat keins. Stattdessen liest
  `ps4_joystick` den rohen HID-Strom (`/dev/hidrawN`) desselben Controllers
  und nimmt ihn als Lebenszeichen — **aber nur, wenn er nachweislich auch
  ohne Eingabe-Änderung durchläuft** (20 Berichte am Stück, ohne dass sich an
  Tasten/Sticks etwas ändert; Log „HID-Strom läuft durchgehend –
  Lebendprüfung aktiv“). Sendet der Controller nur bei Änderungen, bleibt es
  ohne Lebendprüfung (sonst würde ein ruhig gehaltener Stick fälschlich
  stoppen); dann merkt der Kernel einen Funkabriss erst nach dem
  Supervision-Timeout (BlueZ-Standard 4,2 s, in `/etc/bluetooth/main.conf`
  unter `[LE]` mit `ConnectionSupervisionTimeout=50` = 0,5 s einstellbar).
  **Am echten Roboter daher USB-Kabel empfohlen** — Ziehen des Kabels meldet
  der Kernel sofort. Im Sim-Container ist `/dev/hidraw*` nicht eingebunden
  (nur `/dev/input`), dort also immer ohne diese Prüfung.
- **Treiber:** USB läuft über `xpad` (im Kernel). Bluetooth geht mit dem
  Standard-Kernel (`hid-generic`/`hid-microsoft`, Firmware des Controllers
  aktuell halten — ältere Xbox-One-S-Firmware meldet die Tasten anders) oder
  mit `xpadneo`. Die Achsen unterscheiden sich je Treiber (beim Standard-
  Bluetooth liegt der rechte Stick auf `ABS_Z/ABS_RZ`, die Trigger auf
  `ABS_BRAKE/ABS_GAS`); das erkennt `ps4_joystick` selbst. Der
  Xbox-Funkadapter (USB-Dongle) braucht den Zusatztreiber `xone`, ist aber
  nicht getestet.
- **Vibration** geht über Force-Feedback mit `xpad`, `xpadneo` und
  `hid-microsoft` (je nach Kernel). Ohne Force-Feedback fehlt sie — das ist dann
  nur Komfort, der Voll-Notaus löst trotzdem nach 2 s aus.
- Falls ein Treiber X und Y vertauscht meldet (X oben): Parameter
  `xbox_swap_xy:=true` am `ps4_joystick`.

Koppeln (Bluetooth): Controller einschalten, Pairing-Taste oben an der
Stirnseite gedrückt halten, bis die Xbox-Taste schnell blinkt, dann
`bluetoothctl` → `scan on` → `pair <MAC>` → `trust <MAC>` → `connect <MAC>`.
Log beim Verbinden: `Xbox-Controller verbunden: Xbox Wireless Controller …
-- NOT-HALT-Taste: Xbox`.

**Erster Test mit dem Xbox-Controller** (vor dem Roboter, Sim reicht):
`ros2 topic echo /g1pilot/ps4/joy` — jede Taste einmal drücken und prüfen,
dass sie am richtigen Index ankommt (Tabelle [Feste Belegung](11-T-Controller.md#feste-belegung-g1pilotps4joy));
**Xbox-Taste → `buttons[10]` = 1**, LT ganz durchgedrückt → `axes[4]` ≈ 1.
Kommt die Xbox-Taste nicht an: `G1_XBOX_PS_BUTTON=view`. Danach am Roboter
dieselben Schritte wie unter [Echter Roboter](#echter-roboter) mit den
Xbox-Tasten.

| Env-Variable | Default | Bedeutung |
|---|---|---|
| `G1_CONTROLLER` | `auto` | `auto` = PS4 oder Xbox (beide da: PS4), `ps4` / `xbox` = nur dieser |
| `G1_XBOX_PS_BUTTON` | `xbox` | `view` = View-Taste ist NOT-HALT (statt der Xbox-Taste) |

## Echter Roboter

Vorher [50-A-Echter-Roboter.md](50-A-Echter-Roboter.md) lesen.
Mit dem Schalter gilt am echten Roboter:

- **Gehen nur mit L2 gehalten** (siehe [Gehen mit L2](#gehen-mit-l2)):
  `ps4_walk` → `/g1pilot/ps4/walk_cmd` → `loco_client` → `Move()` (1 s
  Dauer, 20 Hz erneuert). Der alte `joystick`-Node startet nicht. Gehen über
  die GUI (Demo-GUI GEHEN) geht weiter; solange L2 gehalten
  wird, hat der Controller Vorrang.
- **Der Controller ändert nie den Zustand der Beine.** R2 übernimmt real ohne
  Walk-Modus nur die Arme, kein Aufstehen/Balancing; L2 geht nur, wenn der G1 schon im
  Walk-Modus steht. Starten und Stehen bleiben in der GUI.
  Einzige Ausnahme ist PS (NOT-HALT).
- **Langsamer:** Start-Tempo Langsam (5 cm/s), mit R1 höchstens Normal
  (15 cm/s). Der arm_controller begrenzt real zusätzlich auf 0,25 m/s.
- **Hüfte drehen (R2 halten):** nur mit übernommenen Armen, höchstens
  **±105°** (`G1_WAIST_YAW_LIMIT_DEG`). Die Taillen-Gains (kp 150 / kd 4) sind
  am G1 noch nicht geprüft — erster Test am Kran, erst kleine Winkel. Beim
  Drehen streichen die Hände durch den Raum (Hand 40 cm vor dem Körper →
  bei 105° rund 63 cm weit, bis seitlich hinter den Körper), dafür gibt es keine Kollisionsprüfung.
- **PS = NOT-HALT:** Arme und Hände werden sofort schlaff (sacken
  gedämpft), die Taille bleibt gehalten, die Beine bekommen nur `StopMove`
  und **kein Damp**: der G1 bleibt stehen (seit 2026-10-15, alter Stand mit
  `G1_ESTOP_DAMP_LEGS=1`). Den ganzen G1 auf Damp setzt nur der
  [Voll-Notaus](#voll-notaus-r1--l1--ps) (R1 + L1 + PS 2 s), dann sackt
  er in den Kran. Zum sanften Anhalten der Hand **Kreuz** (Hand steht,
  Roboter bleibt stehen). Übergeordnet bleibt die Unitree-Fernbedienung.
  Wer PS nur als Stopp will: `G1_PS4_PS_ESTOP=0` (dann wirkt PS wie Kreuz).
- LED-Farbe funktioniert real (Container `privileged`), in der Sim nicht.
- Hände auf/zu nur mit Inspire-Händen (Ausstattung → Inspire-Hände).
- **Seit 2026-10-13 auch real geändert** (siehe [Reaktionszeit](11-T-Controller.md#reaktionszeit)):
  arm_controller liest LowState nicht mehr blockierend im Takt, nimmt nur
  das neueste Hand-Ziel, Halte-Ziel wird 1 s wiederholt, Vorlauf nur
  entlang der Bewegung. Das gilt für Controller, Demo-GUI und Marker
  gleichermaßen: die Hand reagiert eher und läuft nach dem Loslassen
  weniger nach. Glättung und Tempo-Limits sind real unverändert
  (`smoothing_ref_hz` bleibt 0).

Ablauf:

1. Start wie gewohnt; Log zeigt `PS4-Arm-Steuerung bereit … (max. Normal,
   PS=NOT-HALT, …, Options = sperren, R2 Arme = nur Arme)` und `PS4-Controller: Daten kommen an`.
2. In der GUI **Start** (Startleiste), dann **GREIFEN** (Demo-GUI). Für die ersten Tests
   den Roboter hängend lassen und nur **R2**.
3. Steuerkreuz → Hand wählen (LED-Farbe prüfen), Stick **wenig** auslenken.
   Erwartung: Hand fährt langsam, steht beim Loslassen.
4. **Kreuz** testen (Hand steht sofort), dann Kreis/Viereck (Hände).
5. **NOT-HALT-Test nur hängend:** PS. Erwartung wie beim GUI-Knopf (Arme
   schlaff, kein Sprung). Danach Start → R2 (bzw. ENABLE
   MANIPULATION) übernimmt die Arme wieder ruckfrei.
6. **Abriss-Test:** Stick halten und dabei USB-Kabel ziehen bzw. Bluetooth
   am PC trennen (`bluetoothctl disconnect …`). Nicht PS lange drücken —
   das wäre der NOT-HALT. Erwartung: Hand steht innerhalb 0,5 s. Bei
   Bluetooth zusätzlich: Controller mit ausgelenktem Stick abschirmen bzw.
   außer Reichweite bringen (Funk hängt, ohne sauber zu trennen). Erwartung:
   Log „seit 0.3 s keine Daten“, Hand steht nach < 1 s.
7. **Gehen (Kran locker, Platz frei):** in der Startleiste »Walk«. L2 halten,
   linken Stick **wenig** nach vorn. Erwartung: G1 geht langsam, Arme bleiben
   wo sie sind; L2 loslassen → steht. Dann Drehen (rechter Stick), dann
   Tempo mit L2 + ▲. Abriss-Test wie oben mit gehaltenem L2 + Stick:
   G1 bleibt nach < 1 s stehen.

| Env-Variable (Real) | Default | Bedeutung |
|---|---|---|
| `G1_PS4_ARMS` | `0` | `1` = Controller für Arme + Hände statt fürs Gehen |
| `G1_PS4_SPEED` | `Langsam` | Start-Tempo |
| `G1_PS4_MAX_SPEED` | `Normal` | höchstes Tempo per R1 |
| `G1_PS4_PS_ESTOP` | `1` | `0` = PS nur Stopp (wie Kreuz) statt NOT-HALT |
| `G1_JOG_REFERENCE` | `command` | Bezug der Controller-/GUI-Jogs, `tf` = alter Stand (siehe [Präzise Achsen](11-T-Controller.md#präzise-achsen-2026-10-21)) |
| `G1_PS4_WALK` | `1` | `0` = kein Gehen per Controller (L2 ohne Wirkung) |
| `G1_PS4_WALK_SPEED` | `Langsam` | Start-Geh-Tempo |
| `G1_PS4_WALK_MAX_SPEED` | `Schnell` | höchstes Geh-Tempo per R1 bzw. L2 + ▲ (Schnell = volle `G1_MAX_VX/VY/VYAW`) |

## Verbindung, Container, LED

- Der Sim-Container bekommt `/dev/input` über `docker-compose.ps4.yml`
  (start.sh nimmt die Datei automatisch dazu, wenn `G1_PS4_ARMS=1`). Kein
  `privileged`; erlaubt ist nur das Öffnen von Eingabegeräten. Der
  Real-Container (`g1pilot-real`) hat `/dev/input` schon immer.
- Die **LED** wird über sysfs gesetzt. Das ist im Sim-Container schreibgeschützt
  — dann bleibt die LED wie sie ist und die Auswahl zeigen RViz und die
  Vibration. Ein Hinweis steht einmal im Log. Real (privileged) geht es.
- Gerätename: Bluetooth meldet „Wireless Controller“, USB je nach Treiber
  „Sony Interactive Entertainment Wireless Controller“ — beides wird erkannt.
  Anderer Name: `JOYSTICK_NAME=...`. Xbox-Controller („Xbox Wireless
  Controller“, „Microsoft Xbox Series S|X Controller“, „Microsoft X-Box One
  S pad“ …) werden am Namen bzw. am Hersteller Microsoft erkannt.
- Unter WSL gibt es kein `/dev/input` → der Controller ist dort nicht nutzbar
  (start.sh warnt).

## Fehlerbehebung

| Symptom | Ursache / Abhilfe |
|---|---|
| Xbox: NOT-HALT per Xbox-Taste tut nichts | Taste wird abgefangen (Steam/Desktop) → `G1_XBOX_PS_BUTTON=view`, siehe [Xbox-Controller](#xbox-controller). |
| Xbox: Tasten/Sticks vertauscht | `ros2 topic echo /g1pilot/ps4/joy` prüfen; X/Y vertauscht → `xbox_swap_xy:=true`; sonst Controller-Firmware aktualisieren (Xbox-Zubehör-App) oder `xpadneo` nehmen. |
| Log: „Kein PS4-Controller gefunden … Warte“ | Controller nicht gekoppelt oder Container ohne `/dev/input` (über start.sh starten bzw. `COMPOSE_FILE=…:docker-compose.ps4.yml`). |
| LED/Kugel zeigt die Auswahl, aber die Hand bewegt sich nicht | Arme nicht übernommen → **R2** drücken (oder Demo-GUI: GREIFEN). Im Gehen-Modus hängen die Arme in der Lauf-Pose. Real nach NOT-HALT: erst »Start«, dann R2. LED fast aus + GUI „GESPERRT“ → Controller per Options gesperrt, Options drücken. |
| Hand bleibt an einer Stelle hängen | Kollisions-Gate (Log: „Kollisions-Gate …“) oder Reichweite erreicht → in eine andere Richtung bewegen. |
| Sim: Hand reagiert spürbar verzögert / fährt nach dem Loslassen weiter | PC ausgelastet → arm_controller regelt nur mit 10–15 Hz. RViz und Navigation abschalten (`USE_RVIZ=false G1_ENABLE_NAV=0`), Last mit `top` prüfen. Siehe [Reaktionszeit](11-T-Controller.md#reaktionszeit). |
| Nach PS reagiert nichts mehr | NOT-HALT aktiv → in der GUI quittieren (Sim: Modus wählen, Real: Start). |
| L2 + Stick: G1 geht nicht, lange Vibration | G1 steht nicht im Walk-Modus → Startleiste »Walk« (Real) bzw. GEHEN (Sim). Nach NOT-HALT erst quittieren. |
| Steuerkreuz + Kreuz 2 s: lange starke Vibration, Arm fährt nicht | arm_controller hat das Spiegeln abgelehnt (Grund im Log / `/g1pilot/arm_command/status`): meist Arme nicht übernommen (**R2**), NOT-HALT oder die Spiegelpose kollidiert (z. B. Hände würden sich vor dem Bauch treffen). |
| L2 + Stick: G1 geht nicht, keine Vibration | L2 war beim Start/Verbinden schon gedrückt → einmal loslassen. Oder `G1_PS4_WALK=0`. |
