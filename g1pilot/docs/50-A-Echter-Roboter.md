# Echter Roboter — Sicherheit & Ablauf

Richtet sich an: jeden, der den Stack zum ersten Mal (oder nach einer
Software-Änderung) gegen den echten Unitree G1 startet. Dieses Dokument ist
sicherheitsrelevant — vor dem ersten Realbetrieb vollständig lesen.

Architektur in Kürze: Unterkörper = Unitrees Onboard-Loco-Controller
(`loco_client`), Oberkörper/Arme/Hände = dieser Stack. Details in
[02-T-Architektur.md](02-T-Architektur.md),
[30-T-Stehen-und-Gehen.md](30-T-Stehen-und-Gehen.md) und
[20-T-Arm-Manipulation.md](20-T-Arm-Manipulation.md).

## Kurzfassung: frischer PC → Demo-GUI am echten G1

Für Eilige. Jeder Schritt ist unten ausführlich beschrieben.

| # | Wo | Schritt | Prüfen |
|---|---|---|---|
| 1 | PC, mit Internet | Ubuntu nativ, Pakete + Docker ([Abschnitt 0](#0--neuer-pc-einmalig-mit-internet)) | `docker run --rm hello-world` ohne sudo |
| 2 | PC, mit Internet | `git clone https://github.com/NilsMeier1812/G1.git`, `cd G1/g1pilot`, `make build-real` | `docker images \| grep g1pilot-real` |
| 3 | PC | Controller per Bluetooth koppeln (optional) | `ls /dev/input/by-id/` zeigt `…event-joystick` |
| 4 | PC + G1 | LAN-Kabel, feste IP `192.168.123.222/24` ([B](#b--netzwerk-ersteinrichtung)) | `ping 192.168.123.161` |
| 5 | PC | `./start.sh` → *Echter Roboter* ([C](#c--stack-starten)) | Demo-GUI „— REAL“ geht auf |
| 6 | PC | DDS-Check ([D](#d--dds-verbindung-prüfen-bevor-irgendetwas-kommandiert-wird)) | `rt/lowstate` ~500 Hz |
| 7 | G1 am Kran | Demo-GUI: **Start** → **Preparation** → Kran lockern → **Walk** ([F](#f--ablauf-der-ersten-tests)) | Info-Zeile „G1: Walk mit Taille (FSM 501)“ |
| 8 | | Notaus kennen ([Notaus-Reflex](#notaus-reflex-auswendig-lernen)) | zweite Person mit Unitree-Fernbedienung |

## 0 · Neuer PC (einmalig, mit Internet)

Alles in diesem Abschnitt braucht Internet (apt, Docker Hub, GitHub, PyPI).
Am besten erledigen, **bevor** der PC ans Roboter-LAN kommt. Danach läuft
der Real-Stack komplett offline.

**Rechner:** nativer Linux-PC (Ubuntu 22.04 oder 24.04), freier
Ethernet-Port (USB-Ethernet-Adapter geht auch), X11-Desktop. **Kein WSL2**,
denn das Unitree-SDK braucht die Netzwerkkarte direkt, und DDS kommt durch die
WSL2-NAT nicht zuverlässig. Keine GPU nötig.

1. **Pakete und Docker** wie in [01-A-Installation.md](01-A-Installation.md)
   Schritte 1–3 (System-Pakete inkl. `python3-tk` für das grafische
   Startmenü, Docker Engine + Compose-Plugin, Docker ohne `sudo`). Danach
   einmal ab- und wieder anmelden. Für das Prüfen der Hände zusätzlich
   `sudo apt install netcat-openbsd` (für `nc`).
2. **Repository holen** (gearbeitet wird auf Branch `main`):
   ```bash
   git clone https://github.com/NilsMeier1812/G1.git
   cd G1/g1pilot
   git checkout main
   ```
   Ist das Repo privat, braucht der PC einen GitHub-Zugang (SSH-Key oder
   Personal Access Token).
3. **Real-Image bauen** (ca. 10 min):
   ```bash
   make build-real          # -> Image g1pilot-real:v1.2.0
   ```
   Das Sim-Image (`make build-sim`) wird für den echten Roboter **nicht**
   gebraucht. Ohne fertiges Image versucht der erste Start selbst zu bauen.
   Hängt der PC dann schon nur am Roboter-LAN, bricht der Build ab.
4. **Controller koppeln** (optional, PS4 oder Xbox). Bluetooth am Host:
   ```bash
   bluetoothctl
   #  power on
   #  scan on            (PS4: Share + PS halten, bis die LED blitzt)
   #  pair <MAC>
   #  trust <MAC>
   #  connect <MAC>
   ```
   Per USB-Kabel geht es ohne Koppeln. Prüfen:
   `ls /dev/input/by-id/` zeigt einen `…event-joystick`-Eintrag. Details und
   Xbox: [11-A-Controller.md](11-A-Controller.md#xbox-controller).
5. **Daten vom alten PC:**
   - Gespeicherte **Posen und Abläufe** liegen versioniert in
     `g1pilot/data/arm_poses.json`. Wurden auf dem alten PC neue Posen
     gespeichert, dort vorher committen und pushen (oder die Datei kopieren).
   - **Gekoppelte Handys** stehen in `~/.config/g1pilot/phone_devices.json`
     auf dem alten PC. Entweder die Datei kopieren oder die Handys neu
     koppeln ([12-A-Handy.md](12-A-Handy.md)).
   - Die zuletzt gewählten Startmenü-Optionen (`~/.config/g1pilot/launcher.json`)
     sind nur Komfort. Auf dem neuen PC einmal neu auswählen.

## A · Vor dem Einschalten

- [ ] Gantry/Traggurt für die ersten Balance-/Geh-Tests, oder mindestens
      2 m freie Sturzfläche in alle Richtungen.
- [ ] Eine zweite Person hält die **Unitree-Fernbedienung** bereit. Sie ist
      immer der übergeordnete Notaus, über allem anderen.
- [ ] E-Stop-Semantik verstanden (siehe Abschnitt „Notaus“ unten).
- [ ] Akku geladen, keine Kabel/Hindernisse in Fußnähe.
- [ ] Erste Arm-Tests am hängenden/knienden Roboter, nicht am stehenden.

## B · Netzwerk (Ersteinrichtung)

| Adresse | Wer |
|---|---|
| `192.168.123.161` | G1-Onboard-PC (DDS-Quelle) |
| `192.168.123.210` / `.211` | Inspire-Hand links/rechts (Modbus TCP 6000) |
| `192.168.123.222` | eigener PC (selbst vergeben, Konvention; jede freie `.x` geht) |

1. G1 einschalten, per Ethernet-Kabel mit dem PC verbinden.
2. Interface-Namen finden: `ip -br link` (z. B. `enp3s0`, `enx…` bei
   USB-Adaptern, `eth0`; `lo`, `wl…`/WLAN und `docker0` ignorieren). Das
   richtige Interface ist das, das beim Einstecken des Kabels auf `UP`
   springt.
3. Statische IP **dauerhaft** über NetworkManager vergeben. Kein Gateway und
   kein DNS eintragen, sonst versucht der PC, über das Roboter-LAN ins
   Internet zu gehen. Das WLAN bleibt fürs Internet nutzbar.
   ```bash
   sudo nmcli con add type ethernet ifname <NIC> con-name g1-robot \
        ipv4.method manual ipv4.addresses 192.168.123.222/24 \
        ipv4.never-default yes ipv6.method disabled
   sudo nmcli con up g1-robot
   ip -br addr show <NIC>      # muss 192.168.123.222/24 zeigen
   ```
   Nur zum schnellen Ausprobieren geht auch
   `sudo ip addr add 192.168.123.222/24 dev <NIC> && sudo ip link set <NIC> up`.
   Diese Adresse überlebt aber kein Abziehen des Kabels und keinen Neustart,
   und NetworkManager kann sie wieder entfernen.
4. Erreichbarkeit prüfen:
   ```bash
   ping -c3 192.168.123.161
   ping -c3 192.168.123.210   # nur mit Inspire-Händen
   nc -zv 192.168.123.210 6000
   nc -zv 192.168.123.211 6000
   ```
   Ohne Ping zum Roboter hat nichts Weiteres Sinn. Erst Kabel, IP und
   Interface prüfen, dann weitermachen.
5. Firewall beachten: `sudo ufw status`. Ist sie aktiv, den DDS-UDP-Verkehr
   auf dem Roboter-Interface erlauben (`sudo ufw allow in on <NIC>`).

Die Interface-Auswahl im Startmenü markiert die NIC mit
`192.168.123.x`-Adresse automatisch als Vorschlag und warnt, wenn das
gewählte Interface keine solche Adresse hat.

**Wie die beiden DDS-Netze zusammenspielen (nichts einzustellen):** Im
Container laufen zwei DDS-Teilnehmer nebeneinander. Das Unitree-SDK spricht
auf **Domain 0** über das gewählte `ROBOT_INTERFACE` mit dem G1. Der
ROS-2-Graph der eigenen Nodes läuft auf **Domain 1** und nur über Loopback
(`docker/cyclonedds.xml`, kein Multicast). Beides setzt Compose bzw. das
Real-Image selbst. Eine eigene `CYCLONEDDS_URI` oder `ROS_DOMAIN_ID` auf dem
Host ist nicht nötig, der Container nimmt ohnehin seine eigenen Werte.

## C · Stack starten

```bash
cd G1/g1pilot && ./start.sh
```

Im grafischen Startmenü **Echter Roboter** wählen:

| Abschnitt | Einstellung |
|---|---|
| Verbindung zum G1 | Interface mit `192.168.123.x` (wird vorgeschlagen, grüner Hinweis „Im Roboter-LAN“) |
| Ausstattung | Inspire-Hände + IPs; RViz; **PS4-Controller für Arme + Hände** (PS4 oder Xbox, siehe unten); Handy-Zugang |
| Geh-Limits | Default vorwärts 0,4 m/s, seitlich 0,3 m/s, drehen 0,4 rad/s |
| Erweitert | Modbus-Port, Gamepad-Name, Images neu bauen |

Danach den Haken „Sicherheitshinweise gelesen“ setzen und **Echten Roboter
starten**. Die Docker-Ausgabe läuft im Startmenü-Fenster mit.

**Was der Controller-Schalter bedeutet:**

- **an** (`G1_PS4_ARMS=1`, empfohlen mit Demo-GUI): Controller steuert Arme
  und Hände, **L2 halten + Sticks = Gehen** (Totmann), PS4 oder Xbox.
  Siehe [11-A-Controller.md](11-A-Controller.md).
- **aus** (`G1_PS4_ARMS=0`): alter Weg, nur PS4. Der Controller steuert
  ausschließlich das Gehen (Deadman Button 8 + Sticks), keine Arme.

Ohne grafisches Menü (kein `python3-tk`, SSH): `./start.sh --menu` fragt
dasselbe im Terminal ab und verlangt zum Schluss die Eingabe `REAL`. Ganz
ohne Rückfragen:
```bash
G1_MODE=real ROBOT_INTERFACE=<NIC> G1_INSPIRE_HANDS=1 \
  G1_PS4_ARMS=1 G1_REAL_CONFIRM=1 ./start.sh --yes
```
`make real ROBOT_INTERFACE=<NIC>` geht auch, nimmt aber die Compose-Defaults
(u. a. ohne Hände), solange die Variablen nicht
gesetzt sind.

Erwartung: Container startet, `colcon build` läuft durch, RViz und die gewählte Bedienoberfläche (Titel „— REAL")
öffnen sich. **Der Roboter tut nichts von selbst.**

## D · DDS-Verbindung prüfen (bevor irgendetwas kommandiert wird)

In einem zweiten Terminal, während der Stack läuft:

```bash
cd G1/g1pilot
docker compose --profile real exec g1pilot-real \
  bash -ic 'ros2 run g1pilot dds_check --interface $ROBOT_INTERFACE'
```

(`make shell-real` öffnet eine Shell im Container; dort reicht
`ros2 run g1pilot dds_check --interface $ROBOT_INTERFACE`.)

Erwartung: `rt/lowstate` kommt mit ~500 Hz, `mode_machine` ≠ 0, die
Arm-Gelenkwinkel ändern sich plausibel, wenn die Arme von Hand bewegt
werden. Ohne `lowstate`: falsches Interface, Roboter nicht im
192.168.123.x-Netz oder Kabelproblem. Dann nicht weitermachen.

Weitere Lebenszeichen ohne Extra-Werkzeug: Die Demo-GUI zeigt in der
Startleiste den Onboard-Modus („G1: Damp (FSM 1)“ o. ä.). Steht dort
„keine Daten vom loco_client“ oder „Modus unbekannt (keine Antwort vom
Roboter)“, kommt `loco_client` nicht an den G1. Mit Inspire-Händen darf
keine Hand „nicht verbunden“ zeigen.

## E · Guarantees, die bereits im Code stehen

- **Kein Auto-Damp beim Start:** `loco_client` läuft mit `damp_on_init=False`
  — der Node greift beim Hochfahren nicht in den Roboterzustand ein.
- **Kein Auto-Start:** Die Demo-GUI publiziert im Real-Modus von sich aus
  nichts — der Roboter bewegt sich erst nach Klicks (der Auto-Start
  `SimAutoStart` läuft nur in der Simulation, siehe
  [40-T-Simulation.md](40-T-Simulation.md)).
- **WALK ist gesperrt, bis balanciert:** wird sonst mit Warnung ignoriert.
- **Übernehmen ohne Bewegung (`ENABLE`):** `arm_controller` merkt sich die
  gemessene Armpose (= Pose des Onboard-Reglers), kommandiert genau diese und
  rampt das `arm_sdk`-Gewicht 0→1 über 2,0 s. Die Gelenke werden exakt
  gehalten, bis ein Ziel kommt (kein Nachdrehen des Ellbogens durch die IK).
- **Taille wird mitgehalten (seit 2026-10-14):** das `arm_sdk`-Gewicht
  schaltet beim G1 nicht nur die Arme, sondern auch die Taille (Waist
  yaw/roll/pitch, Gelenke 12-14) vom Onboard-Regler weg — so steht es auch in
  den Unitree-Beispielen. Früher bekam die Taille dabei kp=0: Oberkörper
  kraftlos, kippte nach vorn/hinten. Jetzt hält `arm_controller` die Taille in
  der beim `ENABLE` gemessenen Stellung (kp 150 / kd 4 + Schwerkraft-
  Feedforward, `G1_WAIST_KP` / `G1_WAIST_KD`, Obergrenze 250 / 6). Im E-Stop
  bleibt sie **gehalten** (`estop_waist_hold`, Env `G1_ESTOP_WAIST_HOLD`,
  Default `true`; lief der Onboard-Regler vorher allein, wird sie an der
  gemessenen Stellung gehalten). Nur mit `false` ist sie wie früher passiv
  (START packt sie dann weich wieder: kp-Rampe 1,5 s, ab der gemessenen
  Stellung, ≤ 0,3 rad/s zurück). Grund: Real-Test 2026-10-15 im Walk-Modus —
  schlaffe Taille + balancierender Walker = Oberkörper kippt, G1 rennt los. Gesperrte Taille
  (23/29 DoF locked): Parameter `waist_joints` auf `[12]`.
- **Zurückgeben ohne Schwung (`DISABLE`):** die Arme fahren erst geplant und
  langsam in die beim `ENABLE` gemerkte Onboard-Pose; erst dort rampt das
  Gewicht 1→0. Die Überblendung hat also keinen Positionsunterschied mehr.
  Während der Rückfahrt werden neue Ziele, Homing und WALK abgelehnt;
  `ENABLE` bricht die Rückfahrt ab (Arme halten), EMERGENCY STOP macht sie
  sofort schlaff. Zeitlimit 12 s, danach Rampe aus der erreichten Pose.
- **Programmende (Strg+C, `docker compose stop`, Absturz):** `arm_controller`
  fährt die Arme bis zu 2,5 s langsam (≤ 0,8 rad/s) Richtung Onboard-Pose und
  rampt dann das Gewicht 1→0, statt den `arm_sdk`-Strom abzureißen. Im
  E-Stop nur Rampe (Arme bleiben schlaff). Compose startet `ros2 launch`
  per `exec` mit 20 s Gnadenfrist, damit `docker stop` dafür Zeit hat.
- **Harte Grenzen nur auf real:** Gains höchstens kp 80/kd 3 (Handgelenk
  kp 50), Gewichtsrampen ≥ 1 s, Gelenkgeschwindigkeit ≤ 1,5 rad/s,
  Hand ≤ 0,25 m/s. Falsche Werte aus einem Launchfile werden mit
  Fehlermeldung korrigiert.
- **Sanftes Anfahren und begrenzter Schleppfehler (real):** Gelenk-
  beschleunigung ≤ 3 rad/s² (`G1_ARM_ACCEL_LIMIT`), Sollwert höchstens
  0,3 rad von der Messung entfernt (`G1_ARM_MAX_TRACKING_ERROR`) — wird ein
  Arm festgehalten, „lädt" sich kein Sprung auf, der beim Loslassen schnappt.
- **Ein Ziel ersetzt das andere:** setzt PS4, Demo-GUI oder ein Positions-
  speicher ein neues Ziel, bekommt ein offenes RViz-Marker-Ziel „cancelled";
  ein späterer Klick auf den alten Marker zieht den Arm nicht zurück.
- **Hände frieren beim Abschalten ein:** Hand deaktiviert (auch per E-Stop)
  → Soll = aktuelle Ist-Stellung, die Finger schließen nicht weiter.

## F · Ablauf der ersten Tests

### Mit der Demo-GUI (Normalfall)

Der G1 hängt am Kran, die zweite Person hat die Unitree-Fernbedienung.

1. G1 einschalten und warten, bis er in Damp liegt. Die Startleiste der
   Demo-GUI zeigt „G1: Damp (FSM 1)“.
2. **Start** klicken. Das gibt die Modus-Kacheln frei und quittiert einen
   alten NOT-HALT. Liegt der G1 noch in Damp, stellt `loco_client` ihn dabei
   auf **Preparation** (FSM 4, Gelenke werden steif).
3. **Arme zuerst, noch hängend:** Kachel **GREIFEN** übernimmt in
   Preparation nur die Arme (2-s-Rampe, kein Ruck). Hand mit der GUI oder dem
   Controller wenig bewegen, dann den NOT-HALT einmal testen (Arme sacken
   gedämpft) und mit **Start** quittieren.
4. **Kran lockern**, bis die Füße tragen. Die Startleiste zeigt dann
   „Bodenkontakt ✓ – »Walk« möglich“ mit der gemessenen Knie-Last.
5. **Walk** in der Startleiste, zweiter Klick bestätigt „Kran gelockert?“.
   Info-Zeile: „G1: Walk mit Taille (FSM 501)“.
6. **GEHEN**: mit dem Joystick der GUI oder mit dem Controller (L2 halten +
   linker Stick) erst ganz wenig. Loslassen hält an.
7. Ende: **Real beenden** zweimal (siehe [G](#g--sicher-beenden)).

Die Einzelschritte und die Hintergründe stehen in den folgenden
Unterpunkten.

### Einzeltests

**1. Hände (Roboter bleibt passiv, hängend/schlaff)**

In der Demo-GUI die linke Hand öffnen/schließen (Hände-Bereich in
GREIFEN). Erwartung: RViz spiegelt die Finger, die
Viewer-GUI zeigt plausible Kräfte bei Druck.

**2. Arme passiv beobachten**

Arme von Hand bewegen (Manipulation nicht aktiviert). RViz-Modell folgt
exakt; die Marker folgen der Hand (Leader-Follower).

**3. Arm-Manipulation (hängend oder kniend)**

1. **GREIFEN** (übernimmt die Arme, intern `arms/enabled` = ENABLE) —
   Gewicht rampt 2 s von 0 auf 1; die Arme übernehmen die aktuelle Pose
   ohne Ruck. Die beim ENABLE gemessene Pose ist die, in die das Abgeben
   später zurückfährt.
   **Taille prüfen (hängend):** nach der Rampe den Oberkörper an der
   Schulter leicht vor/zurück und seitlich drücken — er muss federnd
   zurückkommen und darf nicht mehr frei kippen. Log: „Taille [12, 13, 14]
   wird bei arm_sdk-Gewicht > 0 gehalten".
2. In RViz einen Marker wenige Zentimeter ziehen — Hand folgt ruhig,
   max. 0,25 m/s.
3. **Gate-Test:** Marker absichtlich in den Torso ziehen. Erwartung: Arm
   stoppt vor dem Körper (Log: „Selbstkollisions-Gate ..."). Marker
   zurückziehen — Arm folgt wieder.
4. **Grundstellung** testen (nur bei übernommenen Armen möglich).
5. **E-Stop-Test (hängend!):** **NOT-HALT** drücken. Erwartung: Arme
   werden **sofort** drehmomentfrei und sacken gedämpft — kein Sprung in
   eine Home-Pose. Danach Startleiste **NOT-HALT quittieren** (Arme bleiben
   schlaff) → **GREIFEN** übernimmt wieder ruckfrei die aktuelle Pose. Fährt der Arm beim E-Stop stattdessen aktiv irgendwohin, sofort
   die Fernbedienungs-Notabschaltung nutzen und den Vorfall melden.
6. **Arme abgeben** (*Real beenden* macht das als ersten Schritt, einzeln:
   `ros2 topic pub --once /g1pilot/arms/enabled std_msgs/Bool "data: false"`)
   — die Arme fahren erst langsam in die Pose vom ENABLE zurück, dann rampt das Gewicht 2 s auf 0 und die
   Roboter-eigene Armsteuerung übernimmt ohne Positionssprung.
7. **Programmende testen (hängend):** Arm mit einem Ziel vom Körper weg
   fahren, dann `Strg+C` bzw. `docker compose stop`. Erwartung: Log
   „Programmende: Arme fahren langsam in die Onboard-Pose …" und „Arme an
   den Onboard-Regler uebergeben", kein Schwung.
8. Optional **PS4-/Xbox-Controller für Arme + Hände** (Start mit
   `G1_PS4_ARMS=1`, Startmenü: Ausstattung): Ablauf in
   [11-A-Controller.md](11-A-Controller.md#echter-roboter). In diesem
   Modus geht das Gehen über **L2 halten + Sticks** (`ps4_walk`), nicht über
   den alten Deadman-Button 8 aus Schritt 5.

Regelrate-Hinweis: Während ein Marker-Ziel aktiv verfolgt wird, kostet der
IK-Solve ~20-30 ms/Zyklus — der Regel-Loop läuft dann effektiv mit
~30-50 Hz statt 250 Hz. Alle Limits rechnen mit dem echten Zeitschritt und
bleiben korrekt.

**4. Balance**

Roboter gesichert am Boden (Gantry). **START** → FSM 4 (Standby). **START
BALANCING** → Höhenrampe + Balance-Start. Erwartung: Roboter steht
selbstständig und balanciert; leichtes Anstupsen wird ausgeglichen.

Firmware-Vorbehalt: Die FSM-ID-Sequenz stammt aus dem ursprünglichen
g1pilot-Projekt. Bei neuerer G1-Firmware ggf. andere FSM-IDs — dann hier
abbrechen und die IDs in `loco_client.py` gegen die aktuelle Unitree-Doku
prüfen.

**5. Gehen**

Zuerst über den **Controller** (physischer Totmann), dann über die GUI:

1. Controller verbinden. **Mit `G1_PS4_ARMS=1`** (Controller für Arme +
   Hände): **L2 halten** (Xbox: LT) + linker Stick geht, rechter Stick dreht; L2
   loslassen stoppt sofort. L2 wirkt nur, wenn der G1 schon in Walk steht
   ([11-A-Controller.md](11-A-Controller.md#gehen-mit-l2)).
   **Mit `G1_PS4_ARMS=0`** (alter Weg, nur PS4): Gerätename passt zu
   `JOYSTICK_NAME`, Log „Joystick found"; Deadman (Button 8) halten + Stick
   bewegt den Roboter, loslassen stoppt.
2. Demo-GUI **GEHEN** (nur wenn die Startleiste Walk zeigt), dann den virtuellen Joystick
   vorsichtig ziehen.
3. **Deadman-Test (Pflicht):** Während der Roboter geht, das
   GUI-Fenster schließen. Erwartung: Roboter stoppt von selbst
   innerhalb ≤ 0,5 s und bleibt balanciert stehen. Läuft er weiter, ist das
   ein Fehler — sofort melden.
4. Geh-Limits konservativ lassen (Default vx=0.4/vy=0.3/vyaw=0.4 m·rad/s,
   über `G1_MAX_VX/VY/VYAW` anpassbar).

## G · Sicher beenden

Am einfachsten über die Demo-GUI: **Real beenden** zweimal tippen. Die GUI
erledigt dann alles der Reihe nach:

1. Sie stoppt das Gehen (StopMove) und alle laufenden Arm-Ziele.
2. Sie schaltet die Arme aus. Die Arme fahren langsam in die Pose vom
   ENABLE zurück und werden weich an den Roboter übergeben.
3. Sie stoppt den Stack sanft.

Von Hand geht es so: Gehen beenden, dann die Arme abgeben (`arms/enabled` False, siehe oben) und auf das
Log „Arme an den Onboard-Regler uebergeben“ warten, dann `Strg+C` bzw.
`make stop`.

Die Beine bekommen beim Beenden **kein** Damp. Der Roboter bleibt im
Onboard-Zustand stehen. Danach ablegen bzw. dämpfen über die
Unitree-Fernbedienung, gesichert.

## Notaus-Reflex (auswendig lernen)

Alle Halt-Wege mit Quittieren und Einstellungen: [51-A-Notaus.md](51-A-Notaus.md).

| Situation | Richtige Reaktion |
|---|---|
| Sanft anhalten beim Gehen | Joystick bzw. **L2 loslassen** (Roboter bleibt stehen) |
| Arm fährt falsch, aber ruhig | Arme abgeben (Demo-GUI GREIFEN verlassen, Arme fahren langsam in die Onboard-Pose, dann Übergabe) |
| Ruckt/schwingt an den Armen, Kontakt mit Mensch/Objekt | **NOT-HALT** (Demo-GUI) / **PS** bzw. Xbox-Taste am Controller (Arme/Hände sofort schlaff; Taille gehalten; Beine: StopMove, Onboard-Regler balanciert weiter) |
| Hand fährt per Controller falsch | Stick loslassen bzw. **Kreuz** / Xbox **A** (Hand steht sofort, Roboter bleibt stehen) |
| G1 soll sofort ganz aus (Beine weich) | **Voll-Notaus am Controller: R1 + L1 + PS 2 s halten** (Xbox: RB + LB + Xbox-Taste) → ganzer G1 Damp, sackt in den Kran. Quittieren mit »Start« (bleibt in Damp), dann Startleiste Preparation → Walk. Siehe [11-A-Controller.md](11-A-Controller.md#voll-notaus-r1--l1--ps) |
| Echte Gefahr / Sturz droht | **Unitree-Fernbedienung** (übergeordnet, Damp dort). Der normale NOT-HALT dämpft die Beine **nicht** (außer `G1_ESTOP_DAMP_LEGS=1`) |

Was EMERGENCY STOP konkret bewirkt (Stand 2026-10-15): Arme werden
drehmomentfrei und sacken gedämpft, ohne in eine Home-Pose zu springen (dazu
bleibt das `arm_sdk`-Gewicht bewusst auf 1 — würde es auf 0 gesetzt, übernähme
der Onboard-Regler die Arme und führe sie aktiv in seine Default-Pose); die
**Taille bleibt gehalten**, damit der Oberkörper nicht kippt; Hände werden
deaktiviert und halten ihre Position. Die Beine bekommen `StopMove()` (immer,
auch wenn gerade kein Move lief) und **kein** `Damp()`: der Onboard-Regler
balanciert den G1 stehend weiter. Der alte Stand (Beine `Damp()`, Taille
schlaff) lässt sich mit `G1_ESTOP_DAMP_LEGS=1` bzw. `G1_ESTOP_WAIST_HOLD=false`
zurückholen. Die Antwort-Codes der Roboter-RPCs stehen im Log
(„EMERGENCY STOP: StopMove (Code 0) …“). Quittieren: Startleiste **NOT-HALT quittieren** — war der G1
dabei im Walk-Zustand, bleibt er darin (kein Sprung nach FSM 4). Die Arme
bleiben danach gedämpft schlaff, bis **GREIFEN** (bzw. Controller **R2**) die Kontrolle
bewusst zurückholt.

## Handy-App / Fernbedienung und GUI (Modus-Abgleich)

`loco_client` liest alle 0,5 s die Onboard-FSM-ID (`state_sync_hz`). Stellt die
Unitree-Handy-App oder die Fernbedienung den G1 auf **Walk** (FSM 500/501,
Parameter `walk_fsm_ids`), übernimmt der Node das als „balanciert“ — **LAUFEN**
in der GUI bewegt den G1 dann sofort. Steht der G1 in **Preparation** (FSM 4),
stellt **LAUFEN** ihn selbst auf Walk (Höhenrampe + `BalanceStand` + `Start`,
wie GREIFEN). Aus Damp/Zero-Torque/Sitzen steht der G1 nie von selbst auf
(Log „WALK ignoriert … FSM n“). Verlässt der G1 den Walk-Zustand (z. B. Damp per
Fernbedienung), sperrt der Node das Gehen (2 Polls + 3 s Schonfrist). `Move()`
läuft nur noch mit begrenzter Dauer (`move_duration_s`, 1 s, alle 50 ms
erneuert): stirbt der Node, stoppt der Onboard-Regler selbst. Bleibt der
Joy-Strom (PS4/Auto-Nav) bei gedrücktem Deadman aus, gilt `cmd_vel_timeout`
(0,5 s) → StopMove. FSM-ID 501 ist aus dem Repo nicht belegt.

Seit 2026-10-16 (Review des Real-Ablaufs):

- **START / »Roboter starten« schaltet nur noch aus Damp/Zero-Torque/Sitzen
  nach FSM 4.** Steht der G1 schon in Walk (z. B. per Handy-App) oder in
  Preparation, wird nur übernommen — früher sprang ein per App balancierender
  G1 beim Klick zurück nach FSM 4. Ist der Modus nicht lesbar, schaltet START
  nicht blind um (Log „Onboard-Modus nicht lesbar"). Gilt auch für
  Steuerkreuz-oben am alten Joystick.
- **GREIFEN (`start_balancing`) stellt nur noch aus Preparation (FSM 4) auf
  Walk um**, wie LAUFEN. Aus Damp/Zero-Torque/Sitzen und im E-Stop: Log
  „START BALANCING ignoriert …".
- **Höhenrampe bricht ohne `Start()` ab**, wenn der FSM-Mode ≠ 0 meldet (Füße
  nicht belastet, z. B. hängend) oder keine Antwort kommt. Früher kam danach
  trotzdem `BalanceStand` + `Start()` (Walk-Regler mit Füßen in der Luft).
- **`balance_mode`** (Parameter, Default 1 wie bisher): Wert für
  `SetBalanceMode` beim Umschalten. Laut Unitree-SDK ist 0 = BalanceStand
  (steht still), 1 = ContinuousGait (tritt auf der Stelle). Am Gantry prüfen,
  was der G1 nach GREIFEN aus Preparation tut.
- **Demo-GUI zeigt den Onboard-Modus** („G1: Walk (FSM 500) · bereit",
  Topic `/g1pilot/onboard_state`) und — im PS4-Arm-Modus — die gewählte Hand,
  Verschieben/Drehen und das Tempo. Klick auf die PS4-Anzeige zeigt die Belegung.
- **Startleiste in der Demo-GUI (nur real): Damp · Preparation · Walk** —
  der Start am Kran ohne Handy-App. Ablauf: G1 einschalten → **Damp** →
  **Preparation** → Kran lockern → **Walk** (zweiter Klick: „Kran gelockert?").
  Die Knöpfe sind nur freigegeben, wo der Schritt passt (Preparation nur aus
  Damp/Walk, Walk nur aus Preparation, beides nicht im NOT-HALT); **Damp** aus
  dem Stehen braucht ebenfalls einen zweiten Klick (der G1 sackt in den Kran).
  `loco_client` prüft jede Anforderung (`/g1pilot/loco_request`) noch einmal.
  **Walk** = `BalanceStand` + `Start()` wie die App, **ohne Höhenrampe**, und
  nur wenn der FSM-Mode 0 meldet (Füße belastet). Hängt der G1 noch straff,
  kommt „Walk: abgelehnt — Füße ohne Last … Kran lockern" in der Statuszeile.
  Die alte Rampe gibt es weiter mit `walk_height_ramp:=true`.
  **Vorab-Anzeige (2026-10-20):** In Preparation liest `loco_client` beim
  Abgleich (2 Hz) den FSM-Mode mit und meldet ihn als `fsm_mode` im
  `onboard_state`. Schon vor dem Klick zeigt der Knopf »Walk
  Bodenkontakt ✓« bzw. »Walk kein Bodenkontakt« (dann gesperrt), die
  Info-Zeile „Bodenkontakt ✓ – »Walk« möglich" bzw. „kein Bodenkontakt
  (FSM-Mode …) – »Walk« gesperrt" und die Kachel GEHEN den Grund. Nur
  Anzeige: beim Klick prüft `loco_client` den Mode wie bisher noch einmal.
  Mode unbekannt (nicht lesbar, älterer `loco_client`) → »Walk« wie bisher frei.
  **Korrektur 2026-10-21 (Real-Test):** Der FSM-Mode bleibt in Preparation
  auch frei am Kran hängend 0 — er ist **kein** Bodenkontakt-Signal (damit
  war auch die alte Prüfung beim Klick wirkungslos). Bodenkontakt kommt jetzt
  aus der **Knie-Last**: `loco_client` abonniert `rt/lowstate` und glättet
  `tau_est` beider Knie (Gelenk 3/9, 0,2 s). Beide Knie ≥
  `knee_load_min_nm` (Env `G1_KNEE_LOAD_MIN_NM`, Default 10 Nm; Real-Test
  2026-10-21: frei stehend ≈ 14 Nm je Knie) → Bodenkontakt. `rt/lowstate` wird
  ohne Listener nur in Preparation mit 10 Hz gelesen (ein Listener mit 500 Hz
  brachte die Loco-RPCs ständig in Zeitüberschreitung).
  MuJoCo-Abschätzung (ungeprüft am G1): stehend ≈ 6,6 Nm je Knie, Kran trägt
  30 % → 3,7, 50 % → 2,0, hängend ≈ 0. Ohne frische `rt/lowstate` gilt
  „nicht messbar“ → »Walk« gesperrt. Die Info-Zeile zeigt die gemessenen
  Werte („Knie-Last 6.4/6.1 Nm“) — damit die Schwelle am G1 einstellen.
  Geprüft wird bei Startleiste »Walk«, LAUFEN, GREIFEN und der alten Rampe.
  `G1_GROUND_CHECK=fsm_mode` = alter Test, `none` = ohne Prüfung.
- **GEHEN/GREIFEN schalten den Onboard-Modus nicht mehr selbst um.** GEHEN ist
  gesperrt, bis die Info-Zeile Walk zeigt; GREIFEN in Preparation (am Kran)
  übernimmt nur die Arme. Die Handy-App funktioniert weiter wie bisher.
- **GEHEN mit aktivierten Armen (2026-10-19):** Die Arme bleiben in ihrer
  aktuellen Pose (kein Einklappen in die Lauf-Pose), `arm_sdk` hält Arme und
  Taille weiter. Altes Verhalten: `G1_WALK_PARK_ARMS=true`. Wer den Walker mit
  eigenem Armschwung will, deaktiviert die Arme vor GEHEN (DISABLE).
- **Alter Joystick (PS4 = Gehen):** Bei Trennung oder hängender Funkverbindung
  publiziert er nichts mehr (früher: letzter Zustand mit gedrücktem Deadman
  weiter), `joy_mux` gibt nur noch frische Eingänge weiter → `loco_client`
  stoppt nach 0,5 s.

Seit 2026-10-19:

- **Walk = FSM 501 („Walk with waist")** statt 500. Startleiste »Walk«, LAUFEN
  und GREIFEN aus Preparation senden `BalanceStand` + `SetFsmId(501)`
  (Parameter `start_fsm_id`, Env `G1_WALK_FSM_ID`; `G1_WALK_FSM_ID=500` = alter
  Walk mit fester Taille). Info-Zeile: „G1: Walk mit Taille (FSM 501)".
- **G1 steht per Handy-App in 500:** wird wie bisher als Walk übernommen
  (GEHEN geht). Die Info-Zeile zeigt „»Walk« schaltet auf FSM 501"; ein
  Doppelklick auf **Walk** wechselt dann nur den Walk-Modus (StopMove vorher,
  kein Preparation dazwischen). LAUFEN/GREIFEN schalten nicht implizit um.

## Fehlerbehebung

| Symptom | Ursache / Fix |
|---|---|
| `dds_check`: kein `lowstate` | Falsches `ROBOT_INTERFACE`; PC nicht im 192.168.123.x-Netz; Kabel. |
| „create domain error" im Log | `ROS_DOMAIN_ID` == Unitree-Domain (muss auf real 1 vs. 0 sein — Compose setzt das automatisch; eigene Terminals ebenfalls auf `ROS_DOMAIN_ID=1` setzen). |
| Hand „NICHT VERBUNDEN" | IP/Port prüfen (`nc -zv <ip> 6000`); Hand mit Strom versorgt? |
| „No joystick found" | `/dev/input` gemountet (Compose-Profil `real` braucht `privileged: true`)? Gerätename weicht ab → `JOYSTICK_NAME` setzen. Geräte anzeigen: `python3 -c "import evdev;[print(p, evdev.InputDevice(p).name) for p in evdev.list_devices()]"` |
| WALK-Klick ohne Wirkung | Demo-GUI: G1 erst über die Startleiste (Damp → Preparation → Walk) oder die App auf Walk stellen. |
| »Walk« abgelehnt „Füße ohne Last" | G1 hängt noch straff am Kran (FSM-Mode ≠ 0): Kran lockern, bis die Füße tragen, dann erneut. |
| Arme ruckeln beim Enable | Rampe verkürzt/0? `arm_weight_ramp_up_s` prüfen (Default 2.0 auf real; < 1 s wird auf real korrigiert). |
| DISABLE dauert mehrere Sekunden | Gewollt: Rückfahrt in die Onboard-Pose vor der Übergabe (max. 12 s). Log „Rueckgabe an den Onboard-Regler". |
| Log „Onboard-Pose unbekannt" bei DISABLE | ENABLE kam aus dem E-Stop-Latch o. ä. ohne gemessene Pose → alte direkte 2-s-Rampe. Arme vorher von Hand/Ziel in eine neutrale Pose bringen. |
| Log „REAL: … zu hart" | Ein Launchfile übergibt Sim-Gains; die Werte wurden begrenzt. Launchfile korrigieren. |
| Roboter geht nach GUI-Absturz weiter | Darf nicht passieren (Deadman) — Log von `loco_client` prüfen, melden. |
| RViz leer | `USE_RVIZ=true`? X11-Freigabe (`xhost +local:root`) gelaufen? |
| `permission denied … docker.sock` | Benutzer nicht in der Gruppe `docker` oder nicht neu eingeloggt (Abschnitt 0). |
| Erster Start hängt beim Bauen / Netzwerkfehler im Build | Image noch nicht gebaut und kein Internet über das Roboter-LAN. Mit Internet `make build-real` laufen lassen (Abschnitt 0). |
| Startmenü zeigt beim Interface „keine IPv4“ | Feste IP fehlt oder ist verloren gegangen (nur mit `ip addr add` gesetzt, Kabel gezogen). `sudo nmcli con up g1-robot`, siehe B. |
| Ping geht, `dds_check` bekommt nichts | Firewall (`sudo ufw status`), falsches Interface in der Auswahl (z. B. WLAN statt LAN), oder zwei Interfaces im 192.168.123.x-Netz. |
| `ros2 topic list` im eigenen Terminal leer | Eigene Terminals im Container öffnen (`make shell-real`), nicht auf dem Host. Dort sind `ROS_DOMAIN_ID=1` und die Loopback-DDS-Config schon gesetzt. |
| Startleiste: „keine Daten vom loco_client“ oder „Modus unbekannt“ | `loco_client` erreicht den G1 nicht (Interface/Kabel) oder die Loco-Firmware spricht einen anderen Dienst: `G1_LOCO_SERVICE=loco` für ai_sport bis 8.1, Default `sport` ab 8.2. |
| »Walk« gesperrt „kein Bodenkontakt“ | Knie-Last unter `G1_KNEE_LOAD_MIN_NM` (10 Nm). Kran weiter lockern; die Info-Zeile zeigt die gemessenen Werte. |
| L2 + Stick: G1 geht nicht, lange Vibration | G1 steht nicht in Walk → Startleiste »Walk«. Nach NOT-HALT erst mit »Start« quittieren. |

## Weiterführend

- Startmenü und Optionen: [03-A-Startmenue.md](03-A-Startmenue.md)
- Alle Umgebungsvariablen (Geh-Limits, Notaus, Taille …): [04-T-Umgebungsvariablen.md](04-T-Umgebungsvariablen.md)
- Bedienung: [10-A-Demo-GUI.md](10-A-Demo-GUI.md), [11-A-Controller.md](11-A-Controller.md), [12-A-Handy.md](12-A-Handy.md)
- Notaus: [51-A-Notaus.md](51-A-Notaus.md), Technik [51-T-Notaus.md](51-T-Notaus.md)
- Bekannter Linksdrall beim Gehen: [30-T-Stehen-und-Gehen.md](30-T-Stehen-und-Gehen.md#bekannte-einschränkungen)

Navigation gibt es am echten G1 nicht (LiDAR/MOLA wurde verworfen und
2026-10-30 entfernt, siehe [31-A-Navigation.md](31-A-Navigation.md)). `arm_sdk` parallel
zum Gehen funktioniert konstruktionsbedingt (Gewichts-Überblendung), für die
ersten Geh-Tests aber die Arme deaktiviert lassen — der natürliche Armschwung
der Unitree-Steuerung ist dafür besser geeignet.
