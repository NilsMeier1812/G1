# Notaus — Anleitung

Richtet sich an alle, die den G1 bedienen. Alle Wege, den Roboter
anzuhalten, an einer Stelle. Technik: [51-T-Notaus.md](51-T-Notaus.md).
Sicherheitsablauf am echten Roboter: [50-A-Echter-Roboter.md](50-A-Echter-Roboter.md).

## Stufen

| Stufe | Auslösen | Was passiert (echter G1) |
|---|---|---|
| **Stopp** | Controller **Kreuz** (Xbox **A**), Stick loslassen | Hand steht sofort, geplante Bewegung bricht ab. Roboter bleibt, wie er ist |
| **Gehen stoppen** | **L2 loslassen** (Totmann), GUI-Gehknopf loslassen | G1 bleibt stehen, balanciert weiter |
| **NOT-HALT** | Demo-GUI **NOT-HALT**, Controller **PS** (Xbox-Taste), Handy **NOT-HALT** | Arme schlaff (sacken gedämpft ab), Hände aus (halten ihre Stellung), Hüftdrehung steht, **Taille gehalten**, Beine `StopMove`: der G1 **bleibt stehen** |
| **Voll-Notaus** | Controller **R1 + L1 + PS 2 s halten** (Xbox **RB + LB + Xbox-Taste**) | ganzer G1 in **Damp**, auch Beine und Taille: er **sackt zusammen** (Kran!) |
| **Unitree-Fernbedienung** | Damp an der Fernbedienung | wirkt an der Firmware, unabhängig von PC und Stack |

Beim Voll-Notaus vibriert der Controller während der 2 s; vorzeitig
loslassen bricht ab, der NOT-HALT vom PS-Druck bleibt aber bestehen.

## In der Simulation

Der NOT-HALT setzt auch die Beine auf DAMP: der Sim-Roboter fällt um. Der
Voll-Notaus wirkt dort wie der NOT-HALT. »Roboter zurücksetzen« stellt ihn
wieder auf ([40-A-Simulation.md](40-A-Simulation.md#nur-in-der-sim)).

## Quittieren

Nach einem NOT-HALT ist alles gesperrt, bis quittiert wird:

| | Sim | Echter G1 |
|---|---|---|
| Demo-GUI | einen Modus wählen (GREIFEN/GEHEN) | Startleiste **NOT-HALT quittieren** (»Start«) |
| Handy | **Start** (mit Recht »Bedienen«) | **Start** |
| Controller | geht nicht, absichtlich | geht nicht, absichtlich |

Danach bleiben die Arme schlaff, bis man sie bewusst wieder übernimmt
(GREIFEN, Controller **R2**). Stand der
G1 vorher im Walk-Modus, bleibt er darin (kein Sprung nach FSM 4).

Nach dem **Voll-Notaus** quittiert »Start« nur; der G1 bleibt in Damp.
Aufstehen: Startleiste **Preparation**, dann **Walk**.

## Was zusätzlich von selbst anhält

- **Controller-Abriss:** 0,3 s kein Signal → Gehen stoppt; `loco_client`
  stoppt nach 0,5 s ohne Gehbefehl, jedes Gehkommando gilt am G1 nur 1 s.
- **Controller sperren** (Options / Xbox Menu): zum Ablegen. Gesperrt wirken
  nur noch PS (NOT-HALT), der Voll-Notaus und Options (entsperren),
  [11-A-Controller.md](11-A-Controller.md#controller-sperren-options).
- **Handy:** »Daten alt« nach 1,5 s ohne Status sperrt alles außer NOT-HALT.
  Der Handy-NOT-HALT läuft über den PC; hängt der PC, wirkt er nicht
  ([12-A-Handy.md](12-A-Handy.md)).
- **Real beenden** (Demo-GUI) gibt die Arme ab, bevor der Stack stoppt; ein
  NOT-HALT bricht das ab.

## Was im Ernstfall

| Situation | Reaktion |
|---|---|
| Arm fährt falsch, aber ruhig | Kreuz bzw. GREIFEN verlassen |
| Ruck, Schwingen, Kontakt mit Mensch oder Objekt | **NOT-HALT** (GUI, PS, Handy) |
| G1 soll sofort ganz weich werden | **Voll-Notaus** R1 + L1 + PS 2 s |
| Sturz droht, PC reagiert nicht | **Unitree-Fernbedienung** |

Die Tabelle »Notaus-Reflex« in
[50-A-Echter-Roboter.md](50-A-Echter-Roboter.md#notaus-reflex-auswendig-lernen)
ist dieselbe Liste in Bedienbegriffen; vor dem ersten Lauf am echten G1
auswendig lernen.

## Altes Verhalten zurückholen

| Variable | Wirkung |
|---|---|
| `G1_ESTOP_DAMP_LEGS=1` | NOT-HALT schickt zusätzlich `Damp` an die Beine (vor 2026-10-15 Standard) |
| `G1_ESTOP_WAIST_HOLD=false` | Taille beim NOT-HALT schlaff statt gehalten |
| `G1_PS4_PS_ESTOP=0` | PS am Controller nur Stopp wie Kreuz, kein NOT-HALT |
| `G1_XBOX_PS_BUTTON=view` | NOT-HALT am Xbox-Controller auf View statt Xbox-Taste |
| `G1_PHONE_VIEW_ESTOP=0` | Handys im Modus »Ansicht« dürfen keinen NOT-HALT |
