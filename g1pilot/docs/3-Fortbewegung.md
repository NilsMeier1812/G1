# 3 · Fortbewegung

Stehen, Gehen und autonomes Fahren. In der Sim übernimmt eine eigene
Lauf-Policy die Beine, am echten G1 die Unitree-Firmware.

| Thema | Kurz | Anleitung | Technik |
|---|---|---|---|
| **Stehen und Gehen** | Zustände Stand (GREIFEN) und Laufen (GEHEN); Sim: Lauf-Policies `g1_wholebody` und AGILE; real: Damp → Preparation → Walk (FSM 501), Bodenkontakt-Prüfung, Geh-Limits | [30-A](30-A-Stehen-und-Gehen.md) | [30-T](30-T-Stehen-und-Gehen.md) |
| **Navigation** | AUTO NAV von Punkt zu Punkt mit Pfadplanung, Stationen aus der Umgebung. nur in der Simulation (LiDAR/MOLA für den echten G1 wurde verworfen) | [31-A](31-A-Navigation.md) | [31-T](31-T-Navigation.md) |

Gesteuert wird das Gehen über die Demo-GUI, den Controller (L2) oder das
Handy, siehe [1 · Bedienung](1-Bedienung.md).

Alle Gruppen: [README.md](README.md).
