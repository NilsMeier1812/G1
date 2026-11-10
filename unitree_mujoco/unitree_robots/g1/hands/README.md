# Hand-Profile (eigene starre Hände, nur Simulation)

Jeder Unterordner mit einer `hand.yaml` ist ein Hand-Profil und erscheint im
Startmenü und in der Start-GUI unter **Hände**.

- `haken/` — die gedruckten Box-Haken
- `flansch_referenz_left.stl`, `flansch_referenz_right.stl` — Handgelenk-Flansch
  in mm im Flansch-Frame (Ursprung = Flansch-Mitte, x = vom Flansch weg) als
  Referenz fürs CAD

```bash
cd ../tools
./hand_tool.sh list
./hand_tool.sh preview haken      # Vorschau, hand.yaml speichern = neu laden
./hand_tool.sh build haken        # nur neu erzeugen
```

Anleitung: [g1pilot/docs/25-A-Eigene-Haende.md](../../../../g1pilot/docs/25-A-Eigene-Haende.md),
Technik: [25-T](../../../../g1pilot/docs/25-T-Eigene-Haende.md).
