# Erweiterungsbeispiel „Arm winken“ — Anleitung

## Zweck

Nachweis der Erweiterbarkeit: ein kleines Programm in einem eigenen Ordner
**außerhalb von `g1pilot/`** (`beispiele/arm_winken/`), das nur über die
dokumentierte Arm-Schnittstelle ([21-A-Arm-API.md](21-A-Arm-API.md),
HTTP/JSON auf `localhost:8770`) eine Armbewegung auslöst. Es nutzt nur die
Python-Standardbibliothek und importiert nichts aus `g1pilot`.

## Aufruf

Stack starten (Simulation oder echter G1), Arme übernehmen (**GREIFEN**),
dann im Repo-Wurzelordner:

```bash
python3 beispiele/arm_winken/winken.py            # --url, --token, --mal
```

Der rechte Arm hebt sich, winkt mit dem Ellbogen und fährt in die
Ausgangsstellung zurück. Alle Sicherheitsgates (NOT-HALT, Kollision,
Grenzen) bleiben im `arm_controller` wirksam.

## Nachweis „keine Zeile im Kern geändert“

```bash
bash beispiele/arm_winken/nachweis_kern_unveraendert.sh
```

Das Skript zeigt für jeden Commit, der `beispiele/arm_winken/` anlegt oder
ändert, `git diff --stat` und prüft mit
`git diff <commit>^ <commit> -- . ':!beispiele/arm_winken'`, dass außerhalb
des Beispielordners nichts geändert wurde (Rückgabewert 0 = Nachweis
erbracht).

## Grenzen

- Die Arm-API ist standardmäßig nur auf `127.0.0.1` erreichbar; von einem
  anderen Rechner aus braucht es `arm_api_host` und einen Token.
- Das Beispiel prüft keine Erreichbarkeit vorab; lehnt der `arm_controller`
  ab, steht der HTTP-Status in der Ausgabe.
