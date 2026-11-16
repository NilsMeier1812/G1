# Beispielprojekt: Arm winken über die Arm-API

Zeigt, dass sich der G1-Stack erweitern lässt, **ohne eine Zeile im Kern
(`g1pilot/`) zu ändern**. Das Programm `winken.py` liegt bewusst außerhalb von
`g1pilot/`, nutzt nur die Python-Standardbibliothek und spricht ausschließlich
die dokumentierte Arm-Schnittstelle an (HTTP/JSON auf `localhost:8770`, siehe
`g1pilot/docs/21-A-Arm-API.md`).

## Aufruf

1. Stack starten (Simulation oder echter G1) und die Arme übernehmen
   (Demo-GUI **GREIFEN** bzw. Controller **R2**).
2. `python3 beispiele/arm_winken/winken.py`

Der rechte Arm hebt sich, winkt zweimal mit dem Ellbogen und fährt zurück in
die Ausgangsstellung. Jede Bewegung plant der `arm_controller` (Kollisions-
schutz, Grenzen, NOT-HALT gelten unverändert).

## Nachweis „Kern unverändert“

`nachweis_kern_unveraendert.sh` listet alle Commits, die dieses Beispiel
anlegen oder ändern, und zeigt per `git diff`, dass sie keine Datei außerhalb
von `beispiele/` berühren. Siehe `g1pilot/docs/75-A-Erweiterungsbeispiel.md`.
