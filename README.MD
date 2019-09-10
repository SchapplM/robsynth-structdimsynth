# Toolbox für kombinierte Struktur- und Maßsynthese für serielle und parallele Roboter

Dieses Repo enthält die kombinierte Struktur- und Maßsynthese für serielle und parallele Roboter von Moritz Schappler.

Mit dieser Software werden alle parallelen Roboterstrukturen für eine gegebene Aufgabe gefunden ("Struktursynthese") und diese Strukturen anschließend optimiert ("Maßsynthese"). Die Struktursynthese serieller Roboter erfolgt teilweise mit der Struktursynthese von Daniel Ramirez.

Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-04  
(C) Institut für mechatronische Systeme, Universität Hannover

## Benutzung

Für die Benutzung des Repos muss mit dem Skript `structgeomsynth_path_init.m` aus dem Hauptordner die Pfadinitialisierung durchgeführt werden.

### Abhängigkeiten

Die Struktursynthese setzen die Toolbox imes-matlab, imes-traj, serrob_mdlbib, parrob_mdlbib und imes-matlab-robotics mit initialisiertem Pfad voraus.

### Struktursynthese

Der Hauptteil der Struktursynthese serieller Ketten erfolgt aktuell im anderen Synthese-Repo mit dem Code von Daniel Ramirez. Hier werden ausgewählte Eigenschaften der seriellen Ketten geprüft. Dazu dienen die Skripte in `struktsynth_ser`:

* `check_first_joint_orientation`: Prüfen, ob die Ausrichtung des ersten Gelenks dem vorgesehenen Schema entspricht.
* `serrob_change_last_joint_to_spherical`: Verändere bestehende allgemeine Strukturen in der Datenbank so, dass Beinketten mit Kugelgelenk am Ende als Varianten zusätzlich entstehen
* `serrob_isomorph_list`: Untersuche die Datenbank auf Isomorphismen
* `serrob_list_parameter_no_influence`: Prüfe, ob Parameter von Robotermodellen eventuell keinen Einfluss auf die Endeffektorbewegung haben.

Für die Struktursynthese paralleler Roboter existiert aktuell nur ein Skript im Ordner `struktsynth_par`:

* `parroblib_add_robots_symact_alldof`: Alle symmetrisch aktuierten parallelen Roboter generieren und zur Datenbank ParRobLib hinzufügen

### Maßsynthese

Die Maßsynthese erfolgt mit den Funktionen aus `dimsynth`. Der komplette Ablauf wird mit den Konfigurationsskripten aus `config` gestartet. Die Skripte bieten entweder Minimalbeispiele für bestimmte Freiheitsgrade oder führen die Maßsynthese für eine größere Menge an Robotern aus.

