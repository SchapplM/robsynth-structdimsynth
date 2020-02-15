# Toolbox für kombinierte Struktur- und Maßsynthese für serielle und parallele Roboter

Dieses Repo enthält die kombinierte Struktur- und Maßsynthese für serielle und parallele Roboter von Moritz Schappler.

Mit dieser Software werden alle parallelen Roboterstrukturen für eine gegebene Aufgabe gefunden ("Struktursynthese") und diese Strukturen anschließend optimiert ("Maßsynthese"). Die Struktursynthese serieller Roboter erfolgt teilweise mit der Struktursynthese von Daniel Ramirez.

Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-04  
(C) Institut für Mechatronische Systeme, Universität Hannover

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
Der grobe Ablauf der Maßsynthese erfolgt durch die folgenden Matlab-Skripte und -Funktionen:

 * Start-Skript, z.B. `config/cds_example_2T1R.m` mit allen Einstellungen der Optimierung und Auswahl der Roboter (-Freiheitsgrade)
 * `cds_start`: Starten des Gesamt-Ablaufs
 * `cds_gen_robot_list`: Liste aller Roboter erzeugen (aus Datenbanken mit Filterkriterien)
 * `cds_dimsynth_robot`: Optimierung starten
 * `cds_fitness`: Diese Fitness-Funktion wird durch wiederholtes Aufrufen optimiert. Dort sind folgende Punkte enthalten:
   * `cds_update_robot_parameters`: Parameter aktualisieren
   * `cds_constraints`: Nebenbedingungen prüfen
   * `cds_dimsynth_design`: Entwurfsparameter eintragen
   * `cds_dimsynth_desopt`: Entwurfsoptimierung durchführen
   * `cds_obj_...`: Zielfunktion berechnen (z.B. `..._condition`, `..._energy`, `..._mass`, ...)
 * `cds_results_table.m`: Nach der Optimierung werden die Endergebnisse (optimale Parameter) jedes Roboters als Tabelle gespeichert
 * `cds_vis_results`: Nach der Optimierung erfolgt eine Darstellung der Ergebnisse. Die Bilder werden in einem Unterordner von `dimsynth/results` gespeichert

#### Entwurfsoptimierung

Optional kann in der Maßsynthese als innere Kaskade eine Optimierung der Entwurfsparameter durchgeführt werden.
Entwurfsparameter sind dabei Parameter, die nicht die Kinematik des Roboters beeinflussen. Also z.B. der Durchmesser und die Stärke der Segmente (im Gegensatz zu ihrer Länge).
Der Ablauf erfolgt in diesen Funktionen:

   * `cds_dimsynth_desopt`: Einrichtung und Start der Optimierung in der innersten Kaskaden
   * `cds_dimsynth_desopt_fitness`: Fitness-Funktion, die durch wiederholten Aufruf optimiert wird
   * `cds_dimsynth_design`: Aktualisieren des Robotermodells basierend auf den optimierten Entwurfsparametern

# Quellen

Die kombinierte Struktur- und Maßsynthese wird in folgenden Quellen beschrieben:

* [SchapplerOrt2020] Schappler, M., Ortmaier, T.: Dimensional Synthesis of Parallel Robots: Unified Kinematics and Dynamics using Full Kinematic Constraints, IFToMM DACH Konferenz (2020)
* [SchapplerTapOrt2019] Schappler, M. and Tappe, S., Ortmaier, T.: Exploiting Dynamics Parameter Linearity for Design Optimization in Combined Structural and Dimensional Robot Synthesis, IFToMM World Congress (2019)
* [Ramirez2018] Ramirez, D.: Automatic Generation of Task-specific Mechanisms using Combined Structural and Dimensional Synthesis, Dissertation, Uni Hannover (2018)
