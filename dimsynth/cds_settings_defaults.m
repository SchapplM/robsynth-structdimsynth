% Standard-Einstellungen für kombinierte Struktur- und Maßsynthese
% 
% Eingabe:
% input_settings
%   Struktur mit Einstellungen zur Generierung aller
%   Optimierungseinstellungen
% 
% Ausgabe:
% settings
%   Struktur mit Feldern für alle Einstellungen der Optimierung
%   Die Einstellungen sind global und nicht spezifisch für den einzelnen
%   Roboter

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function settings = cds_settings_defaults(input_settings)
%% Standard-Eingabeeinstellungen setzen
input_settings_default = struct('DoF', logical([1 1 1 1 1 1]));
if nargin == 0
  input_settings = input_settings_default;
end
for f = fields(input_settings_default)'
  if ~isfield(input_settings, f{1})
    input_settings.(f{1}) = input_settings_default.(f{1});
  end
end
input_settings.DoF = logical(input_settings.DoF);
%% Allgemeine Einstellugen
% Einstellungen mit Auswirkungen auf vom Benutzer sichtbare Ergebnisse und
% deren Nachverarbeitung
general = struct( ...
  'verbosity', 2, ... % Ausgabeleven;0=minimal (Endergebnis), 1=mehr (Generationenweise), 2=noch mehr (Jede Fitness-Eval.), 3=alles, 4=Auch Entwurfsoptimierung
  'matfile_verbosity', 0, ... % Menge an mat-Dateien, die für Debug-Zwecke gespeichert werden; 0=keine, 1=wenige, 2=viele (erfolgreiche Ind.) 3=alle
  'plot_robot_in_fitness', 0, ... % Schwellwert der Gütefunktion zum Zeichnen von Details.
  'plot_details_in_fitness', 0, ... % Positiv: nur bei besseren; negativ: nur bei schlechteren als ...
  'plot_details_in_desopt' , 0, ... % Wie vorheriges Argument, aber für die Gütefunktion der Entwurfsoptimierung
  'save_robot_details_plot_fitness_file_extensions', {''}, ... % Speichern des durch vorherige Einstellung erstellten Bildes
  'save_animation_file_extensions', {{'gif'}}, ... % Format, in denen die Animationen gespeichert werden
  'save_evolution_video', false, ... % Video mit Evolution der Roboter
  'max_retry_bestfitness_reconstruction', 10, ...
  'regenerate_summmary_only', false, ... % Nur die Videos und Zusammenfassungsbilder neu generieren. Keine Optimierung durchführen.
  'nosummary', false, ... % Kompletter Verzicht auf die graphische Endauswertung
  'noprogressfigure', false, ... % Verzicht auf Fortschritts-Bild des PSO
  'debug_calc', false, ... % Doppelte Berechnung zur Prüfung von Funktionen
  'use_mex', true);

%% Einstellungen zur Auswahl der verwendeten Strukturen
% overconstraint: Anzahl der Beingelenke, die die EE-FG übersteigen
% task redundancy: Anzahl der EE-FG, die die Aufgaben-FG übersteigen
structures = struct( ...
  'use_serial', true, ...
  'use_parallel', true, ...
  'use_parallel_rankdef', false, ... Nehme auch parallele Roboter, die mit Rangverlust in der Datenbank stehen
  'maxnumprismatic', 1, ...
  'max_overconstraint', 3, ...
  'max_task_redundancy', 0, ... % Zulässiger Grad der Aufgabenredundanz
  'max_kin_redundancy', 0, ... % Zulässiger Grad der kinematischen Redundanz
  'DoF', input_settings.DoF, ...
  'nopassiveprismatic', true, ...
  'activenotlastjoint', true, ...
  'whitelist', {''}); % Liste, die die Systeme beschränkt

%% Optimierungs-Einstellungen
% Einstellungen mit Auswirkung auf die Optimierung: Auswahl der
% Optimierungsvariablen und Annahmen über die Roboter, die getroffen werden
optimization = struct( ...
  'objective', 'energy', ... % Zielfunktion. Möglich: mass, energy, condition, valid_kin, valid_act, minactforce, stiffness
  'constraint_obj', zeros(5,1), ... % Nebenbedingungen, 1=Mass, 2=Energy, 3=Minactforce, 4=Condition, 5=Stiffness; Eintrag entspricht physikalischem Wert
  'movebase', true, ... % Position der Roboter-Basis
  'ee_translation', true, ... % Freie Verschiebung des EE
  'ee_translation_only_serial', true, ... % ... nur bei seriellen Robotern
  'ee_rotation', true, ... % Freie Rotation des EE
  'base_size', true, ... % Größe des Gestells
  'base_size_limits', [NaN, NaN], ... % Grenzen für Gestell-Größe (Absolut)
  'platform_size', true, ... % Größe der Plattform
  'platform_size_limits', [NaN, NaN], ... % Grenzen für Plattform-Größe (Absolut)
  'base_morphology', false, ... % Aussehen des Gestells (z.B. Schrägheit, Gelenkpaarabstand)
  'platform_morphology', false, ... % Aussehen der Plattform (z.B. Gelenkpaarabstand)
  'rotate_base', false, ... % Orientierung der Roboter-Basis
  'rotate_coupling', true, ... % Koppel-Punkt-Orientierung für PKM
  'max_range_active_revolute', 270*pi/180, ... % Maximaler Drehwinkel aktiver Drehgelenke
  'max_range_passive_revolute', 360*pi/180, ... % Maximaler Drehwinkel passiver Drehgelenke
  'max_velocity_active_revolute', 8, ... % [rad/s] Maximale Drehzahl (zur Singularitätsvermeidung)
  'max_velocity_active_prismatic', 5, ... % [rad/s] Maximale Geschw. (zur Singularitätsvermeidung)
  'use_desopt', false, ... % Schalter für eigene Optimierung der Entwurfsparameter
  'desopt_link_yieldstrength', false, ... % Schalter für Benutzung der Streckgrenze der Segmente für die Entwurfsoptimierung
  'NumIndividuals', 50, ...
  'MaxIter', 10, ...
  'ElectricCoupling', true, ... % Kopplung der Achsen für Energieberechnung. TODO
  'resdir', fullfile(fileparts(which('structgeomsynth_path_init.m')), 'dimsynth', 'results'), ...
  'optname', 'unnamed');

%% Einstellungen für Trajektorie
task = struct( ...
  'profile', 1, ... % Beschleunigungs-Trapez
  'vmax', 1, ...
  'amax', 3, ...
  'Tv', 0.01, ...
  'Ts', 1e-3, ...
  'maxangle', 2*pi, ... % Keine Einschränkung für die maximalen Winkel
  'payload', struct('m', 3, 'rS', zeros(3,1), 'Ic', zeros(6,1)));
task.payload.Ic(1:3) =  2/5 * task.payload.m * (60e-3)^2; % Kugel Radius 60mm
%% Rückgabe Gesamt-Einstellungen
settings = struct(...
  'structures', structures, ...
  'optimization', optimization, ...
  'task', task, ...
  'general', general);
