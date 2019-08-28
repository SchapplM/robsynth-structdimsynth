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
general = struct( ...
  'verbosity', 2, ...
  'plot_robot_in_fitness', 0, ... % Schwellwert der Gütefunktion zum Zeichnen von Details.
  'plot_details_in_fitness', 0, ... % Positiv: nur bei besseren; negativ: nur bei schlechteren als ...
  'save_robot_details_plot_fitness_file_extensions', {''}, ... % Speichern des durch vorherige Einstellung erstellten Bildes
  'save_animation_file_extensions', {{'gif'}}, ... % Format, in denen die Animationen gespeichert werden
  'save_evolution_video', false, ... % Video mit Evolution der Roboter
  'regenerate_summmary_only', false, ... % Nur die Videos und Zusammenfassungsbilder neu generieren. Keine Optimierung durchführen.
  'use_mex', true);

%% Einstellungen zur Auswahl der verwendeten Strukturen
% overconstraint: Anzahl der Beingelenke, die die EE-FG übersteigen
% task redundancy: Anzahl der EE-FG, die die Aufgaben-FG übersteigen
structures = struct( ...
  'use_serial', true, ...
  'use_parallel', true, ...
  'maxnumprismatic', 1, ...
  'max_overconstraint', 3, ...
  'max_task_redundancy', 0, ...
  'DoF', input_settings.DoF, ...
  'nopassiveprismatic', true, ...
  'activenotlastjoint', true, ...
  'whitelist', {''}); % Liste, die die Systeme beschränkt

%% Optimierungs-Einstellungen
optimization = struct( ...
  'objective', 'energy', ... % Zielfunktion
  'constraint', '', ... % Nebenbedingungen
  'movebase', true, ... % Position der Roboter-Basis
  'ee_translation', true, ... % Freie Verschiebung des EE
  'ee_translation_only_serial', true, ... % ... nur bei seriellen Robotern
  'ee_rotation', false, ... % Freie Rotation des EE
  'rotate_base', false, ... % Orientierung der Roboter-Basis
  'rotate_coupling', true, ... % Koppel-Punkt-Orientierung für PKM
  'NumIndividuals', 50, ...
  'MaxIter', 10, ...
  'ElectricCoupling', true, ... % Kopplung der Achsen für Energieberechnung. TODO
  'resdir', fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'dimsynth', 'results'), ...
  'optname', 'unnamed');

%% Einstellungen für Trajektorie
task = struct( ...
  'profile', 1, ... % Beschleunigungs-Trapez
  'vmax', 1, ...
  'amax', 3, ...
  'Tv', 0.01, ...
  'Ts', 1e-3, ...
  'payload', struct('m', 3, 'rS', zeros(3,1), 'Ic', zeros(6,1)));
task.payload.Ic(1:3) =  2/5 * task.payload.m * (60e-3)^2; % Kugel Radius 60mm
%% Rückgabe Gesamt-Einstellungen
settings = struct(...
  'structures', structures, ...
  'optimization', optimization, ...
  'task', task, ...
  'general', general);