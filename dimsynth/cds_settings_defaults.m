% Standard-Einstellungen für kombinierte Struktur- und Maßsynthese

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
  'verbosity', 2);

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
  'activenotlastjoint', true);

%% Optimierungs-Einstellungen
optimization = struct( ...
  'objective', 'energy', ... % Zielfunktion
  'constraint', '', ... % Nebenbedingungen
  'movebase', true, ... % Position der Roboter-Basis
  'ee_translation', false, ... % Freie Verschiebung des EE
  'ee_rotation', false, ... % Freie Rotation des EE
  'rotate_base', false, ... % Orientierung der Roboter-Basis
  'rotate_coupling', true, ... % Koppel-Punkt-Orientierung für PKM
  'NumIndividuals', 20, ...
  'MaxIter', 5, ...
  'vartypes', [], ...% Art der Optimierungsparameter
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
task.payload.Ic(3) =  2/5 * task.payload.m * (60e-3)^2; % Kugel Radius 60mm
%% Rückgabe Gesamt-Einstellungen
settings = struct(...
  'structures', structures, ...
  'optimization', optimization, ...
  'task', task, ...
  'general', general);