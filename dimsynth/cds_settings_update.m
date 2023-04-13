% Aktualisiere eine bestehende Einstellungs-Datei und setze neu ergänzte
% Einstellungsoptionen auf ihre Standardwerte. Dadurch wird die
% Abwärtskompatibilität zur Reproduktion alter Ergebnisse erhöht.
% 
% Eingabe:
% Set
%   Struktur mit Einstellungen aus cds_settings_defaults. Die Einstellungen
%   können aus einer alten Programmversion mit anderen Feldern kommen.
%  verbosity
%   Wert 1: Textausgabe aktivieren durch
% 
% Ausgabe:
% Set
%   Aktualisierte Einstellungsstruktur an die aktuelle Version des Programms
% 
% Siehe auch: cds_settings_defaults.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-02
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function Set = cds_settings_update(Set, verbosity)
if nargin < 2
  verbosity = 0;
end
% Standard-Einstellungen bestimmen
if isfield(Set.structures, 'DoF')
  DoF = Set.structures.DoF;
  % Entferne die alte Speicherung des Freiheitsgrads und nehme die neue
  Set.structures = rmfield(Set.structures, 'DoF');
  Set.task.DoF = DoF;
elseif isfield(Set.task, 'DoF')
  DoF = Set.task.DoF;
else
  error('Datenformat der Eingabe ist unbekannt');
end
Set_defaults = cds_settings_defaults(struct('DoF', DoF));

%% Manuelle Übersetzung einiger Einstellungen
if ~isfield(Set.optimization,'max_range_passive_universal')
  Set.optimization.max_range_passive_universal = ...
    Set.optimization.max_range_passive_revolute;
end
if ~isfield(Set.optimization,'max_range_passive_spherical')
  Set.optimization.max_range_passive_spherical = ...
    Set.optimization.max_range_passive_revolute;
end
if ~isfield(Set.optimization,'max_velocity_passive_universal')
  Set.optimization.max_velocity_passive_universal = ...
    Set.optimization.max_velocity_passive_revolute;
end
if ~isfield(Set.optimization,'max_velocity_passive_spherical')
  Set.optimization.max_velocity_passive_spherical = ...
    Set.optimization.max_velocity_passive_revolute;
end
if ~isfield(Set.optimization,'obj_power') % Neu eingeführt am 13.04.2023
  Set.optimization.obj_power = struct('symmetric_speed_torque_limits', false);
end
if length(Set.optimization.constraint_obj) < ...
  length(Set_defaults.optimization.constraint_obj)
  Set.optimization.constraint_obj = [Set.optimization.constraint_obj; ...
    zeros(length(Set_defaults.optimization.constraint_obj)-...
          length(Set.optimization.constraint_obj))];
  if verbosity
    fprintf('Fehlende Einträge in constraint_obj wurden auf Null gesetzt\n');
  end
end
% Ersetze einige Felder zur Struktur-Filterung, die vor der Implementierung
% nicht dem aktuellen Standardwert entsprachen
if ~isfield(Set.optimization,'min_inclination_conic_base_joint')
  Set.optimization.min_inclination_conic_base_joint = 0;
end
if ~isfield(Set.optimization,'min_inclination_conic_platform_joint')
  Set.optimization.min_inclination_conic_platform_joint = 0;
end
%% Automatische Übersetzung anderer Einstellungen
% Gehe alle Felder der Einstellungen durch und ergänze fehlende
mainfields = {'optimization', 'structures', 'task', 'general'};
for mf = mainfields
  list_added_mf = {}; % Speichere Liste von hinzugefügten Feldern ab.
  for f = fields(Set_defaults.(mf{1}))'
    if ~isfield(Set.(mf{1}), f{1})
      Set.(mf{1}).(f{1}) = Set_defaults.(mf{1}).(f{1});
      list_added_mf = [list_added_mf, f{1}]; %#ok<AGROW>
    end
    if isa(Set.(mf{1}).(f{1}), 'struct')
      % Gehe auch hier nochmal durch die Felder. Keine weitere Rekursion.
      for f2 = fields(Set_defaults.(mf{1}).(f{1}))'
        if ~isfield(Set.(mf{1}).(f{1}), f2{1})
          Set.(mf{1}).(f{1}).(f2{1}) = Set_defaults.(mf{1}).(f{1}).(f2{1});
          list_added_mf = [list_added_mf, [f{1},'.',f2{1}]]; %#ok<AGROW>
        end
      end
    end
  end
  if ~isempty(list_added_mf) && verbosity
    fprintf('Folgende Felder in Set.%s wurden auf Standardwerte gesetzt: %s\n', ...
      mf{1}, disp_array(list_added_mf));
  end
end
% Prüfe auch die mittlerweile obsoleten Felder. Entferne diese, damit kein
% undefinierter Zustand im Programm entsteht.
for mf = mainfields
  list_removed_mf = {}; % Speichere Liste von hinzugefügten Feldern ab.
  for f = fields(Set.(mf{1}))'
    if ~isfield(Set_defaults.(mf{1}), f{1})
      Set.(mf{1}) = rmfield(Set.(mf{1}), f{1});
      list_removed_mf = [list_removed_mf, f{1}]; %#ok<AGROW>
    end
  end
  if ~isempty(list_removed_mf) && verbosity
    fprintf('Folgende Felder in Set.%s wurden entfernt, da nicht mehr verwendet: %s\n', ...
      mf{1}, disp_array(list_removed_mf));
  end
end