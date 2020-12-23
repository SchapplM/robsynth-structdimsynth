% Bereite die Roboterdatenbank für die Auswertung von Ergebnissen vor
% Erstellt Vorlagen-Funktionen und kompiliert notwendige Funktionen für die
% Nachträge Auswertung (z.B. falls die Berechnung auf dem Cluster lief)
% 
% Eingabe:
% OptName
%   Name des Ordners der Maßsynthese, die nachträglich ausgewertet wird.
%   Der Ordner muss im Verzeichnis results in diesem Repo liegen.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function cds_prepare_results_eval(OptName)
%% Optimierung laden
resdir = fullfile(fileparts(which('structgeomsynth_path_init.m')), 'results');
resdir_opt = fullfile(resdir, OptName);
if ~exist(resdir_opt, 'file')
  warning('Ergebnis-Ordner %s existiert nicht.', resdir_opt);
  return
end
setfile = fullfile(resdir_opt, sprintf('%s_settings.mat', OptName));
if ~exist(setfile, 'file')
  warning('Einstellungs-Datei %s existiert nicht.', setfile);
  return
end
d3 = load(setfile, 'Set', 'Structures');
%% Roboter initialisieren
% Gleiche Vorgehensweise wie in cds_start.m
for type = [0 2]
  RobNames = {};
  for i = 1:length(d3.Structures)
    Structure = d3.Structures{i};
    if Structure.Type == type
      RobNames = [RobNames, Structure.Name]; %#ok<AGROW>
    end
  end
  RobNames = unique(RobNames);
  if isempty(RobNames), continue; end
  if type == 0 % Serieller Roboter
    fprintf('Erstelle Vorlagen-Funktionen für serielle Roboter\n');
    serroblib_create_template_functions(RobNames, false, false);
  else % PKM
    LegName_all = [];
    for i = 1:length(RobNames)
      [~,LEG_Names] = parroblib_load_robot(RobNames{i});
      LegName_all = [LegName_all, unique(LEG_Names)]; %#ok<AGROW>
    end
    fprintf('Erstelle Vorlagen-Funktionen für Beinketten paralleler Roboter\n');
    serroblib_create_template_functions(unique(LegName_all), false, false);
    fprintf('Erstelle Vorlagen-Funktionen für parallele Roboter\n');
    parroblib_create_template_functions(RobNames, false, false);
  end
  fprintf('Kompiliere die Funktionen\n');
  for i = 1:length(RobNames)
    if type == 0 % Serieller Roboter
      R = serroblib_create_robot_class(RobNames{i});
    else % PKM
      R = parroblib_create_robot_class(RobNames{i},1,1);
    end
    R.fill_fcn_handles(true, true);
  end
end