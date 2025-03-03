% Wandle die Namen der Roboter einer Maßsynthese in ein lesbares Format um.
% Für Skripte zur Nachverarbeitung von Ergebnissen der Maßsynthese
% 
% Eingabe:
% Robots: Cell Array mit Namen von Robotern.
%   Bspw.: {'P6PRRRRR2V2G8P5A1', 'P6RRRRRR8V3G7P3A1', ...}
% TablePath
%   Pfad zur CSV-Tabelle, die geschrieben werden soll
% writemode {edit, overwrite}
%   Bei edit wird die Tabelle nur bearbetet, bei overwrite neu geschrieben
%   
% updatemode {default, skip}
%   Bei skip: Überspringe bereits vorhandene Einträge (Annahme: Datenbank
%   bleibt gleich)
% 
% Erzeugt Datei:
% * robot_names_latex.csv
%
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function dimsynth_expeval_robot_names(Robots, TablePath, writemode, updatemode, verbosity)
if nargin < 3
  writemode = 'overwrite';
end
if nargin < 4
  updatemode = 'skip';
end
if nargin < 4
  verbosity = 0;
end
%% Hole die Zeichenfolge für die Gelenkkette
% Robots = unique(ResTab.Name);
ResTab_NameTrans = cell2table(cell(0,7), 'VariableNames', {'PKM_Name', ...
  'Gnum', 'Pnum', 'Chain_Name', 'ChainStructure', 'Chain_Structure_Act', 'Chain_ShortName'});
if strcmp(writemode, 'edit') && exist(TablePath, 'file')
  ResTab_NameTrans = readtable(TablePath, 'Delimiter', ';');
end
I = 1:length(Robots);
% Debug: Auswahl eines bestimmten Roboters
% I = find(strcmp(Robots, 'P4RRRRR8V2G1P1A1'))';
for i = I
  RobName = Robots{i};
  if verbosity > 1
    fprintf('Hole Bezeichnung für Rob %d/%d (%s)\n', i, length(Robots), RobName);
  end
  % Prüfe, ob Roboter schon in Tabelle ist
  if strcmp(updatemode, 'skip') && any(strcmp(ResTab_NameTrans.PKM_Name, RobName))
    if verbosity > 1
      fprintf('Roboter %s steht schon in Namenstabelle. Überspringe.\n', RobName);
    end
    continue;
  end
  % Generiere Namen aus abgespeicherten Parallelitäten
  [~, LEG_Names, ~, Coupling] = parroblib_load_robot(RobName, 0);
  Gnum = Coupling(1);
  Pnum = Coupling(2);
  Chain_Name = LEG_Names{1};
  SName_TechJoint = parroblib_format_robot_name(RobName, 2);
  Chain_StructNameAct = parroblib_format_robot_name(RobName, 1);
  Chain_StructName = strrep(Chain_StructNameAct, '\underline', '');
  I_found = strcmp(ResTab_NameTrans.PKM_Name, RobName);
  Row_i = {RobName, Gnum, Pnum, Chain_Name, Chain_StructName, Chain_StructNameAct, SName_TechJoint};
  if any(I_found) % eintragen
    ResTab_NameTrans(I_found,:) = Row_i;
  else % anhängen
    ResTab_NameTrans = [ResTab_NameTrans; Row_i]; %#ok<AGROW>
  end
end

%% Speichere das wieder ab
writetable(ResTab_NameTrans, TablePath, 'Delimiter', ';');
if verbosity > 0
  fprintf('Tabelle %s geschrieben\n', TablePath);
end
