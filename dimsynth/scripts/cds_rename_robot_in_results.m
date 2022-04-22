% Umbenennen eines Roboters in den Ergebnissen der Maßsynthese
% Wird benutzt, wenn in der Roboter-Datenbank ein Roboter umbenannt wird
% und die Ergebnisse mit der aktuellen Programmversion reproduziert werden
% 
% Eingabe:
% OptName
%   Name der Optimierung (z.B. 'ARK_3T2R_20220114_plfmorph')
% RobNameOld
%   Vorheriger Name des Roboters in der Optimierung (z.B. 'P5RRPRR4G2P8')
% RobNameNew
%   Neuer Name nach Umbenennung in Datenbank (z.B. 'P5RRPRR4G1P8')
% ResDir
%   Ordner, in dem die Maßsynthese-Ergebnisse liegen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-04
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function cds_rename_robot_in_results(OptName, RobNameOld, RobNameNew, ResDir)

if strcmp(RobNameOld, RobNameNew), return; end

fl = dir(fullfile(ResDir, OptName, '*'));
% Dateien und Ordner umbenennen
for i = 1:length(fl)
  oldname = fl(i).name;
  newname = strrep(oldname, RobNameOld, RobNameNew);
  if strcmp(oldname, newname), continue; end
  fprintf('Umbenennen: %s -> %s\n', oldname, newname);
  movefile(fullfile(fl(i).folder, oldname), fullfile(fl(i).folder, newname));
end

% In Ergebnis-Mat-Dateien umbenennen
f_endmat = dir(fullfile(ResDir, OptName, '*_Endergebnis.mat'));
for i = 1:length(f_endmat)
  d_i = load(fullfile(f_endmat(i).folder, f_endmat(i).name));
  oldname = d_i.RobotOptRes.Structure.Name;
  newname = strrep(oldname, RobNameOld, RobNameNew);
  if strcmp(oldname, newname), continue; end
  fprintf('Umbenennen in Datei: %s -> %s\n', oldname, newname);
  d_i.RobotOptRes.Structure.Name = newname;
  save(fullfile(f_endmat(i).folder, f_endmat(i).name), '-struct', 'd_i');
end

% In CSV-Tabelle ändern
fcsv = fullfile(ResDir, OptName, [OptName, '_results_table.csv']);
ResTab = readtable(fcsv, 'ReadVariableNames', true);
if any(strcmp(ResTab.Name, RobNameOld))
  ResTab.Name = strrep(ResTab.Name, RobNameOld, RobNameNew);
  fprintf('CSV geändert: %s -> %s\n', RobNameOld, RobNameNew);
  writetable(ResTab, fcsv, 'Delimiter', ';');
end

% In allgemeinen Mat-Dateien ändern
setfile = fullfile(ResDir, OptName, [OptName, '_settings.mat']);
d1 = load(setfile);
changed = false;
for i = 1:length(d1.Structures)
  if contains(d1.Structures{i}.Name, RobNameOld)
    d1.Structures{i}.Name = strrep(d1.Structures{i}.Name, RobNameOld, RobNameNew);
    fprintf('Name %s in settings.mat gesetzt\n', RobNameNew);
    changed = true;
  end
end
if changed
  save(setfile, '-struct', 'd1')
end