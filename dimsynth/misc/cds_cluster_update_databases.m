% Aktualisiere die Ergebnis-Datenbank auf dem Cluster
% Ist notwendig, nachdem eine Maßsynthese durchgeführt wurde, um die
% globale Index-Variable zu aktualisieren (InitPopFromGlobalIndex).
% Das Laden der alten Ergebnisse dauert sonst auf dem Cluster zu lange.
% 
% Eingabe:
% jobIDs_depstruct
%   Struktur mit Feld "afterany" zur Übergabe an die jobStart-Funktion
%   des Cluster-Transfers. Die übergebene Nummer sollte die JobID eines
%   Merge-Jobs sein, die von cds_start ausgegeben wird.
% 
% Ausgabe:
% jobID
%   Nummer des Jobs zum Datenbank-Aktualisieren. Kann wieder als Abhängig-
%   keit zum Starten weiterer Durchläufe der Synthese benutzt werden

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function jobID = cds_cluster_update_databases(jobIDs_depstruct)

if nargin == 0
  jobIDs_depstruct = struct();
end
%% Skript erzeugen
sgpath = fileparts(which('structgeomsynth_path_init.m'));
clusterheaderfile=fullfile(sgpath, 'dimsynth','dimsynth_cluster_header.m');
jobdir = tmpDirFcn(true);
targetfile = fullfile(jobdir, 'update_databases.m');
% Hierdurch Initialisierung der Pfade
copyfile(clusterheaderfile, targetfile);
% Befehl zum Aktualisieren der Datenbank hineinschreiben
fid = fopen(targetfile, 'a');
fprintf(fid, 'cds_gen_init_pop_index([], [], true);\n');
fclose(fid);

%% Skript auf Cluster laden und starten
computation_name = sprintf('update_databases_%s', datestr(now,'yyyymmdd_HHMMSS'));
jobID = jobStart(struct( ...
  'name', computation_name, ...
  'ppn', 1, ... % Eine Node reicht. Skript läuft seriell.
  'matFileName', 'update_databases.m', ...
  'locUploadFolder', jobdir, ...
  'time',5), jobIDs_depstruct); % Angabe in h. Dauert normalerweise ca. 30min.
fprintf('Job zum aktualisieren der Ergebnis-Datenbank hochgeladen. ID: %d\n', jobID);
