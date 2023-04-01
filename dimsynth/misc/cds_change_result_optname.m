% Ändere den Namen einer Optimierung nachträglich
% (Notwendig, da Optimierungsname zur Reproduktion benutzt wird, eine reine
% Umbenennung des Ordners funktioniert nicht)
% 
% Eingabe:
% OptDir
%   Verzeichnis des Ergebnis-Ordners (für den umzubenennenden Ergebnisordner)
% OptName_new
%   Neuer Name der Optimierung

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function cds_change_result_optname(OptDir, OptName_new)

[resdir, OptName_old] = fileparts(OptDir);

% Ändere Einstellungsdatei
setfile_old = fullfile(resdir, OptName_old, [OptName_old, '_settings.mat']);
if ~exist(setfile_old, 'file')
  warning('Einstellungsdatei fehlt: %s', setfile_old);
  return
end
tmp = load(setfile_old);
Set = tmp.Set;
Structures = tmp.Structures;
Traj = tmp.Traj;
Set.optimization.optname = OptName_new;
setfile_new = fullfile(resdir, OptName_old, [OptName_new, '_settings.mat']);
save(setfile_new, 'Set', 'Structures', 'Traj');
delete(setfile_old);

% Umbenennung der Tabelle
movefile(fullfile(resdir, OptName_old, [OptName_old, '_results_table.csv']), ...
         fullfile(resdir, OptName_old, [OptName_new, '_results_table.csv']));

% Änderung in Pareto-Bildern
paretofigs = dir(fullfile(resdir, OptName_old, 'Pareto_Gesamt_*'));
for i = 1:length(paretofigs)
  filepath_i = fullfile(resdir, OptName_old, paretofigs(i).name);
  [~, ~, ext] = fileparts(filepath_i);
  if strcmp(ext, '.fig')
    uiopen(filepath_i,1);
    set(gcf, 'CreateFcn', @(src, dummy)cds_paretoplot_createfcn(src, dummy, OptName_new));
    saveas(gcf, filepath_i);
    close(gcf);
  end
end

% Umbenennung des Ordners
movefile(fullfile(resdir, OptName_old), fullfile(resdir, OptName_new));
