% Lade die Ergebnis-Tabelle für eine durchgeführte Maßsynthese
% Eingabe:
% Set
%   Einstellungsstruktur, siehe cds_settings_defaults.m
% Ausgabe:
% ResTab
%   Ergebnis-Tabelle; siehe cds_results_table.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2024-11
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function ResTab = cds_load_results_table(Set)

resmaindir = fullfile(Set.optimization.resdir, Set.optimization.optname);
tablepath = fullfile(resmaindir, [Set.optimization.optname, '_results_table.csv']);
if ~exist(tablepath, 'file')
  ResTab = [];
  return
end
% Die Tabelle hat zwei Überschriftszeilen. Daher zweimaliges öffnen notwendig
ResTab = readtable(tablepath, 'HeaderLines', 1, 'VariableNamingRule', 'preserve');
ResTab_headers = readtable(tablepath, 'ReadVariableNames', true);
ResTab.Properties.VariableNames = ResTab_headers.Properties.VariableNames;
