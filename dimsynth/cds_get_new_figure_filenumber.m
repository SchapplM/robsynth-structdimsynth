% Herausfinden des Dateinamens für ein abzuspeicherndes Bild. Die
% Dateinamen werden im Laufe des PSO-Algorithmus generiert und enthalten
% laufende Nummern der Generationen und Individuen
% 
% Eingabe:
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Structure
%   Eigenschaften der Roboterstruktur (aus cds_gen_robot_list.m)
% suffix
%   Name des zu speichernden Bildes
% 
% Ausgabe:
% currgen
%   Aktuelle Generation der zu speichernden Bilder
% currimg
%   Laufende Nummer des neu hinzuzufügenden Bildes für diese Generation
% resdir
%   Verzeichnis zum Speichern der Bilder

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [currgen,currimg,resdir] = cds_get_new_figure_filenumber(Set, Structure, suffix)
resdir = fullfile(Set.optimization.resdir, Set.optimization.optname, ...
  'tmp', sprintf('%d_%s', Structure.Number, Structure.Name));
matfiles = dir(fullfile(resdir, 'PSO_Gen*.mat'));
if isempty(matfiles)
  % Es liegen noch keine .mat-Dateien vor. Also wird aktuell die erste
  % Generation berechnet
  currgen = 0;
else
  [tokens_mat,~] = regexp(matfiles(end).name,'PSO_Gen(\d+)','tokens','match');
  currgen = str2double(tokens_mat{1}{1})+1; % Es fängt mit Null an
end
imgfiles = dir(fullfile(resdir, sprintf('PSO_Gen%02d_FitEval*_%s*',currgen,suffix)));
if isempty(imgfiles)
  % Es liegen noch keine Bild-Dateien vor.
  currimg = 0;
else
  [tokens_img,~] = regexp(imgfiles(end).name,sprintf('PSO_Gen%02d_FitEval(\\d+)_%s',currgen,suffix),'tokens','match');
  currimg = str2double(tokens_img{1}{1})+1;
end
end
