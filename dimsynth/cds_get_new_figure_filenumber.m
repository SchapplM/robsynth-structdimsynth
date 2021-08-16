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
% currgen, currind
%   Aktuelle Generation und Individuum der laufenden Optimierung
% currimg
%   Laufende Nummer des neu hinzuzufügenden Bildes für diese Generation
% resdir
%   Verzeichnis zum Speichern der Bilder

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [currgen,currind,currimg,resdir] = cds_get_new_figure_filenumber(Set, Structure, suffix)
resdir = fullfile(Set.optimization.resdir, Set.optimization.optname, ...
  'tmp', sprintf('%d_%s', Structure.Number, Structure.Name));
% Finde die aktuelle Generation anhand der bisher gespeicherten
% Fitness-Werte heraus. Benutze keine mat-Dateien, sondern persistente Var.
PSO_Detail_Data = cds_save_particle_details([], [], 0, 0, NaN, NaN, NaN, NaN, 'output');
if isempty(PSO_Detail_Data)
  % Die Speicherfunktion wurde nicht initialisiert. Voraussichtlich Aufruf
  % außerhalb der Maßsynthese. Bild-Speicherung sowieso nicht sinnvoll.
  currgen = -1; currimg = -1; currind = -1;% Dummy-Werte. Wird dann im Dateinamen benutzt.
  return
end
% Aktuelle Generation und Individuum durch Vergleich der geladenen Daten
[currgen, currind] = cds_load_particle_details(PSO_Detail_Data, NaN(size(PSO_Detail_Data.fval,2),1));

imgfiles = dir(fullfile(resdir, sprintf('Gen%02d_Ind%02d_Eval*_%s*',currgen,currind,suffix)));
if isempty(imgfiles)
  % Es liegen noch keine Bild-Dateien vor.
  currimg = 1;
else
  [tokens_img,~] = regexp(imgfiles(end).name,sprintf('Gen%02d_Ind%02d_Eval(\\d+)_%s',currgen,currind,suffix),'tokens','match');
  currimg = str2double(tokens_img{1}{1})+1;
end
