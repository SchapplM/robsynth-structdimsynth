% Erzeuge die ButtonDownFcns für alle Marker im Pareto-Diagramm
% Wird nicht direkt bei der Erzeugung gemacht, da diese Fcns nicht gut in
% dem Matlab-Figure-Format gespeichert werden können (Datei wird groß).
% Diese Funktion ist eine CreateFcn für Figure-Objekte
% 
% Eingabe:
% fighdl, dummy
%   Vorgegebene Eingaben einer CreateFcn
% OptName
%   Name der Optimierung
% oldcreatefcn
%   Ursprüngliche CreateFcn des Figures. Notwendig für unsichtbares
%   Speichern des Bildes und Sichtbarmachen beim Neu-Öffnen
% 
% Beispiele:
% Zum manuellen Eintragen der CreateFcn oder bei Änderung des Namens der
% Optimierung (OptName in Code-Zeile anpassen):
% `set(gcf, 'CreateFcn', @(src, dummy)cds_paretoplot_createfcn(src, dummy, OptName));`
% (Danach Bild neu speichern)
% Zum Eintragen in offenes Bild:
% `cds_paretoplot_createfcn(gcf, [], OptName)`

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-03
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function cds_paretoplot_createfcn(fighdl, dummy, OptName, oldcreatefcn)
if nargin == 4
  eval(oldcreatefcn);
end
%% Trage die ButtonDownFcns ein
fch = get(fighdl, 'Children');
axhdl = fch(strcmp(get(fch, 'type'), 'axes'));
ach = get(axhdl, 'Children');
for i = 1:length(ach)
  hdl = ach(i);
  if ~strcmp(get(hdl, 'type'), 'line')
    continue
  end
  [tokens, ~] = regexp(get(hdl, 'DisplayName'), 'Rob(\d+)_(.*)', 'tokens', 'match');
  if isempty(tokens) % Vermutlich Legenden-Marker
    continue;
  end
  RobNr = str2double(tokens{1}{1});
  RobName = tokens{1}{2};
  % Funktions-Handle zum Anklicken der Datenpunkte eintragen
  ButtonDownFcn=@(src,event)cds_paretoplot_buttondownfcn(src,event,...
    OptName, RobName, RobNr);
  set(hdl, 'ButtonDownFcn', ButtonDownFcn)
end
