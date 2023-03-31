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

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-03
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function cds_paretoplot_createfcn(fighdl, dummy, OptName)

%% Lade die Daten
% Benutze den Ordner als Speicherort der Daten, in dem auch das Bild liegt.
resdir_opt = fileparts(get(fighdl, 'FileName'));
if isempty(resdir_opt)
  % Das Bild wurde eventuell gerade erst gezeichnet und daher ist kein
  % Dateiname abgespeichert. Suche den Ordner der Ergebnisse.
  resdir = fullfile(fileparts(which('structgeomsynth_path_init.m')), 'results');
  resdir_opt = fullfile(resdir, OptName);
  if ~exist(resdir_opt, 'file')
    warning('Automatisch ermittelter Ergebnis-Ordner %s existiert nicht. Abbruch.', resdir_opt);
    return
  end
elseif ~exist(resdir_opt, 'file')
  warning('Ergebnis-Ordner %s existiert nicht, obwohl Bild von dort geladen wurde. Abbruch.', resdir_opt);
  return
end

%% Trage die ButtonDownFcns ein
fch = get(gcf, 'Children');
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
