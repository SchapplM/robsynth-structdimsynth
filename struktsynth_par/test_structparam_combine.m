% Testskript für Funktion structparam_combine.
% Spiele bekannte Fälle aus der PKM-Struktursynthese durch und prüfe, ob
% das logisch erwartete Ergebnis herauskommt.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-11
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc

for i = 0:7
  % Testfälle durchgehen
  if i == 0
    angles_valid = {};
    angles_red_gt = {};
  elseif i == 1
    angles_valid = {'p','o','a'};
    angles_red_gt = {'a'};
  elseif i == 2
    angles_valid = {'p','o'};
    angles_red_gt = {'b'};
  elseif i == 3
    angles_valid = {'p','a'};
    angles_red_gt = {'a'};
  elseif i == 4
    angles_valid = {'a','o'};
    angles_red_gt = {'a'};
  elseif i == 5
    angles_valid = {'pp','po','pa'};
    angles_red_gt = {'pa'};
  elseif i == 6
    angles_valid = {'pp','po'};
    angles_red_gt = {'pb'};
  elseif i == 7
    angles_valid = {'ap','pa'};
    angles_red_gt = {'ap','pa'};
  end
  % Zusammenfassung der Strukturparameter per Funktionsaufruf
  angles_red = structparam_combine(angles_valid);
  % Prüfen, ob Testfälle erfüllt
  if length(angles_red) ~= length(angles_red_gt)
    error('Anzahl der zusammengefassten Strukturparameter passt nicht: %d vs %d', ...
      length(angles_red), length(angles_red_gt));
  end
  if length(intersect(angles_red, angles_red_gt)) ~= length(angles_red_gt)
    error('Die Zusammenfassung der Strukturparameter passt nicht.');
  end
end
