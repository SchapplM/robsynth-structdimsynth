% Erzeuge PKM mit EE-FG 3T0R auf dem Rechencluster
% Rechne dadurch alle 12 Gestell- und Plattform-Kombinationen sowie all- 
% gemeine und Varianten-Beinketten parallel aus (24 parallele Instanzen).

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear

for select_variants = [false, true]
  settings = struct( ...
    'check_existing', true, ... % alle existierenden nochmal prüfen
    'check_missing', true, ...  % fehlende hinzufügen
    'EE_FG_Nr', 2, ... % 3T0R
    'comp_cluster', true, ... % auf Cluster rechnen
    'check_resstatus', 0:8, ... % Alle testen
    'selectgeneral', ~select_variants, ... % entweder nur allgemeine Beinketten
    'selectvariants', select_variants, ... % oder nur Varianten (dadurch stärker parallelisiert)
    'max_actuation_idx', 4); % Alle möglichen Aktuierungen prüfen
  pause(1.0); % damit nicht zwei gleiche Zeitstempel entstehen.
  parroblib_add_robots_symact(settings);
end