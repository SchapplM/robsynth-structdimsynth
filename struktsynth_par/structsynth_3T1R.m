% Erzeuge PKM mit EE-FG 3T1R

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-03
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear

settings = struct( ...
  'EE_FG', logical([1 1 1 0 0 1]), ... % 3T1R
  'dryrun', false, ...
  'max_actuation_idx', 1); % Gestellnahes Gelenk angetrieben
  
parroblib_add_robots_symact(settings);