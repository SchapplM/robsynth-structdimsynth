% Erzeuge PKM mit EE-FG 3T0R

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-03
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear

settings = struct( ...
  'EE_FG_Nr', 2, ... % 3T0R
  'dryrun', false, ...
  'max_actuation_idx', 1); 
  
parroblib_add_robots_symact(settings);