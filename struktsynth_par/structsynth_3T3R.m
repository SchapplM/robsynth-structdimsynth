% Erzeuge PKM mit EE-FG 3T3R  (mit allgemeinen Beinketten)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-03
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear

settings = struct( ...
  'EE_FG_Nr', 4, ... % 3T3R
  'onlyspherical', false, ... % Allgemeine Beinketten
  'onlygeneral', true, ...
  'dryrun', true); 
  
parroblib_add_robots_symact