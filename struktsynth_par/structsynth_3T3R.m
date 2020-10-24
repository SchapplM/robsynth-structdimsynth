% Erzeuge PKM mit EE-FG 3T3R  (mit allgemeinen Beinketten)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-03
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear

settings = struct( ...
  'EE_FG_Nr', 5, ... % 3T3R
  'onlyspherical', false, ... % Allgemeine Beinketten
  'onlygeneral', true, ...
  'parcomp_structsynth', 0, ...
  'parcomp_mexcompile', 0, ...
  'dryrun', true); 
  
parroblib_add_robots_symact
