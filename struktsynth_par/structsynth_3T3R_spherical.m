% Erzeuge PKM mit EE-FG 3T3R und Kugelgelenk-Enden der Beinketten

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-03
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear

settings = struct( ...
  'EE_FG_Nr', 5, ... % 3T3R
  'onlyspherical', true, ...
  'onlygeneral', false, ... % Kugelgelenk-Beinketten sind spezielle Varianten
  'dryrun', true); 
  
parroblib_add_robots_symact