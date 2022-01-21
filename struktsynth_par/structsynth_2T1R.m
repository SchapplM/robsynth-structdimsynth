% Erzeuge PKM mit EE-FG 2T1R  (mit allgemeinen Beinketten)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear

settings = struct( ...
  'EE_FG', logical([1 1 0 0 0 1]), ... % 2T1R
  'onlyspherical', false, ... % Allgemeine Beinketten
  'onlygeneral', true, ...
  'dryrun', true); 
  
parroblib_add_robots_symact(settings);