% Erzeuge PKM mit EE-FG 2T0R  (mit allgemeinen Beinketten)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-03
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear

settings = struct( ...
  'check_existing', true, ...
  'EE_FG', logical([1 1 0 0 0 0]), ... % 2T0R
  'onlygeneral', true, ...
  'allow_passive_prismatic', true);  % erzeugte PKM ist 2PP

parroblib_add_robots_symact(settings);