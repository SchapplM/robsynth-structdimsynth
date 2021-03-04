% Erzeuge PKM mit EE-FG 2T0R  (mit allgemeinen Beinketten)
% Da dieser Fall nur zum Erzeugen einer Platzhalter-PKM ist, muss noch das
% Skript parroblib_add_robots_symact angepasst werden.
% Der Fall 2T1R muss dort manuell auf 2T0R gesetzt werden (letzte 1 bei
% [1 1 0 0 1] entfernen.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-03
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear

settings = struct( ...
  'check_existing', true, ...
  'EE_FG_Nr', 1, ... % 2T0R, falls angepasst.
  'onlygeneral', true, ...
  'allow_passive_revolute', true);  % erzeugte PKM ist 2PP

parroblib_add_robots_symact % TODO: Manuelle Anpassung hier erforderlich.