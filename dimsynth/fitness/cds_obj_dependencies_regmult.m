% Berechne Abhängigkeiten für die Entwurfsoptimierung durch Ausnutzung der
% Parameterlinearen Form der Dynamik
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% 
% Ausgabe:
% output
%   Struktur mit Diversen zu berechnenden Größen des Roboters im
%   Zeitverlauf. Felder:
%   TAU
%     Alle Antriebsmomente (in den aktiven Gelenken)
%   TAU_reg
%     Regressormatrix für TAU
%   Wges
%     Alle Schnittkräfte (in allen Gelenken; bei PKM für alle Beinketten)
%     Indizes: Siehe SerRob/internforce_traj
%     (1: Zeit, 2:Kraft/Moment)
% 
% Siehe auch: cds_obj_dependencies

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-01
% (C) Institut für Mechatronische Systeme, Universität Hannover

function output = cds_obj_dependencies_regmult(R, Set, input)
output = struct('content', 'cds_obj_dependencies_regmult');
% Berechne Dynamik in Antriebskoordinaten
if any(strcmp(Set.optimization.objective, {'energy', 'minactforce'})) ...
    || Set.optimization.desopt_link_yieldstrength && R.Type == 2
  TAU_reg = input.TAU_reg;
  if R.Type == 0 % Serieller Roboter
    TAU = R.invdyn3_traj(TAU_reg);
  else % PKM
    TAU = R.invdyn3_actjoint_traj(TAU_reg);
  end
  output.TAU = TAU;
end


if Set.optimization.desopt_link_yieldstrength
  % Berechne die Schnittkräfte in allen Segmenten
  W_reg = input.Wges_reg;
  if R.Type == 0 % Seriell
    Wges = R.internforce3_traj(W_reg);
  else % PKM
    % TODO: Schnittkraft-Regressorform implementieren
    Wges = NaN;
  end
  output.Wges = Wges;
end
