% Berechne Abhängigkeiten für die Entwurfsoptimierung durch Ausnutzung der
% Parameterlinearen Form der Dynamik
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% data_dyn
%   Ausgabe von cds_obj_dependencies.m; Struktur mit Regressor-Matrizen für
%   Antriebskräfte und Schnittkräfte
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
%     Indizes: Siehe SerRob/internforce_traj (1: Zeit, 2:Kraft/Moment) oder
%     PerRob/internforce_traj (1:Kraft/Moment, 2:Beinketten, 3:Zeit)
% 
% Siehe auch: cds_obj_dependencies

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-01
% (C) Institut für Mechatronische Systeme, Universität Hannover

function output = cds_obj_dependencies_regmult(R, data_dyn)
output = struct('content', 'cds_obj_dependencies_regmult');
% Berechne Dynamik in Antriebskoordinaten
if isfield(data_dyn, 'TAU_reg')
  TAU_reg = data_dyn.TAU_reg;
  if R.Type == 0 % Serieller Roboter
    TAU = R.invdyn3_traj(TAU_reg);
  else % PKM
    TAU = R.invdyn3_actjoint_traj(TAU_reg);
  end
  output.TAU = TAU;
end

if isfield(data_dyn, 'Wges_reg')
  % Berechne die Schnittkräfte in allen Segmenten
  W_reg = data_dyn.Wges_reg;
  if R.Type == 0 % Seriell
    Wges = R.internforce3_traj(W_reg);
  else % PKM
    Wges = R.internforce3_traj(W_reg);
  end
  output.Wges = Wges;
end

if ~isfield(data_dyn, 'TAU_reg') && isfield(data_dyn, 'Wges_reg')
  % Extrahiere die Dynamik der Antriebe aus den Schnittkräften.
  % Die Schnittkraft in Richtung einer Antriebskoordinate entspricht der
  % Antriebskraft
  if R.Type == 0 % Serieller Roboter
    % Indizes für Schnittkraftvektor. In Vektor erst alle Kräfte und dann
    % alle Momente. Die ersten 3 Einträge gehören jeweils zum Basis-Segment
    % (Nicht für Antriebskräfte relevant)
    I_actjoint = (R.MDH.sigma==1).*(3+(3:3:3*R.NJ))' + ... % Schubgelenke -> Kräfte
                 (R.MDH.sigma==0).*(6+3*R.NJ+(3:3:3*R.NJ))';% Drehgelenke -> Momente
    TAU = Wges(:,I_actjoint);
  else % PKM
    TAU = NaN(size(W_reg,1), sum(R.I_qa));
    for j = 1:R.NLEG
      % Indizes aller Gelenkachsen in den Schnittkräften
      I_joints = (R.Leg(j).MDH.sigma==1) .* (3+(3:3:3*R.Leg(j).NJ)') + ... % erste Einträge entsprechen Schnittkräften und damit Schubgelenken (z-Komponente)
                 (R.Leg(j).MDH.sigma==0) .* (6+3*R.Leg(j).NJ+(3:3:3*R.Leg(j).NJ)');
       % Isolation des aktiven Gelenks
      I_actjoint_j = I_joints(R.I_qa(R.I1J_LEG(j):R.I2J_LEG(j)));
      TAU(:,j) = squeeze(Wges(j, I_actjoint_j, :));
    end
  end
  output.TAU = TAU;
end
