% Berechne Abhängigkeiten für die Entwurfsoptimierung durch Ausnutzung der
% Parameterlinearen Form der Dynamik
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% data_dyn
%   Ausgabe von cds_obj_dependencies.m; Struktur mit Regressor-Matrizen für
%   Antriebskräfte und Schnittkräfte
% Q
%   Zeitverlauf der Gelenkwinkel (Bei PKM auch passive Gelenke)
% 
% Ausgabe:
% output
%   Struktur mit Diversen zu berechnenden Größen des Roboters im
%   Zeitverlauf. Felder:
%   TAU
%     Alle Antriebsmomente (in den aktiven Gelenken)
%   Wges
%     Alle Schnittkräfte (in allen Gelenken; bei PKM für alle Beinketten)
%     Indizes: Siehe SerRob/internforce_traj (1: Zeit, 2:Kraft/Moment) oder
%     PerRob/internforce_traj (1:Kraft/Moment, 2:Beinketten, 3:Zeit)
% 
% Siehe auch: cds_obj_dependencies

% Quelle für den Ansatz:
% [SchapplerTapOrt2019] Schappler, M. and Tappe, S., Ortmaier, T.:
% Exploiting Dynamics Parameter Linearity for Design Optimization in
% Combined Structural and Dimensional Robot Synthesis (2019)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-01
% (C) Institut für Mechatronische Systeme, Universität Hannover

function output = cds_obj_dependencies_regmult(R, data_dyn, Q)
output = struct('content', 'cds_obj_dependencies_regmult');

%% Berechne Dynamik in Antriebskoordinaten
if isfield(data_dyn, 'TAU_ID_reg')
  if R.Type == 0 % Serieller Roboter
    TAU_ID = R.invdyn3_traj(data_dyn.TAU_ID_reg);
  else % PKM
    TAU_ID = R.invdyn3_actjoint_traj(data_dyn.TAU_ID_reg);
  end
  output.TAU_ID = TAU_ID;
  output.TAU = TAU_ID;
end
if isfield(data_dyn, 'TAU_spring_reg') && R.Type ~= 0
  TAU_spring = R.jointtorque2_actjoint_traj(data_dyn.TAU_spring_reg, R.springtorque_traj(Q));
  output.TAU_spring = TAU_spring;
end
%% Berechne Schnittkraft
if isfield(data_dyn, 'Wges_ID_reg')
  % Berechne die Schnittkräfte in allen Segmenten
  W_ID_reg = data_dyn.Wges_ID_reg;
  if R.Type == 0 %#ok<IFBDUP> % Seriell
    Wges_ID = R.internforce3_traj(W_ID_reg);
  else % PKM
    Wges_ID = R.internforce3_traj(W_ID_reg);
  end
  output.Wges = Wges_ID;
end

if isfield(data_dyn, 'Wges_spring_reg') && R.Type ~= 0
  % Berechne die Schnittkräfte in allen Segmenten
  Wges_spring_reg = data_dyn.Wges_spring_reg;
  Wges_spring = R.internforce3_traj(Wges_spring_reg, R.springtorque_traj(Q));
  if isfield(data_dyn, 'Wges_ID_reg')
    output.Wges = output.Wges + Wges_spring;
  else
    output.Wges = data_dyn.Wges_ID + Wges_spring;
  end
end

%% Bestimme Antriebskraft aus Schnittkraft
if ~isfield(data_dyn, 'TAU_ID_reg') && isfield(data_dyn, 'Wges_ID_reg')
  % Extrahiere die Dynamik der Antriebe aus den Schnittkräften.
  % Die Schnittkraft in Richtung einer Antriebskoordinate entspricht der
  % Antriebskraft
  if R.Type == 0 % Serieller Roboter
    % Indizes für Schnittkraftvektor. In Vektor erst alle Kräfte und dann
    % alle Momente. Die ersten 3 Einträge gehören jeweils zum Basis-Segment
    % (Nicht für Antriebskräfte relevant)
    I_actjoint = (R.MDH.sigma==1).*(3+(3:3:3*R.NJ))' + ... % Schubgelenke -> Kräfte
                 (R.MDH.sigma==0).*(6+3*R.NJ+(3:3:3*R.NJ))';% Drehgelenke -> Momente
    TAU_ID = Wges_ID(:,I_actjoint);
  else % PKM
    TAU_ID = NaN(size(W_ID_reg,1), sum(R.I_qa));
    for j = 1:R.NLEG
      % Indizes aller Gelenkachsen in den Schnittkräften
      I_joints = (R.Leg(j).MDH.sigma==1) .* (3+(3:3:3*R.Leg(j).NJ)') + ... % erste Einträge entsprechen Schnittkräften und damit Schubgelenken (z-Komponente)
                 (R.Leg(j).MDH.sigma==0) .* (6+3*R.Leg(j).NJ+(3:3:3*R.Leg(j).NJ)');
       % Isolation des aktiven Gelenks
      I_actjoint_j = I_joints(R.I_qa(R.I1J_LEG(j):R.I2J_LEG(j)));
      TAU_ID(:,j) = squeeze(Wges_ID(j, I_actjoint_j, :));
    end
  end
  output.TAU_ID = TAU_ID;
  output.TAU = TAU_ID;
elseif isfield(data_dyn, 'TAU_ID')
  output.TAU_ID = data_dyn.TAU_ID;
  output.TAU = data_dyn.TAU_ID;
end
if ~isfield(data_dyn, 'TAU_spring_reg') && isfield(data_dyn, 'Wges_spring_reg') && R.Type ~= 0
  TAU_spring = NaN(size(data_dyn.Wges_spring_reg,1), sum(R.I_qa));
  for j = 1:R.NLEG
    % Indizes aller Gelenkachsen in den Schnittkräften
    I_joints = (R.Leg(j).MDH.sigma==1) .* (3+(3:3:3*R.Leg(j).NJ)') + ... % erste Einträge entsprechen Schnittkräften und damit Schubgelenken (z-Komponente)
               (R.Leg(j).MDH.sigma==0) .* (6+3*R.Leg(j).NJ+(3:3:3*R.Leg(j).NJ)');
     % Isolation des aktiven Gelenks
    I_actjoint_j = I_joints(R.I_qa(R.I1J_LEG(j):R.I2J_LEG(j)));
    TAU_spring(:,j) = squeeze(Wges_spring(j, I_actjoint_j, :));
  end
  output.TAU_spring = TAU_spring;
end

%% Füge Federmomente hinzu
if isfield(output, 'TAU_spring')
  output.TAU = output.TAU + output.TAU_spring;
end
