% Zielfunktion ("objective function") für Optimierung in der Maßsynthese
% basierend auf der maximal notwendigen Antriebskraft des Roboters.
% Die max. Antriebskraft wird in einen normierten Zielfunktionswert übersetzt
% 
% Eingabe:
% TAU
%   Alle Antriebsmomente (in den aktiven Gelenken)
% 
% Ausgabe:
% fval [1x1]
%   Zielfunktionswert, der im PSO-Algorithmus minimiert wird
% fval_debugtext [char]
%   Zeile mit Hinweistext, der bei PSO nach Fitness-Berechnung ausgegeben wird
% debug_info [cell]
%   Zusatz-Informationen, die im Debug-Bild des Roboters angezeigt werden
% tau_a_max [1x1]
%   Physikalischer Wert, der dem Zielfunktionswert zugrunde liegt
%   Hier: Maximale Antriebskraft aller Antriebe in N bzw. Nm

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-10
% (C) Institut für Mechatronische Systeme, Universität Hannover



function [fval, fval_debugtext, debug_info, tau_a_max] = cds_obj_actforce(TAU, R, Set, Structure)

debug_info = {''};
if nargin == 1 || ~any(Structure.Type==[0,2]) % Altes Format oder Seriell-hybrider Roboter
  % Es wird einfach die größte Antriebskraft minimiert. Das
  % funktioniert nur bei symmetrischer Aktuierung. Ansonsten werden Kräfte
  % mit Momenten vergleichen (nicht bei gemischer Aktuierung sinnvoll).
  tau_a_max_per_actuator = max(abs(TAU));
  fval_debugtext = sprintf('Antriebskräfte max. [%s] N/Nm.', ...
    disp_array(tau_a_max_per_actuator, '%1.2f'));
else
  % Gewichtung der einzelnen Antriebskräfte gemäß der Einstellungen
  if Structure.Type == 2 % PKM
    I_firstjoint = false(R.NJ,1);
    I_firstjoint(R.I1J_LEG) = true;
    I_linact_base = R.MDH.sigma == 1 & R.I_qa & I_firstjoint;
    I_revact_base = R.MDH.sigma == 0 & R.I_qa & I_firstjoint;
    I_linact_chain = R.MDH.sigma == 1 & R.I_qa & ~I_firstjoint;
    I_revact_chain = R.MDH.sigma == 0 & R.I_qa & ~I_firstjoint;
  elseif Structure.Type == 0 % Serieller Roboter
    I_firstjoint = false(R.NJ,1);
    I_firstjoint(1) = true;
    I_linact_base = R.MDH.sigma == 1 & I_firstjoint;
    I_revact_base = R.MDH.sigma == 0 & I_firstjoint;
    I_linact_chain = R.MDH.sigma == 1 & ~I_firstjoint;
    I_revact_chain = R.MDH.sigma == 0 & ~I_firstjoint;
  else
    error('Nicht implementiert');
  end
  II_joints = NaN(R.NJ,1);
  II_joints(R.I_qa) = 1:sum(R.I_qa);
  weighting = NaN(1,sum(R.I_qa));
  set_actforce = Set.optimization.obj_actforce;
  weighting(II_joints(I_linact_base)) = set_actforce.weighting_linear_actuation_base;
  weighting(II_joints(I_linact_chain)) = set_actforce.weighting_linear_actuation_chain;
  weighting(II_joints(I_revact_base)) = set_actforce.weighting_revolute_actuation_base;
  weighting(II_joints(I_revact_chain)) = set_actforce.weighting_revolute_actuation_chain;
  tau_a_max_per_actuator = max(abs(TAU)) .* weighting;
  if strcmp(Structure.act_type, 'mixed')
    unitstr = 'N/Nm';
  elseif strcmp(Structure.act_type, 'revolute')
    unitstr = 'Nm';
  elseif strcmp(Structure.act_type, 'prismatic')
    unitstr = 'N';
  else
    unitstr = 'undefined_unit';
  end
  if any(weighting~=1)
    weightstr = ' (gewichtet)';
  else
    weightstr = '';
  end
  fval_debugtext = sprintf('Antriebskräfte max. [%s] %s%s.', ...
    disp_array(tau_a_max_per_actuator, '%1.2f'), unitstr, weightstr);
end
tau_a_max = max(tau_a_max_per_actuator);
f_actforce_norm = 2/pi*atan((tau_a_max)/100); % Normierung auf 0 bis 1; 620 ist 0.9.
fval = 1e3*f_actforce_norm; % Normiert auf 0 bis 1e3
