% Berechne Abhängigkeiten für die Zielfunktionen (Auslagerung von
% Vorberechnungen)
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Structure
%   Eigenschaften der Roboterstruktur
% Q, QD, QDD
%   Gelenkpositionen und -geschwindigkeiten des Roboters (für PKM auch
%   passive Gelenke)
% Jinvges
%   Zeilenweise (inverse) Jacobi-Matrizen des Roboters (für PKM). Bezogen
%   auf vollständige Gelenkgeschwindigkeiten und Plattform-Geschw. (in
%   x-Koordinaten, nicht: Winkelgeschwindigkeit)
% 
% Ausgabe:
% data_dyn
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
%   Wges_reg
%     Regressormatrix zu Wges.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function data_dyn = cds_obj_dependencies(R, Traj_0, Set, Structure, Q, QD, QDD, Jinv_ges)
data_dyn = struct('content', 'cds_obj_dependencies');

if ~Structure.calc_reg && ~Set.general.debug_calc && ~Structure.calc_dyn_cut && ~Structure.calc_dyn_act
  % Es soll nichts berechnet werden.
  return
end

XE = Traj_0.X;
XED = Traj_0.XD;
XEDD = Traj_0.XDD;

if Structure.calc_dyn_cut
  % Berechne die Schnittkräfte in allen Segmenten
  if R.Type == 0 % Seriell
    [Wges, Wges_reg] = R.internforce_traj(Q, QD, QDD);
  else % PKM
    if Structure.calc_reg || Set.general.debug_calc
      % Berechne nur die Regressormatrizen der Schnittkraft
      Wges_reg = R.internforce_regmat_traj(Q, QD, QDD, Traj_0.X, Traj_0.XD, Traj_0.XDD, Jinv_ges);
    else
      % Die Schnittkraft für diesen Fall wird ganz unten berechnet
    end
  end
end

if Structure.calc_dyn_act
  if R.Type == 0 % Serieller Roboter
    if ~Structure.calc_dyn_cut
      if Structure.calc_reg || Set.general.debug_calc
        TAU_reg = R.invdynregmat_traj(Q, QD, QDD);
      end
      if ~Structure.calc_reg || Set.general.debug_calc
        % Antriebskräfte berechnen
        TAU = R.invdyn2_traj(Q, QD, QDD);
      end
    elseif ~Structure.calc_reg
      % Hole Inverse Dynamik aus den zuvor berechneten Schnittkräften
      I_actjoint = (R.MDH.sigma==1).*(3+(3:3:3*R.NJ))' + ... % Schubgelenke -> Kräfte
                   (R.MDH.sigma==0).*(6+3*R.NJ+(3:3:3*R.NJ))';% Drehgelenke -> Momente
      TAU = Wges(:,I_actjoint);
    elseif Set.general.debug_calc
      TAU = R.invdyn2_traj(Q, QD, QDD);
    else
      % Hier ist keine Berechnung der Antriebskräfte notwendig, da dies
      % später mit der Regressorform (der Schnittkräfte) gemacht wird
    end
  else % PKM
    % Antriebskräfte berechnen (Momente im Basis-KS, nicht x-Koord.)
    if ~Structure.calc_dyn_cut || Set.general.debug_calc
      % Regressorform nur berechnen, falls später benötigt
      if Structure.calc_reg  || Set.general.debug_calc
        [TAU, TAU_reg] = R.invdyn2_actjoint_traj(Q, QD, QDD, XE, XED, XEDD, Jinv_ges);
      else
        TAU = R.invdyn2_actjoint_traj(Q, QD, QDD, XE, XED, XEDD, Jinv_ges);
      end
    elseif ~Structure.calc_reg || Set.general.debug_calc
      % Die Schnittkräfte werden für PKM mit gegebenen Antriebskräften
      % berechnet (anderer Rechenweg als bei seriellen Robotern)
      TAU = R.invdyn2_actjoint_traj(Q, QD, QDD, XE, XED, XEDD, Jinv_ges);
    else
      % Nichts berechnen. Begründung s.o.
    end
  end
end

if R.Type ~= 0 && (Structure.calc_dyn_cut && ~Structure.calc_reg || Set.general.debug_calc)
  if Set.general.debug_calc
    % Für die Test-Rechnungen wird die Schnittkraft benötigt. Für diese
    % muss aber das Antriebsmoment berechnet sein.
    % Die Abfrage von oben greift nicht (wegen calc_dyn_act)
    TAU = R.invdyn2_actjoint_traj(Q, QD, QDD, XE, XED, XEDD, Jinv_ges);
  end
  Wges = R.internforce_traj(Q, QD, QDD, TAU);
end

if ~Structure.calc_reg && Structure.calc_dyn_act
  data_dyn.TAU = TAU;
  if Structure.calc_dyn_cut
    data_dyn.Wges = Wges;
  end
end
if Structure.calc_reg || Set.general.debug_calc
  if ~Structure.calc_dyn_cut && Structure.calc_dyn_act
    data_dyn.TAU_reg = TAU_reg;
  elseif Structure.calc_dyn_cut
    data_dyn.Wges_reg = Wges_reg;
  end
end