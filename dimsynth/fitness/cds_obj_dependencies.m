% Berechne Abhängigkeiten für die Zielfunktionen (Auslagerung von
% Vorberechnungen)
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
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


if Structure.calc_dyn_cut
  % Berechne die Schnittkräfte in allen Segmenten
  if R.Type == 0 % Seriell
    [Wges, Wges_reg] = R.internforce_traj(Q, QD, QDD);
  else % PKM
    if Structure.calc_reg % || Set.general.debug_calc
      Wges_reg = R.internforce_regmat_traj(Q, QD, QDD, Traj_0.X, Traj_0.XD, Traj_0.XDD, Jinv_ges);
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
      % Dieser Fall kann aber eigentlich nicht eintreten, da immer, wenn
      % die Schnittkräfte berechnet werden müssen (für Entwurfsoptimierung), 
      % auch die Regressorform benutzt wird.
      error('Dieser Fall sollte eigentlich nicht eintreten');
    elseif Set.general.debug_calc
      TAU = R.invdyn2_traj(Q, QD, QDD);
    else
      % Hier ist keine Berechnung der Antriebskräfte notwendig, da dies
      % später mit der Regressorform (der Schnittkräfte) gemacht wird
    end
  else % PKM
    % Trajektorie in Plattform-KS umrechnen
    XE = Traj_0.X;
    XED = Traj_0.XD;
    XEDD = Traj_0.XDD;
    % Antriebskräfte berechnen (Momente im Basis-KS, nicht x-Koord.)
    if ~Structure.calc_dyn_cut || Set.general.debug_calc
      % Regressorform nur berechnen, falls später benötigt
      if Structure.calc_reg  || Set.general.debug_calc
        [TAU, TAU_reg] = R.invdyn2_actjoint_traj(Q, QD, QDD, XE, XED, XEDD, Jinv_ges);
      else
        TAU = R.invdyn2_actjoint_traj(Q, QD, QDD, XE, XED, XEDD, Jinv_ges);
      end
    elseif ~Structure.calc_reg
      error('Dieser Fall sollte eigentlich nicht eintreten'); % Begründung s.o.
    elseif Set.general.debug_calc
      TAU = R.invdyn2_actjoint_traj(Q, QD, QDD, XE, XED, XEDD, Jinv_ges);
    else
      % Nichts berechnen. Begründung s.o.
    end
  end
end

if R.Type ~= 0 && (Structure.calc_dyn_cut && ~Structure.calc_reg || Set.general.debug_calc)
  Wges = R.internforce_traj(Q, QD, QDD, TAU);
end

if ~Structure.calc_reg || Set.general.debug_calc
  data_dyn.TAU = TAU;
  if Structure.calc_dyn_cut
    data_dyn.Wges = Wges;
  end
end
if Structure.calc_reg || Set.general.debug_calc
  if ~Structure.calc_dyn_cut
    data_dyn.TAU_reg = TAU_reg;
  else
    data_dyn.Wges_reg = Wges_reg;
  end
end