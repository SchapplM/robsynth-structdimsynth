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
%   auf vollständige Gelenkgeschwindigkeiten und Endeffektor-Geschw. (in
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

function data_dyn = cds_obj_dependencies(R, Traj_0, Set, Structure, Q, QD, QDD, JinvE_ges)
data_dyn = struct('content', 'cds_obj_dependencies');

if ~Structure.calc_reg && ~Set.general.debug_calc && ~Structure.calc_dyn_cut && ~Structure.calc_dyn_act
  % Es soll nichts berechnet werden.
  return
end

XE = Traj_0.X;
XED = Traj_0.XD;
XEDD = Traj_0.XDD;

if Set.optimization.static_force_only
  % Nur statische Kräfte berechnen (obwohl Trajektorie mit Geschw. gegeben
  % ist).
  % TODO: Effizienter: Nur Gravload-Funktion aufrufen, nicht vollständige
  % Dynamik.
  QD = zeros(size(QD));
  QDD = zeros(size(QDD));
  XED = zeros(size(XED));
  XEDD = zeros(size(XEDD));
  if R.Type == 2
    XP = R.xE2xP_traj(XE);
    XPD = XED; XPDD = XEDD;
  end
elseif R.Type == 2
  [XP, XPD, XPDD] = R.xE2xP_traj(XE, XED, XEDD);
end
% Umrechnung der Jacobi-Matrix auf Plattform-Koordinaten (Dynamik ist nicht
% in EE-Koordinaten definiert, da die Trafo sich ändern kann).
if R.Type == 2
  JinvP_ges = NaN(size(JinvE_ges));
  for i = 1:size(JinvE_ges)
    JinvE_i = reshape(JinvE_ges(i,:), R.NJ, sum(R.I_EE));
    % Bezogen auf EE-Position, Euler-Winkel als Rotation
    JinvE_i_fullx = zeros(R.NJ, 6);
    JinvE_i_fullx(:,R.I_EE) = JinvE_i;
    H_xE = [eye(3,3), zeros(3,3); zeros(3,3), euljac(XE(i,4:6)', R.phiconv_W_E)];
    % Bezogen auf EE-Position, geometrische Rotation
    JinvE_i_fulls = JinvE_i_fullx / H_xE; % [A]/(11)
    % Umrechnung der geometrischen Jacobi auf die Plattform (statt EE)
    T_0_E = R.x2t(XE(i,:)');
    r_P_P_E = R.T_P_E(1:3,4);
    r_E_P_E = R.T_P_E(1:3,1:3)' * r_P_P_E;
    r_0_P_E = T_0_E(1:3,1:3)*r_E_P_E;
    A_E_P = adjoint_jacobian(r_0_P_E);
    JinvP_i_fulls = JinvE_i_fulls * A_E_P;
    % Zurückrechnen auf die Euler-Winkel-Rotation (Plattform)
    H_xP = [eye(3,3), zeros(3,3); zeros(3,3), euljac(XP(i,4:6)', R.phiconv_W_E)];
    JinvP_i_fullx = JinvP_i_fulls * H_xP; % [A]/(12)
    JinvP_i = JinvP_i_fullx(:,R.I_EE);
    JinvP_ges(i,:) = JinvP_i(:);
  end
end

if Structure.calc_dyn_cut
  % Berechne die Schnittkräfte in allen Segmenten
  if R.Type == 0 % Seriell
    [Wges, Wges_reg] = R.internforce_traj(Q, QD, QDD);
  else % PKM
    if Structure.calc_reg || Set.general.debug_calc
      % Berechne nur die Regressormatrizen der Schnittkraft
      Wges_reg = R.internforce_regmat_traj(Q, QD, QDD, XP, XPD, XPDD, JinvP_ges);
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
        [TAU, TAU_reg] = R.invdyn2_actjoint_traj(Q, QD, QDD, XP, XPD, XPDD, JinvP_ges);
      else
        TAU = R.invdyn2_actjoint_traj(Q, QD, QDD, XP, XPD, XPDD, JinvP_ges);
      end
    elseif ~Structure.calc_reg || Set.general.debug_calc
      % Die Schnittkräfte werden für PKM mit gegebenen Antriebskräften
      % berechnet (anderer Rechenweg als bei seriellen Robotern)
      TAU = R.invdyn2_actjoint_traj(Q, QD, QDD, XP, XPD, XPDD, JinvP_ges);
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
    TAU = R.invdyn2_actjoint_traj(Q, QD, QDD, XP, XPD, XPDD, JinvP_ges);
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