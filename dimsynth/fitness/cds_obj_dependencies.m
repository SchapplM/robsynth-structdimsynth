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
%   Struktur mit diversen zu berechnenden Größen des Roboters im
%   Zeitverlauf. Bei Berechnung der Regressorform sind die Größen selbst
%   nicht gesetzt.Felder:
%   TAU
%     Alle Antriebsmomente (in den aktiven Gelenken). Enthält beide nachfolgenden Effekte.
%   TAU_ID
%     Antriebsmomente zur Kompensation der inversen Dynamik
%     (Massenträgheit, Flieh-/Corioliskraft, Gravitation)
%   TAU_spring
%     Antriebsmomente zur Kompensation des Einflusses von Gelenkfedern
%     (Sonderfall für Festkörpergelenke oder zusätzliche Federn in passiven Gelenken von PKM)
%   TAU_ext
%     Antriebsmomente zur Kompensation externer Kräfte aus Traj_0.Fext
%   TAU_ID_reg
%     Regressormatrix für TAU_ID
%   TAU_spring_reg
%     Regressormatrix für TAU_spring
%   Wges
%     Alle Schnittkräfte (in allen Gelenken; bei PKM für alle Beinketten)
%     Indizes: Siehe SerRob/internforce_traj (1: Zeit, 2:Kraft/Moment) oder
%     PerRob/internforce_traj (1:Kraft/Moment, 2:Beinketten, 3:Zeit)
%     Enthält die beiden nachfolgend beschriebenen Effekte.
%   Wges_ID
%     Schnittkräfte resultierend aus der inversen Dynamik (analog zu TAU_ID)
%   Wges_spring
%     Schnittkräfte resultierend aus Gelenkfeder (analog zu TAU_spring)
%   Wges_ID_reg
%     Regressormatrix zu Wges_ID.
%   Wges_spring_reg
%     Regressormatrix zu Wges_spring.
%
% Siehe auch: cds_dimsynth_robot.m, cds_obj_dependencies_regmult.m.
% Die Logik zur Auswahl der Terme zur Berechnung ist damit konsistent.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-10
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function data_dyn = cds_obj_dependencies(R, Traj_0, Set, Structure, Q, QD, QDD, JinvE_ges)
data_dyn = struct('content', 'cds_obj_dependencies');

if ~Structure.calc_dyn_reg && ~Set.general.debug_calc && ~Structure.calc_cut && ...
    ~Structure.calc_dyn_act && ~Structure.calc_spring_reg && ~Structure.calc_spring_act
  % Es soll nichts berechnet werden.
  return
end
%% Initialisierung
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

%% Berechnungen durchführen
if Structure.calc_cut
  % Berechne die Schnittkräfte in allen Segmenten
  if R.Type == 0 % Seriell
    [Wges_ID, Wges_ID_reg] = R.internforce_traj(Q, QD, QDD);
  else % PKM
    if Structure.calc_dyn_reg || Set.general.debug_calc
      % Berechne nur die Regressormatrizen der Schnittkraft
      Wges_ID_reg = R.internforce_regmat_traj(Q, QD, QDD, XP, XPD, XPDD, JinvP_ges);
    else
      % Die Schnittkraft für diesen Fall wird ganz unten berechnet
    end
    if Structure.calc_spring_reg
      % Regressor-Matrix für Schnittkraft bezogen auf Gelenkmomente.
      % Notwendig für Entwurfsoptimierung bei Gelenkelastizität.
      Wges_spring_reg = R.internforce_regmat_traj(Q, Q, Q, XP, XP, XP, ...
        JinvP_ges, ones(size(Q,1), R.NJ));
    end
  end
end

if Structure.calc_dyn_act
  if R.Type == 0 % Serieller Roboter
    if ~Structure.calc_cut
      if Structure.calc_dyn_reg || Set.general.debug_calc
        TAU_ID_reg = R.invdynregmat_traj(Q, QD, QDD);
      end
      if ~Structure.calc_dyn_reg || Set.general.debug_calc
        % Antriebskräfte berechnen
        TAU_ID = R.invdyn2_traj(Q, QD, QDD);
      end
    elseif ~Structure.calc_dyn_reg
      % Hole Inverse Dynamik aus den zuvor berechneten Schnittkräften
      I_actjoint = (R.MDH.sigma==1).*(3+(3:3:3*R.NJ))' + ... % Schubgelenke -> Kräfte
                   (R.MDH.sigma==0).*(6+3*R.NJ+(3:3:3*R.NJ))';% Drehgelenke -> Momente
      TAU_ID = Wges_ID(:,I_actjoint);
    elseif Set.general.debug_calc
      TAU_ID = R.invdyn2_traj(Q, QD, QDD);
    else
      % Hier ist keine Berechnung der Antriebskräfte notwendig, da dies
      % später mit der Regressorform (der Schnittkräfte) gemacht wird
    end
  else % PKM
    % Antriebskräfte berechnen (Momente im Basis-KS, nicht x-Koord.)
    if ~Structure.calc_cut || Set.general.debug_calc
      % Regressorform nur berechnen, falls später benötigt
      if Structure.calc_dyn_reg || Set.general.debug_calc
        [TAU_ID, TAU_ID_reg] = R.invdyn2_actjoint_traj(Q, QD, QDD, XP, XPD, XPDD, JinvP_ges);
      else
        TAU_ID = R.invdyn2_actjoint_traj(Q, QD, QDD, XP, XPD, XPDD, JinvP_ges);
      end
    elseif ~Structure.calc_dyn_reg || Set.general.debug_calc
      % Die Schnittkräfte werden für PKM mit gegebenen Antriebskräften
      % berechnet (anderer Rechenweg als bei seriellen Robotern)
      TAU_ID = R.invdyn2_actjoint_traj(Q, QD, QDD, XP, XPD, XPDD, JinvP_ges);
    else
      % Nichts berechnen. Begründung s.o.
    end
  end
end
if Structure.calc_spring_act && R.Type ~= 0
  % Antriebskräfte berechnen (Momente im Basis-KS, nicht x-Koord.)
  if ~Structure.calc_cut
    % Regressorform nur berechnen, falls später benötigt
    if Structure.calc_spring_reg
      [TAU_spring, TAU_spring_reg] = R.jointtorque_actjoint_traj(Q, ...
        XP, R.springtorque_traj(Q), JinvP_ges);
    else
      TAU_spring = R.jointtorque_actjoint_traj(Q, XP, R.springtorque_traj(Q), JinvP_ges);
    end
  elseif ~Structure.calc_spring_reg
    TAU_spring = R.jointtorque_actjoint_traj(Q, XP, R.springtorque_traj(Q), JinvP_ges);
  else
    % Nichts berechnen. Begründung s.o.
  end
end

if R.Type ~= 0 && (Structure.calc_cut && ~Structure.calc_dyn_reg || Set.general.debug_calc)
  if Set.general.debug_calc
    % Für die Test-Rechnungen wird die Schnittkraft benötigt. Für diese
    % muss aber das Antriebsmoment berechnet sein.
    % Die Abfrage von oben greift nicht (wegen calc_dyn_act)
    TAU_ID = R.invdyn2_actjoint_traj(Q, QD, QDD, XP, XPD, XPDD, JinvP_ges);
  end
  Wges_ID = R.internforce_traj(Q, QD, QDD, TAU_ID);
end

if R.Type ~= 0 && (Structure.calc_cut && ~Structure.calc_spring_reg && ...
    Set.optimization.joint_stiffness_passive_revolute)
  Wges_spring = R.internforce_traj(Q, 0*QD, 0*QDD, TAU_spring, R.springtorque_traj(Q));
end

% Externe Kräfte berücksichtigen. Kräfte greifen in Richtung der
% EE-Koordinaten an und wirken daher entgegen der Richtung der Antriebe:
% M*xDD + C + G = F_m + F_ext
if any(Traj_0.Fext(:))
  if R.Type == 0, I_qa = R.MDH.mu == 1;
  else,           I_qa = R.I_qa;
  end
  data_dyn.TAU_ext = zeros(length(Traj_0.t), sum(I_qa));
  for i = 1:length(Traj_0.t)
    if R.Type == 0
      J = R.jacobig(Q(i,:)');
      tau_aext = - J' * Traj_0.Fext(i,:)'; % negatives Vorzeichen, s.o.
    else
      Ja_inv_E = reshape(JinvE_ges(i,:), R.NJ, sum(R.I_EE));
      Ja_inv_E_3T3R = zeros(sum(R.I_qa), 6);
      Ja_inv_E_3T3R(:, R.I_EE) = Ja_inv_E(R.I_qa, :);
      Tw = [eye(3,3), zeros(3,3); zeros(3,3), euljac(Traj_0.X(i,4:6)', R.phiconv_W_E)];
      Jg_inv_E_3T3R = Ja_inv_E_3T3R / Tw;
      tau_aext = - Jg_inv_E_3T3R' \ Traj_0.Fext(i,:)';
    end
    data_dyn.TAU_ext(i,:) = tau_aext;
  end
  if Structure.calc_cut
    if R.Type == 0
      data_dyn.W_ext = zeros(length(Traj_0.t), size(Wges_ID,2));
      for i = 1:length(Traj_0.t)
        W_ext_i = R.internforce_ext(Q(i,:)', Traj_0.Fext(i,:)', R.I_EElink, zeros(3,1));
        data_dyn.W_ext(i,:) = W_ext_i(:);
      end
    else
      data_dyn.W_ext = R.internforce_traj(Q, 0*QD, 0*QDD, data_dyn.TAU_ext, zeros(size(Q)));
    end
  end
  if Set.general.debug_calc
    % Prüfe Leistung der externen Kraft gegen resultierende Antriebskraft
    for i = 1:length(Traj_0.t)
      p_act = data_dyn.TAU_ext(i,:) * QD(i,I_qa)';
      T_phiW = euljac_mex(Traj_0.X(i, 4:6)', R.phiconv_W_E);
      p_ext = [Traj_0.XD(i,1:3)'; T_phiW*Traj_0.XD(i,4:6)']' * Traj_0.Fext(i,:)';
      assert(abs(p_act - -p_ext) < 1e-8, 'Leistung aus externer Kraft ist nicht konsistent mit Antrieben');
    end
  end
end
%% Ausgabe belegen 
if ~Structure.calc_dyn_reg && Structure.calc_dyn_act
  data_dyn.TAU_ID = TAU_ID;
end
if ~Structure.calc_spring_reg && Structure.calc_spring_act
  data_dyn.TAU_spring = TAU_spring;
end
if ~Structure.calc_dyn_reg && Structure.calc_cut
  data_dyn.Wges_ID = Wges_ID;
end
if ~Structure.calc_spring_reg && Structure.calc_cut && ...
    Set.optimization.joint_stiffness_passive_revolute
  data_dyn.Wges_spring = Wges_spring;
end
if Structure.calc_dyn_reg || Set.general.debug_calc
  if ~Structure.calc_cut && Structure.calc_dyn_act
    data_dyn.TAU_ID_reg = TAU_ID_reg;
  elseif Structure.calc_cut
    data_dyn.Wges_ID_reg = Wges_ID_reg;
  end
end
if Structure.calc_spring_reg
  if ~Structure.calc_cut && Structure.calc_spring_act
    data_dyn.TAU_spring_reg = TAU_spring_reg;
  elseif Structure.calc_cut
    data_dyn.Wges_spring_reg = Wges_spring_reg;
  end
end

if isfield(data_dyn, 'TAU_ID')
  if ~Set.optimization.joint_stiffness_passive_revolute
    data_dyn.TAU = data_dyn.TAU_ID;
  end
  % Drehfeder in Gelenken berücksichtigen (nur PKM). Gebe die Summe aus den
  % Effekten der inversen Dynamik und der Gelenkelastizitäten aus.
  % Gebe diese Werte nur aus, wenn nicht die Regressorform später benutzt
  % werden soll.
  if isfield(data_dyn, 'TAU_spring') && (~isfield(data_dyn, 'Wges_spring_reg') || ...
      ~isfield(data_dyn, 'TAU_spring_reg'))
    data_dyn.TAU = data_dyn.TAU_ID + data_dyn.TAU_spring;
  end
  % Zusätzlich externe Kraft berücksichtigen
  if isfield(data_dyn, 'TAU_ext')
    data_dyn.TAU = data_dyn.TAU + data_dyn.TAU_ext;
  end
end

if isfield(data_dyn, 'Wges_ID')
  % Das gleiche wie im vorherigen Block, nur für Schnittkräfte statt Antriebskräfte.
  if ~Set.optimization.joint_stiffness_passive_revolute
    data_dyn.Wges = data_dyn.Wges_ID;
  end
  if isfield(data_dyn, 'Wges_spring') && ~isfield(data_dyn, 'Wges_spring_reg')
    data_dyn.Wges = data_dyn.Wges_ID + data_dyn.Wges_spring;
  end
  if isfield(data_dyn, 'W_ext')
    data_dyn.Wges = data_dyn.Wges + data_dyn.W_ext;
  end
end

