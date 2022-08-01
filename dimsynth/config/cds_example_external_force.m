% Teste Synthese mit einer zusätzlichen externen Kraft
% 
% Kraft ist so definiert, dass die Dynamik-Gl. so definiert ist: 
% F_act + F_ext = M*xDD+C(x,xD)+G(x)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

% Definition aller FG
EEFG_Ges = logical(...
  [1 1 0 0 0 1; ...
   1 1 1 0 0 0; ...
   1 1 1 0 0 1; ...
   1 1 1 1 1 0; ...
   1 1 1 1 1 1]);
for i_desopt = 0:1 % ohne und mit Entwurfsoptimierung
for i_FG = 1:size(EEFG_Ges,1) % Alle FG einmal durchgehen
  %% Aufgaben-FG und Trajektorie
  EEFG = EEFG_Ges(i_FG,:);
  if i_FG == 1
    Traj_no = 2;
  elseif i_FG == 4 || i_FG == 5
    Traj_no = 3; % Trajektorie ohne 0-Rotation. TODO: Sollte auch so laufen, tut es aber nicht.
  else
    Traj_no = 1;
  end
  %% Einstellungen der Optimierung
  Set = cds_settings_defaults(struct('DoF', EEFG));
  Set.task.Ts = 1e-2; % sonst Trajektorie zu lang
  Set.task.Tv = 1e-1;
  Traj = cds_gen_traj(EEFG, Traj_no, Set.task);
  % Externe Kraft einfügen (so, dass sie Arbeit verrichtet), Zufallszahlen
  for kk = find(EEFG)
    Traj.Fext(Traj.XD(:,kk)>1e-6,kk) = 100*rand(sum(Traj.XD(:,kk)>1e-6), 1);
  end
  % Entwurfsoptimierung aktivieren
  if i_desopt
    Set.optimization.desopt_vars = {'linkstrength'};
    Set.optimization.constraint_obj(6) = 1; % Materialspannung als Nebenbedingung;
  end
  Set.optimization.objective = {'actforce'};
  Set.optimization.constraint_obj(4) = 1e4; % max. Wert für Konditionszahl (damit Umrechnung EE-Antrieb numerisch gut funktioniert). Erlaube auch recht hohe Werte, damit Maßsynthese nicht daran scheitert.
  if ~isempty(Set.optimization.desopt_vars)
    Set.optimization.obj_limit = 1e3; % nur eine Iteration mit Entwurfsopt.
  end
  Set.optimization.optname = sprintf('extforce_test_%dT%dR', ...
    sum(EEFG(1:3)), sum(EEFG(4:6)));
  if ~isempty(Set.optimization.desopt_vars)
    Set.optimization.optname = [Set.optimization.optname, '_do'];
  end
  Set.optimization.NumIndividuals = 50;
  Set.optimization.MaxIter = 30;
  Set.optimization.ee_rotation = false;
  % Starre EE-Transformation (damit Werte belegt sind)
  if i_FG == 1
    Set.optimization.ee_translation_fixed = [0.1,0,0]; % planar: in x-Richtung
  else
    Set.optimization.ee_translation_fixed = [0,0,0.2];
  end
  Set.general.eval_figures = {'dynamics'};
  Set.general.animation_styles = {};
  % Debug:
  Set.general.plot_robot_in_fitness = 0; % 1e10; -> erfolgreiche IK
  
  % Gelenkgrenzen sehr hoch setzen, damit nicht beschränkend.
  Set.optimization.max_range_passive_universal = 4*pi;
  Set.optimization.max_range_passive_spherical = 4*pi;
  % Unplausible Zylinderlänge erlauben für mehr Ergebnisse
  Set.optimization.prismatic_cylinder_allow_overlength = true;
  Set.general.debug_calc = true; % damit Leistungen geprüft werden
  Set.general.max_retry_bestfitness_reconstruction = 1;
  Set.general.verbosity = 3;
  Set.general.matfile_verbosity = 0;
  % Roboter auswählen
  if i_FG == 1
    Set.structures.whitelist = {'P3RRR1G1P1A1', 'S3RRR1'};
    Set.optimization.ee_translation_fixed(:) = NaN;
  elseif i_FG == 2
    Set.structures.whitelist = {'P3RRRRR10V1G2P2A1'};
  elseif i_FG == 3
    Set.structures.whitelist = {'S4RPRR1', 'P4RRRRR5G1P3A1'};
  elseif i_FG == 4
    Set.structures.whitelist = {'P5RPRRR8V1G9P8A1', 'S5RRRRR5', 'S5RRRRR6'};
  elseif i_FG == 5
    Set.structures.whitelist = {'S6RRRRRR10', 'P6PRRRRR6V4G8P4A1'};
  end
  %% Optimierung starten und Ergebnisse verarbeiten
  cds_start(Set,Traj);
  % Ergebnisse laden
  resmaindir = fullfile(Set.optimization.resdir, Set.optimization.optname);
  ds = load(fullfile(resmaindir, [Set.optimization.optname, '_settings.mat']));
  Set = ds.Set; % benutze in cds_start korrigierte Variable
  Structures = ds.Structures;
  for j = 1:length(Structures)
    resdat1 = fullfile(resmaindir, sprintf('Rob%d_%s_Endergebnis.mat', j, Structures{j}.Name));
    if ~exist(resdat1, 'file'), error('Ergebnisdatei %s nicht gefunden. Muss an dieser Stelle vorliegen', resdat1); end
    tmp1 = load(resdat1, 'RobotOptRes');
    if tmp1.RobotOptRes.fval > 1e3
      warning('Kein i.O.-Ergebnis für %s. Überspringe.', Structures{j}.Name);
      continue;
    end
    resdat2 = fullfile(resmaindir, sprintf('Rob%d_%s_Details.mat', j, Structures{j}.Name));
    tmp2 = load(resdat2, 'RobotOptDetails');
    % Prüfe Reproduzierbarkeit des Werts aus der Synthese. Voraussetzung
    % für nächste Schritte unten mit Änderung der externen Kraft
    [R, Structure] = cds_dimsynth_robot(Set, Traj, Structures{j}, true);
    clear cds_fitness
    % Vertraue nicht auf gleichbleibende Zufallswerte, sondern setze
    % IK-Anfangswerte der besten Lösung manuell. Sonst manchmal nicht
    % reproduzierbar
    R.update_qref(tmp1.RobotOptRes.q0);
    [fval_rtest, ~, Q_rtest, QD_rtest] = cds_fitness(R, Set, Traj, ...
      Structure, tmp1.RobotOptRes.p_val, tmp1.RobotOptRes.desopt_pval);
    abserr_fval = fval_rtest - tmp1.RobotOptRes.fval;
    relerr_fval = abserr_fval./tmp1.RobotOptRes.fval;
    if abs(abserr_fval) > 1e-4 && abs(relerr_fval) > 1e-2
      error('Fitness-Wert für %s nicht reproduzierbar', Structures{j}.Name);
    end
    %% Berechne mit Fitness-Funktion Antriebskräfte ohne externe Kraft.
    Traj_0Fext = Traj;
    Traj_0Fext.Fext(:) = 0;
    clear cds_fitness
    [fval_0Fext, ~, Q_0Fext, QD_0Fext, ~, TAU_0Fext] = cds_fitness(R, Set, ...
      Traj_0Fext, Structure, tmp1.RobotOptRes.p_val, tmp1.RobotOptRes.desopt_pval); 
    if fval_0Fext > 1e3
      error('Ergebnis bei erneuter Berechnung nicht mehr i.O.');
    end
    assert(all(abs(Q_rtest(:)-Q_0Fext(:)) < 1e-6), 'Gelenkposition ist anders mit erneuter Berechnung');
    assert(all(abs(QD_rtest(:)-QD_0Fext(:)) < 1e-6), 'Geschwindigkeit ist anders mit erneuter Berechnung');
    %% Berechne mit Fitness-Funktion Antriebskräfte mit neu gewählter externer Kraft.
    Traj_1Fext = Traj;
    Traj_1Fext.Fext(:) = 0;
    for kk = 1:6 % Random-Walk als Daten für alle Kraftkomponenten
      Traj_1Fext.Fext(:,kk) = 100*cumtrapz(Traj.t, -0.5+rand(length(Traj.t), 1));
    end
    clear cds_fitness
    [fval_1Fext, ~, Q_1Fext, QD_1Fext, ~, TAU_1Fext] = cds_fitness(R, Set, ...
      Traj_1Fext, Structure, tmp1.RobotOptRes.p_val, tmp1.RobotOptRes.desopt_pval);
    if fval_1Fext > 1e3
      error('Ergebnis bei erneuter Berechnung nicht mehr i.O.');
    end
    %% Vergleiche Energien mit und ohne externe Kraft.
    if R.Type == 0, I_qa = R.MDH.mu == 1;
    else,           I_qa = R.I_qa;
    end
    P_0Fext_act = sum(QD_0Fext(:, I_qa).* TAU_0Fext, 2);
    P_1Fext_act = sum(QD_1Fext(:, I_qa) .* TAU_1Fext, 2);
    P_ext_act = P_1Fext_act - P_0Fext_act;
    assert(all(abs(Q_1Fext(:)-Q_0Fext(:)) < 1e-6), 'Gelenkposition ist anders mit/ohne externe Kraft');
    assert(all(abs(QD_1Fext(:)-QD_0Fext(:)) < 1e-6), 'Geschwindigkeit ist anders mit/ohne externe Kraft');
    E_0Fext_act = trapz(Traj.t, P_0Fext_act);
    E_1Fext_act = trapz(Traj.t, P_1Fext_act);
    E_ext_act = E_1Fext_act - E_0Fext_act;
    % Umrechnung von XD auf Winkelgeschwindigkeit. Berechne die
    % X-Trajektorie neu, da im Fall von 3T2R der z-Euler-Winkel abhängig
    % ist und sich ändert
    if all(EEFG == [1 1 1 1 1 0])
      [X_0, XD_0] = R.fkineEE2_traj(Q_0Fext, QD_0Fext);
      Traj_W_neu = transform_traj(R, struct('X', X_0, 'XD', XD_0), false);
      X_W = Traj_W_neu.X; XD_W = Traj_W_neu.XD;
    else
      X_W = Traj.X; XD_W = Traj.XD;
    end
    V = [XD_W(:,1:3), NaN(size(XD_W,1),3)];
    for ii = 1:size(V,1)
      T_phiW = euljac_mex(X_W(ii, 4:6)', R.phiconv_W_E);
      V(ii,4:6) = T_phiW * XD_W(ii, 4:6)';
    end
    % Leistung und Energie der externen Kraft bzgl. EE-KS
    P_Fext = sum(V .* Traj_1Fext.Fext, 2);
    E_Fext = trapz(Traj.t, P_Fext);
    % Vergleiche Differenz zwischen den Energiebilanzen. Vorzeichen müssen
    % entgegengesetzt sein
    change_current_figure(1);clf;
    subplot(2,2,1);
    plot(Traj.t, [P_0Fext_act, P_1Fext_act]);
    ylabel('P in W'); xlabel('t in s'); grid on;
    legend({'act,0fext', 'act,1fext'});
    subplot(2,2,2);
    plot(Traj.t, [P_ext_act, P_Fext]);
    grid on;
    ylabel('P in W'); xlabel('t in s'); grid on;
    legend({'act,diff', 'plf,ext'});
    subplot(2,2,3);
    plot(Traj.t, XD_W);
    ylabel('xD in m/s bzw. rad/s (KS W)'); xlabel('t in s'); grid on;
    legend({'vx', 'vy', 'vz', 'wx', 'wy', 'wz'});
    subplot(2,2,4);
    plot(Traj.t, Traj_1Fext.Fext);
    legend({'fx', 'fy', 'fz', 'mx', 'my', 'mz'});
    ylabel('Fext in N bzw. Nm (KS W)'); xlabel('t in s'); grid on;
    sgtitle(sprintf('Leistungen für Rob. %d (%s)', j, Structures{j}.Name));
    % Vorzeichen der Leistungen sind entgegengesetzt. Siehe Definition oben
    test_P = P_Fext + P_ext_act;
    assert(all(abs(test_P)<1e-6), sprintf(['Von externer Kraft erbrachte Leistung ', ...
      'nicht konsistent mit Leistung in den Antrieben. Max Fehler: %1.1e'], max(abs(test_P))));
    test_E = E_Fext + E_ext_act;
    assert(abs(test_E)<1e-6, sprintf(['Von externer Kraft erbrachte Arbeit (%1.1e) ', ...
      'nicht konsistent mit Energie in den Antrieben (%1.1e)'], E_Fext, E_ext_act));
    fprintf('Test der externen Kraft erfolgreich für Rob %d (%s)\n', j, Structures{j}.Name);
  end
  end
end
