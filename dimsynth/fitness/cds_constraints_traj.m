% Nebenbedingungen für Roboter-Maßsynthese berechnen (Teil 2: Geschwindig- 
% keitsebene, inkl Trajektorienberechnung). Entspricht Strafterm aus
% Nebenbedingungsverletzung.
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% Traj_0
%   Endeffektor-Trajektorie (bezogen auf Basis-KS)
% q
%   Anfangs-Gelenkwinkel für die Trajektorien-IK (gradientenbasiert)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Structure
%   Eigenschaften der Roboterstruktur (aus cds_gen_robot_list.m)
% 
% Ausgabe:
% fval
%   Zielfunktionswert, der im PSO-Algorithmus minimiert wird. Entspricht
%   Strafterm in der Fitnessfunktion bei Verletzung der Nebenbedingungen
%   Werte:
%   1e3: Keine Verletzung der Nebenbedingungen. Alles i.O.
%   1e3...2e3: Arbeitsraum-Hindernis-Kollision in Trajektorie
%   2e3...3e3: Bauraumverletzung in Trajektorie
%   3e3...4e3: Selbstkollision in Trajektorie
%   4e3...5e3: Konfiguration springt
%   5e3...6e3: Geschwindigkeitsgrenzen
%   6e3...9e3: Gelenkwinkelgrenzen in Trajektorie
%   9e3...1e4: Parasitäre Bewegung (Roboter strukturell unpassend)
%   1e4...4e4: Inkonsistente Pos./Geschw./Beschl. in Traj.-IK. für Beink. 1 (Sonderfall 3T2R)
%   4e4...5e4: Singularität in Beinkette (obige Betrachtung daher sinnlos)
%   5e4...1e5: IK in Trajektorie nicht lösbar (später mit vorherigem zusammengefasst)
%   1e5...1e9: Nicht belegt (siehe cds_constraints.m)
% Q,QD,QDD
%   Gelenkpositionen und -geschwindigkeiten des Roboters (für PKM auch
%   passive Gelenke)
% Jinvges
%   Zeilenweise (inverse) Jacobi-Matrizen des Roboters (für PKM). Wird hier
%   ausgegeben, da sie bei Berechnung der IK anfällt. Bezogen auf
%   Geschwindigkeit aller Gelenke und EE-Geschwindigkeit
% constrvioltext [char]
%   Text mit Zusatzinformationen, die beim Aufruf der Fitness-Funktion
%   ausgegeben werden

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval,Q,QD,QDD,Jinv_ges,constrvioltext] = cds_constraints_traj(R, Traj_0, q, Set, Structure)
fval = 1e3;
constrvioltext = '';

%% Inverse Kinematik der Trajektorie berechnen
% Einstellungen für IK in Trajektorien
s = struct( ...
  ... % kein Winkel-Normalisierung, da dadurch Sprung in Trajektorie und keine 
  ... % Prüfung gegen vollständige Umdrehungen möglich
  'normalize', false, ... 
  'retry_limit', 0, ... % keine Zufalls-Zahlen. Würde sowieso einen Sprung erzeugen.
  'n_max', 1000, ... % moderate Anzahl Iterationen
  'Phit_tol', 1e-10, 'Phir_tol', 1e-10);
if R.Type == 0 % Seriell
  qlim = R.qlim;
  [Q, QD, QDD, PHI, JP] = R.invkin2_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD, Traj_0.t, q, s);
  Jinv_ges = NaN; % Platzhalter für gleichartige Funktionsaufrufe. Speicherung nicht sinnvoll für seriell.
else % PKM
  qlim = cat(1,R.Leg(:).qlim);
  [Q, QD, QDD, PHI, Jinv_ges, ~, JP] = R.invkin2_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD, Traj_0.t, q, s);
  if Set.general.debug_calc % Rechne nochmal mit Klassenmethode nach
    [Q_debug, QD_debug, QDD_debug, PHI_debug, ~, ~, JP_debug] = R.invkin_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD, Traj_0.t, q, s); %#ok<ASGLU>
    ik_res_ik2 = (all(max(abs(PHI(:,R.I_constr_t_red)))<s.Phit_tol) && ...
        all(max(abs(PHI(:,R.I_constr_r_red)))<s.Phir_tol));% IK-Status Funktionsdatei
    ik_res_iks = (all(max(abs(PHI_debug(:,R.I_constr_t_red)))<s.Phit_tol) && ... 
        all(max(abs(PHI_debug(:,R.I_constr_r_red)))<s.Phir_tol)); % IK-Status Klassenmethode
    if ik_res_ik2 ~= ik_res_iks % Vergleiche IK-Status (Erfolg / kein Erfolg)
      if Set.general.matfile_verbosity > 0
        save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_trajik_error_debug.mat'));
      end
      % Hier keine Warnung wie oben. Traj.-IK darf nicht von Zufall abhängen.
      % TODO: Wirklich Fehlermeldung einsetzen. Erstmal so gelassen, da
      % nicht kritisch.
      warning('Traj.-IK-Berechnung mit Funktionsdatei hat anderen Status (%d) als Klassenmethode (%d).', ik_res_ik2, ik_res_iks);
    end
    % Prüfe, ob die ausgegebenen Gelenk-Positionen auch stimmen
    for i = 1:size(Q,1)
      JointPos_all_i_frominvkin = reshape(JP(i,:)',3,1+R.NJ+R.NLEG);
      Tc_Lges = R.fkine_legs(Q(i,:)');
      JointPos_all_i_fromdirkin = [zeros(3,1), squeeze(Tc_Lges(1:3,4,1:end))];
      % Vergleiche die Positionen. In fkine_legs wird zusätzlich ein
      % virtuelles EE-KS ausgegeben, nicht aber in invkin_ser.
      for kk = 1:R.NLEG
        test_JP = JointPos_all_i_frominvkin(:,kk+(-1+R.I1J_LEG(kk):R.I2J_LEG(kk))) - ...
        JointPos_all_i_fromdirkin(:,kk*2+(-2+R.I1J_LEG(kk):-1+R.I2J_LEG(kk)));
        if any(abs(test_JP(:)) > 1e-8)
          if Set.general.matfile_verbosity > 0
            save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_trajjointpos_error_debug.mat'));
          end
          error(['Ausgegebene Gelenkpositionen stimmen nicht gegen direkte ', ...
            'Kinematik. Zeitpunkt %d, Beinkette %d. Max Fehler %1.1e'], i, kk, max(abs(test_JP(:))));
        end
      end
    end
    % Prüfe ob die Gelenk-Positionen aus Klasse und Vorlage stimmen
    % (nur prüfen, wenn die IK erfolgreich war. Sonst große Fehler bei
    % Zeitschritt des Abbruchs der Berechnung)
    if ik_res_ik2 && ik_res_iks 
      test_Q = Q-Q_debug;
      if any(abs(test_Q(:))>1e-3)
        if Set.general.matfile_verbosity > 0
          save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_trajq_error_debug.mat'));
        end
        error(['Ausgabevariable Q aus invkin_traj vs invkin2_traj stimmt nicht. ', ...
          'Max Fehler %1.1e.'], max(abs(test_Q(:))));
      end
      test_QD = QD-QD_debug;
      if any(abs(test_QD(:))>1e-3)
        if Set.general.matfile_verbosity > 0
          save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_trajqD_error_debug.mat'));
        end
        error(['Ausgabevariable QD aus invkin_traj vs invkin2_traj stimmt nicht. ', ...
          'Max Fehler %1.1e.'], max(abs(test_QD(:))));
      end
      test_QDD_abs = QDD-QDD_debug; % nahe Singularität große Zahlenwerte für QDD ...
      test_QDD_rel = test_QDD_abs./QDD; % ... dadurch Numerik-Probleme möglich.
      I_err = abs(test_QDD_abs)>1e-3 & abs(test_QDD_rel)>1e-3; % 0,1% Abweichung erlaubt
      if any(I_err(:))
        if Set.general.matfile_verbosity > 0
          save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_trajqDD_error_debug.mat'));
        end
        error(['Ausgabevariable QDD aus invkin_traj vs invkin2_traj stimmt nicht. ', ...
          'Max Fehler abs %1.1e., rel %1.4f%%'], max(abs(test_QDD_abs(:))), 100*max(abs(test_QDD_rel(:))));
      end
      test_JPtraj = JP-JP_debug;
      if any(abs(test_JPtraj(:))>1e-6)
        if Set.general.matfile_verbosity > 0
          save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_trajjointpos2_error_debug.mat'));
        end
        error(['Ausgabevariable JP aus invkin_traj vs invkin2_traj stimmt nicht. ', ...
          'Max Fehler %1.1e.'], max(abs(test_JPtraj(:))));
      end
    end
  end
end
% Anfangswerte nochmal neu speichern, damit der Anfangswert exakt der
% Wert ist, der für die Neuberechnung gebraucht wird. Ansonsten ist die
% Reproduzierbarkeit durch die rng-Initialisierung der mex-Funktionen
% gefährdet.
if R.Type == 0 % Seriell
  R.qref = Q(1,:)';
else
  for i = 1:R.NLEG, R.Leg(i).qref = Q(1,R.I1J_LEG(i):R.I2J_LEG(i))'; end
end
% Erkenne eine valide Trajektorie bereits bei Fehler kleiner als 1e-6 an.
% Das ist deutlich großzügiger als die eigentliche IK-Toleranz
I_ZBviol = any(abs(PHI) > 1e-6,2) | any(isnan(Q),2);
if any(I_ZBviol)
  % Bestimme die erste Verletzung der ZB (je später, desto besser)
  IdxFirst = find(I_ZBviol, 1 );
  % Umrechnung in Prozent der Traj.
  Failratio = 1-IdxFirst/length(Traj_0.t); % Wert zwischen 0 und 1
  fval = 1e4*(5+5*Failratio); % Wert zwischen 5e4 und 1e5.
  % Keine Konvergenz der IK. Weitere Rechnungen machen keinen Sinn.
  constrvioltext = sprintf('Keine IK-Konvergenz in Traj. Bis %1.0f%% (%d/%d) gekommen.', ...
    (1-Failratio)*100, IdxFirst, length(Traj_0.t));
  return
end
%% Singularität der Beinketten prüfen (für PKM)
% Im Gegensatz zu cds_obj_condition wird hier die gesamte Beinkette
% betrachtet. Entspricht Singularität der direkten Kinematik der Beinkette.
IdxFirst = 0;
if R.Type == 2 % nur PKM; TODO: Auch für seriell prüfen?
  for jj = 1:length(Traj_0.t)
    Jinv_jj = reshape(Jinv_ges(jj,:), R.NJ, sum(R.I_EE));
    for kk = 1:R.NLEG
      % Jacobi-Matrix für alle Gelenke der Beinkette (bezug zu XD des EE)
      Jinv_kk = Jinv_jj(R.I1J_LEG(kk):R.I2J_LEG(kk),:);
      kappa_jjkk = cond(Jinv_kk);
      if Set.general.debug_calc
        % Probe, ob es die Jinv richtig ist
        qD_kk2 = Jinv_kk*Traj_0.XD(jj,R.I_EE)';
        qD_kk1 = QD(jj,R.I1J_LEG(kk):R.I2J_LEG(kk))';
        if any(abs(qD_kk1-qD_kk2) > 1e-8)
          error('Neu berechnete Geschwindigkeit aus Beinketten-Jacobi-Matrix stimmt nicht');
        end
      end
      if kappa_jjkk > 1e8
        IdxFirst = jj;
        break;
      end
    end
    if IdxFirst ~= 0
      break;
    end
  end
end
if IdxFirst ~= 0
  Failratio = 1-IdxFirst/length(Traj_0.t); % Wert zwischen 0 und 1
  fval = 1e4*(4+1*Failratio); % Wert zwischen 4e4 und 5e4.
  % Singularität in Beinkette. Weitere Rechnungen ergeben keinen Sinn
  % (Geschwindigkeit der Gelenke kann beliebig springen)
  constrvioltext = sprintf('Singularität in Beinkette %d (cond=%1.1e). Bis %1.0f%% (%d/%d) gekommen.', ...
    kk, kappa_jjkk, (1-Failratio)*100, IdxFirst, length(Traj_0.t));
  return
end
%% Plattform-Bewegung neu für 3T2R-Roboter berechnen
% der letzte Euler-Winkel ist nicht definiert und kann beliebige Werte einnehmen).
if all(R.I_EE_Task == [1 1 1 1 1 0]) || Set.general.debug_calc
  if R.Type == 0 % Seriell
    [X2,XD2,XDD2] = R.fkineEE_traj(Q, QD, QDD);
  else
    [X2,XD2,XDD2] = R.fkineEE2_traj(Q, QD, QDD);
  end
  % Teste nur die ersten fünf Einträge (sind vorgegeben). Der sechste
  % Wert wird an dieser Stelle erst berechnet und kann nicht verglichen werden.
  % Hier wird nur eine Hin- und Rückrechnung (InvKin/DirKin) gemacht. 
  test_X = Traj_0.X(:,1:5) - X2(:,1:5);
  test_X([false(size(test_X,1),3),abs(abs(test_X(:,4:5))-2*pi)<1e-3]) = 0; % 2pi-Fehler entfernen
  if any(abs(test_X(:))>1e-6)
    % Bestimme die mittlere Abweichung zwischen Position des Endeffektors
    % aus inverser und direkter Kinematik
    % Dieser Fall darf eigentlich gar nicht auftreten, wenn invkin und
    % fkin korrekt implementiert sind.
    fval_x = mean(test_X(:));
    fval_x_norm = 2/pi*atan(fval_x*70); % Normierung auf 0 bis 1. 0.1 -> 0.9
    fval = 1e4*(3+fval_x_norm); % Werte zwischen 3e4 und 4e4
    constrvioltext=sprintf(['Fehler der EE-Lage der ersten Beinkette ', ...
      'zwischen invkin und fkine. Max Fehler %1.2e'], max(abs(test_X(:))));
    return
  end
  test_XD = Traj_0.XD(:,1:5) - XD2(:,1:5);
  if any(abs(test_XD(:))>1e-6)
    % Bestimme die mittlere Abweichung zwischen Geschwindigkeit des Endeffektors
    % aus inverser und direkter differentieller Kinematik. Darf
    % eigentlich nicht passieren (s.o.).
    fval_xD = mean(test_XD(:));
    fval_xD_norm = 2/pi*atan(fval_xD*70); % Normierung auf 0 bis 1. 0.1 -> 0.9
    fval = 1e4*(2+fval_xD_norm); % Werte zwischen 2e4 und 3e4
    constrvioltext=sprintf(['Fehler der EE-Geschwindigkeit der ersten Beinkette ', ...
      'zwischen invkin und fkine. Max Fehler %1.2e'], max(abs(test_XD(:))));
    return
  end
  test_XDD = Traj_0.XDD(:,1:5) - XDD2(:,1:5);
  if any(abs(test_XDD(:))>1e-6)
    % Bestimme die mittlere Abweichung zwischen Beschleunigung des Endeffektors
    % aus inverser und direkter differentieller Kinematik. Darf
    % eigentlich nicht passieren (s.o.).
    fval_xDD = mean(test_XDD(:));
    fval_xDD_norm = 2/pi*atan(fval_xDD*70); % Normierung auf 0 bis 1. 0.1 -> 0.9
    fval = 1e4*(1+fval_xDD_norm); % Werte zwischen 1e4 und 2e4
    constrvioltext=sprintf(['Fehler der EE-Beschleunigung der ersten Beinkette ', ...
      'zwischen invkin und fkine. Max Fehler %1.2e'], max(abs(test_XDD(:))));
    return
  end
  % Eintragen des dritten Euler-Winkels, damit spätere Vergleiche funktionieren.
  if all(R.I_EE_Task == [1 1 1 1 1 0])
    Traj_0.X(:,6) = X2(:,6);
    Traj_0.XD(:,6) = XD2(:,6);
    Traj_0.XDD(:,6) = XDD2(:,6);
  end
end
%% Prüfe, ob eine Verletzung der Geschwindigkeits-Zwangsbedingungen vorliegt
% Bei 3T2R-PKM kann eine Positions-ZB ungleich Null für die z-Rotation
% korrekt sein. Diese muss aber konstant bleiben und darf sich nicht
% ändern. Durch die Prüfung der ZB-Zeitableitung wird geprüft, ob QD und XD
% konsistent sind.
if any(strcmp(Set.optimization.objective, 'valid_act')) && R.Type ~= 0 % nur sinnvoll bei PKM-Struktursynthese
  % Geschwindigkeits-Zwangsbedingungen der Koppelpunkte.
  PHI4D_ges = R.constr4D2_traj(Q, QD, Traj_0.X, Traj_0.XD);
  % Zum Debuggen: Weitere Zwangsbedingungen
%   PHI1D_ges=NaN(size(PHI4D_ges)); PHI2D_ges=PHI1D_ges;
%   for jj = 1:length(Traj_0.t)
%     [~,PHI1D_ges(jj,:)] = R.constr1D(Q(jj,:)', QD(jj,:)', Traj_0.X(jj,:)',Traj_0.XD(jj,:)');
%     [~,PHI2D_ges(jj,:)] = R.constr2D(Q(jj,:)', QD(jj,:)', Traj_0.X(jj,:)',Traj_0.XD(jj,:)');
%   end
  if any(abs(PHI4D_ges(:))>1e-6)
    % Bilde Kennzahl aus Schwere der parasitären Bewegung
    fval_paras = mean(abs(PHI4D_ges(:)));
    fval_paras_norm = 2/pi*atan(fval_paras*700); % Normierung auf 0 bis 1. 0.01 -> 0.9
    fval = 1e3*(9+1*fval_paras_norm); % Normierung auf 9e3...1e4
    constrvioltext = sprintf(['Es gibt eine parasitäre Bewegung in %d/%d ', ...
      'Zeitschritten. Im Mittel %1.4f (rad/s bzw. m/s). Zuerst bei Zeitschritt %d.'], ...
      sum(any(abs(PHI4D_ges)>1e-3,2)), length(Traj_0.t), fval_paras, ...
      find(any(abs(PHI4D_ges)>1e-6,2),1,'first'));
    return
  end
end
if R.Type ~= 0 && Set.general.debug_calc
  % Debuggen der Geschwindigkeits-Konsistenz: Vergleiche EE-Trajektorie von
  % verschiedenen Beinketten aus berechnet.
  for j = 2:R.NLEG
    [X3,XD3,XDD3] = R.fkineEE2_traj(Q, QD, QDD, uint8(j));
    test_X = Traj_0.X(:,1:6) - X3(:,1:6);
    test_X([false(size(test_X,1),3),abs(abs(test_X(:,4:6))-2*pi)<1e-3]) = 0; % 2pi-Fehler entfernen
    test_XD = Traj_0.XD(:,1:6) - XD3(:,1:6);
    test_XDD = Traj_0.XDD(:,1:6) - XDD3(:,1:6);
    if any(abs(test_X(:))>1e-6)
      if Set.general.matfile_verbosity > 0
        save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_xtraj_legs_inconsistency.mat'));
      end
      error(['Die Endeffektor-Trajektorie X aus Beinkette %d stimmt nicht ', ...
        'gegen Beinkette 1. Zuerst in Zeitschritt %d/%d. Fehler max. %1.1e.'], j, ...
        find(any(abs(test_X)>1e-6,2),1,'first'), length(Traj_0.t), max(abs(test_X(:))));
    end
    if any(abs(test_XD(:))>1e-6)
      if Set.general.matfile_verbosity > 0
        save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_xDtraj_legs_inconsistency.mat'));
      end
      error(['Die Endeffektor-Trajektorie XD aus Beinkette %d stimmt nicht ', ...
        'gegen Beinkette 1. Zuerst in Zeitschritt %d/%d. Fehler max. %1.1e.'], j, ...
        find(any(abs(test_XD)>1e-6,2),1,'first'), length(Traj_0.t), max(abs(test_XD(:))));
    end
    if any(abs(test_XDD(:))>1e-6)
      if Set.general.matfile_verbosity > 0
        save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_xDDtraj_legs_inconsistency.mat'));
      end
      error(['Die Endeffektor-Trajektorie XDD aus Beinkette %d stimmt nicht ', ...
        'gegen Beinkette 1. Zuerst in Zeitschritt %d/%d. Fehler max. %1.1e.'], j, ...
        find(any(abs(test_XDD)>1e-6,2),1,'first'), length(Traj_0.t), max(abs(test_XDD(:))));
    end
  end
end
%% Prüfe, ob die Gelenkwinkelgrenzen verletzt werden
% Andere Prüfung als in cds_constraints.m. Gehe davon aus, dass die
% Trajektorie stetig und sprungfrei ist. Ist eine Winkelspannweite von mehr
% als 360° erlaubt, ist die Prüfung auf Winkelspannweite mit angle_range
% immer erfolgreich, auch wenn sich Gelenke mehrfach umdrehen.
q_range_T = diff(minmax2(Q')');
qlimviol_T = (qlim(:,2)-qlim(:,1))' - q_range_T;
I_qlimviol_T = (qlimviol_T < 0);
if any(I_qlimviol_T)
  if Set.general.matfile_verbosity > 2
    save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_qviolT.mat'));
  end
  % Bestimme die größte relative Verletzung der Winkelgrenzen
  [fval_qlimv_T, I_worst] = min(qlimviol_T(I_qlimviol_T)./(qlim(I_qlimviol_T,2)-qlim(I_qlimviol_T,1))');
  II_qlimviol_T = find(I_qlimviol_T); IIw = II_qlimviol_T(I_worst);
  fval_qlimv_T_norm = 2/pi*atan((-fval_qlimv_T)/0.3); % Normierung auf 0 bis 1; 2 ist 0.9
  fval = 1e3*(6+3*fval_qlimv_T_norm); % Wert zwischen 6e3 und 9e3
  % Überschreitung der Gelenkgrenzen (bzw. -bereiche). Weitere Rechnungen machen keinen Sinn.
  constrvioltext = sprintf(['Gelenkgrenzverletzung in Traj. Schlechteste ', ...
    'Spannweite: %1.2f/%1.2f (Gelenk %d)'], q_range_T(IIw), qlim(IIw,2)-qlim(IIw,1), IIw);
  if 1e4*fval < Set.general.plot_details_in_fitness
    change_current_figure(1001); clf;
    plot(Traj_0.t, Q-repmat(min(Q), length(Traj_0.t), 1));
  end
  return
end

%% Prüfe, ob die Geschwindigkeitsgrenzen verletzt werden
% Diese Prüfung erfolgt zusätzlich zu einer Antriebsauslegung.
% Gedanke: Wenn die Gelenkgeschwindigkeit zu schnell ist, ist sowieso kein
% Antrieb auslegbar und die Parameter können schneller verworfen werden.
% Außerdem liegt wahrscheinlich eine Singularität vor.
if any(~isinf(Structure.qDlim(:)))
  qD_max = max(abs(QD))';
  qD_lim = Structure.qDlim(:,2); % Annahme symmetrischer Geschw.-Grenzen
  [f_qD_exc,ifmax] = max(qD_max./qD_lim);
  if f_qD_exc>1
    f_qD_exc_norm = 2/pi*atan((f_qD_exc-1)); % Normierung auf 0 bis 1; 1->0.5; 10->0.94
    fval = 1e3*(5+1*f_qD_exc_norm); % Wert zwischen 5e3 und 6e3
    % Weitere Berechnungen voraussichtlich wenig sinnvoll, da vermutlich eine
    % Singularität vorliegt
    constrvioltext = sprintf('Geschwindigkeit eines Gelenks zu hoch: max Verletzung %1.1f%% (Gelenk %d)', ...
      (f_qD_exc-1)*100, ifmax);
    if 1e4*fval < Set.general.plot_details_in_fitness
      RP = ['R', 'P'];
      change_current_figure(1004);clf;
      for i = 1:R.NJ
        if R.Type ~= 0
          legnum = find(i>=R.I1J_LEG, 1, 'last');
          legjointnum = i-(R.I1J_LEG(legnum)-1);
        end
        subplot(ceil(sqrt(R.NJ)), ceil(R.NJ/ceil(sqrt(R.NJ))), i);
        hold on; grid on;
        plot(Traj_0.t, QD(:,i), '-');
        plot(Traj_0.t([1,end]), repmat(Structure.qDlim(i,:),2,1), 'r-');
        ylim(minmax2([QD(:,i);QD(:,i)]'));
        if R.Type == 0
          title(sprintf('qD %d (%s)', i, RP(R.MDH.sigma(i)+1)));
        else
          title(sprintf('qD %d (%s), L%d,J%d', i, RP(R.MDH.sigma(i)+1), legnum, legjointnum));
        end
      end
      linkxaxes
      sgtitle('Gelenkgeschwindigkeiten');
    end
    return
  end
end

%% Prüfe, ob die Konfiguration umklappt während der Trajektorie
% Geschwindigkeit neu mit Differenzenquotient berechnen
QD_num = zeros(size(Q));
QD_num(2:end,R.MDH.sigma==1) = diff(Q(:,R.MDH.sigma==1))./...
  repmat(diff(Traj_0.t), 1, sum(R.MDH.sigma==1)); % Differenzenquotient
QD_num(2:end,R.MDH.sigma==0) = (mod(diff(Q(:,R.MDH.sigma==0))+pi, 2*pi)-pi)./...
  repmat(diff(Traj_0.t), 1, sum(R.MDH.sigma==0)); % Siehe angdiff.m
% Position neu mit Trapezregel berechnen (Integration)
Q_num = repmat(Q(1,:),size(Q,1),1)+cumtrapz(Traj_0.t, QD);
% Bestimme Korrelation zwischen den Verläufen (1 ist identisch)
corrQD = diag(corr(QD_num, QD));
corrQ = diag(corr(Q_num, Q));
if any(corrQD < 0.95) || any(corrQ < 0.98)
  % Bilde normierten Strafterm aus Korrelationskoeffizienten (zwischen -1
  % und 1).
  fval_jump_norm = 0.5*(mean(1-corrQ) + mean(1-corrQD));
  fval = 1e3*(4+1*fval_jump_norm); % Wert zwischen 4e3 und 5e3
  constrvioltext = sprintf('Konfiguration scheint zu springen. Korrelation Geschw. min. %1.2f, Position %1.2f', ...
    min(corrQD), min(corrQ));
  if 1e4*fval < Set.general.plot_details_in_fitness
    % Geschwindigkeit neu mit Trapezregel berechnen (Integration)
    QD_num2 = repmat(QD(1,:),size(QD,1),1)+cumtrapz(Traj_0.t, QDD);
    RP = ['R', 'P'];
    change_current_figure(1001);clf;
    for i = 1:R.NJ
      legnum = find(i>=R.I1J_LEG, 1, 'last');
      legjointnum = i-(R.I1J_LEG(legnum)-1);
      subplot(ceil(sqrt(R.NJ)), ceil(R.NJ/ceil(sqrt(R.NJ))), i);
      hold on; grid on;
      hdl1=plot(Traj_0.t, QD(:,i), '-');
      hdl2=plot(Traj_0.t, QD_num(:,i), '--');
      hdl3=plot(Traj_0.t, QD_num2(:,i), ':');
      plot(Traj_0.t([1,end]), repmat(Structure.qDlim(i,:),2,1), 'r-');
      ylim(minmax2([QD_num(:,i);QD_num(:,i)]'));
      title(sprintf('qD %d (%s), L%d,J%d', i, RP(R.MDH.sigma(i)+1), legnum, legjointnum));
      if i == length(q), legend([hdl1;hdl2;hdl3], {'qD','diff(q)', 'int(qDD)'}); end
      if legjointnum == 1, ylabel(sprintf('Beinkette %d',legnum)); end
    end
    linkxaxes
    sgtitle('Vergleich Gelenkgeschw.');
    change_current_figure(1002);clf;
    for i = 1:R.NJ
      legnum = find(i>=R.I1J_LEG, 1, 'last');
      legjointnum = i-(R.I1J_LEG(legnum)-1);
      subplot(ceil(sqrt(R.NJ)), ceil(R.NJ/ceil(sqrt(R.NJ))), i);
      hold on; grid on;
      hdl1=plot(Traj_0.t, Q(:,i), '-');
      hdl2=plot(Traj_0.t, Q_num(:,i), '--');
      title(sprintf('q %d (%s), L%d,J%d', i, RP(R.MDH.sigma(i)+1), legnum, legjointnum));
      if i == length(q), legend([hdl1;hdl2], {'q','int(qD)'}); end
      if legjointnum == 1, ylabel(sprintf('Beinkette %d',legnum)); end
    end
    linkxaxes
    sgtitle('Verlauf Gelenkkoordinaten');
    change_current_figure(1003);clf;
    QDD_num = zeros(size(Q)); % Differenzenquotient
    QDD_num(2:end,:) = diff(QD(:,:))./ repmat(diff(Traj_0.t), 1, R.NJ);
    for i = 1:R.NJ
      legnum = find(i>=R.I1J_LEG, 1, 'last');
      legjointnum = i-(R.I1J_LEG(legnum)-1);
      subplot(ceil(sqrt(R.NJ)), ceil(R.NJ/ceil(sqrt(R.NJ))), i);
      hold on; grid on;
      hdl1=plot(Traj_0.t, QDD(:,i), '-');
      hdl2=plot(Traj_0.t, QDD_num(:,i), '--');
      title(sprintf('q %d (%s), L%d,J%d', i, RP(R.MDH.sigma(i)+1), legnum, legjointnum));
      if i == length(q), legend([hdl1;hdl2], {'qDD','diff(qD)'}); end
      if legjointnum == 1, ylabel(sprintf('Beinkette %d',legnum)); end
    end
    linkxaxes
    sgtitle('Verlauf Gelenkbeschleunigungen');
  end
  return
end
%% Aktualisiere Roboter für Kollisionsprüfung (geänderte Grenzen aus Traj.-IK)
if Set.optimization.constraint_collisions || ...
    ~isempty(Set.task.installspace.type) || ~isempty(Set.task.obstacles.type)
  [Structure.collbodies_robot, Structure.installspace_collbodies] = ...
    cds_update_collbodies(R, Set, Structure, Q);
end
%% Selbstkollisionserkennung für Trajektorie
if Set.optimization.constraint_collisions
  [fval_coll_traj, coll_traj] = cds_constr_collisions_self(R, Traj_0.X, ...
    Set, Structure, JP, Q, [3e3; 4e3]);
  if fval_coll_traj > 0
    fval = fval_coll_traj; % Normierung auf 3e3 bis 4e3 -> bereits in Funktion
    constrvioltext = sprintf('Kollision in %d/%d Traj.-Punkten.', ...
      sum(any(coll_traj,2)), size(coll_traj,1));
    return
  end
end

%% Bauraumprüfung für Trajektorie
if ~isempty(Set.task.installspace.type)
  [fval_instspc_traj, f_constrinstspc_traj] = cds_constr_installspace( ...
    R, Traj_0.X, Set, Structure, JP, Q, [2e3;3e3]);
  if fval_instspc_traj > 0
    fval = fval_instspc_traj; % Normierung auf 2e3 bis 3e3 -> bereits in Funktion
    constrvioltext = sprintf(['Verletzung des zulässigen Bauraums in Traj.', ...
      'Schlimmstenfalls %1.1f mm draußen.'], 1e3*f_constrinstspc_traj);
    return
  end
end
%% Arbeitsraum-Hindernis-Kollisionsprüfung für Trajektorie
if ~isempty(Set.task.obstacles.type)
  [fval_obstcoll_traj, coll_obst_traj, f_constr_obstcoll_traj] = cds_constr_collisions_ws( ...
    R, Traj_0.X, Set, Structure, JP, Q, [1e3;2e3]);
  if fval_obstcoll_traj > 0
    fval = fval_obstcoll_traj; % Normierung auf 1e3 bis 2e3 -> bereits in Funktion
    constrvioltext = sprintf(['Arbeitsraum-Kollision in %d/%d Traj.-Punkten. ', ...
      'Schlimmstenfalls %1.1f mm in Kollision.'], sum(any(coll_obst_traj,2)), ...
      size(coll_obst_traj,1), f_constr_obstcoll_traj);
    return
  end
end
