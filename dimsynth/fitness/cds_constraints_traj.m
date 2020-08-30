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
%   1e4...1e5: IK in Trajektorie nicht lösbar
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
s = struct('normalize', false, ... % nicht notwendig, da Prüfen der Winkel-Spannweite. Außerdem sonst Sprung in Traj
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
% Plattform-Bewegung neu für 3T2R-Roboter berechnen (der letzte Euler-Winkel
% ist nicht definiert und kann beliebige Werte einnehmen).
if all(R.I_EE_Task == [1 1 1 1 1 0])
  [X2,XD2,XDD2] = R.fkineEE_traj(Q, QD, QDD);
  % Teste nur die ersten fünf Einträge (sind vorgegeben). Der sechste
  % Wert wird an dieser Stelle erst berechnet und kann nicht verglichen werden.
  % Hier wird nur eine Hin- und Rückrechnung (InvKin/DirKin) gemacht. 
  test_X = Traj_0.X(:,1:5) - X2(:,1:5);
  test_XD = Traj_0.XD(:,1:5) - XD2(:,1:5);
  test_XDD = Traj_0.XDD(:,1:5) - XDD2(:,1:5);
  if any(abs([test_X(:);test_XD(:);test_XDD(:)])>1e-6)
    error('Die Endeffektor-Trajektorie aus direkter Kinematik stimmt nicht');
  end
  % Eintragen des dritten Euler-Winkels, damit spätere Vergleiche
  % funktionieren.
  Traj_0.X(:,6) = X2(:,6);
  Traj_0.XD(:,6) = XD2(:,6);
  Traj_0.XDD(:,6) = XDD2(:,6);
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
  fval = 1e4*(1+9*Failratio); % Wert zwischen 1e4 und 1e5
  % Keine Konvergenz der IK. Weitere Rechnungen machen keinen Sinn.
  constrvioltext = sprintf('Keine IK-Konvergenz in Traj. Bis %1.0f%% (%d/%d) gekommen.', ...
    (1-Failratio)*100, IdxFirst, length(Traj_0.t));
  return
end

%% Prüfe, ob eine Verletzung der Geschwindigkeits-Zwangsbedingungen vorliegt
% Bei 3T2R-PKM kann eine Positions-ZB ungleich Null für die z-Rotation
% korrekt sein. Diese muss aber konstant bleiben und darf sich nicht
% ändern. Durch die Prüfung der ZB-Zeitableitung wird geprüft, ob QD und XD
% konsistent sind.
if any(strcmp(Set.optimization.objective, 'valid_act')) && R.Type ~= 0 % nur sinnvoll bei PKM-Struktursynthese
  PHI4D_ges = NaN(length(Traj_0.t), 6*R.NLEG);
%   PHI1D_ges=PHI4D_ges; PHI2D_ges=PHI4D_ges;
  for jj = 1:length(Traj_0.t)
    % Zum Debuggen: Weitere Zwangsbedingungen
%     [~,PHI1D_ges(jj,:)] = R.constr1D(Q(jj,:)', QD(jj,:)', Traj_0.X(jj,:)',Traj_0.XD(jj,:)');
%     [~,PHI2D_ges(jj,:)] = R.constr2D(Q(jj,:)', QD(jj,:)', Traj_0.X(jj,:)',Traj_0.XD(jj,:)');
    % Geschwindigkeits-Zwangsbedingungen der Koppelpunkte.
    [~,PHI4D_ges(jj,:)] = R.constr4D(Q(jj,:)', QD(jj,:)', Traj_0.X(jj,:)',Traj_0.XD(jj,:)');
  end
%   PHI2D_ges(abs(PHI2D_ges)<1e-6) = 0; % zur Lesbarkeit beim Debuggen
%   PHI4D_ges(abs(PHI4D_ges)<1e-6) = 0; % zur Lesbarkeit beim Debuggen
  if any(abs(PHI4D_ges(:))>1e-6)
    % Bilde Kennzahl aus Schwere der parasitären Bewegung
    fval_paras = mean(abs(PHI4D_ges(:)));
    fval_paras_norm = 2/pi*atan(fval_paras*700); % Normierung auf 0 bis 1. 0.01 -> 0.9
    fval = 9e3*(1+1/9*fval_paras_norm); % Normierung auf 9e3...1e4
    constrvioltext = sprintf(['Es gibt eine parasitäre Bewegung in %d/%d ', ...
      'Zeitschritten. Im Mittel %1.4f (rad/s bzw. m/s). Zuerst bei Zeitschritt %d.'], ...
      sum(any(abs(PHI4D_ges)>1e-3,2)), length(Traj_0.t), fval_paras, ...
      find(any(abs(PHI4D_ges)>1e-6,2),1,'first'));
    return
  end
  % Debuggen der Geschwindigkeits-Konsistenz: Vergleiche EE-Trajektorie von
  % verschiedenen Beinketten aus berechnet.
  if Set.general.debug_calc
    for j = 2:R.NLEG
      [X3,XD3,~] = R.fkineEE_traj(Q, QD, QDD, j);
      test_X = Traj_0.X(:,1:5) - X3(:,1:5);
      test_XD = Traj_0.XD(:,1:6) - XD3(:,1:6);
      % test_XDD = Traj_0.XDD(:,1:6) - XDD3(:,1:6);
      if any(abs(test_X(:))>1e-6)
        if Set.general.matfile_verbosity > 0
          save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_xtraj_legs_inconsistency.mat'));
        end
        error(['Die Endeffektor-Trajektorie X aus Beinkette %d stimmt nicht ', ...
          'gegen Beinkette 1. Zuerst in Zeitschritt %d/%d.'], j, ...
          find(any(abs(test_X)>1e-6,2),1,'first'), length(Traj_0.t));
      end
      if any(abs(test_XD(:))>1e-6)
        error(['Die Endeffektor-Trajektorie XD aus Beinkette %d stimmt nicht ', ...
          'gegen Beinkette 1. Zuerst in Zeitschritt %d/%d.'], j, ...
          find(any(abs(test_XD)>1e-6,2),1,'first'), length(Traj_0.t));
      end
    end
  end
end
if R.Type ~= 0
  % Debuggen der Geschwindigkeits-Konsistenz: Vergleiche EE-Trajektorie von
  % verschiedenen Beinketten aus berechnet.
  if Set.general.debug_calc
    for j = 2:R.NLEG
      [X3,XD3,~] = R.fkineEE2_traj(Q, QD, QDD, uint8(j));
      test_X = Traj_0.X(:,1:5) - X3(:,1:5);
      test_XD = Traj_0.XD(:,1:6) - XD3(:,1:6);
      test_X([false(size(test_X,1),3),abs(test_X(:,4:5)-2*pi)<1e-3]) = 0; % 2pi-Fehler entfernen
      % test_XDD = Traj_0.XDD(:,1:6) - XDD3(:,1:6);
      if any(abs(test_X(:))>1e-6)
        if Set.general.matfile_verbosity > 0
          save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_xtraj_legs_inconsistency.mat'));
        end
        error(['Die Endeffektor-Trajektorie X aus Beinkette %d stimmt nicht ', ...
          'gegen Beinkette 1. Zuerst in Zeitschritt %d/%d.'], j, ...
          find(any(abs(test_X)>1e-6,2),1,'first'), length(Traj_0.t));
      end
      if any(abs(test_XD(:))>1e-6)
        error(['Die Endeffektor-Trajektorie XD aus Beinkette %d stimmt nicht ', ...
          'gegen Beinkette 1. Zuerst in Zeitschritt %d/%d.'], j, ...
          find(any(abs(test_XD)>1e-6,2),1,'first'), length(Traj_0.t));
      end
    end
  end
end
%% Prüfe, ob die Gelenkwinkelgrenzen verletzt werden
% Gleiche Prüfung wie in cds_constraints.m
q_range_T = NaN(1, R.NJ);
q_range_T(R.MDH.sigma==1) = diff(minmax2(Q(:,R.MDH.sigma==1)')');
q_range_T(R.MDH.sigma==0) = angle_range( Q(:,R.MDH.sigma==0));
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
  if fval < Set.general.plot_details_in_fitness
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
    fval = 3e3*(5+1*f_qD_exc_norm); % Wert zwischen 5e3 und 6e3
    % Weitere Berechnungen voraussichtlich wenig sinnvoll, da vermutlich eine
    % Singularität vorliegt
    constrvioltext = sprintf('Geschwindigkeit eines Gelenks zu hoch: max Verletzung %1.1f%% (Gelenk %d)', ...
      (f_qD_exc-1)*100, ifmax);
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
  if fval < Set.general.plot_details_in_fitness
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
    end
    linkxaxes
    sgtitle('Vergleich Gelenkgeschw.');
    change_current_figure(1002);clf;
    for i = 1:R.NJ
      subplot(ceil(sqrt(R.NJ)), ceil(R.NJ/ceil(sqrt(R.NJ))), i);
      hold on; grid on;
      hdl1=plot(Traj_0.t, Q(:,i), '-');
      hdl2=plot(Traj_0.t, Q_num(:,i), '--');
      title(sprintf('q %d (%s), L%d,J%d', i, RP(R.MDH.sigma(i)+1), legnum, legjointnum));
      if i == length(q), legend([hdl1;hdl2], {'q','int(qD)'}); end
    end
    linkxaxes
    sgtitle('Verlauf Gelenkkoordinaten');
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
