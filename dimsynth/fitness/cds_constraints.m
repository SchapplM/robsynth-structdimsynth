% Nebenbedingungen für Roboter-Maßsynthese berechnen (Teil 1: Positions-
% ebene). Entspricht Strafterm aus Nebenbedingungsverletzung
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% Traj_0
%   Endeffektor-Trajektorie (bezogen auf Basis-KS)
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
%   1e3...1e5: Nicht belegt (siehe cds_fitness)
%   1e5...2e5: Arbeitsraum-Hindernis-Kollision in Einzelpunkten
%   2e5...3e5: Bauraumverletzung in Einzelpunkten
%   3e5...4e5: Selbstkollision in Einzelpunkten
%   4e5...5e5: Gestell ist wegen Schubgelenken zu groß (nach IK erkannt)
%   5e5...1e6: Gelenkwinkelgrenzen in Einzelpunkten
%   1e6...1e7: IK in Einzelpunkten nicht lösbar
%   1e7...1e8: Geometrie nicht plausibel lösbar (2: Reichweite PKM-Koppelpunkte)
%   1e8...1e9: Geometrie nicht plausibel lösbar (1: Schließen PKM-Ketten)
% QE_all (Anz. Eckpunkte x Anz. Gelenke x Anz. Konfigurationen)
%   Gelenkpositionen des Roboters (für PKM auch passive Gelenke)
%   für alle Eckpunkte im Arbeitsraum und für alle gefundenen
%   IK-Konfigurationen
% Q0
%   Erste Gelenkkonfiguration der Trajektorie (als Startwert für zukünftige
%   Berechnung der IK)
% constrvioltext [char]
%   Text mit Zusatzinformationen, die beim Aufruf der Fitness-Funktion
%   ausgegeben werden

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [fval,QE_all,Q0,constrvioltext] = cds_constraints(R, Traj_0, Set, Structure)
Q0 = NaN(1,R.NJ);
QE_all = Q0;
%% Geometrie auf Plausibilität prüfen (1)
if R.Type == 0 % Seriell
  % Prüfe, ob alle Eckpunkte der Trajektorie im Arbeitsraum des Roboters liegen
  dist_max = R.reach();
  dist_exc_tot = NaN(size(Traj_0.XE,1),1);
  for i = 1:size(Traj_0.XE,1)
    dist_i = norm(Traj_0.XE(i,1:3));
    dist_exc_tot(i) = dist_max-dist_i;
  end
  if any(dist_exc_tot < 0)
    % Mindestens ein Punkt überschreitet die maximale Reichweite des Roboters
    f_distviol = -min(dist_exc_tot)/dist_max; % maximale Abstandsverletzung (relativ zu Maximalreichweite)
    % Werte sind typischerweise zwischen 0 und 100. Je kleiner der Roboter,
    % desto größer der Wert; das regt dann zur Vergrößerung des Roboters an.
    f_distviol_norm = 2/pi*atan((f_distviol)); % 1->0.5; 10->0.94
    %  normiere auf 1e8 bis 1e9
    fval = 1e8*(1+9*f_distviol_norm);
    constrvioltext = sprintf('Roboter zu kurz. Es fehlen %1.2f m bei einer Reichweite von max. %1.2f m (%1.0f%%).', -min(dist_exc_tot), dist_max,100*f_distviol);
    return
  end
else % PKM
  % Prüfe, ob die Ketten sich überhaupt schließen können (sind die Beine lang
  % genug um sich zu den Koppelpunkten zu verbinden.
  % Dieses Maß ist nur eine Eigenschaft der Kinematik, unabhängig von der
  % gewünschten Trajektorie
  d_base = 2*R.DesPar.base_par(1);
  d_platf = 2*R.DesPar.platform_par(1);
  l_max_leg = R.Leg(1).reach(); % Annahme: Symmetrischer Roboter; alle Beine max. gleich lang

  l_legtooshort = abs(d_base - d_platf)/2 - l_max_leg; % Fehlende Länge jedes Beins (in Meter)
  if l_legtooshort > 0
    f_legtooshort = l_legtooshort/l_max_leg; % fehlende Beinlänge (relativ zu Maximalreichweite)
    % Verhältnis im Bereich 1 bis 100
    f_distviol_norm = 2/pi*atan((f_legtooshort)); % 1->0.5; 10->0.94
    %  normiere auf 1e8 bis 1e9
    fval = 1e8*(1+9*f_distviol_norm);
    constrvioltext = sprintf('Beinkette mit max. Länge %1.2fm zu kurz für Plattform. Es fehlen max. %1.2fm.', l_max_leg, l_legtooshort);
    return
  end
end

%% Geometrie auf Plausibilität prüfen (2)
if R.Type == 0 % Seriell
  % Entfällt. Nur eine Prüfung für serielle Roboter
else % PKM
  % Berechne die Position der Koppelpunkte für die vorgesehenen Eckpunkte der
  % Trajektorie. Dieses Maß wird durch die Trajektorie bestimmt.
  % TODO: Funktioniert noch nicht bei Aufgabenredundanz.
  dist_exc_tot = NaN(size(Traj_0.XE,1),R.NLEG);
  T_P_P_E = R.T_P_E;
  for i = 1:R.NLEG
    % Transformationen für die Beinkette i
    T_0_0i = R.Leg(i).T_W_0;
    r_P_P_Bi = R.r_P_B_all(:,i);
    for j = 1:size(Traj_0.XE,1)
      % EE-Transformation für Bahnpunkt j
      T_0_Ej = R.x2t(Traj_0.XE(j,:)');
      % Position des Plattform-Koppelpunktes der Beinkette i für den
      % Bahnpunkt j (und die dafür vorgesehene Orientierung)
      rh_0i_0i_Bij = invtr(T_0_0i)*T_0_Ej*invtr(T_P_P_E)*[r_P_P_Bi;1];
      dist_j = norm(rh_0i_0i_Bij);
      dist_exc_tot(j,i) = l_max_leg-dist_j;
    end
  end
  if any(dist_exc_tot(:) < 0)
    % Mindestens ein Punkt überschreitet die maximale Reichweite einer Beinkette
    f_distviol = -min(dist_exc_tot(:))/l_max_leg; % maximale Abstandsverletzung (relativ zu Maximalreichweite)
    % Werte sind typischerweise zwischen 0 und 100. Je kleiner die Beinkette,
    % desto größer der Wert; das regt dann zur Vergrößerung des Roboters an.
    f_distviol_norm = 2/pi*atan((f_distviol)); % 1->0.5; 10->0.94
    %  normiere auf 1e7 bis 1e8
    fval = 1e7*(1+9*f_distviol_norm);
    constrvioltext = sprintf('Beinkette mit max. Länge %1.2fm zu kurz für Bahnpunkte. Es fehlen max. %1.2fm.', ...
      l_max_leg, -min(dist_exc_tot(:)));
    return
  end
end

%% Inverse Kinematik für Eckpunkte der Trajektorie berechnen
if R.Type == 0 % Seriell
  qlim = R.qlim;
  qref = R.qref;
  Phi_E = NaN(size(Traj_0.XE,1), sum(Set.structures.DoF));
  QE = NaN(size(Traj_0.XE,1), R.NQJ);
  if Set.task.profile ~= 0
    % Normale Trajektorie mit stetigem Zeitverlauf. Nur Berechnung der
    % Eckpunkte zur Prüfung. Setze die Zufallszahlen-Initialisierung mit
    % rng_seed, damit die Ergebnisse exakt reproduzierbar werden.
    s = struct('Phit_tol', 1e-3, 'Phir_tol', 1e-3, 'retry_limit', 20, ...
      'normalize', false, 'rng_seed', 0);
  else
    % Eckpunkte haben keinen direkten Bezug zueinander und bilden die
    % Trajektorie. Da keine Traj. berechnet wird, kann hier mehr Aufwand
    % betrieben werden (besonders bei seriellen Robotern auch notwendig.
    s = struct('Phit_tol', 1e-9, 'Phir_tol', 1e-9, 'retry_limit', 50, ...
      'normalize', false, 'n_max', 5000, 'rng_seed', 0);
  end
  % Variable zum Speichern der Gelenkpositionen (für Kollisionserkennung)
  JPE = NaN(size(Traj_0.XE,1), R.NL*3);
else % PKM
  qlim = cat(1,R.Leg(:).qlim);
  qref = cat(1,R.Leg(:).qref);
  nPhi = R.I2constr_red(end);
  Phi_E = NaN(size(Traj_0.XE,1), nPhi);
  QE = NaN(size(Traj_0.XE,1), R.NJ);
  if Set.task.profile ~= 0 % Normale Trajektorie mit stetigem Zeitverlauf
    s = struct('Phit_tol', 1e-4, 'Phir_tol', 1e-3, 'retry_limit', 20, ...
      'normalize', false, 'rng_seed', 0);
  else % Nur Eckpunkte
    s = struct('Phit_tol', 1e-9, 'Phir_tol', 1e-9, 'retry_limit', 50, ...
      'normalize', false, 'n_max', 5000, 'rng_seed', 0);
  end
  % Abbruch der IK-Berechnung, wenn eine Beinkette nicht erfolgreich war.
  % Dadurch wesentlich schnellerer Durchlauf der PKM-IK
  s_par = struct('abort_firstlegerror', true);
  JPE = NaN(size(Traj_0.XE,1), (R.NL-1+R.NLEG)*3);
end
n_jic = 30;
fval_jic = NaN(1,n_jic);
constrvioltext_jic = cell(n_jic,1);
Q_jic = NaN(size(Traj_0.XE,1), R.NJ, n_jic);
q0_jic = NaN(R.NJ, n_jic); % zum späteren Nachvollziehen des Ergebnisses
for jic = 1:n_jic % Schleife über IK-Konfigurationen (30 Versuche)
  Phi_E(:) = NaN; QE(:) = NaN; % erneut initialisieren wegen jic-Schleife.
  if jic == 1 && all(~isnan(qref)) && any(qref~=0) % nehme Referenz-Pose (kann erfolgreiche gespeicherte Pose bei erneutem Aufruf enthalten)
    q0 = qref; % Wenn hier nur Nullen stehen, werden diese ignoriert.
  else
    q0 = qlim(:,1) + rand(R.NJ,1).*(qlim(:,2)-qlim(:,1)); % Zufällige Anfangswerte geben vielleicht neue Konfiguration.
  end
  q0_jic(:,jic) = q0;
  % Anpassung der IK-Anfangswerte für diesen Durchlauf der IK-Konfigurationen.
  % Versuche damit eine andere Konfiguration zu erzwingen
  if fval_jic(1) > 1e6
    % IK hat beim ersten Mal schon nicht funktioniert (dort werden aber
    % zufällige Neuversuche gemacht). Andere Anfangswerte sind zwecklos.
    break;
  end

  if jic > 10 && jic < 20
    % Setze die Anfangswerte (für Schubgelene) ganz weit nach "links"
    q0(R.MDH.sigma==1) = q0(R.MDH.sigma==1)-(0.5*rand(1))*(qlim(R.MDH.sigma==1,2)-qlim(R.MDH.sigma==1,1));
  elseif jic > 21
    % Anfangswerte weit nach rechts
    q0(R.MDH.sigma==1) = q0(R.MDH.sigma==1)+(0.5*rand(1))*(qlim(R.MDH.sigma==1,2)-qlim(R.MDH.sigma==1,1));
  end
  % Normalisiere den Anfangswert (außerhalb [-pi,pi) nicht sinnvoll).
  % (Betrifft nur Fall, falls Winkelgrenzen groß gewählt sind)
  q0(R.MDH.sigma==0) = normalize_angle(q0(R.MDH.sigma==0));
  % Setze bei PKM die Anfangswerte für alle Beinketten identisch
  if R.Type == 2
    q0(R.I1J_LEG(2):end) = NaN; % Dadurch in invkin_ser Werte der ersten Beinkette genommen
  end

  % IK für alle Eckpunkte, beginnend beim letzten (dann ist q der richtige
  % Startwert für die Trajektorien-IK)
  for i = size(Traj_0.XE,1):-1:1
    if Set.task.profile ~= 0 % Trajektorie wird weiter unten berechnet
      if i == size(Traj_0.XE,1)-1 % zweiter Berechneter Wert
        % Annahme: Weniger Neuversuch der IK. Wenn die Gelenkwinkel zufällig neu
        % gewählt werden, springt die Konfiguration voraussichtlich. Dann ist
        % die Durchführung der Trajektorie unrealistisch. Kann nicht zu
        % Null gewählt werden, da die Einzelpunkt-IK nicht immer gut kon-
        % vergiert. Bei 0 werden teilweise funktionierende Roboter wieder verworfen.
        s.retry_limit = 15;
      elseif i == 1
        % Setze die Toleranz für diesen Punkt wieder herunter. Der Startpunkt
        % der Trajektorie muss exakt bestimmt werden
        s.Phit_tol = 1e-9; s.Phir_tol = 1e-9;
      end
    end
    if R.Type == 0
      [q, Phi, Tc_stack] = R.invkin2(Traj_0.XE(i,:)', q0, s);
    else
      q0(R.I1J_LEG(2):end) = NaN; % Für Beinkette 2 Ergebnis von BK 1 nehmen
      [q, Phi, Tc_stack] = R.invkin2(Traj_0.XE(i,:)', q0, s, s_par); % kompilierter Aufruf
      if Set.general.debug_calc
        [~, Phi_debug, ~] = R.invkin_ser(Traj_0.XE(i,:)', q0, s, s_par); % Klassenmethode
        ik_res_ik2 = (all(abs(Phi(R.I_constr_t_red))<s.Phit_tol) && ...
            all(abs(Phi(R.I_constr_r_red))<s.Phir_tol));% IK-Status Funktionsdatei
        ik_res_iks = (all(abs(Phi_debug(R.I_constr_t_red))<s.Phit_tol) && ... 
            all(abs(Phi_debug(R.I_constr_r_red))<s.Phir_tol)); % IK-Status Klassenmethode
        if ik_res_ik2 ~= ik_res_iks % Vergleiche IK-Status (Erfolg / kein Erfolg)
          if Set.general.matfile_verbosity > 0
            save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_ik_error_debug.mat'));
          end
          % Mögliche Ursache: Mehr Glück mit Zufallszahlen für Anfangswert.
          % Das darf nicht allzu oft passieren. Die Zufallszahlen sollten
          % eigentlich gleich gebildet werden.
          cds_log(-1, sprintf(['IK-Berechnung mit Funktionsdatei hat anderen ', ...
            'Status (%d) als Klassenmethode (%d).'], ik_res_ik2, ik_res_iks));
        elseif ik_res_iks % beide IK erfolgreich
          % Dieser Test wird vorerst nicht weiter verfolgt (Ergebnisse nicht
          % identisch wegen unterschiedlicher Zufallszahlen)
%         ik_test_q = q - q_debug;
%         ik_test_Tcstack = Tc_stack - Tc_stack_debug;
%         if Set.general.matfile_verbosity > 0
%           save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_ikfkin_error_debug.mat'));
%         end
%         if max(abs(ik_test_Tcstack(:))) > 1e-2
%           warning('Tc_stack zwischen invkin2 und invkin_ser passen nicht. Max. Fehler: %1.1e', max(abs(ik_test_Tcstack(:))));
%         end
%         if max(abs(ik_test_q(:))) > 1e-2
%           warning('q zwischen invkin2 und invkin_ser passen nicht. Max. Fehler: %1.1e', max(abs(ik_test_q(:))));
%         end
        end
      end
    end
    % Normalisiere den Winkel. Bei manchen Robotern springt das IK-Ergebnis
    % sehr stark. Dadurch wird die Gelenkspannweite sonst immer verletzt.
    q(R.MDH.sigma==0) = normalize_angle(q(R.MDH.sigma==0));
    Phi_E(i,:) = Phi;
    if ~any(isnan(q))
      q0 = q; % Annahme: Startwert für nächsten Eckwert nahe aktuellem Eckwert
    end
    QE(i,:) = q;
    if any(abs(Phi(:)) > 1e-2) || any(isnan(Phi))
      break; % Breche Berechnung ab (zur Beschleunigung der Berechnung)
    end
    JPE(i,:) = Tc_stack(:,4); % Vierte Spalte ist Koordinatenursprung der Körper-KS
    if Set.general.debug_calc && R.Type == 2
      % Prüfe, ob die ausgegebenen Gelenk-Positionen auch stimmen
      JointPos_all_i_frominvkin = reshape(JPE(i,:)',3,1+R.NLEG+R.NJ);
      Tc_Lges = R.fkine_legs(QE(i,:)');
      JointPos_all_i_fromdirkin = [zeros(3,1), squeeze(Tc_Lges(1:3,4,1:end))];
      % Vergleiche die Positionen. In fkine_legs wird zusätzlich ein
      % virtuelles EE-KS ausgegeben, nicht aber in invkin_ser.
      for kk = 1:R.NLEG
        test_JPE = JointPos_all_i_frominvkin(:,kk+(-1+R.I1J_LEG(kk):R.I2J_LEG(kk))) - ...
        JointPos_all_i_fromdirkin(:,kk*2+(-2+R.I1J_LEG(kk):-1+R.I2J_LEG(kk)));
        if any(abs(test_JPE(:)) > 1e-8)
          if Set.general.matfile_verbosity > 0
            save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_trajjointpos_error_debug.mat'));
          end
          error(['Ausgegebene Gelenkpositionen stimmen nicht gegen direkte ', ...
            'Kinematik. Zeitpunkt %d, Beinkette %d. Max Fehler %1.1e'], i, kk, max(abs(test_JPE(:))));
        end
      end
    end
    % Prüfe, ob für die Berechnung der neuen Konfiguration das gleiche
    % rauskommt. Dann könnte man aufhören. Prüfe das für den ersten Eck-
    % punkt. Entspricht der ersten Berechnung.
    if jic > 1 && i == size(Traj_0.XE,1)
      test_vs_all_prev = repmat(q,1,jic-1)-reshape(squeeze(Q_jic(end,:,1:jic-1)),R.NJ,jic-1);
      % 2pi-Fehler entfernen
      test_vs_all_prev(abs(abs(test_vs_all_prev)-2*pi)<1e-1) = 0; % ungenaue IK ausgleichen
      if any(all(abs(test_vs_all_prev)<1e-2,1)) % Prüfe ob eine Spalte gleich ist wie die aktuellen Gelenkwinkel
        break;  % Das IK-Ergebnisse für den ersten Eckpunkt gibt es schon. Nicht weiter rechnen.
      end
    end
  end
  QE(isnan(QE)) = 0;
  Q_jic(:,:,jic) = QE;
  Phi_E(isnan(Phi_E)) = 1e6;
  if any(abs(Phi_E(:)) > 1e-2) || ... % Die Toleranz beim IK-Verfahren ist etwas größer
      any(abs(Phi_E(1,:))>1e-9) % Startpunkt für Traj. Hat feine Toleranz, sonst missverständliche Ergebnisse
    % Nehme die mittlere IK-Abweichung aller Eckpunkte (Translation/Rotation
    % gemischt). Typische Werte von 1e-2 bis 10.
    % Bei vorzeitigem Abbruch zählt die Anzahl der erfolgreichen Eckpunkte
    f_PhiE = mean(abs(Phi_E(:)));
    f_phiE_norm = 2/pi*atan(f_PhiE/0.9e6*35); % Normierung auf 0 bis 1. 0.9e6 -> 0.98
    fval_jic(jic) = 1e6*(1+9*f_phiE_norm); % Normierung auf 1e6 bis 1e7
    % Keine Konvergenz der IK. Weitere Rechnungen machen keinen Sinn.
    constrvioltext_jic{jic} = sprintf(['Keine IK-Konvergenz in Eckwerten. Untersuchte Eckpunkte: %d/%d. ', ...
      'Durchschnittliche ZB-Verl. %1.2e'], size(Traj_0.XE,1)-i+1,size(Traj_0.XE,1), f_PhiE);
    if jic<length(fval_jic), continue; else, break; end
  end

  %% Bestimme die Spannweite der Gelenkkoordinaten (getrennt Dreh/Schub)
  q_range_E = NaN(1, R.NJ);
  q_range_E(R.MDH.sigma==1) = diff(minmax2(QE(:,R.MDH.sigma==1)')');
  q_range_E(R.MDH.sigma==0) = angle_range( QE(:,R.MDH.sigma==0));
  % Bestimme ob die maximale Spannweite der Koordinaten überschritten wurde
  qlimviol_E = (qlim(:,2)-qlim(:,1))' - q_range_E;
  I_qlimviol_E = (qlimviol_E < 0);
  if any(I_qlimviol_E)
    if Set.general.matfile_verbosity > 2
      save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_qviolE.mat'));
    end
    % Bestimme die größte relative Verletzung der Winkelgrenzen
    [fval_qlimv_E, I_worst] = min(qlimviol_E(I_qlimviol_E)./(qlim(I_qlimviol_E,2)-qlim(I_qlimviol_E,1))');
    II_qlimviol_E = find(I_qlimviol_E); IIw = II_qlimviol_E(I_worst);
    fval_qlimv_E_norm = 2/pi*atan((-fval_qlimv_E)/0.3); % Normierung auf 0 bis 1; 2 ist 0.9
    fval = 1e5*(5+5*fval_qlimv_E_norm); % Normierung auf 5e5 bis 1e6
    fval_jic(jic) = fval;
    % Überschreitung der Gelenkgrenzen (bzw. -bereiche). Weitere Rechnungen machen keinen Sinn.
    constrvioltext_jic{jic} = sprintf(['Gelenkgrenzverletzung in AR-Eckwerten. ', ...
      'Schlechteste Spannweite: %1.2f/%1.2f (Gelenk %d)'], q_range_E(IIw), qlim(IIw,2)-qlim(IIw,1), IIw);
    if 1e4*fval < Set.general.plot_details_in_fitness
      change_current_figure(1000); clf; hold on;
      % Gut-Einträge: Dummy-NaN-Eintrag mit plotten, damit Handle für Legende nicht leer bleibt.
      hdl_iO= plot([find(~I_qlimviol_E),NaN], [QE(:,~I_qlimviol_E)-min(QE(:,~I_qlimviol_E)),NaN(size(QE,1),1)], 'co');
      hdl_niO=plot(find( I_qlimviol_E), QE(:, I_qlimviol_E)-min(QE(:, I_qlimviol_E)), 'bx'); % Kein NaN-Dummy notwendig.
      hdl1=plot(qlim(:,2)'-qlim(:,1)', 'r--');
      hdl2=plot([1;size(QE,2)], [0;0], 'm--');
      xlabel('Koordinate Nummer'); ylabel('Koordinate Wert');
      grid on;
      legend([hdl_iO(1);hdl_niO(1);hdl1;hdl2], {'iO-Gelenke', 'niO-Gelenke', 'qmax''', 'qmin''=0'});
      sgtitle(sprintf('Auswertung Grenzverletzung AR-Eckwerte. fval=%1.2e', fval));
    end
    if jic<length(fval_jic), continue; else, break; end
  end
  if Set.general.matfile_verbosity > 2
    save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_2.mat'));
  end
  % Debug:
  % load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_2.mat'));

  %% Aktualisiere Roboter für Kollisionsprüfung (geänderte Winkelgrenzen aus IK)
  if Set.optimization.constraint_collisions || ... % Kollisionsprüfung
      ~isempty(Set.task.installspace.type) || ... % Bauraum-Grenzen definiert
      ~isempty(Set.task.obstacles.type) || ... % Arbeitsraum-Objekte definiert
      ~isnan(Set.optimization.base_size_limits(2)) && any(Structure.I_firstprismatic) % Gestell-Schubachsen
    [Structure.collbodies_robot, Structure.installspace_collbodies] = ...
      cds_update_collbodies(R, Set, Structure, QE);
  end
  %% Prüfe Gestell-Durchmesser durch geänderte gestellfeste Führungsschienen
  if ~isnan(Set.optimization.base_size_limits(2)) && ...% Obere Grenze für Gestell-Radius gesetzt
      any(Structure.I_firstprismatic) % Es gibt ein gestellfestes Schubgelenk
    % Finde den effektiven Gestell-Radius heraus (weitesten vom
    % Mittelpunkt entfernte Enden der Führungsschienen)
    I_guidance = Structure.collbodies_robot.type==13;
    pts = [Structure.collbodies_robot.params(I_guidance,1:3); ...
           Structure.collbodies_robot.params(I_guidance,4:6)];
    % Max. Abstand aller Punkte der Führungsschiene von PKM-Basis (Maschinenmitte)
    r_base_eff = max((pts(:,1).^2 + pts(:,2).^2).^0.5);
    % Um so viel ist der Radius zu groß (bezogen auf Grenze). Die Grenze
    % für das Gestell wird in diesem Fall noch vergrößert, da die Führungs-
    % schienen nach oben abstehen.
    fval_rbase = r_base_eff/(Set.optimization.base_size_limits(2) * ...
      Set.optimization.base_tolerance_prismatic_guidance)-1;
    if fval_rbase > 0
      constrvioltext_jic{jic} = sprintf(['Gestell-Radius ist durch Schub', ...
        'achsen-Führungsschienen um %1.0f%% zu groß (%1.1fmm>%1.1fmm).'], ...
        100*fval_rbase, 1e3*r_base_eff, 1e3*1/((fval_rbase+1)/r_base_eff));
      fval_rbase_norm = 2/pi*atan(fval_rbase*3); % Normierung auf 0 bis 1; 100% zu groß ist 0.8
      fval = 1e5*(4+1*fval_rbase_norm); % Normierung auf 4e5 bis 5e5
      fval_jic(jic) = fval;
      if jic<length(fval_jic), continue; else, break; end
    end
  end
  %% Anpassung des Offsets für Schubgelenke
  % Hierdurch wird der Ort der Führungsschienen auf der Gelenkachse
  % verschoben. Das beeinflusst sowohl die Bauraumprüfung, als auch die
  % Prüfung auf Selbstkollision.
  if Structure.desopt_prismaticoffset
    % Bei Optimierung des Offsets wird auch bereits die Kollisionsprüfung
    % durchgeführt und die Ergebnisse weiter unten genutzt.
    [fval_coll_tmp, fval_instspc_tmp] = cds_desopt_prismaticoffset(R, ...
      Traj_0.XE, Set, Structure, JPE, QE);
    % Kollisionskörper müssen nochmal aktualisiert werden (wegen Offset)
    [Structure.collbodies_robot, Structure.installspace_collbodies] = ...
      cds_update_collbodies(R, Set, Structure, QE);
  else
    fval_coll_tmp = NaN; % Keine Berechnung durchgeführt ...
    fval_instspc_tmp = NaN; % ... Erstmalige Berechnung unten erforderlich.
  end
  %% Selbst-Kollisionsprüfung für Einzelpunkte
  if Set.optimization.constraint_collisions && ...
      (isnan(fval_coll_tmp) || fval_coll_tmp > 0) % nutze bereits vorliegende Daten
    [fval_coll, coll_self] = cds_constr_collisions_self(R, Traj_0.XE, Set, Structure, JPE, QE, [3e5;4e5]);
    if fval_coll > 0
      fval_jic(jic) = fval_coll; % Normierung auf 3e5 bis 4e5 bereits in Funktion
      constrvioltext_jic{jic} = sprintf('Selbstkollision in %d/%d AR-Eckwerten.', ...
        sum(any(coll_self,2)), size(coll_self,1));
      if jic<length(fval_jic), continue; else, break; end
    end
  end
  %% Bauraumprüfung für Einzelpunkte
  if ~isempty(Set.task.installspace.type) && ...
      (isnan(fval_instspc_tmp) || fval_instspc_tmp > 0) % nutze bereits vorliegende Daten
    [fval_instspc, f_constrinstspc] = cds_constr_installspace(R, Traj_0.XE, Set, Structure, JPE, QE, [2e5;3e5]);
    if fval_instspc > 0
      fval_jic(jic) = fval_instspc; % Normierung auf 2e5 bis 3e5 -> bereits in Funktion
      constrvioltext_jic{jic} = sprintf(['Verletzung des zulässigen Bauraums in AR-', ...
        'Eckpunkten. Schlimmstenfalls %1.1f mm draußen.'], 1e3*f_constrinstspc);
      if jic<length(fval_jic), continue; else, break; end
    end
  end
  %% Arbeitsraum-Hindernis-Kollisionsprüfung für Einzelpunkte
  if ~isempty(Set.task.obstacles.type)
    [fval_obstcoll, coll_obst, f_constr_obstcoll] = cds_constr_collisions_ws(R, Traj_0.XE, Set, Structure, JPE, QE, [1e5;2e5]);
    if fval_obstcoll > 0
      fval_jic(jic) = fval_obstcoll; % Normierung auf 1e5 bis 2e5 -> bereits in Funktion
      constrvioltext_jic{jic} = sprintf(['Arbeitsraum-Kollision in %d/%d AR-Eckwerten. ', ...
        'Schlimmstenfalls %1.1f mm in Kollision.'], sum(any(coll_obst,2)), ...
        size(coll_obst,1), 1e3*f_constr_obstcoll);
      if jic<length(fval_jic), continue; else, break; end
    end
  end
  fval_jic(jic) = 1e3; % Bis hier hin gekommen. Also erfolgreich.
end % Schleife über IK-Konfigurationen
%% IK-Konfigurationen für Eckpunkte auswerten. Nehme besten.
[fval, jic_best] = min(fval_jic);
constrvioltext = constrvioltext_jic{jic_best};

I_iO = find(fval_jic == 1e3);
if ~any(I_iO) % keine gültige Lösung für Eckpunkte
  Q0 = Q_jic(1,:,jic_best); % Gebe nur eine einzige Konfiguration aus
  QE_all = Q_jic(:,:,jic_best);
  % Wenn die IK nicht für alle Punkte erfolgreich war, wurde abgebrochen.
  % Dann stehen nur Nullen im Ergebnis. Schlecht zum Debuggen.
  if all(Q0==0)
    Q0 = QE_all(end,:); % Der letzte Wert wurde oben zuerst berechnet. Ergebnis liegt vor.
    % Setze für die Eckpunkte überall den (falschen) IK-Ergebniswert ein.
    % Macht das Debuggen einfacher (Bild plausibler)
    I_zeros = all(QE_all==0,2);
    QE_all(I_zeros,:) = repmat(Q0,sum(I_zeros),1);
  end
else % Gebe alle gültigen Lösungen aus
  Q0 = reshape(squeeze(Q_jic(1,:,I_iO)),R.NJ,length(I_iO))';
  % Falls der Roboter Schubgelenke hat, dominiert die Stellung der
  % Schubgelenke die Eigenschaften durch die IK-Konfiguration. Nehme nur
  % bezüglich der Schubgelenke unterschiedliche Konfigurationen. Sonst ist
  % der Rechenaufwand zu hoch. TODO: Feinere Unterscheidung für 3T3R-PKM
  if any(R.MDH.sigma==1)
    [~,I,~] = unique(round(Q0(:,R.MDH.sigma==1),5), 'rows', 'first');
    Q0 = Q0(I,:);
    I_iO = I_iO(I); % Entferne Indizes, die sich auf doppelte Schubgelenk-Konfig. beziehen
  end
  % Ausgabe der IK-Werte für alle Eckpunkte. Im weiteren Verlauf der
  % Optimierung benötigt, falls keine Trajektorie berechnet wird.
  QE_all = Q_jic(:,:,I_iO);
end

