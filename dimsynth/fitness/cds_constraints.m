% Nebenbedingungen für Roboter-Maßsynthese berechnen (bzw. Strafterm aus
% Nebenbedingungsverletzung)
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% Traj_0, Traj_W
%   Endeffektor-Trajektorie (bezogen auf Basis-KS und Welt-KS)
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
%   1e5...3e5: Arbeitsraum-Hindernis-Kollision in Einzelpunkten
%   3e5...4e5: Bauraumverletzung in Einzelpunkten
%   4e5...5e5: Selbstkollision in Einzelpunkten
%   5e5...1e6: Gelenkwinkelgrenzen in Einzelpunkten
%   1e6...1e7: IK in Einzelpunkten nicht lösbar
%   1e7...1e8: Geometrie nicht plausibel lösbar (2: Reichweite PKM-Koppelpunkte)
%   1e8...1e9: Geometrie nicht plausibel lösbar (1: Schließen PKM-Ketten)
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
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [fval,Q,QD,QDD,Jinv_ges,constrvioltext] = cds_constraints(R, Traj_0, Traj_W, Set, Structure)
fval = 1e3;
constrvioltext = '';
Q = NaN(1,R.NJ);
QD = [];
QDD = [];
Jinv_ges = [];
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
  Phi_E = NaN(sum(Set.structures.DoF), size(Traj_0.XE,1));
  QE = NaN(size(Traj_0.XE,1), R.NQJ);
  if Set.task.profile ~= 0
    % Normale Trajektorie mit stetigem Zeitverlauf. Nur Berechnung der
    % Eckpunkte zur Prüfung. Setze die Zufallszahlen-Initialisierung mit
    % rng_seed, damit die Ergebnisse exakt reproduzierbar werden.
    s = struct('Phit_tol', 1e-3, 'Phir_tol', 1e-3, 'retry_limit', 5, ...
      'normalize', false, 'rng_seed', 0);
  else
    % Eckpunkte haben keinen direkten Bezug zueinander und bilden die
    % Trajektorie. Da keine Traj. berechnet wird, kann hier mehr Aufwand
    % betrieben werden (besonders bei seriellen Robotern auch notwendig.
    s = struct('Phit_tol', 1e-9, 'Phir_tol', 1e-9, 'retry_limit', 10, ...
      'normalize', false, 'n_max', 5000, 'rng_seed', 0);
  end
  % Variable zum Speichern der Gelenkpositionen (für Kollisionserkennung)
  JPE = NaN(size(Traj_0.XE,1), R.NL*3);
else % PKM
  qlim = cat(1,R.Leg(:).qlim);
  nPhi = R.I2constr_red(end);
  Phi_E = NaN(nPhi, size(Traj_0.XE,1));
  QE = NaN(size(Traj_0.XE,1), R.NJ);
  if Set.task.profile ~= 0 % Normale Trajektorie mit stetigem Zeitverlauf
    s = struct('Phit_tol', 1e-4, 'Phir_tol', 1e-3, 'retry_limit', 5, ...
      'normalize', false, 'rng_seed', 0);
  else % Nur Eckpunkte
    s = struct('Phit_tol', 1e-9, 'Phir_tol', 1e-9, 'retry_limit', 10, ...
      'normalize', false, 'n_max', 5000, 'rng_seed', 0);
  end
  JPE = NaN(size(Traj_0.XE,1), (R.NL-1+R.NLEG)*3);
end
fval_jic = NaN(1,3);
constrvioltext_jic = cell(1,3);
Q_jic = NaN(size(Traj_0.XE,1), R.NJ, 3);
for jic = 1:3 % Schleife über IK-Konfigurationen
q0 = qlim(:,1) + rand(R.NJ,1).*(qlim(:,2)-qlim(:,1));
% Anpassung der IK-Anfangswerte für diesen Durchlauf der IK-Konfigurationen.
% Versuche damit eine andere Konfiguration zu erzwingen
if fval_jic(1) > 1e6
  % IK hat beim ersten Mal schon nicht funktioniert (dort werden aber
  % zufällige Neuversuche gemacht). Andere Anfangswerte sind zwecklos.
  continue
end
  
if jic > 1 && ~any(R.MDH.sigma==1)
  % Die Wahl einer anderen Konfiguration ist aktuell nur für Schubgelenke
  % definiert.
  continue
end
if jic == 2
  % Setze die Anfangswerte (für Schubgelene) ganz weit nach "links"
  q0(R.MDH.sigma==1) = q0(R.MDH.sigma==1)-1.5*(qlim(R.MDH.sigma==1,2)-qlim(R.MDH.sigma==1,1));
elseif jic == 3
  % Anfangswerte weit nach rechts
  q0(R.MDH.sigma==1) = q0(R.MDH.sigma==1)+1.5*(qlim(R.MDH.sigma==1,2)-qlim(R.MDH.sigma==1,1));
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
    if i == size(Traj_0.XE,1)-1
      % Annahme: Kein Neuversuch der IK. Wenn die Gelenkwinkel zufällig neu
      % gewählt werden, springt die Konfiguration voraussichtlich. Dann ist
      % die Durchführung der Trajektorie unrealistisch.
      s.retry_limit = 0;
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
    [q, Phi, Tc_stack] = R.invkin2(Traj_0.XE(i,:)', q0, s); % kompilierter Aufruf
    if Set.general.debug_calc
      [q_debug, Phi_debug, Tc_stack_debug] = R.invkin_ser(Traj_0.XE(i,:)', q0, s); % Klassenmethode
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
  Phi_E(:,i) = Phi;
  if ~any(isnan(q))
    q0 = q; % Annahme: Startwert für nächsten Eckwert nahe aktuellem Eckwert
  end
  QE(i,:) = q;
  if any(abs(Phi(:)) > 1e-2)
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
end
QE(isnan(QE)) = 0;
Phi_E(isnan(Phi_E)) = 1e6;
if any(abs(Phi_E(:)) > 1e-2) % Die Toleranz beim IK-Verfahren ist etwas größer
  % Nehme die mittlere IK-Abweichung aller Eckpunkte (Translation/Rotation
  % gemischt). Typische Werte von 1e-2 bis 10.
  % Bei vorzeitigem Abbruch zählt die Anzahl der erfolgreichen Eckpunkte
  f_PhiE = mean(abs(Phi_E(:)));
  f_phiE_norm = 2/pi*atan(f_PhiE/0.9e6*35); % Normierung auf 0 bis 1. 0.9e6 -> 0.98
  fval_jic(jic) = 1e6*(1+9*f_phiE_norm); % Normierung auf 1e6 bis 1e7
  % Keine Konvergenz der IK. Weitere Rechnungen machen keinen Sinn.
  constrvioltext_jic{jic} = sprintf(['Keine IK-Konvergenz in Eckwerten. Untersuchte Eckpunkte: %d/%d. ', ...
    'Durchschnittliche ZB-Verl. %1.2f'], size(Traj_0.XE,1)-i+1,size(Traj_0.XE,1), f_PhiE);
  Q_jic(:,:,jic) = QE; % Ausgabe dient nur zum Zeichnen des Roboters
  if jic<length(fval_jic), continue; else, break; end
end

%% Bestimme die Spannweite der Gelenkkoordinaten (getrennt Dreh/Schub)
QE_korr = [QE; QE(end,:)];
% Berücksichtige Sonderfall des erten Schubgelenks bei der Bestimmung der
% Gelenkposition-Spannweite
if R.Type == 2
  % Hänge Null-Koordinate an, damit erstes Schubgelenk keine sehr große
  % Auslenkung haben kann. Das widerspricht der Anordnung von Basis und
  % Koppelpunkten. TODO: Bessere Methode dafür finden.
  QE_korr(end,Structure.I_firstprismatic) = 0;
end
q_range_E = NaN(1, R.NJ);
q_range_E(R.MDH.sigma==1) = diff(minmax2(QE_korr(:,R.MDH.sigma==1)')');
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
  if fval < Set.general.plot_details_in_fitness
    change_current_figure(1000); clf; hold on;
    hdl_iO= plot(find(~I_qlimviol_E), QE_korr(:,~I_qlimviol_E)-min(QE_korr(:,~I_qlimviol_E)), 'co');
    hdl_niO=plot(find( I_qlimviol_E), QE_korr(:, I_qlimviol_E)-min(QE_korr(:, I_qlimviol_E)), 'bx');
    hdl1=plot(qlim(:,2)'-qlim(:,1)', 'r--');
    hdl2=plot([1;size(QE,2)], [0;0], 'm--');
    xlabel('Koordinate Nummer'); ylabel('Koordinate Wert');
    grid on;
    legend([hdl_iO(1);hdl_niO(1);hdl1;hdl2], {'iO-Gelenke', 'niO-Gelenke', 'qmax''', 'qmin''=0'});
    sgtitle(sprintf('Auswertung Grenzverletzung AR-Eckwerte. fval=%1.2e', fval));
  end
  Q_jic(:,:,jic) = QE; % Ausgabe dient nur zum Zeichnen des Roboters
  if jic<length(fval_jic), continue; else, break; end
end
if Set.general.matfile_verbosity > 2
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_2.mat'));
end
% Debug:
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_2.mat'));

%% Aktualisiere Roboter für Kollisionsprüfung (geänderte Winkelgrenzen aus IK)
if Set.optimization.constraint_collisions || ...
    ~isempty(Set.task.installspace.type) || ~isempty(Set.task.obstacles.type)
  [Structure.collbodies_robot, Structure.installspace_collbodies] = ...
    cds_update_collbodies(R, Set, Structure, QE);
end
%% Selbst-Kollisionsprüfung für Einzelpunkte
if Set.optimization.constraint_collisions
  [fval_coll, coll_self] = cds_constr_collisions_self(R, Traj_0.XE, Set, Structure, JPE, QE, [4e5;5e5]);
  if fval_coll > 0
    fval_jic(jic) = fval_coll; % Normierung auf 4e5 bis 5e5 bereits in Funktion
    constrvioltext_jic{jic} = sprintf('Selbstkollision in %d/%d AR-Eckwerten.', ...
      sum(any(coll_self,2)), size(coll_self,1));
    Q_jic(:,:,jic) = QE; % Ausgabe dient nur zum Zeichnen des Roboters
    if jic<length(fval_jic), continue; else, break; end
  end
end

%% Bauraumprüfung für Einzelpunkte
if ~isempty(Set.task.installspace.type)
  [fval_instspc, f_constrinstspc] = cds_constr_installspace(R, Traj_0.XE, Set, Structure, JPE, QE, [3e5;4e5]);
  if fval_instspc > 0
    fval_jic(jic) = fval_instspc; % Normierung auf 3e5 bis 4e5 -> bereits in Funktion
    constrvioltext_jic{jic} = sprintf(['Verletzung des zulässigen Bauraums in AR-', ...
      'Eckpunkten. Schlimmstenfalls %1.1f mm draußen.'], 1e3*f_constrinstspc);
    Q_jic(:,:,jic) = QE; % Ausgabe dient nur zum Zeichnen des Roboters
    if jic<length(fval_jic), continue; else, break; end
  end
end
%% Arbeitsraum-Hindernis-Kollisionsprüfung für Einzelpunkte
if ~isempty(Set.task.obstacles.type)
  [fval_obstcoll, coll_obst, f_constr_obstcoll] = cds_constr_collisions_ws(R, Traj_0.XE, Set, Structure, JPE, QE, [1e5;3e5]);
  if fval_obstcoll > 0
    fval_jic(jic) = fval_obstcoll; % Normierung auf 1e5 bis 3e5 -> bereits in Funktion
    constrvioltext_jic{jic} = sprintf(['Arbeitsraum-Kollision in %d/%d AR-Eckwerten. ', ...
      'Schlimmstenfalls %1.1f mm in Kollision.'], sum(any(coll_obst,2)), size(coll_obst,1), f_constr_obstcoll);
    Q_jic(:,:,jic) = QE; % Ausgabe dient nur zum Zeichnen des Roboters
    if jic<length(fval_jic), continue; else, break; end
  end
end
end % Schleife über IK-Konfigurationen
%% IK-Konfigurationen für Eckpunkte auswerten. Nehme besten.
[fval, jic_best] = min(fval_jic);
constrvioltext = constrvioltext_jic{jic_best};
Q = Q_jic(:,:,jic_best);
% Speichere die Anfangs-Winkelstellung in der Roboterklasse für später.
% Dient zum Vergleich und zur Reproduktion der Ergebnisse
if R.Type == 0 % Seriell
  R.qref = Q(1,:)';
else
  for i = 1:R.NLEG, R.Leg(i).qref = Q(1,R.I1J_LEG(i):R.I2J_LEG(i))'; end
end
if fval > 1e3
  return % für keine IK-Konfiguration gültige Lösung. Abbruch.
end
QE = Q_jic(:,:,jic_best);
%% Inverse Kinematik der Trajektorie berechnen
if Set.task.profile ~= 0 % Nur Berechnen, falls es eine Trajektorie gibt
  % Einstellungen für IK in Trajektorien
  s = struct('normalize', false, ... % nicht notwendig, da Prüfen der Winkel-Spannweite. Außerdem sonst Sprung in Traj
    'retry_limit', 0, ... % keine Zufalls-Zahlen. Würde sowieso einen Sprung erzeugen.
    'n_max', 1000, ... % moderate Anzahl Iterationen
    'Phit_tol', 1e-8, 'Phir_tol', 1e-8);
  if R.Type == 0 % Seriell
    [Q, QD, QDD, PHI, JP] = R.invkin2_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD, Traj_0.t, q, s);
    Jinv_ges = NaN; % Platzhalter für gleichartige Funktionsaufrufe. Speicherung nicht sinnvoll für seriell.
  else % PKM
    [Q, QD, QDD, PHI, Jinv_ges, ~, JP] = R.invkin2_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD, Traj_0.t, q, s);
    if Set.general.debug_calc % Rechne nochmal mit Klassenmethode nach
      [~, ~, ~, PHI_debug, ~, ~, JP_debug] = R.invkin_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD, Traj_0.t, q, s);
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
      test_JPtraj = JP-JP_debug;
      if any(abs(test_JPtraj(:))>1e-6)
        error('Ausgabevariable JP aus invkin_traj vs invkin2_traj stimmt nicht');
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
    fval = 1e4*(1+9*Failratio); % Wert zwischen 1e4 und 1e5
    % Keine Konvergenz der IK. Weitere Rechnungen machen keinen Sinn.
    constrvioltext = sprintf('Keine IK-Konvergenz in Traj. Bis %1.0f%% (%d/%d) gekommen.', ...
      (1-Failratio)*100, IdxFirst, length(Traj_0.t));
    return
  end
else
  % Es liegt keine Trajektorie vor. Es reicht also, das Ergebnis der IK von
  % der Eckpunkt-Berechnung zu benutzen um die Jacobi-Matrix zu berechnen
  Q = QE;
  QD = 0*Q; QDD = 0*Q;
  if R.Type == 0 % Seriell
    Jinv_ges = NaN; % Platzhalter
  else % Parallel
    Jinv_ges = NaN(size(Q,1), sum(R.I_EE)*size(Q,2));
    for i = 1:size(Q,1)
      [~,J_x_inv] = R.jacobi_qa_x(Q(i,:)', Traj_0.X(i,:)');
      if any(isnan(J_x_inv(:))) || any(isinf(J_x_inv(:)))
        % Durch numerische Fehler können Inf- oder NaN-Einträge in der
        % Jacobi-Matrix entstehen (Singularität)
        if Set.general.matfile_verbosity > 2
          save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_J_infnan.mat'));
        end
        J_x_inv = zeros(size(J_x_inv));
      end
      Jinv_ges(i,:) = J_x_inv(:);
    end
  end
end
%% Prüfe, ob eine parasitäre Bewegung in der Trajektorie vorliegt
if strcmp(Set.optimization.objective, 'valid_act') && R.Type ~= 0 % nur sinnvoll bei PKM-Struktursynthese
  for jj = 1:length(Traj_0.t)
    [~,PhiD_jj] = R.constr4D(Q(jj,:)', QD(jj,:)', Traj_0.X(jj,:)',Traj_0.XD(jj,:)');
    if any(abs(PhiD_jj)> min(s.Phir_tol,s.Phit_tol))
      fval = 1e4; % Konstanter Wert (Bereich 9e3...1e4 erstmal ungenutzt)
      constrvioltext = sprintf('Es gibt eine parasitäre Bewegung.');
      return
    end
  end
end
%% Prüfe, ob die Gelenkwinkelgrenzen verletzt werden
Q_korr = [Q; Q(end,:)];
% Berücksichtige Sonderfall des erten Schubgelenks bei der Bestimmung der
% Gelenkposition-Spannweite für PKM (s.o.)
if R.Type == 2
  Q_korr(end,Structure.I_firstprismatic) = 0;
end
q_range_T = NaN(1, R.NJ);
q_range_T(R.MDH.sigma==1) = diff(minmax2(Q_korr(:,R.MDH.sigma==1)')');
q_range_T(R.MDH.sigma==0) = angle_range(Q(:,R.MDH.sigma==0));
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
  constrvioltext = sprintf('Gelenkgrenzverletzung in Traj. Schlechteste Spannweite: %1.2f/%1.2f', ...
    q_range_T(IIw), qlim(IIw,2)-qlim(IIw,1) );
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
if Set.task.profile ~= 0 % Nur Berechnen, falls es eine Trajektorie gibt
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
      RP = ['R', 'P'];
      change_current_figure(1001);clf;
      for i = 1:R.NJ
        legnum = find(i>=R.I1J_LEG, 1, 'last');
        legjointnum = i-(R.I1J_LEG(legnum)-1);
        subplot(ceil(sqrt(R.NJ)), ceil(R.NJ/ceil(sqrt(R.NJ))), i);
        hold on; grid on;
        plot(Traj_0.t, QD(:,i), '-');
        plot(Traj_0.t, QD_num(:,i), '--');
        plot(Traj_0.t([1,end]), repmat(Structure.qDlim(i,:),2,1), 'r--');
        ylim(minmax2([QD_num(:,i);QD_num(:,i)]'));
        title(sprintf('qD %d (%s), L%d,J%d', i, RP(R.MDH.sigma(i)+1), legnum, legjointnum));
      end
      linkxaxes
      sgtitle('Vergleich Gelenkgeschw.');
      change_current_figure(1002);clf;
      for i = 1:R.NJ
        subplot(ceil(sqrt(R.NJ)), ceil(R.NJ/ceil(sqrt(R.NJ))), i);
        hold on; grid on;
        plot(Traj_0.t, Q(:,i), '-');
        plot(Traj_0.t, Q_num(:,i), '--');
        title(sprintf('q %d (%s), L%d,J%d', i, RP(R.MDH.sigma(i)+1), legnum, legjointnum));
      end
      linkxaxes
      sgtitle('Verlauf Gelenkkoordinaten');
    end
    return
  end
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
    Q = QE; % Ausgabe dient nur zum Zeichnen des Roboters
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
    Q = QE; % Ausgabe dient nur zum Zeichnen des Roboters
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
      size(coll_obst,1), f_constr_obstcoll_traj);
    Q = QE; % Ausgabe dient nur zum Zeichnen des Roboters
    return
  end
end
