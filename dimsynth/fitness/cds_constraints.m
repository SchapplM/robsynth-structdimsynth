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
%   1e3: Keine Verletzung der Nebenbedingungen
%   1e3...5e3: Geschwindigkeitsgrenzen
%   5e3...1e4: Gelenkwinkelgrenzen
%   1e4...1e5: IK in Trajektorie nicht lösbar
%   ...: Siehe Quelltext
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
Q = [];
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
    constrvioltext = sprintf('Beinkette zu kurz für Plattform. Es fehlen max. %1.2fm.', l_legtooshort);
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
    constrvioltext = sprintf('Beinkette zu kurz für Bahnpunkte. Es fehlen max. %1.2fm.', -min(dist_exc_tot(:)));
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
    % Eckpunkte zur Prüfung
    s = struct('Phit_tol', 1e-3, 'Phir_tol', 1e-3, 'retry_limit', 5, ...
      'normalize', false);
  else
    % Eckpunkte haben keinen direkten Bezug zueinander und bilden die
    % Trajektorie. Da keine Traj. berechnet wird, kann hier mehr Aufwand
    % betrieben werden (besonders bei seriellen Robotern auch notwendig.
    s = struct('Phit_tol', 1e-9, 'Phir_tol', 1e-9, 'retry_limit', 10, ...
      'normalize', false, 'n_max', 5000);
  end
else % PKM
  qlim = cat(1,R.Leg(:).qlim);
  nPhi = R.I2constr_red(end);
  Phi_E = NaN(nPhi, size(Traj_0.XE,1));
  QE = NaN(size(Traj_0.XE,1), R.NJ);
  if Set.task.profile ~= 0
    s = struct('Phit_tol', 1e-4, 'Phir_tol', 1e-3, 'retry_limit', 5, ...
      'normalize', false);
  else
    s = struct('Phit_tol', 1e-9, 'Phir_tol', 1e-9, 'retry_limit', 10, ...
      'normalize', false, 'n_max', 5000);
  end
end
q0 = qlim(:,1) + rand(R.NJ,1).*(qlim(:,2)-qlim(:,1));
% IK für alle Eckpunkte, beginnend beim letzten (dann ist q der richtige
% Startwert für die Trajektorien-IK)
for i = size(Traj_0.XE,1):-1:1
  if R.Type == 0
    [q, Phi] = R.invkin2(Traj_0.XE(i,:)', q0, s);
  else
    [q, Phi] = R.invkin_ser(Traj_0.XE(i,:)', q0, s);
  end
  Phi_E(:,i) = Phi;
  if ~any(isnan(q))
    q0 = q; % Annahme: Startwert für nächsten Eckwert nahe aktuellem Eckwert
  end
  QE(i,:) = q;
  if any(abs(Phi(:)) > 1e-2)
    break; % Breche Berechnung ab (zur Beschleunigung der Berechnung)
  end
end
QE(isnan(QE)) = 0;
Phi_E(isnan(Phi_E)) = 1e6;
if any(abs(Phi_E(:)) > 1e-2) % Die Toleranz beim IK-Verfahren ist etwas größer
  % Nehme die mittlere IK-Abweichung aller Eckpunkte (Translation/Rotation
  % gemischt). Typische Werte von 1e-2 bis 10.
  % Bei vorzeitigem Abbruch zählt die Anzahl der erfolgreichen Eckpunkte
  f_PhiE = sum(abs(Phi_E(:))) / size(Traj_0.XE,1);
  f_phiE_norm = 2/pi*atan((f_PhiE)/10); % Normierung auf 0 bis 1
  fval = 1e6*(1+9*f_phiE_norm); % Normierung auf 1e6 bis 1e7
  % Keine Konvergenz der IK. Weitere Rechnungen machen keinen Sinn.
  constrvioltext = sprintf(['Keine IK-Konvergenz in Eckwerten. Untersuchte Eckpunkte: %d/%d. ', ...
    'Durchschnittliche ZB-Verl. %1.2f'], size(Traj_0.XE,1)-i+1,size(Traj_0.XE,1), f_PhiE);
  return
end

%% Bestimme die Spannweite der Gelenkkoordinaten (getrennt Dreh/Schub)
QE_korr = [QE; QE(end,:)];
% Berücksichtige Sonderfall des erten Schubgelenks bei der Bestimmung der
% Gelenkposition-Spannweite
if R.Type == 2
  % Hänge Null-Koordinate an, damit erstes Schubgelenk keine sehr große
  % Auslenkung haben kann. Das widerspricht der Anordnung von Basis und
  % Koppelpunkten.
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
  fval = 1e5*(1+9*fval_qlimv_E_norm); % Normierung auf 1e5 bis 1e6
  % Überschreitung der Gelenkgrenzen (bzw. -bereiche). Weitere Rechnungen machen keinen Sinn.
  constrvioltext = sprintf('Gelenkgrenzverletzung in AR-Eckwerten. Schlechteste Spannweite: %1.2f/%1.2f', ...
    q_range_E(IIw), qlim(IIw,2)-qlim(IIw,1) );
  if fval < Set.general.plot_details_in_fitness
    change_current_figure(1000); clf; hold on;
    plot(1:size(QE,2), QE-min(QE), 'x');
    plot(qlim(:,2)'-qlim(:,1)', 'r--')
    plot([1;size(QE,2)], [0;0], 'r--')
    xlabel('Koordinate Nummer'); ylabel('Koordinate Wert');
    grid on;
    sgtitle(sprintf('Auswertung Grenzverletzung AR-Eckwerte. fval=%1.2e', fval));
  end
  return
end
if Set.general.matfile_verbosity > 2
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_2.mat'));
end
% Debug:
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_2.mat'));

%% Inverse Kinematik der Trajektorie berechnen
if Set.task.profile ~= 0 % Nur Berechnen, falls es eine Trajektorie gibt
  % s = struct('debug', true, 'retry_limit', 1);
  s = struct('normalize', false, 'retry_limit', 10, 'n_max', 10000);
  if R.Type == 0 % Seriell
    [Q, QD, QDD, PHI] = R.invkin2_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD, Traj_0.t, q, s);
    Jinv_ges = NaN; % Platzhalter für gleichartige Funktionsaufrufe. Speicherung nicht sinnvoll für seriell.
  else % PKM
    [Q, QD, QDD, PHI, Jinv_ges] = R.invkin_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD, Traj_0.t, q, s);
  end
  % Speichere die Anfangs-Winkelstellung in der Roboterklasse für später
  if R.Type == 0 % Seriell
    R.qref = q;
  else
    for i = 1:R.NLEG
      R.Leg(i).qref = q(R.I1J_LEG(i):R.I2J_LEG(i));
    end
  end
  I_ZBviol = any(abs(PHI) > 1e-3,2) | any(isnan(Q),2);
  if any(I_ZBviol)
    % Bestimme die erste Verletzung der ZB (je später, desto besser)
    IdxFirst = find(I_ZBviol, 1 );
    % Umrechnung in Prozent der Traj.
    Failratio = 1-IdxFirst/length(Traj_0.t); % Wert zwischen 0 und 1
    fval = 1e4*(1+9*Failratio); % Wert zwischen 1e4 und 1e5
    % Keine Konvergenz der IK. Weitere Rechnungen machen keinen Sinn.
    constrvioltext = sprintf('Keine IK-Konvergenz in Traj. Bis %1.0f%% gekommen.', Failratio*100);
    return
  end
else
  % Es liegt keine Trajektorie vor. Es reicht also, das Ergebnis der IK von
  % der Eckpunkt-Berechnung zu benutzen
  Q = QE;
  QD = 0*Q; QDD = 0*Q;
  if R.Type == 0 % Seriell
    Jinv_ges = NaN; % Platzhalter
  else % Parallel
    Jinv_ges = NaN(size(Q,1), sum(R.I_EE)*size(Q,2));
    for i = 1:size(Q,1)
      [~,J_x_inv] = R.jacobi_qa_x(Q(i,:)', Traj_0.X(i,:)');
      Jinv_ges(i,:) = J_x_inv(:);
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
  fval = 1e3*(5+5*fval_qlimv_T_norm); % Wert zwischen 5e3 und 1e4
  % Überschreitung der Gelenkgrenzen (bzw. -bereiche). Weitere Rechnungen machen keinen Sinn.
  constrvioltext = sprintf('Gelenkgrenzverletzung in Traj. Schlechteste Spannweite: %1.2f/%1.2f', ...
    q_range_T(IIw), qlim(IIw,2)-qlim(IIw,1) );
  if fval < Set.general.plot_details_in_fitness
    change_current_figure(1001); clf;
    plot(Traj_0.t, Q-repmat(min(Q), length(Traj_0.t), 1));
  end
  return
end

%% Prüfe, ob die Geschwindigkeitsgrenzen (Antriebe) verletzt werden
% Diese Prüfung erfolgt zusätzlich zu einer Antriebsauslegung.
% Gedanke: Wenn die Gelenkgeschwindigkeit zu schnell ist, ist sowieso kein
% Antrieb auslegbar und die Parameter können schneller verworfen werden.
% Außerdem liegt wahrscheinlich eine Singularität vor.
if ~isinf(Set.optimization.max_velocity_active_revolute) && ~isinf(Set.optimization.max_velocity_active_prismatic)
  if R.Type == 0 % Seriell
    qaD_max = max(abs(QD));
    qD_lim = repmat(Set.optimization.max_velocity_active_revolute, 1, R.NJ);
    qD_lim(R.MDH.sigma==1) = Set.optimization.max_velocity_active_prismatic;
  else % PKM
    qaD_max = max(abs(QD(:,R.I_qa)));
    qD_lim = repmat(Set.optimization.max_velocity_active_revolute, 1, sum(R.I_qa));
    qD_lim(R.MDH.sigma(R.I_qa)==1) = Set.optimization.max_velocity_active_prismatic;
  end
  f_qD_exc = max(qaD_max./qD_lim);
  if f_qD_exc>1
    f_qD_exc_norm = 2/pi*atan((f_qD_exc-1)); % Normierung auf 0 bis 1; 1->0.5; 10->0.94
    fval = 1e3*(1+4*f_qD_exc_norm); % Wert zwischen 1e3 und 5e3
  end
  % Weitere Berechnungen voraussichtlich wenig sinnvoll, da vermutlich eine
  % Singularität vorliegt
  constrvioltext = sprintf('Geschwindigkeit des Antriebsgelenks zu hoch: max Verletzung %1.1f%%', ...
    (f_qD_exc-1)*100 );
  return
end