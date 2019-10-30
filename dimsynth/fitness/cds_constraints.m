% Nebenbedingungen für Roboter-Maßsynthese

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [fval,Q,QD,Jinvges,constrvioltext] = cds_constraints(R, Traj_0, Traj_W, Set, Structure)
fval = 0;
constrvioltext = '';
Q = [];
QD = [];
Jinvges = [];
%% Geometrie auf Plausibilität prüfen (1)
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


%% Geometrie auf Plausibilität prüfen (2)
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


%% Inverse Kinematik für Eckpunkte der Trajektorie berechnen
qlim_PKM = cat(1,R.Leg(:).qlim);
q0 = qlim_PKM(:,1) + rand(R.NJ,1).*(qlim_PKM(:,2)-qlim_PKM(:,1));

nPhi = R.I2constr_red(end);
Phi_E = NaN(nPhi, size(Traj_0.XE,1));
QE = NaN(size(Traj_0.XE,1), R.NJ);
for i = size(Traj_0.XE,1):-1:1
  s = struct('Phit_tol', 1e-4, 'Phir_tol', 1e-3, 'retry_limit', 5, ...
    'normalize', false);
  [q, Phi] = R.invkin_ser(Traj_0.XE(i,:)', q0, s);
  Phi_E(:,i) = Phi;
  if ~any(isnan(q))
    q0 = q; % Annahme: Startwert für nächsten Eckwert nahe aktuellem Eckwert
  end
  QE(i,:) = q;
end

QE(isnan(QE)) = 0;
Phi_E(isnan(Phi_E)) = 1e6;
if any(abs(Phi_E(:)) > 1e-2) % Die Toleranz beim IK-Verfahren ist etwas größer
  % Nehme die mittlere IK-Abweichung aller Eckpunkte (Translation/Rotation
  % gemischt). Typische Werte von 1e-2 bis 10
  f_PhiE = max(sum(abs(Phi_E(:)))) / size(Traj_0.XE,1);
  f_phiE_norm = 2/pi*atan((f_PhiE)/10); % Normierung auf 0 bis 1
  fval = 1e6*(1+9*f_phiE_norm); % Normierung auf 1e6 bis 1e7
  % Keine Konvergenz der IK. Weitere Rechnungen machen keinen Sinn.
  constrvioltext = sprintf('Keine IK-Konvergenz in Eckwerten. Durchschnittliche ZB-Verl. %1.2f', f_PhiE);
  return
end

% Bestimme die Spannweite der Gelenkkoordinaten (getrennt Dreh/Schub)
q_range_E = NaN(1, R.NJ);
q_range_E(R.MDH.sigma==1) = diff(minmax2(QE(:,R.MDH.sigma==1)')');
q_range_E(R.MDH.sigma==0) = angle_range(QE(:,R.MDH.sigma==0));
% Bestimme ob die maximale Spannweite der Koordinaten überschritten wurde
qlimviol_E = (qlim_PKM(:,2)-qlim_PKM(:,1))' - q_range_E;
I_qlimviol_E = (qlimviol_E < 0);
if any(I_qlimviol_E)
  if Set.general.matfile_verbosity > 2
    save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_fitness_par_qviolE.mat'));
  end
  % Bestimme die größte relative Verletzung der Winkelgrenzen
  [fval_qlimv_E, I_worst] = min(qlimviol_E(I_qlimviol_E)./(qlim_PKM(I_qlimviol_E,2)-qlim_PKM(I_qlimviol_E,1))');
  II_qlimviol_E = find(I_qlimviol_E); IIw = II_qlimviol_E(I_worst);
  fval_qlimv_E_norm = 2/pi*atan((-fval_qlimv_E)/0.3); % Normierung auf 0 bis 1; 2 ist 0.9
  fval = 1e5*(1+9*fval_qlimv_E_norm); % Normierung auf 1e5 bis 1e6
  % Überschreitung der Gelenkgrenzen (bzw. -bereiche). Weitere Rechnungen machen keinen Sinn.
  fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3e. Gelenkgrenzverletzung in AR-Eckwerten. Schlechteste Spannweite: %1.2f/%1.2f', ...
    toc(t1), fval, q_range_E(IIw), qlim_PKM(IIw,2)-qlim_PKM(IIw,1) );
  debug_plot_robot(R, QE(1,:)', Traj_0, Traj_W, Set, Structure, p, fval, debug_info);
  if fval < Set.general.plot_details_in_fitness
    change_current_figure(1000); clf; hold on;
    plot(1:size(QE,2), QE-min(QE), 'x');
    plot(qlim_PKM(:,2)'-qlim_PKM(:,1)', 'r--')
    plot([1;size(QE,2)], [0;0], 'r--')
    xlabel('Koordinate Nummer'); ylabel('Koordinate Wert');
    grid on;
    sgtitle(sprintf('Auswertung Grenzverletzung AR-Eckwerte. fval=%1.2e', fval));
  end
  return
end
if Set.general.matfile_verbosity > 2
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_fitness_par2.mat'));
end
% Debug:
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_fitness_par2.mat'));

%% Inverse Kinematik der Trajektorie berechnen
% s = struct('debug', true, 'retry_limit', 1);
s = struct('normalize', false, 'retry_limit', 1);
[Q, QD, ~, PHI, Jinvges] = R.invkin_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD, Traj_0.t, q, s);
% Speichere die Anfangs-Winkelstellung in der Roboterklasse für später
for i = 1:R.NLEG
  R.Leg(i).qref = q(R.I1J_LEG(i):R.I2J_LEG(i));
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
% Prüfe, ob die Gelenkwinkelgrenzen verletzt werden
q_range_T = NaN(1, R.NJ);
q_range_T(R.MDH.sigma==1) = diff(minmax2(Q(:,R.MDH.sigma==1)')');
q_range_T(R.MDH.sigma==0) = angle_range(Q(:,R.MDH.sigma==0));
qlimviol_T = (qlim_PKM(:,2)-qlim_PKM(:,1))' - q_range_T;
I_qlimviol_T = (qlimviol_T < 0);
if any(I_qlimviol_E)
  if Set.general.matfile_verbosity > 2
    save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_fitness_par_qviolT.mat'));
  end
  % Bestimme die größte relative Verletzung der Winkelgrenzen
  [fval_qlimv_T, I_worst] = min(qlimviol_T(I_qlimviol_T)./(qlim_PKM(I_qlimviol_T,2)-qlim_PKM(I_qlimviol_T,1))');
  II_qlimviol_T = find(I_qlimviol_T); IIw = II_qlimviol_T(I_worst);
  fval_qlimv_T_norm = 2/pi*atan((-fval_qlimv_T)/0.3); % Normierung auf 0 bis 1; 2 ist 0.9
  fval = 1e3*(1+9*fval_qlimv_T_norm); % Wert zwischen 1e3 und 1e4
  % Überschreitung der Gelenkgrenzen (bzw. -bereiche). Weitere Rechnungen machen keinen Sinn.
  constrvioltext = sprintf('Gelenkgrenzverletzung in Traj. Schlechteste Spannweite: %1.2f/%1.2f', ...
    q_range_T(IIw), qlim_PKM(IIw,2)-qlim_PKM(IIw,1) );
  if fval < Set.general.plot_details_in_fitness
    change_current_figure(1001); clf;
    plot(Traj_0.t, Q-repmat(min(Q), length(Traj_0.t), 1));
  end
  return
end