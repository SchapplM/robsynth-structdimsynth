% Gütefunktion für Parallele Roboter

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function fval = cds_dimsynth_fitness_par(R_in, Set, Traj_W, Structure, p)
% Debug: 
% save(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_fitness_par1.mat'));
% error('Halte hier');
% load(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_fitness_par1.mat'));

t1=tic();
%% Parameter prüfen
if p(1) == 0
  error('Roboterskalierung kann nicht Null werden');
end

%% Parameter aktualisieren
R = cds_update_robot_parameters(R_in, Set, Structure, p);

%% Trajektorie anpassen
Traj_0 = cds_rotate_traj(Traj_W, R.T_W_0);

%% Geometrie auf Plausibilität prüfen
% Berechne die Position der Koppelpunkte für die vorgesehenen Eckpunkte der
% Trajektorie.
% TODO: Funktioniert noch nicht bei Aufgabenredundanz.
dist_exc_tot = NaN(size(Traj_0.XE,1),R.NLEG);
dist_max = R.Leg(1).reach(); % Annahme: Symmetrischer Roboter; alle Beine max. gleich lang
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
    dist_exc_tot(j,i) = dist_max-dist_j;
  end
end
if any(dist_exc_tot(:) < 0)
  % Mindestens ein Punkt überschreitet die maximale Reichweite einer Beinkette
  f_distviol = -min(dist_exc_tot(:))/dist_max; % maximale Abstandsverletzung (relativ zu Maximalreichweite)
  % Werte sind typischerweise zwischen 0 und 100. Je kleiner die Beinkette,
  % desto größer der Wert; das regt dann zur Vergrößerung des Roboters an.
  f_distviol_norm = 2/pi*atan((f_distviol)); % 1->0.5; 10->0.94
  %  normiere auf 1e6 bis 1e7
  fval = 1e6+9e6*f_distviol_norm;
  fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3e. Beinkette zu kurz.\n', toc(t1), fval);
  debug_plot_robot(R, zeros(R.NJ,1), Traj_0, Traj_W, Set, Structure, p, fval);
  return
end
% basediam = 0;
% if any(Structure.vartypes == 6)
%   basediam = 2*p(Structure.vartypes == 6)*p(1);
% else
%   % TODO: Das ist noch eine zu große Annäherung an den Kreis-Wert. Besser
%   % Parameter abspeichern und auslesen
%   error('Noch nicht definiert');
%   basediam = norm(diff(minmax2(R.r_0_A_all)')');
% end
% maxleglength = sum(abs(p_lengthpar));
% lengthdeficit = basediam-maxleglength/2;
% TODO: Für jede Beinkette prüfen, ob alle Traj.-Punkte im theoretisch
% möglichen maximalen Arbeitsraum sind.

%% Inverse Kinematik für Eckpunkte der Trajektorie berechnen
q0 = rand(R.NJ,1);
nPhi = R.I2constr_red(end);
Phi_E = NaN(nPhi, size(Traj_0.XE,1));
for i = size(Traj_0.XE,1):-1:1
  s = struct('Phit_tol', 1e-4, 'Phir_tol', 1e-3, 'retry_limit', 5);
  [q, Phi] = R.invkin_ser(Traj_0.XE(i,:)', q0, s);
  Phi_E(:,i) = Phi;
  if ~any(isnan(q))
    q0 = q; % Annahme: Startwert für nächsten Eckwert nahe aktuellem Eckwert
  end
end

Phi_E(isnan(Phi_E)) = 1e6;
if any(abs(Phi_E(:)) > 1e-2) % Die Toleranz beim IK-Verfahren ist etwas größer
  % Nehme die mittlere IK-Abweichung aller Eckpunkte (Translation/Rotation
  % gemischt). Typische Werte von 1e-2 bis 10
  f_PhiE = max(sum(abs(Phi_E(:)))) / size(Traj_0.XE,1);
  f_phiE_norm = 2/pi*atan((f_PhiE)/10); % Normierung auf 0 bis 1
  fval = 1e5+9e5*f_phiE_norm; % Normierung auf 1e5 bis 1e6
  % Keine Konvergenz der IK. Weitere Rechnungen machen keinen Sinn.
  fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3e. Keine IK-Konvergenz in Eckwerten\n', toc(t1), fval);
  debug_plot_robot(R, zeros(R.NJ,1), Traj_0, Traj_W, Set, Structure, p, fval);
  return
end
% save(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_fitness_par2.mat'));
% Debug:
% load(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_fitness_par2.mat'));
%% Inverse Kinematik der Trajektorie berechnen
% s = struct('debug', true, 'retry_limit', 1);
s = struct('retry_limit', 1);
[Q, ~, ~, PHI] = R.invkin_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD, Traj_0.t, q, s);
I_ZBviol = any(abs(PHI) > 1e-3,2) | any(isnan(Q),2);
if any(I_ZBviol)
  % Bestimme die erste Verletzung der ZB (je später, desto besser)
  IdxFirst = find(I_ZBviol, 1 );
  % Umrechnung in Prozent der Traj.
  Failratio = 1-IdxFirst/length(Traj_0.t); % Wert zwischen 0 und 1
  fval = 1e4+9e4*Failratio; % Wert zwischen 1e4 und 1e5 -> IK-Abbruch bei Traj.
  % Keine Konvergenz der IK. Weitere Rechnungen machen keinen Sinn.
  fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3e. Keine IK-Konvergenz in Traj.\n', toc(t1), fval);
  debug_plot_robot(R, Q(1,:)', Traj_0, Traj_W, Set, Structure, p, fval);
  return
end

%% Zielfunktion berechnen
if strcmp(Set.optimization.objective, 'condition')
  Cges = NaN(length(Traj_0.t), 1);
  n_qa = sum(R.I_qa);
  % Berechne Konditionszahl für alle Punkte der Bahn
  for i = 1:length(Traj_0.t)
    Jinv_xred = R.jacobi_qa_x(Q(i,:)', Traj_0.X(i,:)');
    Jinv_3T3R = zeros(6, n_qa);
    Jinv_3T3R(R.I_EE,:) = Jinv_xred;
    Jinv_task = Jinv_3T3R(Set.structures.DoF,:);
    Cges(i,:) = cond(Jinv_task);
  end
  % Schlechtester Wert der Konditionszahl
  % Nehme Logarithmus, da Konditionszahl oft sehr groß ist.
  f_cond1 = max(Cges);
  f_cond = log(f_cond1); 
  f_cond_norm = 2/pi*atan((f_cond)/20); % Normierung auf 0 bis 1; 150 ist 0.9
  fval = 1e3*f_cond_norm; % Normiert auf 0 bis 1e3
  fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3f. Konditionszahl %1.3e\n', toc(t1),fval,  f_cond1);
else
  error('Zielfunktion nicht definiert');
end
debug_plot_robot(R, Q(1,:)', Traj_0, Traj_W, Set, Structure, p, fval);
end


function debug_plot_robot(R, q, Traj_0, Traj_W, Set, Structure, p, fval)
% Zeichne den Roboter für den aktuellen Parametersatz.
if ~Set.general.plot_robot_in_fitness
  return
end
figure(200);clf;hold all;
view(3);
axis auto
hold on;grid on;
xlabel('x in m');ylabel('y in m');zlabel('z in m');
plot3(Traj_W.X(:,1), Traj_W.X(:,2),Traj_W.X(:,3), 'k-');
set(200,'units','normalized','outerposition',[0 0 1 1])
s_plot = struct( 'ks_legs', [], 'straight', 0);
R.plot( q, Traj_0.X(1,:)', s_plot);
title(sprintf('fval=%1.2e; p=[%s]', fval,disp_array(p','%1.3f')));
xlim([-1,1]*Structure.Lref*3+mean(minmax2(Traj_W.XE(:,1)')'));
ylim([-1,1]*Structure.Lref*3+mean(minmax2(Traj_W.XE(:,2)')'));
zlim([-1,1]*Structure.Lref*1+mean(minmax2(Traj_W.XE(:,3)')'));
end