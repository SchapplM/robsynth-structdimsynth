function fval = cds_dimsynth_fitness_par(R_in, Set, Traj_W, Structure, p)
% Debug: 
% save(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_fitness_par1.mat'));
% error('Halte hier');
% load(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_fitness_par1.mat'));
t1=tic();
%% Parameter prüfen
if p(1) == 0
  error('Referenzlänge kann nicht Null werden');
end

%% Parameter aktualisieren
R = cds_update_robot_parameters(R_in, Set, p);

%% Trajektorie anpassen
Traj_0 = cds_rotate_traj(Traj_W, R.T_W_0);

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
  return
end
save(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_fitness_par2.mat'));
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
  return
end

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
  f_cond = log(max(Cges)); 
  f_cond_norm = 2/pi*atan((f_cond)/20); % Normierung auf 0 bis 1; 150 ist 0.9
  fval = 1e3*f_cond_norm; % Normiert auf 0 bis 1e3
end
fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3f\n', toc(t1), fval);

return
i=1;
% Debug: Bild zeichnen
figure(200);clf;hold all;
view(3);
axis auto
hold on;grid on;
xlabel('x in m');ylabel('y in m');zlabel('z in m');
plot3(Traj.X(:,1), Traj.X(:,2),Traj.X(:,3), 'k-');
set(200,'units','normalized','outerposition',[0 0 1 1])
s_plot = struct( 'ks_legs', [], 'straight', 0);
R.plot( Q(i,:)', Traj_0.X(i,:)', s_plot);