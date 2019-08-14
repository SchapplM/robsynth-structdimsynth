function fval = cds_dimsynth_fitness_par(R_in, Set, Traj_W, Structure, p)
% Debug: 
% save(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_fitness_par1.mat'));
% error('Halte hier');
% load(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_fitness_par1.mat'));
t1=tic();
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
  % Nehme die Summe der IK-Abweichung aller Eckpunkte (Translation/Rotation
  % gemischt)
  fval = 1e5+1e5*max(sum(abs(Phi_E(:))));
  % Keine Konvergenz der IK. Weitere Rechnungen machen keinen Sinn.
  fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3f. Keine IK-Konvergenz in Eckwerten\n', toc(t1), fval);
  return
end
% save(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_fitness_par2.mat'));
%% Inverse Kinematik der Trajektorie berechnen
% s = struct('debug', true, 'retry_limit', 1);
s = struct('retry_limit', 1);
[Q, ~, ~, PHI] = R.invkin_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD, Traj_0.t, q, s);
if any(abs(PHI(:)) > 1e-3)
  fval = 1e4+1e4*max(abs(PHI(:)));
  % Keine Konvergenz der IK. Weitere Rechnungen machen keinen Sinn.
  fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3f. Keine IK-Konvergenz in Traj.\n', toc(t1), fval);
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
  fval = max(Cges); % Schlechtester Wert der Konditionszahl
end
fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3f\n', toc(t1), fval);