function fval = cds_dimsynth_fitness_ser_plin(R_in, Set, Traj_W, Structure, p)
% Debug: 
% save(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_fitness_ser_plin.mat'));
% error('Halte hier');
% load(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_fitness_ser_plin.mat'));
t1=tic();
%% Parameter aktualisieren
R = cds_update_robot_parameters(R_in, Set, p);

%% Trajektorie anpassen
Traj_0 = cds_rotate_traj(Traj_W, R.T_W_0);

%% Inverse Kinematik für Eckpunkte der Trajektorie berechnen
q0 = rand(R.NQJ,1);
Phi_E = NaN(sum(Set.structures.DoF), size(Traj_0.XE,1));
for i = size(Traj_0.XE,1):-1:1
  s = struct('Phit_tol', 1e-3, 'Phir_tol', 1e-3, 'retry_limit', 5);
  [q, Phi] = R.invkin2(Traj_0.XE(i,:)', q0, s);
  q0 = q; % Annahme: Startwert für nächsten Eckwert nahe aktuellem Eckwert
  Phi_E(:,i) = Phi;
  if any(isnan(q))
    error('Ergebnis NaN');
  end
end
if any(abs(Phi_E(:)) > 1e-3)
  % Nehme die Summe der IK-Abweichung aller Eckpunkte (Translation/Rotation
  % gemischt)
  fval = 1e5*max(sum(abs(Phi_E(:))));
  % Keine Konvergenz der IK. Weitere Rechnungen machen keinen Sinn.
  fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3f. Keine IK-Konvergenz\n', toc(t1), fval);
  return
end
%% Inverse Kinematik der Trajektorie berechnen
[Q, ~, ~, PHI] = R.invkin2_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD, Traj_0.t, q);
if any(abs(PHI(:)) > 1e-3)
  fval = 1e4*max(abs(PHI(:)));
  % Keine Konvergenz der IK. Weitere Rechnungen machen keinen Sinn.
  fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3f. Keine IK-Konvergenz in Traj.\n', toc(t1), fval);
  return
end

if strcmp(Set.optimization.objective, 'condition')
  Cges = NaN(length(Traj_0.t), 1);
  % Berechne Konditionszahl für alle Punkte der Bahn
  for i = 1:length(Traj_0.t)
    J_3T3R = R.jacobig(Q(i,:)');
    J_task = J_3T3R(Set.structures.DoF,:);
    Cges(i,:) = cond(J_task);
  end
  fval = max(Cges); % Schlechtester Wert der Konditionszahl
end
fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3f\n', toc(t1), fval);