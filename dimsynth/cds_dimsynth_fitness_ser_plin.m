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
  f_PhiE = max(sum(abs(Phi_E(:)))) / size(Traj_0.XE,1);
  f_phiE_norm = 2/pi*atan((f_PhiE)/10); % Normierung auf 0 bis 1
  fval = 1e5+9e5*f_phiE_norm; % Normierung auf 1e5 bis 1e6
  % Keine Konvergenz der IK. Weitere Rechnungen machen keinen Sinn.
  fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3e. Keine IK-Konvergenz\n', toc(t1), fval);
  return
end
%% Inverse Kinematik der Trajektorie berechnen
[Q, ~, ~, PHI] = R.invkin2_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD, Traj_0.t, q);
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
  % Berechne Konditionszahl für alle Punkte der Bahn
  for i = 1:length(Traj_0.t)
    J_3T3R = R.jacobig(Q(i,:)');
    J_task = J_3T3R(Set.structures.DoF,:);
    Cges(i,:) = cond(J_task);
  end
  % Schlechtester Wert der Konditionszahl
  % Nehme Logarithmus, da Konditionszahl oft sehr groß ist.
  f_cond = log(max(Cges)); 
  f_cond_norm = 2/pi*atan((f_cond)/20); % Normierung auf 0 bis 1; 150 ist 0.9
  fval = 1e3*f_cond_norm; % Normiert auf 0 bis 1e3
end
fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3f\n', toc(t1), fval);