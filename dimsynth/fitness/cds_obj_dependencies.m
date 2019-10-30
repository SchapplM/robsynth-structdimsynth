function output = cds_obj_dependencies(R, Traj_0, Set, Q, QD, QDD, Jinvges)
output = struct('content', 'cds_obj_dependencies');
if any(strcmp(Set.optimization.objective, {'energy', 'minactforce'}))
  if R.Type == 0
    % Antriebskräfte berechnen
    TAU = R.invdyn2_traj(Q, QD, QDD);
  else
    % Trajektorie in Plattform-KS umrechnen
    [XP,XPD,XPDD] = R.xE2xP_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD);
    % Antriebskräfte berechnen
    Fx_red_traj = invdyn_platform_traj(R, Q, XP, XPD, XPDD);
    TAU = NaN(length(Traj_0.t), sum(R.I_qa));
    for i = 1:length(Traj_0.t)
      Jinv_IK = reshape(Jinvges(i,:), sum(R.I_EE), sum(R.I_qa));
      TAU(i,:) = (Jinv_IK') \ Fx_red_traj(i,:)';
    end
  end
  output.TAU = TAU;
end