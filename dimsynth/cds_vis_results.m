% Visualisierung der Ergebnisse der Maßsynthese für einen Roboter

function cds_vis_results(Set, Traj, Structures)
resmaindir = fullfile(Set.optimization.resdir, Set.optimization.optname);
for i = 4%:length(Structures)
  % Ergebnisse laden
  Name = Structures{i}.Name;
  tmp = load(fullfile(resmaindir, ...
    sprintf('Rob%d_%s', i, Name)), 'RobotOptRes', 'Set', 'Traj');
  RobotOptRes = tmp.RobotOptRes;
  R = RobotOptRes.R;
  Q = RobotOptRes.Q;
  
  figure(10*i+1);clf;hold all;
  view(3);
  axis auto
  hold on;grid on;
  xlabel('x in m');ylabel('y in m');zlabel('z in m');
  plot3(Traj.X(:,1), Traj.X(:,2),Traj.X(:,3), 'k-');
  if Structures{i}.Type == 0 % Seriell
    s_anim = struct( 'gif_name', fullfile(resmaindir, sprintf('Rob%d_%s.gif', i, Name)));
    s_plot = struct( 'straight', 0);

    
    set(10*i+1,'units','normalized','outerposition',[0 0 1 1])
    R.anim( Q(1:20:end,:), s_anim, s_plot);
  else
    error('Nicht implementiert');
  end
  fprintf('%d/%d: Animation für %s gespeichert: %s\n', i, length(Structures), Name, s_anim.gif_name);
end