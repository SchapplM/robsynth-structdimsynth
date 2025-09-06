% Zeige eine Visualisierung von Aufgabe, Bauraum und Hindernissen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function cds_show_task(Traj, Set)
TaskSet = Set.task;

%% Aufgabe
T_ges = Traj.t;
X_ges = Traj.X;
XD_ges = Traj.XD;
XDD_ges = Traj.XDD;
XE = Traj.XE;
% Kippwinkelaus Euler-Winkeln berechnen
phiK_ges = NaN(length(T_ges),1);
for i = 1:length(phiK_ges)
  % Berechne z-Achse
  R = eulxyz2r(X_ges(i,4:6)');
  % Kippwinkel aus 
  phiK_ges(i) = acos(R(:,3)' * [0;0;1]);
end
change_current_figure(1);clf;
subplot(2,2,1);view(3);
plot3(1e3*X_ges(:,1), 1e3*X_ges(:,2), 1e3*X_ges(:,3));
xlabel('x in mm');ylabel('y in mm');zlabel('z in mm');
for i = 1:size(XE,1)
  text(1e3*XE(i,1), 1e3*XE(i,2), 1e3*XE(i,3), sprintf('%d', i));
end
grid on;
subplot(2,2,2);
plot(T_ges, 1e3*X_ges(:,1:3));
ylabel('Pos. in mm');
legend({'x', 'y', 'z'});
grid on;
subplot(2,2,3);
plot(T_ges, 180/pi*X_ges(:,4:6));
legend({'Eul X', 'Eul Y', 'Eul Z'});
ylabel('Euler-Winkel in Grad');
grid on;
subplot(2,2,4);
plot(T_ges, 180/pi*phiK_ges);
ylabel('Schwenk-Winkel in Grad');
grid on;
sgtitle('Position und Orientierung der Trajektorie');

change_current_figure(2);clf;
subplot(3,2,sprc2no(3,2,1,1));
plot(T_ges, 1e3*X_ges(:,1:3));
ylabel('Pos. in mm'); grid on;
legend({'x', 'y', 'z'});
subplot(3,2,sprc2no(3,2,2,1));
plot(T_ges, XD_ges(:,1:3));
ylabel('Geschw. in m/s'); grid on;
subplot(3,2,sprc2no(3,2,3,1));
plot(T_ges, XDD_ges(:,1:3));
ylabel('Beschl. in m/s²'); grid on;
xlabel(sprintf('Zeit in s (%d Samples)', length(T_ges)));
subplot(3,2,sprc2no(3,2,1,2));
plot(T_ges, 180/pi*X_ges(:,4:6));
ylabel('Winkel in deg'); grid on;
legend({'x', 'y', 'z'});
subplot(3,2,sprc2no(3,2,2,2));
plot(T_ges, 180/pi*XD_ges(:,4:6));
ylabel('Euler-Geschw. in deg/s'); grid on;
subplot(3,2,sprc2no(3,2,3,2));
plot(T_ges, 180/pi*XDD_ges(:,4:6));
ylabel('Euler-Beschl. in deg/s²'); grid on;
xlabel(sprintf('Zeit in s (%d Samples)', length(T_ges)));
linkxaxes
sgtitle('Zeitverlauf der Trajektorie');
%% Objekte und 3D-Aufgabe
change_current_figure(3);clf; hold all
view(3); axis equal; grid on;
xlabel('x in m');ylabel('y in m');zlabel('z in m');

% Trajektorie einzeichnen
plot3(X_ges(:,1),X_ges(:,2),X_ges(:,3),'k-');
% Eckpunkte als Koordinatensystem
for i = 1:size(XE,1)
  r_i = XE(i,1:3)';
  R_i = eulxyz2r(XE(i,4:6)');
  T_i = [R_i, r_i; [0 0 0 1]];
  trplot(T_i, 'rgb', 'labels', '   ', 'length', 5*1e-3);
end

n_iobj = size(TaskSet.installspace.type,1);
n_cobj = size(TaskSet.obstacles.type,1);
n_aobj = size(TaskSet.interactionspace.type,1);
collbodies = struct('params', [], 'type', []);
for i = 1:(n_iobj+n_cobj+n_aobj)
  if i <= n_iobj % Bauraum
    type_i = TaskSet.installspace.type(i);
    params_W = TaskSet.installspace.params(i,:);
  elseif i <= n_iobj+n_cobj % Hindernisse
    type_i = TaskSet.obstacles.type(i-n_iobj);
    params_W = TaskSet.obstacles.params(i-n_iobj,:);
  else
    type_i = TaskSet.interactionspace.type(i-n_iobj-n_cobj);
    params_W = TaskSet.interactionspace.params(i-n_iobj-n_cobj,:);
  end
  % Ändere Nummer "körperfestes Objekt" zu "Welt-festes Objekt" (für
  % Implementierung)
  if type_i == 1 % Quader
    type_i = uint8(10);
  elseif type_i == 2 % Zylinder
    type_i = uint8(12);
  elseif type_i == 3 % Kapsel
    type_i = uint8(13);
  end
  collbodies.params = [collbodies.params; ...
    [params_W, NaN(1,10-length(params_W))]];
  collbodies.type = [collbodies.type; type_i];
end
legh = NaN(3,1); % Handles für Legende
for i = 1:size(collbodies.type,1)
  params_W = collbodies.params(i,:);
  if i <= n_iobj % Bauraum
    color = 'g';
  elseif i <= n_iobj+n_cobj % Hindernisse
    color = 'r';
  else
    color = 'b'; % Interaktionsbereich
  end
  switch collbodies.type(i)
    case 4
      r = collbodies.params(i,4);
      h=drawSphere([params_W(1:3),r],'FaceColor', color, 'FaceAlpha', 0.3);
    case 6
      r = collbodies.params(i,1)*3; % Vergrößere den Radius für den Plot
      h=drawCapsule([params_W(1:3),params_W(4:6),r],'FaceColor', color, 'FaceAlpha', 0.3);
    case 9
      plot3(params_W(4), params_W(5), params_W(6), [color,'x'], 'markersize', 20);
    case 10
      % Parameter auslesen. Transformation ins Welt-KS für Plot
      q_W = eye(3,4)*[params_W(1:3)';1];
      u1_W = params_W(4:6)';
      u2_W = params_W(7:9)';
      % letzte Kante per Definition senkrecht auf anderen beiden.
      u3_W = cross(u1_W,u2_W); u3_W = u3_W/norm(u3_W)*params_W(10);
      % Umrechnen in Format der plot-Funktion
      cubpar_c = q_W(:)+(u1_W(:)+u2_W(:)+u3_W(:))/2; % Mittelpunkt des Quaders
      cubpar_l = [norm(u1_W); norm(u2_W); norm(u3_W)]; % Dimension des Quaders
      cubpar_a = 180/pi*tr2rpy([u1_W(:)/norm(u1_W), u2_W(:)/norm(u2_W), u3_W(:)/norm(u3_W)],'zyx')'; % Orientierung des Quaders
      h=drawCuboid([cubpar_c', cubpar_l', cubpar_a'], ...
        'FaceColor', color, 'FaceAlpha', 0.1);
    case 12
      % Transformation ins Welt-KS
      p1 = eye(3,4)*[params_W(1:3)';1];
      p2 = eye(3,4)*[params_W(4:6)';1];
      h=drawCylinder([p1', p2', params_W(7)], ...
        'FaceColor', color, 'FaceAlpha', 0.2);
    case 13
      % Transformation ins Welt-KS
      p1 = eye(3,4)*[params_W(1:3)';1];
      p2 = eye(3,4)*[params_W(4:6)';1];
      h=drawCapsule([p1', p2', params_W(7)], ...
        'FaceColor', color, 'FaceAlpha', 0.2);
    otherwise
      error('Der Fall %d darf nicht auftreten', collbodies.type(i));
  end
  if i <= n_iobj % Bauraum
    legh(1) = h(1);
  elseif i <= n_iobj+n_cobj % Hindernisse
    legh(2) = h(1);
  else
    legh(3) = h(1); % Interaktionsraum
  end
end
% Mögliche Basis-Positionen des Roboters
bpmean = mean(Set.optimization.basepos_limits,2);
if ~any(diff(Set.optimization.basepos_limits'))
  % Basis ist als Punkt festgelegt.
  legh(4) = plot3(bpmean(1),bpmean(2),bpmean(3), 'kx', 'MarkerSize', 3);
else
  % Basis kann in festgelegten Grenzen liegen
  plot3(Set.optimization.basepos_limits(1,:)',bpmean(2)*[1;1],...
    bpmean(3)*[1;1], 'k--', 'LineWidth', 3);
  plot3(bpmean(1)*[1;1],Set.optimization.basepos_limits(2,:)',...
    bpmean(3)*[1;1], 'k--', 'LineWidth', 3);
  legh(4) = plot3(bpmean(1)*[1;1],bpmean(2)*[1;1],Set.optimization.basepos_limits(3,:)', ...
    'k--', 'LineWidth', 3);
end
I_legh = ~isnan(legh);
if any(I_legh)
  lgtxt={'Bauraum (außerhalb unzulässig)', 'Hindernisse', 'Interaktionsraum', 'Basis-Position'};
  legend(legh(I_legh), lgtxt(I_legh));
end

sgtitle('Trajektorie und Objekte');
axis auto; 
drawnow();
