% Visualisierung der Ergebnisse der Maßsynthese für einen Roboter
% 
% Eingabe:
% figname
%   Name des zu erstellenden Bildes. Entspricht Set.general.eval_figures
% Set
%   Einstellungen des Optimierungsalgorithmus. Siehe cds_settings_defaults.
% Traj
%   Trajektorie (bezogen auf Welt-KS)
% RobData
%   Daten zum Roboter. Größtenteils ähnlich zu Variable "Structures" aus
%   andere Funktionen.
% ResTab
%   Inhalt der Ergebnis-Tabelle aus cds_results_table.
% RobotOptRes
%   Zusammengefasste Ergebnisse der Maßsynthese. Siehe cds_dimsynth_robot.
% RobotOptDetails
%   Detaillierte Ergebnisse der Maßsynthese. Siehe cds_dimsynth_robot.
% 
% Siehe auch: cds_vis_results.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function fhdl = cds_vis_results_figures(figname, Set, Traj, RobData, ...
  ResTab, RobotOptRes, RobotOptDetails)
%% Initialisierung
RNr = RobData.Number;
PNr = RobData.ParetoNumber;
R = RobotOptDetails.R;
Traj_0 = cds_transform_traj(R, Traj);
Name = RobData.Name;
resmaindir = fullfile(Set.optimization.resdir, Set.optimization.optname);
resrobdir = fullfile(resmaindir, sprintf('Rob%d_%s', RNr, Name));
if all(RobotOptRes.fval > 1e3) % NB verletzt. Es gibt nur ein Kriterium
  fval_str = sprintf('%1.1e', RobotOptRes.fval(1));
elseif length(RobotOptRes.fval) == 1 % Einkriteriell
  fval_str = sprintf('%1.1f', RobotOptRes.fval);
else % Mehrkriteriell
  fval_str = ['[',disp_array(RobotOptRes.fval', '%1.1f'),']'];
end
%% Animation
if ~strcmp(figname, 'animation')
  Set.general.animation_styles = {};
end

% Hole Erklärungstext zum Fitness-Wert aus Tabelle
fval_text = ResTab.Fval_Text{strcmp(ResTab.Name, Name)};
for kk = 1:length(Set.general.animation_styles)
  anim_mode = Set.general.animation_styles{kk}; % Strichzeichnung, 3D-Modell, Kollisionskörper
  fhdl = figure();clf;hold all;
  set(fhdl, 'Name', sprintf('Rob%d_P%d_anim_%s', RNr, PNr, anim_mode), 'NumberTitle', 'off', 'color','w');
  if ~strcmp(get(fhdl, 'windowstyle'), 'docked')
    set(fhdl,'units','normalized','outerposition',[0 0 1 1]);
  end
  title(sprintf('Rob.%d, P.%d %s: fval=%s (%s)', RNr, PNr, Name, fval_str, fval_text));
  view(3);
  axis auto
  hold on;grid on;
  xlabel('x in m');ylabel('y in m');zlabel('z in m');
  plot3(Traj.X(:,1), Traj.X(:,2),Traj.X(:,3), 'k-');
  % Arbeitsraum-Objekte einzeichnen
  for co_sets = 1:2 % einmal Bauraum-Begrenzung, einmal Störkulissen
    if co_sets == 1, color = 'c'; collobjset = Set.task.installspace;
    else,            color = 'm'; collobjset = Set.task.obstacles; end
    for jj = 1:size(collobjset.type,1)
      switch collobjset.type(jj)
        case 1 % Quader
          q_W = collobjset.params(jj,1:3)';
          u1_W = collobjset.params(jj,4:6)';
          u2_W = collobjset.params(jj,7:9)';
          u3_W = cross(u1_W,u2_W); u3_W = u3_W/norm(u3_W)*collobjset.params(jj,10);
          % Umrechnen in Format der plot-Funktion
          cubpar_c = q_W(:)+(u1_W(:)+u2_W(:)+u3_W(:))/2; % Mittelpunkt des Quaders
          cubpar_l = [norm(u1_W); norm(u2_W); norm(u3_W)]; % Dimension des Quaders
          cubpar_a = 180/pi*tr2rpy([u1_W(:)/norm(u1_W), u2_W(:)/norm(u2_W), u3_W(:)/norm(u3_W)],'zyx')'; % Orientierung des Quaders
          drawCuboid([cubpar_c', cubpar_l', cubpar_a'], ...
            'FaceColor', color, 'FaceAlpha', 0.1);
        case 2 % Zylinder
          drawCylinder(collobjset.params(jj,1:7), ...
            'FaceColor', color, 'FaceAlpha', 0.1);
        case 3 % Kapsel
          drawCapsule(collobjset.params(jj,1:7),'FaceColor', color, 'FaceAlpha', 0.1);
        case 4 % Kugel
          drawSphere(collobjset.params(jj,1:4),'FaceColor', color, 'FaceAlpha', 0.1);
        otherwise
          warning('Geometrie mit Nummer %d kann nicht gezeichnet werden', collobjset.type(jj));
      end
    end % for collobjset.type
  end % for co_sets
  s_anim = struct('gif_name', '', 'avi_name', '', 'mp4_name', '');
  for kkk = 1:length(Set.general.save_animation_file_extensions)
    file_ext = Set.general.save_animation_file_extensions{kkk};
    s_anim.(sprintf('%s_name', file_ext)) = fullfile(resrobdir, ...
      sprintf('Rob%d_%s_P%d_Animation_%s.%s', RNr, Name, PNr, anim_mode, file_ext));
  end
  if Set.task.profile == 0
    I_anim = 1:size(RobotOptDetails.Traj_Q,1); % Zeichne jeden Zeitschritt
  else
    if Traj_0.t(end) > isinf(Set.general.maxduration_animation)
      % Reduziere das Video auf die maximale Länge. Die Abtastrate ist 30Hz
      % Nehme das Verhältnis von Ist- und Soll-Zeit als Beschleunigungsfaktor
      % dieser vorgegebenen Abtastrate des Videos
      t_Vid = (0:1/30*(Traj_0.t(end)/Set.general.maxduration_animation):Traj_0.t(end))';
    else
      % Stelle so ein, dass eine Abtastrate von 30Hz erreicht wird. Das Video
      % läuft dann genauso schnell wie die Trajektorie.
      t_Vid = (0:1/30:Traj_0.t(end))'; % Zeitstempel des Videos
    end
    I_anim = knnsearch( Traj_0.t , t_Vid ); % Berechne Indizes in Traj.-Zeitstempeln
  end
  if RobData.Type == 0 % Seriell
    s_plot = struct( 'straight', 1);
  else % Parallel
    s_plot = struct( 'ks_legs', [], 'straight', 1);
    if kk == 2, s_plot.mode = 4; end
  end
  if strcmp(anim_mode, 'stick')
    s_plot.mode = 1;
  elseif strcmp(anim_mode, '3D')
    s_plot.mode = 4;
  elseif strcmp(anim_mode, 'collision')
    s_plot.mode = 5;
  else
    error('Modus %s für Animation nicht definiert', anim_mode);
  end
  R.anim( RobotOptDetails.Traj_Q(I_anim,:), Traj_0.X(I_anim,:), s_anim, s_plot);
  if any(strcmp(Set.general.eval_figures, 'robvisuanim')) % nur speichern, wenn gewünscht.
  saveas(fhdl,     fullfile(resrobdir, sprintf('Rob%d_%s_P%d_Skizze_%s.fig', RNr, Name, PNr, anim_mode)));
  export_fig(fhdl, fullfile(resrobdir, sprintf('Rob%d_%s_P%d_Skizze_%s.png', RNr, Name, PNr, anim_mode)));
  end
end
%% Kinematik-Bild
if strcmp(figname, 'jointtraj')
  fhdl = figure();clf;hold all;
  set(fhdl, 'Name', sprintf('Rob%d_P%d_KinematikZeit', RNr, PNr), 'NumberTitle', 'off', 'color','w');
  if ~strcmp(get(fhdl, 'windowstyle'), 'docked')
    set(fhdl,'units','normalized','outerposition',[0 0 1 1]);
  end
  sgtitle(sprintf('Rob.%d, P.%d: fval=%s', RNr, PNr, fval_str));
  if RobData.Type == 0
    x_row = 2; nrows = 2;
  else
    x_row = 3; nrows = 3;
  end
  if RobData.Type == 0
    subplot(2,3,sprc2no(2,3,1,1));
    plot(Traj_0.t, RobotOptDetails.Traj_Q);
    grid on; ylabel('q in rad oder m');
    subplot(2,3,sprc2no(2,3,1,2));
    plot(Traj_0.t, RobotOptDetails.Traj_QD);
    grid on; ylabel('qD in rad/s oder m/s');
    subplot(2,3,sprc2no(2,3,1,3));
    plot(Traj_0.t, RobotOptDetails.Traj_QDD);
    grid on; ylabel('qDD in rad/s² oder m/s²');
  else
    subplot(3,3,sprc2no(3,3,1,1));
    plot(Traj_0.t, RobotOptDetails.Traj_Q(:,R.I_qa));
    grid on; ylabel('q_a in rad oder m');
    subplot(3,3,sprc2no(3,3,1,2));
    plot(Traj_0.t, RobotOptDetails.Traj_QD(:,R.I_qa));
    grid on; ylabel('qD_a in rad/s oder m/s');
    subplot(3,3,sprc2no(3,3,1,3));
    plot(Traj_0.t, RobotOptDetails.Traj_QDD(:,R.I_qa));
    grid on; ylabel('qDD_a in rad/s² oder m/s²');
    
    subplot(3,3,sprc2no(3,3,2,1));
    plot(Traj_0.t, RobotOptDetails.Traj_Q(:,R.I_qd));
    grid on; ylabel('q_p in rad oder m');
    subplot(3,3,sprc2no(3,3,2,2));
    plot(Traj_0.t, RobotOptDetails.Traj_QD(:,R.I_qd));
    grid on; ylabel('qD_p in rad/s oder m/s');
    subplot(3,3,sprc2no(3,3,2,3));
    plot(Traj_0.t, RobotOptDetails.Traj_QDD(:,R.I_qd));
    grid on; ylabel('qDD_p in rad/s² oder m/s²');
  end
  
  subplot(nrows,3,sprc2no(nrows,3,x_row,1));
  plot(Traj_0.t, Traj_0.X);
  grid on; ylabel('x in m oder rad');
  legend({'rx', 'ry', 'rz', 'phix', 'phiy', 'phiz'});
  subplot(nrows,3,sprc2no(nrows,3,x_row,2));
  plot(Traj_0.t, Traj_0.XD);
  grid on; ylabel('xD in m/s oder rad/s');
  subplot(nrows,3,sprc2no(nrows,3,x_row,3));
  plot(Traj_0.t, Traj_0.XDD);
  grid on; ylabel('xDD in m/s² oder rad/s²');
  linkxaxes;
  saveas(fhdl,     fullfile(resrobdir, sprintf('Rob%d_%s_P%d_KinematikZeit.fig', RNr, Name, PNr)));
  export_fig(fhdl, fullfile(resrobdir, sprintf('Rob%d_%s_P%d_KinematikZeit.png', RNr, Name, PNr)));
end
%% Zeichnung der Roboters mit Trägheitsellipsen und Ersatzdarstellung
if strcmp(figname, 'robvisu')
  fhdl = figure();clf;hold all;
  set(fhdl, 'Name', sprintf('Rob%d_P%d_Visu', RNr, PNr), 'NumberTitle', 'off', 'color','w');
  if ~strcmp(get(fhdl, 'windowstyle'), 'docked')
    set(fhdl,'units','normalized','outerposition',[0 0 1 1]);
  end
  sgtitle(sprintf('Rob.%d, P.%d: fval=%s', RNr, PNr, fval_str));
  plotmode = [1 3 4];
  for jj = 1:3
    subplot(2,2,jj);  hold on;grid on;
    view(3); axis auto;
    if RobData.Type == 0 % Seriell
      s_plot = struct( 'straight', 0, 'mode', plotmode(jj));
      R.plot(RobotOptDetails.Traj_Q(1,:)', s_plot);
    else
      s_plot = struct( 'ks_legs', [], 'straight', 0, 'mode', plotmode(jj));
      R.plot(RobotOptDetails.Traj_Q(1,:)', Traj_0.X(1,:)', s_plot);
    end
  end
  saveas(fhdl,     fullfile(resrobdir, sprintf('Rob%d_%s_P%d_Skizze_Plausib.fig', RNr, Name, PNr)));
  export_fig(fhdl, fullfile(resrobdir, sprintf('Rob%d_%s_P%d_Skizze_Plausib.png', RNr, Name, PNr)));
end