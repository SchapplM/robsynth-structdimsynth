% Visualisierung der Ergebnisse der Maßsynthese für einen Roboter

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function cds_vis_results(Set, Traj, Structures)
save(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_vis_results1.mat'));
% error('Halte hier');
% load(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_vis_results1.mat'));

resmaindir = fullfile(Set.optimization.resdir, Set.optimization.optname);

for i = 1:length(Structures)
  save(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_vis_results2.mat'));
  %% Initialisierung der Ergebnisse dieser Struktur
  % load(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_vis_results2.mat'));
  Structure = Structures{i};
  Name = Structures{i}.Name;
  tmp = load(fullfile(resmaindir, ...
    sprintf('Rob%d_%s_Endergebnis.mat', i, Name)), 'RobotOptRes', 'Set', 'Traj');
  RobotOptRes = tmp.RobotOptRes;
  R = RobotOptRes.R;
  if Structure.Type == 0
    serroblib_addtopath({Name});
  else
    parroblib_addtopath({Name});
  end
  Q = RobotOptRes.Traj_Q;
  Traj_0 = cds_rotate_traj(Traj, R.T_W_0);
  save(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_vis_results3.mat'));
  % load(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_vis_results3.mat'));
  %% Statistische Verteilung der Ergebnisse aller Generationen
  resdir_pso = fullfile(resmaindir, ...
    'tmp', sprintf('%d_%s', Structure.Number, Structure.Name));
  ergdatpso = dir(fullfile(resdir_pso, '*.mat'));
  Erg_All_Gen = NaN(length(ergdatpso), Set.optimization.NumIndividuals);
  for jj = 1:length(ergdatpso)
    tmp = load(fullfile(resdir_pso, ergdatpso(jj).name));
    Erg_All_Gen(jj,:) = tmp.optimValues.swarmfvals;
  end
  I_zul = Erg_All_Gen(:) < 1e3;
  Klassengrenzen_Alle = [0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9];
  Klassengrenzen_Alle_Log = log10(Klassengrenzen_Alle);
  Klassengrenzen_Alle_Log(1) = 0;
  % Histogramm erstellen
  figure(10*i+1);clf;hold all;
  set(10*i+1, 'Name', sprintf('Rob%d_Hist', i), 'NumberTitle', 'off', 'color','w');
  if ~strcmp(get(10*i+1, 'windowstyle'), 'docked')
    set(10*i+1,'units','normalized','outerposition',[0 0 1 1]);
  end
  sgtitle(sprintf('Erg.-Vert. für %s: %d Parametersätze in %d Gen.', Name, length(I_zul), size(Erg_All_Gen,1)));
  subplot(2,2,sprc2no(2,2,1,1)); % Histogramm über zulässige Lösungen
  h_zul = histogram(Erg_All_Gen(I_zul));
  edges = h_zul.BinEdges';
  grid on;
  xlim(minmax2(edges'));
  xlabel('Fitness');
  ylabel('Häufigkeit (abs)');
  title(sprintf('Zulässige Lösungen (%d)', sum(I_zul)));
  
  % Histogramm
  means = (edges(1:end-1)+edges(2:end))/2;
  % Histogramm für einzelne Generationen des PSO erstellen
  Erg_Zul_Gen_Hist = zeros(size(Erg_All_Gen,1), length(edges)-1);
  for ii = 1:size(Erg_Zul_Gen_Hist,1)
    for jj = 2:length(edges)
      Erg_Zul_Gen_Hist(ii,jj-1) = sum((Erg_All_Gen(ii,:) > edges(jj-1)) & (Erg_All_Gen(ii,:) < edges(jj)));
    end
  end

  % Gestapelte Säulen mit unterschiedlichen Farben plotten
  % Siehe https://de.mathworks.com/matlabcentral/answers/295950-how-can-i-get-a-stacked-bar-graph-with-a-single-bar#comment_516535
  subplot(2,2,sprc2no(2,2,2,1));cla;hold on;
  Farben = {};
  color_green = [0, 1, 0];
  color_blue = [0, 0, 1]; % color_orange = [1, 0.65, 0];
  for kk = 1:size(Erg_Zul_Gen_Hist,1) % Farbe entspricht Zähler der Generationen
    ant = kk/size(Erg_Zul_Gen_Hist,1);
    Farben{kk} = color_green*(1-ant) + color_blue*ant;
  end
  for ii = 1:size(Erg_Zul_Gen_Hist,2)
    legbarhdl = bar([means(ii);means(ii)+h_zul.BinWidth], [Erg_Zul_Gen_Hist(:,ii)'; NaN(1,size(Erg_Zul_Gen_Hist,1))], 'stacked');
    for j = 1:size(Erg_Zul_Gen_Hist,1)
      set(legbarhdl(j), 'EdgeColor', 'none', 'FaceColor', Farben{j});
    end
  end
  legend(legbarhdl([1,2,3,end]), {'Gen. 1', 'Gen. 2', '...', 'Letzte Gen.'});
  grid on;
  xlim(minmax2(edges'));
  xlabel('Fitness');
  ylabel('Häufigkeit (abs)');
  title('Vert. zul. Lsg. (nach Gen.)');

  subplot(2,2,sprc2no(2,2,1,2)); % Histogramm über alle Lösungen (Log)
  histogram(log10(Erg_All_Gen(:)), Klassengrenzen_Alle_Log);
  grid on;
  xlim(minmax2(Klassengrenzen_Alle_Log));
  xlabel('log(Fitness)');
  ylabel('Häufigkeit (abs)');
  title(sprintf('Alle Lösungen (%d)', length(I_zul)));
  set(gca, 'xtick', [2, 3.5, 4.5, 5.5, 6.5, 7.5, 8.5], ...
    'xticklabel', {'Ziel (0-1e3)', 'qlim Tr (1e4)', 'IK Tr (1e5)', 'qlim AR (1e6)', ...
                   'IK AR (1e7)', 'Rw (1e8)', 'G (1e9)'});
  
  % Histogramm für einzelne Generationen des PSO erstellen
  Erg_All_Gen_Hist = zeros(size(Erg_All_Gen,1), length(Klassengrenzen_Alle)-1);
  for ii = 1:size(Erg_All_Gen_Hist,1)
    for jj = 2:length(Klassengrenzen_Alle)
      Erg_All_Gen_Hist(ii,jj-1) = sum((Erg_All_Gen(ii,:) > Klassengrenzen_Alle(jj-1)) & (Erg_All_Gen(ii,:) < Klassengrenzen_Alle(jj)));
    end
  end
  subplot(2,2,sprc2no(2,2,2,2));hold on;
  for ii = 1:size(Erg_All_Gen_Hist,2)
    legbarhdl = bar([Klassengrenzen_Alle_Log(ii)+0.5;Klassengrenzen_Alle_Log(ii)+1.4], [Erg_All_Gen_Hist(:,ii)'; NaN(1,size(Erg_All_Gen_Hist,1))], 'stacked');
    for j = 1:size(Erg_All_Gen_Hist,1)
      set(legbarhdl(j), 'EdgeColor', 'none', 'FaceColor', Farben{j}, 'BarWidth', 1);
    end
  end
  grid on;
  xlim(minmax2(Klassengrenzen_Alle_Log))
  xlabel('log(Fitness)');
  ylabel('Häufigkeit (abs)');
  title('Vert. alle Lsg. (nach Gen.)');
  
  % Auskommentierte Plots: Nicht sinnvolle Kombinationen Log vs Bereich
%   subplot(2,2,sprc2no(2,2,1,2)); % Histogramm über alle Lösungen
%   histogram(Erg_All_Gen(:), Klassengrenzen_Log);
%   xlabel('fitness');
%   ylabel('Häufigkeit (abs)');
%   title(sprintf('Alle Lösungen (%d)', length(I_zul)));
  
%   subplot(2,2,sprc2no(2,2,2,1)); % Histogramm über zulässige Lösungen (Log)
%   histogram(log10(Erg_All_Gen(I_zul)));
%   xlabel('log(fitness)');
%   ylabel('Häufigkeit (abs)');
  
  
  saveas(10*i+1,     fullfile(resmaindir, sprintf('Rob%d_%s_Histogramm.fig', i, Name)));
  export_fig(10*i+1, fullfile(resmaindir, sprintf('Rob%d_%s_Histogramm.png', i, Name)));
  fprintf('%d/%d: Histogramm für %s gespeichert.\n', i, length(Structures), Name);
  %% Animation des besten Roboters für die Trajektorie
  figure(10*i+2);clf;hold all;
  set(10*i+2, 'Name', sprintf('Rob%d_anim', i), 'NumberTitle', 'off', 'color','w');
  if ~strcmp(get(10*i+2, 'windowstyle'), 'docked')
    set(10*i+2,'units','normalized','outerposition',[0 0 1 1]);
  end
  title(sprintf('Rob. %d: fval=%1.3f', i, RobotOptRes.fval));
  view(3);
  axis auto
  hold on;grid on;
  xlabel('x in m');ylabel('y in m');zlabel('z in m');
  plot3(Traj.X(:,1), Traj.X(:,2),Traj.X(:,3), 'k-');
  s_anim = struct('gif_name', '', 'avi_name', '');
  for file_ext = Set.general.save_animation_file_extensions
    s_anim.(sprintf('%s_name', file_ext{1})) = fullfile(resmaindir, sprintf('Rob%d_%s_Animation.%s', i, Name, file_ext{1}));
  end
  if Structures{i}.Type == 0 % Seriell
    s_plot = struct( 'straight', 0);
    R.anim( Q(1:20:end,:), s_anim, s_plot);
  else % Parallel
    s_plot = struct( 'ks_legs', [], 'straight', 0);
    R.anim( Q(1:20:end,:), Traj_0.X(1:20:end,:), s_anim, s_plot);
  end
  saveas(10*i+2,     fullfile(resmaindir, sprintf('Rob%d_%s_Skizze.fig', i, Name)));
  export_fig(10*i+2, fullfile(resmaindir, sprintf('Rob%d_%s_Skizze.png', i, Name)));
  fprintf('%d/%d: Animation für %s gespeichert: %s\n', i, length(Structures), Name, s_anim.gif_name);
  %% Zeichnung der Roboters mit Trägheitsellipsen und Ersatzdarstellung
  figure(10*i+3);clf;hold all;
  set(10*i+3, 'Name', sprintf('Rob%d_Visu', i), 'NumberTitle', 'off', 'color','w');
  if ~strcmp(get(10*i+3, 'windowstyle'), 'docked')
    set(10*i+3,'units','normalized','outerposition',[0 0 1 1]);
  end
  sgtitle(sprintf('Rob. %d: fval=%1.3f', i, RobotOptRes.fval));
  plotmode = [1 3 4];
  for jj = 1:3
    subplot(2,2,jj);  hold on;grid on;
    view(3); axis auto;
    if Structures{i}.Type == 0 % Seriell
      s_plot = struct( 'straight', 0, 'mode', plotmode(jj));
      R.plot(Q(1,:)', s_plot);
    else
      s_plot = struct( 'ks_legs', [], 'straight', 0, 'mode', plotmode(jj));
      R.plot(Q(1,:)', Traj_0.X(1,:)', s_plot);
    end
  end
  saveas(10*i+3,     fullfile(resmaindir, sprintf('Rob%d_%s_Skizze_Plausib.fig', i, Name)));
  export_fig(10*i+3, fullfile(resmaindir, sprintf('Rob%d_%s_Skizze_Plausib.png', i, Name)));
  
  %% Verlauf der Gelenkgrößen für den Roboter
  figure(10*i+4);clf;hold all;
  set(10*i+4, 'Name', sprintf('Rob%d_KinematikZeit', i), 'NumberTitle', 'off', 'color','w');
  if ~strcmp(get(10*i+4, 'windowstyle'), 'docked')
    set(10*i+4,'units','normalized','outerposition',[0 0 1 1]);
  end
  sgtitle(sprintf('Rob. %d: fval=%1.3f', i, RobotOptRes.fval));
  subplot(2,3,sprc2no(2,3,1,1));
  plot(Traj.t, RobotOptRes.Traj_Q);
  grid on; ylabel('q in rad oder m');
  subplot(2,3,sprc2no(2,3,1,2));
  plot(Traj.t, RobotOptRes.Traj_QD);
  grid on; ylabel('qD in rad/s oder m/s');
  subplot(2,3,sprc2no(2,3,1,3));
  plot(Traj.t, RobotOptRes.Traj_QDD);
  grid on; ylabel('qDD in rad/s² oder m/s²');
  subplot(2,3,sprc2no(2,3,2,1));
  plot(Traj.t, Traj.X);
  grid on; ylabel('x in m oder rad');
  legend({'rx', 'ry', 'rz', 'phix', 'phiy', 'phiz'});
  subplot(2,3,sprc2no(2,3,2,2));
  plot(Traj.t, Traj.XD);
  grid on; ylabel('xD in m/s oder rad/s');
  subplot(2,3,sprc2no(2,3,2,3));
  plot(Traj.t, Traj.XDD);
  grid on; ylabel('xDD in m/s² oder rad/s²');
  linkxaxes;
  saveas(10*i+4,     fullfile(resmaindir, sprintf('Rob%d_%s_KinematikZeit.fig', i, Name)));
  export_fig(10*i+4, fullfile(resmaindir, sprintf('Rob%d_%s_KinematikZeit.png', i, Name)));
  
  
  
  
end