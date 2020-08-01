% Visualisierung der Ergebnisse der Maßsynthese für einen Roboter
% 
% Eingabe:
% Set
%   Einstellungen des Optimierungsalgorithmus
% Traj
%   Trajektorie (bezogen auf Welt-KS)
% Structures
%   Eigenschaften der Roboterstrukturen (alle an Optimierung beteiligten)
%   Siehe cds_gen_robot_list.m
% 
% Erzeugt Bilder:
% 1: Statistik der Ergebnisse
% 2, 3: Animation
% 4: Trägheitsellipsen
% 5: Gelenkverläufe
% 6: Zeitauswertung Fitness-Funktion
% 
% Speichert die Bilder für jeden Roboter in einem eigenen Unterordner

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function cds_vis_results(Set, Traj, Structures)
if Set.general.matfile_verbosity > 0
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_vis_results1.mat'));
end
if Set.general.nosummary
  return
end
% Zum Debuggen
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_vis_results1.mat'));

resmaindir = fullfile(Set.optimization.resdir, Set.optimization.optname);
% Ergebnistabelle laden
restabfile = fullfile(resmaindir, sprintf('%s_results_table.csv', Set.optimization.optname));
ResTab = readtable(restabfile, 'Delimiter', ';');

for i = 1:length(Structures)
  if Set.general.matfile_verbosity > 0
    save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_vis_results2.mat'));
  end
  %% Initialisierung der Ergebnisse dieser Struktur
  % load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_vis_results2.mat'));
  Structure = Structures{i};
  Name = Structures{i}.Name;
  resfile = fullfile(resmaindir, sprintf('Rob%d_%s_Endergebnis.mat', i, Name));
  if ~exist(resfile, 'file')
    warning('Ergebnis-Datei für Roboter %d (%s) existiert nicht: %s', i, Name, resfile);
    continue
  end
  tmp = load(resfile, 'RobotOptRes', 'Set', 'Traj', 'PSO_Detail_Data');
  resrobdir = fullfile(resmaindir, sprintf('Rob%d_%s', i, Name));
  mkdirs(resrobdir); % Speicherort für Bilder dieses Roboters
  RobotOptRes = tmp.RobotOptRes;
  PSO_Detail_Data = tmp.PSO_Detail_Data;
  R = RobotOptRes.R;
  if Structure.Type == 0
    serroblib_addtopath({Name});
  else
    parroblib_addtopath({Name});
  end
  Q = RobotOptRes.Traj_Q;
  Traj_0 = cds_rotate_traj(Traj, R.T_W_0);
  if Set.general.matfile_verbosity > 1
    save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_vis_results3.mat'));
  end
  % Zum Debuggen:
  % load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_vis_results3.mat'));
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
  Klassengrenzen_Alle = [0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9, 1e10, 1e11, 1e12];
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
  plot([3;3], [0;max(sum(Erg_All_Gen_Hist,1))], 'k-');
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
  
  
  saveas(10*i+1,     fullfile(resrobdir, sprintf('Rob%d_%s_Histogramm.fig', i, Name)));
  export_fig(10*i+1, fullfile(resrobdir, sprintf('Rob%d_%s_Histogramm.png', i, Name)));
  fprintf('%d/%d: Histogramm für %s gespeichert.\n', i, length(Structures), Name);
  %% Verschiedene Auswertungen
  figure(10*i+6);clf;
  sgtitle('Diverse Auswertungsbilder');
  % Verteilung der Rechenzeit über die Zielfunktionswerte
  % Streudiagramm
  subplot(2,2,1); hold all;
  plot(log10(PSO_Detail_Data.fval), PSO_Detail_Data.comptime, 'kx');
  xlabel('Zielfunktion (log)');
  ylabel('Rechenzeit in s');
  title('Rechenzeit je nach Zielfunktionswert');
  grid on;
  
  % Verteilung der Konditionszahlen in den Ergebnissen
  subplot(2,2,2); hold all;
  h = histogram(log10(PSO_Detail_Data.Jcond(:)));
  plot(log10(Set.optimization.constraint_obj(4))*[1;1], [0;1.2*max(h.Values)], 'k-', 'LineWidth', 2);
  xlim(minmax2(h.BinEdges)+h.BinWidth*[-1 1])
  title('Verteilung der Konditionszahlen über die Optimierung');
  xlabel('Konditionszahl (log)');
  ylabel(sprintf('Häufigkeit (Anzahl i.O.: %d/%d)', sum(~isnan(PSO_Detail_Data.Jcond(:))), ...
    length(PSO_Detail_Data.Jcond(:))));
    
  % Verteilung der Materialbeanspruchung gegen die Jacobi-Konditionszahl
  % Streudiagramm
  subplot(2,2,3); hold all;
  plot(log10(PSO_Detail_Data.Jcond(:)), 100*PSO_Detail_Data.f_maxstrengthviol(:), 'kx');
  plot([0;log10(max(PSO_Detail_Data.Jcond(:)))], [100;100], 'g--'); % Grenze Material
  plot(log10(Set.optimization.constraint_obj(4))*[1;1], [0;100], 'r--'); % Grenze für Jacobi
  if sum(~isnan(unique(PSO_Detail_Data.Jcond))) > 1
    xlim(log10(minmax2(PSO_Detail_Data.Jcond(:)'))); % geht nur, wenn zwei Werte da sind
  end
  if sum(~isnan(unique(PSO_Detail_Data.f_maxstrengthviol))) > 1
    ylim(100*minmax2(PSO_Detail_Data.f_maxstrengthviol(:)'));
  end
  xlabel('Konditionszahl (log)');
  ylabel('Materialbeanspruchung in Prozent');
  title('Materialbelastung vs Kondition');
  grid on;  
  
  saveas(10*i+6,     fullfile(resrobdir, sprintf('Rob%d_%s_Population_Fitness.fig', i, Name)));
  export_fig(10*i+6, fullfile(resrobdir, sprintf('Rob%d_%s_Population_Fitness.png', i, Name)));

  %% Animation des besten Roboters für die Trajektorie
  % Hole Erklärungstext zum Fitness-Wert aus Tabelle
  fval_text = ResTab.Fval_Text{strcmp(ResTab.Name, Name)};
  kk = 0;
  for anim_mode_cell = Set.general.animation_styles % Strichzeichnung, 3D-Modell, Kollisionskörper
  kk = kk + 1; anim_mode = anim_mode_cell{1}; % Zur Anpassung an cell-Eingabe.
  figure(10*i+1+kk);clf;hold all;
  set(10*i+1+kk, 'Name', sprintf('Rob%d_anim_%s', i, anim_mode), 'NumberTitle', 'off', 'color','w');
  if ~strcmp(get(10*i+1+kk, 'windowstyle'), 'docked')
    set(10*i+1+kk,'units','normalized','outerposition',[0 0 1 1]);
  end
  title(sprintf('Rob. %d: fval=%1.3e (%s)', i, RobotOptRes.fval, fval_text));
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
        otherwise
          warning('Geometrie mit Nummer %d kann nicht gezeichnet werden', collobjset.type(jj));
      end
    end % for collobjset.type
  end % for co_sets
  s_anim = struct('gif_name', '', 'avi_name', '', 'mp4_name', '');
  for file_ext = Set.general.save_animation_file_extensions
    s_anim.(sprintf('%s_name', file_ext{1})) = fullfile(resrobdir, sprintf('Rob%d_%s_Animation_%s.%s', i, Name, anim_mode, file_ext{1}));
  end
  if Set.task.profile == 0
    I_anim = 1:size(Q,1); % Zeichne jeden Zeitschritt
  else
    if Traj.t(end) > isinf(Set.general.maxduration_animation)
      % Reduziere das Video auf die maximale Länge. Die Abtastrate ist 30Hz
      % Nehme das Verhältnis von Ist- und Soll-Zeit als Beschleunigungsfaktor
      % dieser vorgegebenen Abtastrate des Videos
      t_Vid = (0:1/30*(Traj.t(end)/Set.general.maxduration_animation):Traj.t(end))';
    else
      % Stelle so ein, dass eine Abtastrate von 30Hz erreicht wird. Das Video
      % läuft dann genauso schnell wie die Trajektorie.
      t_Vid = (0:1/30:Traj.t(end))'; % Zeitstempel des Videos
    end
    I_anim = knnsearch( Traj.t , t_Vid ); % Berechne Indizes in Traj.-Zeitstempeln
  end
  if Structures{i}.Type == 0 % Seriell
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
  R.anim( Q(I_anim,:), Traj_0.X(I_anim,:), s_anim, s_plot);
  saveas(10*i+1+kk,     fullfile(resrobdir, sprintf('Rob%d_%s_Skizze_%s.fig', i, Name, anim_mode)));
  export_fig(10*i+1+kk, fullfile(resrobdir, sprintf('Rob%d_%s_Skizze_%s.png', i, Name, anim_mode)));
  fprintf('%d/%d: Animation für %s gespeichert nach %s\n', i, length(Structures), Name, resrobdir);
  end
  %% Zeichnung der Roboters mit Trägheitsellipsen und Ersatzdarstellung
  figure(10*i+4);clf;hold all;
  set(10*i+4, 'Name', sprintf('Rob%d_Visu', i), 'NumberTitle', 'off', 'color','w');
  if ~strcmp(get(10*i+4, 'windowstyle'), 'docked')
    set(10*i+4,'units','normalized','outerposition',[0 0 1 1]);
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
  saveas(10*i+4,     fullfile(resrobdir, sprintf('Rob%d_%s_Skizze_Plausib.fig', i, Name)));
  export_fig(10*i+4, fullfile(resrobdir, sprintf('Rob%d_%s_Skizze_Plausib.png', i, Name)));
  
  %% Verlauf der Gelenkgrößen für den besten Roboter
  figure(10*i+5);clf;hold all;
  set(10*i+5, 'Name', sprintf('Rob%d_KinematikZeit', i), 'NumberTitle', 'off', 'color','w');
  if ~strcmp(get(10*i+5, 'windowstyle'), 'docked')
    set(10*i+5,'units','normalized','outerposition',[0 0 1 1]);
  end
  sgtitle(sprintf('Rob. %d: fval=%1.3f', i, RobotOptRes.fval));
  if Structure.Type == 0
    x_row = 2; nrows = 2;
  else
    x_row = 3; nrows = 3;
  end
  if Structure.Type == 0
    subplot(2,3,sprc2no(2,3,1,1));
    plot(Traj.t, RobotOptRes.Traj_Q);
    grid on; ylabel('q in rad oder m');
    subplot(2,3,sprc2no(2,3,1,2));
    plot(Traj.t, RobotOptRes.Traj_QD);
    grid on; ylabel('qD in rad/s oder m/s');
    subplot(2,3,sprc2no(2,3,1,3));
    plot(Traj.t, RobotOptRes.Traj_QDD);
    grid on; ylabel('qDD in rad/s² oder m/s²');
  else
    subplot(3,3,sprc2no(3,3,1,1));
    plot(Traj.t, RobotOptRes.Traj_Q(:,R.I_qa));
    grid on; ylabel('q_a in rad oder m');
    subplot(3,3,sprc2no(3,3,1,2));
    plot(Traj.t, RobotOptRes.Traj_QD(:,R.I_qa));
    grid on; ylabel('qD_a in rad/s oder m/s');
    subplot(3,3,sprc2no(3,3,1,3));
    plot(Traj.t, RobotOptRes.Traj_QDD(:,R.I_qa));
    grid on; ylabel('qDD_a in rad/s² oder m/s²');
    
    subplot(3,3,sprc2no(3,3,2,1));
    plot(Traj.t, RobotOptRes.Traj_Q(:,R.I_qd));
    grid on; ylabel('q_p in rad oder m');
    subplot(3,3,sprc2no(3,3,2,2));
    plot(Traj.t, RobotOptRes.Traj_QD(:,R.I_qd));
    grid on; ylabel('qD_p in rad/s oder m/s');
    subplot(3,3,sprc2no(3,3,2,3));
    plot(Traj.t, RobotOptRes.Traj_QDD(:,R.I_qd));
    grid on; ylabel('qDD_p in rad/s² oder m/s²');
  end
  
  subplot(nrows,3,sprc2no(nrows,3,x_row,1));
  plot(Traj.t, Traj.X);
  grid on; ylabel('x in m oder rad');
  legend({'rx', 'ry', 'rz', 'phix', 'phiy', 'phiz'});
  subplot(nrows,3,sprc2no(nrows,3,x_row,2));
  plot(Traj.t, Traj.XD);
  grid on; ylabel('xD in m/s oder rad/s');
  subplot(nrows,3,sprc2no(nrows,3,x_row,3));
  plot(Traj.t, Traj.XDD);
  grid on; ylabel('xDD in m/s² oder rad/s²');
  linkxaxes;
  saveas(10*i+5,     fullfile(resrobdir, sprintf('Rob%d_%s_KinematikZeit.fig', i, Name)));
  export_fig(10*i+5, fullfile(resrobdir, sprintf('Rob%d_%s_KinematikZeit.png', i, Name)));
  
  if length(Structures) > 3
    close all; % schließe alle Bilder wieder. Sonst sind Hunderte Bilder am Ende offen
  end
  
  %% Finalisierung
  % Alle Auswertungsbilder wieder schließen. Sonst gibt es eventuell
  % Probleme mit dem Arbeitsspeicher.
  if Set.general.only_save_summary_figures
    close all;
  end  
end
