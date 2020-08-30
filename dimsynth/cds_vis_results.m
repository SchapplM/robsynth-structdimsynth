% Visualisierung der Ergebnisse der Maßsynthese für einen Roboter
% 
% Eingabe:
% Set
%   Einstellungen des Optimierungsalgorithmus. Hier insbesondere folgende:
%   .general.parcomp_plot
%     Plotten in parfor-Schleife: Geht schneller, aber kein openGL; also
%     ohne Transparenz-Effekte o.ä.
%   .general.eval_figures
%     Auswahl der zu erstellenden Bilder. Siehe Beschreibung unten.
%   .general.animation_styles
%     Steuert die zu speichernde Animation (keine, falls leer)
%   .general.save_animation_file_extensions
%     Dateiformat der Animation (gif und/oder mp4)
%   Ansonsten siehe cds_settings_defaults.m.
% Traj
%   Trajektorie (bezogen auf Welt-KS)
% Structures
%   Eigenschaften der Roboterstrukturen (alle an Optimierung beteiligten)
%   Siehe cds_gen_robot_list.m
% 
% Erzeugt Bilder:
% 1: Statistik der Ergebnisse ('histogram')
% 2, 3: Animation ('animation')
% 4: Trägheitsellipsen ('robvisu')
% 5: Gelenkverläufe ('jointtraj')
% 6: Diverse Auswertungen zur Fitness-Funktion ('fitness_various')
%    (Daten, die in cds_save_particle_details.m gespeichert werden:
%    Zeitauswertung, Kondition, Materialbelastung)
% 7: 2D-Pareto-Front (nur bei mehrkriterieller Optimierung)
% 8: 3D-Pareto-Front (nur falls 3 oder mehr Kriterien)
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

%% Initialisierung
resmaindir = fullfile(Set.optimization.resdir, Set.optimization.optname);
% Ergebnistabelle laden
restabfile = fullfile(resmaindir, sprintf('%s_results_table.csv', Set.optimization.optname));
ResTab = readtable(restabfile, 'Delimiter', ';');

%% Parallele Durchführung der Plots vorbereiten
if Set.general.parcomp_plot
  try %#ok<TRYNC>
    parpool(Set.general.parcomp_maxworkers);
  end
  Pool=gcp();
  parfor_numworkers = Pool.NumWorkers;
  if ~isinf(Set.general.parcomp_maxworkers) && parfor_numworkers ~= Set.general.parcomp_maxworkers
    warning('Die gewünschte Zahl von %d Parallelinstanzen konnte nicht erfüllt werden. Es sind jetzt %d.', ...
      Set.general.parcomp_maxworkers, parfor_numworkers)
  end
  % Warnungen auch in ParPool-Workern unterdrücken: https://github.com/altmany/export_fig/issues/75
  parfevalOnAll(gcp(), @warning, 0, 'off', 'MATLAB:prnRenderer:opengl');
else
  parfor_numworkers = 0;
end
%% Ergebnisse für jeden Roboter plotten
length_Structures = length(Structures);
parfor (i = 1:length_Structures, parfor_numworkers)
  t_start_i = tic();
  %% Initialisierung der Ergebnisse dieser Struktur
  % load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_vis_results2.mat'));
  Structure = Structures{i};
  Name = Structures{i}.Name;
  fprintf('Visualisiere Ergebnisse für Rob %d (%s)\n', i, Name);
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

  %% Statistische Verteilung der Ergebnisse aller Generationen
  if any(strcmp(Set.general.eval_figures, 'histogram')) %#ok<PFBNS>
  t1 = tic();
  Erg_All_Gen = PSO_Detail_Data.fval;
  I_zul = Erg_All_Gen(:) < 1e3;
  Klassengrenzen_Alle = [0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9, 1e10, 1e11, 1e12, 1e13];
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
  
  % Mehrere Generationen zusammenfassen. Ansonsten benötigt man bei vielen
  % Generationen teilweise Stunden, bis das Diagramm geplottet ist.
  n_genc = min(size(Erg_All_Gen,1),19); % max. 30 verschiedene Farben benutzen.
  nindvirt = ceil(length(Erg_All_Gen(:))/n_genc); % Generationen-Anzahl für Plot zusammenfassen
  Erg_All_GenT = Erg_All_Gen'; % Transponieren, da Spaltenweise indiziert wird
  Erg_All_Gen2T = NaN(nindvirt,n_genc);
  % Trage die Individuen der Reihe nach ein (Generationen stimmen nicht mehr exakt)
  Erg_All_Gen2T(1:length(Erg_All_Gen(:))) = Erg_All_GenT(:);
  Erg_All_Gen2 = Erg_All_Gen2T';
  Erg_All_Gen2 = Erg_All_Gen2(~all(isnan(Erg_All_Gen2),2),:); % Entferne reine NaN-Zeilen
  % Histogramm
  means = (edges(1:end-1)+edges(2:end))/2;
  % Histogramm für einzelne Generationen des PSO erstellen
  Erg_Zul_Gen_Hist = zeros(size(Erg_All_Gen2,1), length(edges)-1);
  for ii = 1:size(Erg_Zul_Gen_Hist,1)
    for jj = 2:length(edges)
      Erg_Zul_Gen_Hist(ii,jj-1) = sum((Erg_All_Gen2(ii,:) > edges(jj-1)) & (Erg_All_Gen2(ii,:) < edges(jj)));
    end
  end

  % Gestapelte Säulen mit unterschiedlichen Farben plotten
  % Siehe https://de.mathworks.com/matlabcentral/answers/295950-how-can-i-get-a-stacked-bar-graph-with-a-single-bar#comment_516535
  % TODO: Bessere Lösung seit Matlab R2019b verfügbar?
  subplot(2,2,sprc2no(2,2,2,1));cla;hold on;
  Farben = {};
  color_green = [0, 1, 0];
  color_blue = [0, 0, 1]; % color_orange = [1, 0.65, 0];
  for jj = 1:size(Erg_Zul_Gen_Hist,1) % Farbe entspricht Zähler der Generationen
    ant = jj/size(Erg_Zul_Gen_Hist,1);
    Farben{jj} = color_green*(1-ant) + color_blue*ant;
  end
  for ii = 1:size(Erg_Zul_Gen_Hist,2)
    legbarhdl = bar([means(ii);means(ii)+h_zul.BinWidth], [Erg_Zul_Gen_Hist(:,ii)'; NaN(1,size(Erg_Zul_Gen_Hist,1))], 'stacked');
    for j = 1:size(Erg_Zul_Gen_Hist,1)
      set(legbarhdl(j), 'EdgeColor', 'none', 'FaceColor', Farben{j});
    end
    if ii == size(Erg_Zul_Gen_Hist,2) % letzte Iteration. Trage Legende ein
      legend(legbarhdl([1,2,3,end]), {'Gen. 1', 'Gen. 2', '...', 'Letzte Gen.'});
    end
  end
  
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
  Erg_All_Gen_Hist = zeros(size(Erg_All_Gen2,1), length(Klassengrenzen_Alle)-1);
  for ii = 1:size(Erg_All_Gen_Hist,1)
    for jj = 2:length(Klassengrenzen_Alle)
      Erg_All_Gen_Hist(ii,jj-1) = sum((Erg_All_Gen2(ii,:) > Klassengrenzen_Alle(jj-1)) & (Erg_All_Gen2(ii,:) < Klassengrenzen_Alle(jj)));
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

  saveas(10*i+1,     fullfile(resrobdir, sprintf('Rob%d_%s_Histogramm.fig', i, Name)));
  export_fig(10*i+1, fullfile(resrobdir, sprintf('Rob%d_%s_Histogramm.png', i, Name)));
  fprintf('%d/%d: Histogramm für %s gespeichert. Dauer: %1.1fs\n', ...
    i, length_Structures, Name, toc(t1));
  end
  %% Verschiedene Auswertungen
  if any(strcmp(Set.general.eval_figures, 'fitness_various'))
  t1 = tic();
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
  fprintf('%d/%d: Weitere Auswertungsbilder für %s gespeichert. Dauer: %1.1fs\n', ...
    i, length_Structures, Name, toc(t1));
  end
  %% Animation des besten Roboters für die Trajektorie
  t1 = tic();
  % Hole Erklärungstext zum Fitness-Wert aus Tabelle
  fval_text = ResTab.Fval_Text{strcmp(ResTab.Name, Name)}; %#ok<PFBNS>
  for kk = 1:length(Set.general.animation_styles)
  anim_mode = Set.general.animation_styles{kk}; % Strichzeichnung, 3D-Modell, Kollisionskörper
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
  for kkk = 1:length(Set.general.save_animation_file_extensions)
    file_ext = Set.general.save_animation_file_extensions{kkk};
    s_anim.(sprintf('%s_name', file_ext)) = fullfile(resrobdir, ...
      sprintf('Rob%d_%s_Animation_%s.%s', i, Name, anim_mode, file_ext));
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
  fprintf('%d/%d: Animation für %s gespeichert nach %s. Dauer: %1.1fs\n', ...
    i, length_Structures, Name, resrobdir, toc(t1));
  end
  %% Zeichnung der Roboters mit Trägheitsellipsen und Ersatzdarstellung
  if any(strcmp(Set.general.eval_figures, 'robvisu'))
  t1 = tic();
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
  end
  %% Verlauf der Gelenkgrößen für den besten Roboter
  if any(strcmp(Set.general.eval_figures, 'jointtraj'))
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
  fprintf('%d/%d: Restliche Bilder für %s gespeichert. Dauer: %1.1fs\n', ...
    i, length_Structures, Name, toc(t1));
  end

  %% (2D)-Pareto-Fronten für die Zielkriterien
  if length(Set.optimization.objective) > 1 % Mehrkriterielle Optimierung
    % Gehe alle Kombinationen von zwei Zielkriterien durch (falls mehr als
    % zwei gewählt).
    objcomb = allcomb(1:length(Set.optimization.objective), 1:length(Set.optimization.objective));
    objcomb(objcomb(:,1)==objcomb(:,2),:) = [];
    objcomb(objcomb(:,1)>objcomb(:,2),:) = [];
    figure(10*i+7);clf;hold all;
    sprows = floor(sqrt(size(objcomb,1)));
    spcols = ceil(size(objcomb,1)/sprows);
    for kk = 1:size(objcomb,1)
      kk1 = objcomb(kk,1); kk2 = objcomb(kk,2);
      subplot(sprows,spcols,kk);
      plot(RobotOptRes.fval_pareto(:,kk1), RobotOptRes.fval_pareto(:,kk2), 'm*');
      xlabel(sprintf('Zielf. %d (%s) (norm.)', kk1, Set.optimization.objective{kk1}));
      ylabel(sprintf('Zielf. %d (%s) (norm.)', kk2, Set.optimization.objective{kk2}));
      grid on;
    end
    sgtitle('Pareto-Fronten für mehrkrit. Opt.');
    saveas(10*i+7,     fullfile(resrobdir, sprintf('Rob%d_%s_Pareto2D.fig', i, Name)));
    export_fig(10*i+7, fullfile(resrobdir, sprintf('Rob%d_%s_Pareto2D.png', i, Name)));
  end
  %% (3D)-Pareto-Fronten für die Zielkriterien
  if length(Set.optimization.objective) > 2 % Mehrkriterielle Optimierung
    objcomb = allcomb(1:length(Set.optimization.objective), 1:length(Set.optimization.objective), ...
      1:length(Set.optimization.objective));
    objcomb(objcomb(:,1)==objcomb(:,2) | objcomb(:,2)==objcomb(:,3),:) = [];
    objcomb(objcomb(:,1)>objcomb(:,2),:) = [];
    objcomb(objcomb(:,2)>objcomb(:,3),:) = [];
    figure(10*i+8);clf;hold all;
    sprows = floor(sqrt(size(objcomb,1)));
    spcols = ceil(size(objcomb,1)/sprows);
    for kk = 1:size(objcomb,1)
      kk1 = objcomb(kk,1); kk2 = objcomb(kk,2); kk3 = objcomb(kk,3);
      subplot(sprows,spcols,kk);
      plot3(RobotOptRes.fval_pareto(:,kk1), RobotOptRes.fval_pareto(:,kk2), RobotOptRes.fval_pareto(:,kk3), 'm*');
      xlabel(sprintf('Zielf. %d (%s) (norm.)', kk1, Set.optimization.objective{kk1}));
      ylabel(sprintf('Zielf. %d (%s) (norm.)', kk2, Set.optimization.objective{kk2}));
      zlabel(sprintf('Zielf. %d (%s) (norm.)', kk3, Set.optimization.objective{kk3}));
      grid on;
    end
    sgtitle('Pareto-Fronten (3D) für mehrkrit. Opt.');
    saveas(10*i+8,     fullfile(resrobdir, sprintf('Rob%d_%s_Pareto3D.fig', i, Name)));
    export_fig(10*i+8, fullfile(resrobdir, sprintf('Rob%d_%s_Pareto3D.png', i, Name)));
  end

  %% Finalisierung
  if length_Structures > 3
    close all; % schließe alle Bilder wieder. Sonst sind Hunderte Bilder am Ende offen
  end
  % Alle Auswertungsbilder wieder schließen. Sonst gibt es eventuell
  % Probleme mit dem Arbeitsspeicher.
  if Set.general.only_save_summary_figures
    close all;
  end

  fprintf('Visualisierung für Rob %d (%s) beendet. Dauer: %1.1fs\n', i, Name, toc(t_start_i));
end
