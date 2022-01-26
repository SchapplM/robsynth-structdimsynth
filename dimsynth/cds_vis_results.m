% Visualisierung der Ergebnisse der Maßsynthese für alle Roboter
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
% Erzeugt Bilder für jeden Roboter. In Klamern: Schalter in eval_figures
% zum Auswählen des Bildes.
% * Statistik der Ergebnisse ('histogram')
% * Animation (gesteuert durch Set.general.animation_styles) ('robvisuanim')
% * Trägheitsellipsen ('dynparvisu')
% * Gelenkverläufe der Kinematik ('jointtraj')
% * Antriebskraftverläufe aus Komponenten der Dynamik ('dynamics')
% * Diverse Auswertungen zur Fitness-Funktion ('fitness_various')
%   (Daten, die in cds_save_particle_details.m gespeichert werden:
%   Zeitauswertung, Kondition, Materialbelastung)
% * 2D-Pareto-Front (nur bei mehrkriterieller Optimierung) ('pareto')
% * 3D-Pareto-Front (nur falls 3 oder mehr Kriterien) ('pareto')
% * Dynamik-Komponenten in Plattform-KS
% Bilder für alle Roboter:
% * Pareto-Front mit physikalischen Werten und normierten Werten der Zielf.
%   ('pareto_all_phys', 'pareto_all_fval')
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
vis_settings = struct(...
  'figure_invisible', false, ...
  'delete_figure', false);
if Set.general.isoncluster
  % Auf Cluster. Lösche die Bilder sofort, damit weniger Speicher benötigt
  % wird.
  vis_settings.delete_figure = true;
else
  % Auf lokalem Rechner. Erzeuge die Bilder immer zuerst unsichtbar, damit
  % der Fokus-Klau den Rechner nicht lahmlegt
  vis_settings.figure_invisible = true;
end
% Zum Debuggen
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_vis_results1.mat'));

%% Initialisierung
resmaindir = fullfile(Set.optimization.resdir, Set.optimization.optname);
% Ergebnistabelle laden
restabfile = fullfile(resmaindir, sprintf('%s_results_table.csv', Set.optimization.optname));
ResTab = readtable(restabfile, 'Delimiter', ';');

% Einheiten für die physikalischen Werte der Zielfunktionen vorbereiten
[obj_units, objscale] = cds_objective_plotdetails(Set);

length_Structures = length(Structures);
% Prüfe, ob überhaupt roboterspezifische Plots erzeugt werden sollen
length_Structures_parfor = length_Structures;
if isempty(Set.general.animation_styles) && isempty(setdiff( ...
    Set.general.eval_figures, {'pareto_all_fval','pareto_all_phys'}))
  length_Structures_parfor = 0;
end
%% Parallele Durchführung der Plots vorbereiten
if Set.general.parcomp_plot && length_Structures_parfor > 0 && ...
    Set.general.parcomp_maxworkers > 0 % Parallele Berechnung auch so deaktivierbar
  Pool = gcp('nocreate');
  if isempty(Pool) % kein Pool aktiv
    if Set.general.isoncluster % auf Cluster möglicher Zugriffskonflikt für ParPool
      parpool_writelock('lock', 180, true); % Synchronisationsmittel für ParPool
    end
    try % Pool starten
      Pool=parpool([1,Set.general.parcomp_maxworkers]);
      parfor_numworkers = Pool.NumWorkers;
    catch err
      fprintf('Fehler beim Starten des parpool: %s\n', err.message);
      parfor_numworkers = 1;
    end
    if Set.general.isoncluster
      parpool_writelock('free', 0, true);
    end
  else
    parfor_numworkers = Pool.NumWorkers;
  end
  clear Pool
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
if parfor_numworkers > 0
  close all % Speicher freigeben
end
parfor (i = 1:length_Structures_parfor, parfor_numworkers)
  % Auflösung für Debug-Bilder setzen (wird auf ParPool auf Cluster nicht
  % vererbt aus globalen Einstellungen)
  pause(2*(i-1)); % Damit nicht alle parfor-Worker gleichzeitig starten (unklar ob notwendig)
  if parfor_numworkers > 0
    close all % Speicher freigeben (evtl. Bilder auf Worker offen)
    set(0, 'defaultfigureposition', [1 1 1920 1080]);
    set(0, 'defaultfigureunits', 'pixels');
  end
  t_start_i = tic();
  %% Initialisierung der Ergebnisse dieser Struktur
  % load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_vis_results2.mat'));
  Structure = Structures{i};
  Name = Structures{i}.Name;
  fprintf('%d/%d: Visualisiere Ergebnisse für (%s)\n', i, length_Structures, Name);
  resfile1 = fullfile(resmaindir, sprintf('Rob%d_%s_Endergebnis.mat', i, Name));
  resfile2 = fullfile(resmaindir, sprintf('Rob%d_%s_Details.mat', i, Name));
  if ~exist(resfile1, 'file') || ~exist(resfile2, 'file')
    warning('Ergebnis-Datei für Roboter %d/%d (%s) existiert nicht: %s oder %s', ...
      i, length_Structures, Name, resfile1, resfile2);
    continue
  end
  tmp1 = load(resfile1, 'RobotOptRes');
  tmp2 = load(resfile2, 'RobotOptDetails', 'PSO_Detail_Data');
  fprintf('%d/%d: Dateien für %s geladen.\n', i, length_Structures, Name);
  resrobdir = fullfile(resmaindir, sprintf('Rob%d_%s', i, Name));
  mkdirs(resrobdir); % Speicherort für Bilder dieses Roboters
  RobotOptRes = tmp1.RobotOptRes;
  RobotOptDetails = tmp2.RobotOptDetails;
  PSO_Detail_Data = tmp2.PSO_Detail_Data;
  R = RobotOptDetails.R;
  if Structure.Type == 0
    serroblib_addtopath({Name});
  else
    parroblib_addtopath({Name});
  end
  RobData = struct('Name', Name, 'Number', i, 'ParetoNumber', 1, ...
    'Type', RobotOptRes.Structure.Type);
  %% Statistische Verteilung der Ergebnisse aller Generationen
  if any(strcmp(Set.general.eval_figures, 'histogram')) %#ok<PFBNS>
  t1 = tic();
  Erg_All_Gen = PSO_Detail_Data.fval_mean;
  I_zul = Erg_All_Gen(:) < 1e3;
  Klassengrenzen_Alle = [0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9, 1e10, 1e11, 1e12, 1e13];
  Klassengrenzen_Alle_Log = log10(Klassengrenzen_Alle);
  Klassengrenzen_Alle_Log(1) = 0;
  % Histogramm erstellen
  f = change_current_figure(100*i+1, vis_settings.figure_invisible);
  clf; hold all;
  set(f, 'Name', sprintf('Rob%d_Hist', i), 'NumberTitle', 'off', 'color','w');
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
      % Zähle die Partikel, die innerhalb der Grenze liegen oder auf der
      % oberen Grenze liegen (falls eine NB "unendlich" (inf) schlecht ist).
      Erg_Zul_Gen_Hist(ii,jj-1) = sum((Erg_All_Gen2(ii,:) > edges(jj-1)) & (Erg_All_Gen2(ii,:) <= edges(jj)));
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
    if ii == size(Erg_Zul_Gen_Hist,2) && length(legbarhdl)>3 % letzte Iteration. Trage Legende ein
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

  saveas(f,     fullfile(resrobdir, sprintf('Rob%d_%s_Histogramm.fig', i, Name)));
  export_fig(f, fullfile(resrobdir, sprintf('Rob%d_%s_Histogramm.png', i, Name)));
  fprintf('%d/%d: Histogramm für %s gespeichert. Dauer: %1.1fs\n', ...
    i, length_Structures, Name, toc(t1));
  end
  %% Verschiedene Auswertungen
  if any(strcmp(Set.general.eval_figures, 'fitness_various'))
  t1 = tic();
  f = change_current_figure(100*i+6, vis_settings.figure_invisible); clf;
  sgtitle('Diverse Auswertungsbilder');
  % Verteilung der Rechenzeit über die Zielfunktionswerte
  % Streudiagramm
  subplot(2,2,1); hold all;
  plot(log10(PSO_Detail_Data.fval_mean), PSO_Detail_Data.comptime, 'kx');
  xlabel('Zielfunktion (log)');
  ylabel('Rechenzeit in s');
  title('Rechenzeit je nach Zielfunktionswert');
  grid on;
  
  % Verteilung der Konditionszahlen in den Ergebnissen
  subplot(2,2,2); hold all;
  Jcond_all = PSO_Detail_Data.constraint_obj_val(:,4,:);
  h = histogram(log10(Jcond_all(:)));
  plot(log10(Set.optimization.constraint_obj(4))*[1;1], [0;1.2*max(h.Values)], 'k-', 'LineWidth', 2);
  xlim(minmax2(h.BinEdges)+h.BinWidth*[-1 1])
  title('Verteilung der Konditionszahlen über die Optimierung');
  xlabel('Konditionszahl (log)');
  ylabel(sprintf('Häufigkeit (Anzahl i.O.: %d/%d)', sum(~isnan(Jcond_all(:))), ...
    length(Jcond_all(:))));
    
  % Verteilung der Materialbeanspruchung gegen die Jacobi-Konditionszahl
  % Streudiagramm
  subplot(2,2,3); hold all;
  f_maxstrengthviol_all = PSO_Detail_Data.constraint_obj_val(:, 6, :);
  plot(log10(Jcond_all(:)), 100*f_maxstrengthviol_all(:), 'kx');
  plot([0;log10(max(Jcond_all(:)))], [100;100], 'g--'); % Grenze Material
  plot(log10(Set.optimization.constraint_obj(4))*[1;1], [0;100], 'r--'); % Grenze für Jacobi
  if sum(~isnan(unique(Jcond_all))) > 1
    xlim(log10(minmax2(Jcond_all(:)'))); % geht nur, wenn zwei Werte da sind
  end
  if sum(~isnan(unique(f_maxstrengthviol_all(:)))) > 1
    ylim(100*minmax2(f_maxstrengthviol_all(:)'));
  end
  xlabel('Konditionszahl (log)');
  ylabel('Materialbeanspruchung in Prozent');
  title('Materialbelastung vs Kondition');
  grid on;  
  
  saveas(f,     fullfile(resrobdir, sprintf('Rob%d_%s_Population_Fitness.fig', i, Name)));
  export_fig(f, fullfile(resrobdir, sprintf('Rob%d_%s_Population_Fitness.png', i, Name)));
  fprintf('%d/%d: Weitere Auswertungsbilder für %s gespeichert. Dauer: %1.1fs\n', ...
    i, length_Structures, Name, toc(t1));
  end
  %% Animation des besten Roboters für die Trajektorie
  if ~isempty(Set.general.animation_styles)
    t1 = tic();
    fprintf('%d/%d: Beginne Animation für %s\n', i, length_Structures, Name);
    cds_vis_results_figures('animation', Set, Traj, RobData, ...
      ResTab, RobotOptRes, RobotOptDetails, [], vis_settings);
    fprintf('%d/%d: Animation für %s gespeichert nach %s. Dauer: %1.1fs\n', ...
      i, length_Structures, Name, resrobdir, toc(t1));
  end
  t1 = tic();
  %% Zeichnung der Roboters mit Trägheitsellipsen und Ersatzdarstellung
  if any(strcmp(Set.general.eval_figures, 'dynparvisu'))
    cds_vis_results_figures('dynparvisu', Set, Traj, RobData, ...
      ResTab, RobotOptRes, RobotOptDetails, [], vis_settings);
  end
  %% Verlauf der Gelenkgrößen für den besten Roboter
  if any(strcmp(Set.general.eval_figures, 'jointtraj'))
    cds_vis_results_figures('jointtraj', Set, Traj, RobData, ...
      ResTab, RobotOptRes, RobotOptDetails, [], vis_settings);
  end
  %% Dynamik
  if any(strcmp(Set.general.eval_figures, 'dynamics'))
    cds_vis_results_figures('dynamics', Set, Traj, RobData, ...
      ResTab, RobotOptRes, RobotOptDetails, [], vis_settings);
  end
  %% Pareto-Fronten für die Zielkriterien
  if any(strcmp(Set.general.eval_figures, 'pareto')) && ...
     length(Set.optimization.objective) > 1 % Mehrkriterielle Optimierung
    cds_vis_results_figures('pareto', Set, Traj, RobData, ...
      ResTab, RobotOptRes, RobotOptDetails, PSO_Detail_Data, vis_settings);
  end
  fprintf('%d/%d: Restliche Bilder für %s gespeichert. Dauer: %1.1fs\n', ...
    i, length_Structures, Name, toc(t1));
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
%% Erzeuge Pareto-Diagramme für alle Roboter (2D)
if any(length(Set.optimization.objective) == [2 3]) % Für mehr als drei Kriterien gleichzeitig nicht sinnvoll
  for pffig = 1:2 % Zwei Bilder: Physikalische Werte und normierte Werte
  if pffig == 1 && ~any(strcmp(Set.general.eval_figures, 'pareto_all_phys')) || ...
     pffig == 2 && ~any(strcmp(Set.general.eval_figures, 'pareto_all_fval'))
    continue
  end
  if pffig == 1, name_suffix = 'phys';
  else,          name_suffix = 'fval'; end
  
  f = change_current_figure(10+pffig, vis_settings.figure_invisible);
  clf; hold on; grid on;
  set(f, 'name', sprintf('Pareto_Gesamt_%s',name_suffix), ...
    'NumberTitle', 'off', 'color','w');
  leghdl = []; legstr = {}; % Für Erstellung der Legende am Ende
  countmarker = 0; % Stelle Marker für jeden Roboter zusammen
  markerlist = {'x', 's', 'v', '^', '*', 'o', 'd', 'v', '<', '>', 'p', 'h'};
  colorlist =  {'r', 'g', 'b', 'c', 'm', 'k'};
  for i = 1:length_Structures
    % Lade Ergebnisse Für Roboter i
    Name = Structures{i}.Name;
    resfile1 = fullfile(resmaindir, sprintf('Rob%d_%s_Endergebnis.mat', i, Name));
    if ~exist(resfile1, 'file')
      warning('Ergebnis-Datei für Roboter %d/%d (%s) existiert nicht: %s', ...
        i, length_Structures, Name, resfile1);
      continue
    end
    tmp1 = load(resfile1, 'RobotOptRes');
    RobotOptRes = tmp1.RobotOptRes;
    if any(RobotOptRes.fval > 1e3)
      continue
    end
    % Für Legende: Nur erfolgreiche PKM zählen
    countmarker = countmarker + 1; % Hochzählen für die Marker und Farben
    if countmarker > length(markerlist)*length(colorlist)
      warning('Zu viele verschiedene Roboter. Eindeutiges Plotten nicht mehr möglich.');
      marker = 'k.'; % ab jetzt sehen alle Marker gleich aus
    else
      ic = mod((countmarker-1),6)+1; % Index für Farben und Marker generieren
      im = ceil(countmarker/6);
      marker = [markerlist{im}, colorlist{ic}];
    end
    % Pareto-Front für diesen Roboter einzeichnen
    if length(Set.optimization.objective) == 2
      if pffig == 1 % Bild mit physikalischen Werten
        hdl=plot(objscale(1)*tmp1.RobotOptRes.physval_pareto(:,1), ...
                 objscale(2)*tmp1.RobotOptRes.physval_pareto(:,2), marker);
      else % Bild mit normierten Zielfunktionswerten
        hdl=plot(tmp1.RobotOptRes.fval_pareto(:,1), ...
                 tmp1.RobotOptRes.fval_pareto(:,2), marker);
      end
    else % length(Set.optimization.objective) == 3
      if pffig == 1 % Bild mit physikalischen Werten
        hdl=plot3(objscale(1)*tmp1.RobotOptRes.physval_pareto(:,1), ...
                  objscale(2)*tmp1.RobotOptRes.physval_pareto(:,2), ...
                  objscale(3)*tmp1.RobotOptRes.physval_pareto(:,3), marker);
      else % Bild mit normierten Zielfunktionswerten
        hdl=plot3(tmp1.RobotOptRes.fval_pareto(:,1), ...
                  tmp1.RobotOptRes.fval_pareto(:,2), ...
                  tmp1.RobotOptRes.fval_pareto(:,3), marker);
      end
    end
    leghdl(countmarker,:) = hdl; %#ok<AGROW>
    if ~isa(ResTab.Beschreibung, 'cell')
      RobShortName = ''; % Wenn kein Wert belegt ist, wird NaN gesetzt
    else
      RobShortName = ResTab.Beschreibung{strcmp(ResTab.Name,Name) & ResTab.LfdNr==i};
    end
    if ~isempty(RobShortName)
      addtxt = sprintf('; %s', RobShortName);
    else
      addtxt = '';
    end
    legstr{countmarker} = sprintf('%d/%d (%s%s)', i, length_Structures, ...
      Structures{i}.Name, addtxt); %#ok<AGROW>
    % Funktions-Handle zum Anklicken der Datenpunkte
    ButtonDownFcn=@(src,event)cds_paretoplot_buttondownfcn(src,event,...
      Set.optimization.optname,Structures{i}.Name, i);
    set(hdl, 'ButtonDownFcn', ButtonDownFcn)
  end
  if pffig == 1
    xlabel(sprintf('%s in %s', Set.optimization.objective{1}, obj_units{1}));
    ylabel(sprintf('%s in %s', Set.optimization.objective{2}, obj_units{2}));
  else
    xlabel(sprintf('%s (normiert)', Set.optimization.objective{1}));
    ylabel(sprintf('%s (normiert)', Set.optimization.objective{2}));
  end
  if length(Set.optimization.objective) == 2
    if pffig == 1
      title(sprintf('%s: %s vs %s (physikalisch)', Set.optimization.optname, ...
        Set.optimization.objective{1}, Set.optimization.objective{2}), 'interpreter', 'none');
    else
      title(sprintf('%s: %s vs %s (normiert)', Set.optimization.optname, ...
        Set.optimization.objective{1}, Set.optimization.objective{2}), 'interpreter', 'none');
    end
  else
    if pffig == 1
      zlabel(sprintf('%s in %s', Set.optimization.objective{3}, obj_units{3}));
    else
      zlabel(sprintf('%s (normiert)', Set.optimization.objective{3}));
    end
    if pffig == 1
      title(sprintf('%s: %s vs %s vs %s (physikalisch)', ...
        Set.optimization.optname, Set.optimization.objective{1}, ...
        Set.optimization.objective{2}, Set.optimization.objective{3}), 'interpreter', 'none');
    else
      title(sprintf('%s: %s vs %s vs %s (normiert)', ...
        Set.optimization.optname, Set.optimization.objective{1}, ...
        Set.optimization.objective{2}, Set.optimization.objective{3}), 'interpreter', 'none');
    end
  end
  axhdl = get(f, 'children');
  legend(leghdl, legstr);
  % Prüfe ob ein Wertebereich sehr stark unausgeglichen ist (z.B. wenn die
  % Konditionszahl als Kriterium bis unendlich geht). Dann logarithmisch.
  if length(axhdl) == 1
    linhdl = get(axhdl, 'children');
    for kk = 1:3 % x,y,z
      dataminmax = [NaN, NaN];
      for jj = 1:length(linhdl) % xdata, ydata, zdata
        if ~strcmp(get(linhdl(jj), 'type'), 'line'); continue; end
        dataminmax = minmax2([dataminmax, get(linhdl(jj), [char(119+kk), 'data'])]);
      end
      if dataminmax(2) / dataminmax(1) > 1e3 && all(dataminmax>0)
        set(axhdl, [char(119+kk), 'scale'], 'log');
      end
    end
  end
  % PNG-Export bereits hier, da Probleme mit uicontrol.
  export_fig(f, fullfile(resmaindir, sprintf('Pareto_Gesamt_%s.png',name_suffix)));
  % Auswahlmenü für eine nachträglich zu plottende Auswertung. Wird in der
  % ButtonDownFcn (cds_paretoplot_buttondownfcn) bei Klicken auf einen
  % Pareto-Punkt ausgelesen.
  menuitems = {'Visualisierung', 'Parameter', 'Kinematik', 'Pareto', ...
    'Animation', 'Dynamik', 'Dynamikparameter'};
  % Füge Möglichkeiten hinzu, die nur bei Aufgabenredundanz bestehen
  % Prüfe dafür, ob bei einem Roboter Aufgabenredundanz vorliegt
  task_red = false;
  for i = 1:length(Structures)
    S = Structures{i};
    if S.Type == 0
      NJ = str2double(S.Name(2));
      task_red = sum(Set.task.DoF) < NJ; % Seriell: Redundant wenn mehr Gelenke als Aufgaben-FG
    else
      task_red = sum(Set.task.DoF) < sum(S.DoF); % Parallel: Redundant wenn mehr Plattform-FG als Aufgaben-FG
    end
    if task_red, break; end
  end
  if task_red
    menuitems = [menuitems, 'Redundanzkarte']; %#ok<AGROW>
  end
  if Set.optimization.joint_stiffness_passive_revolute
    menuitems = [menuitems, 'Feder-Ruhelage']; %#ok<AGROW>
  end
  uicontrol('Style', 'popupmenu', ...
            'String', menuitems, ...
            'Units', 'pixels', ...
            'Position', [10, 30, 120, 24]);
  saveas(f, fullfile(resmaindir, sprintf('Pareto_Gesamt_%s.fig',name_suffix)));
  end
end
