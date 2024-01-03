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
% * Pareto-Diagramm der Entwurfsoptimierung ('pareto')
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
  % Auf Cluster. Schließe die Bilder sofort, damit weniger Speicher benötigt wird.
  vis_settings.delete_figure = true;
else
  % Auf lokalem Rechner. Erzeuge die Bilder immer zuerst unsichtbar, damit
  % der Fokus-Klau den Rechner nicht lahmlegt
  vis_settings.figure_invisible = true;
end
% Zum Debuggen
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_vis_results1.mat'));

%% Debug: Structures-Variable aktualisieren (geht nur für PKM)
% Benutze diesen Code-Abschnitt provisorisch, um die am 02.08.2022
% eingeführte Schnittstelle auch für ältere Ergebnisse zu nutzen.
% TODO: Diesen Abschnitt entfernen, falls nicht mehr notwendig. (ab ca. 2024)
for k = 1:length(Structures)
  if isfield(Structures{k}, 'act_type'), continue; end
  if Structures{k}.Type == 0
    Structures{k}.act_type = 'mixed';
    break; % für serielle Roboter nicht erfassen
  end
  [~,Leg_Names, Actuation] = parroblib_load_robot(Structures{k}.Name, 2);
  for j = 1:length(Leg_Names)
    NlegJ = str2double(Leg_Names{j}(2));
    Chain = Leg_Names{j}(3:3+NlegJ-1);
    if any(strcmp(Chain(Actuation{j}), 'P'))
      Structures{k}.act_type = 'prismatic';
    else
      Structures{k}.act_type = 'revolute';
    end
  end
end

%% Initialisierung
resmaindir = fullfile(Set.optimization.resdir, Set.optimization.optname);
% Ergebnistabelle laden
restabfile = fullfile(resmaindir, sprintf('%s_results_table.csv', Set.optimization.optname));
opts = detectImportOptions(restabfile,'NumHeaderLines',2);
opts.VariableNamesLine = 1;
opts.VariableDescriptionsLine = 2;
ResTab = readtable(restabfile, opts);
if isempty(ResTab) % Scheinbar keine Ergebnisse
  return
end

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
  parfor_numworkers = cds_start_parpool(Set);
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
  Number = Structures{i}.Number;
  fprintf('%d/%d: Visualisiere Ergebnisse für (%d/%s): {%s}\n', i, ...
    length_Structures, Number, Name, disp_array(Set.general.eval_figures, '%s')); %#ok<PFBNS> 
  resfile1 = fullfile(resmaindir, sprintf('Rob%d_%s_Endergebnis.mat', Number, Name));
  resfile2 = fullfile(resmaindir, sprintf('Rob%d_%s_Details.mat', Number, Name));
  if ~exist(resfile1, 'file') || ~exist(resfile2, 'file')
    warning('Ergebnis-Datei für Roboter %d/%d (%s) existiert nicht: %s oder %s', ...
      i, length_Structures, Name, resfile1, resfile2);
    continue
  end
  tmp1 = load(resfile1, 'RobotOptRes');
  tmp2 = load(resfile2, 'RobotOptDetails', 'PSO_Detail_Data');
  fprintf('%d/%d: Dateien für Rob %d (%s) geladen.\n', i, length_Structures, Number, Name);
  resrobdir = fullfile(resmaindir, sprintf('Rob%d_%s', Number, Name));
  mkdirs(resrobdir); % Speicherort für Bilder dieses Roboters
  RobotOptRes = tmp1.RobotOptRes;
  RobotOptDetails = tmp2.RobotOptDetails;
  PSO_Detail_Data = tmp2.PSO_Detail_Data;
  if Structure.Type == 0
    serroblib_addtopath({Name});
  else
    parroblib_addtopath({Name});
  end
  RobData = struct('Name', Name, 'Number', Number, 'ParetoNumber', 1, ...
    'Type', RobotOptRes.Structure.Type);
  %% Statistische Verteilung der Ergebnisse aller Generationen
  if any(strcmp(Set.general.eval_figures, 'histogram'))
  t1 = tic();
  Erg_All_Gen = PSO_Detail_Data.fval_mean;
  I_zul = Erg_All_Gen(:) < 1e3;
  Klassengrenzen_Alle = [0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9, 1e10, 1e11, 1e12, 1e13];
  Klassengrenzen_Alle_Log = log10(Klassengrenzen_Alle);
  Klassengrenzen_Alle_Log(1) = 0;
  % Histogramm erstellen
  f = change_current_figure(100*i+1, vis_settings.figure_invisible); %#ok<PFBNS> 
  clf; hold all;
  set(f, 'Name', sprintf('Rob%d_Hist', Number), 'NumberTitle', 'off', 'color','w');
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

  saveas(f,     fullfile(resrobdir, sprintf('Rob%d_%s_Histogramm.fig', Number, Name)));
  export_fig(f, fullfile(resrobdir, sprintf('Rob%d_%s_Histogramm.png', Number, Name)));
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
  
  saveas(f,     fullfile(resrobdir, sprintf('Rob%d_%s_Population_Fitness.fig', Number, Name)));
  export_fig(f, fullfile(resrobdir, sprintf('Rob%d_%s_Population_Fitness.png', Number, Name)));
  fprintf('%d/%d: Weitere Auswertungsbilder für Rob %d (%s) gespeichert. Dauer: %1.1fs\n', ...
    i, length_Structures, Number, Name, toc(t1));
  end
  %% Animation des besten Roboters für die Trajektorie
  if ~isempty(Set.general.animation_styles)
    t1 = tic();
    fprintf('%d/%d: Beginne Animation für Rob %d (%s)\n', i, length_Structures, Number, Name);
    cds_vis_results_figures('animation', Set, Traj, RobData, ...
      ResTab, RobotOptRes, RobotOptDetails, [], vis_settings);
    fprintf('%d/%d: Animation für Rob %d (%s) gespeichert nach %s. Dauer: %1.1fs\n', ...
      i, length_Structures, Number, Name, resrobdir, toc(t1));
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
  if any(strcmp(Set.general.eval_figures, 'pareto_desopt')) && ...
     length(Set.optimization.objective) > 1
    cds_vis_results_figures('pareto_desopt', Set, Traj, RobData, ...
      ResTab, RobotOptRes, RobotOptDetails, PSO_Detail_Data, vis_settings);
  end
  if any(strcmp(Set.general.eval_figures, 'pareto_dimsynth_desopt')) && ...
     length(Set.optimization.objective) > 1
    cds_vis_results_figures('pareto_dimsynth_desopt', Set, Traj, RobData, ...
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

  fprintf('Visualisierung für Rob %d (%s) beendet. Dauer: %1.1fs\n', Number, Name, toc(t_start_i));
end
%% Erzeuge Pareto-Diagramme für alle Roboter (2D oder 3D)
if length(Set.optimization.objective) > 1 % Mehrkriterielle Optimierung
  % Für mehr als drei Kriterien gleichzeitig nicht sinnvoll
  if length(Set.optimization.objective) > 3
    % Gehe alle Kombinationen von drei Zielkriterien durch
    objcomb3D = allcomb(1:length(Set.optimization.objective), 1:length(Set.optimization.objective), 1:length(Set.optimization.objective));
    % Entferne Duplikate (Schema: Aufsteigende ZF-Nummer mit xyz-Achse)
    objcomb3D(objcomb3D(:,1)==objcomb3D(:,2),:) = [];
    objcomb3D(objcomb3D(:,1)==objcomb3D(:,3),:) = [];
    objcomb3D(objcomb3D(:,2)==objcomb3D(:,3),:) = [];
    objcomb3D(objcomb3D(:,1)>objcomb3D(:,2),:) = [];
    objcomb3D(objcomb3D(:,1)>objcomb3D(:,3),:) = [];
    objcomb3D(objcomb3D(:,2)>objcomb3D(:,3),:) = [];
  else
    objcomb3D = 1:length(Set.optimization.objective);
  end
  for pfcomb = 1:size(objcomb3D,1)
  for pffig = 1:2 % Zwei Bilder: Physikalische Werte und normierte Werte
  if pffig == 1 && ~any(strcmp(Set.general.eval_figures, 'pareto_all_phys')) || ...
     pffig == 2 && ~any(strcmp(Set.general.eval_figures, 'pareto_all_fval'))
    continue
  end
  if pffig == 1, name_suffix_phys = 'phys';
  else,          name_suffix_phys = 'fval'; end
  name_suffix_obj = '';
  for kk = 1:size(objcomb3D,2)
    if kk > 1
      name_suffix_obj = [name_suffix_obj, '_vs_']; %#ok<AGROW> 
    end
    name_suffix_obj = [name_suffix_obj, Set.optimization.objective{objcomb3D(pfcomb, kk)}]; %#ok<AGROW> 
  end
  % Prüfe die Art der Aktuierung der Roboter. Wenn gemischt Dreh- und
  % Schubantriebe vorkommen, erzeuge zwei zusätzliche Bilder. Sonst
  % ist der Vergleich in einem Diagramm nicht zielführend
  I_acttype = false(length(Structures), 4); % Jede Spalte entspricht einem Diagramm
  % Einmal Pareto-Diagramm für alle Roboter zeichnen, unabhängig von Typ
  I_acttype(:,4) = 1;
  if ~isempty(intersect(Set.optimization.objective, {'actforce', 'actvelo'}))
    for k = 1:length(Structures)
      I_acttype(k,1) = strcmp(Structures{k}.act_type, 'revolute');
      I_acttype(k,2) = strcmp(Structures{k}.act_type, 'prismatic');
      I_acttype(k,3) = ~(I_acttype(k,1)|I_acttype(k,2)); % Mischung bereits in den Antrieben
    end
  end
  if size(I_acttype,1) == 1 % Sonderfall einzelner Roboter
    I_acttype = [I_acttype; false(1,4)]; %#ok<AGROW> % Dummy-Zeile für any-Befehl unten
    I_acttype(:,1:3) = false; % nur die letzte Spalte aktivieren für Standard-Bild
  end
  % Falls es nur einen Typ von Antrieben gibt (nur Dreh- oder Schubantriebe)
  % dann nur ein Diagramm und nicht doppelt
  if all(I_acttype(:,1)) || all(I_acttype(:,2))
    I_acttype(:,1:2) = 0; % Spezifisches Diagramm Dreh-/Schub weglassen
    I_acttype(:,4) = true; % allgemeines Diagramm ist bereits richtig
  end
  % n.i.O.-Strukturen direkt hier aussortieren
  for k = 1:length(Structures)
    II = ResTab.LfdNr==Structures{k}.Number;
    if ~any(II) || ResTab.Fval_Opt(II) > 1e3
      I_acttype(k,:) = false;
    end
  end
  % Schleife über Zusammenfassung der Gestell-/Plattform-Varianten
  for pfvar = 1:2 % 1=alle Roboter, 2=Varianten mit gleichem Marker zusammengefasst
  % Schleife über verschiedene Aktuierungstypen (Dreh/Schub/Mischung)
  for pfact = find(any(I_acttype))
  t1 = tic();
  if     pfact == 1, name_suffix = [name_suffix_phys, '_revact'];
  elseif pfact == 2, name_suffix = [name_suffix_phys, '_prisact'];
  elseif pfact == 3, name_suffix = [name_suffix_phys, '_mixact'];
  else,              name_suffix =  name_suffix_phys;
  end
  if     pfvar == 2, name_suffix = [name_suffix, '_groups']; %#ok<AGROW>
  end
  name_suffix = [name_suffix, '_', name_suffix_obj]; %#ok<AGROW> 
  if ~any(I_acttype(:,pfact)), continue; end
  if sum(I_acttype(:,pfact)) == 1 && pfvar == 2 % Sonderfall einzelner Roboter
    continue; % Nichts zu gruppieren
  end
  if sum(I_acttype(:,pfact)) > 80 && pfvar == 1 % Zu viele Roboter. Kein Einzel-Bild (sowieso zu unübersichtlich)
    continue;
  end
  % Achsbeschriftungen für Diagramm für diese Roboterauswahl aktualisieren
  [obj_units, objscale, objtext] = cds_objective_plotdetails(Set, Structures(I_acttype(:,pfact)'));
  % Indizes der betrachteten Zielkriterien
  IO1 = objcomb3D(pfcomb, 1);
  IO2 = objcomb3D(pfcomb, 2);
  IO3 = objcomb3D(pfcomb, 3);

  % Daten für Pareto-Front sammeln
  pf_data = NaN(0, length(Set.optimization.objective));
  pf_robnum = []; % zugehörige Roboter-Nummer für jedes Partikel
  RobName_base = cell(1,size(I_acttype,1)); % Name des Roboters zum Zusammenfassen zu Gruppen
  % Suche alle PKM mit der aktuell gesuchten Aktuierung
  II_acttype_act = find(I_acttype(:,pfact)');
  for i = II_acttype_act % Auswahl der Roboter durchgehen
    % Lade Ergebnisse Für Roboter i
    Name = Structures{i}.Name;
    Number = Structures{i}.Number;
    resfile1 = fullfile(resmaindir, sprintf('Rob%d_%s_Endergebnis.mat', Number, Name));
    if ~exist(resfile1, 'file')
      warning('Ergebnis-Datei für Roboter %d/%d (%d/%s) existiert nicht: %s', ...
        i, length_Structures, Number, Name, resfile1);
      continue
    end
    tmp1 = load(resfile1, 'RobotOptRes');
    RobotOptRes = tmp1.RobotOptRes;
    if any(RobotOptRes.fval > 1e3)
      continue
    end
    % Pareto-Front anhängen
    if pffig == 1 % Bild mit physikalischen Werten (bereits mit Plot-Skalierung)
      pf_data_add = repmat(objscale(:)', size(...
        tmp1.RobotOptRes.physval_pareto,1),1).*tmp1.RobotOptRes.physval_pareto;
    else % Bild mit normierten Werten
      pf_data_add = tmp1.RobotOptRes.fval_pareto;
    end
    if all(isnan(pf_data_add(:)))
      warning('Daten für Pareto-Diagramm sind NaN für Roboter %d/%d (%d/%s).', ...
        i, length_Structures, Number, Name);
      continue;
    end
    pf_data = [pf_data; pf_data_add]; %#ok<AGROW>
    pf_robnum = [pf_robnum; i*ones(size(tmp1.RobotOptRes.physval_pareto,1),1)]; %#ok<AGROW>
    % Erst hier den Namen des Roboters einspeichern (nach allen continues)
    % Sonst könnte ein ungültiger Roboter in die Liste kommen und die
    % Nummerierung in der Legende durcheinanderbringen. Das verwirrt dann.
    if Structures{i}.Type == 0 % Seriell: Keine Unterscheidung
      RobName_base{i} = Structures{i}.Name;
    else % PKM-Name ohne G-P-Nummer
      [~,~,Actuation,~,~,~,~,~,PName_Legs] = parroblib_load_robot(Structures{i}.Name, 2);
      % TODO: Der Name mit Aktuierung sollte direkt aus der Datenbank
      % kommen und es sollte die Unterstrich-Notation in die Legende.
      RobName_base{i} = sprintf('%s,Act=%d', PName_Legs, Actuation{1});
    end
  end % for i = II_acttype_act
  robgroups = zeros(length(RobName_base), 1); % Zuordnung der Roboter-Nummern zu Gruppen-Nummern
  if pfvar == 2 % Bild mit Gruppierung der Varianten
    pf_groupnum = zeros(length(pf_robnum),1); % Gruppenzuordnung jedes Paretofront-Datenpunkts
    % Pareto-Front nachverarbeiten, falls Roboter zusammengefasst werden.
    for i = II_acttype_act
      if isempty(RobName_base{i}), continue; end % nicht belegt
      I_find = find(strcmp(RobName_base{i}, RobName_base(1:i-1)));
      if ~isempty(I_find)
        % Dieser Roboter ist nicht der erste seiner Art. Nehme gleiche Gruppe
        robgroups(i) = robgroups(I_find(1));
        if any(robgroups(1:i-1) > robgroups(i))
          warning('Rob. %d (%s): Gruppennummern nicht aufsteigend. Hier %d, max. %d', ...
            i, RobName_base{i}, robgroups(i), max(robgroups(1:i-1)));
        end
      else
        % Roboter ist der erste seiner Gruppe. Erzeuge neue Gruppe
        robgroups(i) = max(robgroups(:))+1;
      end
      pf_groupnum(pf_robnum==i) = robgroups(i);
    end
    % Gehe alle Gruppen durch und erzeuge eine neue Gruppen-Pareto-Front.
    % Sonst wird das Bild zu unübersichtlich und es gibt nicht-dominierende
    % Partikel im Bild, die missverständlich sind.
    for kk = 1:max(robgroups)
      Idom = pareto_dominance(pf_data(pf_groupnum==kk, :));
      I_group_kk = find(pf_groupnum==kk);
      pf_data(I_group_kk(Idom), :) = NaN; % Deaktiviere die Punkte
    end
    % Deaktiviere Roboter-Struktur wieder, wenn sie komplett dominiert wird
    % Damit kann die Anzahl der Roboter pro Gruppe in der Legende bestimmt
    % werden.
    for kk = II_acttype_act
      if all(any(isnan(pf_data(pf_robnum==kk,:)),2))
        pf_robnum(pf_robnum==kk) = 0; %#ok<AGROW>
        pf_groupnum(pf_robnum==kk) = 0;
        robgroups(kk) = 0;
      end
    end
  end
  % Bild zeichnen
  f = change_current_figure(10+pffig, vis_settings.figure_invisible);
  clf; hold on; grid on;
  set(f, 'name', sprintf('Pareto_Gesamt_%s',name_suffix), ...
    'NumberTitle', 'off', 'color','w');
  leghdl = []; legstr = {}; % Für Erstellung der Legende am Ende
  countmarker = 0; % Stelle Marker für jeden Roboter zusammen
  markerlist = {'x', 's', 'v', '^', '*', 'o', 'd', 'v', '<', '>', 'p', 'h'};
  colorlist =  {'r', 'g', 'b', 'c', 'm', 'k'};
  Plot_Indices = NaN(size(I_acttype,1),1);
  Line_Handles = NaN(size(I_acttype,1),1);

  % Sortiere die Strukturen so, dass die Roboter-Gruppen aufsteigend sind
  % (unklar, warum die Reihenfolge durcheinander gehen kann, eventuell bei manuellem Eingriff)
  % Falls keine Gruppen angezeigt werden, hat das hier keine Wirkung.
  [~, I_acttype_pfact_sort] = sort(robgroups(II_acttype_act));
  % Gehe durch die sortierten Strukturen durch und zeichne die Ergebnisse ein
  II_acttype_act_sort = II_acttype_act(I_acttype_pfact_sort);
  for j = 1:length(II_acttype_act_sort)
    i = II_acttype_act_sort(j);
    Name = Structures{i}.Name;
    Number = Structures{i}.Number;
    % Für Legende: Nur erfolgreiche PKM zählen (werden oben schon gefiltert
    if ~any(pf_robnum==i), continue; end
    % Falls alle Pareto-Partikel dieses Roboters dominiert wurden, weiter.
    if all(any(isnan(pf_data(pf_robnum==i,:)),2)), continue; end
    if pfvar == 1 || ... % Bild-Variante 1: jeden Roboter zählen
        pfvar == 2 && ... % Bild-Variante 2: Gruppen zählen
        (countmarker == 0 || all(robgroups(i)>robgroups(II_acttype_act_sort(1:j-1))))
      countmarker = countmarker + 1; % Hochzählen für die Marker und Farben
    end
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
      hdl=plot(pf_data(pf_robnum==i,IO1), pf_data(pf_robnum==i,IO2), marker);
    else % length(Set.optimization.objective) == 3
      hdl=plot3(pf_data(pf_robnum==i,IO1), pf_data(pf_robnum==i,IO2), pf_data(pf_robnum==i,IO3), marker);
    end
    set(hdl, 'DisplayName', sprintf('Rob%d_%s', Number, Name)); % zur Zuordnung später
    % Der Legendeneintrag wird im Fall von gruppierten Ergebnissen mehrmals
    % überschrieben. Dadurch nur ein Legendeneintrag pro Gruppe.
    leghdl(countmarker,:) = hdl; %#ok<AGROW>
    Plot_Indices(i) = countmarker;
    Line_Handles(i) = hdl;
    if ~isa(ResTab.Beschreibung, 'cell')
      RobShortName = ''; % Wenn kein Wert belegt ist, wird NaN gesetzt
    else
      RobShortName = ResTab.Beschreibung(strcmp(ResTab.Name,Name)); % Filter weglassen: `ResTab.LfdNr==i`. Damit auch Inkonsistenz csv zu Struktures-Var. möglich.
      RobShortName = RobShortName{1}; % Falls mehrere gleiche Namen vorkommen
    end
    if ~isempty(RobShortName)
      addtxt = sprintf('; %s', RobShortName);
    else
      addtxt = '';
    end
    if pfvar == 1 % Legendeneintrag einzelner Roboter
      legstr{countmarker} = sprintf('%d/%d (%s%s)', i, length_Structures, ...
        Structures{i}.Name, addtxt); %#ok<AGROW>
    else % Variante 2: Legendeneintrag mit Gruppe
       if sum(robgroups(1:i)==robgroups(i)) == 1
        % Nur ein Roboter dieser Gruppe. Dann direkt den vollen Namen
        % hinschreiben (aber Nummerierung der Gruppe beibehalten)
        legstr{countmarker} = sprintf('%d/%d (%s%s)', robgroups(i), max(robgroups), ...
          Structures{i}.Name, addtxt); %#ok<AGROW>
      else % Mehrere Roboter. Anderer Text mit Bezug auf Gruppe
        addtxt2 = sprintf(' (%d Rob.)', sum(robgroups(1:i)==robgroups(i)));
        legstr{countmarker} = sprintf('%d/%d (%s%s)%s', robgroups(i), max(robgroups), ...
          RobName_base{i}, addtxt, addtxt2); %#ok<AGROW>
      end
    end
  end % for i (Roboter)
  if ~isempty(objtext{IO1}), addtxtx=[' (', objtext{IO1}, ')']; else, addtxtx=''; end
  if ~isempty(objtext{IO2}), addtxty=[' (', objtext{IO2}, ')']; else, addtxty=''; end
  if pffig == 1
    xlabel(sprintf('%s%s in %s', Set.optimization.objective{IO1}, addtxtx, obj_units{IO1}));
    ylabel(sprintf('%s%s in %s', Set.optimization.objective{IO2}, addtxty, obj_units{IO2}));
  else
    xlabel(sprintf('%s%s (normiert)', Set.optimization.objective{IO1}, addtxtx));
    ylabel(sprintf('%s%s (normiert)', Set.optimization.objective{IO2}, addtxty));
  end
  if length(Set.optimization.objective) == 2
    if pffig == 1
      title(sprintf('%s: %s vs %s (physikalisch)', Set.optimization.optname, ...
        Set.optimization.objective{IO1}, Set.optimization.objective{IO2}), 'interpreter', 'none');
    else
      title(sprintf('%s: %s vs %s (normiert)', Set.optimization.optname, ...
        Set.optimization.objective{IO1}, Set.optimization.objective{IO2}), 'interpreter', 'none');
    end
  else
    if ~isempty(objtext{IO3}), addtxtz=[' (', objtext{IO3}, ')']; else, addtxtz=''; end
    if pffig == 1
      zlabel(sprintf('%s%s in %s', Set.optimization.objective{IO3}, addtxtz, obj_units{IO3}));
    else
      zlabel(sprintf('%s%s (normiert)', Set.optimization.objective{IO3}, addtxtz));
    end
    if pffig == 1
      title(sprintf('%s: %s vs %s vs %s (physikalisch)', ...
        Set.optimization.optname, Set.optimization.objective{IO1}, ...
        Set.optimization.objective{IO2}, Set.optimization.objective{IO3}), 'interpreter', 'none');
    else
      title(sprintf('%s: %s vs %s vs %s (normiert)', ...
        Set.optimization.optname, Set.optimization.objective{IO1}, ...
        Set.optimization.objective{IO2}, Set.optimization.objective{IO3}), 'interpreter', 'none');
    end
  end
  axhdl = get(f, 'children');
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
  % Erzeuge Dummy-Legende (damit DisplayName gleich bleibt)
  leghdl2 = leghdl;
  for i = 1:length(leghdl)
    leghdl2(i) = plot(NaN, NaN, 'x');
    set(leghdl2(i), 'Color', get(leghdl(i), 'Color'));
    set(leghdl2(i), 'Marker', get(leghdl(i), 'Marker'));
    set(leghdl2(i), 'MarkerSize', get(leghdl(i), 'MarkerSize'));
  end
  legend(leghdl2, legstr);
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
  if Set.general.debug_desopt
    menuitems = [menuitems, {'Pareto DesOpt', 'Pareto Einfluss DesOpt'}]; %#ok<AGROW>
  end
  if Set.optimization.joint_stiffness_active_revolute ~= 0 || ...
     Set.optimization.joint_stiffness_passive_revolute ~= 0 || ...
     Set.optimization.joint_stiffness_passive_universal ~= 0
    menuitems = [menuitems, 'Feder-Ruhelage']; %#ok<AGROW>
  end
  uicontrol('Style', 'popupmenu', ...
            'String', menuitems, ...
            'Units', 'pixels', ...
            'Position', [10, 30, 120, 24]);
  % ButtonDownFcn wird erst beim erneuten Laden der Datei eingetragen
  % (sonst wird die gespeicherte Datei zu groß).
  CreateFcn=@(src, dummy)cds_paretoplot_createfcn(src, dummy, Set.optimization.optname);
  set(f, 'CreateFcn', CreateFcn);
  saveas(f, fullfile(resmaindir, sprintf('Pareto_Gesamt_%s.fig',name_suffix)));
  % Für das bereits offene Fenster die ButtonDownFcns hier eintragen.
  cds_paretoplot_createfcn(f, [], Set.optimization.optname);
  fprintf('Pareto_Gesamt_%s.fig gespeichert. Dauer: %1.1fs\n', name_suffix, toc(t1));
  end % for pfact
  end % for pfvar
  end % for pffig
  end % for pfcomb
end
