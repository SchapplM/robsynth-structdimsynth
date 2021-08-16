% Debug-Bild für Aufgabenredundanz: Zeichne Karte der Leistungsmerkmale
% 
% Eingabe:
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Structure
%   Eigenschaften der Roboterstruktur (aus cds_gen_robot_list.m)
% H_all [NI x NP x NK]
%   Diskretisierung aller `NK` IK-Zielfunktionen, die in der Einzelpunkt-IK
%   benutzt werden können. Für jedes Kriterium wird eine Rasterung erstellt
%   mit Auflösung NI x NP. Siehe RobBase/perfmap_taskred_ik
% s_ref [NI x 1]
%   Stützstellen des Trajektorienverlaufs für das zu erstellende Bild
%   Normalisierte Bahnkoordinate der Trajektorie, Auswahl von Zeit-Stütz-
%   stellen mit äquidistantem Wegverlauf.
% s_tref [NT x 1]
%   Normalisierte Stützstellen der Zeit-Trajektorie phiz_traj
%   Jeder Eckpunkt entspricht einer ganzen Zahl.
% phiz_range [NP x 1]
%   Stützstellen der EE-Rotation (phi_z) für zu erstellendes Bild
%   (Anzahl NP richtet sich nach der Auflösung des Bildes aus Einstellung)
% phiz_traj [NT x 1]
%   Trajektorie der redundanten Koordinate (phi_z) über `NT` Zeitschritte
% s_in
%   Struktur mir Einstellungswerten (siehe Quelltext)
% 
% Erzeugt Bild: Farbkarte des Verlaufs von Optimierungskriterien
% 
% Quelle: Schappler, M. and Ortmaier, T.: Singularity Avoidance of
% Task-Redundant Robots in Pointing Tasks: On Nullspace Projection and
% Cardan Angles as Orientation Coordinates, ICINCO 2021.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function cds_debug_taskred_perfmap(Set, Structure, H_all, s_ref, s_tref, phiz_range, phiz_traj, s_in)
%% Initialisierung
% Einstellungen laden und Standard-Werte setzen
s = struct( ...
  ... % high condition numers all get the same dark (or magenta) color to
  ... be able to better distinguish the good values.
  'colorlimit', 1e4, ...
  ... % Saturate all values above 100 to have more colors in the range of low
  ... % condition numbers. Saturate by decadic logarithm.
  'condsat_limit', 100, ...
  'i_ar', 0, ... % Iteration der Aufgabenredundanz-Schleife
  'wn', NaN); % Standard-Werte
if nargin == 8
  for f = fields(s_in)'
    if isfield(s, f{1})
      s.(f{1}) = s_in.(f{1});
    else % Fall soll eigentlich nicht vorkommen. Daher Prüfung als zweites
      error('Feld %s aus s_in kann nicht übergeben werden', f{1});
    end
  end
end
wn_plot = s.wn;
% Standard-Einstellung für Darstellung der Kriterien im Plot setzen
if all(isnan(s.wn)) || all(s.wn==0)
  if Structure.Type == 0
    wn_plot = [0;1;1;0]; % Grenzen und Jacobi-Kondition
  else
    wn_plot = [0;1;0;1;0]; % Grenzen und PKM-Jacobi-Kondition
  end
end
condsat_limit = s.condsat_limit;
colorlimit = s.colorlimit;

%% Bild vorbereiten
phiz_range_ext = phiz_range;
fighdl = change_current_figure(2400+s.i_ar);clf;hold on;
set(fighdl, 'Name', sprintf('PerfMap_Iter%d', s.i_ar), 'NumberTitle', 'off');
[X_ext,Y_ext] = meshgrid(s_ref,180/pi*phiz_range_ext);
Z_ext = zeros(size(X_ext));
% Create color code (CC) from performance criteria (i.e. condition number)
% Kriterien für PKM: 2 Winkelgrenzen (hyperb.), 4 Konditionszahl, 5 Kollision
% Benutze die Gewichtung, die auch für die IK der Trajektorie galt.
CC_ext = zeros(size(X_ext));
for i = 1:length(wn_plot)
  if wn_plot(i) == 0, continue; end
  CC_ext = CC_ext + wn_plot(i) * H_all(:,:,i)';
end
%% Farbskala berechnen
% Stelle die Grenze für den Farbbereich aus den Konditionszahlen her.
if Structure.Type == 0 % Seriell
  Hcond = H_all(:,:,3);
else % Parallel (PKM)
  Hcond = H_all(:,:,4);
end
if isnan(condsat_limit)
  condsat_limit = max(100,min(Hcond(:)));
end
if isnan(colorlimit)
  colorlimit = condsat_limit + 40;
end
assert(colorlimit > condsat_limit, 'colorlimit muss größer als condsat_limit sein');
% Wenn für die IK kein Kriterium aktiv war, zeichne trotzdem das
% Konditionszahl-Kriterium
if all(wn_plot(i)==0)
  CC_ext = Hcond';
end
% Begrenze die Farbwerte (siehe oben, Beschreibung von colorlimit)
I_colorlim = CC_ext>colorlimit;
CC_ext(I_colorlim) = colorlimit;
% Sättige Werte oberhalb des Schwellwertes. Dadurch mehr Farben in Skala
% für unteren Bereich mit guten Konditionszahlen
I_exc = CC_ext > condsat_limit;
CC_ext(I_exc) = condsat_limit+10*log10(CC_ext(I_exc)/condsat_limit); % last term gives 0...30 for condsat_limit=1e3

%% Bild erstellen
% Create color plot
surf(X_ext,Y_ext,Z_ext,CC_ext, 'EdgeColor', 'none');
xlabel('Normalized trajectory progress s (per point of support)', 'interpreter', 'none');
ylabel('Redundant coordinate phi_z in deg', 'interpreter', 'tex');
view([0,90])
% Set Colormap: low condition numbers white, high/singularity dark red.
colors_map = flipud(hot(1024));
% Falls starke Singularitäten oder Grenzverletzungen vorliegen, wird dies
% durch eine neue Farbe (Magenta) hervorgehoben
if any(I_colorlim(:))
  numcolors_sat = 1;  % add magenta for worst value. Do not show magenta in legend (1 value not visible)
  colors_map = [colors_map; repmat([255 0 255]/255, numcolors_sat,1)];
end
colormap(colors_map); % Farbskalierung mit Magenta als Farbe aktualisieren
% Titel eintragen
if Structure.Type == 0 % Seriell
  critnames = {'quadlim', 'hyplim', 'cond', 'coll'};
else % Parallel
  critnames = {'quadlim', 'hyplim', 'cond_ik', 'cond_pkm', 'coll'};
end
titlestr = '';
for i = 1:length(s.wn)
  if s.wn(i)
    if ~isempty(titlestr), titlestr = [titlestr, ', ']; end %#ok<AGROW>
    titlestr = [titlestr, sprintf('wn(%s)=%1.1f', critnames{i}, s.wn(i))]; %#ok<AGROW>
  end
end
if isempty(titlestr)
  titlestr = 'wn=0. Plot: cond(J).';
end
sgtitle(sprintf('Traj.-IK Iteration %d; %s', s.i_ar, titlestr), 'interpreter', 'none');
% Legende für Farben eintragen.
cb = colorbar();
cbtext = 'h(s,phi_z)';
if any(I_exc(:))
  cbtext = [cbtext, sprintf('; logscale for h>%1.0f (%1.0f%%)', ...
    condsat_limit, 100*sum(I_exc(:))/numel(I_exc))];
end
if any(I_colorlim(:))
  cbtext = [cbtext, sprintf('; h>%1.1e marked magenta (%1.1f%%)', ...
    colorlimit, 100*sum(I_colorlim(:))/numel(I_colorlim))];
end
ylabel(cb, cbtext, 'Rotation', 90, 'interpreter', 'tex');

% insert trajectory into plot
change_current_figure(fighdl);
plot(s_tref, 180/pi*phiz_traj, 'c-', 'linewidth', 2);

% Sonstige Formatierung
xlim([0, ceil(max(s_tref))]);
% Diagramm-Grenzen sind die Grenzen des berechneten Bereiches
ylim(180/pi*minmax2(phiz_range') + [-20, +20]);
% Alternative: Diagramm-Grenzen nur um gegebene Trajektorie
% ylim(180/pi*[min([-120*pi/180;min(phiz_traj)-20*pi/180]), ...
%              max([120*pi/180;max(phiz_traj)+20*pi/180])]);
set(fighdl, 'color','w');
if ~strcmp(get(fighdl, 'windowstyle'), 'docked')
  set(fighdl,'units','normalized','outerposition',[0 0 1 1]);
end
drawnow();

%% Save files
name = sprintf('TaskRedPerfMap_Iter%d', s.i_ar);
[currgen,currind,currimg,resdir] = cds_get_new_figure_filenumber(Set, Structure, name);
for fileext=Set.general.save_robot_details_plot_fitness_file_extensions
  if strcmp(fileext{1}, 'fig')
    saveas(fighdl, fullfile(resdir, sprintf('Gen%02d_Ind%02d_Eval%d_%s.fig', currgen, currind, currimg, name)));
  else
    export_fig(fighdl, fullfile(resdir, sprintf('Gen%02d_Ind%02d_Eval%d_%s.%s', currgen, currind, currimg, name, fileext{1})));
  end
end