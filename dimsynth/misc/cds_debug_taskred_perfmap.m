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

function cds_debug_taskred_perfmap(Set, Structure, H_all, s_ref, s_tref, phiz_range, phiz_traj, h_traj, s_in)
%% Initialisierung
if any(diff(s_ref<0)) || any(diff(phiz_range)<0) % Prüfe für Stützstellen
  cds_log(-1, '[debug/taskred_perfmap] Eingabedaten s_ref oder phiz_range sind nicht monoton');
  return
end
assert(size(s_ref,2)==1, 's_ref muss NI x 1 sein');
assert(size(s_tref,2)==1, 's_tref muss NT x 1 sein');
assert(size(phiz_range,2)==1, 'phiz_range muss NP x 1 sein');
assert(size(h_traj,2)==1&&size(h_traj,1)==size(s_tref,1), 'h_traj muss NT x 1 sein');
assert(size(H_all,1)==size(s_ref,1)&&size(H_all,2)==size(phiz_range,1), 'H_all muss NI x NP x NK sein');

% Einstellungen laden und Standard-Werte setzen
s = struct( ...
  ... % high condition numers all get the same dark (or magenta) color to
  ... be able to better distinguish the good values.
  'colorlimit', 1e8, ...
  ... % Saturate all values above e.g. 100 to have more colors in the range
  ... % of low condition numbers. Saturate by decadic logarithm.
  'condsat_limit', 100, ...
  'name_prefix_ardbg', '', ... % Für Dateinamen der zu speichernden Bilder
  'fval', NaN, ... % Für Titelbeschriftung
  'constrvioltext', '', ... % Für Titelbeschriftung
  'i_ar', 0, ... % Iteration der Aufgabenredundanz-Schleife
  'i_fig', 0, ... % Index von möglichen Bildern für Trajektorie
  'deactivate_time_figure', false, ... % Deaktiviere das zweite Bild
  'ignore_h0', true, ... % Breche ab, wenn gesuchte Kriterien alle Null sind
  'wn', NaN); % Standard-Werte
if nargin == 9
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
if all(isnan(s.wn))
  wn_plot(:) = 0;
end
condsat_limit = s.condsat_limit;
colorlimit = s.colorlimit;
% NaN-Werte aus Eingabe entfernen
I_valid = ~isnan(s_ref);
if any(~I_valid)
  cds_log(-1, sprintf('[debug/taskred_perfmap] Ungültige Bahnkoordinate in Eingabe'));
  s_ref = s_ref(I_valid);
  H_all = H_all(I_valid,:,:);
end
%% Bild vorbereiten
phiz_range_ext = phiz_range;
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
if s.ignore_h0 && all(CC_ext(:)==0 | isnan(CC_ext(:)))
  % Das zu zeichnende Bild enthält keine Informationen
  return
end
%% Farbskala berechnen
% Stelle die Grenze für den Farbbereich aus den Konditionszahlen her.
plot_cond = false;
if all(CC_ext(:)==0 | isnan(CC_ext(:))) % Keine der Gütekriterien hat zu einem Wert>0 geführt.
  % Der letzte Wert in H_all ist die Konditionszahl (direkt, nicht als
  % Kriterium mit Aktivierungsschwelle)
  CC_ext = H_all(:,:,end)';
  plot_cond = true;
end
if isnan(condsat_limit)
  condsat_limit = 100;
end
% Begrenze den Farbraum. Bezieht sich auf Werte oberhalb der Sättigung.
% Diese werden unten logarithmisch behandelt.
condsat_limit_rel = min(CC_ext(:)) + condsat_limit;
if isnan(colorlimit)
  condsat_limit_rel = condsat_limit + 1e4;
end
if isinf(condsat_limit_rel)
  condsat_limit_rel = condsat_limit;
end
assert(colorlimit > condsat_limit, 'colorlimit muss größer als condsat_limit sein');
colorlimit_rel = min(CC_ext(:)) + colorlimit;
if isinf(colorlimit_rel)
  colorlimit_rel = colorlimit;
end
assert(colorlimit_rel > condsat_limit_rel, 'colorlimit_rel muss größer als condsat_limit_rel sein');
CC_ext_orig = CC_ext;
% Begrenze die Farbwerte (siehe oben, Beschreibung von colorlimit)
I_colorlim = CC_ext>=colorlimit_rel;
CC_ext(I_colorlim) = colorlimit_rel;
% Sättige Werte oberhalb des Schwellwertes. Dadurch mehr Farben in Skala
% für unteren Bereich mit guten Konditionszahlen
I_exc = CC_ext >= condsat_limit_rel;
CC_ext(I_exc) = condsat_limit_rel+10*log10(CC_ext(I_exc)/condsat_limit_rel);

%% Bild erstellen
fighdl = change_current_figure(2400+10*s.i_ar+s.i_fig);clf;hold on;
set(fighdl, 'Name', sprintf('PerfMap_Iter%d_Fig%d', s.i_ar, s.i_fig), 'NumberTitle', 'off');
% Create color plot
surf(X_ext,Y_ext,Z_ext,CC_ext, 'EdgeColor', 'none');
xlabel('Normalized trajectory progress s (per point of support)', 'interpreter', 'none');
ylabel('Redundant coordinate phiz in deg', 'interpreter', 'tex');
view([0,90])
% Set Colormap:
% Alternative 1: Aufpassen: NaN-Werte (keine IK-Lösung) sind auch weiß.
% Müssen gekennzeichnet werden.
colors_map = flipud(hot(1024)); % white to dark red.
% Alternative 2: Bester Wert gelb hebt sich gegen weiß für NaN ab.
% colors_map = colormap(flipud(parula(1024))); % yellow to blue

% Falls starke Singularitäten oder Grenzverletzungen vorliegen, wird dies
% durch eine neue Farbe (Magenta) hervorgehoben
if any(I_colorlim(:))
  numcolors_sat = 1;  % add magenta for worst value. Do not show magenta in legend (1 value not visible)
  colors_map = [colors_map; repmat([255 0 255]/255, numcolors_sat,1)];
end
colormap(colors_map); % Farbskalierung mit Magenta als Farbe aktualisieren
% Titel eintragen
critnames = {'qlim_quad', 'qlim_hyp'};
if Structure.Type == 0 % Seriell
  critnames = [critnames(:)', {'cond'}];
else % Parallel
  critnames = [critnames(:)', {'cond_ik', 'cond_pkm'}];
end
critnames = [critnames(:)', {'coll_hyp', 'installspace', 'xlim_quad', 'xlim_hyp', 'coll_quad'}];
assert(length(critnames)==length(s.wn), sprintf(['Anzahl der Kriterien ', ...
  'unerwartet (ist %d, soll %d)'], length(s.wn), length(critnames)));
wnstr = '';
for i = 1:length(s.wn)
  if s.wn(i)
    if ~isempty(wnstr), wnstr = [wnstr, ', ']; end %#ok<AGROW>
    wnstr = [wnstr, sprintf('wn(%s)=%1.1g', critnames{i}, s.wn(i))]; %#ok<AGROW>
  end
end
if isempty(wnstr)
  wnstr = 'wn=0. Plot: cond(J)';
end
if plot_cond
  if ~isempty(wnstr), wnstr=[wnstr,'; ']; end
  wnstr = [wnstr, 'h=0. Plot: cond(J)'];
end
titlestr = sprintf('It. %d, fval=%1.1e; %s; %s', s.i_ar, s.fval, wnstr, s.constrvioltext);
sgtitle(titlestr, 'interpreter', 'none');
% Legende für Farben eintragen.
cb = colorbar();
cbtext = 'h(s,phiz)';
if any(I_exc(:))
  cbtext = [cbtext, sprintf('; logscale for h>%1.0f (%1.0f%%)', ...
    condsat_limit_rel, 100*sum(I_exc(:))/numel(I_exc))];
end
if any(I_colorlim(:))
  cbtext = [cbtext, sprintf('; h>%1.1e marked magenta (%1.1f%%)', ...
    colorlimit_rel, 100*sum(I_colorlim(:))/numel(I_colorlim))];
end
ylabel(cb, cbtext, 'Rotation', 90, 'interpreter', 'tex');

% insert trajectory into plot
change_current_figure(fighdl);
hdl = NaN(6,1);
hdl(1) = plot(s_tref, 180/pi*phiz_traj, 'b+-', 'linewidth', 2);

if Structure.Type == 0 % Seriell
  idxshift = 0;
else % Parallel (PKM)
  idxshift = 1;
end
formats = {'bx', 'g*', 'g^', 'co', 'gv', 'm+'};
legtxt = {'Traj', ... % Legendeneintrag für die T. in hdl(1)
  'Joint Lim', 'Act. Sing.', 'IK Sing.', 'Collision', 'Install. Space', 'Out of Range'};
for i = 1:6
  % Bestimme Indizes für bestimmte Sonderfälle, wie Gelenküberschreitung,
  % Singularität, Kollision, Bauraumverletzung.
  % Mit den Farben sind diese Bereiche nicht eindeutig zu kennzeichnen, da
  % immer die summierte Zielfunktion gezeichnet wird
  % Zeichne auch ein, wenn die Nebenbedingungen in der Farbskala gar nicht
  % vorkommen. Sonst schlechter vorhersehbar, warum Traj. im nächsten Schritt fehlschlägt.
  switch i % Index passend zu Einträgen in legtxt
    case 1
      I = isinf(H_all(:,:,2)'); % Gelenkgrenzen
    case 2
      I = H_all(:,:,end)' > 1e3; % Kondition Jacobi (Antriebe)
    case 3
      if Structure.Type == 2
        I = H_all(:,:,end-1)' > 1e3; % Kondition IK-Jacobi (Beinketten)
      else
        continue; % Keine Singularitäten der IK
      end
    case 4
      I = isinf(H_all(:,:,4+idxshift)'); % Kollision
    case 5
      I = isinf(H_all(:,:,5+idxshift)'); % Bauraum
    case 6 % ist immer vor weißem Hintergrund (siehe colormap). Daher passt magenta gut.
      I = isnan(H_all(:,:,1)'); % IK ungültig / nicht lösbar (Reichweite)
  end
  if ~any(I(:))
    continue
  end
  x_i = X_ext(I);
  y_i = Y_ext(I);
  hdl(1+i) = plot(x_i(:), y_i(:), formats{i}, 'MarkerSize', 3);
end
I_hdl = ~isnan(hdl);

% Grenzen einzeichnen
plot(minmax2(s_tref')', 180/pi*min(phiz_range)*[1;1], 'k--');
plot(minmax2(s_tref')', 180/pi*max(phiz_range)*[1;1], 'k--');

% Sonstige Formatierung
xlim([0, ceil(max(s_tref))]);
% Diagramm-Grenzen sind die Grenzen des berechneten Bereiches und die der
% Trajektorie
ylim(180/pi*minmax2([phiz_range', phiz_traj(phiz_traj>-4*pi&phiz_traj<4*pi)']) + [-20, +20]);
% Alternative: Diagramm-Grenzen nur um gegebene Trajektorie
% ylim(180/pi*[min([-120*pi/180;min(phiz_traj)-20*pi/180]), ...
%              max([120*pi/180;max(phiz_traj)+20*pi/180])]);

% Replizieren der Daten des Bildes über berechneten Bereich hinaus. Dadurch
% bessere Lesbarkeit bei Rotationen >180°. Funktioniert nicht für das
% Gelenkwinkel-Kriterium, nur z.B. bei Konditionszahl
for ysign = [-1, +1]
  Y_ext2 = Y_ext + ysign*360;
  % Wähle nur die Indizes aus, die nicht mit den bisherigen Daten
  % überlappen. Alle Spalten von Y_ext sind identisch (grid)
  if ysign < 0
    Iy2 = Y_ext2(:,1)<=min(Y_ext(:,1));
  else
    Iy2 = Y_ext2(:,1)>=max(Y_ext(:,1));
  end
  % Erneuter Plot
  surf(X_ext(Iy2,:),Y_ext2(Iy2,:),Z_ext(Iy2,:),CC_ext(Iy2,:), 'EdgeColor', 'none', 'FaceAlpha', 0.5);
end
% Legende erst hier einzeichnen. Sonst werden spätere Objekte auch
% eingetragen.
legend(hdl(I_hdl), legtxt(I_hdl), 'Location', 'NorthOutside', 'Orientation', 'horizontal');
set(fighdl, 'color','w'); grid on;
drawnow();

%% Save files
[~,~,~,resdir] = cds_get_new_figure_filenumber(Set, Structure, '');
for fileext=Set.general.save_robot_details_plot_fitness_file_extensions
  if strcmp(fileext{1}, 'fig')
    saveas(fighdl, fullfile(resdir, sprintf('%s_TaskRed_Traj%d_Fig%d_PerfMap.fig', ...
      s.name_prefix_ardbg, s.i_ar, s.i_fig)));
  else
    export_fig(fighdl, fullfile(resdir, sprintf('%s_TaskRed_Traj%d_Fig%d_PerfMap.%s', ...
      s.name_prefix_ardbg, s.i_ar, s.i_fig, fileext{1})));
  end
end

%% Bild mit Verlauf der Kriterien über die Trajektorie
if all(isnan(phiz_traj)) || s.deactivate_time_figure
  return; % Nichts zu zeichnen
end
% Dient zum Abgleich der Redundanz-Karte mit der IK
% Interpoliere die Werte der Redundanz-Karte über die Trajektorie. Abfangen
% von Fehlerhaften Daten für die Interp-Funktion
[~,I] = unique(s_ref);
h_interp = interp2(s_ref(I),180/pi*phiz_range_ext,CC_ext_orig(:,I),s_tref,phiz_traj);
fighdl2 = change_current_figure(2500+10*s.i_ar+s.i_fig);clf;hold on;
set(fighdl2, 'Name', sprintf('PerfValues_Iter%d_Fig%d', s.i_ar, s.i_fig), 'NumberTitle', 'off');
plot(s_tref, h_traj);
plot(s_tref, h_interp);
xlabel('Normalized trajectory progress s (per point of support)', 'interpreter', 'none');
ylabel(sprintf('Performance criterion for trajectory IK: %s', wnstr), 'interpreter', 'none');
set(fighdl2, 'color','w');
legend({'from IK', 'interp. from map'});
grid on;
sgtitle(titlestr, 'interpreter', 'none');

% Zweites Bild speichern
for fileext=Set.general.save_robot_details_plot_fitness_file_extensions
  if strcmp(fileext{1}, 'fig')
    saveas(fighdl2, fullfile(resdir, sprintf('%s_TaskRed_Traj%d_Fig%d_PerfValues.fig', ...
      s.name_prefix_ardbg, s.i_ar, s.i_fig)));
  else
    export_fig(fighdl2, fullfile(resdir, sprintf('%s_TaskRed_Traj%d_Fig%d_PerfValues.%s', ...
      s.name_prefix_ardbg, s.i_ar, s.i_fig, fileext{1})));
  end
end

