% Berechne Kennzahl aus relativem Abstand von Gelenken und Plattform
% (nur für PKM definiert)
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Structure
%   Eigenschaften der Roboterstruktur (aus cds_gen_robot_list.m)
% JP [NT x ...]
%   Gelenkpositionen aller Gelenke des Roboters
%   (Zeilen: Zeitschritte der Trajektorie bzw. Nummer der Eckpunkte)
% Q
%   Gelenkpositionen (für PKM auch passive Gelenke)
% X
%   Trajektorie im Arbeitsraum (Basis-KS)
% scale [2x1]
%   Skalierung der Ausgabe fval in bestimmten Wertebereich:
%   Untere und obere Grenze. Dazwischen mit arctan.
% 
% Ausgabe:
% fval
%   Strafterm für Kollisionen. 0 falls keine Kollision. Sonst im Bereich
%   vorgegeben durch Eingabegröße scale
% 
% Erzeugt Bild:
% Visualisierung der Überschreitung am Roboter (schlimmster Fall)
% 
% Siehe auch: cds_constr_installspace

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2025-04
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval, JPz_joints_beyond_plf_max] = cds_constraints_platform_position_rel(R, Set, Structure, JP, Q, X, scale)
if Structure.Type ~= 2
  error('Funktion cds_constraints_platform_position_rel ist nur für PKM definiert');
end
if Set.general.matfile_verbosity > 1
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_platform_position_rel_0.mat'));
  % load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_platform_position_rel_0.mat'));
end
% z-Koordinate aller Gelenkpunkte (im Basis-KS der PKM)
% z0-Achse zeigt zur Plattform (in Normal-Konfiguration)
JP_z = JP(:,3+(3:3:end-3)); % Nehme nicht das PKM-Basis-KS (ist eh [0,0,0])
% Bestimme die Gelenk-Indizes: Erster Eintrag ist die jeweilige Beinketten-Basis
I_JPsel = []; % Auszuwählende Indizes für die positionsbeeinflussenden Gelenke des Koppelgelenks in der Beinkette
for j = 1:R.NLEG
  % Wähle den Index für die Gelenkpunkte aus. Bei Schubgelenken ist das
  % Gelenkkoordinatensystem nach der Schubgelenkkoordinate. Das liegt dann
  % schon im nächsten Gelenk. Bspw. bei 6-UPS im S-Gelenk (Koppelgelenk).
  if R.Leg(1).MDH.sigma(R.NQJ_LEG_bc) == 1
    idx_lastchainjoint = R.NQJ_LEG_bc - 1;
  else
    idx_lastchainjoint = R.NQJ_LEG_bc;
  end
  I_JPsel = [I_JPsel, (j-1)*R.Leg(j).NL+(1:(1+idx_lastchainjoint))]; %#ok<AGROW>
end
JPz_joints = JP_z(:,I_JPsel); % z-Koordinaten der betrachteten Gelenke
JPz_plf = JP_z(:,end-1); % z-Komponente des Plattform-KS; nicht EE-KS.
% Bestimme, wie weit die Gelenke jenseits der Plattform liegen (positiv gezählt)
JPz_joints_beyond_plf = JPz_joints - repmat(JPz_plf,1,size(JPz_joints,2));
JPz_joints_beyond_plf_max = max(JPz_joints_beyond_plf(:));
% Berücksichtigung des maximalen Abstands, den die Gelenke jenseits der
% Plattform liegen dürfen (ist in der Regel positiv, damit die Plattform
% immer etwas unter den Gelenken liegt)
JPz_exceed_dist = JPz_joints_beyond_plf_max + ...
  Set.optimization.platform_beyond_robot_structure_min_abs;
if JPz_exceed_dist > 0
  % Normierung der Ausgabe.
  % * 1m Überschreitung entspricht Wert 0.5
  % * 2m entsprechen 0.7
  f_constr_norm = 2/pi*atan(JPz_exceed_dist);
  % Skaliere auf Wertebereich. Dadurch Anpassung an doppelte Nutzung
  % (Eckwerte und Trajektorie möglich)
  fval = scale(1) + (scale(2)-scale(1))*f_constr_norm;
else
  fval = 0; % Alle Gelenke nah genug an der Basis (weit genug von Plattform weg). Alles i.O.
end

%% Debug: Zeichnen der Situation (Abfrage für Ausstieg aus Funktion)
if fval ~= 0 && ... % Nur Zeichnen, wenn auch Kollisionen auftreten
   (Set.general.plot_details_in_fitness < 0 && 1e4*fval >= abs(Set.general.plot_details_in_fitness) || ... % Gütefunktion ist schlechter als Schwellwert: Zeichne
    Set.general.plot_details_in_fitness > 0 && 1e4*fval <= abs(Set.general.plot_details_in_fitness))
  % Plotten
else
  return
end
%% Debug: Zeichnen der Situation
% Neuberechnung des obigen Kriteriums für alle Gelenke des Roboters (nicht
% nur der positionsbeeinflussenden); enthält dann auch Plattform-Koppelgelenke
[JPz_joints_beyond_plf_max2,idx_point_worst] = max(JPz_joints_beyond_plf,[],2);
[JPz_joints_beyond_plf_max3,idx_timestep_worst] = max(JPz_joints_beyond_plf_max2);
idx_point_worst2 = idx_point_worst(idx_timestep_worst);
assert(JPz_joints_beyond_plf_max == JPz_joints_beyond_plf_max3);
% fprintf(['Schlimmste Entfernung bei Zeitschritt %d (Gelenk %d in ' ...
%   'reduzierter Menge; %d in vollständiger Menge)\n'], idx_timestep_worst, ...
%   idx_point_worst(idx_timestep_worst), I_JPsel(idx_point_worst2));
assert(JPz_joints_beyond_plf(idx_timestep_worst,idx_point_worst2)==JPz_joints_beyond_plf_max3);
JPz_joints_beyond_plf_all = JP_z - repmat(JPz_plf,1,size(JP_z, 2));
JPz_exceed_dist_all = JPz_joints_beyond_plf_all + ...
  Set.optimization.platform_beyond_robot_structure_min_abs;

if isempty(idx_timestep_worst)
  j = 1; % Debug: Keine Verletzung aufgetreten. Trotzdem zeichnen
else
  j = idx_timestep_worst; % Index für Zeitschritt in Daten
end

% Auch x- und y-Komponenten der Gelenkpositionen bestimmen
JP_x = JP(:,3+(1:3:end-3));
JP_y = JP(:,3+(2:3:end-3));

% Bild zeichnen
fhdl = change_current_figure(870); clf; hold all
view(3); axis auto; grid on;
xlabel('x in m');ylabel('y in m');zlabel('z in m');
title(sprintf('Konfiguration für Punkt %d/%d', j, size(JP_z, 1)));
s_plot = struct('straight', 1, 'mode', 1);
s_plot.jointsize = Structure.plot_jointsize/2; % Sehr kleine Gelenke
if any(R.Type == [0 1]) % Seriell
  s_plot.ks = 1:R.NJ+2;
  R.plot( Q(j,:)', s_plot);
else % PKM
  s_plot.ks_legs = [];
  R.plot( Q(j,:)', X(j,:)', s_plot);
end

% Punkte einzeichnen
leghdl = gobjects(4,1);
for i = 1:size(JP_z,2)
  pt_0 = [JP_x(j,i); JP_y(j,i); JP_z(j,i)];
  % Umrechnung ins Welt-KS
  pt_W = R.T_W_0(1:3,4) + R.T_W_0(1:3,1:3)*pt_0;
  hdl = plot3(pt_W(1), pt_W(2), pt_W(3), 'bo');

  color = 'k';
  marker = 'x';
  MarkerSize = 25;
  if i == I_JPsel(idx_point_worst2)
    MarkerSize = 40;
    color = 'm';
    marker = '+';
    leghdl(4) = hdl;
  elseif any(i == I_JPsel) % Punkt ist relevant
    if JPz_exceed_dist_all(j,i) > 0
      color = 'r';
      leghdl(2) = hdl;
    else
      color = 'g';
      leghdl(3) = hdl;
    end
  else % Punkt ist nicht relevant
    leghdl(1) = hdl;
  end
  set(hdl, 'Color', color, 'Marker', marker, 'MarkerSize', MarkerSize)
end
% Legende (einzeichnen, für existierende Linien)
I = false(4,1);
for i = 1:4
  I(i) = ~isa(leghdl(i), 'matlab.graphics.GraphicsPlaceholder');
end
legtxt = {'nicht relevant', 'i.O.: weit genug hinter Plattform', ...
  'n.i.O.: zu weit hinter der Plattform', sprintf('größte Verletzung: %1.1fmm', 1e3*JPz_exceed_dist)};
legend(leghdl(I), legtxt(I));