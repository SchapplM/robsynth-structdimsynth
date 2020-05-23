% Berechne Kennzahl aus Kollisionserkennung
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% X
%   Trajektorie im Arbeitsraum (Basis-KS)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Structure
%   Eigenschaften der Roboterstruktur (aus cds_gen_robot_list.m)
% JP
%   Gelenkpositionen aller Gelenke des Roboters
%   (Zeilen: Zeitschritte der Trajektorie)
% Q
%   Gelenkpositionen(für PKM auch passive Gelenke)
% scale [2x1]
%   Skalierung der Ausgabe fval in bestimmten Wertebereich:
%   1: Untere Grenze,
%   2: Anzahl der Vielfachen der unteren Grenze minus 1 bis obere Grenze
% 
% Ausgabe:
% fval
%   Strafterm für Kollisionen. 0 falls keine Kollision. Sonst im Bereich
%   vorgegeben durch Eingabegröße range
% coll
%   Binärmatrix mit Kollision ja/nein für alle Zeitschritte aus Q und
%   Kollisionspaare aus Structure
% 
% Erzeugt Bild:
% Visualisierung der Kollisionen am Roboter (schlimmster Fall)
% Nur zu testende Kollisionspaare werden auch angezeigt.
% Werden nur benachbarte Beinketten einer PKM getestet, können
% gegenüberliegende Beinketten als Kollisionsfrei angezeigt werden.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval, coll] = cds_constr_collisions(R, X, Set, Structure, JP, Q, scale)

if Set.general.matfile_verbosity > 1
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constr_collisions_0.mat'));
  % load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constr_collisions_0.mat'));
end

%% Daten der Roboterstruktur aufbereiten
% TODO: Sollte das einmalig zu Beginn gemacht werden?
if Structure.Type == 0  % Seriell 
  collbodies = R.collbodies;
  collbodies.params = R.collbodies.params(:,1); % Aktuell nur Kapseln implementiert
  v = uint8(R.MDH.v);
else % PKM
  collbodies = struct('link', [], 'type', [], 'params', []);
  for k = 1:R.NLEG
    % Hänge die Kollisionskörper der Beinkette an
    % in I1L wird auch EE-Link noch mitgezählt. Hier nicht. Die Basis der
    % Beinkette muss gezählt werden
    if k > 1, NLoffset = 1+R.I2L_LEG(k-1)-(k-1)*1;
    else, NLoffset = 1; end % Offset für PKM-Basis (entspricht "nulltem" Eintrag)
    collbodies.link = [collbodies.link; R.Leg(k).collbodies.link + NLoffset];
    collbodies.type = [collbodies.type; R.Leg(k).collbodies.type];
    collbodies.params = [collbodies.params; R.Leg(k).collbodies.params(:,1)]; % nehme nur ersten Parameter (Radius)
  end
  % Vorgänger-Indizes zusammenstellen. Jede Beinkette hat zusätzliches
  % Basis-KS
  v = uint8(zeros(R.NJ+R.NLEG,1));
  for k = 1:R.NLEG
    if k > 1, NLoffset = R.I2L_LEG(k-1)-k+2;
    else, NLoffset = 1; end
    v(R.I1J_LEG(k)+k-1:R.I2J_LEG(k)+k) = [0; NLoffset+R.Leg(k).MDH.v];
  end
end
%% Kollisionen und Strafterm berechnen
[coll, colldepth] = check_collisionset_simplegeom_mex(v, collbodies, Structure.selfcollchecks_collbodies, JP);

if any(abs(colldepth(:))>1) || any(abs(colldepth(:))<0)
  error('Relative Eindringtiefe ist außerhalb des erwarteten Bereichs');
end
% Strafterm aus Kollisionserkennung: Wert ist automatisch zwischen 0 und 1.
% Allerdings voraussichtlich <<1, da 1 nur erreicht wird, wenn alle Körper
% komplett ineinanderliegen
f_constr = sum( colldepth(coll(:)==true) ) / length(coll(:));
% Größere Ausnutzung des Bereichs von 0 bis 1 -> 0.01 wird 0.5
f_constr_norm = 2/pi*atan(100*f_constr);
if f_constr_norm > 0
  % Skaliere auf Wertebereich. Dadurch Anpassung an doppelte Nutzung
  % (Eckwerte und Trajektorie möglich)
  fval = scale(1)*(1+scale(2)*f_constr_norm);
else
  % Als Rückgabe, dass es keine Kollision gibt
  fval = 0;
end
%% Debug: Zeichnen der Situation
if fval == 0 || fval >= Set.general.plot_details_in_fitness
  return
end
% Suche Datenpunkt mit größter Kollision
colldepth(isnan(colldepth)) = 0;
colldepth_t = sum(colldepth,2);
[~,j] = max(colldepth_t); % Index für Zeitschritt in Daten
% Bild zeichnen
change_current_figure(867); clf; hold all
view(3); axis auto; grid on;
xlabel('x in m');ylabel('y in m');zlabel('z in m');
if R.Type == 0 % Seriell
  s_plot = struct( 'ks', 1:R.NJ+2, 'straight', 1, 'mode', 1);
  R.plot( Q(j,:)', s_plot);
else % PKM
  s_plot = struct( 'ks_legs', [], 'straight', 1, 'mode', 1);
  R.plot( Q(j,:)', X(j,:)', s_plot);
end
collfound = false; % Prüf-Variable (s.u.)
for i = 1:size(collbodies.link,1)
  % Anfangs- und Endpunkt des Ersatzkörpers bestimmen
  if collbodies.type(i) ~= 6
    warning('Andere Methoden als Kapsel-Direktverbindung nicht implementiert');
    continue
  end
  r = collbodies.params(i,1)*3; % Vergrößere den Radius für den Plot
  % Nummer der Starrkörper in mathematischer Notation: 0=Basis
  jj2 = collbodies.link(i);
  jj1 = v(collbodies.link(i)); % 0=PKM-Basis, 1=Beinkette-1-Basis, 2=Beinkette-1-Körper-1
  % Nummer in Matlab-Notation (1=Basis)
  ii2 = jj2+1;
  ii1 = jj1+1;
  pts = JP(1,[3*(ii1-1)+1:3*ii1, 3*(ii2-1)+1:3*ii2]); % bezogen auf Basis-KS
  if all(pts(1:3) == pts(4:6))
    warning('Kollisionskörper %d/%d hat Länge Null. Verbindet Körper %d und %d.', ...
      i, size(collbodies.link,1), jj1, jj2);
  end
  % Umrechnung ins Welt-KS
  pts_W = repmat(R.T_W_0(1:3,4),2,1) + rotate_wrench(pts', R.T_W_0(1:3,1:3));
  % Kollisions-Ergebnis für diesen Kollisionskörper herausfinden
  I = Structure.selfcollchecks_collbodies(:,1) == i | ...
      Structure.selfcollchecks_collbodies(:,2) == i;
  collstate_i = coll(j,I);
  if any(collstate_i) % Es gibt eine Kollision
    color = 'r'; 
    collfound = true;
  else
    color = 'b';
  end
  drawCapsule([pts_W(1:3)',pts_W(4:6)',r],'FaceColor', color, 'FaceAlpha', 0.3);
end
sgtitle(sprintf('Kollisionsprüfung. Schritt %d/%d: %d/%d Koll. Sum. rel. Tiefe: %1.2f', ...
  j, size(Q,1), sum(coll(j,:)), size(coll, 2), colldepth_t(j)));
drawnow();
if fval > 0 && ~collfound
  warning('Vorher Kollision erkannt, aber jetzt nicht gezeichnet. Logik-Fehler.');
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constr_collisions_wrongplot.mat'));
end

