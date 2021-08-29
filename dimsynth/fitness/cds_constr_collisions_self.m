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
%   Untere und obere Grenze. Dazwischen linear
% 
% Ausgabe:
% fval
%   Strafterm für Kollisionen. 0 falls keine Kollision. Sonst im Bereich
%   vorgegeben durch Eingabegröße scale
% coll
%   Binärmatrix mit Kollision ja/nein für alle Zeitschritte aus Q und
%   Kollisionspaare aus Structure
% 
% Erzeugt Bild:
% Visualisierung der Kollisionen am Roboter (schlimmster Fall)
% Nur zu testende Kollisionspaare werden auch angezeigt.
% Werden nur benachbarte Beinketten einer PKM getestet, können
% gegenüberliegende Beinketten als Kollisionsfrei angezeigt werden.
% 
% Siehe auch: cds_constr_installspace, cds_constr_collisions_ws

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval, coll] = cds_constr_collisions_self(R, X, Set, Structure, JP, Q, scale)

if Set.general.matfile_verbosity > 1
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constr_collisions_self_0.mat'));
  % load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constr_collisions_self_0.mat'));
end
%% Daten der Roboterstruktur laden
% Wird bereits in cds_dimsynth_robot vorbereitet
collbodies = Structure.collbodies_robot;
collchecks = Structure.selfcollchecks_collbodies;
%% Kollisionen und Strafterm berechnen
CollSet = struct('collsearch', true);
[coll, ~, colldepth] = check_collisionset_simplegeom_mex( ...
  collbodies, collchecks, JP, CollSet);

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
  fval = scale(1)+(scale(2)-scale(1))*f_constr_norm;
else
  % Als Rückgabe, dass es keine Kollision gibt
  fval = 0;
end
%% Debug: Zeichnen der Situation
if fval == 0 || 1e4*fval >= Set.general.plot_details_in_fitness
  return
end
% Suche Datenpunkt mit größter Kollision
colldepth(isnan(colldepth)) = 0;
colldepth_t = sum(colldepth,2);
[~,j] = max(colldepth_t); % Index für Zeitschritt in Daten
% Debug: Detaillierte Informationen
% fprintf('%d Kollisionen für Zeitschritt %d/%d berechnet:\n', sum(coll(j,:)), j, size(coll,1));
% for kk = find(coll(j,:))
%   fprintf('Prüfung %d: Koll.-Körper %d vs %d (Rob.-Seg. %d vs %d)\n', kk, collchecks(kk,1), ...
%     collchecks(kk,2), collbodies.link(collchecks(kk,1),1), collbodies.link(collchecks(kk,2),1));
% end

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
num_coll_plot = 0; % zum Debuggen, s.u.
for i = 1:size(collbodies.link,1)
  % Anfangs- und Endpunkt des Ersatzkörpers bestimmen
  if ~any(collbodies.type(i) == [6 13 15])
    warning('Andere Methoden als Kapsel-Direktverbindung und Kapsel im Basis-KS nicht implementiert');
    continue
  end
  if collbodies.type(i) == 6 % Kapsel zum vorherigen KS
    r = collbodies.params(i,1)*3; % Vergrößere den Radius für den Plot
    % Nummer der Starrkörper in mathematischer Notation: 0=Basis
    jj2 = collbodies.link(i,1); % 0=PKM-Basis, 1=Beinkette-1-Basis, 2=Beinkette-1-Körper-1
    jj1 = collbodies.link(i,2);
    % Nummer in Matlab-Notation (1=Basis)
    ii2 = jj2+1;
    ii1 = jj1+1;
    pts = JP(j,[3*(ii1-1)+1:3*ii1, 3*(ii2-1)+1:3*ii2]); % bezogen auf Basis-KS
    if all(pts(1:3) == pts(4:6))
      warning('Kollisionskörper %d/%d hat Länge Null. Verbindet Körper %d und %d.', ...
        i, size(collbodies.link,1), jj1, jj2);
    end
  elseif collbodies.type(i) == 13 % Kapsel mit 2 angegebenen Punkten im Basis-KS
    r = collbodies.params(i,7)*3; % Vergrößerung für Plot
    pts = collbodies.params(i,1:6);
  elseif collbodies.type(i) == 15 % Kugel mit angegebenem Punkt im Basis-KS
    r = collbodies.params(i,4); % Keine Vergrößerung für Plot
    pts = [collbodies.params(i,1:3), NaN(1,3)]; % zweiter Teil ist Platzhalter
  else
    error('Fall nicht implementiert');
  end
  % Umrechnung ins Welt-KS
  pts_W = repmat(R.T_W_0(1:3,4),2,1) + rotate_wrench(pts', R.T_W_0(1:3,1:3));
  % Kollisions-Ergebnis für diesen Kollisionskörper herausfinden
  I = collchecks(:,1) == i | collchecks(:,2) == i;
  collstate_i = coll(j,I);
  if any(collstate_i) % Es gibt eine Kollision
    color = 'r'; num_coll_plot = num_coll_plot + 1;
    % Debug: Zusätzliche Diagnose:
    % collchecks_i = collchecks(I,:);
    % collpairs_i = collchecks_i(collstate_i,:);
    % collpartners_i = unique(collpairs_i(:));
    % collpartners_i(collpartners_i==i) = []; % Entferne Element selbst
    % fprintf('Koll.-Körper %d (Rob.-Seg. %d): Kollision mit KK [%s]\n', i, ...
    %   collbodies.link(i,1), disp_array(collpartners_i', '%d'));
  else
    color = 'b';
  end
  if any(collbodies.type(i) == [6 13]) % Kapsel
    drawCapsule([pts_W(1:3)',pts_W(4:6)',r],'FaceColor', color, 'FaceAlpha', 0.3);
  else
    drawSphere([pts_W(1:3)',r],'FaceColor', color, 'FaceAlpha', 0.3);
  end
end
sgtitle(sprintf('Selbstkollisionsprüfung. Schritt %d/%d: %d/%d Koll. Sum. rel. Tiefe: %1.2f', ...
  j, size(Q,1), sum(coll(j,:)), size(coll, 2), colldepth_t(j)));
drawnow();
[currgen,currind,currimg,resdir] = cds_get_new_figure_filenumber(Set, Structure,'CollisionsSelf');
for fileext=Set.general.save_robot_details_plot_fitness_file_extensions
  if strcmp(fileext{1}, 'fig')
    saveas(867, fullfile(resdir, sprintf('Gen%02d_Ind%02d_Eval%d_CollisionsSelf.fig', currgen, currind, currimg)));
  else
    export_fig(867, fullfile(resdir, sprintf('Gen%02d_Ind%02d_Eval%d_CollisionsSelf.%s', currgen, currind, currimg, fileext{1})));
  end
end
if any(num_coll_plot) ~= any(coll(j,:))
  % save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constr_collisions_self_1_errplot.mat'));
  error(['Status der geplotteten Kollisionen (logisch %d) stimmt nicht mit vorab ', ...
    'berechnetem (logisch %d) überein'], any(num_coll_plot), any(coll(j,:)));
end

