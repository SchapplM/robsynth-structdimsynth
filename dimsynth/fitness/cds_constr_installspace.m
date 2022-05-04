% Berechne Nebenbedingungs-Kennzahl aus Bauraumbegrenzung
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
%   Strafterm für Bauraumverletzung. 0 falls keine Überschreitung. Sonst im
%   Bereich vorgegeben durch Eingabegröße scale
% f_constr
%   Abstand des am weitesten entfernten Punktes des Roboters im
%   schlechtesten Fall
% 
% Erzeugt Bild:
% Visualisierung der Baumraumverletzung am Roboter (schlimmster Fall)
% 
% Siehe auch: cds_constr_collisions_self, cds_constr_collisions_ws

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-05
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval, f_constr] = cds_constr_installspace(R, X, Set, Structure, JP, Q, scale)

if Set.general.matfile_verbosity > 1
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constr_installspace_0.mat'));
  % load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constr_installspace_0.mat'));
end
%% Daten der Roboterstruktur laden
% Lade die vorab initialisierten Ersatzpunkte für die Bauraumprüfung.
% Hier sind die Bauraum-Geometrien und die Roboter-Punkte bereits definiert
% Siehe cds_update_collbodies.
collbodies = Structure.installspace_collbodies;
% Merke, bis wohin die Kollisionsobjekte zum Bauraum gehören (die ersten
% der Liste). Die restlichen gehören zum Roboter
n_cb_instspc = size(Set.task.installspace.type,1);
n_cb_robot = size(collbodies.type,1) - n_cb_instspc;
% Liste der Kollisionsprüfungen: Schon vorab bestimmt.
collchecks = Structure.installspace_collchecks_collbodies;

%% Bestimme Geometrieübereinstimmung mit Bauraum
% Nehme Funktion für Kollisionsprüfung mit geänderter Einstellung.
% Eine "Kollision" des Roboters mit der Bauraum-Geometrie erfüllt die
% Nebenbedingung, dass der Roboter im Bauraum enthalten ist
CollSet = struct('collsearch', false);
[coll, absdist] = check_collisionset_simplegeom_mex(collbodies, collchecks, JP, CollSet);
% Ergebnisse nachverarbeiten
ininstallspace_all = false(n_cb_robot,1); % ist Roboterobjekt im Bauraum?
mindist_all = zeros(n_cb_robot,1); % Wie ist der minimale Abstand zum Bauraum?
idx_timestep_worst = ones(n_cb_robot,1); % Zu welchem Zeitschritt tritt der schlechteste Fall ein?
% Alle Roboter-Objekte durchgehen
for i = 1:n_cb_robot
  % Indizes aller Kollisionsprüfungen mit diesem Roboter-Objekt i
  % (Roboter-Objekte sind immer die letzten Kollisionsobjekte (s.o.)
  I = collchecks(:,1) == n_cb_instspc+i;
  % Kollisionsergebnis für alle Bauraum-Prüfungen für dieses Roboter-Objekt
  collstate_i = coll(:,I);
  % Wenn Objekt i in irgendeinem Bauraum-Objekt (any-Prüfung) und das für
  % alle Zeitschritte gilt (all-Prüfung), dann ist Objekt i immer im
  % Bauraum und damit keine Verletzung der Nebenbedingung
  ininstallspace_all(i) = all(any(collstate_i,2));

  % Optional überspringen (dann keine Ausgabe f_constr verfügbar):
%   if ininstallspace_all(i)
%     continue % keine Prüfung der Distanz zum Bauraum notwendig
%   end
  % Da es mehrere Bauraumobjekte geben kann, zählt immer das am nächsten
  % gelegene für die Prüfung. Bestimme das Minimum über die Spalten.
  % Der kleinste Wert ist die geringste Verletzung (negativ = enthalten)
  mindist_i = min(absdist(:,I),[],2);

  % Alternativ: Prüfe nur die Zeitschritte I_i_out, bei denen Roboter-Objekt 
  % i außerhalb des Bauraums liegt. Dann weniger Aufwand für max-Funktion.
%   I_i_out = ~any(coll(:,I),2); % Binär-Indizes
%   II_i_out = find(I_i_out); % Zähl-Indizes zum besseren abspeichern
%   mindist_i = min(absdist(I_i_out,I),[],2);

  % Bestimme den Zeitschritt, an dem das Roboter-Objekt i am weitesten
  % vom Bauraum entfernt ist. Das entspricht dem maximalen Minimalabstand
  % zu allen Bauraum-Geometrien
  [mindist_all(i),idx_tmp] = max(mindist_i);
  % finde den Index des oben gefundenen Objektes in der Variable I_i_out.
  % Das wird dann auf die Zeitindizes der Eingabevariablen (JP,...) um-
  % gerechnet.
  idx_timestep_worst(i) = idx_tmp;
  % Alternative (s.o.):
%   idx_timestep_worst(i) = II_i_out(idx_tmp);
end

% Nehme den schlechtesten Fall. Also den Zeitpunkt an dem ein Körper den
% maximal möglichen Abstand (von außen) zum Bauraum hat (beziehungsweise
% am nächsten von innen an der Grenze dran ist)
[f_constr, idx_body_worst] = max(mindist_all);

% Strafterm für Bauraumprüfung:
if all(ininstallspace_all) % && all(mindist_all < 0) % zweite Abfrage für Prüfung auf Gleichheit
  fval = 0; % Alle Punkte im Bauraum. Alles i.O.
else
  % Normierung der Ausgabe. Wert von f_constr (in m) ist größer 0 aber
  % voraussichtlich nicht >> 1.
  % * 1m schlechtester Abstand entspricht Wert 0.5
  % * 2m entsprechen 0.7
  f_constr_norm = 2/pi*atan(f_constr);
  % Skaliere auf Wertebereich. Dadurch Anpassung an doppelte Nutzung
  % (Eckwerte und Trajektorie möglich)
  fval = scale(1) + (scale(2)-scale(1))*f_constr_norm;
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
% Suche Datenpunkt mit weitester Entfernung vom Bauraum (schlechtester Fall)
if isempty(idx_timestep_worst)
  j = 1; % Debug: Keine Verletzung aufgetreten. Trotzdem zeichnen
else
  j = idx_timestep_worst(idx_body_worst); % Index für Zeitschritt in Daten
end
% Bild zeichnen
change_current_figure(868); clf; hold all
view(3); axis auto; grid on;
xlabel('x in m');ylabel('y in m');zlabel('z in m');
% Trajektorie zeichnen
X_W = (repmat(R.T_W_0(1:3,4), 1, size(X,1)) + R.T_W_0(1:3,1:3)*X(:,1:3)')';
plot3(X_W(:,1), X_W(:,2), X_W(:,3), 'ko');
if R.Type == 0 % Seriell
  s_plot = struct( 'ks', 1:R.NJ+2, 'straight', 1, 'mode', 1);
  R.plot( Q(j,:)', s_plot);
else % PKM
  s_plot = struct( 'ks_legs', [], 'straight', 1, 'mode', 1);
  R.plot( Q(j,:)', X(j,:)', s_plot);
end
num_outside_plot = 0; % zum Debuggen, s.u.
for i = 1:size(collbodies.link,1)
  % Anfangs- und Endpunkt des Ersatzkörpers bestimmen
  if all(collbodies.type(i) ~= [6 9 10 12 13 14])
    warning('Methode %d nicht implementiert', collbodies.type(i));
    continue
  end
  % Nummer der Starrkörper in mathematischer Notation: 0=Basis
  jj2 = collbodies.link(i,1); % 0=PKM-Basis, 1=Beinkette-1-Basis, 2=Beinkette-1-Körper-1
  jj1 = collbodies.link(i,2);
  % Nummer in Matlab-Notation (1=Basis)
  ii2 = jj2+1;
  ii1 = jj1+1;
  pts = JP(j,[3*(ii1-1)+1:3*ii1, 3*(ii2-1)+1:3*ii2]); % bezogen auf Basis-KS
  % Umrechnung ins Welt-KS
  pts_W = repmat(R.T_W_0(1:3,4),2,1) + rotate_wrench(pts', R.T_W_0(1:3,1:3));
  if i > n_cb_instspc
    % Das Objekt ist Teil des Roboters und muss innerhalb mindestens eines
    % Bauraum-Objektes liegen
    % Kollisions-Ergebnis für diesen Kollisionskörper herausfinden
    I = collchecks(:,1) == i | collchecks(:,2) == i;
    collstate_i = coll(j,I);
    if ~any(collstate_i) % Das Roboterobjekt ist in keinem einzigen Bauraum-Objekt
      color = 'r'; num_outside_plot = num_outside_plot + 1;
      % Zusätzliche Diagnose
      % fprintf('Koll.-Körper %d (Rob.-Seg. %d): In keinem zugeordneten Bauraum-Volumen.\n', i, ...
      %   collbodies.link(i,1));
    else
      % Das Roboterobjekt ist im Bauraum -> gut
      color = 'g';
    end
  else
    % Das Objekt ist eine Bauraumbegrenzung und muss nur gezeichnet werden
    color = 'b'; 
  end
  switch collbodies.type(i) % Siehe check_collisionset_simplegeom für Nummern
    case 6 % Kapsel im Körper-KS
      r = collbodies.params(i,1)*3; % Vergrößere den Radius für den Plot
      drawCapsule([pts_W(1:3)',pts_W(4:6)',r],'FaceColor', color, 'FaceAlpha', 0.3);
    case 9 % Punkt im Körper-KS
      % Zweiter Punkt des Punktepaars, das für den Kollisionskörper aus den
      % KS-Ursprüngen gespeichert wurde. (erster ist Vorgänger).
      plot3(pts_W(4), pts_W(5), pts_W(6), [color,'x'], 'markersize', 20);
    case 10 % Quader im Basis-KS
      % Parameter auslesen. Transformation ins Welt-KS für Plot
      q_W = eye(3,4)*R.T_W_0*[collbodies.params(i,1:3)';1];
      u1_W = R.T_W_0(1:3,1:3)*collbodies.params(i,4:6)';
      u2_W = R.T_W_0(1:3,1:3)*collbodies.params(i,7:9)';
      % letzte Kante per Definition senkrecht auf anderen beiden.
      u3_W = cross(u1_W,u2_W); u3_W = u3_W/norm(u3_W)*collbodies.params(i,10);
      % Umrechnen in Format der plot-Funktion
      cubpar_c = q_W(:)+(u1_W(:)+u2_W(:)+u3_W(:))/2; % Mittelpunkt des Quaders
      cubpar_l = [norm(u1_W); norm(u2_W); norm(u3_W)]; % Dimension des Quaders
      cubpar_a = 180/pi*tr2rpy([u1_W(:)/norm(u1_W), u2_W(:)/norm(u2_W), u3_W(:)/norm(u3_W)],'zyx')'; % Orientierung des Quaders
      drawCuboid([cubpar_c', cubpar_l', cubpar_a'], ...
        'FaceColor', color, 'FaceAlpha', 0.1);
    case 12 % Zylinder im Basis-KS
      % Transformation ins Welt-KS
      p1 = eye(3,4)*R.T_W_0*[collbodies.params(i,1:3)';1];
      p2 = eye(3,4)*R.T_W_0*[collbodies.params(i,4:6)';1];
      drawCylinder([p1', p2', collbodies.params(i,7)], ...
        'FaceColor', color, 'FaceAlpha', 0.2);
    case 13 % Kapsel im Basis-KS
      % Transformation ins Welt-KS
      p1 = eye(3,4)*R.T_W_0*[collbodies.params(i,1:3)';1];
      p2 = eye(3,4)*R.T_W_0*[collbodies.params(i,4:6)';1];
      drawCapsule([p1', p2', collbodies.params(i,7)], ...
        'FaceColor', color, 'FaceAlpha', 0.2);
    case 14 % Punkt im Basis-KS
      p = eye(3,4)*R.T_W_0*[collbodies.params(i,1:3)';1];
      plot3(p(1), p(2), p(3), [color,'*'], 'markersize', 15);
    otherwise
      error('Dieser Fall darf nicht auftreten');
  end
end
sgtitle(sprintf('Bauraumprüfung. Schritt %d/%d: Weitester Abstand: %1.2fmm', ...
  j, size(Q,1), 1e3*f_constr));
drawnow();
[currgen,currind,currimg,resdir] = cds_get_new_figure_filenumber(Set, Structure,'InstallSpace');
for fileext=Set.general.save_robot_details_plot_fitness_file_extensions
  if strcmp(fileext{1}, 'fig')
    saveas(868, fullfile(resdir, sprintf('Gen%02d_Ind%02d_Eval%d_InstallSpace.fig', currgen, currind, currimg)));
  else
    export_fig(868, fullfile(resdir, sprintf('Gen%02d_Ind%02d_Eval%d_InstallSpace.%s', currgen, currind, currimg, fileext{1})));
  end
end
if num_outside_plot == 0
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constr_installspace_1_errplot.mat'));
  error('Anzahl der geplotteten Bauraumverletzungen stimmt nicht mit vorab berechneten überein');
end
return
