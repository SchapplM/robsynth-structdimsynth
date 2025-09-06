% Zielfunktion ("objective function") für Optimierung in der Maßsynthese
% für MRK-Kennzahl (basierend auf Segmentabständen).
% Ansatz: Je weiter die Kollisionskörper des Roboters voneinander entfernt
% sind, desto besser zur Vermeidung von Klemmungen.
% Benutzt alle Aufenthaltsorte der Gelenke des Roboters in der Trajektorie
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Structure
%   Eigenschaften der Roboterstruktur (aus cds_gen_robot_list.m)
% Traj_0
%   Endeffektor-Trajektorie (bezogen auf Basis-KS)
% Q
%   Gelenkwinkel-Trajektorie
% JP
%   Gelenkpositionen aller Gelenke des Roboters
%   (Zeilen: Zeitschritte der Trajektorie)
% 
% Ausgabe:
% fval [1x1]
%   Zielfunktionswert, der im PSO-Algorithmus minimiert wird. Benutze den
%   Kehrwert des Kollisionsabstandes
% fval_debugtext [char]
%   Zeile mit Hinweistext, der bei PSO nach Fitness-Berechnung ausgegeben wird
% debug_info [cell]
%   Zusatz-Informationen, die im Debug-Bild des Roboters angezeigt werden
% f_colldist [1x1]
%   Physikalischer Wert, der dem Zielfunktionswert zugrunde liegt
%   Hier: Geringster Abstand zweier Kollisionskörper (negativer gezählt,
%   damit bester Wert am weitesten links im Pareto-Diagramm ist).
%   Also: negativ=Keine Kollision; positiv=Kollision
% 
% Siehe auch: cds_constr_collisions_self.m, cds_obj_colldist.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval, fval_debugtext, debug_info, f_colldist] = cds_obj_mrk3(R, Set, Structure, Traj_0, Q, JP)
% Debug-Code:
if Set.general.matfile_verbosity > 2 % Debug
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_obj_mrk3.mat'));
end
% clear
% clc
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_obj_mrk3.mat'));

debug_info = '';
fval_debugtext = '';
f_colldist = NaN;
fval = 1e3;
if isempty(R.collchecks)
  % Es gibt keine Kollisionsprüfungen. Kennzahl ergibt keinen Sinn.
  return
end
%% Kollisionsabstände berechnen (für Gelenkpunkte innerhalb des Interaktionsraums)
% Interaktions-Arbeitsraum in das KS 0 rotieren
collbodies_iaspc = cds_update_interactionspace(R, Set);

% Nachverarbeitung der Gelenkpunkte. Ziehe die Plattform-Koppelgelenke
% für die Prüfung weiter in Richtung der Beinkette. Somit können trotzdem
% die Unterarme gegeneinander geprüft werden, auch wenn Abstände rund um
% die Plattform ignoriert werden.
JP_korr = NaN(size(JP));
platformcheckthreshold = 0.050;
for i = 1:size(JP,1) % Zeitschritte
  jp_i = reshape(JP(i,:), 3, size(JP,2)/3);
  i_jp = 1; % Für PKM-Basis
  for k = 1:R.NLEG % Indizes: siehe fkine_coll und weiter unten
    i_jp = i_jp + 1; % Überspringe Beinketten-Basis-KS
    i_jp = i_jp + R.Leg(k).NJ; % Letztes Gelenk-KS
    last_point = jp_i(:,i_jp); % Koordinaten des Plattform-Koppelgelenks
    % Zurückgehen bis die Koordinaten anders sind (mehrwertige Gelenke)
    for i_jp_delta = 1:3 % bis zu drei Spalten zurückgehen
      secondlast_point = jp_i(:,i_jp-i_jp_delta);
      if any(abs(last_point-secondlast_point)>1e-10), break; end
    end
    % Verschiebe die letzten Punkte der PKM in Richtung des vorletzten
    last_point_new = last_point + (secondlast_point-last_point) * ...
      1.5*platformcheckthreshold/norm(last_point-secondlast_point);
    jp_i(:,i_jp-i_jp_delta+1:i_jp) = repmat(last_point_new, 1, i_jp_delta);
  end
  i_jp = i_jp + 2; % Plattform- und EE-KS der PKM
  assert(i_jp == size(jp_i,2), 'Indizierung von jp_i ist falsch');
  JP_korr(i,:) = jp_i(:);
end
% Benutze Kollisionsprüfungen aus der Roboterklasse. Die Ersatzkörper sind
% als Kapseln von einem Gelenkpunkt zum nächsten modelliert.
[~, colldist, ~, p_coll] = check_collisionset_simplegeom_mex(R.collbodies, R.collchecks, ...
  JP_korr, struct('collsearch', false));
% Debug: Prüfe, ob die geänderten Punkte etwas bewirken (TODO: Noch unklar,
% warum die Differenz positiv und negativ ist)
% [~, colldist2, ~, p_coll2] = check_collisionset_simplegeom_mex(R.collbodies, R.collchecks, ...
%   JP, struct('collsearch', false));
% test_cd = colldist2 - colldist;
% test_cd(abs(test_cd)<1e-10) = 0;
% if any(test_cd(:) > 0)
%   warning(['Die Kollisionsabstände haben sich verringert, obwohl sich die ' ...
%     'Körper voneinander entfernt haben. Logik-Fehler.']);
% end

% Prüfe anhand der Punkte, ob der Kollisionsabstand im kritischen
% Interaktionsraum liegt
% Mittleren Punkt zwischen den beiden Punkten auf
% den beteiligten Körpern bestimmen
p_coll_mean = (p_coll(:,1:3,:) + p_coll(:,4:6,:)) / 2;
% Setze Prüfung mit den Kollisionspunkten auf. Zu jeder Abstandsprüfung aus
% R.collchecks entsteht ein Punkt, der geprüft wird
n_cb_robot = length(R.collchecks);
collbodies_iaspc2 = struct(...
  'link', uint16([ collbodies_iaspc.link; ...
    repmat((1:n_cb_robot)', 1,2) ]), ...
  'type', [ collbodies_iaspc.type; ...
    repmat(uint8(9), n_cb_robot, 1) ], ...
  'params', [ Set.task.interactionspace.params; ...
    NaN(n_cb_robot, 10) ]);
% Virtuelle Kollisionsprüfung mit den Punkten und Interaktionsraum
collchecks_iaspc2 = [];
n_cb_iaspc = size(Set.task.interactionspace.type,1);
for i = 1:n_cb_iaspc
  % Prüfe, ob in diesem Interaktionsarbeitsraum die Strukturklemmung
  % betrachtet wird
  if ~Set.task.interactionspace.check_clamp_dist, continue; end
  collchecks_iaspc2 = [collchecks_iaspc2; ...
    [n_cb_iaspc+(1:n_cb_robot)', repmat(uint8(i), n_cb_robot, 1)]]; %#ok<AGROW> 
end
coll_iaspc = true(size(p_coll,3), size(p_coll,1));
for k = 1:2
  % Punkte in Eingabeformat der Kollisionsprüfung definieren
  CP = [zeros(size(p_coll,3),3), NaN(size(p_coll,3), 3*size(p_coll,1))];
  if      k == 1, p_coll_k = p_coll(:,1:3,:);
  elseif  k == 2, p_coll_k = p_coll(:,4:6,:);
  else,           p_coll_k = p_coll_mean; % wird nicht geprüft (konvexe Körper: Mitte liegt automatisch drin)
  end
  for ll = 1:3
    tmp1 = reshape(p_coll_k(:,ll,:), size(p_coll_mean,1), size(p_coll_mean,3))';
    CP(:,3+ll:3:end) = tmp1;
  end
  % Zuweisung prüfen (Debuggen Matrixformat)
%     assert(all(p_coll_k(1,1:3,1) == CP(1,3+(1:3))));
%     assert(all(p_coll_k(2,1:3,1) == CP(1,3+(4:6))));
%     assert(all(p_coll_k(1,1:3,2) == CP(2,3+(1:3))));
  % Eigentliche Prüfung auf Überschneidung
  CollSet = struct('collsearch', false);
  coll_iaspc_k = check_collisionset_simplegeom_mex(collbodies_iaspc2, ...
    collchecks_iaspc2, CP, CollSet);
  % Alle geprüften Punkte müssen innerhalb des Interaktionsraums liegen.
  % Daher UND-Verknüpfung.
  coll_iaspc = coll_iaspc & coll_iaspc_k;
end
% Ignoriere bestimmte Kollisionsabstände, die eher konstruktive Ursachen
% haben und für MRK-Klemmabstände nicht kritisch sind.
for i = 1:size(colldist,1) % Zeitschritte
  jp_i = reshape(JP(i,:), 3, size(JP,2)/3);
  % Definiere einen Toleranzbereich in dem keine Kollisionsabstände
  % gewertet werden. Annahme: An Plattform konstruktiv vermeidbar.
  % Lege eine Kugel um jedes Plattform-Koppelgelenk
  r_c_collignore_all = NaN(3,R.NLEG+2);
  i_jp = 1; % Für PKM-Basis
  for k = 1:R.NLEG % Indizes: siehe fkine_coll
    i_jp = i_jp + 1; % Überspringe Beinketten-Basis-KS
    i_jp = i_jp + R.Leg(k).NJ; % Letztes Gelenk-KS
    r_c_collignore_all(:,k) = jp_i(:,i_jp);
  end
  % Ergänze einen Punkt für die Mitte der Plattform (und EE)
  r_c_collignore_all(:,end-1:end) = jp_i(:,end-1:end);

  i_jp = i_jp + 2; % Plattform- und EE-KS der PKM
  assert(i_jp == size(jp_i,2), 'Indizierung von jp_i ist falsch');
  for j = 1:size(colldist,2) % Kollisionsprüfungen
    % Am nächsten zueinander liegende Punkte der beiden Kollisionskörper
    r_0_C1 = p_coll(j,1:3,i)';
    r_0_C2 = p_coll(j,4:6,i)';
    % Prüfe, ob die Punkte im Interaktionsbereich liegen.
    % Wenn ja (coll_iaspc=true), dann Punkt verwerfen
    exclude_point_ij_var1 = ~coll_iaspc(i,j);
    % Prüfe, ob die Punkte in einem Toleranzbereich der Plattform liegen
    % Wenn ja, dann Kollision nicht berücksichtigen
    if ~exclude_point_ij_var1 && false
      for k = 1:R.NLEG+2
        exclude_point_ij_var2 = norm(r_c_collignore_all(:,k) - (r_0_C1+r_0_C2)/2) < platformcheckthreshold;
        if exclude_point_ij_var2
          break;
        end
      end
    else
      exclude_point_ij_var2 = false;
    end
    colldist_ijval = colldist(i,j);
    if exclude_point_ij_var1 || exclude_point_ij_var2
      colldist(i,j) = NaN; % Deaktiviere den Kollisionsabstand
    end
    % Debug-Bild
    if isnan(colldist(i,j)) && false
      fhdl = change_current_figure(869); clf; hold all %#ok<NASGU> 
      view(3); axis auto; grid on;
      xlabel('x in m');ylabel('y in m');zlabel('z in m');
      plotmode = 5; % Kollisionskörper
      if R.Type == 0 % Seriell
        s_plot = struct( 'ks', 1:R.NJ+2, 'straight', 1, 'mode', plotmode, 'nojoints', 1);
        R.plot( Q(i,:)', s_plot);
      else % PKM
        s_plot = struct( 'ks_legs', [], 'straight', 1, 'mode', plotmode, 'nojoints', 1);
        R.plot( Q(i,:)', Traj_0.X(i,:)', s_plot);
      end
      rh_W_C1 = R.T_W_0 * [r_0_C1;1];
      rh_W_C2 = R.T_W_0 * [r_0_C2;1];
      r_W_M = (rh_W_C1+rh_W_C2)/2;
      plot3(rh_W_C1(1), rh_W_C1(2), rh_W_C1(3), 'rx', 'MarkerSize', 15);
      plot3(rh_W_C2(1), rh_W_C2(2), rh_W_C2(3), 'rx', 'MarkerSize', 15);
      plot3(r_W_M(1), r_W_M(2), r_W_M(3), 'b+', 'MarkerSize', 15);
      plot3([rh_W_C1(1);rh_W_C2(1)], [rh_W_C1(2);rh_W_C2(2)], ...
        [rh_W_C1(3);rh_W_C2(3)], 'c-', 'LineWidth', 5);
      text(r_W_M(1), r_W_M(2), r_W_M(3), sprintf('%1.1fmm', ...
        colldist_ijval*1e3), 'BackgroundColor', 'y');
      % Zeichne den Toleranzbereich ein
      if exclude_point_ij_var2
        pts_W_sphere = R.T_W_0 * [r_c_collignore_all(:,k);1];
        drawSphere([pts_W_sphere(1:3)',platformcheckthreshold],'FaceColor', 'g', 'FaceAlpha', 0.3);
      end
      % Zeichne Interaktionsbereich
      for kk = 1:length(collbodies_iaspc.type)
        if collbodies_iaspc.type(kk) == 10
          params_W = Set.task.interactionspace.params(kk,:); % ist schon im Welt-KS
          q_W = eye(3,4)*[params_W(1:3)';1];
          u1_W = params_W(4:6)';
          u2_W = params_W(7:9)';
          % letzte Kante per Definition senkrecht auf anderen beiden.
          u3_W = cross(u1_W,u2_W); u3_W = u3_W/norm(u3_W)*params_W(10);
          % Umrechnen in Format der plot-Funktion
          cubpar_c = q_W(:)+(u1_W(:)+u2_W(:)+u3_W(:))/2; % Mittelpunkt des Quaders
          cubpar_l = [norm(u1_W); norm(u2_W); norm(u3_W)]; % Dimension des Quaders
          cubpar_a = 180/pi*tr2rpy([u1_W(:)/norm(u1_W), u2_W(:)/norm(u2_W), u3_W(:)/norm(u3_W)],'zyx')'; % Orientierung des Quaders
          drawCuboid([cubpar_c', cubpar_l', cubpar_a'], ...
            'FaceColor', 'm', 'FaceAlpha', 0.1);
        end
      end
      numberstring = sprintf('i=%d/%d (Traj.), j=%d/%d (Punkt)', i, size(colldist,1), j, size(colldist,2));
      if exclude_point_ij_var1
        title(sprintf('%s: Ausschluss weil nicht in Interaktionsraum', numberstring));
      elseif exclude_point_ij_var2
        title(sprintf('%s: Ausschluss weil zu nah an Plattform', numberstring));
      else
        title(sprintf('%s: Berücksichtige Punkt für Kennzahl', numberstring));
      end
      drawnow();
    end
  end % for j
end % for i

% Prüfe, welche Kollisionsabstände sich nie ändern. Wird eigentlich bereits
% in cds_dimsynth_robot geprüft. Dort kann aber noch nicht die PKM mit
% geschlossenen kinematischen Ketten geprüft werden (IK noch nicht gelöst).
colldist_range = diff(minmax2(colldist')');
I_norange = abs(colldist_range) < 1e-10;
if all(I_norange)
  % Alle Kollisionsprüfungen haben immer den gleichen Abstand. Nehme den.
  min2colldist = colldist(1);
  IIcmin = 1; IItmin = 1; % Dummy-Werte
else
  % Bestimme minimale Kollisionsabstände für alle Zeitschritte.
  % Berücksichtige nur Kollisionen, die sich auch ändern.
  [mincolldist, IIcmin] = min(colldist(:,~I_norange),[],2); % kleinster jeweils alle Zeitschritte
  [min2colldist, IItmin] = min(mincolldist); % Zeitschritt für kleinsten Abstand von allen
end

% Kennzahl berechnen: Werte von oben sind positiv, wenn keine Kollision
% vorliegt. Daher negativ zählen und unten Kehrwert bilden.
f_colldist = -min2colldist;
if min2colldist > 0 % keine Kollision
  f_colldist2 = 1/min2colldist;
  f_colldist_norm = 2/pi*atan(f_colldist2/10); % Normierung auf 0 bis 1; 1m ist 0.06; 0.1m ist 0.5
  fval = 1e2*f_colldist_norm; % Normiert auf 0 bis 1e2. über 1e2 ist reserviert für Kollision
  fval_debugtext = sprintf('Kollisionsabstand %1.1fmm.', 1e3*min2colldist);
elseif isnan(min2colldist)
  % Keine Objekt-Abstände im Interaktionsraum
  fval = 0; % Bestmöglicher Wert
  % Bestmöglichen physikalischen Wert generieren: Einmal quer durch größte
  % Ausdehnung des Robots in alle Dimensionen.
  robrange = NaN(3,1);
  for k = 1:3
    allentries = JP(:,k:3:end);
    robrange(k) = diff(minmax2(allentries(:)'));
  end
  f_colldist = -norm(robrange);
  fval_debugtext = sprintf('Keine Kollisionsabstände im Interaktionsraum');
else % Kollision
  f_colldist2 = -min2colldist; % je größer der Betrag desto tiefer in Kollision
  f_colldist_norm = 2/pi*atan(f_colldist2*10); % 0.2m Eindringung entspricht 0.7; 0.1m entspricht 0.5; 1mm entspricht 0.0064
  fval = 1e2*(1+9*f_colldist_norm); % normiere auf 1e2 bis 1e3
  fval_debugtext = sprintf('Kollisionsdurchdringung %1.1fmm.', -1e3*min2colldist);
end

if min2colldist < 0
  fval_debugtext = [fval_debugtext, ' (Kollision)'];
end
%% Debug
% Hierfür Variable `names_collbodies` manuell aus cds_dimsynth_robot holen
% for kk = 1:2
%   if kk == 1
%     I = I_norange; str = 'ohne';
%   else
%     I = ~I_norange; str = 'mit';
%   end
%   fprintf('%d/%d Kollisionsprüfungen %s Änderung der Abstände:\n', sum(I), length(I), str);
%   for i = find(I)
%     fprintf('%d. dist %1.1f...%4.1fmm. Bei t=%d: %1.1fmm: [%d %d], %s - %s\n', i, ...
%       1e3*min(colldist(:,i)), 1e3*max(colldist(:,i)), IItmin, 1e3*colldist(IItmin, i), R.collchecks(i,1), R.collchecks(i,2), ...
%       names_collbodies{R.collchecks(i,1)}, names_collbodies{R.collchecks(i,2)});
%   end
% end
% Zum Testen auch Nachrechnen mit Prüfungen aus der Maßsynthese
% if Set.general.debug_calc
%   [~, colldist_dbg] = check_collisionset_simplegeom_mex(Structure.collbodies_robot,...
%     Structure.selfcollchecks_collbodies, JP, struct('collsearch', false));
%   [mincolldist_dbg, I_mcd_dbg] = min(colldist_dbg,[],2);
%   [min2colldist_dbg, IImin_dbg] = min(mincolldist_dbg);
% end
% Debug: Bester Fall
% [mincolldist, I_mcd] = min(colldist,[],2);
% [min2colldist, IImin] = max(mincolldist);
% Set_tmp = Set;
% Set_tmp.general.plot_details_in_fitness = inf;
% [fval, coll, colldepth_abs] = cds_constr_collisions_self(R, Traj_0.X, ...
%   Set_tmp, Structure, JP, Q, [1 2]);
%% Prüfe Aktivierung des Debug-Plots
if Set.general.plot_details_in_fitness < 0 && 1e4*fval > abs(Set.general.plot_details_in_fitness) || ... % Gütefunktion ist schlechter als Schwellwert: Zeichne
   Set.general.plot_details_in_fitness > 0 && 1e4*fval < abs(Set.general.plot_details_in_fitness) % Gütefunktion ist besser als Schwellwert: Zeichne
  % Zeichnen/Debuggen (s.u.)
else
  return
end
%% Debug-Plot anzeigen
fhdl = change_current_figure(722); clf; hold all;
view(3); axis auto; hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m');

plotmode = 5; % Kollisionskörper
if R.Type == 0 % Seriell
  s_plot = struct( 'ks', [], 'straight', 1, 'mode', plotmode, 'nojoints', 1);
  R.plot( Q(IItmin,:)', s_plot);
else % PKM
  s_plot = struct( 'ks_legs', [], 'ks_platform', [], ...
    'straight', 1, 'mode', plotmode, 'nojoints', 1);
  R.plot( Q(IItmin,:)', Traj_0.X(IItmin,:)', s_plot);
end
% Plotte Kollisionsverbindungen
p_coll_min = p_coll(:,:,IItmin);
II_norange = find(I_norange);
II_range = find(~I_norange);
legtxt = {'Keine Änderung', 'Kritischer Punkt', 'Kritische Verbindung'};
leghdl = NaN(3,1);
for i = 1:size(p_coll_min,1)
  % Kollisionspunkte ins Welt-KS transformieren. Prüfung im Basis-KS.
  p1 = R.T_W_0 * [p_coll_min(i,1:3)';1];
  p2 = R.T_W_0 * [p_coll_min(i,4:6)';1];
  % Fall der Kollisionsprüfung unterscheiden
  casenum = 0; %#ok<NASGU> 
  if any(i == II_norange) % Betrifft eine Prüfung, deren Wert sich nicht ändert
    s = 'g-'; lw = 5; casenum = 1;
  elseif IIcmin(IItmin) == find(II_range == i) % Kleinster Abstand
    % Variable IIcmin ist bezogen auf reduzierte Menge der Prüfungen aus
    % II_range. Variable i bezogen auf vollständige Menge.
    s = 'r-'; lw = 5; casenum = 3;
    leghdl(2) = plot3(mean([p1(1);p1(1)]), mean([p1(2);p1(2)]), mean([p1(3);p1(3)]), 'rx', 'markersize', 30);
  else % Alle anderen Prüfungen
    s = 'k--'; lw = 0.5; %#ok<NASGU>
    continue
  end
  hdl = plot3([p1(1);p2(1)], [p1(2);p2(2)], [p1(3);p2(3)], s, 'linewidth', lw);
  if casenum > 0, leghdl(casenum) = hdl; end
end
% Zeichne Interaktionsbereich
for kk = 1:length(collbodies_iaspc.type)
  if collbodies_iaspc.type(kk) == 10
    params_W = Set.task.interactionspace.params(kk,:); % ist schon im Welt-KS
    q_W = eye(3,4)*[params_W(1:3)';1];
    u1_W = params_W(4:6)';
    u2_W = params_W(7:9)';
    % letzte Kante per Definition senkrecht auf anderen beiden.
    u3_W = cross(u1_W,u2_W); u3_W = u3_W/norm(u3_W)*params_W(10);
    % Umrechnen in Format der plot-Funktion
    cubpar_c = q_W(:)+(u1_W(:)+u2_W(:)+u3_W(:))/2; % Mittelpunkt des Quaders
    cubpar_l = [norm(u1_W); norm(u2_W); norm(u3_W)]; % Dimension des Quaders
    cubpar_a = 180/pi*tr2rpy([u1_W(:)/norm(u1_W), u2_W(:)/norm(u2_W), u3_W(:)/norm(u3_W)],'zyx')'; % Orientierung des Quaders
    h=drawCuboid([cubpar_c', cubpar_l', cubpar_a'], ...
      'FaceColor', 'm', 'FaceAlpha', 0.1);
  end
end
title(sprintf(['Kollisionsabstände schlechtester Fall. ', ...
  'Dist=%1.1fmm, I=%d/%d'], 1e3*min2colldist, IItmin, size(Q,1)));
legend(leghdl(~isnan(leghdl)), legtxt(~isnan(leghdl)));
drawnow();
[currgen,currind,currimg,resdir] = cds_get_new_figure_filenumber(Set, Structure,'ObjMRK3');
for fileext=Set.general.save_robot_details_plot_fitness_file_extensions
  if strcmp(fileext{1}, 'fig')
    saveas(fhdl, fullfile(resdir, sprintf('Gen%02d_Ind%02d_Eval%d_ObjMRK3.fig', currgen, currind, currimg)));
  else
    export_fig(fhdl, fullfile(resdir, sprintf('Gen%02d_Ind%02d_Eval%d_ObjMRK3.%s', currgen, currind, currimg, fileext{1})));
  end
end
