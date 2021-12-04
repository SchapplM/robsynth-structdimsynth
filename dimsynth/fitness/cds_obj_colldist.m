% Zielfunktion ("objective function") für Optimierung in der Maßsynthese
% basierend auf Kollisionsabständen des Roboters.
% Ansatz: Je weiter die Kollisionskörper des Roboters voneinander entfernt
% sind, desto unwahrscheinlicher ist eine Selbstkollision auch bei Parameteränderung.
% Benutzt alle Aufenthaltsorte der Gelenke des Roboters in der Trajektorie
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
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
%   Zielfunktionswert, der im PSO-Algorithmus minimiert wird
% fval_debugtext [char]
%   Zeile mit Hinweistext, der bei PSO nach Fitness-Berechnung ausgegeben wird
% debug_info [cell]
%   Zusatz-Informationen, die im Debug-Bild des Roboters angezeigt werden
% f_colldist [1x1]
%   Physikalischer Wert, der dem Zielfunktionswert zugrunde liegt
%   Hier: Geringster Abstand zweier Kollisionskörper
% 
% Siehe auch: cds_constr_collisions_self.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-11
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval, fval_debugtext, debug_info, f_colldist] = cds_obj_colldist(R, Set, Structure, Traj_0, Q, JP)

debug_info = '';
fval_debugtext = '';
f_colldist = NaN;
fval = 1e3;
if isempty(R.collchecks)
  % Es gibt keine Kollisionsprüfungen. Kennzahl ergibt keinen Sinn.
  return
end
%% Kollisionsabstände berechnen
% Benutze Kollisionsprüfungen aus der Roboterklasse
[~, colldist, ~, p_coll] = check_collisionset_simplegeom_mex(R.collbodies, R.collchecks, ...
  JP, struct('collsearch', false));
% Prüfe, welche Kollisionsabstände sich nie ändern. Wird eigentlich bereits
% in cds_dimsynth_robot geprüft. Dort kann aber noch nicht die PKM mit
% geschlossenen kinematischen Ketten geprüft werden (IK noch nicht gelöst).
colldist_range = diff(minmax2(colldist')');
I_norange = abs(colldist_range) < 1e-10;

% Bestimme minimale Kollisionsabstände für alle Zeitschritte.
% Berücksichtige nur Kollisionen, die sich auch ändern.
[mincolldist, IIcmin] = min(colldist(:,~I_norange),[],2); % kleinster jeweils alle Zeitschritte
[min2colldist, IItmin] = min(mincolldist); % Zeitschritt für kleinsten Abstand von allen

% Kennzahl berechnen
f_colldist = min2colldist;
f_colldist_norm = 2/pi*atan((f_colldist)/0.1); % Normierung auf 0 bis 1; 1m ist 0.94
fval = 1e3*f_colldist_norm; % Normiert auf 0 bis 1e3
fval_debugtext = sprintf('Kollisionsabstand %1.1fmm.', 1e3*min2colldist);

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

%% Debug-Plot
if Set.general.plot_robot_in_fitness < 0 && 1e4*fval > abs(Set.general.plot_robot_in_fitness) || ... % Gütefunktion ist schlechter als Schwellwert: Zeichne
   Set.general.plot_robot_in_fitness > 0 && 1e4*fval < abs(Set.general.plot_robot_in_fitness) % Gütefunktion ist besser als Schwellwert: Zeichne
  % Zeichnen/Debuggen (s.u.)
else
  return
end
change_current_figure(722); clf; hold all;
if ~strcmp(get(722, 'windowstyle'), 'docked')
  % set(722,'units','normalized','outerposition',[0 0 1 1]);
end
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
for i = 1:size(p_coll_min,1)
  % Kollisionspunkte ins Welt-KS transformieren. Prüfung im Basis-KS.
  p1 = R.T_W_0 * [p_coll_min(i,1:3)';1];
  p2 = R.T_W_0 * [p_coll_min(i,4:6)';1];
  % Fall der Kollisionsprüfung unterscheiden
  if any(i == II_norange) % Betrifft eine Prüfung, deren Wert sich nicht ändert
    s = 'g-'; lw = 5;
  elseif IIcmin(IItmin) == find(II_range == i) % Kleinster Abstand
    % Variable IIcmin ist bezogen auf reduzierte Menge der Prüfungen aus
    % II_range. Variable i bezogen auf vollständige Menge.
    s = 'r-'; lw = 5;
    plot3(mean([p1(1);p1(1)]), mean([p1(2);p1(2)]), mean([p1(3);p1(3)]), 'rx', 'markersize', 30);
  else % Alle anderen Prüfungen
    s = 'k--'; lw = 0.5; %#ok<NASGU>
    continue
  end
  plot3([p1(1);p2(1)], [p1(2);p2(2)], [p1(3);p2(3)], s, 'linewidth', lw)
end
title(sprintf(['Kollisionsabstände schlechtester Fall. ', ...
  'Dist=%1.1fmm, I=%d/%d'], 1e3*min2colldist, IItmin, size(Q,1)));
drawnow();
[currgen,currind,currimg,resdir] = cds_get_new_figure_filenumber(Set, Structure,'ObjInstallspace');
for fileext=Set.general.save_robot_details_plot_fitness_file_extensions
  if strcmp(fileext{1}, 'fig')
    saveas(722, fullfile(resdir, sprintf('Gen%02d_Ind%02d_Eval%d_ObjCollDist.fig', currgen, currind, currimg)));
  else
    export_fig(722, fullfile(resdir, sprintf('Gen%02d_Ind%02d_Eval%d_ObjCollDist.%s', currgen, currind, currimg, fileext{1})));
  end
end
