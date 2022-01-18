% Zielfunktion ("objective function") für Optimierung in der Maßsynthese
% basierend auf Volumen des Bauraums des Roboters (insbes. PKM).
% Ansatz: Je weniger Bauraum (Volumen) der Roboter einnimmt, desto besser.
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
% JP_in (Optional)
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
% f_instspc [1x1]
%   Physikalischer Wert, der dem Zielfunktionswert zugrunde liegt
%   Hier: Volumen der konvexen Hülle aller Punkte
% 
% Siehe auch: cds_obj_footprint.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval, fval_debugtext, debug_info, f_instspc] = cds_obj_installspace(R, Set, Structure, Traj_0, Q, JP_in)

debug_info = '';
fval_debugtext = '';
f_instspc = NaN;
fval = 1e3;
%% Gelenkpositionen berechnen
if nargin < 6 || isempty(JP_in) || Set.general.debug_calc
  if R.Type == 0
    JP = NaN(size(Q,1), 3*(1+R.NJ));
  else
    JP = NaN(size(Q,1), 3*(1+R.NJ+R.NLEG+1));
  end
  for i = 1:size(Q,1)
    if R.Type == 0
      Tc = R.fkine(Q(i,:)');
      JointPos_all_i_fromdirkin = squeeze(Tc(1:3,4,1:end));
    else
      Tc_stack_PKM = R.fkine_coll2(Q(i,:)');
      JointPos_all_i_fromdirkin = reshape(Tc_stack_PKM(:,4),3,size(Tc_stack_PKM,1)/3);
    end
    JP(i,:) = JointPos_all_i_fromdirkin(:);
  end
  if Set.general.debug_calc && nargin == 6 && ~isempty(JP_in)
    % Prüfe, ob die neu berechneten Gelenkpositionen korrekt sind
    for i = 1:size(Q,1)
      JointPos_all_i_fromdirkin = reshape(JP(i,:)',   3,size(JP,2)/3);
      JointPos_all_i_frominvkin = reshape(JP_in(i,:)',3,size(JP_in,2)/3);
      test_JP = JointPos_all_i_fromdirkin - JointPos_all_i_frominvkin;
      if any(abs(test_JP(:)) > 1e-6) || any(isnan(test_JP(:)))
        if Set.general.matfile_verbosity > 0
          save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_trajjointpos_error_debug.mat'));
        end
        error(['Ausgegebene Gelenkpositionen stimmen nicht gegen direkte ', ...
          'Kinematik. Zeitpunkt %d. Max Fehler %1.1e.'], i, max(abs(test_JP(:))));
      end
    end
  end
else
  JP = JP_in;
end
% Gelenkpositionen ins Welt-KS umrechnen (aktuell im Basis-KS).
% Nur für Plot wichtig, nicht für berechnetes Volumen.
for i = 1:size(JP,1)
  for j = 1:size(JP,2)/3
    JP(i, j*3-2:j*3) = eye(3,4)*R.T_W_0*[JP(i, j*3-2:j*3)';1];
  end
end

if any(isnan(JP(:))) || any(isinf(JP(:)))
  save(fullfile(Set.optimization.resdir, Set.optimization.optname, 'tmp', ...
    sprintf('%d_%s', Structure.Number, Structure.Name), 'cds_obj_installspace_infnan.mat'));
  fval_debugtext = 'Ungültige Gelenkpositionen. Kein Bauraum bestimmbar.';
  return
end
%% AlphaShape für die Punkte erzeugen
% Bestimme die xyz-Koordinaten aller Gelenkpunkte (ohne die Basis)
x = JP(:,3+(1:3:end-3));
y = JP(:,3+(2:3:end-3));
z = JP(:,3+(3:3:end-3));
xyz = unique([x(:) y(:) z(:)], 'rows');
% Alpha-Shape berechnen. Benutze nur die ganz außen liegenden Punkte um
% einen konvexen Polyeder zu erzeugen (alpha=inf)
shp = alphaShape(xyz, inf);
% Volumen berechnen
f_instspc = shp.volume();
% Kennzahl berechnen
f_instspc_norm = 2/pi*atan((f_instspc)/0.1); % Normierung auf 0 bis 1; 1m³ ist 0.9
fval = 1e3*f_instspc_norm; % Normiert auf 0 bis 1e3
fval_debugtext = sprintf('Bauraum %1.4fm³.', f_instspc);

%% Debug-Plot
if Set.general.plot_robot_in_fitness < 0 && 1e4*fval > abs(Set.general.plot_robot_in_fitness) || ... % Gütefunktion ist schlechter als Schwellwert: Zeichne
   Set.general.plot_robot_in_fitness > 0 && 1e4*fval < abs(Set.general.plot_robot_in_fitness) % Gütefunktion ist besser als Schwellwert: Zeichne
  % Zeichnen/Debuggen (s.u.)
else
  return
end
change_current_figure(700); clf; hold all;
view(3); axis auto; hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
% Gelenkpunkte zeichnen
plot3(xyz(:,1), xyz(:,2), xyz(:,3), 'kx');
plotmode = 1; % Strichzeichnung
if R.Type == 0 % Seriell
  s_plot = struct( 'ks', 1:R.NJ+2, 'straight', 1, 'mode', plotmode);
  R.plot( Q(1,:)', s_plot);
else % PKM
  s_plot = struct( 'ks_legs', [R.I1L_LEG; R.I2L_LEG], 'ks_platform', 1:6, ...
    'straight', 1, 'mode', plotmode);
  R.plot( Q(1,:)', Traj_0.X(1,:)', s_plot);
end
shp.plot('FaceAlpha', 0.3, 'EdgeAlpha', 0);
title(sprintf('AlphaShape of Joint Positions over Trajectory. V=%1.1fm³, A=%1.1fm²', shp.volume(), shp.surfaceArea()));
drawnow();
[currgen,currind,currimg,resdir] = cds_get_new_figure_filenumber(Set, Structure,'ObjInstallspace');
for fileext=Set.general.save_robot_details_plot_fitness_file_extensions
  if strcmp(fileext{1}, 'fig')
    saveas(700, fullfile(resdir, sprintf('Gen%02d_Ind%02d_Eval%d_ObjInstallspace.fig', currgen, currind, currimg)));
  else
    export_fig(700, fullfile(resdir, sprintf('Gen%02d_Ind%02d_Eval%d_ObjInstallspace.%s', currgen, currind, currimg, fileext{1})));
  end
end