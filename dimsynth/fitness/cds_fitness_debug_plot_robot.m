% Plot-Funktion zum Debuggen der Fitness-Funktion in der Maßsynthese
% Zeichnet den zu optimierenden Roboter je nach eingestellten Schwellwerten
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% q
%   Gelenkwinkelstellung zum Zeichnen des Roboters
% Traj_0, Traj_W
%   Endeffektor-Trajektorie (bezogen auf Basis- und Welt-KS)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Structure
%   Eigenschaften der Roboterstruktur (aus cds_gen_robot_list.m)
% p
%   Vektor der Optimierungsvariablen für PSO
% fval
%   Fitness-Wert für den Parametervektor p.
% debug_info [cell]
%   Zusatz-Informationen, die im Debug-Bild des Roboters angezeigt werden
%   (Zeilenweise als Text)
% 
% Ausgabe:
% fighdl
%   Handle zum erzeugten Bild

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function fighdl = cds_fitness_debug_plot_robot(R, q, Traj_0, Traj_W, Set, Structure, p, fval, debug_info)
fighdl = NaN;
% Zeichne den Roboter für den aktuellen Parametersatz.
if Set.general.plot_robot_in_fitness < 0 && fval >= abs(Set.general.plot_robot_in_fitness) || ... % Gütefunktion ist schlechter als Schwellwert: Zeichne
   Set.general.plot_robot_in_fitness > 0 && fval <= abs(Set.general.plot_robot_in_fitness) % Gütefunktion ist besser als Schwellwert: Zeichne
  % Zeichnen fortsetzen
else 
  return
end
if isempty(q)
  q = zeros(R.NJ,1);
elseif any(isnan(q))
  q(isnan(q)) = 0;
end
if isempty(Traj_0)
  Traj_0 = cds_transform_traj(R, Traj_W);
end
tt = '';
for i = 1:length(debug_info), tt = [tt, newline(), debug_info{i}]; end %#ok<AGROW>

fighdl = change_current_figure(200); clf; hold all;
view(3);
axis auto
hold on;grid on;
xlabel('x in m');ylabel('y in m');zlabel('z in m');
if ~isempty(Traj_W) && isfield(Traj_W, 'X')
  plot3(Traj_W.X(:,1), Traj_W.X(:,2),Traj_W.X(:,3), 'k-');
end
if fval > 1e9 % keine vernünftigen Parametergrenzen ableitbar
  plotmode = 1; % Strichzeichnung
else
  plotmode = 4; % Entwurfsparameter
end
if R.Type == 0 % Seriell
  s_plot = struct( 'ks', 1:R.NJ+2, 'straight', 1, 'mode', plotmode);
  R.plot( q, s_plot);
else % PKM
  s_plot = struct( 'ks_legs', [R.I1L_LEG; R.I2L_LEG], 'ks_platform', 1:6, ...
    'straight', 1, 'mode', plotmode);
  R.plot( q, Traj_0.XE(1,:)', s_plot);
end
if ~isempty(p)
  title(sprintf('fval=%1.2e; p=[%s]; %s', fval,disp_array(p','%1.3f'), tt));
end
axis auto
[currgen,currind,currimg,resdir] = cds_get_new_figure_filenumber(Set, Structure,'Details');
if ~isempty(Set.general.save_robot_details_plot_fitness_file_extensions)
  for fileext=Set.general.save_robot_details_plot_fitness_file_extensions
    if strcmp(fileext{1}, 'fig')
      saveas(fighdl, fullfile(resdir, sprintf('Gen%02d_Ind%02d_Eval%d_Details.fig', currgen, currind, currimg)));
    else
      export_fig(fighdl, fullfile(resdir, sprintf('Gen%02d_Ind%02d_Eval%d_Details.%s', currgen, currind, currimg, fileext{1})));
    end
  end
end
drawnow();

