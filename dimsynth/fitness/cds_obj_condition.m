% Zielfunktion ("objective function") für Optimierung in der Maßsynthese
% basierend auf der Konditionszahl des Roboters.
% Die Konditionszahl wird in einen normierten Zielfunktionswert übersetzt
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Structure
%   Eigenschaften der Roboterstruktur (aus cds_gen_robot_list.m)
% Jinvges
%   Zeilenweise (inverse) Jacobi-Matrizen des Roboters (für PKM). Bezogen
%   auf vollständige Gelenkgeschwindigkeiten und EE-Geschw.
% Traj_0
%   Endeffektor-Trajektorie (bezogen auf Basis-KS)
% Q, QD
%   Gelenkpositionen und -geschwindigkeiten des Roboters (für PKM auch
%   passive Gelenke)
% 
% Ausgabe:
% fval [1x1]
%   Zielfunktionswert, der im PSO-Algorithmus minimiert wird
% fval_debugtext [char]
%   Zeile mit Hinweistext, der bei PSO nach Fitness-Berechnung ausgegeben wird
% debug_info [cell]
%   Zusatz-Informationen, die im Debug-Bild des Roboters angezeigt werden

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [fval, fval_debugtext, debug_info] = cds_obj_condition(R, Set, Structure, Jinvges, Traj_0, Q, QD)
debug_info = {};

% Berechne Konditionszahl über Trajektorie
Cges = NaN(length(Traj_0.t), 1);
if R.Type == 0
  % Berechne Konditionszahl für alle Punkte der Bahn
  for i = 1:length(Traj_0.t)
    J_3T3R = R.jacobig(Q(i,:)');
    J_task = J_3T3R(Set.structures.DoF,:);
    Cges(i,:) = cond(J_task);
  end
else
  % Berechne Konditionszahl für alle Punkte der Bahn
  for i = 1:length(Traj_0.t)
    Jinv_IK = reshape(Jinvges(i,:), R.NJ, sum(R.I_EE));
    Cges(i) = cond(Jinv_IK(R.I_qa,:));
  end
end
% Schlechtester Wert der Konditionszahl ist Kennzahl
% Nehme Logarithmus, da Konditionszahl oft sehr groß ist.
f_cond1 = max(Cges);
f_cond = log(f_cond1); 
f_cond_norm = 2/pi*atan((f_cond)/20); % Normierung auf 0 bis 1; 150 ist 0.9
fval = 1e3*f_cond_norm; % Normiert auf 0 bis 1e3
fval_debugtext = sprintf('Schlechteste Konditionszahl %1.3e', f_cond1);

%% Debug-Plot (nur für PKM)
if R.Type == 2
  if fval < Set.general.plot_details_in_fitness
    % Debug-Werte berechnen
    det_ges = NaN(size(Jinvges,1), 1);
    for i = 1:size(Jinvges,1)
      Jinv_IK = reshape(Jinvges(i,:), R.NJ, sum(R.I_EE));
      if Set.general.debug_calc
        % Debug: Vergleich der Jacobi-Matrizen (falls keine Singularität
        % auftritt). TODO: Nur Optional mit Debug-Schalter
        test_Jinv = Jinv_IK(R.I_qa,:) - R.jacobi_qa_x(Q(i,:)',Traj_0.X(i,:)');
        if any(abs(test_Jinv(:)) > 1e-6)%  && Cges(i) < 1e10
          if Set.general.matfile_verbosity > 0
            save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_fitness_par3.mat'));
          end
          error('Jacobi numerisch vs. symbolisch stimmt nicht. Fehler %e, Kondition Jinv %e', ...
            max(abs(abs(test_Jinv(:)))), Cges(i));
        end
      end
      det_ges(i,:) = det(Jinv_IK(R.I_qa,:));
    end

    if Set.general.plot_robot_in_fitness < 0 && fval > abs(Set.general.plot_robot_in_fitness) || ... % Gütefunktion ist schlechter als Schwellwert: Zeichne
       Set.general.plot_robot_in_fitness > 0 && fval < abs(Set.general.plot_robot_in_fitness) % Gütefunktion ist besser als Schwellwert: Zeichne
      change_current_figure(201); clf;
      if ~strcmp(get(201, 'windowstyle'), 'docked')
        % set(201,'units','normalized','outerposition',[0 0 1 1]);
      end
      sgtitle('Auswertung Jacobi-Matrizen')
      subplot(2,3,1);
      plot(Traj_0.t, Cges); hold on;
      ylabel('Konditionszahl'); grid on;
      subplot(2,3,2);
      plot(Traj_0.t, Traj_0.XD); hold on;
      ylabel('EE-Geschw.'); grid on;
      legend({'r_x', 'r_y', 'r_z', '\phi_1', '\phi_2', '\phi_3'});
      subplot(2,3,3);
      plot(Traj_0.t, QD(:,R.I_qa)); hold on;
      ylabel('Gelenk-Geschw. (aktiv)'); grid on;
      subplot(2,3,4);
      plot(Traj_0.t, QD(:,~R.I_qa)); hold on;
      ylabel('Gelenk-Geschw. (passiv)'); grid on;
      subplot(2,3,5);
      plot(Traj_0.t, det_ges); hold on;
      ylabel('Determinanten'); grid on;
      legend({'Jinv (x->qa)'}); % 'A (dh/dx; DirKin)', 'B (dh/dq; InvKin)', 
      subplot(2,3,6);
      plot(Traj_0.t, log(abs(det_ges))); hold on;
      ylabel('Log |Determinanten|'); grid on;
      legend({'Jinv (x->qa)'}); % 'A (dh/dx; DirKin)', 'B (dh/dq; InvKin)', 
      linkxaxes
      [currgen,currimg,resdir] = cds_get_new_figure_filenumber(Set, Structure,'ParRobJacobian');
      for fileext=Set.general.save_robot_details_plot_fitness_file_extensions
        if strcmp(fileext{1}, 'fig')
          saveas(201, fullfile(resdir, sprintf('PSO_Gen%02d_FitEval%03d_ParRobJacobian.fig', currgen, currimg)));
        else
          export_fig(201, fullfile(resdir, sprintf('PSO_Gen%02d_FitEval%03d_ParRobJacobian.%s', currgen, currimg, fileext{1})));
        end
      end
    end
  end
end