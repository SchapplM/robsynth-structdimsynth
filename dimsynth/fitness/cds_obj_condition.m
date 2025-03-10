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
% f_cond1 [1x1]
%   Physikalischer Wert, der dem Zielfunktionswert zugrunde liegt
%   Hier: Maximale Konditionszahl der Jacobi-Matrix

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [fval, fval_debugtext, debug_info, f_cond1] = cds_obj_condition(R, Set, Structure, Jinvges, Traj_0, Q, QD)
debug_info = {''};

% Berechne Konditionszahl über Trajektorie
Cges = NaN(size(Q,1), 1);
if any(R.Type == [0 1])
  % Berechne Konditionszahl für alle Punkte der Bahn
  for i = 1:size(Q,1)
    % Nehme die analytische Jacobi-Matrix (für 3T2R wichtig, da dort die
    % ersten fünf Zeilen der Aufgabe entsprechen. Bei jacobig falsch).
    J_3T3R = R.jacobia(Q(i,:)');
    J_task = J_3T3R(Set.task.DoF,:);
    Cges(i,:) = cond(J_task);
  end
else
  % Berechne Konditionszahl für alle Punkte der Bahn
  for i = 1:size(Q,1)
    Jinv_IK = reshape(Jinvges(i,:), R.NJ, sum(R.I_EE));
    Cges(i) = cond(Jinv_IK(R.I_qa,:));
  end
end
if Set.optimization.constraint_obj(4)
  debug_info = {sprintf('Konditionszahl-Grenzwert %d/%d mal überschritten', ...
    sum(Cges>Set.optimization.constraint_obj(4)), length(Cges))};
end
% Schlechtester Wert der Konditionszahl ist Kennzahl
% Nehme Logarithmus, da Konditionszahl oft sehr groß ist.
f_cond1 = max(Cges);
f_cond = log(f_cond1); 
f_cond_norm = 2/pi*atan((f_cond)/20); % Normierung auf 0 bis 1; 150 ist 0.9
fval = 1e3*f_cond_norm; % Normiert auf 0 bis 1e3
fval_debugtext = sprintf('Schlechteste Konditionszahl %1.3e.', f_cond1);

%% Debug-Plot
if Set.general.plot_details_in_fitness < 0 && 1e4*fval > abs(Set.general.plot_details_in_fitness) || ... % Gütefunktion ist schlechter als Schwellwert: Zeichne
   Set.general.plot_details_in_fitness > 0 && 1e4*fval < abs(Set.general.plot_details_in_fitness) % Gütefunktion ist besser als Schwellwert: Zeichne
  % Zeichnen/Debuggen (s.u.)
else
  return
end
if any(R.Type == [0 1]) % Seriell
  change_current_figure(201); clf;
  sgtitle('Auswertung Jacobi-Matrizen')
  subplot(2,2,1);
  plot(Traj_0.t, Cges); hold on;
  ylabel('Konditionszahl'); grid on;
  subplot(2,2,2);
  plot(Traj_0.t, Traj_0.XD); hold on;
  ylabel('EE-Geschw.'); grid on;
  legend({'r_x', 'r_y', 'r_z', '\phi_1', '\phi_2', '\phi_3'});
  subplot(2,1,2);
  plot(Traj_0.t, QD); hold on;
  ylabel('Gelenk-Geschw.'); grid on;
else % PKM
  % Debug-Werte berechnen
  det_ges = NaN(size(Jinvges,1), 1);
  for i = 1:size(Jinvges,1)
    Jinv_IK = reshape(Jinvges(i,:), R.NJ, sum(R.I_EE));
    if Set.general.debug_calc
      % Debug: Vergleich der Jacobi-Matrizen
      test_Jinv = Jinv_IK(R.I_qa,:) - R.jacobi_qa_x(Q(i,:)',Traj_0.X(i,:)');
      if any(abs(test_Jinv(:)) > 1e-6)%  && Cges(i) < 1e10
        if Set.general.matfile_verbosity > 0
          save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_fitness_par3.mat'));
        end
        error('Jacobi numerisch vs. symbolisch stimmt nicht. Fehler %e, Kondition Jinv %e', ...
          max(abs(abs(test_Jinv(:)))), Cges(i));
      end
      det_ges(i,:) = det(Jinv_IK(R.I_qa,:));
      % Debug: Vergleich der Geschwindigkeiten
      test_qaD = QD(i,R.I_qa)' - Jinv_IK(R.I_qa,:)*Traj_0.XD(i,R.I_EE)';
      test_qD = QD(i,:)' - Jinv_IK(:,:)*Traj_0.XD(i,R.I_EE)';
      test_xD = Jinv_IK(R.I_qa,:) \ QD(i,R.I_qa)' - Traj_0.XD(i,R.I_EE)';
      assert(all(abs(test_qaD) < 1e-10), 'Umrechnung xD->qaD falsch');
      assert(all(abs(test_qD) < 1e-10), 'Umrechnung xD->qD falsch');
      assert(all(abs(test_xD) < 1e-10), 'Umrechnung qD->xD falsch');
    end
  end
  fhdl = change_current_figure(201); clf;
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
  [currgen,currind,currimg,resdir] = cds_get_new_figure_filenumber(Set, Structure,'ParRobJacobian');
  for fileext=Set.general.save_robot_details_plot_fitness_file_extensions
    if strcmp(fileext{1}, 'fig')
      saveas(fhdl, fullfile(resdir, sprintf('Gen%02d_Ind%02d_Eval%d_ParRobJacobian.fig', currgen, currind, currimg)));
    else
      export_fig(fhdl, fullfile(resdir, sprintf('Gen%02d_Ind%02d_Eval%d_ParRobJacobian.%s', currgen, currind, currimg, fileext{1})));
    end
  end
end
