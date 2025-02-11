% Einstellungen für Maßsynthese für 3T1R Seriellhybrid-Roboter
% Starte mehrere Optimierungen in aufsteigender Komplexität zur
% Demonstration, dass die Maßsynthese damit funktioniert.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2024-10
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
if isempty(which('hybroblib_path_init.m'))
  warning('Datenbank mit seriell-hybriden Robotern ist nicht im Pfad. Beispiel nicht ausführbar.');
  return
end
for robnum = 1:2 % für zwei verschiedene Roboter zeigen
  %% Allgemeine Einstellungen
  % Aufgaben-FG
  DoF = [1 1 1 0 0 1];
  Traj_no = 1;
  Set_default = cds_settings_defaults(struct('DoF', DoF));
  Set = Set_default;
  Set.task.Ts = 1e-2;
  Set.task.Tv = 1e-1;
  Traj = cds_gen_traj(DoF, Traj_no, Set.task);
  Set.optimization.objective = 'mass';
  Set.optimization.NumIndividuals = 5;
  Set.optimization.MaxIter = 10;
  Set.structures.use_serialhybrid = true;
  Set.structures.use_serial = false;
  Set.structures.use_parallel = false;
  % Set.general.plot_details_in_fitness = 1e3;
  % Set.general.plot_robot_in_fitness = 1e3;
  Set.general.verbosity = 3;
  
  if robnum == 1 % Palettierer mit zwei Parallelogrammen
    Set.structures.whitelist = {'palh3m2_palh3m2KR1'};
  else % Palettierer mit drei Parallelogrammen (aktuell noch kein "echtes" Modell)
    Set.structures.whitelist = {'palh1m2_palh1m2Bsp1'};
  end
  Set.optimization.fix_joint_limits = true; % Es sind plausible Grenzen in der Datenbank
  %% Test 1: Kuka-Palettierroboter Optimierung (nur z-Rotation Basis/EE)
  % Sehr einfache Optimierung zweier quasi wirkungsloser Parameter (einzig im
  % Bezug auf Gelenkgrenzen) mit einer gegebenen Roboterkinematik
  Set.optimization.optname = sprintf('3T1R_SerHyRob%d_Test1', robnum);
  
  % Nur Aufstellort optimieren
  % Mittelstellung des Endeffektors im Arbeitsraum des Roboters (Basis-KS)
  if robnum == 1
    xE_mean = [2.1367; 0; 0.8818];
  else
    xE_mean = [1.9123; 0; 0.6521];
  end
  if false
    % Berechnung mit Robotermodell und gespeicherten Gelenkgrenzen
    if robnum == 1
      RS_TE = hybroblib_create_robot_class('palh3m2', 'TE', 'palh3m2KR1');
    else
      RS_TE = hybroblib_create_robot_class('palh1m2', 'TE', 'palh1m2Bsp1');
    end
    q_mean = mean(RS_TE.qlim,2);
    xE_mean = RS_TE.t2x(RS_TE.fkineEE(q_mean));
    figure(5);clf;hold on;grid on;
    xlabel('x [m]');ylabel('y [m]');zlabel('z [m]');view(3);
    s_plot = struct( 'ks', [1:RS_TE.NJ, RS_TE.NJ+2], 'mode', 1);
    RS_TE.plot( q_mean, s_plot );
  end
  xT_range = minmax2(Traj.XE(:,1:3)');
  xT_mean = mean(xT_range,2);
  % Verschiebe die Beispiel-Aufgabe so, dass ihre Mitte mit der Mitte der
  % Gelenk-Mittelstellung übereinstimmt. 
  % Der Roboter müsste dann optimal zur Aufgabe stehen.
  x_delta = -xE_mean(1:3)+xT_mean;
  Set.optimization.basepos_limits = repmat(x_delta,1,2)+repmat([-0.100, 0.100],3,1);
  % Keine EE-Transformation optimieren
  Set.optimization.ee_translation = false;
  % Rotation von Basis und EE. Ist nur Offset, da dort direkt auch ein
  % senkrechtes Drehgelenk sitzt. Aufgrund der Gelenkgrenzen evtl. sinnvoll.
  Set.optimization.ee_rotation = true;
  Set.optimization.rotate_base = true;
  cds_start(Set,Traj);
  ResTab = cds_load_results_table(Set);
  % Vorher ausprobiert: Muss funktionieren (Arbeitsraum wurde ja eingestellt).
  assert(ResTab.Fval_Opt(1)<1e3, 'Kein Erfolg in Optimierung. Sollte nicht sein');
  %% Test 2: Kollisionserkennung aktivieren (sonst alles wie oben)
  Set.optimization.optname = sprintf('3T1R_SerHyRob%d_Test2', robnum);
  Set.optimization.constraint_collisions = true;
  Set.general.animation_styles = {}; % Kein Video mehr erzeugen (identisch zu vorher)
  cds_start(Set,Traj);
  ResTab = cds_load_results_table(Set);
  % Vorher ausprobiert: Muss funktionieren.
  assert(ResTab.Fval_Opt(1)<1e3, 'Kein Erfolg in Optimierung. Sollte nicht sein');
  %% Test 3: Bauraumprüfung und sehr freie Positionierung
  Set.optimization.optname = sprintf('3T1R_SerHyRob%d_Test3', robnum);
  % Roboter nur auf einer Seite erlauben
  Set.optimization.basepos_limits = [-3, 3; -3, 3; -1.5, 1];
  % Set.general.plot_details_in_fitness = 3e9; % Debuggen: Zeige die Bauraumverletzung
  Set.optimization.NumIndividuals = 50; % Benötigt sehr viele Wiederholungen
  Set.optimization.MaxIter = 20;
  Set.optimization.obj_limit = 1e3; % Bei erstem i.O.-Ergebnis aufhören
  Set.task.installspace = struct( ...
    'type', uint8(2), ...
    'params', [[1,1,-2], [1,1,1.5], 2.7, NaN(1,3)], ... % Zylinder
    'links', {{1:12}});  % Betrifft alle Segmente
  cds_start(Set,Traj);
  ResTab = cds_load_results_table(Set);
  % Vorher ausprobiert: Muss funktionieren.
  assert(ResTab.Fval_Opt(1)<1e3, 'Kein Erfolg in Optimierung. Sollte nicht sein');
  %% Test 4: Kinematik-Parameter mit optimieren
  Set.optimization.optname = sprintf('3T1R_SerHyRob%d_Test4', robnum);
  % (nehme die Einstellungen von oben, die Kinematik-Parameter aus der
  % Datenbank über cds_gen_init_pop als Startwert genommen werden können).
  if robnum == 1 % Palettierer mit drei Parallelogrammen (allgemeines Modell)
    Set.structures.whitelist = {'palh3m2'};
  else % Palettierer mit zwei Parallelogrammen (allgemeines Modell)
    Set.structures.whitelist = {'palh1m2'};
  end
  Set.optimization.fix_joint_limits = false;
  % Set.general.plot_robot_in_fitness = 1e3; % Zum Debuggen
  Set.general.plot_details_in_fitness = 0;
  Set.optimization.obj_limit = 0; % wieder zurücksetzen, nachdem es oben gesetzt war
  Set.general.animation_styles = Set_default.general.animation_styles; % zurücksetzen
  cds_start(Set,Traj);
  ResTab = cds_load_results_table(Set);
  % Da die vorherigen Parameter geladen werden und als Startwerte
  % funktionieren müssen, muss es ein erfolgreiches Ergebnis geben
  assert(ResTab.Fval_Opt(1)<1e3, 'Kein Erfolg in Optimierung. Sollte nicht sein');
end