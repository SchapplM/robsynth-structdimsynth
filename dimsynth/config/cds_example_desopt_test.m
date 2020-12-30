% Einstellungen für komb. Struktur und Maßsynthese für Beispiel der
% Entwurfsoptimierung mit verschiedenen Einstellungs-Kombinationen
% Damit Test, ob Modus funktioniert.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-01
% (C) Institut für Mechatronische Systeme, Universität Hannover

clc
clear

%% Einstellungen für alle Fälle setzen
% Aufgaben-FG
DoF = [1 1 1 1 1 1];
Traj_no = 1;

Set = cds_settings_defaults(struct('DoF', DoF));
Set.task.Ts = 1e-2;
Set.task.Tv = 1e-1;
Set.task.profile = 1; % Vollständige Trajektorie

Traj = cds_gen_traj(DoF, Traj_no, Set.task);
Set.optimization.objective = 'energy';
Set.optimization.constraint_obj(1) = 100; % max. 100kg
Set.optimization.NumIndividuals = 20;
Set.optimization.MaxIter = 10;
Set.optimization.ee_rotation = false;
Set.optimization.movebase = false;
Set.optimization.safety_link_yieldstrength = 1.2; % 20% Sicherheitsaufschlag
Set.optimization.max_range_active_revolute = 2*pi;
Set.optimization.obj_limit = 1e3; % Bei erstem funktionierenden Ergebnis aufhören (dauert sonst ewig)
Set.general.plot_details_in_fitness = 0*1e3;
Set.general.plot_robot_in_fitness = 0*1e7;
Set.general.max_retry_bestfitness_reconstruction = 1;
Set.general.verbosity = 4;
Set.general.matfile_verbosity = 2; % Modus 3/4 nur zum Debuggen bei Fehler. Dauert zu lange.
Set.general.nosummary = true;
Set.structures.whitelist = {'S6RRPRRR14', 'P6RRPRRR14V3G1P4A1'};

%% Detail-Einstellungen für Fälle vornehmen und Optimierung starten
for dbc = [false, true]
  % Die Debug-Berechnungen beeinflussen die Anzahl der berechneten Terme
  % stark. Mit und ohne durchführen, um als Fehlereinfluss auszuschließen
  Set.general.debug_calc = dbc;
  for i = 1:3
    % Verschiedene Fälle durchgehen und Optimierung damit starten
    switch i
      case 1 % Berechnung von Schnittkräften notwendig
        % Zielfunktion Energie
        Set.optimization.objective = 'energy';
        Set.optimization.constraint_obj(1) = 100; % max. 100kg
        % Nebenbedingung Materialbelastung (bedingt Schnittkraftberechnung)
        Set.optimization.constraint_obj(6) = 1;
      case 2 % Keine Berechnung der Schnittkräfte, nur Dynamik in Entwurfsoptimierung
        Set.optimization.objective = 'energy';
        Set.optimization.constraint_obj(1) = 100; % max. 100kg
        Set.optimization.constraint_obj(6) = 0; % Keine Materialspannung
      case 3 % Berechnung der Schnittkräfte, aber keine Dynamik notwendig
        Set.optimization.objective = 'mass';
        Set.optimization.constraint_obj(:) = 0; % keine Nebenbedingung ...
        Set.optimization.constraint_obj(6) = 1; % ... außer Materialspannung
    end
    for k = 1:3
      switch k
        case 1
          % Stärke der Segmente optimieren
          Set.optimization.desopt_vars = {'linkstrength'};
          Set.optimization.joint_stiffness_passive_revolute = 0;
        case 2
          % Ruhelage der Gelenkelastizität optimieren
          Set.optimization.desopt_vars = {'joint_stiffness_qref'};
          % Geringe Gelenksteifigkeit passiver Drehgelenke von 1Nm/Grad
          Set.optimization.joint_stiffness_passive_revolute = 1*180/pi;
        case 3
          % Sowohl Segmentstärke, als auch Feder-Ruhelage optimieren
          Set.optimization.desopt_vars = {'joint_stiffness_qref', 'linkstrength'};
          Set.optimization.joint_stiffness_passive_revolute = 1*180/pi;
        case 4
          Set.optimization.desopt_vars = {};
          Set.optimization.joint_stiffness_passive_revolute = 0;
        case 5
          Set.optimization.desopt_vars = {};
          Set.optimization.joint_stiffness_passive_revolute = 1*180/pi;
      end
      Set.optimization.optname = sprintf('desopt_test_%d_%d_dbc%d', i, k, dbc);
      cds_start
    end
  end
end
