% Gütefunktion für Maßsynthese von Robotern (allgemein)
% 
% Eingabe:
% R
%   Matlab-Klasse für Roboter (SerRob/ParRob)
% Traj_0
%   Roboter-Trajektorie (EE) bezogen auf Basis-KS des Roboters
% Set
%   Einstellungen des Optimierungsalgorithmus
% Q, QD, QDD
%   Gelenkwinkel-Trajektorie
% Jinvges
%   Zeilenweise (inverse) Jacobi-Matrizen des Roboters (für PKM). Bezogen
%   auf vollständige Gelenkgeschwindigkeiten und Plattform-Geschw. (in
%   x-Koordinaten, nicht: Winkelgeschwindigkeit)
% data_dyn_reg
%   Struktur mit Dynamik-Eigenschaften des Roboters als Regressormatrizen.
%   Felder: Siehe cds_obj_dependencies_regmult
% Structure
%   Eigenschaften der Roboterstruktur
% p_desopt
%   Vektor der Optimierungsvariablen für PSO. Siehe cds_dimsynth_design.
%
% Ausgabe:
% fval
%   Fitness-Wert für den Parametervektor p. Enthält Strafterme für
%   Verletzung von Nebenbedingungen oder Wert der Zielfunktion (je nachdem)
%   Werte:
%   0...1e3: gewählte Zielfunktion
%   1e3...1e4: Nebenbedingung von Zielfunktion überschritten
%   1e4...1e5: Überschreitung Belastungsgrenze der Segmente
%   1e8...1e9: Unplausible Eingabe (Radius vs Wandstärke)
% 
% Siehe auch: cds_fitness.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-01
% (C) Institut für Mechatronische Systeme, Universität Hannover

function fval = cds_dimsynth_desopt_fitness(R, Set, Traj_0, Q, QD, QDD, Jinv_ges, data_dyn_reg, Structure, p_desopt)
t1 = tic();
% Debug:
if Set.general.matfile_verbosity > 3
  repopath = fileparts(which('structgeomsynth_path_init.m'));
  save(fullfile(repopath, 'tmp', 'cds_dimsynth_desopt_fitness.mat'));
end
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_desopt_fitness.mat'));

fval = 0;

%% Plausibilität der Eingabe prüfen
if p_desopt(1) > p_desopt(2)/2 % Wandstärke darf nicht größer als Radius sein
  f_wall_vs_rad = (p_desopt(1) - p_desopt(2)/2)/p_desopt(2); % Grad der Überschreitung
  f_wall_vs_rad_norm = 2/pi*atan(f_wall_vs_rad); % 50% -> 0.3
  fval = 1e8*(1+9*f_wall_vs_rad_norm); % Normierung auf 1e8...1e9
  constrvioltext = sprintf('Radius (%1.1fmm) ist kleiner als Wandstärke (%1.1fmm)', ...
    1e3*p_desopt(2)/2, 1e3*p_desopt(1));
end

%% Dynamikparameter aktualisieren
% Trage die Dynamikparameter
if fval == 0
  cds_dimsynth_design(R, Q, Set, Structure, p_desopt);
end
%% Dynamik neu berechnen
if fval == 0 && Structure.calc_reg
  % Abhängigkeiten neu berechnen (Dynamik)
  data_dyn = cds_obj_dependencies_regmult(R, Set, data_dyn_reg);
  if Set.general.debug_calc
    % Zu Testzwecken die Dynamik neu ohne Regressorform berechnen und mit
    % Regressor-Berechnung vergleichen
    data_dyn2 = cds_obj_dependencies(R, Traj_0, Set, Structure, Q, QD, QDD, Jinv_ges);
    test_TAU = data_dyn2.TAU - data_dyn.TAU;
    if any(abs(test_TAU(:))>1e-8)
      error('Antriebskräfte aus Regressorform stimmt nicht');
    end
    test_W = data_dyn2.Wges - data_dyn.Wges;
    if any(abs(test_W(:))>1e-8)
      error('Schnittkräfte aus Regressorform stimmt nicht');
    end
  end
end

%% Nebenbedingungen der Entwurfsvariablen berechnen: Festigkeit der Segmente
if fval == 0 && Set.optimization.constraint_link_yieldstrength > 0
  [fval, constrvioltext] = cds_constr_yieldstrength(R, Set, data_dyn, Jinv_ges, Q, Traj_0);
end

%% Nebenbedingungen der Zielfunktionswerte berechnen
if fval == 0 && any(Set.optimization.constraint_obj)
  if Set.optimization.constraint_obj(1) % NB für Masse gesetzt
    [fval_mass, fval_debugtext_mass, ~, fphys] = cds_obj_mass(R);
    viol_rel = (fphys - Set.optimization.constraint_obj(1))/Set.optimization.constraint_obj(1);
    if viol_rel > 0
      f_massvio_norm = 2/pi*atan((viol_rel)); % 1->0.5; 10->0.94
      fval = 1e3*(1+9*f_massvio_norm); % 1e3 ... 1e4
      constrvioltext = sprintf('Masse ist zu groß (%1.1f > %1.1f)', fphys, Set.optimization.constraint_obj(1));
    end
  end
  if any(Set.optimization.constraint_obj([2 3]))
    error('Grenzen für andere Zielfunktionen als die Masse noch nicht implementiert');
  end
end
if fval > 1000 % Nebenbedingungen verletzt.
  if Set.general.verbosity > 3
    fprintf('[desopt/fitness] DesOpt-Fitness-Evaluation in %1.1fs. fval=%1.3e. %s\n', toc(t1), fval, constrvioltext);
  end
  return
end

%% Fitness-Wert berechnen
% TODO: Die gleichen Zielfunktionen wie für die Maßsynthese können auch in
% der Entwurfsoptimierung berechnet werden
if strcmp(Set.optimization.objective, 'mass')
  if Set.optimization.constraint_obj(1) % Vermeide doppelten Aufruf der Funktion
    fval = fval_mass; % Nehme Wert von der NB-Berechnung oben
    fval_debugtext = fval_debugtext_mass;
  else
    [fval,fval_debugtext] = cds_obj_mass(R);
  end
elseif strcmp(Set.optimization.objective, 'energy')
  if Set.optimization.constraint_obj(2) % Vermeide doppelten Aufruf der Funktion
    fval = fval_energy; % Nehme Wert von der NB-Berechnung oben
    fval_debugtext = fval_debugtext_energy;
  else
    [fval,fval_debugtext] = cds_obj_energy(R, Set, Structure, Traj_0, data_dyn.TAU, QD);
  end
else
  error('Andere Zielfunktion als Masse noch nicht implementiert');
end
if Set.general.verbosity >3
  fprintf('[desopt/fitness] DesOpt-Fitness-Evaluation in %1.1fs. Parameter: [%s]. fval=%1.3e. Erfolgreich. %s.\n', ...
    toc(t1), disp_array(p_desopt', '%1.3f'), fval, fval_debugtext);
end
