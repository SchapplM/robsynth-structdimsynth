% Gütefunktion für Maßsynthese von Robotern (allgemein)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function fval = cds_fitness(R, Set, Traj_W, Structure, p)
repopath = fileparts(which('structgeomsynth_path_init.m'));
% Debug:
if Set.general.matfile_verbosity > 2
  save(fullfile(repopath, 'tmp', 'cds_fitness1.mat'));
end
% Debug:
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_fitness1.mat'));

t1=tic();
debug_info = {};

%% Parameter prüfen
if p(1) == 0
  error('Roboterskalierung kann nicht Null werden');
end

%% Parameter aktualisieren
% Keine Verwendung der Ausgabe: Parameter werden direkt in ursprüngliche
% Funktion geschrieben; R.pkin ist vor/nach dem Aufruf unterschiedlich
cds_update_robot_parameters(R, Set, Structure, p);

%% Trajektorie anpassen
Traj_0 = cds_rotate_traj(Traj_W, R.T_W_0);

%% Nebenbedingungen prüfen
[fval,Q,QD,QDD,Jinvges,constrvioltext] = cds_constraints(R, Traj_0, Traj_W, Set, Structure);
cds_fitness_debug_plot_robot(R, zeros(R.NJ,1), Traj_0, Traj_W, Set, Structure, p, fval, debug_info);
if fval > 0 % Nebenbedingungen verletzt.
  fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3e. %s\n', toc(t1), fval, constrvioltext);
  return
end

if Set.general.matfile_verbosity > 2
  save(fullfile(repopath, 'tmp', 'cds_fitness_2.mat'));
end
% Debug:
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_fitness_2.mat'));
%% Dynamik-Parameter
if any(strcmp(Set.optimization.objective, {'energy', 'mass', 'minactforce'}))
  % Gelenkgrenzen in Roboterklasse neu eintragen
  if R.Type == 0 % Seriell
    R.qlim = minmax2(Q');
  else % PKM
    for i = 1:R.NLEG
      R.Leg(i).qlim = minmax2(Q(:,R.I1J_LEG(i):R.I2J_LEG(i))');
    end
  end
  % Dynamik-Parameter aktualisieren. Keine Nutzung der Ausgabe der Funktion
  % (Parameter werden direkt in Klasse geschrieben; R.DesPar.seg_par ist
  % vor/nach dem Aufruf unterschiedlich)
  cds_dimsynth_desopt(R, Q, Traj_0, Set, Structure);
end
if Set.general.matfile_verbosity > 1
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_fitness3.mat'));
end
% Debug:
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_fitness3.mat'));

%% Berechnungen für Zielfunktionen
output = cds_obj_dependencies(R, Traj_0, Set, Q, QD, QDD, Jinvges);
if any(strcmp(Set.optimization.objective, {'energy', 'minactforce'}))
  TAU = output.TAU;
end

%% Zielfunktion berechnen
if strcmp(Set.optimization.objective, 'valid_act')
  [fval,fval_debugtext, debug_info] = cds_obj_valid_act(R, Set, Jinvges);
elseif strcmp(Set.optimization.objective, 'condition')
  [fval,fval_debugtext, debug_info] = cds_obj_condition(R, Set, Structure, Jinvges, Traj_0, Q, QD);
elseif strcmp(Set.optimization.objective, 'energy')
  [fval,fval_debugtext, debug_info] = cds_obj_energy(R, Set, Structure, Traj_0, TAU, QD);
elseif strcmp(Set.optimization.objective, 'mass')
  [fval,fval_debugtext, debug_info] = cds_obj_mass(R);
elseif strcmp(Set.optimization.objective, 'minactforce')
  [fval,fval_debugtext, debug_info] = cds_obj_minactforce(TAU);
else
  error('Zielfunktion "%s" nicht definiert', Set.optimization.objective{1});
end
fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3e. Erfolgreich. %s.\n', toc(t1), fval, fval_debugtext);
cds_fitness_debug_plot_robot(R, Q(1,:)', Traj_0, Traj_W, Set, Structure, p, fval, debug_info);
