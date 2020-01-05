% Gütefunktion für Maßsynthese von Robotern (allgemein)
% 
% Eingabe:
% R
%   Matlab-Klasse für Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus
% Q
%   Gelenkwinkel-Trajektorie
% Structure
%   Eigenschaften der Roboterstruktur
% p_desopt
%   Vektor der Optimierungsvariablen für PSO. Siehe cds_dimsynth_design.
%
% Ausgabe:
% fval
%   Fitness-Wert für den Parametervektor p. Enthält Strafterme für
%   Verletzung von Nebenbedingungen oder Wert der Zielfunktion (je nachdem)
% 
% Siehe auch: cds_fitness.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-01
% (C) Institut für Mechatronische Systeme, Universität Hannover

function fval = cds_dimsynth_desopt_fitness(R, Set, Q, Structure, p_desopt)
t1 = tic();
% Debug:
if Set.general.matfile_verbosity > 3
  repopath = fileparts(which('structgeomsynth_path_init.m'));
  save(fullfile(repopath, 'tmp', 'cds_dimsynth_desopt_fitness.mat'));
end
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_desopt_fitness.mat'));

fval = 0;
fval_debugtext = '';

%% Plausibilität der Eingabe prüfen
if p_desopt(1) > p_desopt(2)/2 % Wandstärke darf nicht größer als Radius sein
  p_desopt(1) = p_desopt(2)/2;
  fval = 1e8;
  constrvioltext = 'Radius ist kleiner als Wandstärke';
end

%% Dynamikparameter aktualisieren
% Trage die Dynamikparameter
cds_dimsynth_design(R, Q, Set, Structure, p_desopt);

%% Nebenbedingungen der Entwurfsvariablen berechnen
% TODO: Festigkeit der Segmente, Grenzen der Antriebe

%% Nebenbedingungen der Zielfunktionswerte berechnen
if any(Set.optimization.constraint_obj)
  if Set.optimization.constraint_obj(1) % NB für Masse gesetzt
    [fval_mass, fval_debugtext_mass, ~, fphys] = cds_obj_mass(R);
    viol_rel = (fphys - Set.optimization.constraint_obj(1))/Set.optimization.constraint_obj(1);
    if viol_rel > 0
      f_massvio_norm = 2/pi*atan((viol_rel)); % 1->0.5; 10->0.94
      fval = 1e4*(1+9*f_massvio_norm);
      constrvioltext = sprintf('Masse ist zu groß (%d > %d)', fval, fphys, Set.optimization.constraint_obj(1));
    elseif Set.general.verbosity > 3
      fprintf('fval = %1.3e. Masse i.O. (%d < %d)\n', fval, fphys, Set.optimization.constraint_obj(1));
    end
  elseif any(Set.optimization.constraint_obj(1))
    error('Grenzen für andere Zielfunktionen als die Masse noch nicht implementiert');
  end
end
if fval > 1000 % Nebenbedingungen verletzt.
  if Set.general.verbosity > 3
    fprintf('DesOpt-Fitness-Evaluation in %1.1fs. fval=%1.3e. %s\n', toc(t1), fval, constrvioltext);
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
else
  error('Andere Zielfunktion als Masse noch nicht implementiert');
end
if Set.general.verbosity >3
  fprintf('DesOpt-Fitness-Evaluation in %1.1fs. Parameter: [%s]. fval=%1.3e. Erfolgreich. %s.\n', ...
    toc(t1), disp_array(p_desopt', '%1.3f'), fval, fval_debugtext);
end