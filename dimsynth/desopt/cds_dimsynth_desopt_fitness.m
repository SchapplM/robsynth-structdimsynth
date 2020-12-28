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
fval_debugtext = '';
% Abbruch prüfen
persistent abort_fitnesscalc
if isempty(abort_fitnesscalc)
  abort_fitnesscalc = false;
elseif abort_fitnesscalc
  fval = Inf;
  return;
end
vartypes = Structure.desopt_ptypes(Structure.desopt_ptypes~=1);
p_ls = p_desopt(vartypes==2);
p_js = p_desopt(vartypes==3);

%% Plausibilität der Eingabe prüfen
if any(vartypes==2) && p_ls(1) > p_ls(2)/2 % Wandstärke darf nicht größer als Radius sein
  f_wall_vs_rad = (p_ls(1) - p_ls(2)/2)/p_ls(2); % Grad der Überschreitung
  f_wall_vs_rad_norm = 2/pi*atan(f_wall_vs_rad); % 50% -> 0.3
  fval = 1e8*(1+9*f_wall_vs_rad_norm); % Normierung auf 1e8...1e9
  constrvioltext = sprintf('Radius (%1.1fmm) ist kleiner als Wandstärke (%1.1fmm)', ...
    1e3*p_ls(2)/2, 1e3*p_ls(1));
end

%% Dynamikparameter aktualisieren
if any(vartypes==2) && fval == 0
  cds_dimsynth_design(R, Q, Set, Structure, p_ls);
end
%% Gelenksteifigkeiten aktualisieren
if any(vartypes==3)
  for i = 1:R.NLEG
    R.Leg(i).DesPar.joint_stiffness_qref(R.Leg(i).MDH.sigma==0) = p_js;
  end
end
%% Dynamik neu berechnen
if any(vartypes==3)
  % TODO: Aufgrund der Gelenkelastizität keine Regressorform verfügbar. Ist
  % so viel zu langsam.
  Structure_tmp = Structure;
  Structure_tmp.calc_reg = false;
  data_dyn = cds_obj_dependencies(R, Traj_0, Set, Structure_tmp, Q, QD, QDD, Jinv_ges);
elseif fval == 0 && Structure.calc_reg
  % Abhängigkeiten neu berechnen (Dynamik)
  data_dyn = cds_obj_dependencies_regmult(R, data_dyn_reg);
  if Set.general.debug_calc && Set.optimization.joint_stiffness_passive_revolute==0
    % Zu Testzwecken die Dynamik neu ohne Regressorform berechnen und mit
    % Regressor-Berechnung vergleichen
    Structure_tmp = Structure;
    Structure_tmp.calc_dyn_act = true;
    Structure_tmp.calc_reg = false;
    data_dyn2 = cds_obj_dependencies(R, Traj_0, Set, Structure_tmp, Q, QD, QDD, Jinv_ges);
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

%% Nebenbedingungen der Zielfunktionswerte berechnen
% Festigkeit der Segmente (mit höherem Strafterm)
if fval == 0 && Set.optimization.constraint_obj(6)
  [fval_ms, constrvioltext_ms] = cds_constr_yieldstrength(R, Set, data_dyn, Jinv_ges, Q, Traj_0);
  fval = fval_ms;
  constrvioltext = constrvioltext_ms;
end
if fval == 0 && Set.optimization.constraint_obj(1) % NB für Masse gesetzt
  [fval_mass, fval_debugtext_mass, ~, fphys_m] = cds_obj_mass(R);
  viol_rel_m = (fphys_m - Set.optimization.constraint_obj(1))/Set.optimization.constraint_obj(1);
  if viol_rel_m > 0
    f_massvio_norm = 2/pi*atan((viol_rel_m)); % 1->0.5; 10->0.94
    fval = 1e3*(1+1*f_massvio_norm); % 1e3 ... 2e3
    constrvioltext = sprintf('Masse ist zu groß (%1.1f > %1.1f)', ...
      fphys_m, Set.optimization.constraint_obj(1));
  end
end
if fval == 0  && Set.optimization.constraint_obj(3) % NB für Antriebskraft gesetzt
  [fval_actforce, fval_debugtext_actforce, ~, fphys_actforce] = cds_obj_actforce(data_dyn.TAU);
  viol_rel_actforce = (fphys_actforce - Set.optimization.constraint_obj(3))/Set.optimization.constraint_obj(3);
  if viol_rel_actforce > 0
    f_actforcevio_norm = 2/pi*atan((viol_rel_actforce)); % 1->0.5; 10->0.94
    fval = 1e3*(1+1*f_actforcevio_norm); % 2e3 ... 3e3
    constrvioltext = sprintf('Antriebskraft ist zu groß (%1.1f > %1.1f)', ...
      fphys_actforce, Set.optimization.constraint_obj(3));
  end
end
if fval == 0  && Set.optimization.constraint_obj(2) % NB für Energie gesetzt
  error('Grenzen für Zielfunktionen Energie noch nicht implementiert');
end
if fval == 0  && Set.optimization.constraint_obj(5) % NB für Steifigkeit gesetzt
  [fval_st, fval_debugtext_st, ~, fphys_st] = cds_obj_stiffness(R, Set, Q);
  viol_rel_st = (fphys_st - Set.optimization.constraint_obj(5))/Set.optimization.constraint_obj(5);
  if viol_rel_st > 0
    f_stvio_norm = 2/pi*atan((viol_rel_st)); % 1->0.5; 10->0.94
    fval = 1e3*(2+1*f_stvio_norm); % 3e3 ... 4e3
    constrvioltext = sprintf('Nachgiebigkeit ist zu groß (%1.1f > %1.1f)', ...
      fphys_st, Set.optimization.constraint_obj(5));
  end
end
if fval > 1000 % Nebenbedingungen verletzt.
  cds_log(4,sprintf('[desopt/fitness] DesOpt-Fitness-Evaluation in %1.1fs. fval=%1.3e. %s', toc(t1), fval, constrvioltext));
  return
end

%% Fitness-Wert berechnen
if any(strcmp(Set.optimization.objective, 'mass'))
  if Set.optimization.constraint_obj(1) % Vermeide doppelten Aufruf der Funktion
    fval = fval_mass; % Nehme Wert von der NB-Berechnung oben
    fval_debugtext = fval_debugtext_mass;
  else
    [fval,fval_debugtext] = cds_obj_mass(R);
  end
elseif any(strcmp(Set.optimization.objective, 'energy'))
  if Set.optimization.constraint_obj(2) % Vermeide doppelten Aufruf der Funktion
    fval = fval_energy; % Nehme Wert von der NB-Berechnung oben
    fval_debugtext = fval_debugtext_energy;
  else
    [fval,fval_debugtext] = cds_obj_energy(R, Set, Structure, Traj_0, data_dyn.TAU, QD);
  end
elseif any(strcmp(Set.optimization.objective, 'actforce'))
  if Set.optimization.constraint_obj(3) % Vermeide doppelten Aufruf der Funktion
    fval = fval_actforce; % Nehme Wert von der NB-Berechnung oben
    fval_debugtext = fval_debugtext_actforce;
  else
    [fval,fval_debugtext] = cds_obj_actforce(data_dyn.TAU);
  end
elseif any(strcmp(Set.optimization.objective, 'stiffness'))
  if Set.optimization.constraint_obj(5) % Vermeide doppelten Aufruf der Funktion
    fval = fval_st; % Nehme Wert von der NB-Berechnung oben
    fval_debugtext = fval_debugtext_st;
  else
    [fval,fval_debugtext] = cds_obj_stiffness(R, Set, Q);
  end
elseif any(strcmp(Set.optimization.objective, 'materialstress'))
  if Set.optimization.constraint_obj(6) % Vermeide doppelten Aufruf der Funktion
    fval = fval_ms; % Nehme Wert von der NB-Berechnung oben
    fval_debugtext = constrvioltext_ms;
  else
    [fval,fval_debugtext] = cds_obj_materialstress(R, Set, data_dyn, Jinv_ges, Q, Traj_0);
  end
else
  % Es wurde eine Dimensionierung gefunden, die alle Nebenbedingungen ein-
  % hält. Keine Zielfunktion definiert, die jetzt noch profitieren würde.
  abort_fitnesscalc = true;
end
cds_log(4,sprintf('[desopt/fitness] DesOpt-Fitness-Evaluation in %1.1fs. Parameter: [%s]. fval=%1.3e. Erfolgreich. %s', ...
    toc(t1), disp_array(p_desopt', '%1.3f'), fval, fval_debugtext));
end
