% Gütefunktion für Maßsynthese von Robotern (allgemein)
% 
% Eingabe:
% R
%   Matlab-Klasse für Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus
% Traj_W
%   Trajektorie (bezogen auf Welt-KS)
% Structure
%   Eigenschaften der Roboterstruktur
% p
%   Vektor der Optimierungsvariablen für PSO
% 
% Ausgabe:
% fval
%   Fitness-Wert für den Parametervektor p. Enthält Strafterme für
%   Verletzung von Nebenbedingungen oder Wert der Zielfunktion (je nachdem)
%   Werte:
%   0...1e3: gewählte Zielfunktion
%   1e3...1e4: Überschreitung dynamischer NB (Antriebskraft)
%   1e4...1e5: Nebenbedingung für Zielfunktion in Entwurfsopt. überschritten
%   1e5...1e6: Überschreitung Belastungsgrenze der Segmente (aus cds_dimsynth_desopt_fitness)
%   1e6...1e7: Überschreitung kinematischer NB (Kondition)
%   1e7...1e13: Siehe cds_constraints. Werte von dort mit Faktor 1e4 multipliziert

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function fval = cds_fitness(R, Set, Traj_W, Structure, p)
repopath = fileparts(which('structgeomsynth_path_init.m'));
rng(0); % Für Wiederholbarkeit der Versuche: Zufallszahlen-Initialisierung

% Debug:
if Set.general.matfile_verbosity > 2
  save(fullfile(repopath, 'tmp', 'cds_fitness_1.mat'));
end
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_fitness_1.mat'),  '-regexp', '^(?!abort_fitnesscalc$).');

t1=tic();
debug_info = {};
Jcond = NaN; % Konditionszahl der Jacobi-Matrix (für spätere Auswertung)
f_maxstrengthviol = NaN; % Überschreitung der Materialspannungsgrenzen (...)
fval = NaN(length(Set.optimization.objective),1);
physval = fval;
%% Abbruch prüfen
% Prüfe, ob Berechnung schon abgebrochen werden kann, weil ein anderes
% Partikel erfolgreich berechnet wurde. Dauert sonst eine ganze Generation.
persistent abort_fitnesscalc
if isempty(abort_fitnesscalc)
  abort_fitnesscalc = false;
elseif abort_fitnesscalc
  fval(:) = Inf;
  cds_log(2,sprintf(['[fitness] Fitness-Evaluation in %1.1fs. fval=[%s]. ', ...
    'Bereits anderes Gut-Partikel berechnet.'], toc(t1), disp_array(fval', '%1.3e')));
  cds_save_particle_details(Set, R, toc(t1), fval, p, physval, Jcond, f_maxstrengthviol);
  return;
end
%% Parameter prüfen
if p(1) == 0
  error('Roboterskalierung kann nicht Null werden');
end

%% Parameter aktualisieren
% Keine Verwendung der Ausgabe: Parameter werden direkt in ursprüngliche
% Funktion geschrieben; R.pkin ist vor/nach dem Aufruf unterschiedlich
cds_update_robot_parameters(R, Set, Structure, p);

%% Trajektorie anpassen
Traj_0 = cds_transform_traj(R, Traj_W);

%% Nebenbedingungen prüfen (für Eckpunkte)
% NB-Verletzung wird in Ausgabe mit Werten von 1e3 aufwärts angegeben.
% Umwandlung in Werte von 1e6 aufwärts.
[fval_constr,QE_iIKC, Q0, constrvioltext] = cds_constraints(R, Traj_0, Set, Structure);
fval(:) = fval_constr*1e4; % Erhöhung, damit später kommende Funktionswerte aus Entwurfsoptimierung kleiner sein können
cds_fitness_debug_plot_robot(R, Q0(1,:)', Traj_0, Traj_W, Set, Structure, p, fval(1), debug_info);
if fval_constr > 1000 % Nebenbedingungen verletzt.
  % Speichere die Anfangs-Winkelstellung in der Roboterklasse für später.
  % Dient zum Vergleich und zur Reproduktion der Ergebnisse
  if R.Type == 0 % Seriell
    R.qref = Q0(1,:)';
  else
    for i = 1:R.NLEG, R.Leg(i).qref = Q0(1,R.I1J_LEG(i):R.I2J_LEG(i))'; end
  end
  cds_log(2,sprintf('[fitness] Fitness-Evaluation in %1.1fs. fval=%1.3e. %s', toc(t1), fval(1), constrvioltext));
  cds_save_particle_details(Set, R, toc(t1), fval, p, physval, Jcond, f_maxstrengthviol);
  return
end
% Gelenkgrenzen merken (werden später überschrieben)
if R.Type == 0, qlim = R.qlim; % Seriell
else,           qlim = cat(1, R.Leg.qlim); end % PKM
%% Alle IK-Konfigurationen durchgehen
% Berechne jeweils alle Zielfunktionen. Bestimme erst danach, welcher
% Parametersatz der beste ist

fval_IKC = NaN(size(Q0,1), length(Set.optimization.objective));
physval_IKC = fval_IKC; 
constrvioltext_IKC = cell(size(Q0,1), 1);
Jcond_IKC = NaN(size(Q0,1),1);
mges_IKC = NaN(size(Q0,1),1); % Gesamtmasse der PKM
fval_debugtext_IKC = constrvioltext_IKC;
f_maxstrengthviol_IKC = Jcond_IKC;
Q_IKC = NaN(size(Traj_0.X,1), R.NJ, size(Q0,1));
% Zum Debuggen
% if R.Type == 0, n_actjoint = R.NJ;
% else,           n_actjoint = sum(R.I_qa); end
% TAU_IKC = NaN(size(Traj_0.X,1), n_actjoint, size(Q0,1));

for iIKC = 1:size(Q0,1)
  % Gelenkgrenzen von oben wieder erneut einsetzen. Notwendig, da Grenzen
  % in Traj.-IK berücksichtigt werden. Unten wird der Wert aktualisiert
  if R.Type == 0 % Seriell
    R.qlim = qlim;
  else % PKM
    for i = 1:R.NLEG, R.Leg(i).qlim = qlim(R.I1J_LEG(i):R.I2J_LEG(i),:); end
  end
  %% Trajektorie berechnen
  if Set.task.profile ~= 0 % Nur Berechnen, falls es eine Trajektorie gibt
    [fval_trajconstr,Q,QD,QDD,Jinv_ges,constrvioltext_IKC{iIKC}] = cds_constraints_traj( ...
      R, Traj_0, Q0(iIKC,:)', Set, Structure);
    fval_IKC(iIKC,:) = fval_trajconstr;
  else
    Q = QE_iIKC(:,:,iIKC);
    QD = 0*QE_iIKC; QDD = QD;
    % Es liegt keine Trajektorie vor. Es reicht also, das Ergebnis der IK von
    % der Eckpunkt-Berechnung zu benutzen um die Jacobi-Matrix zu berechnen
    if R.Type == 0 % Seriell
      Jinv_ges = NaN; % Platzhalter
    else % Parallel
      Jinv_ges = NaN(size(Q,1), sum(R.I_EE)*size(Q,2));
      for i = 1:size(Q,1)
        [~,J_x_inv] = R.jacobi_qa_x(Q(i,:)', Traj_0.X(i,:)');
        if any(isnan(J_x_inv(:))) || any(isinf(J_x_inv(:)))
          % Durch numerische Fehler können Inf- oder NaN-Einträge in der
          % Jacobi-Matrix entstehen (Singularität)
          if Set.general.matfile_verbosity > 2
            save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_fitness_J_infnan.mat'));
          end
          J_x_inv = zeros(size(J_x_inv));
        end
        Jinv_ges(i,:) = J_x_inv(:);
      end
    end
  end
  Q_IKC(:,:,iIKC) = Q;
  % Normalisieren der Winkel (erst hier durchführen, da einige Prüfungen oben
  % davon beeinflusst werden).
  Q(:,R.MDH.sigma==0) = wrapToPi(Q(:,R.MDH.sigma==0));
  if R.Type == 0
    R.qref(R.MDH.sigma==0) = wrapToPi(R.qref(R.MDH.sigma==0));
  else
    for k = 1:R.NLEG
      R.Leg(k).qref(R.Leg(k).MDH.sigma==0) = wrapToPi(R.Leg(k).qref(R.Leg(k).MDH.sigma==0));
    end
  end

  if fval_trajconstr > 1000 % Nebenbedingungen verletzt.
    continue; % Nächste Anfangs-Konfiguration
  end
  % Prüfe Validität der Jacobi (nur für PKM)
  if any(isinf(Jinv_ges(:))) || Structure.Type~=0 && any(isnan(Jinv_ges(:)))
    save(fullfile(repopath, 'tmp', 'cds_fitness_J_infnan.mat'));
    error('Jacobi hat Inf oder NaN. Darf hier eigentlich nicht sein!');
  end
  % Prüfe Validität der Geschwindigkeiten
  if isempty(QD) || isempty(QDD)
    save(fullfile(repopath, 'tmp', 'cds_fitness_QD_empty.mat'));
    error('Geschwindigkeit ist nicht belegt. Darf hier eigentlich nicht sein!');
  end
  if Set.general.matfile_verbosity > 2
    save(fullfile(repopath, 'tmp', 'cds_fitness_2.mat'));
  end
  % Debug:
  % load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_fitness_2.mat'));
  %% Konditionszahl als Nebenbedingung prüfen
  if Set.optimization.constraint_obj(4) > 0 % NB für Kondition gesetzt
    [fval_cond,fval_debugtext_cond, debug_info_cond, Jcond_IKC(iIKC)] = cds_obj_condition(R, Set, Structure, Jinv_ges, Traj_0, Q, QD);
    if Jcond_IKC(iIKC) > Set.optimization.constraint_obj(4)
      fval_IKC(iIKC,:) = 1e6*(1+9*fval_cond/1e3); % normiert auf 1e6 bis 1e7
      % debug_info = {sprintf('Kondition %1.1e > %1.1e', Jcond, Set.optimization.constraint_obj(4)); debug_info_cond{1}};
      constrvioltext_IKC{iIKC} = sprintf('Konditionszahl ist zu schlecht: %1.1e > %1.1e', Jcond_IKC(iIKC), Set.optimization.constraint_obj(4));
      continue
    end
  end

  %% Dynamik-Parameter
  % Gelenkgrenzen für Schubgelenke in Roboterklasse neu eintragen.
  % Für Drehgelenke keine Aktualisierung notwendig (wird nicht benutzt).
  % Nutzen: Berechnung der Masse von Führungsschienen und Hubzylindern,
  % Anpassung der Kollisionskörper (für nachgelagerte Prüfung und Plots)
  % Bereits hier, damit Ergebnis-Visualisierung konsistent ist.
  if R.Type == 0 % Seriell
    R.qlim(R.MDH.sigma==1,:) = minmax2(Q(:,R.MDH.sigma==1)');
  else % PKM
    for i = 1:R.NLEG
      Q_i = Q(:,R.I1J_LEG(i):R.I2J_LEG(i));
      R.Leg(i).qlim(R.Leg(i).MDH.sigma==1,:) = minmax2(Q_i(:,R.Leg(i).MDH.sigma==1)');
    end
  end
  massparam_set = false; % Marker, ob Masseparameter gesetzt wurden
  if ~isempty(intersect(Set.optimization.objective, {'energy', 'mass', 'actforce', 'stiffness'})) || ... % Für Zielfunktion benötigt
      Set.optimization.constraint_obj(3) ~= 0 % Für Nebenbedingung benötigt
    % Dynamik-Parameter aktualisieren. Keine Nutzung der Ausgabe der Funktion
    % (Parameter werden direkt in Klasse geschrieben; R.DesPar.seg_par ist
    % vor/nach dem Aufruf unterschiedlich)
    if ~Set.optimization.use_desopt
      cds_dimsynth_design(R, Q, Set, Structure);
    else
      % Berechne Dynamik-Funktionen als Regressorform für die Entwurfsopt.
      data_dyn = cds_obj_dependencies(R, Traj_0, Set, Structure, Q, QD, QDD, Jinv_ges);

      fval_desopt = cds_dimsynth_desopt(R, Traj_0, Q, QD, QDD, Jinv_ges, data_dyn, Set, Structure);
      if fval_desopt > 1e5
        warning('Ein Funktionswert > 1e5 ist nicht für Entwurfsoptimierung vorgesehen');
      end
      if fval_desopt > 1000 % Nebenbedingungen in Entwurfsoptimierung verletzt.
        % Neue Werte (geändert gegenüber cds_dimsynth_desopt_fitness.)
        % 1e4...1e5: Nebenbedingung von Zielfunktion überschritten
        % 1e5...1e6: Überschreitung Belastungsgrenze der Segmente
        fval_IKC(iIKC,:) = 10*fval_desopt; % Wert ist im Bereich 1e3...1e5. Bringe in Bereich 1e4 bis 1e6
        cds_fitness_debug_plot_robot(R, zeros(R.NJ,1), Traj_0, Traj_W, Set, Structure, p, fval(1), debug_info);
        constrvioltext_IKC{iIKC} = 'Verletzung der Nebenbedingungen in Entwurfsoptimierung';
        continue
      end
    end
    massparam_set = true;
  end
  if Set.general.matfile_verbosity > 2
    save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_fitness_3.mat'));
  end
  % Debug:
  % load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_fitness_3.mat'));

  %% Berechnungen für Zielfunktionen
  if ~Structure.calc_reg
    data_dyn2 = cds_obj_dependencies(R, Traj_0, Set, Structure, Q, QD, QDD, Jinv_ges);
  else
    % Dynamik nochmal mit Regressorform mit neuen Dynamikparameter berechnen
    data_dyn2 = cds_obj_dependencies_regmult(R, data_dyn);
  end
  if ~isempty(intersect(Set.optimization.objective, {'energy', 'actforce'})) || ...  % Für Zielf. benötigt
      Set.optimization.constraint_obj(3) ~= 0 % Für NB benötigt
    TAU = data_dyn2.TAU;
%     TAU_IKC(:,:,iIKC) = TAU; % Zum Debuggen
  end

  %% Nebenbedingungen der Entwurfsvariablen berechnen: Festigkeit der Segmente
  if Set.optimization.constraint_link_yieldstrength > 0 && ~Set.optimization.use_desopt
    % Wenn use_desopt gemacht wurde, wurde die Nebenbedingung bereits oben
    % geprüft und hier ist keine Berechnung notwendig.
    % Für den anderen Fall wird hier der gleiche Wertebereich genutzt (1e5..1e6)
    [fval_ys, constrvioltext_IKC{iIKC}, f_maxstrengthviol_IKC(iIKC)] = cds_constr_yieldstrength(R, Set, data_dyn2, Jinv_ges, Q, Traj_0);
    if fval_ys > 1e5 % Muss schon im Bereich 1e4...1e5 sein (im Fehlerfall)
      error('Dieser Fall ist nicht vorgesehen');
    elseif fval_ys>1e4
      fval_IKC(iIKC,:) = 10*fval_ys; % Bringe in Bereich 1e5 ... 1e6
      continue
    end
  end
  %% Nebenbedingungen der Entwurfsvariablen berechnen: Steifigkeit
  if Set.optimization.constraint_obj(5)
    [fval_st, ~, ~, fval_phys_st] = cds_obj_stiffness(R, Set, Q);
    if fval_phys_st > Set.optimization.constraint_obj(5)
      % Nutze den gleichen Wertebereich wie Entwurfsoptimierung oben.
      fval_IKC(iIKC,:) = fval_st*100; % Bringe in Bereich 1e4 ... 1e5
      constrvioltext_IKC{iIKC} = sprintf('Die Nachgiebigkeit ist zu groß: %1.1e > %1.1e', ...
        fval_phys_st, Set.optimization.constraint_obj(5));
      cds_log(2,sprintf('[fitness] Fitness-Evaluation in %1.1fs. fval=%1.3e. %s', toc(t1), fval(1), constrvioltext_stiffness));
      continue
    end
  end

  %% Antriebskraft als Nebenbedingung prüfen
  if Set.optimization.constraint_obj(3) > 0 % NB für Antriebskraft gesetzt
    [fval_actforce,fval_debugtext_actforce, debug_info_actforce, tau_a_max] = cds_obj_actforce(TAU);
    if tau_a_max > Set.optimization.constraint_obj(3)
      fval_IKC(iIKC,:) = 1e3*(1+9*fval_actforce/1e3); % normiert auf 1e3 bis 1e4
      debug_info = {sprintf('Antriebskraft %1.1e > %1.1e', tau_a_max, Set.optimization.constraint_obj(3)); debug_info_cond{1}};
      cds_fitness_debug_plot_robot(R, Q(1,:)', Traj_0, Traj_W, Set, Structure, p, fval(1), debug_info);
      constrvioltext_IKC{iIKC} = sprintf('Antriebskraft zu hoch: %1.1e > %1.1e', tau_a_max, Set.optimization.constraint_obj(3));
      continue
    end
  end
  %% Zielfunktion berechnen
  fval_debugtext = '';
  if any(strcmp(Set.optimization.objective, 'valid_act'))
    [fval_va,fval_debugtext_va, debug_info] = cds_obj_valid_act(R, Set, Jinv_ges);
    if fval_va < 1e3 % Wenn einmal der Freiheitsgrad festgestellt wurde, reicht das 
      abort_fitnesscalc = true;
    end
    fval_IKC(iIKC,strcmp(Set.optimization.objective, 'valid_act')) = fval_va;
    physval_IKC(iIKC,strcmp(Set.optimization.objective, 'valid_act')) = NaN; % nicht definiert.
    fval_debugtext = [fval_debugtext, ' ', fval_debugtext_va]; %#ok<AGROW>
  end
  if any(strcmp(Set.optimization.objective, 'condition'))
    if Set.optimization.constraint_obj(4) == 0
      [fval_cond, fval_debugtext_cond, debug_info, Jcond_IKC(iIKC)] = cds_obj_condition(R, Set, Structure, Jinv_ges, Traj_0, Q, QD);
    else % Bereits oben berechnet. Keine Neuberechnung notwendig.
      debug_info = debug_info_cond;
    end
    fval_IKC(iIKC,strcmp(Set.optimization.objective, 'condition')) = fval_cond;
    physval_IKC(iIKC,strcmp(Set.optimization.objective, 'condition')) = Jcond_IKC(iIKC);
    fval_debugtext = [fval_debugtext, ' ', fval_debugtext_cond]; %#ok<AGROW>
  end
  if any(strcmp(Set.optimization.objective, 'energy'))
    [fval_en,fval_debugtext_en, debug_info, physval_en] = cds_obj_energy(R, Set, Structure, Traj_0, TAU, QD);
    fval_IKC(iIKC,strcmp(Set.optimization.objective, 'energy')) = fval_en;
    physval_IKC(iIKC,strcmp(Set.optimization.objective, 'energy')) = physval_en;
    fval_debugtext = [fval_debugtext, ' ', fval_debugtext_en]; %#ok<AGROW>
  end
  if massparam_set
    % Berechne in jedem Fall die Gesamtmasse, sobald das möglich ist (geht sehr 
    % schnell und hilft im Entscheidungsfall am Ende dieser Funktion)
    [fval_m,fval_debugtext_m, debug_info, mges_IKC(iIKC)] = cds_obj_mass(R);
  end
  if any(strcmp(Set.optimization.objective, 'mass'))
    fval_IKC(iIKC,strcmp(Set.optimization.objective, 'mass')) = fval_m;
    physval_IKC(iIKC,strcmp(Set.optimization.objective, 'mass')) = mges_IKC(iIKC);
    fval_debugtext = [fval_debugtext, ' ', fval_debugtext_m]; %#ok<AGROW>
  end
  if any(strcmp(Set.optimization.objective, 'actforce'))
    if Set.optimization.constraint_obj(3) == 0
      [fval_actforce,fval_debugtext_actforce, debug_info, tau_a_max] = cds_obj_actforce(TAU);
    else % Bereits oben berechnet.
      debug_info = debug_info_actforce;
    end
    fval_IKC(iIKC,strcmp(Set.optimization.objective, 'actforce')) = fval_actforce;
    physval_IKC(iIKC,strcmp(Set.optimization.objective, 'actforce')) = tau_a_max;
    fval_debugtext = [fval_debugtext, ' ', fval_debugtext_actforce]; %#ok<AGROW>
  end
  if any(strcmp(Set.optimization.objective, 'jointrange'))
    [fval_jr,fval_debugtext_jr, debug_info, physval_jr] = cds_obj_jointrange(R, Set, Structure, Q);
    fval_IKC(iIKC,strcmp(Set.optimization.objective, 'jointrange')) = fval_jr;
    physval_IKC(iIKC,strcmp(Set.optimization.objective, 'jointrange')) = physval_jr;
    fval_debugtext = [fval_debugtext, ' ', fval_debugtext_jr]; %#ok<AGROW>
  end
  if any(strcmp(Set.optimization.objective, 'stiffness'))
    [fval_st,fval_debugtext_st, debug_info, physval_st] = cds_obj_stiffness(R, Set, Q);
    fval_IKC(iIKC,strcmp(Set.optimization.objective, 'stiffness')) = fval_st;
    physval_IKC(iIKC,strcmp(Set.optimization.objective, 'stiffness')) = physval_st;
    fval_debugtext = [fval_debugtext, ' ', fval_debugtext_st]; %#ok<AGROW>
  end
  fval_debugtext_IKC{iIKC} = fval_debugtext;
  if any(fval_IKC(iIKC,:)>1e3)
    error('Zielfunktion "%s" nicht definiert', Set.optimization.objective{fval_IKC(iIKC,:)>1e3});
  end
  if all(fval_IKC(iIKC,:)' <= Set.optimization.obj_limit) || ...
     all(physval_IKC(iIKC,:)' <= Set.optimization.obj_limit_physval)
    % Die Fitness-Funktion ist besser als die Grenze. Optimierung kann
    % hiernach beendet werden.
    abort_fitnesscalc = true;
    % break; % zur Nachvollziehbarkeit kein direkter Abbruch.
  end
end % Schleife über IK-Konfigurationen
if Set.general.matfile_verbosity > 2
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_fitness_4.mat'));
end
%% Bestes Ergebnis heraussuchen
I_IKC_iO = find(all(fval_IKC < 1e3, 2)); % i.O.-Partikel (für Auswertungen)
if length(Set.optimization.objective) == 1
  fval_min = min(fval_IKC);
  iIKCopt = find(fval_IKC==fval_min); % es kann mehrere gleich gute Lösungen geben
else
  % Prüfe, welche Ergebnisse Pareto-optimal sind (durch Ausschluss)
  % Nehme alle Kriterien als Differenz zum jeweils besten. Dadurch ist noch
  % ein Runden möglich. Exakte Gleichheit tritt aufgrund der numerischen IK
  % nicht auf. Identische Zielfunktionswerte sind bei kinematischen Größen
  % möglich.
  fval_IKC_check = fval_IKC; % Kleinster Spaltenwert ist jeweils Null.
  for mm = 1:size(fval_IKC,2) % Kriterien durchgehen
    fval_IKC_check(:,mm) = fval_IKC_check(:,mm) - min(fval_IKC(:,mm));
  end
  fval_IKC_check(abs(fval_IKC_check)<1e-10) = 0; % Runde kleine Differenzen auf Gleichheit
  Idom_ges = false(size(fval_IKC,1), 1);
  for iIKC = 1:size(fval_IKC,1)
    Idom_i = true(size(fval_IKC,1),1); % Marker, dass ll dominiert wird
    for mm = 1:size(fval_IKC,2) % Kriterien durchgehen
      % Wenn IK-Konfig. iIKC in allen Zielfunktionen schlechter ist als eine 
      % andere, bleibt die 1 stehen. Ansonsten steht am Ende eine Null und 
      % Partikel iIKC ist Pareto-optimal.
      Idom_i = Idom_i & (fval_IKC_check(iIKC,mm) >= fval_IKC_check(:,mm));
    end
    Idom_i(iIKC) = false; % Partikel kann nicht von sich selbst dominiert werden
    if any(Idom_i) % irgend ein anderes Partikel ist in allen Kategorien besser ...
      Idom_ges(iIKC) = true; % ... daher dieses entfernen
    end
  end
  iIKCopt = find(~Idom_ges); % Menge der möglichen Pareto-optimalen Lösungen
end
% Definition "optimaler" Lösungen: Die beste/besten Lösungen.
% "Beste" Lösung: Nach bestimmten Kriterien aus mehreren Optimalen gewählt
n_fval_opt = length(iIKCopt);
if length(iIKCopt) == 1
  iIKCbest = iIKCopt(1);
else
  % Es gibt mehrere gleichwertige "optimale" Lösungen (einkriteriell) oder
  % Pareto-Optimale Lösungen. Wähle anhand eines weiteren Kriteriums aus.
  % Die gewählte Lösung ist dann die "beste" Lösung.
  I_best_opt = 0; % Index zur Auswahl in iIKCopt
  % Nehme die Lösung mit der besten Konditionszahl. Toleranz gegen
  % Rundungsfehler; sonst beliebige Wahl quasi identischer Lösungen.
  Jcond_IKC_check = Jcond_IKC - Jcond_IKC(iIKCopt(1));
  Jcond_IKC_check(abs(Jcond_IKC_check)<1e-10) = 0;
  if all(~isnan(Jcond_IKC(iIKCopt))) && any(Jcond_IKC_check(iIKCopt))
    [~,I_best_opt] = min(Jcond_IKC_check(iIKCopt));
  end
  % Nehme den Roboter mit der geringsten (bewegten) Masse. Kann unterschied-
  % lich sein, wenn Schubgelenke verschieden lang sind.
  mges_IKC_check = mges_IKC - mges_IKC(iIKCopt(1));
  mges_IKC_check(abs(mges_IKC_check)<1e-10) = 0;
  if all(~isnan(mges_IKC_check(iIKCopt))) && any(mges_IKC_check(iIKCopt))
    [~,I_best_opt] = min(mges_IKC_check(iIKCopt));
  end
  % Wenn es Schubgelenke gibt, nehme die Möglichkeit mit der geringsten
  % Auslenkung. Das ist am ehesten plausibel. Meistens betrifft es das
  % gestellfeste Gelenk.
  if I_best_opt==0 && any(R.MDH.sigma==1)
    qP_range_opt = NaN(length(iIKCopt), 1);
    for j = 1:length(iIKCopt)
      qP_range_opt(j) = max(max(abs(Q_IKC(:,R.MDH.sigma==1,iIKCopt(j)))));
    end
    [~,I_best_opt] = min(qP_range_opt); % Wähle die "beste" der "optimalen" Lösungen
  end
  if I_best_opt==0 % keine weiteren Kriterien definiert.
    I_best_opt = 1;% Nehme das erste (beliebig);
  end
  iIKCbest = iIKCopt(I_best_opt);

  if false % Debug: Prüfe, warum die eine Lösung besser als die andere ist
    cds_fitness_debug_plot_robot(R, Q_IKC(1,:,iIKCbest)', Traj_0, ...
      Traj_W, Set, Structure, p, mean(fval_IKC(iIKCbest,:)), debug_info); %#ok<UNRCH>
    figure(333);clf;
    for i = 1:size(TAU_IKC,2)
      for k = 1:length(I_IKC_iO)
        [~,I_max] = max(abs(TAU_IKC(:,i,I_IKC_iO(k))));
        subplot(size(TAU_IKC,2),1,i); hold on;
        plot(Traj_0.t, TAU_IKC(:,i,I_IKC_iO(I_IKC_iO(k))));
        plot(Traj_0.t(I_max), TAU_IKC(I_max,i,k), 'ko');
      end
      ylabel(sprintf('tau %d', i)); grid on;
    end
    linkxaxes
    figure(334);clf;
    for i = 1:R.NJ
      for k = 1:length(I_IKC_iO)
        subplot(ceil(sqrt(R.NJ)), ceil(R.NJ/ceil(sqrt(R.NJ))), i); hold on;
        plot(Traj_0.t, Q_IKC(:,i,I_IKC_iO(k)));
      end
      ylabel(sprintf('q %d', i)); grid on;
    end
    linkxaxes
  end
end
fval = fval_IKC(iIKCbest,:)';
physval = physval_IKC(iIKCbest,:)';
Jcond = Jcond_IKC(iIKCbest);
f_maxstrengthviol = f_maxstrengthviol_IKC(iIKCbest);
n_fval_iO = length(I_IKC_iO);

%% Ende
% Anfangs-Gelenkwinkel in Roboter-Klasse speichern (zur Reproduktion der
% Ergebnisse)
if R.Type == 0 % Seriell
  R.qref = Q0(iIKCbest,:)';
else
  for i = 1:R.NLEG, R.Leg(i).qref = Q0(iIKCbest,R.I1J_LEG(i):R.I2J_LEG(i))'; end
end
% Erneutes Eintragen der Gelenkgrenzen (so.o)
if R.Type == 0 % Seriell
   % Grenzen numerisch etwas erweitern. Dadurch kein Fehlschlag, falls
   % rundungsbedingte Abweichungen auftreten. Ansonsten dadurch Verletzung
   % der Grenzen möglich.
  R.qlim(R.MDH.sigma==1,:) = minmax2(Q_IKC(:,R.MDH.sigma==1, iIKC)') + ...
    1e-6*repmat([-1, 1], sum(R.MDH.sigma==1),1);
else % PKM
  for i = 1:R.NLEG
    Q_i = Q_IKC(:,R.I1J_LEG(i):R.I2J_LEG(i), iIKC);
    R.Leg(i).qlim(R.Leg(i).MDH.sigma==1,:) = minmax2(Q_i(:,R.Leg(i).MDH.sigma==1)') + ...
      1e-6*repmat([-1, 1], sum(R.Leg(i).MDH.sigma==1),1);
  end
end

if all(fval<1e3)
  cds_log(2,sprintf(['[fitness] Fitness-Evaluation in %1.1fs. fval=[%s]. Erfolg', ...
    'reich. %s Auswahl aus %d IK-Konfigurationen (davon %d i.O., %d optimal)'], ...
    toc(t1), disp_array(fval', '%1.3e'), fval_debugtext(2:end), ...
    size(fval_IKC,1), n_fval_iO, n_fval_opt));
else
  cds_log(2,sprintf('[fitness] Fitness-Evaluation in %1.1fs. fval=%1.3e. %s', ...
    toc(t1), fval(1), constrvioltext_IKC{iIKCbest}));
end
cds_fitness_debug_plot_robot(R, Q(1,:)', Traj_0, Traj_W, Set, Structure, p, mean(fval), debug_info);
cds_save_particle_details(Set, R, toc(t1), fval, p, physval, Jcond, f_maxstrengthviol);
rng('shuffle'); % damit Zufallszahlen in anderen Funktionen zufällig bleiben
