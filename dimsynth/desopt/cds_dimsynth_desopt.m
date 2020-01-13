% Entwurfsoptimierung für Robotermodell mit Optimierungsalgorithmus
% 
% Eingabe:
% R
%   Matlab-Klasse für Roboter (SerRob/ParRob)
% Traj_0
%   Roboter-Trajektorie (EE) bezogen auf Basis-KS des Roboters
% Q, QD, QDD
%   Gelenkwinkel-Trajektorie
% Jinvges
%   Zeilenweise (inverse) Jacobi-Matrizen des Roboters (für PKM). Bezogen
%   auf vollständige Gelenkgeschwindigkeiten und Plattform-Geschw. (in
%   x-Koordinaten, nicht: Winkelgeschwindigkeit)
% Set
%   Einstellungen des Optimierungsalgorithmus
% Structure
%   Eigenschaften der Roboterstruktur
% 
% Ausgabe:
% fval
%   Funktionswert für besten Parametersatz nach Optimierung
%   Gibt Aufschluss, ob alle Nebenbedingungen erfüllt werden konnten.
%   Werte: Siehe Ausgabe von cds_dimsynth_desopt_fitness
% Die anderen Ergebnisse werden in Roboterklasse `R` gespeichert

% Siehe auch: cds_dimsynth_robot.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-01
% (C) Institut für Mechatronische Systeme, Universität Hannover

function fval = cds_dimsynth_desopt(R, Traj_0, Q, QD, QDD, Jinv_ges, data_dyn, Set, Structure)
t1 = tic();
if Set.general.matfile_verbosity > 2
save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_desopt1.mat'));
end
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_desopt1.mat'));
% Dynamik-Parameter aktualisieren. Keine Nutzung der Ausgabe der Funktion
% (Parameter werden direkt in Klasse geschrieben; R.DesPar.seg_par ist
% vor/nach dem Aufruf unterschiedlich)

%% Optimierungsalgorithmus vorbereiten
options_desopt = optimoptions('particleswarm');
options_desopt.Display = 'off';
options_desopt.MaxIter = 5;
NumIndividuals = 20;
options_desopt.MaxStallIterations = 1; % Oft ist das Optimum der Startwert
options_desopt.ObjectiveLimit = 1e3; % Erfüllung aller Nebenbedingungen reicht
nvars = 2; % Variablen: Wandstärke, Durchmesser der Segmente

% Allgemeine Einstellungen (werden für serielle Roboter beibehalten)
varlim = [ 5e-3,  150e-3; ... % Grenzen für Wandstärke
          80e-3, 600e-3];  % Grenze für Durchmesser
if R.Type ~= 0 % Parallel
   % Bei PKM geringere Durchmesser (Aufteilung auf Beine, aber auch mehr
   % interne Verspannung)
  varlim = ceil(1e3*varlim/R.NLEG/2)*1e-3; % Aufrunden auf ganze Millimeter
end

% Erzeuge zufällige Startpopulation
options_desopt.SwarmSize = NumIndividuals;
InitPop = repmat(varlim(:,1)', NumIndividuals,1) + rand(NumIndividuals, nvars) .* ...
                        repmat(varlim(:,2)'-varlim(:,1)',NumIndividuals,1);
% Wähle nur plausible Anfangswerte
I_unplaus = InitPop(:,1) > InitPop(:,2)/2 - 1e-4;
InitPop(I_unplaus,1) = InitPop(I_unplaus,2)/2 + 1e-4; % Setze auf Vollmaterial und fast maximale Wandstärke
% Setze minimale und maximale Werte direkt ein (da diese oft das Optimum
% darstellen, wenn keine einschränkenden Nebenbedingungen gesetzt sind)
InitPop(1,:) = varlim(:,1); % kleinste Werte
InitPop(2,:) = varlim(:,2); % größte Werte
options_desopt.InitialSwarmMatrix = InitPop;
% Erstelle die Fitness-Funktion und führe sie einmal zu testzwecken aus
fitnessfcn_desopt=@(p_desopt)cds_dimsynth_desopt_fitness(R, Set, Traj_0, Q, QD, QDD, Jinv_ges, data_dyn, Structure, p_desopt(:));

% Prüfe, ob eine Entwurfsoptimierung sinnvoll ist
tic();
fval_minpar = fitnessfcn_desopt(InitPop(1,:)');
T2 = toc();
avoid_optimization = false;
if Set.optimization.desopt_link_yieldstrength && fval_minpar<1e3 && ...
  ~strcmp(Set.optimization.objective, 'stiffness') && Set.optimization.constraint_obj(5) == 0
  % Das schwächste Segment erfüllt alle Nebenbedingungen. Das Ergebnis muss
  % damit optimal sein (alle Zielfunktionen wollen Materialstärke minimieren)
  % Die Steifigkeit wird nicht betrachtet
  avoid_optimization = true;
  fval_opt = fval_minpar;
  p_val_opt = InitPop(1,:)';
end
if ~Set.optimization.desopt_link_yieldstrength && ...
  (strcmp(Set.optimization.objective, 'stiffness') || Set.optimization.constraint_obj(5) == 0)
  % Optimierung der Steifigkeit ohne Prüfung der Materialstärke. Die
  % stärkste Segmentauslegung könnte das Optimum darstellen.
  % Die Grenze der Masse wird betrachtet, da sie als Nebenbedingung bei
  % Prüfung der Fitness-Funktion enthalten ist.
  fval_maxpar = fitnessfcn_desopt(InitPop(2,:)');
  if fval_maxpar < 1e3
    avoid_optimization = true;
    fval_opt = fval_maxpar;
    p_val_opt = InitPop(2,:)';
  end
end
% Für Profiler: `for i=1:10,fitnessfcn_desopt(InitPop(1,:)'); end`
if Set.general.matfile_verbosity > 3
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_desopt2.mat'));
end
% Debug:
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_desopt2.mat'));
%% Optimierung der Entwurfsparameter durchführen
if ~avoid_optimization
  if Set.general.verbosity > 2
    fprintf('[desopt] Führe Entwurfsoptimierung durch. Dauer für eine Zielfunktionsauswertung: %1.1fs. Max. Dauer für Optimierung: %1.1fs (%d Iterationen, %d Individuen)\n', ...
      T2, NumIndividuals*(options_desopt.MaxIter+1)*T2, NumIndividuals, options_desopt.MaxIter);
  end
  [p_val,fval,~,output] = particleswarm(fitnessfcn_desopt,nvars,varlim(:,1),varlim(:,2),options_desopt);
  if fval < 1000
    detailstring = sprintf('Lösung gefunden (fval=%1.1f)', fval);
  else
    detailstring = sprintf('Keine zulässige Lösung gefunden (fval=%1.1e)', fval);
  end
else
  % Es wurde oben festgestellt, das die schwächstmögliche Dimensionierung
  % bereits optimal hinsichtlich der gewählten Zielfunktion+Nebenbedingung
  % ist
  p_val = p_val_opt;
  fval = fval_opt;
  output = struct('iterations', 0, 'funccount', 0);
  detailstring = sprintf('Keine Optimierung notwendig aufgrund der Definition des Optimierungsproblems (fval=%1.1f)', fval);
end
I_bordersol = any(repmat(p_val(:),1,2) == varlim,2); % Prüfe, ob Endergebnis eine Parameterraumgrenze darstellt
if any(I_bordersol)
  detailstring = [detailstring, sprintf('. Lösung bei %d/%d Par. an Grenze', sum(I_bordersol), length(I_bordersol))];
end
if Set.general.verbosity > 2
  fprintf('[desopt] Entwurfsoptimierung durchgeführt. Dauer: %1.1fs. %s. %d Iterationen, %d Funktionsauswertungen.\n', ...
    toc(t1), detailstring, output.iterations, output.funccount);
end

%% Ausgabe
% Belege die Robotereigenschaften mit dem Ergebnis der Optimierung
cds_dimsynth_design(R, Q, Set, Structure, p_val);

return
%% Debug

% Zeichne Verlauf der Fitness-Funktion
np1 = 8; np2 = 8; %#ok<UNRCH>
fval_grid = NaN(np1,np2);
p1_grid = linspace(varlim(1,1), varlim(1,2), np1)
p2_grid = linspace(varlim(2,1), varlim(2,2), np2)
for i = 1:np1
  for j = 1:np2
    p_ij = [p1_grid(i); p2_grid(j)];
    fval_grid(i,j) = fitnessfcn_desopt(p_ij);
  end
end
fval_grid(fval_grid==1e8) = NaN; % Damit unzulässiger Bereich im Plot leer bleibt
figure(10);clf;
hold on;
surf(1e3*p1_grid, 1e3*p2_grid, fval_grid, 'FaceAlpha',0.7);
xlabel('p1 (Wandstärke) in mm');
ylabel('p2 (Durchmesser) in mm');
zlabel('Zielfunktion Entwurfsoptimierung');
hdl=plot3(1e3*p_val(1), 1e3*p_val(2), fval, 'ro', 'MarkerSize', 8);
legend(hdl, {'Optimum'});
view(3)
