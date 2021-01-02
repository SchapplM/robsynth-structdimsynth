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
% p_val
%   Parameter als Ergebnis der Entwurfsoptimierung. Die Parameter werden
%   zusätzlich in der Roboterklasse `R` gespeichert.
% vartypes
%   Variablen-Typen in der Ausgabe `p_val`. Entspricht Structure.desopt_ptypes.
%   Kann unterschiedlich sein, da nicht alle Entwurfsparameter hier optimiert werden.

% Siehe auch: cds_dimsynth_robot.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-01
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [fval, p_val, vartypes] = cds_dimsynth_desopt(R, Traj_0, Q, QD, QDD, Jinv_ges, data_dyn, Set, Structure)
t1 = tic();
if Set.general.matfile_verbosity > 2
save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_desopt1.mat'));
end
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_desopt1.mat'));
% Dynamik-Parameter aktualisieren. Keine Nutzung der Ausgabe der Funktion
% (Parameter werden direkt in Klasse geschrieben; R.DesPar.seg_par ist
% vor/nach dem Aufruf unterschiedlich)

%% Optimierungsalgorithmus vorbereiten
% Wähle die Variablen der Entwurfsoptimierung aus (siehe
% cds_dimsynth_robot)
vartypes = Structure.desopt_ptypes(Structure.desopt_ptypes~=1);
options_desopt = optimoptions('particleswarm');
options_desopt.Display = 'off';
options_desopt.MaxIter = 5;
options_desopt.MaxStallIterations = 1; % Oft ist das Optimum der Startwert
nvars = length(vartypes); % Variablen: Wandstärke, Durchmesser der Segmente, Ruhelage von Gelenkfedern
NumIndividuals = 10*nvars;

varlim = [];
if any(vartypes == 2)
  % Allgemeine Einstellungen (werden für serielle Roboter beibehalten)
  varlim_ls = [ 5e-3, 150e-3; ... % Grenzen für Wandstärke
               80e-3, 600e-3];  % Grenze für Durchmesser
  if R.Type ~= 0 % Parallel
     % Bei PKM geringere Durchmesser (Aufteilung auf Beine, aber auch mehr
     % interne Verspannung)
    varlim_ls = ceil(1e3*varlim_ls/R.NLEG/2)*1e-3; % Aufrunden auf ganze Millimeter
  end
  varlim = [varlim; varlim_ls];
end
if any(vartypes == 3)
  % Annahme: Roboter ist symmetrische PKM.
  I_joints = R.Leg(1).MDH.sigma==0;
  % Bestimme die Grenzen der Gelenkkoordinaten der Beinketten. Fasse die
  % Gelenke jeder Beinkette zusammen, da eine symmetrische Anordnung für
  % die Feder-Mittelstellungen gesucht wird.
  qminmax_legs = reshape(minmax2(Q'),R.Leg(1).NJ,2*R.NLEG);
  qminmax_leg = minmax2(qminmax_legs);
  varlim_js = qminmax_leg(I_joints,:);
  % Setze die Grenzen außerhalb des Bewegungsbereichs. Das erlaubt auch 
  % dauerhaft vorgespannte Gelenke, die eine Gravitationskompensation
  % bieten können. Die Stärke der möglichen Vorspannung ist stark von der
  % Steifigkeit in den Gelenken, der Zielfunktion und den Antrieben abhängig.
  varlim_js = varlim_js + repmat([-45, 45]*pi/180, sum(I_joints), 1);
  varlim = [varlim; varlim_js];
end
% Erzeuge zufällige Startpopulation
options_desopt.SwarmSize = NumIndividuals;
InitPop = repmat(varlim(:,1)', NumIndividuals,1) + rand(NumIndividuals, nvars) .* ...
                        repmat(varlim(:,2)'-varlim(:,1)',NumIndividuals,1);
% Wähle nur plausible Anfangswerte
if any(vartypes == 2)
  IIls = find(vartypes==2);
  I_unplaus = InitPop(:,IIls(1)) > InitPop(:,IIls(2))/2;
  InitPop(I_unplaus,IIls(1)) = InitPop(I_unplaus,IIls(2))/2; % Setze auf Vollmaterial
  % Setze minimale und maximale Werte direkt ein (da diese oft das Optimum
  % darstellen, wenn keine einschränkenden Nebenbedingungen gesetzt sind)
  InitPop(1,IIls) = varlim(IIls,1); % kleinste Werte
  InitPop(2,IIls) = varlim(IIls,2); % größte Werte
end
if any(vartypes == 3)
  % Setze die mittlere Gelenkstellung als ein Wert ein. Es wird erwartet,
  % dass dies (ungefähr) der beste ist. Eintrag als erstes Individuum,
  % damit es beim testweisen Aufruf der Funktion geprüft wird.
  InitPop(1,vartypes==3) = mean(qminmax_leg(I_joints,:),2);
  if any(vartypes == 2)
    % entspricht stärkstmöglicher Dimensionierung der Segmente. Kombinere
    % mit Mittelstellung der Feder-Ruhelage zur Vorab-Schätzung.
    InitPop(2,vartypes==3) = InitPop(1,vartypes==3);
  end
  % Setze bei der ersten Hälfte der Population die Ruhelage geringfügig um
  % die Mittelstellung. Hier werden die besten Ergebnisse erwartet.
  n_red = floor(NumIndividuals/2)-2;
  varlim_js_red = repmat(InitPop(1,vartypes==3)',1,2) + repmat([-10, 10]*pi/180, sum(I_joints), 1);
  InitPop(3:3+n_red-1,vartypes==3) = repmat(varlim_js_red(:,1)', n_red,1) + ...
    rand(n_red, sum(vartypes==3) ).* repmat(varlim_js_red(:,2)'-varlim_js_red(:,1)',n_red,1);
end
options_desopt.InitialSwarmMatrix = InitPop;
% Erstelle die Fitness-Funktion und führe sie einmal zu testzwecken aus
clear cds_dimsynth_desopt_fitness % Für persistente Variablen von vorheriger Iteration in Maßsynthese
fitnessfcn_desopt=@(p_desopt)cds_dimsynth_desopt_fitness(R, Set, Traj_0, Q, QD, QDD, Jinv_ges, data_dyn, Structure, p_desopt(:));
t2 = tic();
[fval_test, physval_ms_test] = fitnessfcn_desopt(InitPop(1,:)');
T2 = toc(t2);
% Prüfe, ob eine Entwurfsoptimierung sinnvoll ist (falls nur Segmentstärke)
avoid_optimization = false;
p_val_opt = NaN(nvars,1);
fval_opt = NaN;
detailstring = '';
if fval_test == 0
  % Die Abbruchbedingung in der Fitness-Funktion wurde bereits beim Start-
  % wert erfüllt. Die Optimierung ist nicht notwendig (nur Nebenbedingungen
  % prüfen, keine Optimierung einer Gütefunktion).
  avoid_optimization = true;
  fval_opt = fval_test;
  p_val_opt = InitPop(1,:)';
  detailstring = 'Abbruchbedingung bereits bei erstem Versuch erfüllt';
elseif all(vartypes == 2) % Nur Segmentstärke wird optimiert
  fval_minpar = fval_test; % Aufruf oben mit InitPop(1,:) entspricht schwächstem Wert
  if Set.optimization.constraint_obj(6) > 0 && fval_minpar<1e3 && ...
    ~strcmp(Set.optimization.objective, 'stiffness') && Set.optimization.constraint_obj(5) == 0
    % Das schwächste Segment erfüllt alle Nebenbedingungen. Das Ergebnis muss
    % damit optimal sein (alle Zielfunktionen wollen Materialstärke minimieren)
    % Die Steifigkeit wird nicht betrachtet
    avoid_optimization = true;
    fval_opt = fval_minpar;
    p_val_opt = InitPop(1,:)';
    detailstring = 'Schwächste Segmentdimensionierung erfüllt bereits die Nebenbedingungen';
  end
  clear cds_dimsynth_desopt_fitness % für persistente Variable
  fval_maxpar = fitnessfcn_desopt(InitPop(2,:)');
  if fval_minpar > 1e4 && fval_maxpar > 1e4
    % Sowohl die schwächstmögliche, als auch die stärkstmögliche
    % Dimensionierung verletzen die Materialspannung. Annahme, dass eine
    % stärkere Struktur die Festigkeit mehr erhöht als die aus der
    % zusätzlichen Masse resultierende Belastung.
    avoid_optimization = true;
    fval_opt = fval_maxpar;
    p_val_opt = InitPop(2,:)';
    detailstring = 'Materialspannung auch bei stärkster Segmentdimensionierung überschritten';
  end
  if Set.optimization.constraint_obj(6) == 0 && ...
    (strcmp(Set.optimization.objective, 'stiffness') || Set.optimization.constraint_obj(5) == 0)
    % Optimierung der Steifigkeit ohne Prüfung der Materialstärke. Die
    % stärkste Segmentauslegung könnte das Optimum darstellen.
    % Die Grenze der Masse wird betrachtet, da sie als Nebenbedingung bei
    % Prüfung der Fitness-Funktion enthalten ist.
    if fval_maxpar < 1e3
      avoid_optimization = true;
      fval_opt = fval_maxpar;
      p_val_opt = InitPop(2,:)';
      detailstring = 'Nebenbedingung bei stärkster Segmentdimensionierung erfüllt';
    end
  end
elseif all(vartypes == 3) % Nur Gelenkfeder-Ruhelagen werden optimiert
  if fval_test > 1e4 && physval_ms_test > 5
    % Wenn die Ruhelage die Mittelstellung ist, wird keine Lösung gefunden.
    % Nehme an, dass die PKM so schlecht dimensioniert ist, dass die Suche
    % nach der besten Feder-Ruhelage nicht sinnvoll ist. Grenze mit Faktor
    % 5 überschritten.
    avoid_optimization = true;
    fval_opt = fval_test;
    p_val_opt = InitPop(1,:)';
    detailstring = 'Materialspannung bei mittiger Gelenkfeder-Ruhelage stark überschritten';
  end
elseif any(vartypes == 2) && any(vartypes == 3) % Gemeinsame Optimierung
  clear cds_dimsynth_desopt_fitness % für persistente Variable
  fval_minpar = fval_test;
  fval_maxpar = fitnessfcn_desopt(InitPop(2,:)');
  if fval_minpar > 1e4 && fval_maxpar > 1e4
    % Siehe gleiche Überprüfung oben. Annahme, dass bei
    % Gelenkfeder-Mittelstellung die stärksten Segmente reichen müssten.
    % Wähle daher auch die stärkste Dimensionierung als Ergebnis.
    avoid_optimization = true;
    fval_opt = fval_maxpar;
    p_val_opt = InitPop(2,:)';
    detailstring = 'Materialspannung auch bei stärkster Segmentdimensionierung überschritten';
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
  cds_log(3,sprintf(['[desopt] Führe Entwurfsoptimierung durch. Dauer für ', ...
    'eine Zielfunktionsauswertung: %1.1fms. Max. Dauer für Optimierung: ', ...
    '%1.1fs (%d Iterationen, %d Individuen)'], 1e3*T2, NumIndividuals*...
    (options_desopt.MaxIter+1)*T2, NumIndividuals, options_desopt.MaxIter));
  clear cds_dimsynth_desopt_fitness % für persistente Variable
  [p_val,fval,~,output] = particleswarm(fitnessfcn_desopt,nvars,varlim(:,1),varlim(:,2),options_desopt);
  p_val = p_val(:);
  if fval < 1000
    detailstring = sprintf('Lösung gefunden (fval=%1.1f)', fval);
  else
    detailstring = sprintf('Keine zulässige Lösung gefunden (fval=%1.1e)', fval);
  end
else
  % Es wurde oben festgestellt, das die ausprobierte Dimensionierung
  % bereits optimal hinsichtlich der gewählten Zielfunktion+Nebenbedingung
  % ist. Bei reiner Segment-Optimierung entspricht dies der schwächsten
  % Dimensionierung. Bei Optimierung der Gelenkfeder-Ruhelagen der Mittel-
  % stellung in der Trajektorie.
  p_val = p_val_opt;
  fval = fval_opt;
  output = struct('iterations', 0, 'funccount', 0);
  detailstring = sprintf('Keine Optimierung notwendig (fval=%1.1f). %s', fval, detailstring);
end
I_bordersol = any(repmat(p_val(:),1,2) == varlim,2); % Prüfe, ob Endergebnis eine Parameterraumgrenze darstellt
if any(I_bordersol)
  detailstring = [detailstring, sprintf('. Lösung bei %d/%d Par. an Grenze', sum(I_bordersol), length(I_bordersol))];
end
cds_log(3,sprintf('[desopt] Entwurfsoptimierung durchgeführt. Dauer: %1.1fs. %s. %d Iterationen, %d Funktionsauswertungen.', ...
    toc(t1), detailstring, output.iterations, output.funccount));
% Debug: Fitness-Funktion mit bestem Ergebnis nochmal aufrufen. Mit Bildern
% Set.general.plot_details_in_desopt = 1e3;
% Set.general.plot_details_in_fitness = 1e3;
% cds_dimsynth_desopt_fitness(R, Set, Traj_0, Q, QD, QDD, Jinv_ges, data_dyn, Structure, p_val(:))
%% Ausgabe
% Belege die Robotereigenschaften mit dem Ergebnis der Optimierung
if any(vartypes == 2)
  cds_dimsynth_design(R, Q, Set, Structure, p_val(vartypes==2));
end
if any(vartypes == 3)
  for i = 1:R.NLEG
    R.Leg(i).DesPar.joint_stiffness_qref(R.Leg(i).MDH.sigma==0) = p_val(vartypes==3);
  end
end
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
