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

options_desopt.MaxStallIterations = 2; % Oft ist das Optimum der Startwert. Nicht sehr lange nach besseren Lösungen suchen.
nvars = length(vartypes); % Variablen: Wandstärke, Durchmesser der Segmente, Ruhelage von Gelenkfedern
if isnan(Set.optimization.desopt_NumIndividuals)
  NumIndividuals = 10*nvars;
else
  NumIndividuals = Set.optimization.desopt_NumIndividuals;
end
if isnan(Set.optimization.desopt_MaxIter)
  options_desopt.MaxIter = 5;
else
  options_desopt.MaxIter = Set.optimization.desopt_MaxIter;
end
varlim = [];
if any(vartypes == 2) % Dimensionierung der Segmente
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
if any(vartypes == 3) % Einbaulage von Gelenkfedern
  % Annahme: Roboter ist symmetrische PKM.
  I_actrevolute_opt = R.Leg(1).MDH.mu ~= 1 & R.Leg(1).DesPar.joint_type==0 & ...
    Set.optimization.joint_stiffness_active_revolute ~= 0;
  I_passrevolute_opt = R.Leg(1).MDH.mu == 1 & R.Leg(1).DesPar.joint_type==0 & ...
    Set.optimization.joint_stiffness_passive_revolute ~= 0;
  I_passuniversal_opt = R.Leg(1).MDH.mu == 1 & R.Leg(1).DesPar.joint_type==2 & ...
    Set.optimization.joint_stiffness_passive_universal ~= 0;
  I_joints = I_actrevolute_opt | I_passrevolute_opt | I_passuniversal_opt;
  % Bestimme die Grenzen der Gelenkkoordinaten der Beinketten. Fasse die
  % Gelenke jeder Beinkette zusammen, da eine symmetrische Anordnung für
  % die Feder-Mittelstellungen gesucht wird.
  qminmax_legs = reshape(minmax2(Q'),R.Leg(1).NJ,2*R.NLEG);
  qminmax_leg = minmax2(qminmax_legs);
  varlim_js_from_traj = qminmax_leg(I_joints,:);
  % Setze die Grenzen außerhalb des Bewegungsbereichs. Das erlaubt auch 
  % dauerhaft vorgespannte Gelenke, die eine Gravitationskompensation
  % bieten können. Die Stärke der möglichen Vorspannung ist stark von der
  % Steifigkeit in den Gelenken, der Zielfunktion und den Antrieben abhängig.
  % Beachte dabei die Nebenbedingung der Gelenkspannweite
  rangelim = Set.optimization.max_range_passive_revolute;
  if isnan(rangelim) || rangelim > pi
     % Nicht gesetzt. Keine Berücksichtigung und in folgender Formel 45°
     % über die Trajektorie in beide Richtungen hinaus.
    rangelim = -pi/4;
  end
  % Gehe so weit über die Verfahrbewegung hinaus, dass in dem Grenzfall der
  % Ruhelage die mögliche Spannweite maximal ausgenutzt wird. Ziehe 1e-3
  % von der Grenze ab, um Reserve für numerische Fehler zu haben. Sonst
  % schlagen spätere Tests an.
  varlim_from_jointrange = [ ...
                            varlim_js_from_traj(:,2)-rangelim+1e-3, ...
                            varlim_js_from_traj(:,1)+rangelim-1e-3];
  % Bestimme die Ober- und Untergrenze aus den beiden Fällen
  varlim_js_off = minmax2([varlim_js_from_traj, varlim_from_jointrange]);
  varlim = [varlim; varlim_js_off]; % in Grenzen für PSO eintragen
end
if any(vartypes == 4) % Steifigkeit von Gelenkfedern
  I_actrevolute_opt = R.Leg(1).MDH.mu ~= 1 & R.Leg(1).DesPar.joint_type==0 & ...
    isnan(Set.optimization.joint_stiffness_active_revolute);
  I_passrevolute_opt = R.Leg(1).MDH.mu == 1 & R.Leg(1).DesPar.joint_type==0 & ...
    isnan(Set.optimization.joint_stiffness_passive_revolute);
  I_passuniversal_opt = R.Leg(1).MDH.mu == 1 & R.Leg(1).DesPar.joint_type==2 & ...
    isnan(Set.optimization.joint_stiffness_passive_universal);
  I_joints = I_actrevolute_opt | I_passrevolute_opt | I_passuniversal_opt;
  % Maximale sinnvolle Gelenksteifigkeit einstellen. Für alle Gelenke gleich.
  varlim_js = repmat([0, Set.optimization.joint_stiffness_max], sum(I_joints), 1);
  varlim = [varlim; varlim_js]; % in Grenzen für PSO eintragen
end
assert(size(varlim,1)==nvars, 'Dimension von varlim passt nicht zu nvars');
% Erzeuge zufällige Startpopulation
options_desopt.SwarmSize = NumIndividuals;
InitPop = repmat(varlim(:,1)', NumIndividuals,1) + rand(NumIndividuals, nvars) .* ...
                        repmat(varlim(:,2)'-varlim(:,1)',NumIndividuals,1);
% Standardwert für Optimierungsparameter immer zum Vergleich rechnen
InitPop(end,:) = Structure.desopt_pdefault;
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
  % Eine weitere plausible Lösung liegt am Rand (dann maximaler Effekt der
  % Feder zur Gravitationskompensation)
  allcombinput = cell(1,size(varlim_js_off,1)); % Definiere die Eingabe als Cell
  for k = 1:length(allcombinput)
    allcombinput{k} = varlim_js_off(k,:)';
  end
  InitPop_js_border = allcomb(allcombinput{:});
  % Begrenze die Anzahl dieser Grenz-Partikel
  n_border = size(InitPop_js_border,1);
  if n_border > NumIndividuals-2
    n_border = NumIndividuals-2;
  end
  InitPop(3:3+n_border-1,vartypes==3) = InitPop_js_border(1:n_border,:);
  % Setze bei dem Rest der Population die Ruhelage geringfügig um
  % die Mittelstellung. Hier werden die besten Ergebnisse erwartet.
  n_red = NumIndividuals-2-n_border;
  varlim_js_red = repmat(InitPop(1,vartypes==3)',1,2) + repmat([-10, 10]*pi/180, sum(I_joints), 1);
  InitPop(3+n_border:end,vartypes==3) = repmat(varlim_js_red(:,1)', n_red,1) + ...
    rand(n_red, sum(vartypes==3) ).* repmat(varlim_js_red(:,2)'-varlim_js_red(:,1)',n_red,1);
end
options_desopt.InitialSwarmMatrix = InitPop;
% Erstelle die Fitness-Funktion und führe sie einmal zu testzwecken aus
clear cds_dimsynth_desopt_fitness % Für persistente Variablen von vorheriger Iteration in Maßsynthese
fval_main_dummy = NaN(length(Set.optimization.objective), 1);
cds_desopt_save_particle_details(0, 0, zeros(nvars,1), 0, fval_main_dummy, fval_main_dummy, 'reset', ... % Zurücksetzen der ...
  struct('comptime', NaN([options_desopt.MaxIter+1, NumIndividuals]))); % ... Detail-Speicherfunktion
fitnessfcn_desopt=@(p_desopt)cds_dimsynth_desopt_fitness(R, Set, Traj_0, Q, QD, QDD, Jinv_ges, data_dyn, Structure, p_desopt(:));
t2 = tic();
[fval_test, physval_test, abort_fitnesscalc] = fitnessfcn_desopt(InitPop(1,:)');
T2 = toc(t2);
% Prüfe, ob eine Entwurfsoptimierung sinnvoll ist (falls nur Segmentstärke)
avoid_optimization = false;
p_val_opt = NaN(nvars,1);
fval_opt = NaN;
detailstring = '';
if fval_test == 0 || abort_fitnesscalc
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
  % Wenn die Ruhelage die Mittelstellung ist, wird keine Lösung gefunden.
  % Nehme an, dass die PKM so schlecht dimensioniert ist, dass die Suche
  % nach der besten Feder-Ruhelage nicht sinnvoll ist. Das wird gemacht,
  % wenn die Grenzen stark verletzt werden. Wenn sie "fast" erreicht
  % werden, bringt die Optimierung voraussichtlich eine Lösung.
  if fval_test > 1e4 && physval_test > 5
    % Grenze für Materialspannung mit Faktor 5 überschritten.
    avoid_optimization = true;
    fval_opt = fval_test;
    p_val_opt = InitPop(1,:)';
    detailstring = sprintf(['Materialspannung bei mittiger Gelenkfeder-', ...
      'Ruhelage stark überschritten (Faktor %1.1f)'], physval_test);
  elseif fval_test > 1e3 && fval_test < 2e3 && physval_test > 2
    avoid_optimization = true;
    fval_opt = fval_test;
    p_val_opt = InitPop(1,:)';
    detailstring = sprintf(['Antriebskraft bei mittiger Gelenkfeder-', ...
      'Ruhelage stark überschritten (Faktor %1.1f)'], physval_test);
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
cds_desopt_save_particle_details(0, 0, zeros(nvars,1), 0, fval_main_dummy, fval_main_dummy, 'reset', ...
  struct('comptime', NaN([options_desopt.MaxIter+1, NumIndividuals])));
if ~avoid_optimization
  cds_log(3,sprintf(['[desopt] Führe Entwurfsoptimierung durch. Dauer für ', ...
    'eine Zielfunktionsauswertung: %1.1fms. Max. Dauer für Optimierung: ', ...
    '%1.1fs (%d+1 Iterationen, %d Individuen)'], 1e3*T2, NumIndividuals*...
    (options_desopt.MaxIter+1)*T2, options_desopt.MaxIter, NumIndividuals));
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
detailstring = [detailstring, sprintf('. Ergebnis: ')];
for i = 1:length(p_val)
  detailstring = [detailstring, sprintf('(%s: %1.1f)', Structure.desopt_pnames{i}, p_val(i))]; %#ok<AGROW>
  if i < length(p_val), detailstring = [detailstring, ', ']; end %#ok<AGROW>
end
cds_log(3,sprintf(['[desopt] Entwurfsoptimierung durchgeführt. %d Iter', ...
  'ationen, %d Funktionsauswertungen. Dauer: %1.1fs. %s.'], ...
    output.iterations, output.funccount, toc(t1), detailstring));
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
    I_actrevolute_opt = R.Leg(i).MDH.mu ~= 1 & R.Leg(i).DesPar.joint_type==0 & ...
      Set.optimization.joint_stiffness_active_revolute ~= 0;
    I_passrevolute_opt = R.Leg(i).MDH.mu == 1 & R.Leg(i).DesPar.joint_type==0 & ...
      Set.optimization.joint_stiffness_passive_revolute ~= 0;
    I_passuniversal_opt = R.Leg(i).MDH.mu == 1 & R.Leg(i).DesPar.joint_type==2 & ...
      Set.optimization.joint_stiffness_passive_universal ~= 0;
    I_update = I_actrevolute_opt | I_passrevolute_opt | I_passuniversal_opt;
    R.Leg(i).DesPar.joint_stiffness_qref(I_update) = p_val(vartypes==3);
  end
end
if any(vartypes == 4)
  for i = 1:R.NLEG
    I_actrevolute_opt = R.Leg(i).MDH.mu ~= 1 & R.Leg(i).DesPar.joint_type==0 & ...
      isnan(Set.optimization.joint_stiffness_active_revolute);
    I_passrevolute_opt = R.Leg(i).MDH.mu == 1 & R.Leg(i).DesPar.joint_type==0 & ...
      isnan(Set.optimization.joint_stiffness_passive_revolute);
    I_passuniversal_opt = R.Leg(i).MDH.mu == 1 & R.Leg(i).DesPar.joint_type==2 & ...
      isnan(Set.optimization.joint_stiffness_passive_universal);
    I_update = I_actrevolute_opt | I_passrevolute_opt | I_passuniversal_opt;
    R.Leg(i).DesPar.joint_stiffness(I_update) = p_val(vartypes==4);
  end
end

% Speichere die Zwischenergebnisse ab (falls notwendig für die Auswertung)
if Set.general.debug_desopt
  [currgen,currind,~,resdir] = cds_get_new_figure_filenumber(Set, Structure, '');
  name_matfile = sprintf('Gen%02d_Ind%02d_Konfig%d_desopt_dbg.mat', currgen, ...
    currind, Structure.config_index);
  % Extrahiere Detail-Daten aus den einzelnen PSO-Partikeln
  PSO_Detail_Data = cds_desopt_save_particle_details(0, 0, NaN, 0, fval_main_dummy, fval_main_dummy, 'output');
  % Nehme die Standard-Werte aus dem letzten Aufruf aus der Initial- 
  % Population (siehe InitPop).
  fval_main_default = PSO_Detail_Data.fval_main(1,:,end);
  physval_main_default = PSO_Detail_Data.physval_main(1,:,end);
  save(fullfile(resdir, name_matfile), 'PSO_Detail_Data', 'fval', 'p_val', ...
    'fval_main_default', 'physval_main_default');
end
return
%% Debug

% Rufe Fitness-Funktion mit bestem Partikel auf (zum Plotten von
% zusätzlichen Debug-Bildern).
Set_tmp = Set;
Set_tmp.general.plot_details_in_desopt = 1e10;
cds_dimsynth_desopt_fitness(R, Set_tmp, Traj_0, Q, QD, QDD, Jinv_ges, ...
  data_dyn, Structure, p_val);

% Zeichne Fitness-Fortschritt über Iteration des PSO
figure(9);clf;
axhdl = subplot(2,1,1);
plot(PSO_Detail_Data.fval, 'kx');
set(axhdl, 'yscale', 'log');
grid on;
ylabel('Alle Fitness-Werte (log)');
subplot(2,1,2);
plot(min(PSO_Detail_Data.fval,[],2), 'kx-');
ylabel('Bester Fitness-Wert (linear)');
xlabel('Generation');
grid on;
linkxaxes
sgtitle('Konvergenz der Fitness-Werte über die Optimierung');

% Zeichne Ausnutzung des Parameterraums
pval_stack_norm = NaN(size(PSO_Detail_Data.pval,1)*size(PSO_Detail_Data.pval,3), ...
  size(PSO_Detail_Data.pval,2));
pval_stack = pval_stack_norm;
for i = 1:size(PSO_Detail_Data.pval,2)
  pval_stack(:,i) = reshape(squeeze(PSO_Detail_Data.pval(:,i,:)), size(pval_stack,1),1);
  pval_stack_norm(:,i) = (pval_stack(:,i)-varlim(i,1))./... % untere Grenze abziehen
    repmat(varlim(i,2)-varlim(i,1),size(pval_stack,1),1); % auf 1 normieren
end
figure(8);clf;
plot(pval_stack_norm', 'x-');
xlabel('Optimierungsparameter');
set(gca, 'xtick', 1:length(p_val));
set(gca, 'xticklabel', Structure.desopt_pnames);
ylabel('Parameter Wert (normiert)');
title('Ausnutzung des möglichen Parameterraums');

% Zeichne Verlauf der Fitness-Funktion (für den Fall der
% Entwurfsoptimierung der Segmentdimensionierung
if all(vartypes == 2) && nvars == 2
  np1 = 8; np2 = 8;
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
end
