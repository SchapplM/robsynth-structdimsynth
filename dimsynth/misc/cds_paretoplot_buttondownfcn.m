% Auswertungsbilder nach Klick auf Punkt in Paretofront starten.
% Diese Funktion wird als ButtonDownFcn der Line-Objekte eingesetzt.
% 
% Eingabe:
% hObj
%   Handle zum Objekt, dass der ButtonDownFcn zugeordnet ist (Line-Objekt)
% event
%   Eigenschaft des Klick-Events
% OptName
%   Name der Optimierung (entspricht Ordner in Ergebnissen)
% RobName
%   Name des Roboters (zum Auffinden der Dateien)
% RobNr
%   Nummer des Roboters (zum Auffinden der Dateien)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function cds_paretoplot_buttondownfcn(hObj, event, OptName, RobName, RobNr)
t1=tic();
%% Finde die Koordinaten des geklickten Punkts auf der Pareto-Front
% https://de.mathworks.com/matlabcentral/answers/315760-find-out-which-point-is-clicked-in-axes
% https://de.mathworks.com/matlabcentral/answers/476609-how-to-get-the-point-under-mouse-click-on-the-plot-or-histogram#answer_388094
dataObjs = hObj;
xData = get(dataObjs, 'XData')';
yData = get(dataObjs, 'YData')';
pointsInPlot = [xData, yData];
clickedPoint = event.IntersectionPoint(1:2);
distances = sqrt(sum(bsxfun(@minus, pointsInPlot, clickedPoint).^2,2));
closest = pointsInPlot(distances==min(distances),:);
if size(closest,1) > 1
  warning(['Es wurden %d Punkte gleichzeitig erkannt. Liegen anscheinend ', ...
    'übereinander'],  size(closest,1));
  closest = closest(1,:);
end
% Index des geklickten Punkts entspricht Index in Ergebnis-Variablen
I_point = xData==closest(1) & yData==closest(2);
%% Auslesen des Menüs zur Entscheidung der Plot-Aktion
axhdl = get(hObj, 'Parent');
fighdl = get(axhdl, 'Parent');
uihdl = findobj(fighdl, 'Type', 'UIControl');
Selection = get(uihdl, 'Value');
SelStr = get(uihdl,'String');
fprintf('[%s] Starte Vorbereitung und Plot für %s/%s (Rob. %d) "%s"\n', ...
  datestr(now(),'yyyy-mm-dd HH:MM:SS'), OptName, RobName, RobNr, SelStr{Selection});
%% Lade die Daten
% Benutze den Ordner als Speicherort der Daten, in dem auch das Bild liegt.
resdir_opt = fileparts(get(fighdl, 'FileName'));
if isempty(resdir_opt)
  % Das Bild wurde eventuell gerade erst gezeichnet und daher ist kein
  % Dateiname abgespeichert. Suche den Ordner der Ergebnisse.
  resdir = fullfile(fileparts(which('structgeomsynth_path_init.m')), 'results');
  resdir_opt = fullfile(resdir, OptName);
  if ~exist(resdir_opt, 'file')
    warning('Automatisch ermittelter Ergebnis-Ordner %s existiert nicht. Abbruch.', resdir_opt);
    return
  end
elseif ~exist(resdir_opt, 'file')
  warning('Ergebnis-Ordner %s existiert nicht, obwohl Bild von dort geladen wurde. Abbruch.', resdir_opt);
  return
end
resfile1 = fullfile(resdir_opt, sprintf('Rob%d_%s_Endergebnis.mat', ...
  RobNr, RobName));
resfile2 = fullfile(resdir_opt, sprintf('Rob%d_%s_Details.mat', ...
  RobNr, RobName));
setfile = fullfile(resdir_opt, sprintf('%s_settings.mat', OptName));
if ~exist(resfile1, 'file')
  warning('Ergebnis-Datei %s existiert nicht.', resfile1);
  return
end
% if ~exist(resfile2, 'file')
%   warning('Ergebnis-Datei %s existiert nicht.', resfile2);
%   return
% end
if ~exist(setfile, 'file')
  warning('Einstellungs-Datei %s existiert nicht.', setfile);
  return
end
dres1 = load(resfile1, 'RobotOptRes');
RobotOptRes = dres1.RobotOptRes;
if exist(resfile2, 'file')
  dres2 = load(resfile2, 'RobotOptDetails', 'PSO_Detail_Data');
  RobotOptDetails = dres2.RobotOptDetails;
  PSO_Detail_Data = dres2.PSO_Detail_Data;
else
  RobotOptDetails = [];
  PSO_Detail_Data = [];
end
d3 = load(setfile, 'Traj', 'Set', 'Structures');
Set = d3.Set;
Traj = d3.Traj;
Structure = d3.Structures{RobNr};

% Ergebnistabelle laden
restabfile = fullfile(resdir_opt, sprintf('%s_results_table.csv', OptName));
opts = detectImportOptions(restabfile,'NumHeaderLines',2);
opts.VariableNamesLine = 1;
opts.VariableDescriptionsLine = 2;
ResTab = readtable(restabfile, opts);
% Ergebnis-Ordner lokal überschreiben (da neue Bilder gespeichert werden).
[resdir_tmp, optfolder] = fileparts(resdir_opt);
if ~strcmp(optfolder, OptName)
  error(['Der Ordnername der Optimierung heißt lokal anders, als in der ', ...
    'Datei: %s vs %s. Das gibt Probleme beim Speichern der Bilder. Abbruch.'], ...
    optfolder, OptName);
end
Set.optimization.resdir = resdir_tmp;

% Fehlende Felder in den Einstellungen ergänzen, sonst Fehler in Funktionen
Set = cds_settings_update(Set, 1);
% Debug-Plots teilweise deaktivieren, da hier nicht gewünscht.
Set.general.debug_taskred_perfmap = false;
%% Bestimme die Nummer des Pareto-Partikels
% Nummer istschon in I_point richtig sein, da xData und yData identisch
% mit den Werten aus fval_pareto (bzw. physval) sind.
% Wenn mehrere Punkte übereinander liegen, wird der erste genommen. Ist
% egal, da sie ja ein identisches Ergebnis haben.
PNr = find(I_point, 1, 'first');
fval = dres1.RobotOptRes.fval_pareto(PNr,:)';
if isempty(PNr)
  error('Kein Punkt in Pareto-Daten gefunden');
end
RobData = struct('Name', RobName, 'Number', RobNr, 'ParetoNumber', PNr, ...
  'Type', RobotOptRes.Structure.Type);
%% Berechne fehlende Größen
fitness_recalc_necessary = true;
if any(strcmp(SelStr(Selection), {'Pareto', 'Parameter', ...
    'Pareto Einfluss DesOpt', 'Pareto DesOpt'})) % Namen konsistent mit `SelStr`
  % Neuberechnung der Fitness-Funktion für diese Bilder nicht notwendig.
  fitness_recalc_necessary = false;
end
% Initialisiere Abhängigkeiten von cds_fitness und cds_dimsynth_robot
if Structure.Type == 0 % Seriell
  serroblib_update_template_functions({Structure.Name});
else % Parallel
  parroblib_update_template_functions({Structure.Name});
end
if fitness_recalc_necessary
  % Berechne die Fitness-Funktionen
  clear cds_save_particle_details cds_fitness cds_log
  p = RobotOptRes.p_val_pareto(PNr,:)';
  p_desopt = RobotOptRes.desopt_pval_pareto(PNr,:)';
  if any(isnan(p_desopt))
    warning('Ergebnisse für die Entwurfsoptimierung sind NaN.');
    p_desopt = [];
  end
  if isempty(RobotOptDetails) || ~isempty(p_desopt)
    % Falls nur die reduzierten Ergebnis-Daten vorliegen oder die Ergebnisse
    % der Entwurfsoptimierung direkt eingetragen werden sollen. Vermeide die
    % erneute Durchführung der Entwurfsoptimierung, falls diese gemacht wurde.
    [R, Structure] = cds_dimsynth_robot(Set, Traj, Structure, true);
    % Trage die gespeicherte Anfangswerte der Gelenkkonfigurationen in die
    % Roboter-Klasse ein. Dadurch wird wieder die gleiche IK-Konfiguration
    % getroffen.
    if isfield(RobotOptRes, 'q0_pareto')
      q0 = RobotOptRes.q0_pareto(PNr,:)';
    elseif ~isempty(RobotOptDetails)
      % Lade IK-Anfangswerte aus gespeicherten Daten
      [k_gen, k_ind] = cds_load_particle_details(PSO_Detail_Data, fval);
      q0 = PSO_Detail_Data.q0_ik(k_ind,:,k_gen)';
    else
      q0 = NaN(R.NJ,1);
    end
    if any(~isnan(q0))
      if R.Type == 0
        R.qref = q0;
      else
        for iLeg = 1:R.NLEG
          R.Leg(iLeg).qref = q0(R.I1J_LEG(iLeg):R.I2J_LEG(iLeg));
        end
      end
    end
    % Erzwinge Prüfung der gespeicherten Konfiguration. Ist teilweise
    % redundant zu Speicherung in qref. Führt Traj.-IK aus, auch wenn Pos.-
    % IK bei Umklappen der Konfiguration scheitert.
    Structure.q0_traj = q0;
  else
    % Alternative Berechnung (erfordert Laden der Detail-Daten).
    % Hier ist die Reproduktion der Zielfunktion besser möglich, da Anfangs-
    % werte für die Gelenkwinkel besser passen.
    R = RobotOptDetails.R;
    Structure = RobotOptRes.Structure;
  end
  % Falls Die Fitness-Funktion mit Debug-Einstellungen gestartet wird, muss
  % der Speicher-Ordner für die Bilder erstellt werden (Debug-Ordner)
  if ~isempty(Set.general.save_robot_details_plot_fitness_file_extensions)
    mkdirs(fullfile(Set.optimization.resdir, Set.optimization.optname, ...
      'tmp', sprintf('%d_%s', Structure.Number, Structure.Name)));
  end
  if isempty(p_desopt)
    [fval2, ~, Q, QD, QDD, TAU] = cds_fitness(R,Set,Traj,Structure,p);
  else
    % Keine erneute Entwurfsoptimierung, also auch keine Regressorform notwendig.
    % Direkte Berechnung der Dynamik, falls für Zielfunktion notwendig.
    Structure.calc_dyn_act = Structure.calc_dyn_act | Structure.calc_dyn_reg;
    Structure.calc_spring_act = Structure.calc_spring_act | Structure.calc_spring_reg;
    Structure.calc_spring_reg = false;
    Structure.calc_dyn_reg = false;
    [fval2, ~, Q, QD, QDD, TAU] = cds_fitness(R,Set,Traj,Structure,p,p_desopt);
  end
  if any(abs(fval-fval2) > 1e-4)
    warning(['Fitness-Wert von [%s] aus Pareto-Front konnte nicht reproduziert ', ...
      'werden (neu: [%s])'], disp_array(fval', '%1.4f'), disp_array(fval2', '%1.4f'));
  end
  if isempty(Q)
    warning('Keine Gelenkwinkel aus inverser Kinematik berechnet. Abbruch.');
    return
  end
  RobotOptDetails = struct('Traj_Q', Q, 'Traj_QD', QD, 'Traj_QDD', QDD, ...
    'R', R, 'Dyn_Tau', TAU);
else % Roboter-Klasse muss trotzdem neu erstellt werden.
  R = cds_dimsynth_robot(Set, Traj, Structure, true);
  RobotOptDetails = struct('R', R, 'Traj_Q', []);
end

%% Rufe die Plot-Funktion auf
t2=tic();
if strcmp(SelStr(Selection), 'Visualisierung')
  cds_vis_results_figures('robvisu', Set, Traj, RobData, ResTab, ...
    RobotOptRes, RobotOptDetails);
elseif strcmp(SelStr(Selection), 'Parameter')
  cds_vis_results_figures('optpar', Set, Traj, RobData, ResTab, ...
    RobotOptRes, RobotOptDetails);
elseif strcmp(SelStr(Selection), 'Kinematik')
  cds_vis_results_figures('jointtraj', Set, Traj, RobData, ResTab, ...
    RobotOptRes, RobotOptDetails);
elseif strcmp(SelStr(Selection), 'Animation')
  if isempty(Set.general.animation_styles)
    Set.general.animation_styles = {'3D'};
  end
  cds_vis_results_figures('animation', Set, Traj, RobData, ResTab, ...
    RobotOptRes, RobotOptDetails);
elseif strcmp(SelStr(Selection), 'Dynamik')
  cds_vis_results_figures('dynamics', Set, Traj, RobData, ResTab, ...
    RobotOptRes, RobotOptDetails);
elseif strcmp(SelStr(Selection), 'Dynamikparameter')
  cds_vis_results_figures('dynparvisu', Set, Traj, RobData, ResTab, ...
    RobotOptRes, RobotOptDetails);
elseif strcmp(SelStr(Selection), 'Feder-Ruhelage')
  cds_vis_results_figures('springrestpos', Set, Traj, RobData, ResTab, ...
    RobotOptRes, RobotOptDetails);
elseif strcmp(SelStr(Selection), 'Pareto')
  cds_vis_results_figures('pareto', Set, Traj, RobData, ResTab, ...
    RobotOptRes, RobotOptDetails, PSO_Detail_Data);
elseif strcmp(SelStr(Selection), 'Pareto Einfluss DesOpt')
  cds_vis_results_figures('pareto_dimsynth_desopt', Set, Traj, RobData, ResTab, ...
    RobotOptRes, RobotOptDetails, PSO_Detail_Data);
elseif strcmp(SelStr(Selection), 'Pareto DesOpt')
  cds_vis_results_figures('pareto_desopt', Set, Traj, RobData, ResTab, ...
    RobotOptRes, RobotOptDetails, PSO_Detail_Data);
elseif strcmp(SelStr(Selection), 'Redundanzkarte')
  task_red = R.Type == 0 && sum(R.I_EE_Task) < R.NJ || ... % Seriell: Redundant wenn mehr Gelenke als Aufgaben-FG
             R.Type == 2 && sum(R.I_EE_Task) < sum(R.I_EE); % Parallel: Redundant wenn mehr Plattform-FG als Aufgaben-FG
  if ~task_red
    error('Redundanzkarte für %s nicht sinnvoll (keine Redundanz)', RobName);
  end
  % Suche die gespeicherten Werte der Redundanzkarte in den Debug-Dateien
  % Prüfe, ob Bild erzeugt werden kann.
  tmpdir = fullfile(resdir_opt, 'tmp', sprintf('%d_%s', RobNr, RobName));
  % Lade bereits während der Optimierung generierte Redundanzkarte aus
  % gespeicherten Daten.
  if ~isempty(PSO_Detail_Data) % daraus Gen.-/Ind.-Nummer bestimmbar
    [k_gen, k_ind] = cds_load_particle_details(PSO_Detail_Data, fval);
    matcand = dir(fullfile(tmpdir, sprintf( ...
      'Gen%02d_Ind%02d_Konfig*_TaskRedPerfMap_Data.mat', k_gen, k_ind)));
  else
    matcand = dir(fullfile(tmpdir, 'Gen*_Ind*_Konfig*_TaskRedPerfMap_Data.mat'));
  end
  % Öffne mat-Dateien und prüfe, ob passende Daten zur Reproduktion des
  % Bildes enthalten sind. Benutze dafür die gespeicherten Gelenkwinkel und
  % den Zielfunktionswert (aus cds_constraints_traj), da es mehrere
  % passende Gelenkwinkel geben könnte. Nehme dann den besten F.-Wert unter
  % der Annahme, dass dieser der gewählten Alternative entspricht.
  i_bestf = 0; % Index in mat-Dateien für besten Funktionswert
  bestf = inf; % bester Funktionswert (mit passendem q)
  bestf_filename = '';
  bestf_qdist = inf; % Gelenkwinkel-Distanz dazu
  bestqdist = inf; % Beste Gelenkwinkel-Distanz unabhängig vom F.-Wert
  n_iOmat = 0; % Anzahl der i.O.-mat-Dateien (für Textausgabe)
  dlist = NaN(length(matcand), 2); % Speichere Werte für f und qdist ab
  for i = 1:length(matcand)
    d = load(fullfile(tmpdir, matcand(i).name), 'q');
    dist_qi = max(abs((Q(1,:)'-d.q))); % exakte Reproduktion müsste möglich sein
    dlist(i,:) = [dist_qi, NaN];
    if dist_qi < bestqdist
      bestqdist = dist_qi;
    end
    if dist_qi > 1e-1
      continue % Geladene Datei passt nicht
    end
    n_iOmat = n_iOmat + 1;
    % Lade Trajektorien-Dateien
    [tokens_genind,~] = regexp(matcand(i).name,'^Gen(\d+)_Ind(\d+)_','tokens','match');
    k_gen2 = str2double(tokens_genind{1}{1});
    k_ind2 = str2double(tokens_genind{1}{2});
    if ~isempty(PSO_Detail_Data)
      assert(k_gen2==k_gen&&k_indk_ind2==k_ind, 'Geladene Gen.-/Ind.-Nummer falsch');
    end
    [tokens_mat,~] = regexp(matcand(i).name,sprintf(['Gen%02d_Ind%02d_', ...
      'Konfig(\\d+)_TaskRedPerfMap_Data'],k_gen2,k_ind2),'tokens','match');
    k_conf = str2double(tokens_mat{1}{1});
    matcand2 = dir(fullfile(tmpdir, sprintf( ...
      'Gen%02d_Ind%02d_Konfig%d_TaskRed_Traj*.mat', k_gen2, k_ind2, k_conf)));
    for i2 = 1:length(matcand2)
      d2 = load(fullfile(tmpdir, matcand2(i2).name), 'q', 'fval');
      dlist(i,2) = d2.fval;
      assert(all(abs(d.q-d2.q)<1e-8), 'Geladene Daten nicht konsistent');
      if d2.fval < bestf % besserer Kandidat für Werte, die zum Pareto-Punkt gehören
        bestf = d2.fval;
        bestf_filename = matcand2(i2).name;
        bestf_qdist = dist_qi;
        i_bestf = i;
      end
    end
  end
  % Trajektorie neu berechnen
  Traj_0 = cds_transform_traj(R, Traj);
  [X2, XD2] = R.fkineEE2_traj(Q, QD); nt_red = size(Traj.X,1);
  X2(:,6) = denormalize_angle_traj(X2(:,6));
  
  if n_iOmat > 0
    fprintf(['Gespeicherter Wert für Redundanzkarte in Datei %s und %s ', ...
      'mit Gelenkwinkel-Abstand %1.2e (aus %d Kandidaten)\n'], ...
      matcand(i_bestf).name, bestf_filename, bestf_qdist, n_iOmat);
    % erst hier vollständig laden
    d = load(fullfile(tmpdir, matcand(i_bestf).name));
    d2 = load(fullfile(tmpdir, bestf_filename));
    wn = d2.s.wn;
    h1 = d2.Stats.h(:,1);
    % Plausibilisierung der geladenen Daten
    test_X2 = [d2.X2(:,1:3)-X2(:,1:3), angleDiff(d2.X2(:,4:6), X2(:,4:6))];
    if any(abs(test_X2(:)) > 1e-10)
      warning(['EE-Trajektorie nicht reproduzierbar, trotz gleichem ', ...
        'Anfangswert. Fehler: %1.2e'], max(abs(test_X2(:))));
    end
    test_Q = d2.Q - Q;
    if any(abs(test_Q(:)) > 1e-10)
      warning(['Gelenk-Trajektorie nicht reproduzierbar, trotz gleichem ', ...
        'Anfangswert. Fehler: %1.2e'], max(abs(test_Q(:))));
    end
  end
  if n_iOmat == 0
    fprintf(['Keine vorab generierten Daten zur Redundanzkarte in %d mat-', ...
      'Dateien gefunden. Nächster Wert für Gelenkwinkel mit Abstand %1.2e. ', ...
      'Neuberechnung der Karte.\n'], bestqdist, length(matcand));
    % Bild neu generieren
    d2 = struct('X2', X2);
    [d.H_all, ~, d.s_ref, d.s_tref, d.phiz_range] = R.perfmap_taskred_ik( ...
      Traj_0.X(1:nt_red,:), Traj_0.IE(Traj_0.IE~=0), struct( ...
      'q0', Q(1,:)', 'I_EE_red', Set.task.DoF, 'map_phistart', d.X2(1,end), ...
      ... % nur grobe Diskretisierung für die Karte (geht schneller)
      'mapres_thresh_eepos', 10e-3, 'mapres_thresh_eerot', 5*pi/180, ...
      'mapres_thresh_pathcoordres', 0.2, 'mapres_redcoord_dist_deg', 5, ...
      'maplim_phi', [-1, +1]*210*pi/180, ... % 210° statt 180° als Grenze
      'verbose', true));
    wn = zeros(20,1); % Platzhalter-Variable
    h1 = NaN(size(d2.X2,1),1);
  end
  % Rechne die IK-Kriterien von Traj.- zu Pos.-IK um. Siehe cds_constraints_traj
  i=0; I_wn_traj = zeros(R.idx_ik_length.wnpos,1);
  for f = fields(RS.idx_ikpos_wn)'
    i=i+1; I_wn_traj(i) = R.idx_iktraj_wnP.(f{1});
  end
  % Bild zeichnen
  cds_debug_taskred_perfmap(Set, Structure, d.H_all, d.s_ref, d.s_tref(:), ...
    d.phiz_range, d2.X2(:,6), h1, struct('wn', wn(I_wn_traj), 'i_ar', 0, ...
    'critnames', {fields(R.idx_ikpos_wn)'}));
else
  error('Fall %s nicht vorgesehen. Versionsfehler?', SelStr{Selection});
end
fprintf(['Bilder gezeichnet. Dauer: %1.1fs zur Vorbereitung, %1.1fs zum ', ...
  'Zeichnen. Speicherort: %s\n'], toc(t1)-toc(t2), toc(t2), fullfile( ...
  Set.optimization.resdir, Set.optimization.optname, sprintf('Rob%d_%s', ...
  RobNr, RobName)));
