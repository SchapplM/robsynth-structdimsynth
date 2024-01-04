% Anfangspopulation der zu optimierenden Parameter bestimmen.
% Benutze bereits durchgeführte Optimierungen sowie Zufallszahlen.
% 
% Eingabe:
% Set
%   Einstellungen des Optimierungsalgorithmus aus cds_settings_defaults
% Structure
%   Eigenschaften der Roboterstruktur
% 
% Ausgabe:
% InitPop
%   Anfangspopulation für PSO-Optimierung
% Q_Pop
%   Gelenk-Konfigurationen für die Parameter aus InitPop zu gespeicherten
%   Ergebnissen führen (NaN, falls keine Daten vorliegen).
% 
% Siehe auch: cds_gen_init_pop_index

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [InitPop, Q_Pop] = cds_gen_init_pop(Set, Structure, Traj)
nIndTotal = Set.optimization.NumIndividuals;
varlim = Structure.varlim;
varnames = Structure.varnames;
nvars = length(varnames);
%% Lade Ergebnisse bisheriger Optimierungen aus dem Ergebnis-Ordner
t1 = tic();
counter_optresults = 0;
InitPopLoadTmp = [];
Q_PopTmp = [];
OptNamesTmp = {};
RobNamesTmp = {};
ScoreLoad = [];
resdir_main = fullfile(Set.optimization.resdir, Set.optimization.optname);
if any(strcmp(Set.optimization.objective,'valid_act')) && Structure.Type==2
  % Bei Struktursynthese sind die Ergebnisse bei einer anderen Aktuierung
  % auch verwertbar. Nehme nur den Roboternahmen ohne "A"-Suffix
  [~,~,~,~,~,~,~,RobName] = parroblib_load_robot(Structure.Name, 0);
else
  RobName = Structure.Name;
end
if Set.optimization.InitPopRatioOldResults == 0
  % Es sollen keine alten Ergebnisse geladen werden. Kein Durchsuchen der
  % Ordner notwendig.
  Set.optimization.result_dirs_for_init_pop = {};
  initpop_matlist = {};
else
  % Lade bisherige Liste vorhandener Ergebnisse, erstellt von cds_gen_init_pop_index
  if Set.optimization.InitPopFromGlobalIndex
    filename_idx = fullfile(Set.optimization.resdir, 'index_results.mat');
  else
    filename_idx = fullfile(resdir_main, 'tmp', 'old_results.mat');
  end
  initpop_matlist = {};
  if exist(filename_idx, 'file')
    try
      tmp = load(filename_idx, 'initpop_matlist');
      initpop_matlist = tmp.initpop_matlist;
    catch err % Auf dem Cluster kann es trotz vorher korrekt vorhandener Datei in seltenen Fällen zu Dateisystemfehlern kommen
      cds_log(-1, sprintf('[gen_init_pop] Fehler beim Öffnen des Index: %s', err.message));
    end
  else
    cds_log(-1, sprintf('[gen_init_pop] Datei %s existiert nicht.', filename_idx));
  end
end
%% Lade Ergebnisse aus dem Temp-Ordner
% Lade zusätzlich aus dem tmp-Ordner der aktuellen Optimierung die
% Zwischenergebnisse. Damit kann bei Fortsetzen einer abgebrochenen
% Optimierung der Stand weitergenutzt werden. Betrifft besonders die
% Fortsetzung bei Abbruch auf dem Cluster durch Cluster-Fehler.
tmpdirsrob = dir(fullfile(resdir_main, 'tmp', sprintf('*_%s', RobName)));
for i = 1:length(tmpdirsrob)
  % Lade Datei mit Zwischenständen der Optimierung
  genfiles = dir(fullfile(tmpdirsrob(i).folder, tmpdirsrob(i).name, ...
    '*_Gen*_AllInd.mat')); % aus cds_save_all_results_mopso
  [ttt, ~] = regexp(tmpdirsrob(i).name, ['(\d+)_', RobName], 'tokens', 'match');
  irob = str2double(ttt{1}{1});
  for j = 1:length(genfiles)
    try % Auf Cluster Dateisystem-Fehler möglich
      tmp = load(fullfile(genfiles(j).folder, genfiles(j).name));
    catch err
      cds_log(-1, sprintf('[gen_init_pop] Fehler beim Öffnen der Generations-Datei: %s.', err.message));
      continue
    end
    % Erzeuge eine Dummy-Datei mit den Daten der Generation, die dann
    % später wieder geladen werden kann.
    [ttt, ~] = regexp(genfiles(j).name, '_Gen(\d+)_', 'tokens', 'match');
    igen = str2double(ttt{1}{1});
    RobotOptRes = struct('Structure', Structure); % Annahme: Gleiche Einstellungen der Optimierung
    if size(tmp.PSO_Detail_Data.fval,2) > 1 % siehe cds_save_particle_details
      I_dom = pareto_dominance(tmp.PSO_Detail_Data.fval(:,:,1+igen)); % Sonst später Warnungen, da keine valide Pareto-Front
      % Entferne auch NaN-Einträge (erzeugen beim späteren Laden Probleme)
      % (treten auf, wenn es kein Status am Ende einer Generation ist,
      % sondern wenn die Berechnung in einer Generation vorzeitig abbricht)
      I_dom = I_dom | any(isnan(tmp.PSO_Detail_Data.fval(:,:,1+igen)),2);
      RobotOptRes.fval_pareto = tmp.PSO_Detail_Data.fval(~I_dom,:,1+igen);
      RobotOptRes.p_val_pareto = tmp.PSO_Detail_Data.pval(~I_dom,:,1+igen);
      RobotOptRes.desopt_pval_pareto = tmp.PSO_Detail_Data.desopt_pval(~I_dom,:,1+igen);
      RobotOptRes.q0_pareto = tmp.PSO_Detail_Data.q0_ik(~I_dom,:,1+igen);
      RobotOptRes.q0 = RobotOptRes.q0_pareto(1,:)';
      RobotOptRes.fval = RobotOptRes.fval_pareto(1,:)';
      RobotOptRes.timestamps_start_end = repmat(genfiles(j).datenum,1,2); % setze beides auf den Zeitstempel der Datei
    else
      % TODO: Fall noch nicht definiert.
      continue
    end
    filename_dummy = fullfile(resdir_main, sprintf( ...
      'Rob%d_%s_Endergebnis_Gen%d.mat', irob, RobName, igen));
    save(filename_dummy,'RobotOptRes');
    cds_log(3, sprintf(['[gen_init_pop] Ergebnis-Datei %s aus vorhandenem ', ...
      'Zwischenstand %s erstellt.'], filename_dummy, genfiles(j).name));
    initpop_matlist = [initpop_matlist; filename_dummy]; %#ok<AGROW> 
  end
end

%% Alle möglichen Ergebnis-Dateien durchgehen
if any(strcmp(Set.optimization.objective,'valid_act')) && Structure.Type==2
  % PKM-Struktursynthese: Suchbegriff enthält nicht die Aktuierung
  RobFilter  = ['_', RobName, 'A']; % Aktuierung hinzufügen, damit passend
else % Normale Maßsynthese. Begrenze Suchbegriff, damit nicht gierig zu viel gefunden wird
  if Structure.Type == 2 % PKM: Wähle auch Ergebnisse mit anderen Koppelgelenk-Anordnungen
    [~, ~, ~, ~, ~, ~, ~, ~, PName_Legs] = parroblib_load_robot(RobName, 0);
    RobFilter  = ['_', PName_Legs];
  else % Serielle Kinematik: Nur exakte Treffer nehmen
    RobFilter  = ['_', RobName, '_'];
  end
end
cds_log(2, sprintf(['[gen_init_pop] Suche nach Ergebnissen für \"*%s*\" in %d ' ...
  'Ergebnis-Dateien für Optimierungs-Parameter {%s}'], RobFilter, ...
  length(initpop_matlist), disp_array(Structure.varnames, '%s')));
I_RobMatch = contains(initpop_matlist, RobFilter);
for i = find(I_RobMatch)'% Unterordner durchgehen.
  dirname_i = fileparts(initpop_matlist{i});
  [~,optname_tmp] = fileparts(dirname_i);
  setfile_i = fullfile(dirname_i, [optname_tmp,'_settings.mat']);
  if ~exist(setfile_i, 'file') % Keine Einstellungsdatei. Ungültig.
    continue % wenn es mehr als eine gibt
  end
  % fprintf('Daten für Roboter %s gefunden (%s)\n', RobName, dirname_i);
  % Gehe alle Ergebnisdateien zu dem Roboternamen durch. Es kann mehrere
  % geben (bei Struktursynthese oder bei paralleler Optimierung eines
  % einzigen Roboters
  score_i = 0; % Bewertung der Vergleichbarkeit dieser Optimierung mit der aktuellen Optimierung
  % Daten laden (Keine Abbruchbedingung)
  try % Auf Cluster teilweise Probleme mit Dateizugriff.
    d = load(initpop_matlist{i});
  catch err %  Ist hier nicht so schlimm, falls übersprungen wird.
    cds_log(-1, sprintf(['[gen_init_pop] Datei %s konnte nicht geladen ', ...
      'werden. Fehler: %s'], initpop_matlist{i}, err.message));
    continue
  end
  if ~isfield(d.RobotOptRes, 'p_val_pareto') % (Altes Dateiformat. Dieser Code kann irgendwann weg)
    cds_log(2, sprintf(['[gen_init_pop] Datei übersprungen, da Feld ', ...
      'p_val_pareto fehlt: %s '], initpop_matlist{i}));
    continue
  end
  % Platzhalter für Detail-Datei
  d2 = [];
  file2 = strrep(initpop_matlist{i}, 'Endergebnis', 'Details');
  if ~isfield(d.RobotOptRes, 'q0') % Altes Dateiformat, aber eventuell korrigierbar (Dieser Code kann irgendwann weg, fehlende Daten führen dann zum Ausschluss)
    % Versuche die fehlende Information aus zweiter Datei zu laden
    if exist(file2, 'file')
      try
        d2 = load(file2);
      catch err
        cds_log(-1, sprintf(['[gen_init_pop] Datei %s konnte nicht geladen ', ...
          'werden. Fehler: %s. Dann keine Rekonstruktion von q0.'], file2, err.message));
      end
      if ~isempty(d2)
        % Suche die Parameter-Werte in den gespeicherten Zwischenständen, um
        % die IK-Startkonfiguration zu rekonstruieren.
        q0_pareto_i = NaN(size(d.RobotOptRes.p_val_pareto, 1), size(d2.PSO_Detail_Data.q0_ik, 2));
        for k = 1:size(d.RobotOptRes.p_val_pareto, 1)
          try
            [k_gen, k_ind] = cds_load_particle_details(struct('fval',d2.PSO_Detail_Data.pval), ...
              d.RobotOptRes.p_val_pareto(k,:)');
            q0_pareto_i(k,:) = d2.PSO_Detail_Data.q0_ik(k_ind,:,k_gen);
          catch err
            cds_log(-1, sprintf(['[gen_init_pop] In Datei %s keine Rekon', ...
              'struktion von q0 möglich für Parametersatz %d'], file2, err.message));
          end
        end
      end
    else % Datei existiert nicht. Es werden NaN gelassen
      cds_log(-1, sprintf(['[gen_init_pop] IK-Konfiguration für Datei ', ...
        '%s nicht rekonstruierbar'], file2));
      q0_pareto_i = NaN(size(d.RobotOptRes.p_val_pareto, 1), size(Structure.qlim, 1));
    end
    if ~isempty(d.RobotOptRes.p_val_pareto)
      d.RobotOptRes.q0_pareto = q0_pareto_i;
    else
      d.RobotOptRes.q0 = q0_pareto_i(:);
    end
  end
  counter_optresults = counter_optresults + 1;
  
  % Strukturinformationen laden
  Structure_i = d.RobotOptRes.Structure;
  if ~isfield(Structure_i, 'angles_values'), continue; end % altes Format
  if ~(isempty(Structure.angles_values) && isempty(Structure_i.angles_values)) && ...
      ~strcmp(Structure.angles_values, Structure_i.angles_values)
    % Freie Parameter haben anderen festen Wert (z.B. Struktursynthese).
    continue % Ergebnis nicht verwertbar.
  end
  % Einstellungen laden
  try
    settings_i = load(setfile_i);
  catch err
    cds_log(-1, sprintf(['[gen_init_pop] Datei %s konnte nicht geladen ', ...
      'werden. Fehler: %s'], setfile_i, err.message));
    continue
  end
  Set_i = settings_i.Set;
  % Aktualisiere mittlerweile geänderte Einstellungen.
  Structure_i.varnames(strcmp(Structure_i.varnames,'platform_morph')) = ...
    {'platform_morph_pairdist'}; % Optimierungsvariable wurde umbenannt.
  if ~isfield(Structure_i, 'xref_W') % Kompatibilität für altes Format
    Structure_i.xref_W = settings_i.Traj.X(1,:)'; % siehe cds_dimsynth_robot.m
  end
  % Prüfe, ob es sich um den identischen Roboter handelt (PKM-Koppelgelenkanord-
  % nungen können anders sein)
  score_i = score_i - 20*double(~strcmp(Structure_i.Name, Structure.Name));
  % Prüfe, ob die Zielfunktion die gleiche ist
  score_i = score_i + length(intersect(Set_i.optimization.objective,...
    Set.optimization.objective));
  % Prüfe sonstige Felder der Optimierungseinstellungen. Übereinstimmung
  % erhöht die Punktzahl zur Bewertung der Nutzbarkeit
  for f = fields(Set.optimization)'
    if length(Set.optimization.(f{1})) ~= 1, continue; end
    if isa(Set.optimization.(f{1}), 'cell'), continue; end
    if isa(Set.optimization.(f{1}), 'struct'), continue; end
    if ~isfield(Set_i.optimization, f{1})
      % Gespeicherte Ergebnisse haben andere Einstellungen. Vermutlich
      % alte Repo-Version. Daher Punktabzug.
      score_i = score_i - 2;
      continue
    end
    if Set.optimization.(f{1}) == Set_i.optimization.(f{1})
      score_i = score_i + 1;
    end
  end
  % Prüfe Montagerichtung des Roboters. Falls unterschiedlich, sind die
  % Parameter voraussichtlich kaum nutzbar.
  if Structure.Type ~= 2 && (~isfield(Set_i.structures, 'mounting_serial') || ...
      ~strcmp(Set.structures.mounting_serial, Set_i.structures.mounting_serial)) || ...
     Structure.Type == 2 && (~isfield(Set_i.structures, 'mounting_parallel') || ...
      ~strcmp(Set.structures.mounting_parallel, Set_i.structures.mounting_parallel))
    score_i = score_i - 10;
  end
  % Bei Portalsystemen sollten die Typen der Schubgelenke gleich sein
  if ~isempty(Structure.prismatic_types) && ...
      (~isfield(Structure_i, 'prismatic_types') || ... % Altes Dateiformat
       ~strcmp(Structure_i.prismatic_types, Structure.prismatic_types))
    % Ergebnisse sind nicht direkt vergleichbar, da aufgrund der Schubgelenk- 
    % Offsets bei anderen Gelenktypen eventuell die Kollisionen nicht passen
    score_i = score_i - 2;
  end
  
  % Auslesen der Parameter (bezogen auf die Datei)
  if ~isempty(d.RobotOptRes.p_val_pareto)
    pval_i_file = d.RobotOptRes.p_val_pareto;
    fval_i = d.RobotOptRes.fval_pareto;
    qval_i = d.RobotOptRes.q0_pareto;
  else
    pval_i_file = d.RobotOptRes.p_val(:)';
    fval_i = d.RobotOptRes.fval(:)';
    qval_i = d.RobotOptRes.q0(:)';
  end
  % Laden zusätzlicher Detail-Ergebnisse um die Daten anzureichern
  if Set.optimization.InitPopFromDetailResults && exist(file2, 'file')
    try
      if isempty(d2) % Kann oben schon gelade worden sein
        d2 = load(file2);
      end
    catch err
      cds_log(-1, sprintf(['[gen_init_pop] Datei %s konnte nicht geladen ', ...
        'werden. Fehler: %s. Dann keine Detail-Ergebnisse importierbar'], file2, err.message));
    end
    if ~isempty(d2)
      RobotOptRes2_i = cds_convert_pareto_details2front(d2.PSO_Detail_Data);
      pval_i_file = [pval_i_file; RobotOptRes2_i.pval_all]; %#ok<AGROW> 
      fval_i = [fval_i; RobotOptRes2_i.fval_all]; %#ok<AGROW> 
      qval_i = [qval_i; RobotOptRes2_i.q0_all]; %#ok<AGROW> 
    end
  end

  % Nachverarbeiten von Parametern in der Datei (für Abwärtskompatibilität bei Code-Aktualisierung)
  % Korrigiere den axoffset-Parameter für neue Implementierung; ab 22.04.22
  I_pmao = strcmp(d.RobotOptRes.Structure.varnames, 'platform_morph_axoffset');
  if d.RobotOptRes.timestamps_start_end(1) < datenum(2022, 4, 22) && any(I_pmao) % für ARK-Paper zu 3T2R-PKM
    I_pfr = d.RobotOptRes.Structure.vartypes == 7;
    pval_i_file(:,I_pmao) = pval_i_file(:,I_pmao) .* pval_i_file(:,I_pfr);
  end
  
  pval_i_const = NaN(nvars,1); % In Datei konstante Parameter, umgerechnet auf aktuelle Parameter
  % Index-Vektor zum Finden der aktuellen Optimierungsparameter pval in
  % den Optimierungsparametern pval_i_file. 0-Einträge deuten auf nicht
  % vorhandene Parameter, NaN-Einträge auf aus Einstellungen gelesene.
  I_p_file = zeros(nvars,1);
  for jjj = 1:length(I_p_file)
    Ii = find(strcmp(Structure.varnames{jjj},Structure_i.varnames));
    if ~isempty(Ii)
      I_p_file(jjj) = Ii;
    end
  end
  % TODO: Belege die Parameter aus den gespeicherten Eigenschaften des Roboters

  % Bestimme die Variablennamen ohne die laufende Nummber bei pkin
  for ll = 1:2
    if ll == 1, varnames_tmp = Structure_i.varnames;
    else,       varnames_tmp = Structure.varnames;
    end
    [t,~] = regexp(varnames_tmp, 'pkin [\d]+: (.*)', 'tokens', 'match');
    for ii = 1:length(varnames_tmp)
      if ~isempty(t{ii}), varnames_tmp{ii} = t{ii}{1}{1}; end
    end
    if ll == 1, varnames_i = varnames_tmp;
    else,       varnames_this = varnames_tmp;
    end
  end
  % Bestimme Indizes der in der Datei benutzten und aktuell nicht benutzten Parameter
  [~,missing_local_in_file, missing_file_in_local] = ...
    setxor(varnames_i, varnames_this);
  % Debug:
%   fprintf('Lokale Parameter, fehlend in Datei: {%s}\n', ...
%     disp_array(Structure_i.varnames(missing_local_in_file),'%s'));
%   fprintf('Parameter in Datei, lokal fehlend: {%s}\n', ...
%     disp_array(Structure.varnames(missing_file_in_local), '%s'));

  % Rechne Parameter aus Datei in aktuelle Parameter um.
  pval_i = NaN(size(pval_i_file,1), nvars);
  for jjj = 1:length(I_p_file)
    if I_p_file(jjj) ~= 0
      pval_i(:,jjj) = pval_i_file(:,I_p_file(jjj));
    end
  end
  % Trage zusätzliche Parameter ein, die in Datei konstant sind und jetzt
  % Variabel und damit direkt umgerechnet werden können.
  
  % Wenn in Datei Kinematik-Parameter nicht optimiert werden, nehme an,
  % dass diese dort Null sind. Jetzt werden sie optimiert. Es können
  % mehrere Kinematik-Parameter sein. Daher Binär-Index-Logik.
  I_pkin = contains(Structure.varnames, 'pkin'); % Format: "pkin 1: a1"
  I_missing_file_in_local = false(1,length(I_pkin));
  I_missing_file_in_local(missing_file_in_local) = true; % Binär-Indizes
  I_pkinzero = I_pkin & I_missing_file_in_local;
  if any(I_pkinzero)
    pval_i_const(I_pkinzero) = 0;
  end
  % Wenn in der Datei kein rotate_base benutzt wird, und hier schon,
  % ist es egal. Dann kann der Parameter direkt Null gesetzt werden.
  I_baserotz = find(strcmp(Structure.varnames, 'baserotation z'));
  if ~isempty(I_baserotz) && any(I_baserotz == missing_file_in_local)
    pval_i_const(I_baserotz) = 0;
  end
  % Das gleiche für Tilt Base
  I_baserotx = find(strcmp(Structure.varnames, 'baserotation x'));
  if ~isempty(I_baserotx) && any(I_baserotx == missing_file_in_local)
    pval_i_const(I_baserotx) = 0;
  end
  I_baseroty = find(strcmp(Structure.varnames, 'baserotation y'));
  if ~isempty(I_baseroty) && any(I_baseroty == missing_file_in_local)
    pval_i_const(I_baseroty) = 0;
  end
  % Gleiches für Plattform-Morphologie (G8-Methode). Standardmäßig Null.
  I_platform_morph_axoffset = find(strcmp(Structure.varnames, 'platform_morph_axoffset'));
  if ~isempty(I_platform_morph_axoffset) && any(I_platform_morph_axoffset == missing_file_in_local)
    pval_i_const(I_platform_morph_axoffset) = 0;
  end
  for i_xyz = 1:3
    % Index des Basis-Positions-Parameters in allen Parametern finden
    I_basexyz = find(strcmp(Structure.varnames, ['base ', char(119+i_xyz)]));
    % Prüfen, ob dieser Parameter hier optimiert wird und in geladenen
    % Werten fehlt
    if isempty(I_basexyz) || ~any(I_basexyz == missing_file_in_local)
      continue
    end
    % Falls die Grenzen auf NaN gesetzt sind, werden sie mit relativen
    % Werten skaliert und die Parameter sind nicht übertragbar
    if any(isnan(Set.optimization.basepos_limits(i_xyz,:)))
      continue
    end
    % x-y- oder z-Komponente der Basisposition wird optimiert und der
    % Parameter fehlt in den geladenen Parametern
    if all(~isnan(Set_i.optimization.basepos_limits(i_xyz,:))) && ...
      diff(Set_i.optimization.basepos_limits(i_xyz,:))==0
      % Der Parameter wird in den geladenen Werten auf einen konstanten
      % Wert gesetzt. Nehme diesen direkt
      pval_i_const(I_basexyz) = Set_i.optimization.basepos_limits(i_xyz,1);
    end
  end
  
  % TODO: Weitere Abfragen zu anderen Parametern
  % Aus konstanten Einstellungen gesetzte Parameter eintragen
  for jjj = 1:length(I_p_file)
    if any(isnan(pval_i(:,jjj))) && ~isnan(pval_i_const(jjj))
      pval_i(:,jjj) = pval_i_const(jjj);
      % Parameter ist nicht in Datei explizit gesetzt, aber kein
      % Null-Eintrag da aus Einstellungen ableitbar
      I_p_file(jjj) = NaN;
    end
  end
  % Parameter modifizieren, die gerundet wurden. Runde auch hier, damit die
  % geladenen Parameter den gerundeten entsprechen. Siehe cds_update_robot_parameters
  % Betrifft alpha/theta und Basis-Rotation
  if isfield(Set_i.structures, 'orthogonal_joints') && ... % Feld existiert eventuell nicht in alten Daten
      Set_i.structures.orthogonal_joints && ~Set.structures.orthogonal_joints
    I_alphatheta = contains(Structure_i.varnames, 'pkin') & ...
      (contains(Structure_i.varnames, 'alpha') | contains(Structure_i.varnames, 'theta'));
    pval_i(:,I_alphatheta) = round(pval_i(:,I_alphatheta)/(pi/2))*pi/2;
  end
  if isfield(Set_i.structures, 'tilt_base_only_orthogonal') && ...
      Set_i.optimization.tilt_base_only_orthogonal && ~Set.optimization.tilt_base_only_orthogonal
    I_baserotxy = contains(Structure_i.varnames, 'baserotation x') | ...
                  contains(Structure_i.varnames, 'baserotation y');
    pval_i(:,I_baserotxy) = round(pval_i(:,I_baserotxy)/(pi/2))*pi/2;
  end
  if isfield(Set_i.structures, 'rotate_base_only_orthogonal') && ...
      Set_i.optimization.rotate_base_only_orthogonal && ~Set.optimization.rotate_base_only_orthogonal
    I_baserotz = contains(Structure_i.varnames, 'baserotation z');
    pval_i(:,I_baserotz) = round(pval_i(:,I_baserotz)/(pi/2))*pi/2;
  end

  % Prüfe erlaubte Werte für Parameter.
  % Benutze dafür die Variable Structure_i statt Structure, da auch PKM mit anderen
  % Gestellanordnungen geladen werden und die Gültigkeit der geladenen
  % Daten geprüft wird.
  if Structure_i.Type == 2 && any(Structure_i.Coupling(1) == [4 8]) && ...% Konische Gestellgelenke
     Set.optimization.min_inclination_conic_base_joint > 0 && ...% Einstellung gesetzt
     (~isfield(Set_i.optimization, 'min_inclination_conic_base_joint')||...% geladene Einstellung nicht so streng
     Set_i.optimization.min_inclination_conic_base_joint<Set.optimization.min_inclination_conic_base_joint) ...
     || ...% Konische Plattformgelenke
     Structure_i.Type == 2 && any(Structure_i.Coupling(2) == 9) && ... % Passende Struktur
     Set.optimization.min_inclination_conic_platform_joint > 0 && ... % Einstellung gesetzt
     (~isfield(Set_i.optimization, 'min_inclination_conic_platform_joint') ||... % geladene Einstellung nicht so streng
     Set_i.optimization.min_inclination_conic_platform_joint<Set.optimization.min_inclination_conic_platform_joint) ...
     || ... % Mindestabstand der Gelenke ist gefordert und in geladener Einstellung nicht so streng
     Set.optimization.min_joint_distance > 0 && (~isfield(Set_i.optimization, 'min_joint_distance') ||...
     Set_i.optimization.min_joint_distance<Set.optimization.min_joint_distance) ...
     || ... % Gestell-Durchmesser ist gefordert (für Paarweise Anordnung anders)
     Structure_i.Type == 2 && any(Structure_i.Coupling(1) == [5 6 7 8]) && ...
     all(~isnan(Set.optimization.base_size_limits)) && ... % Gestell-Grenzen gegeben
     any(Structure_i.vartypes == 8) && ... % Morphologie wird optimiert
     Set.optimization.base_size_limits(1)~=Set.optimization.base_size_limits(2) ...% Grenzen nicht gleich
     || ... % Plattform-Durchmesser ist gefordert (für Paarweise Anordnung anders)
     Structure_i.Type == 2 && any(Structure_i.Coupling(2) == [4 5 6]) && ... % PKM, Plattform paarweise
     all(~isnan(Set.optimization.platform_size_limits)) && ... % Plattform-Grenzen gegeben
     any(Structure_i.vartypes == 9) && ... % Morphologie wird optimiert
     Set.optimization.platform_size_limits(1)~=Set.optimization.platform_size_limits(2) ... % Grenzen nicht gleich
     || Set.optimization.tilt_base  % Neigungswinkel für Gestell werden optimiert
    % Aktualisiere die geladenen Einstellungen, falls sie auf einer alten
    % Version basieren. Sonst Fehler in update_robot_parameters
    Set_i = cds_settings_update(Set_i); % Erst hier wegen Rechenzeit
    for jjj = 1:size(pval_i,1)
      p_phys_jjj=cds_update_robot_parameters([], Set_i, Structure_i, pval_i_file(jjj,:)');
      % Der Winkel wird direkt physikalisch eingesetzt. Alle anderen Parameter sind egal.
      fval_jjj = cds_constraints_parameters([], Set, Structure_i, pval_i_file(jjj,:)', p_phys_jjj);
      if fval_jjj > 0
        % Belege die Fitness-Werte dieses Partikels neu (hat dann sehr
        % schlechte Chancen, ist aber nicht komplett deaktiviert)
        fval_i(jjj,:) = fval_jjj;
      end
    end
  end

  % Falls Optimierungsparameter in der Datei nicht gesetzt sind, müssen
  % diese zufällig neu gewählt werden. Dafür gibt es Abzug in der
  % Bewertung.
  score_i = score_i - sum(I_p_file==0)*5;
  
  % Prüfe, ob die Optimierungsparameter gleich sind
  if length(Structure.vartypes) ~= length(Structure_i.vartypes) || ...
      any(Structure.vartypes ~= Structure_i.vartypes)
    % Die Optimierungsparameter sind unterschiedlich. 
    I_basez_i = strcmp(Structure_i.varnames, 'base z');
    for ll = find(I_basez_i)
      I_basez = (ll == missing_local_in_file);
      % PKM mit Schubantrieben die nach oben zeigen. Die Basis-Position ist
      % egal. Falls der Parameter nicht mehr optimiert wird, sind vorherige
      % Ergebnisse trotzdem verwendbar.
      if any(I_basez) && ...
          Structure.Type == 2 && Structure.Name(3) == 'P' && any(Structure.Coupling(1)==[1 4])
        missing_local_in_file(missing_local_in_file==ll) = 0;
      end
    end
    % Wenn in der Datei 3T3R benutzt wurde und jetzt 3T2R, ist die letzte
    % EE-Rotation egal und der Parameter wird ignoriert
    I_eerotz_i = strcmp(Structure_i.varnames, 'ee rot 3');
    for ll = find(I_eerotz_i)
      I_eerotz = (ll == missing_local_in_file);
      if any(I_eerotz) && Set.task.pointing_task
        missing_local_in_file(missing_local_in_file==ll) = 0;
      end
    end
    % Ignoriere die Neigung des Gestells. Annahme: Könnte geometrisch
    % trotzdem gut funktionieren und wird bestraft durch Abweichung der
    % Einstellungen.
    for ll = 1:length(Structure_i.varnames)
      if contains(Structure_i.varnames{ll}, 'baserotation')
        missing_local_in_file(missing_local_in_file==ll) = 0;
      end
    end
    % Ignoriere die Neigung des EE. Gleiche Annahme
    for ll = 1:length(Structure_i.varnames)
      if contains(Structure_i.varnames{ll}, 'ee rot')
        missing_local_in_file(missing_local_in_file==ll) = 0;
      end
    end
    % Falls unterschiedliche Koppelgelenkanordnungen geladen werden, setze
    % die zusätzlichen Parameter auf Null
    I_coupl_i = (Structure_i.vartypes == 8 | Structure_i.vartypes == 9)';
    for ll = find(I_coupl_i)
      I_coupl = (ll == missing_local_in_file);
      if any(I_coupl)
        missing_local_in_file(missing_local_in_file==ll) = 0;
      end
    end
    if all(missing_local_in_file==0)
      % Alle in Datei überflüssigen Parameter sind egal. Mache weiter.
    else
      % Noch nichts ableitbar. TODO: Logik weiter ausbauen, um die
      % Parameter aus unvollständigen Daten neu aufzubauen.
      cds_log(1, sprintf(['[gen_init_pop] Optimierungsparameter in Ergebnis-', ...
        'Ordner %s unterschiedlich (%d vs %d). Keine Anfangswerte ableitbar. ', ...
        'Unterschied: {%s}. Bestimmbar: {%s}'], fileparts(initpop_matlist{i}), ...
        length(Structure.vartypes), length(Structure_i.vartypes), ...
        disp_array(setxor(varnames_i, varnames_this)), ...
        disp_array(Structure.varnames(~isnan(pval_i_const)), '%s')));
      continue
    end
  end
  % Prüfe, ob die Trajektorie gleich ist
  trajectory_similarity = NaN; % Maß der Ähnlichkeit der Trajektorie
  if ~all(abs(Structure_i.xT_mean-Structure.xT_mean) < 1e-6)
    if size(Traj.X,1) == size(settings_i.Traj.X,1)
      % Bestimme Ähnlichkeit
      xc = diag(corr(Traj.X, settings_i.Traj.X));
      if all(xc > 0.98 | isnan(xc))
        % Trajektorie ist sehr ähnlich, aber nur verschoben. Nehme an, dass
        % die Ergebnisse nutzbar sind
        trajectory_similarity = 2;
      else
        % Gleich lang aber etwas anders. Strafterm.
        score_i = score_i - 3;
        trajectory_similarity = 1;
      end
    else
      % Trajektorienmittelpunkt und Länge anders. Vergleich der Strukturen
      % evtl nicht sinnvoll. Daher Straferm
      score_i = score_i - 5;
      trajectory_similarity = 0;
    end
  else
    trajectory_similarity = 3; % Annahme: Ist identisch (weitere Prüfungen ignorieren)
  end

  % Prüfe, ob die Parametergrenzen eingehalten werden
  % Grenzen als Matrix, damit Vergleich einfacher Implementierbar.
  ll_repmat = repmat(varlim(:,1)',size(pval_i,1),1);
  ul_repmat = repmat(varlim(:,2)',size(pval_i,1),1);
  % Manuelle Anpassung der Parametergrenzen
  % Setze bei Verletzung des Basis-Position-Z-Parameters diesen in manchen
  % Fällen auf die Grenze
  I_bpz = strcmp(Structure_i.varnames, 'base z'); % Index des Parameters
  % PKM mit Schubantrieben die nach oben zeigen. Die Basis-Position ist
  % egal. In der IK ergibt sich automatisch eine neue Lösung.  
  B_verticalprismatic = Structure.Type == 2 && Structure.Name(3) == 'P' && any(Structure.Coupling(1)==[1 4]);
  if any(I_bpz) && (B_verticalprismatic || trajectory_similarity>=2)
    I_bpz_llviol = pval_i(:,I_bpz)<ll_repmat(:,I_bpz);
    pval_i(I_bpz_llviol,I_bpz) = ll_repmat(I_bpz_llviol,I_bpz);
    I_bpz_ulviol = pval_i(:,I_bpz)>ul_repmat(:,I_bpz);
    pval_i(I_bpz_ulviol,I_bpz) = ul_repmat(I_bpz_ulviol,I_bpz);
  end
  % Eigentliche Prüfung der Parametergrenzen. 
  % Nehme Parameter von der Prüfung aus, die nicht direkt geprüft werden
  I_ignore = false(size(pval_i));
  I_pfsize = Structure.vartypes == 7; % Optimierung der Plattform-Größe
  if Structure.Type == 2 && any(I_pfsize) && ...
      ... % Geladene Daten von Paarweiser Anordnung, Aktuell nicht-paarweise
      ... % Wurde daher schon oben geprüft, ob Plattformgröße passt
      any(Structure_i.Coupling(2) == [4 5 6]) && ~any(Structure.Coupling(2) == [4 5 6])
    I_ignore(:,I_pfsize) = true;
  end
  % Nehme an, dass ein auf NaN gesetzter Parameter in Ordnung ist. 
  % Es werden dann unten statt gespeicherter Werte Zufallswerte eingesetzt.
  I_param_iO = all(ll_repmat <= pval_i & ul_repmat >= pval_i | isnan(pval_i) |I_ignore ,2);
  cds_log(2, sprintf(['[gen_init_pop] Auswertung %d (%s) geladen. ', ...
    'Bewertung: %d. Bei %d/%d Parametergrenzen passend.'], i, ...
    initpop_matlist{i}, score_i, sum(I_param_iO), size(pval_i,1)));
  if any(~I_param_iO)
    for jjj = find(~I_param_iO & ~any(isnan(pval_i),2))'
      I_pniO = varlim(:,1)'-1e-10 > pval_i(jjj,:) | ...  % Grenzen gegen numerische ... 
               varlim(:,2)'+1e-10 < pval_i(jjj,:); % ... Ungenauigkeit aufweiten
      for kkk = find(I_pniO & ~I_ignore(jjj,:))
        delta_ul = pval_i(jjj,kkk)-varlim(kkk,2)';
        delta_ll = pval_i(jjj,kkk)-varlim(kkk,1)';
        mindelta = min(abs([delta_ul;delta_ll]));
        cds_log(3, sprintf(['[gen_init_pop] Partikel %d/%d: Parameter ', ...
          '%d (%s) nicht passend. %1.3f < %1.3f < %1.3f. Delta: %1.3e'], ...
          jjj, size(pval_i,1), kkk, varnames{kkk}, ...
          varlim(kkk,1), pval_i(jjj,kkk), varlim(kkk,2), mindelta));
      end
    end
  end
  % Hinzufügen zu Liste von Parametern
  InitPopLoadTmp = [InitPopLoadTmp; pval_i(I_param_iO,:)]; %#ok<AGROW>
  Q_PopTmp = [Q_PopTmp; qval_i(I_param_iO,:)]; %#ok<AGROW>
  OptNamesTmp = [OptNamesTmp(:)', repmat({Set_i.optimization.optname}, 1, sum(I_param_iO))]; %#ok<AGROW> 
  RobNamesTmp = [RobNamesTmp(:)', repmat({Structure_i.Name}, 1, sum(I_param_iO))]; %#ok<AGROW> 
  fval_mean_all = mean(fval_i(I_param_iO,:),2);
  ScoreLoad = [ScoreLoad; [score_i-2*floor(log10(fval_mean_all)), ...
    repmat(score_i,size(fval_mean_all,1),1),fval_mean_all]]; %#ok<AGROW>
  continue
  % Lade Details aus den Iterationen (für weitere Vergleiche)
%   resfilename_II_details = strrep(resfiles(II).name, 'Endergebnis', 'Details');
%   if isfield(d, 'PSO_Detail_Data')
%     % Altes Dateiformat
%     PSO_Detail_Data = d.PSO_Detail_Data;
%   elseif exist(resfilename_II_details, 'file')
%     d2 = load(resfilename_II_details);
%     PSO_Detail_Data = d2.PSO_Detail_Data;
%   else
%     PSO_Detail_Data = [];
%   end
%   if ~isempty(PSO_Detail_Data) && isfield(PSO_Detail_Data, 'pval')
%     PSO_Detail_Data.pval
%   end
end
%% Entferne Duplikate. Sortiere dafür die Partikel anhand ihrer Bewertung
% Damit die beste Bewertung bei doppelten Partikeln genommen wird.
if ~isempty(InitPopLoadTmp)
  num1 = size(InitPopLoadTmp,1);
  [~,I] = sort(ScoreLoad(:,1), 'descend');
  InitPopLoadTmp = InitPopLoadTmp(I,:);
  Q_PopTmp = Q_PopTmp(I,:);
  OptNamesTmp = OptNamesTmp(I);
  RobNamesTmp = RobNamesTmp(I);
  ScoreLoad = ScoreLoad(I,:);
  % Lösche Duplikate. Behalte die mit den besten Bewertungen. Prüfe mit
  % Toleranz (geht nur ohne NaN-Werte in Eingabe für uniquetol)
  InitPopLoadTmp2 = InitPopLoadTmp;
  InitPopLoadTmp2(isnan(InitPopLoadTmp2)) = inf; % Dummy-Wert
  [~,I] = uniquetol(InitPopLoadTmp2, 'ByRows', true);
  I = sort(I); % für gleichbleibende Reihenfolge der Partikel (unklar ob notwendig)
  InitPopLoadTmp = InitPopLoadTmp(I,:);
  Q_PopTmp = Q_PopTmp(I,:);
  OptNamesTmp = OptNamesTmp(I);
  RobNamesTmp = RobNamesTmp(I);
  ScoreLoad = ScoreLoad(I,:);
  num2 = size(InitPopLoadTmp,1);
  cds_log(1, sprintf(['[gen_init_pop] Insgesamt %d Partikel geladen. ', ...
    'Davon %d unterschiedlich.'], num1, num2));
end
%% Wähle aus den geladenen Parametern eine Anfangspopulation mit hoher Diversität
% Anzahl der zu ladenden Parameter (begrenzt durch vorhandene)
nIndLoad = floor(Set.optimization.InitPopRatioOldResults*nIndTotal);
nIndLoad = min(nIndLoad, size(InitPopLoadTmp,1));
if size(InitPopLoadTmp,1) > 0
  % Normiere die geladenen Parameter auf die Parametergrenzen. Dadurch
  % Bestimmung der Diversität der Population besser möglich.
  InitPopLoadTmpNorm = (InitPopLoadTmp-repmat(varlim(:,1)',size(InitPopLoadTmp,1),1)) ./ ...
    repmat(varlim(:,2)'-varlim(:,1)',size(InitPopLoadTmp,1),1);
  % Erzwinge Begrenzung der Parameter auf vorher vorgegebene Grenzen. Falls
  % zu große/kleine Parameter geladen werden, werden sie begrenzt.
  InitPopLoadTmpNorm(InitPopLoadTmpNorm>1) = 1;
  InitPopLoadTmpNorm(InitPopLoadTmpNorm<0) = 0;
  % Entferne Duplikate erneut (wegen Rundungsfehler bei Rechenungenauigkeit
  % möglich). Reihenfolge soll beibehalten werden (-> stable)
  [~,I] = unique(InitPopLoadTmpNorm, 'rows', 'stable');
  InitPopLoadTmpNorm = InitPopLoadTmpNorm(I,:);
  ScoreLoad = ScoreLoad(I,:);
  Q_PopLoadTmp = Q_PopTmp(I,:);
  OptNamesLoadTmp = OptNamesTmp(I);
  RobNamesLoadTmp = RobNamesTmp(I);
  nIndLoad = min(nIndLoad, size(InitPopLoadTmpNorm,1));
  % Indizies der bereits ausgewählten Partikel (in InitPopLoadTmpNorm)
  I_selected = false(size(ScoreLoad,1),1);
  
  % Beste und schlechteste Bewertung zur Einordnung der Ergebnisse
  bestscore = max(ScoreLoad(:,1));
  worstscore = min(ScoreLoad(:,1));
  % Population der gewählten geladenen Partikel. Baue eins nach dem anderen
  % auf.
  InitPopLoadNorm = NaN(nIndLoad, nvars);
  Q_PopLoad = NaN(nIndLoad, size(Q_PopTmp,2));
  for i = 1:nIndLoad
    % Erlaube insgesamt nur die besten 30% der Bewertungen. Nehme am Anfang
    % nur die besten, am Ende dann bis zu 30%
    if mod(i,3) ~= 0 % Bezug auf die Bewertung der Partikel
      bestscoreratio = 1-0.3*i/nIndLoad;
      I_score_allowed = ScoreLoad(:,1) > worstscore + bestscoreratio*(bestscore-worstscore);
    else % Bezug auf den Fitness-Wert (bei jedem dritten)
      I_score_allowed = ScoreLoad(:,3) < 1e3 & ScoreLoad(:,1)~=-inf;
    end
    % Bestimme die Indizes der Partikel, die durchsucht werden. Schließe
    % bereits gewählte aus.
    I_search = I_score_allowed & ~I_selected;
    if ~any(I_search) % falls keiner gefunden wurde: Prüfe alle
      [~, Isort] = sort(ScoreLoad(:,1), 'descend');
      % Entferne die bereits vorhandenen Partikel. Ansonsten doppelte.
      Isort = Isort(ScoreLoad(Isort,1)~=-inf);
      if isempty(Isort)
        cds_log(4, sprintf('[gen_init_pop] Keine weiteren Partikel mehr verfügbar'));
        break;
      end
      I_search(Isort(1:min(10,length(Isort)))) = true; % Wähle die 10 besten aus
    end
    II_search = find(I_search); % Zähl-Indizes zusätzlich zu Binär-Indizes
    % Bilde in jeder Iteration den Mittelwert der Parameter neu
    pnorm_mean_i = mean(InitPopLoadNorm, 1, 'omitnan');
    % Bestimme den quadratischen Abstand aller durchsuchter Partikel gegen
    % den aktuellen Mittelwert. Das ist ein vereinfachtes Diversitätsmaß.
    % Ignoriere freie Parameter (NaN) und normiere daher auf Parameterzahl
    score_div = sum((InitPopLoadTmpNorm(I_search,:) - repmat(pnorm_mean_i, ...
      sum(I_search), 1)).^2,2, 'omitnan')/size(InitPopLoadTmpNorm,2);
    if i == 1 % bei erstem Wert
      % nehme zufällig einen der erlaubten
      I_best = randi(length(score_div));
      score_div_best = inf; % Bedeutungsloser Wert, da ohne Bezug
    elseif all(isnan(score_div))
      warning('NaN in normierten Parametern. Darf hier nicht auftreten');
      [score_div_best,I_best] = max(score_div); % größte Diversität (vorherige Standard-Einstellung)
    else
      % Wähle zufällig einen derjenigen unter den 10 mit der höchsten
      % Diversität. So gibt es noch eine weitere Zufallskomponenten. Sonst
      % wären bei mehrfacher Durchführung alle Startgenerationen gleich.
      [~,I_sortdiv] = sort(score_div, 'descend');
      I_best = I_sortdiv( randi(min(length(I_sortdiv), 10)) );
      score_div_best = score_div(I_best);
    end
    % Wähle das beste Partikel aus und füge es zur Initialpopulation hinzu.
    % Vereinfachte Annahme: Dadurch wird die Diversität maximal vergrößert.
    InitPopLoadNorm(i,:) = InitPopLoadTmpNorm(II_search(I_best),:);
    Q_PopLoad(i,:) = Q_PopLoadTmp(II_search(I_best),:);
    % Markiere als bereits gewählt, damit es nicht erneut gewählt wird.
    I_selected(II_search(I_best)) = true;
    cds_log(4, sprintf(['[gen_init_pop] Partikel %d hinzugefügt ', ...
      '(Bewertung %d, fval %1.1e, gew. Bew. %d, divers=%1.2f). p_norm=[%s]; aus %s (%s)'], ...
      II_search(I_best), ScoreLoad(II_search(I_best),2), ScoreLoad(II_search(I_best),3), ...
      ScoreLoad(II_search(I_best),1), score_div_best, disp_array(InitPopLoadNorm(i,:), '%1.3f'), ...
      OptNamesLoadTmp{II_search(I_best)}, RobNamesLoadTmp{II_search(I_best)}));
    % Entferne den Wert aus ScoreLoad, damit diese Werte nicht ein weiteres
    % Mal genutzt werden und keine Filterung gemacht werden muss
    ScoreLoad(II_search(I_best),1) = -inf;
  end
  % Entferne die Normierung.
  InitPopLoad = repmat(varlim(:,1)',size(InitPopLoadNorm,1),1) + InitPopLoadNorm .* ...
    (repmat(varlim(:,2)'-varlim(:,1)',size(InitPopLoadNorm,1),1));
else
  InitPopLoad = [];
  Q_PopLoad = [];
end
InitPopLoad_unique = unique(InitPopLoad, 'rows');
if size(InitPopLoad_unique,1) ~= size(InitPopLoad,1)
  cds_log(-1, sprintf(['[gen_init_pop] Fehler beim Laden der Initialpopulation ', ...
    'aus bestehenden Dateien. Es sind %d/%d doppelte Einträge entstanden'], ...
    size(InitPopLoad,1)-size(InitPopLoad_unique,1), size(InitPopLoad,1)));
  nIndLoad = size(InitPopLoad_unique,1);
  InitPopLoad = InitPopLoad_unique;
end

%% Auffüllen mit zufälligen Werten für alle Optimierungsparameter
% Fülle die restlichen Individuen mit NaN auf
nIndRand = nIndTotal - nIndLoad;
InitPopRand = NaN(nIndRand,nvars);
InitPop = [InitPopLoad(1:nIndLoad,:); InitPopRand(1:nIndRand,:)];
Q_Pop = [Q_PopLoad(1:nIndLoad,:); NaN(nIndRand,length(Structure.q0_traj))];
% Belege alle mit NaN gesetzten Parameter mit Zufallswerten. Dadurch können
% auch unvollständige Parameter aus vorherigen Optimierungen nachträglich
% mit Zufallswerten erweitert und genutzt werden.
for i = 1:nvars
  I = isnan(InitPop(:,i));
  InitPop(I,i) = varlim(i,1) + rand(sum(I), 1) .* ...
                          repmat(varlim(i,2)'-varlim(i,1)',sum(I),1);
end

cds_log(1, sprintf(['[gen_init_pop] %d Partikel für Initialpopulation ', ...
  'aus %d vorherigen Optimierungen (mit %d unterschiedlichen Partikeln) ', ...
  'geladen. Dauer: %1.1fs. Davon %d genommen. Die restlichen %d ', ...
  'zufällig.'], size(InitPopLoad,1), counter_optresults, size(InitPopLoadTmp,1), ...
  toc(t1), nIndLoad, nIndRand));
