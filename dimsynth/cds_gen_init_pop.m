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

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [InitPop, Q_Pop] = cds_gen_init_pop(Set, Structure)
nIndTotal = Set.optimization.NumIndividuals;
varlim = Structure.varlim;
varnames = Structure.varnames;
vartypes = Structure.vartypes;
nvars = length(varnames);
%% Lade Ergebnisse bisheriger Optimierungen aus dem Ergebnis-Ordner
t1 = tic();
counter_optresults = 0;
counter_filesize = 0;
InitPopLoadTmp = [];
Q_PopTmp = [];
ScoreLoad = [];

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
end

% Alle möglichen Ergebnis-Ordner durchgehen
for kk = 1:length(Set.optimization.result_dirs_for_init_pop)
  resdir = Set.optimization.result_dirs_for_init_pop{kk};
  % Unterordner sind die Ergebnis-Ordner einzelner Optimierungen
  % Lade die Liste der Verzeichnisse mit Linux-Find-Befehl. Scheinbar ist
  % der Dateisystemzugriff auf dem Cluster über Matlab sehr langsam
  status = 1;
  if isunix()
    [status,dirlist] = system(sprintf(['find -L "%s" -maxdepth 2 ', ...
      '-name "Rob*_%s*_Endergebnis.mat"'], resdir, RobName));
    % Erzeuge Liste der Verzeichnisse aus der Vorauswahl
    if status == 0
      dirlist_cell = splitlines(dirlist);
      for i = 1:length(dirlist_cell) % Entferne den Dateinamen
        dirlist_cell{i} = fileparts(dirlist_cell{i});
      end
      dirlist_cell = unique(dirlist_cell);
    else
      cds_log(-1, sprintf('Find-Befehl funktionierte nicht in %s. Ausgabe:\n%s', resdir, dirlist));
    end
  end
  if status ~= 0 % Entweder Windows oder Find-Befehl erfolglos
    optdirs = dir(fullfile(resdir, '*'));
    dirlist_cell = cell(length(optdirs), 1);
    for i = 1:length(optdirs) % Unterordner durchgehen.
      if ~optdirs(i).isdir || optdirs(i).name(1) == '.'
        continue % Kein passendes Verzeichnis
      end
      dirlist_cell{i} = fullfile(resdir, optdirs(i).name);
    end
  end
  % Entferne leere Einträge
  dirlist_cell = dirlist_cell(~cellfun(@isempty,dirlist_cell));
  cds_log(2, sprintf(['[gen_init_pop] Lade Ergebnisse aus %d Unter', ...
    'verzeichnissen von %s'], length(dirlist_cell), resdir));
  for i = 1:length(dirlist_cell) % Unterordner durchgehen.
    if isempty(dirlist_cell{i})
      continue % Kein passendes Verzeichnis
    end
    dirname_i = dirlist_cell{i};
    % Aktuellen Roboter suchen
    resfiles = dir(fullfile(dirname_i, sprintf('Rob*%s*_Endergebnis.mat',RobName)));
    III = find(contains({resfiles(:).name}, RobName));
    if isempty(III)
      continue % Roboter nicht enthalten
    end
    sflist = dir(fullfile(dirname_i, '*_settings.mat'));
    if length(sflist) > 1
      continue % Mehr als eine Einstellungsdatei. Ungültig.
    end
    % fprintf('Daten für Roboter %s gefunden (%s)\n', RobName, dirname_i);
    % Gehe alle Ergebnisdateien zu dem Roboternamen durch. Es kann mehrere
    % geben (bei Struktursynthese oder bei paralleler Optimierung eines
    % einzigen Roboters
    for II = III(:)'
    score_i = 0; % Bewertung der Vergleichbarkeit dieser Optimierung mit der aktuellen Optimierung
    % Daten laden (Keine Abbruchbedingung)
    try % Auf Cluster teilweise Probleme mit Dateizugriff.
      d = load(fullfile(dirname_i, resfiles(II).name));
    catch err %  Ist hier nicht so schlimm, falls übersprungen wird.
      cds_log(-1, sprintf(['[gen_init_pop] Datei %s konnte nicht geladen ', ...
        'werden. Fehler: %s'], resfiles(II).name, err.message));
      continue
    end
    counter_filesize = counter_filesize + resfiles(II).bytes;
    if ~isfield(d.RobotOptRes, 'p_val_pareto') || ~isfield(d.RobotOptRes, 'q0')
      continue % Altes Dateiformat
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
    if ~isempty(sflist)
      try
        settings_i = load(fullfile(dirname_i, sflist(1).name));
      catch err
        cds_log(-1, sprintf(['[gen_init_pop] Datei %s konnte nicht geladen ', ...
          'werden. Fehler: %s'], sflist(1).name, err.message));
        continue
      end
      Set_i = settings_i.Set;
    else
      if ~isfield(d, 'Set')
        continue % Altes Dateiformat
      end
      Set_i = d.Set; % Altes Format
    end
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
    
    % Bestimme Indizes der in der Datei benutzten und aktuell nicht benutzten Parameter
    [~,missing_local_in_file, missing_file_in_local] = ...
      setxor(Structure_i.varnames,Structure.varnames);
    % Debug:
%     fprintf('Lokale Parameter, fehlend in Datei: {%s}\n', ...
%       disp_array(Structure_i.varnames(missing_local_in_file),'%s'));
%     fprintf('Parameter in Datei, lokal fehlend: {%s}\n', ...
%       disp_array(Structure.varnames(missing_file_in_local), '%s'));

    % Rechne Parameter aus Datei in aktuelle Parameter um.
    pval_i = NaN(size(pval_i_file,1), nvars);
    for jjj = 1:length(I_p_file)
      if I_p_file(jjj) ~= 0
        pval_i(:,jjj) = pval_i_file(:,I_p_file(jjj));
      end
    end
    % Trage zusätzliche Parameter ein, die in Datei konstant sind und jetzt
    % Variabel und damit direkt umgerechnet werden können.
    
    % Wenn in der Datei kein rotate_base benutzt wird, und hier schon,
    % ist es egal. Dann kann der Parameter direkt Null gesetzt werden.
    I_baserotz = find(strcmp(Structure.varnames, 'baserotation z'));
    if ~isempty(I_baserotz) && any(I_baserotz == missing_file_in_local)
      pval_i_const(I_baserotz) = 0;
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
    
    % Falls Optimierungsparameter in der Datei nicht gesetzt sind, müssen
    % diese zufällig neu gewählt werden. Dafür gibt es Abzug in der
    % Bewertung.
    score_i = score_i - sum(I_p_file==0)*5;
    
    % Prüfe, ob die Optimierungsparameter gleich sind
    if length(Structure.vartypes) ~= length(Structure_i.vartypes) || ...
        any(Structure.vartypes ~= Structure_i.vartypes)
      % Die Optimierungsparameter sind unterschiedlich. 
      I_basez = find(strcmp(Structure_i.varnames, 'base z')) == missing_local_in_file;
      % PKM mit Schubantrieben die nach oben zeigen. Die Basis-Position ist
      % egal. Falls der Parameter nicht mehr optimiert wird, sind vorherige
      % Ergebnisse trotzdem verwendbar.
      if ~isempty(I_basez) && ...
          Structure.Type == 2 && Structure.Name(3) == 'P' && any(Structure.Coupling(1)==[1 4])
        missing_local_in_file(missing_local_in_file==find(strcmp(Structure_i.varnames, 'base z'))) = 0;
      end
      % Wenn in der Datei 3T3R benutzt wurde und jetzt 3T2R, ist die letzte
      % EE-Rotation egal und der Parameter wird ignoriert
      I_eerotz = find(strcmp(Structure_i.varnames, 'ee rot 3')) == missing_local_in_file;
      if ~isempty(I_eerotz) && Set.task.pointing_task
        missing_local_in_file(missing_local_in_file==find(strcmp(Structure_i.varnames, 'ee rot 3'))) = 0;
      end
      if all(missing_local_in_file==0)
        % Alle in Datei überflüssigen Parameter sind egal. Mache weiter.
      else
        % Noch nichts ableitbar. TODO: Logik weiter ausbauen, um die
        % Parameter aus unvollständigen Daten neu aufzubauen.
        cds_log(1, sprintf(['[gen_init_pop] Optimierungsparameter in Ergebnis-', ...
          'Ordner %s unterschiedlich (%d vs %d). Keine Anfangswerte ableitbar. Unterschied: {%s}'], ...
          dirlist_cell{i}, length(Structure.vartypes), length(Structure_i.vartypes), ...
          disp_array(setxor(Structure_i.varnames, Structure.varnames))));
        continue
      end
    end
    % Prüfe, ob die Trajektorie gleich ist
    if ~all(abs(Structure_i.xT_mean-Structure.xT_mean) < 1e-6)
      % Trajektorienmittelpunkt anders. Vergleich der Strukturen evtl nicht
      % sinnvoll. Besserer Vergleich wäre sinnvoll.
      score_i = score_i - 5;
    end

    % Prüfe, ob die Parametergrenzen eingehalten werden
    % Grenzen als Matrix, damit Vergleich einfacher Implementierbar.
    ll_repmat = repmat(varlim(:,1)',size(pval_i,1),1);
    ul_repmat = repmat(varlim(:,2)',size(pval_i,1),1);
    % Manuelle Anpassung der Parametergrenzen
    if Structure.Type == 2 && Structure.Name(3) == 'P' && any(Structure.Coupling(1)==[1 4])
      % PKM mit Schubantrieben die nach oben zeigen. Die Basis-Position ist
      % egal. Verletzung der Grenzen wird auf Grenze gesetzt. In der IK
      % ergibt sich eine neue Lösung.
      I_bpz = strcmp(Structure_i.varnames, 'base z');
      I_bpz_llviol = pval_i(:,I_bpz)<ll_repmat(:,I_bpz);
      pval_i(I_bpz_llviol,I_bpz) = ll_repmat(I_bpz_llviol,I_bpz);
      I_bpz_ulviol = pval_i(:,I_bpz)>ul_repmat(:,I_bpz);
      pval_i(I_bpz_ulviol,I_bpz) = ul_repmat(I_bpz_ulviol,I_bpz);
    end
    % Eigentliche Prüfung der Parametergrenzen. Nehme an, dass ein auf NaN
    % gesetzter Parameter in Ordnung ist. Es werden dann unten statt
    % gespeicherter Werte Zufallswerte eingesetzt.
    I_param_iO = all(ll_repmat <= pval_i & ul_repmat >= pval_i | isnan(pval_i) ,2);
    cds_log(1, sprintf(['[gen_init_pop] Auswertung %d/%d Nr. %d (%s) geladen. ', ...
      'Bewertung: %d. Bei %d/%d Parametergrenzen passend.'], i, length(dirlist_cell), ...
      II, dirlist_cell{i}, score_i, sum(I_param_iO), size(pval_i,1)));
    if any(~I_param_iO)
      for jjj = find(~I_param_iO & ~any(isnan(pval_i),2))'
        I_pniO = varlim(:,1)'-1e-10 > pval_i(jjj,:) | ...  % Grenzen gegen numerische ... 
                 varlim(:,2)'+1e-10 < pval_i(jjj,:); % ... Ungenauigkeit aufweiten
        for kkk = find(I_pniO)
          cds_log(3, sprintf(['[gen_init_pop] Partikel %d/%d: Parameter ', ...
            '%d (%s) nicht passend. %1.2f < %1.2f < %1.2f'], ...
            jjj, size(pval_i,1), kkk, varnames{kkk}, ...
            varlim(kkk,1), pval_i(jjj,kkk), varlim(kkk,2)));
        end
      end
    end
    InitPopLoadTmp = [InitPopLoadTmp; pval_i(I_param_iO,:)]; %#ok<AGROW>
    Q_PopTmp = [Q_PopTmp; qval_i(I_param_iO,:)]; %#ok<AGROW>
    fval_mean_all = mean(fval_i(I_param_iO,:),2);
    ScoreLoad = [ScoreLoad; [score_i-2*floor(log10(fval_mean_all)), ...
      repmat(score_i,size(fval_mean_all,1),1),fval_mean_all]]; %#ok<AGROW>
    continue
    % Lade Details aus den Iterationen (für weitere Vergleiche)
%     resfilename_II_details = strrep(resfiles(II).name, 'Endergebnis', 'Details');
%     if isfield(d, 'PSO_Detail_Data')
%       % Altes Dateiformat
%       PSO_Detail_Data = d.PSO_Detail_Data;
%     elseif exist(resfilename_II_details, 'file')
%       d2 = load(resfilename_II_details);
%       PSO_Detail_Data = d2.PSO_Detail_Data;
%     else
%       PSO_Detail_Data = [];
%     end
%     if ~isempty(PSO_Detail_Data) && isfield(PSO_Detail_Data, 'pval')
%       PSO_Detail_Data.pval
%     end
    end
  end
end
%% Entferne Duplikate. Sortiere dafür die Partikel anhand ihrer Bewertung
% Damit die beste Bewertung bei doppelten Partikeln genommen wird.
if ~isempty(InitPopLoadTmp)
  num1 = size(InitPopLoadTmp,1);
  [~,I] = sort(ScoreLoad(:,1), 'descend');
  InitPopLoadTmp = InitPopLoadTmp(I,:);
  Q_PopTmp = Q_PopTmp(I,:);
  ScoreLoad = ScoreLoad(I,:);
  % Lösche Duplikate. Behalte die mit den besten Bewertungen.
  [~,I] = unique(InitPopLoadTmp, 'rows', 'stable');
  InitPopLoadTmp = InitPopLoadTmp(I,:);
  Q_PopTmp = Q_PopTmp(I,:);
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
  % möglich)
  [~,I] = unique(InitPopLoadTmpNorm, 'rows');
  InitPopLoadTmpNorm = InitPopLoadTmpNorm(I,:);
  ScoreLoad = ScoreLoad(I,:);
  Q_PopLoadTmp = Q_PopTmp(I,:);
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
    bestscoreratio = 1-0.3*i/nIndLoad;
    I_score_allowed = ScoreLoad(:,1) > worstscore + bestscoreratio*(bestscore-worstscore);
    % Bestimme die Indizes der Partikel, die durchsucht werden. Schließe
    % bereits gewählte aus.
    I_search = I_score_allowed & ~I_selected;
    if ~any(I_search)
      [~, Isort] = sort(ScoreLoad(:,1), 'descend');
      % Entferne die bereits vorhandenen Partikel. Ansonsten doppelte.
      for k = find(I_selected)'
        Isort = Isort(Isort~=k);
      end
      I_search(Isort(1:min(10,length(Isort)))) = true; % Wähle die 10 besten aus
    end
    II_search = find(I_search); % Zähl-Indizes zusätzlich zu Binär-Indizes
    % Bilde in jeder Iteration den Mittelwert der Parameter neu
    pnorm_mean_i = mean(InitPopLoadNorm, 1, 'omitnan');
    % Bestimme den quadratischen Abstand aller durchsuchter Partikel gegen
    % den aktuellen Mittelwert. Das ist ein vereinfachtes Diversitätsmaß.
    score_div = sum((InitPopLoadTmpNorm(I_search,:) - ...
      repmat(pnorm_mean_i, sum(I_search), 1)).^2,2);
    % Wähle das beste Partikel aus und füge es zur Initialpopulation hinzu.
    % Vereinfachte Annahme: Dadurch wird die Diversität maximal vergrößert.
    [~,I_best] = min(score_div);
    InitPopLoadNorm(i,:) = InitPopLoadTmpNorm(II_search(I_best),:);
    Q_PopLoad(i,:) = Q_PopLoadTmp(II_search(I_best),:);
    % Markiere als bereits gewählt, damit es nicht erneut gewählt wird.
    I_selected(II_search(I_best)) = true;
    cds_log(4, sprintf(['[gen_init_pop] Partikel %d hinzugefügt ', ...
      '(Bewertung %d, fval %1.1e, gew. Bew. %d). p_norm=[%s]'], II_search(I_best), ...
      ScoreLoad(II_search(I_best),2), ScoreLoad(II_search(I_best),3), ...
      ScoreLoad(II_search(I_best),1), disp_array(InitPopLoadNorm(i,:), '%1.3f')));
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
  'geladen (%1.1fMB). Dauer: %1.1fs. Davon %d genommen. Die restlichen %d ', ...
  'zufällig.'], size(InitPopLoad,1), counter_optresults, size(InitPopLoadTmp,1), ...
  counter_filesize/1e6, toc(t1), nIndLoad, nIndRand));
