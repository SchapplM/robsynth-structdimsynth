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

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function InitPop = cds_gen_init_pop(Set, Structure)
nIndTotal = Set.optimization.NumIndividuals;
varlim = Structure.varlim;
varnames = Structure.varnames;
vartypes = Structure.vartypes;
nvars = length(varnames);
%% Lade Ergebnisse bisheriger Optimierungen aus dem Ergebnis-Ordner
InitPopLoadTmp = [];
ScoreLoad = [];
RobName = Structure.Name;
% Alle möglichen Ergebnis-Ordner durchgehen
for kk = 1:length(Set.optimization.result_dirs_for_init_pop)
  resdir = Set.optimization.result_dirs_for_init_pop{kk};
  % Unterordner sind die Ergebnis-Ordner einzelner Optimierungen
  optdirs = dir(fullfile(resdir, '*'));
  for i = 1:length(optdirs) % Unterordner durchgehen.
    score_i = 0; % Bewertung der Vergleichbarkeit dieser Optimierung mit der aktuellen Optimierung
    if ~optdirs(i).isdir || optdirs(i).name(1) == '.'
      continue % Kein passendes Verzeichnis
    end
    dirname_i = fullfile(resdir, optdirs(i).name);
    % Aktuellen Roboter suchen
    resfiles = dir(fullfile(dirname_i, 'Rob*_Endergebnis.mat'));
    II = find(contains({resfiles(:).name}, RobName),1);
    if isempty(II)
      continue % Roboter nicht enthalten
    end
    if length(II) > 1
      cds_log(-1, sprintf(['[cds_gen_init_pop] Roboter %s scheint mehrfach ', ...
        'vorzukommen: %s.'], RobName, dirname_i));
      continue
    end
    sflist = dir(fullfile(dirname_i, '*_settings.mat'));
    if length(sflist) > 1
      continue % Mehr als eine Einstellungsdatei. Ungültig.
    end
    % fprintf('Daten für Roboter %s gefunden (%s)\n', RobName, dirname_i);
    
    % Daten laden (Keine Abbruchbedingung)
    d = load(fullfile(dirname_i, resfiles(II).name));
    if ~isfield(d.RobotOptRes, 'p_val_pareto')
      continue % Altes Dateiformat
    end
    % Einstellungen laden
    if ~isempty(sflist)
      settings_i = load(fullfile(dirname_i, sflist(1).name));
      Set_i = settings_i.Set;
    else
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

    Structure_i = d.RobotOptRes.Structure;
    % Prüfe, ob die Optimierungsparameter gleich sind
    if length(Structure.vartypes) ~= length(Structure_i.vartypes) || ...
        any(Structure.vartypes ~= Structure_i.vartypes)
      % Die Optimierungsparameter sind unterschiedlich. Noch nichts
      % ableitbar. TODO: Logik, um die Parameter aus unvollständigen Daten
      % neu aufzubauen.
      cds_log(1, sprintf(['[cds_gen_init_pop] Optimierungsparameter in Ergebnis-', ...
        'Ordner %s unterschiedlich (%d vs %d). Keine Anfangswerte ableitbar. Unterschied: {%s}'], ...
        optdirs(i).name, length(Structure.vartypes), length(Structure_i.vartypes), ...
        disp_array(setxor(Structure_i.varnames, Structure.varnames))));
      continue
    end
    % Prüfe, ob die Trajektorie gleich ist
    if ~all(abs(Structure_i.xT_mean-Structure.xT_mean) < 1e-6)
      % Trajektorienmittelpunkt anders. Vergleich der Strukturen evtl nicht
      % sinnvoll. Besserer Vergleich wäre sinnvoll.
      score_i = score_i - 5;
    end

    % Prüfe, ob die Parametergrenzen eingehalten werden
    if ~isempty(d.RobotOptRes.p_val_pareto)
      pval_i = d.RobotOptRes.p_val_pareto;
      fval_i = d.RobotOptRes.fval_pareto;
    else
      pval_i = d.RobotOptRes.p_val';
      fval_i = d.RobotOptRes.fval;
    end
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
    % Eigentliche Prüfung der Parametergrenzen
    I_param_iO = all(ll_repmat <= pval_i & ul_repmat >= pval_i,2);
    cds_log(1, sprintf(['[cds_gen_init_pop] Auswertung %s geladen. Bewertung: %d. Bei %d/%d ', ...
      'Parametergrenzen passend.'], optdirs(i).name, score_i, sum(I_param_iO), size(pval_i,1)));
    if any(~I_param_iO)
      for jjj = find(~I_param_iO)'
        I_pniO = varlim(:,1)' > pval_i(jjj,:) | varlim(:,2)' < pval_i(jjj,:);
        for kkk = find(I_pniO)
          cds_log(3, sprintf(['[cds_gen_init_pop] Partikel %d/%d: Parameter ', ...
            '%d (%s) nicht passend. %1.2f < %1.2f < %1.2f'], ...
            jjj, size(pval_i,1), kkk, [d.RobotOptRes.Structure.varnames{kkk}], ...
            varlim(kkk,1), pval_i(jjj,kkk), varlim(kkk,2)));
        end
      end
    end
    InitPopLoadTmp = [InitPopLoadTmp; pval_i(I_param_iO,:)]; %#ok<AGROW>
    fval_mean_all = mean(fval_i(I_param_iO,:),2);
    ScoreLoad = [ScoreLoad; score_i-floor(log10(fval_mean_all))]; %#ok<AGROW>

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
% TODO: Bevorzuge Parameter, die sich voneinander entscheiden.

% Wähle die geladenen alten Ergebnisse
[~, I] = sort(ScoreLoad, 'descend');
InitPopLoad = InitPopLoadTmp(I,:);
nIndLoad = Set.optimization.InitPopRatioOldResults*nIndTotal;
nIndLoad = min(nIndLoad, size(InitPopLoad,1));

%% Zufällige Werte für alle Optimierungsparameter (normiert mit varlim)
nIndRand = nIndTotal - nIndLoad;
InitPopRand = repmat(varlim(:,1)', nIndRand,1) + rand(nIndRand, nvars) .* ...
                        repmat(varlim(:,2)'-varlim(:,1)',nIndRand,1);

% Alle PSO-Parameter durchgehen
for i = 1:nvars
  if vartypes(i) ~= 1
    % Nur Betrachtung von Kinematikparametern der Beinkette.
    % Alle anderen werden auf obige Zufallswerte gesetzt
    continue % nicht pkin parameter
  end
  if contains(varnames(i),{'theta'}) % theta parameter
    % Setze nur auf 0 oder pi/2. Das verspricht eine bessere Lösbarkeit der
    % Kinematik. Mache nur, wenn das mit den Grenzen passt.
    if varlim(i,2) >= pi/2 && varlim(i,1) <= 0
      InitPopRand(:,i) = pi/2 * round(rand(nIndRand,1)); 
      continue
    end
  end
  if contains(varnames(i),{'alpha'}) % alpha parameter
    % nur 0 oder pi/2
    if varlim(i,2) >= pi/2 && varlim(i,1) <= 0
      InitPopRand(:,i) = pi/2 * round(rand(nIndRand,1)); 
      continue
    end
  end
end
%% Mische die Populationen
InitPop = [InitPopLoad(1:nIndLoad,:); InitPopRand(1:nIndRand,:)];
cds_log(1, sprintf(['[cds_gen_init_pop] %d Partikel für Initialpopulation aus vorherigen ', ...
  'Optimierungen geladen. Davon %d genommen. Die restlichen %d zufällig.'], ...
  size(InitPopLoad,1), nIndLoad, nIndRand));