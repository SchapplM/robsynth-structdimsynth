% Füge PKM dauerhaft zur Bibliothek zu.
% Wähle nur Roboter aus, bei denen die IK und die Jacobi stimmen
% (Überprüfung mit modifizierter Optimierung in Maßsynthese)
% Probiere alle möglichen Beinketten aus der SerRobLib aus
% 
% TODO: Bei Abbruch des Skripts bleiben PKM mit "?"-Eintrag in der DB
% zurück. Abhilfe: remove_invalid_robots.m und remove_orphaned_entries.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-09
% (C) Institut für Mechatronische Systeme, Universität Hannover

function parroblib_add_robots_symact(settings)
%% Standardeinstellungen für Benutzereingabe
settings_default = struct( ...
  'check_existing', false, ... % Falls true: Prüfe existierende Roboter in Datenbank nochmal
  'check_missing', true, ... % Falls true: Prüfe auch nicht existierende Roboter
  'check_rankdef_existing', true, ... % Falls true: Prüfe existierende Roboter, deren Rang vorher als zu niedrig festgestellt wurde (zusätzlich zu anderen Optionen notwendig)
  'check_resstatus', 1:6, ... % Filter für PKM, die einen bestimmten Status in synthesis_result_lists/xTyR.csv haben
  'check_only_missing_joint_parallelity', false, ... % Filter für PKM, bei denen die Angabe der Beingelenk-Parallelität fehlt (für Roboternamen wichtig)
  'resstatus_downgrade_possible', true, ... % Bei erneuter Durchführung können auch PKM entfernt werden
  'lfdNr_min', 1, ... % Auslassen der ersten "x" kinematischer Strukturen (zum Debuggen)
  ... % Prüfung ausgewählter Beinketten (zum Debuggen):
  'whitelist_SerialKin', {''}, ... % z.B. 'S6RRPRRR14V2', 'S6RRPRRR14V3' 'S6RRRRRR10V3' 'S6PRRRRR6V2'
  ...% Alternative 1: Nur Beinketten mit Kugelgelenk-Ende
  'onlyspherical', false, ...
  ...% Alternative 2: Allgemeine Beinketten
  'onlygeneral', false, ...
  ... % Optionen zur Wahl nach anderen Kriterien
  'selectgeneral', true, ... % Auch allgemeine Modelle wählen
  'selectvariants', true, ... % Auch alle Varianten wählen
  'ignore_check_leg_dof', false, ... % Plausibilitätsregeln aus parrob_structsynth_check_leg_dof können ignoriert werden
  'allow_passive_prismatic', false, ... % Technisch sinnvoll. Zum Testen auf true setzen (z.B. für 2T0R und 2T1R PKM)
  'fixed_number_prismatic', NaN, ... % Vorgabe, wie viele Schubgelenke die Beinkette haben muss (NaN = egal)
  'comp_cluster', false, ... % Rechne auf PBS-Rechen-Cluster. Parallel-Instanz für G-/P-Kombis
  'compile_job_on_cluster', true, ... % Separater Job auf Cluster zum kompilieren der Mex-Funktionen
  'results_max_age', 30, ... % Keine alten Ergebnisse laden. Annahme: Wurden schon verarbeitet. In Tagen.
  'clustercomp_if_res_olderthan', 2, ... % Falls in den letzten zwei Tagen bereits ein vollständiger Durchlauf gemacht wurde, dann nicht nochmal auf dem Cluster rechnen. Deaktivieren durch Null-Setzen
  'clusterjobdepend', [], ...% Start-Abhängigkeit für alle Cluster-Jobs (z.B. Index-Erstellung der Datenbank
  'isoncluster', false, ... % Marker um festzustellen, dass gerade auf Cluster parallel gerechnet wird
  'optname', '', ... % Name, den die Optimierung auf dem Cluster haben soll (muss einheitlich sein)
  'dryrun', false, ... % Falls true: Nur Anzeige, was gemacht werden würde
  'offline', false, ... % Falls true: Keine Optimierung durchführen, stattdessen letztes passendes Ergebnis laden
  ... % ... dieser Modus kann genutzt werden, wenn die Optimierung korrekt durchgeführt wurde, aber die Nachverarbeitung fehlerhaft war
  'EE_FG_Nr', NaN, ... % Kann als Index für Variable EE_FG_ges benutzt werden. 1=2T0R, 2=2T1R, 3=3T0R, ...
  'EE_FG', [1 1 0 0 0 1], ... % Beispiel: 2T1R. Können auch mehrere gestapelt sein
  'parcomp_structsynth', 1, ... % parfor-Struktursynthese (schneller, aber mehr Speicher notwendig)
  'parcomp_mexcompile', 1, ... % parfor-Mex-Kompilierung (schneller, aber Dateikonflikt möglich)
  'use_mex', 1, ... % Die nutzung kompilierter Funktionen kann deaktiviert werden. Dann sehr langsam. Aber Start geht schneller, da keine Kompiliertung zu Beginn.
  'max_actuation_idx', 4, ... % Aktuierung bis zum vierten Gelenk-FG zulassen
  'base_couplings', 1:10, ... % siehe ParRob/align_base_coupling
  'plf_couplings', 1:9 ... % siehe ParRob/align_platform_coupling
  );

%% Benutzereingabe verwalten
if ~exist('settings', 'var')
  % Keine Eingabe in Programm. Nehme Standard-Einstellungen
  settings = settings_default;
end
% Prüfe, ob alle Felder in Eingabe vorhanden sind
for ftmp = fields(settings)'
  if ~isfield(settings_default, ftmp{1})
    warning('Feld %s in der Eingabestruktur ist nicht vorgesehen', ftmp{1})
  end
end
% Trage alle Felder der Eingabe ein (es dürfen auch Felder fehlen)
settings_new = settings_default;
for f = fields(settings)'
  if ~isfield(settings_new, f{1})
    error('Feld %s kann nicht übergeben werden', f{1});
  else
    settings_new.(f{1}) = settings.(f{1});
  end
end
settings = settings_new;
% Eingaben prüfen
assert(isscalar(settings.max_actuation_idx), 'max_actuation_idx muss Skalar sein');
% Eingaben nachverarbeiten
if settings.comp_cluster
  % Die Datenbank muss lokal geändert werden, um die Namen der neu zu
  % erstellenden PKM festzustellen. Sonst lässt sich nicht feststellen, ob
  % schon Ergebnisse vorliegen. Die Datenbank muss also eventuell vor der
  % Auswertung wieder zurückgesetzt werden.
  if settings.clustercomp_if_res_olderthan > 0
    if settings.dryrun && settings.clustercomp_if_res_olderthan == 0
      error(['Option "dryrun" nicht zusammen mit "comp_cluster" und ', ...
        '"clustercomp_if_res_olderthan" möglich.']);
    end
    if settings.offline == false || settings.dryrun == true
      warning(['Einstellungen offline (%d->%d) und dryrun (%d->%d) werden ', ...
        'neu gesetzt'], settings.offline, 1, settings.dryrun, 0);
    end
    settings.offline = true; % Es werden offline vorhandene Ergebnisse geprüft.
    settings.dryrun = false; % Dazu muss die Datenbank gefüllt werden
  else % Keine Prüfung der Offline-Ergebnisse
    if settings.offline == true || settings.dryrun == false
      warning(['Einstellungen offline (%d->%d) und dryrun (%d->%d) werden ', ...
        'neu gesetzt'], settings.offline, 0, settings.dryrun, 1);
    end
    settings.offline = false; % Kein Laden vorheriger Ergebnisse
    settings.dryrun = true; % Dann kein Füllen der Datenbank notwendig
  end
end
% Indizes der geprüften Freiheitsgrade bestimmen
EE_FG_ges = [ ...
  1 1 0 0 0 0; ...
  1 1 0 0 0 1; ...
  1 1 1 0 0 0; ...
  1 1 1 0 0 1; ...
  1 1 1 1 1 0; ...
  1 1 1 1 1 1];
if all(~isnan(settings.EE_FG_Nr))
  settings.EE_FG = EE_FG_ges(settings.EE_FG_Nr,:);
end
EE_FG_Nr = [];
for i = 1:size(EE_FG_ges,1)
  if any(all(repmat(EE_FG_ges(i,:),size(settings.EE_FG,1),1)==settings.EE_FG))
    EE_FG_Nr = [EE_FG_Nr, i]; %#ok<AGROW>
  end
end
%% Initialisierung
EE_FG_Mask = [1 1 1 1 1 1]; % Die FG müssen genauso auch vom Roboter erfüllt werden (0 darf nicht auch 1 sein)
serroblibpath=fileparts(which('serroblib_path_init.m'));
parroblibpath=fileparts(which('parroblib_path_init.m'));
assert(~isempty(serroblibpath), 'Seriell-Roboter-Datenbank ist nicht im Pfad initialisiert');
assert(~isempty(parroblibpath), 'Parallel-Roboter-Datenbank ist nicht im Pfad initialisiert');
if settings.comp_cluster
  assert(~isempty(which('jobStart.m')), 'Cluster-Repo ist nicht im Pfad initialisiert');
end
% Abhängigkeiten der Cluster-Jobs in Struktur sammeln
startsettings = struct('afterok', settings.clusterjobdepend, 'afternotok', [], 'afterany', []);
% zwei Tage lang in 5min-Abständen versuchen (falls Cluster voll und
% die Jobs nach und nach erst gestartet werden dürfen)
startsettings.waittime_max = 3600*24*2; %  2 Tage
startsettings.retry_interval = 60*5; % 5 Minuten
%% Alle PKM generieren
fprintf('Beginne Schleife über %d verschiedene EE-FG\n', length(settings.EE_FG_Nr));
for iFG = EE_FG_Nr % Schleife über EE-FG (der PKM)
  EE_FG = EE_FG_ges(iFG,:);
  EE_FG_Name = sprintf( '%dT%dR', sum(EE_FG(1:3)), sum(EE_FG(4:6)) );
  % Ergebnis-Tabelle initialisieren
  ResTab_empty = cell2table(cell(0,7), 'VariableNames', ...
    {'FG', 'G', 'P', 'Chain', 'ResultCode', 'NumRankSuccess', 'Remove'});
  ResTab = ResTab_empty;
  % Pfad mit vollständigen Ergebnissen der Struktursynthese
  parroblib_writelock('check', 'csv', logical(EE_FG), 600, true); % nicht lesen, wenn gleichzeitig geschrieben.
  synthrestable = readtable( ...
    fullfile(parroblibpath,'synthesis_result_lists',[EE_FG_Name,'.csv']), ...
    'ReadVariableNames', true);
  synthrestable_var = readtable( ...
    fullfile(parroblibpath,'synthesis_result_lists',[EE_FG_Name,'_var.csv']), ...
    'ReadVariableNames', true);
  if ~isempty(synthrestable_var)
    synthrestable = [synthrestable; synthrestable_var]; %#ok<AGROW>
  end
  % Aktuellen Stand der Kinematik-Datenbank öffnen
  kintabmatfile = fullfile(parroblibpath, ['sym_', EE_FG_Name], ['sym_',EE_FG_Name,'_list_kin.mat']);
  tmp = load(kintabmatfile); % erfordert parroblib_gen_bitarrays
  KinTab = tmp.KinTab;
  % Merker, ob Kompilieren serieller Ketten gestartet wurde. Wenn ja, dann
  % hier Job-IDs. Betrifft nur Cluster (s.u.). Das gleiche für PKM
  serrob_compile_jobId = []; parrob_compile_jobId = [];
  % Alle seriellen Beinketten, die bisher für die Kompilierung gestartet wurden
  LegChainListMexUpload = {};
  fprintf('Prüfe PKM mit %s Plattform-FG\n', EE_FG_Name);
  % Bestimme Möglichkeiten für Koppelpunkte
  [Cpl1_grid,Cpl2_grid] = ndgrid(settings.base_couplings,settings.plf_couplings);
  % Binär-Matrix zum Entfernen von Koppelpunkt-Kombinationen
  I1del = false(size(Cpl1_grid)); I2del = I1del;
  if iFG==1 % 2T1R: Nur G1P1 ist sinnvoll.
    I1del(Cpl1_grid>1) = true;
    I2del(Cpl2_grid>1) = true;
  end
  if all(EE_FG(1:5)==[1 1 1 0 0]) % 3T0R oder 3T1R: keine paarweise Anordnung
    I1del(Cpl1_grid>4&Cpl1_grid<9) = true; % Entferne G5 bis G8
    I2del(Cpl2_grid>3&Cpl2_grid<7) = true; % Entferne P5 und P6
  end
  if ~all(EE_FG == [1 1 1 0 0 0])
    % Nur für 3T0R ist die Methode 7 bisher implementiert
    I2del(Cpl2_grid==7) = true;
  end
  if all(EE_FG==[1 1 1 1 1 0]) % 3T2R: keine paarweise Anordnung
    I1del(Cpl1_grid>4&Cpl1_grid<9) = true; % nur Methode 1 bis 4 oder 9 ist sinnvoll
    I2del(Cpl2_grid>3&Cpl2_grid<8) = true; % nur Methode 1 bis 3 oder 8 ist sinnvoll
  end
  Cpl1_grid_filt = Cpl1_grid(~I1del&~I2del);
  Cpl2_grid_filt = Cpl2_grid(~I1del&~I2del);
  Coupling_all = [Cpl1_grid_filt(:),Cpl2_grid_filt(:)];
  Coupling_all = unique(Coupling_all,'row');
  for kk = 1:size(Coupling_all,1) % Schleife über Koppelpunkt-Möglichkeiten
    Coupling = Coupling_all(kk,:);
    if Coupling(1) > 10 || Coupling(2) > 9
      error('Fall nicht implementiert');
    end
    %% Serielle Beinketten auswählen
    N_Legs = sum(EE_FG); % Voll-Parallel: So viele Beine wie EE-FG
    if all(EE_FG == [1 1 1 0 0 0]) || all(EE_FG == [1 1 1 0 0 1])
      % PKM mit reduziertem FG dürfen keine 6FG-Beinketten haben
      % Die Beinketten müssen mindestens so viele FG wie die PKM haben
      % Alles weitere wird weiter unten gefiltert (kinematische Eigenschaften)
      LegDoF_allowed = 5:-1:N_Legs;
      if all(EE_FG == [1 1 1 0 0 0]) && Coupling(2) == 7
        % Methode P7 funktioniert nur mit Beinketten mit vier Gelenken
        LegDoF_allowed = 4;
      end
    elseif all(EE_FG == [1 1 0 0 0 1]) || all(EE_FG == [1 1 1 1 1 1]) || all(EE_FG == [1 1 1 1 1 0])
      LegDoF_allowed = N_Legs; % Fall 2T1R und 3T3R
    elseif all(EE_FG == [1 1 0 0 0 0])
      LegDoF_allowed = N_Legs; % Fall 2T0R (Platzhalter, um 2PP-PKM zu erzeugen
    else
      error('Fall nicht implementiert');
    end
    % EE_FG_Mask einstellen
    if all(EE_FG == [1 1 0 0 0 1])
      EE_FG_Mask = [1 1 0 0 0 1];
    elseif all(EE_FG == [1 1 1 0 0 0])
      EE_FG_Mask = [1 1 1 0 0 0];
    elseif all(EE_FG == [1 1 1 0 0 1]) % && Coupling(1) == 1
      EE_FG_Mask = [1 1 1 0 0 1];
    elseif all(EE_FG == [1 1 1 1 1 0])
      EE_FG_Mask = [1 1 1 0 0 0]; % Rotations-Komponente nicht angucken (Drehung wird teilw. durch Basis vorgegeben)
    elseif all(EE_FG == [1 1 1 1 1 1])
      EE_FG_Mask = [1 1 1 1 1 1];
    end
    
    l = struct('Names_Ndof', [],'AdditionalInfo',[], 'BitArrays_EEdof0', []);
    I_FG = [];
    for i = LegDoF_allowed
      mdllistfile_Ndof = fullfile(serroblibpath, sprintf('mdl_%ddof', i), sprintf('S%d_list.mat',i));
      l_tmp = load(mdllistfile_Ndof, 'Names_Ndof', 'AdditionalInfo', 'BitArrays_EEdof0');
      l.Names_Ndof = [l.Names_Ndof,l_tmp.Names_Ndof];
      l.AdditionalInfo = [l.AdditionalInfo;     l_tmp.AdditionalInfo];
      l.BitArrays_EEdof0 = [l.BitArrays_EEdof0; l_tmp.BitArrays_EEdof0];
      [~,I_FG_tmp] = serroblib_filter_robots(i, EE_FG, EE_FG_Mask);
      I_FG = [I_FG;I_FG_tmp]; %#ok<AGROW>
    end
    I_novar = (l.AdditionalInfo(:,2) == 0); % keine Modell-Varianten, nur Hauptmodelle
    I_var = (l.AdditionalInfo(:,2) == 1);
    % Nur die ersten drei Gelenke beeinflussen die Koppelgelenk-Position:
    % (Bedingung für Generierung symbolischen Codes und Bedingung für
    % "einfache Kinematik")
    I_spherical = (l.AdditionalInfo(:,1) == 3);
    
    if settings.onlyspherical && ~settings.onlygeneral
      % Nur Modell-Varianten, keine Hauptmodelle (Hauptmodelle haben sowieso
      % nicht die Eigenschaft der Koppelgelenk-Position):
      % Alle Eigenschaften kombinieren und Beinketten auswählen
      I = I_FG & I_var & I_spherical;
    elseif settings.onlygeneral && ~settings.onlyspherical
      % Eigenschaften kombinieren
      I = I_FG & I_novar;
    elseif settings.onlyspherical && settings.onlygeneral
      error('Kombination von ausschließenden Filtern nicht vorgesehen');
    else % Jetzt werden hinzufügende Filter getestet. Keine ausschließenden Filter gesetzt
      % Eigenschaften kombinieren, je nach gesetzter Einstellung
      I = false(size(I_FG)); % Erstmal ohne eine Auswahl anfangen
      if settings.selectgeneral
        I = I | I_FG & I_novar;
      end
      if settings.selectvariants
        I = I | I_FG & I_var;
      end
    end
    % Indizes der möglichen aktuierten Gelenke (wird später noch gefiltert)
    Actuation_possib = 1:settings.max_actuation_idx;
    II = find(I); % Umwandlung von Binär-Indizes in Nummern
    ii = 0; % Laufende Nummer für aktuierte PKM
    ii_kin = 0; % Laufende Nummer für Kinematik-Struktur der PKM
    % Variablen zum Erzeugen der Statistik
    num_rankloss = 0;
    num_dimsynthfail = 0;
    num_fullmobility = 0;
    num_isomorph = 0;
    num_checked_dimsynth = 0;
    % Prüfe ob gewünschte Liste von Beinketten in Auswahl vorhanden ist
    whitelist_notinDB = setdiff(settings.whitelist_SerialKin,l.Names_Ndof(II));
    if ~isempty(whitelist_notinDB) && ~isempty(whitelist_notinDB{1})
      warning('%d/%d Einträge aus Auswahl-Liste nicht in Datenbank: %s', ...
        length(whitelist_notinDB), length(settings.whitelist_SerialKin), ...
        disp_array(whitelist_notinDB, '%s'));
    end
    fprintf('G%dP%d: Beginne Schleife über %d (prinzipiell) mögliche Beinketten-Kinematiken\n', ...
      Coupling(1), Coupling(2), length(II));
    
    Whitelist_PKM = {}; Whitelist_Leg = {}; LegChainList_Coupling = {};
    tlm_iFKloop = tic(); % zur Speicherung des Zeitpunkts der letzten Meldung
    % Sperre csv-Dateien während des Hinzufügens. Dieser Abschnitt ist
    % kritisch für parallel arbeitende weitere Instanzen der Synthese
    if ~settings.dryrun
      parroblib_writelock('lock', 'csv', logical(EE_FG), 30*60, true);
    end
    if settings.isoncluster
      pause(10.0); % Warte nach dem Sperren der csv-Tabellen, damit check-Befehl anderer Instanzen auslaufen kann.
    end
    for iFK = II' % Schleife über serielle Führungsketten
      ii_kin = ii_kin + 1;
      SName = l.Names_Ndof{iFK};
      PName = sprintf('P%d%s', N_Legs, SName(3:end));
      PNameGP = sprintf('%sG%dP%d', PName, Coupling(1), Coupling(2));
      SName_TechJoint = fliplr(regexprep(num2str(l.AdditionalInfo(iFK,7)), ...
        {'1','2','3','4','5'}, {'R','P','C','U','S'}));
      if toc(tlm_iFKloop) > 10 % nach 10s neue Meldung ausgeben
        fprintf('Kinematik %d/%d: Beinkette %s (%s)\n', ii_kin, length(II), SName, SName_TechJoint);
        tlm_iFKloop = tic(); % Zeitpunkt der letzten Meldung abspeichern
      end
      if ~isempty(settings.whitelist_SerialKin) && ~any(strcmp(settings.whitelist_SerialKin, SName))
        % Aktuelle Roboterstruktur für Beinketten nicht in Positivliste
        continue
      end
      if sum(SName=='P') ~= settings.fixed_number_prismatic
        continue
      end
      
      % Öffnen der csv-Datei mit allen Ergebnissen und Abgleich, ob
      % schon geprüft ist. Nur wenn Filter-Option aktiviert ist.
      % (Bei Standard-Einstellung "1:6" gibt es nichts zu filtern
      if ~(length(settings.check_resstatus) == 6 && all(settings.check_resstatus==1:6))
        % Tabelle nach der gesuchten PKM filtern
        I_name = strcmp(table2cell(synthrestable(:,1)), SName);
        I_coupl = table2array(synthrestable(:,3))==Coupling(1) & ...
                  table2array(synthrestable(:,4))==Coupling(2);
        i_restab = find(I_name & I_coupl);
        % aktuellen Status feststellen
        if isempty(i_restab)
          Status_restab = 6; % Werte als "nicht geprüft"
        elseif length(i_restab) > 1
          error('Doppelter Eintrag in CSV-Tabelle für %s', PNameGP);
        else
          Status_restab = table2array(synthrestable(i_restab,5));
        end
        % Vergleichen von Liste zu prüfender Status-Werte
        if ~any(Status_restab == settings.check_resstatus)
          continue
        end
      end
      if settings.check_only_missing_joint_parallelity
        I = strcmp(KinTab.Name, PNameGP);
        if sum(I) == 1
          if ~all(isnan(KinTab.Gelenkgruppen{I}))
            fprintf('Kinematik %s ist bereits mit Gelenkgruppen [%s] in Datenbank. Überspringe.\n', ...
              PNameGP, disp_array(KinTab.Gelenkgruppen{I}, '%d'));
            continue
          end
        end
      end

      if sum(SName=='P')>1 && ~settings.allow_passive_prismatic
        % Hat mehr als ein Schubgelenk. Kommt nicht für PKM in Frage.
        % (es muss dann zwangsläufig ein Schubgelenk passiv sein)
        parroblib_update_csv(SName, Coupling, logical(EE_FG), 1, 0);
        continue
      end
      
      if SName_TechJoint(1) == 'S'
        % Nur die Gestell-Konfigurationen 1 (Kreisförmig) und 5 (Paar-
        % weise) sind unterscheidbar. Siehe align_base_coupling.
        if all(Coupling(1) ~= [1 5])
          parroblib_update_csv(SName, Coupling, logical(EE_FG), 8, 0);
          fprintf(['Beinkette %s (%s) mit Gestell-Koppelgelenk Nr. %d wird ', ...
            'aufgrund der Kugelgelenk-Isomorphismen verworfen.\n'], ...
            SName, SName_TechJoint, Coupling(1));
          continue
        end
      end
      if SName_TechJoint(end) == 'S'
        % Nur die Plattform-Konfigurationen 1 (Kreisförmig) und 4 (Paar-
        % weise) sind unterscheidbar. Alle anderen lassen sich bei Kugel-
        % gelenken darauf zurückführen. Siehe align_platform_coupling.
        if all(Coupling(2) ~= [1 4])
          parroblib_update_csv(SName, Coupling, logical(EE_FG), 8, 0);
          fprintf(['Beinkette %s (%s) mit Plattform-Koppelgelenk Nr. %d wird ', ...
            'aufgrund der Kugelgelenk-Isomorphismen verworfen.\n'], ...
            SName, SName_TechJoint, Coupling(2));
          continue
        end
      end
      if SName(2+1) == 'R' && Coupling(1) == 10
        % Methode 10 ist identisch zu Methode 1, wenn das Gestellgelenk ein Dreh-
        % gelenk ist. Dann spielt die Ausrichtung um die z-Achse keine Rolle.
        continue
      end
      if strcmp(SName(3:4), 'PR') && Coupling(1) == 10
        % Methode 10 ist identisch zu Methode 1, wenn nach einem
        % Schubgelenk direkt ein dazu paralleles Drehgelenk kommt.
        % Gilt nur, wenn es keinen Versatz gibt. Gelenke also koaxial. Ist
        % durch Implementierung in der Maßsynthese gegeben.
        [~,PS] = serroblib_create_robot_class(SName, '', true);
        if PS.alpha(2) == 0
          fprintf(['Beinkette %s verworfen, da Gestell-Koppelgelenk G10 ', ...
            'und G1 in diesem Fall identisch sind\n'], SName);
          continue
        end
      end
      
      N_LegDoF = str2double(SName(2));% Beinkette FHG
      


      fprintf('Kinematik %d/%d: %s, %s\n', ii_kin, length(II), PName, SName);

      % Beinketten-FG aus Datenbank auslesen:
      EE_dof_legchain = dec2bin(l.BitArrays_EEdof0(iFK,:))=='1'; % FG vom Typ [1 1 1 0 0 1 0 0 1]
      if length(EE_dof_legchain)<6 % bei 2T0R oder 3T0R-Beinketten, sind zu wenige Stellen gesetzt
        EE_dof_legchain = [EE_dof_legchain, false(1,6-length(EE_dof_legchain))]; %#ok<AGROW>
      end

      % Plausibilitäts-Prüfungen basierend auf Beinketten und Kopplung
      % Beinketten-FG auf Plausibilität prüfen
      if ~settings.ignore_check_leg_dof % Kann testweise deaktiviert werden
        leg_success = parrob_structsynth_check_leg_dof(SName, Coupling, EE_FG, EE_dof_legchain);
        if ~leg_success
          fprintf('Beinkette %s mit Koppelpunkt-Nr. %d-%d wird aufgrund geometrischer Überlegungen verworfen.\n', ...
            SName, Coupling(1), Coupling(2));
          parroblib_update_csv(SName, Coupling, logical(EE_FG), 2, 0);
          continue
        end
      end
      for jj = Actuation_possib % Schleife über mögliche Aktuierungen
        ii = ii + 1;
        if ii < settings.lfdNr_min, continue; end % Starte erst später
        % Prüfe schon hier auf passive Schubgelenke (weniger Rechenaufwand)
        IdxP = (SName(3:3+N_LegDoF-1)=='P'); % Nummer des Schubgelenks finden
        if ~settings.allow_passive_prismatic && any(IdxP) && find(IdxP)~=jj
          continue % Es gibt ein Schubgelenk und es ist nicht das aktuierte Gelenk
        end
        % Prüfe, ob ein Teil eines technischen Gelenks (Kardan, Kugel
        % aktuiert werden würde).
        % Das letzte positionsbeeinflussende Gelenk ist das letzte 
        % aktuierte Gelenk. Danach kommt nur noch das Koppelgelenk (Kardan/Kugel)
        if jj > l.AdditionalInfo(iFK,1) % siehe serroblib_gen_bitarrays.
          continue
        end
        % Prüfe auch technische Gelenke am Anfang der Beinkette. Z.B. keine
        % Aktuierung eines gestellfesten Kardan-Gelenks möglich
        Joints_Actuation_Possible = [];
        for iii = 1:length(SName_TechJoint)
          switch SName_TechJoint(iii)
            case 'R'
              Joints_Actuation_Possible = [Joints_Actuation_Possible, 1]; %#ok<AGROW>
            case 'P'
              Joints_Actuation_Possible = [Joints_Actuation_Possible, 1]; %#ok<AGROW>
            case 'U'
              Joints_Actuation_Possible = [Joints_Actuation_Possible, [0 0]]; %#ok<AGROW>
            case 'S'
              Joints_Actuation_Possible = [Joints_Actuation_Possible, [0 0 0]]; %#ok<AGROW>
            case 'C'
              Joints_Actuation_Possible = [Joints_Actuation_Possible, [0 0]]; %#ok<AGROW>
          end
        end
        if ~Joints_Actuation_Possible(jj)
          fprintf('Aktuierung von Gelenk %d in %s (%s) nicht möglich\n', jj, SName, SName_TechJoint);
          continue
        end
        fprintf('Untersuchte PKM %d (Gestell %d, Plattform %d): %s mit symmetrischer Aktuierung Gelenk %d\n', ...
          ii, Coupling(1), Coupling(2), PName, jj);
        Actuation = cell(1,N_Legs);
        Actuation(:) = {jj};
        LEG_Names = {SName};
        %% Roboter pauschal zur Datenbank hinzufügen
        % Mit dem dann eindeutigen Robotermodell sind weitere Berechnungen
        % möglich: Prüfe, ob PKM in Datenbank ist
        [found_tmp, Name] = parroblib_find_robot(N_Legs, LEG_Names, Actuation, Coupling, true);
        found = found_tmp(2); % Marker, dass Aktuierung der PKM gespeichert war
        if ~found && ~settings.check_missing
          % nicht in DB. Soll nicht geprüft werden. Weiter
          fprintf('PKM %s ist nicht in Datenbank. Keine weitere Untersuchung.\n', PName);
          continue
        elseif found && settings.check_existing
          % in Datenbank. Soll auch geprüft werden.
          fprintf('PKM %s ist in Datenbank. Führe Untersuchung fort.\n', Name);
        elseif found && ~settings.check_existing
          fprintf('PKM %s ist in Datenbank. Überspringe nochmalige Prüfung.\n', Name);
          continue
        elseif ~found && settings.check_missing
          % Nicht in DB. Soll geprüft werden. Füge hinzu
          if ~settings.dryrun
            [Name, new] = parroblib_add_robot(N_Legs, LEG_Names, Actuation, Coupling, EE_FG);
            if new, fprintf('PKM %s zur Datenbank hinzugefügt. Jetzt weitere Untersuchung\n', Name);
            else,   error('PKM erst angeblich nicht in DB enthalten, jetzt aber doch'); end
            [~, PNames_Akt] = parroblib_filter_robots(EE_FG, 6);
            Icheck = find(strcmp(PNames_Akt, Name));
          else
            fprintf('Der Roboter %s würde zur Datenbank hinzugefügt werden\n', PName);
            Name = '<Neuer Name>';
            % Setze Status 6 ("noch nicht geprüft").
            parroblib_update_csv(SName, Coupling, logical(EE_FG), 6, 0);
          end
        else
          error('Dieser Fall darf nicht eintreten. Nicht-logische Eingabe');
        end
        Whitelist_PKM = [Whitelist_PKM;{Name}]; %#ok<AGROW>
        if ~settings.dryrun % Liste nur bei Produktiv-Lauf notwendig
          [~, ~, ~, ~, ~, ~, ~, ~, PName_Leg_tmp] = parroblib_load_robot(Name,0);
          Whitelist_Leg = [Whitelist_Leg, PName_Leg_tmp]; %#ok<AGROW>
        end
      end % for jj (actuation)
      % Merke die Beinkette vor. Bei mehreren Koppelgelenken mehrfache
      % Eintragung. Daher Doppelte wieder entfernen.
      LegChainList_Coupling = unique([LegChainList_Coupling, SName]);
    end % for iFK (serielle Kette)
    save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', ...
      sprintf('parroblib_add_robots_symact_%s_0.mat', EE_FG_Name)));
    % Aktualisiere die mat-Dateien (werden für die Maßsynthese benötigt)
    if ~settings.dryrun, parroblib_gen_bitarrays(logical(EE_FG)); end
    if ~settings.dryrun
      parroblib_writelock('free', 'csv', logical(EE_FG), 0, true);
    end
    if isempty(Whitelist_PKM)
      fprintf('Für FG %s und G%dP%d gibt es keine PKM.\n', EE_FG_Name, Coupling(1), Coupling(2));
      continue
    end
    % Zeichenfolge zum Erstellen von eindeutigen Namen
    if settings.selectvariants, varstr = 'v'; else, varstr = ''; end
    if settings.selectgeneral, genstr = 'g'; else, genstr = ''; end
    
    if settings.dryrun && ~settings.comp_cluster, continue; end
    %% Alle Matlab-Funktionen generieren
    % Muss hier gemacht werden, da später nicht mehr zwischen den G-/P-Nummern
    % unterschieden wird. Die Kinematik-Funktionen sind dort identisch.
    % TODO: Nicht machen, wenn Roboter schon in Datenbank war. Also vorher
    % prüfen, ob existient
    % TODO: Auch andere Funktionen kompilieren? Ja: Sonst kommt eine
    % automatische Kompilierung in der Struktursynthese
    % TODO: Parallel kompilieren, seriell generieren
    Whitelist_Kin = cell(length(Whitelist_PKM),1);
    for i = 1:length(Whitelist_PKM)
      Whitelist_Kin{i} = Whitelist_PKM{i}(1:end-2);
    end
    % Duplikate entfernen (falls mehr als eine Aktuierung erzeugt wird)
    Whitelist_Kin = unique(Whitelist_Kin);
    Whitelist_Leg = unique(Whitelist_Leg);
    save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', ...
      sprintf('parroblib_add_robots_symact_%s_1.mat', EE_FG_Name)));
    if ~settings.offline && ~settings.comp_cluster
      kompstr = '';
      if settings.use_mex, kompstr=' und kompiliere anschließend'; end
      fprintf('Generiere Template-Funktionen für %d Roboter%s.\n', ...
        length(Whitelist_Kin), kompstr);
      % Erzeuge alle Template-Dateien neu (ohne Kompilierung). Dadurch wird
      % sichergestellt, dass sie die richtige Version haben.
      for i = 1:length(Whitelist_Kin)
        parroblib_writelock('check', 'csv', logical(EE_FG), 10*60, true);
        parroblib_create_template_functions(Whitelist_Kin(i),false,false);
      end
      % Benötigte Funktionen kompilieren (nur wenn Struktursynthese
      % parallel). Bei serieller Struktursynthese wird dort kompiliert.
      if settings.parcomp_structsynth == 1
        parfor (i = 1:length(Whitelist_Kin), settings.parcomp_mexcompile*12)
          % Lesen der csv-Tabelle für Funktionskompilierung notwendig.
          % Hoffe, dass direkt beim Freiwerden nicht schon wieder
          % geschrieben wird (deshalb oben an gleichwertiger Stelle Pause)
          % (unproblematisch, wenn Parallelinstanz auf eigener Kopie der
          % ParRobLib arbeitet)
          parroblib_writelock('check', 'csv', logical(EE_FG), 10*60, true);
          % Erzeuge Klasse. Dafür Aktuierung A1 angenommen. Ist aber für
          % Generierung der Funktionen egal.
          RP = parroblib_create_robot_class([Whitelist_Kin{i},'A1'],1,1);
          % Hierdurch werden fehlende mex-Funktionen kompiliert.
          if settings.use_mex %#ok<PFBNS>
            parroblib_writelock('lock', Whitelist_Kin{i}, logical(EE_FG), 60*60, true);
            RP.fill_fcn_handles(true, true);
            parroblib_writelock('free', Whitelist_Kin{i}, logical(EE_FG), 0, true);
          end
        end
      end
    end
    %% Maßsynthese für Liste von Robotern durchführen
    % Mit dem dann eindeutigen Robotermodell sind weitere Berechnungen
    % möglich
    num_checked_dimsynth = num_checked_dimsynth + 1;
    save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', ...
      sprintf('parroblib_add_robots_symact_%s_2.mat', EE_FG_Name)));
    % Damit wird geprüft, ob das System sinnvoll ist
    Set = cds_settings_defaults(struct('DoF', EE_FG));
    Set.task.Ts = 1e-2;
    Set.task.Tv = 1e-1;
    Set.task.profile = 1; % Komplette Trajektorie mit Geschwindigkeit und Zeitverlauf
    Set.task.maxangle = 5*pi/180; % Reduzierung der Winkel auf 5 Grad (ist für FG-Untersuchung ausreichend)
    if all(EE_FG==[1 1 1 1 1 0])
      Set.task.maxangle = 3*pi/180;
    end
    Traj = cds_gen_traj(EE_FG, 1, Set.task);
    if all(EE_FG==[1 1 1 1 1 0])
      % Verändere die Trajektorie so, dass keine parallele Stellung der
      % Plattform zum Gestell auftritt. Bei den meisten 3T2R-PKM ist das
      % eine Singularität. Dann springen die Gelenkwinkel wegen numerischer 
      % Probleme. TODO: Besser abfangen. Auch klären: Ist die Eignung auch 
      % in Null-Stellung Voraussetzung für erfolgreiche Struktursynthese?
      Traj.X(:,4:5) = Traj.X(:,4:5) + 4*pi/180;
      Traj.XE(:,4:5) = Traj.XE(:,4:5) + 4*pi/180;
    end
    % cds_show_task(Traj, Set.task);
    Set.optimization.objective = 'valid_act';
    rs = ['a':'z', 'A':'Z', '0':'9'];
    if ~isempty(settings.optname)
      % Name für die Berechnung auf dem Cluster wurde bereits beim Start
      % vorgegeben und darf auf dem Cluster nicht mehr geändert werden.
      % Sonst geht der Finish-Job nicht.
      Set.optimization.optname = settings.optname;
    else
      Set.optimization.optname = sprintf('add_robots_sym_%s_G%dP%d_tmp_%s_%s%s_%s', ...
        EE_FG_Name, Coupling(1), Coupling(2), datestr(now,'yyyymmdd_HHMMSS'), ...
        genstr, varstr, rs(randi([1 length(rs)], 5, 1))); % zufällige String anhängen, falls Sekundengleicher Start einer Optimierung
    end
    Set.optimization.NumIndividuals = 200;
    Set.optimization.MaxIter = 50;
    Set.optimization.ee_rotation = false;
    Set.optimization.ee_translation = false;
    Set.optimization.movebase = false;
    % Die Größe von Plattform und Gestell muss mit optimiert werden. Sonst
    % ist alles von der Standard-Einstellung abhängig (bei 3T2R dann z.B.
    % Gelenkgeschwindigkeiten immer zu groß, da anscheinend stark durch
    % Plattform-Geometrie und weniger durch Beinketten beeinflusst).
    Set.optimization.base_size = true;
    Set.optimization.platform_size = true;
    % Volle Umdrehungen der Drehgelenke erlauben (geht eher um
    % mathematische Plausibilität der PKM
    Set.optimization.max_range_active_revolute = 2*pi;
    Set.general.max_retry_bestfitness_reconstruction = 1;
    Set.general.plot_details_in_fitness = 0e3;
    Set.general.plot_robot_in_fitness = 0e3;
    Set.general.noprogressfigure = true;
    % Reduziere Log-Level der Konsolen-Ausgabe auf Cluster. Bei massiv par-
    % alleler Struktursynthese läuft der Speicher auf dem Cluster sonst voll.
    if settings.comp_cluster
      Set.general.verbosity = 1;
    else % Lokal immer volle Ausgabe, da meistens zum Debugging benutzt
      Set.general.verbosity = 3;
    end
    % Auf dem Cluster alle Roboter in einem Job (auf einer Node)
    % simulieren. Die Ergebnisse werden teilweise schnell fertig. Ein
    % Roboter pro Node wäre unwirtschaftlich.
    Set.general.cluster_maxrobotspernode = inf;
    % Erhöhe Grenzen für maximale Geschwindigkeiten. Für Erfolg der Struktur-
    % synthese zählt eher die Plausibilität (Sprünge, Singularitäten) als
    % die technische Umsetzbarkeit für die Beispiel-Trajektorie
    Set.optimization.max_velocity_active_revolute = 50;
    Set.optimization.max_velocity_passive_revolute = 50;
    Set.optimization.max_velocity_active_prismatic = 20;
    % Ignoriere auch Grenzen für Gelenkwinkel. Die technische Realisierbarkeit
    % steht nicht im Vordergrund und lässt sich durch weitere Optimierung
    % erreichen.
    Set.optimization.max_range_active_revolute = inf;
    Set.optimization.max_range_passive_universal = inf;
    Set.optimization.max_range_passive_spherical = inf;
    Set.optimization.max_range_prismatic = inf;
    Set.optimization.InitPopFromGlobalIndex = true; % Sonst sehr lange Wartezeit am Anfang der Synthese
    Set.general.matfile_verbosity = 0;
    Set.general.nosummary = true; % Keine Bilder erstellen.
    Set.structures.mounting_parallel = 'floor'; % Sieht plausibler aus auf Bildern.
    Set.structures.prismatic_cylinder_no_lever = false; % allgemeine Parametrierung. Für den fall ohne Hebel gibt es eigene Beinketten-Varianten.
    Set.structures.whitelist = Whitelist_PKM; % nur diese PKM untersuchen
    Set.structures.use_serial = false; % nur PKM (keine seriellen)
    Set.structures.use_parallel_rankdef = 6*settings.check_rankdef_existing;
    Set.structures.parrob_basejointfilter = settings.base_couplings;
    Set.structures.parrob_platformjointfilter = settings.plf_couplings;
    Set.general.save_animation_file_extensions = {'gif'};
    Set.general.parcomp_struct = settings.parcomp_structsynth;
    Set.general.use_mex = settings.use_mex;
    Set.general.compile_missing_functions = true; % wurde schon weiter oben gemacht. Mache nochmal, da es manchmal nicht funktioniert (unklare Gründe, womöglich Synchronisationsprobleme der parallelen Ausführung)
    if ~settings.isoncluster % Füge den Ergebnisordner aus der Projektablage hinzu
      Set.optimization.result_dirs_for_init_pop = {fullfile(fileparts( which(...
        'robsynth_projektablage_path.m')), '03_Entwicklung', 'Struktursynthese', 'Ergebnisordner_Optimierung')};
    end
    offline_result_complete = false;
    if ~settings.offline && ~settings.comp_cluster
      fprintf(['Starte Prüfung des Laufgrads der PKM mit Maßsynthese für ', ...
        '%d Roboter\n'], length(Whitelist_PKM));
      cds_start(Set,Traj);
      resmaindir = fullfile(Set.optimization.resdir, Set.optimization.optname);
      dssetfile = fullfile(resmaindir, [Set.optimization.optname, '_settings.mat']);
      if ~exist(dssetfile, 'file')
        % Logik-Fehler. Speichere Status zum Debuggen.
        fprintf('Beginne Komprimierung der PKM-Datenbank für Debug-Abbild\n');
        tmpdir = fullfile(Set.optimization.resdir, Set.optimization.optname, 'tmp');
        mkdirs(tmpdir);
        save(fullfile(tmpdir, 'parroblib_add_robots_symact_debug_norobots.mat'));
        if settings.isoncluster %  Auf dem Cluster wird im Tmp-Ordner einer Node gerechnet.
          % Sichere PKM-Datenbank zum Debuggen und aktuellen Status, sonst ist er weg.
          zip(fullfile(tmpdir, 'parroblib.zip'), parroblibpath);
        end
        error(['Einstellungsdatei %s existiert nicht. Logik-Fehler. Tmp-', ...
          'Daten in Ergebnis-Ordner gespeichert'], dssetfile)
      end
      ds = load(dssetfile);
      Structures = ds.Structures;
    elseif settings.offline
      % Debug: Lade Ergebnisse aus temporärem Cluster-Download-Ordner
      % Set.optimization.resdir = '/mnt/FP500/IMES/CLUSTER/REPO/structgeomsynth/dimsynth/results';
      % Finde den Namen der letzten Optimierung. Nur der Zeitstempel darf
      % anders sein.
      reslist = dir(fullfile(Set.optimization.resdir,[Set.optimization.optname(1:29),'*']));
      % Suche das neuste Ergebnis aus der Liste und benutze es als Namen
      if isempty(reslist)
        if ~settings.comp_cluster
          % Falls auf Cluster gerechnet werden soll, war das hier nur eine
          % Prüfung.
          warning(['Offline-Modus gewählt, aber keine Ergebnisse für %s* im ', ...
            'passenden Ordner.'], Set.optimization.optname(1:29));
          continue % Beendet Prüfung dieser Koppelgelenk-Kombination
        end
      else
        % Suche den Ergebnis-Ordner mit der größten Übereinstimmung
        reslist_nummatch = zeros(length(reslist),1); % Anzahl der Treffer
        reslist_rationomatch = zeros(length(reslist),1); % Anzahl der unpassenden Ergebnisse im Ordner
        reslist_age = inf(length(reslist),1); % Alter der durchgeführten Optimierungen in Ergebnisordner
        for i = 1:length(reslist) % Alle Ergebnis-Ordner durchgehen
          % Prüfe, wie viele passende Ergebnisse in dem Ordner sind
          Whitelist_PKM_match = false(length(Whitelist_PKM),1);
          reslist_pkm = dir(fullfile(reslist(i).folder, reslist(i).name, 'Rob*_Endergebnis.mat'));
          reslist_pkm_names = cell(length(reslist_pkm),1); % PKM-Namen der Ergebnisse
          for j = 1:length(reslist_pkm) % Alle Endergebnisse
            % Bestimme den PKM-Namen zu diesem Ergebnis
            [tokens, match] = regexp(reslist_pkm(j).name, ...
              'Rob[\d]+_([A-Za-z0-9_]+)_Endergebnis\.mat', 'tokens', 'match');
            reslist_pkm_names{j} = tokens{1}{1};
            % Prüfe, ob die gesuchten PKM zu diesem Ergebnis passen
            for k = find(~Whitelist_PKM_match)' % Suche nur nach bisher noch nicht gefundenen
              if contains(reslist_pkm(j).name, Whitelist_PKM{k})
                Whitelist_PKM_match(k) = true; % für gesuchte PKM liegt ein Endergebnis vor
                break;
              end
            end
          end
          % Prüfe zusätzlich, ob Informationen zu den Ergebnissen in der
          % csv-Tabelle stehen. Dann auch Aussage ohne mat-Dateien möglich
          csvfile = fullfile(reslist(i).folder, reslist(i).name, ...
            sprintf('%s_results_table.csv', reslist(i).name));
          if exist(csvfile, 'file')
            ResData_i = readtable(csvfile, 'HeaderLines', 2);
            ResData_i_headers = readtable(csvfile, 'ReadVariableNames', true);
            if size(ResData_i, 1) > 0 % Leere Tabelle führt zu Fehler
              ResData_i.Properties.VariableNames = ResData_i_headers.Properties.VariableNames;
              for k = 1:length(Whitelist_PKM_match)
                Whitelist_PKM_match(k) = any(strcmp(ResData_i.Name, Whitelist_PKM{k}));
              end
              reslist_pkm_names = [reslist_pkm_names; ResData_i.Name]; %#ok<AGROW> 
            end
          end
          % Alter des Ordners bestimmen (aus bekanntem Namensschema)
          [datestr_match, ~] = regexp(reslist(i).name,'[A-Za-z0-9_]*_tmp_(\d+)_(\d+)', 'tokens','match');
          if isempty(datestr_match)
            warning('Ergebnis-Ordner "%s" passt nicht ins Datums-Namensschema. Überspringe.', reslist(i).name);
            continue
          end
          date_i = datenum([datestr_match{1}{1}, ' ', datestr_match{1}{2}], 'yyyymmdd HHMMSS');
          reslist_age(i) = now() - date_i; % Alter in Tagen
          % Prüfe, ob die Einstellungsdatei mit der Tabelle übereinstimmt
          settingsfile=fullfile(reslist(i).folder, reslist(i).name, ...
            sprintf('%s_settings.mat', reslist(i).name));
          if exist(settingsfile, 'file')
            try
              tmpset = load(settingsfile, 'Structures');
            catch err
              warning('Fehler beim Laden von Einstellungsdatei für %s. Beschädigte Daten? %s', reslist(i).name, err.message);
              continue;
            end
            Structures_Names_i = cell(length(tmpset.Structures),1);
            for lll = 1:length(tmpset.Structures)
              Structures_Names_i{lll} = tmpset.Structures{lll}.Name;
            end
            % Alle Ergebnisse müssen auch in der Einstellungsdatei sein.
            % Sonst ist es ein inkonsistenter Datensatz.
            if ~isempty(intersect(setxor(Structures_Names_i, ...
                reslist_pkm_names), Structures_Names_i))
              warning('Ergebnisse in %s passen nicht zu Einstellungsdatei', reslist(i).name);
              continue
            end
            % Folgender Fall darf nicht vorkommen, außer die Einstellungen
            % werden durch Programm-/Benutzerfehler neu überschrieben.
            if exist(csvfile, 'file') && length(tmpset.Structures) < max(ResData_i.LfdNr)
              warning(['Inkonsistente Daten für %s (weniger Strukturen in ', ...
                'Einstellung/Eingabe als Ergebnisse in Tabelle/Ausgabe)'], reslist(i).name);
              continue
            end
          end
          reslist_nummatch(i) = sum(Whitelist_PKM_match); % Anzahl der Treffer
          % Verhältnis der gefundenen PKM: Berücksichtige freie alpha-/theta-
          % Parameter, die zu doppelten Ergebnissen für einen Namen führen.
          reslist_rationomatch(i) = 1 - reslist_nummatch(i)/length(unique(reslist_pkm_names));
        end % for i
        reslist_rationomatch(isnan(reslist_rationomatch)) = 0;
        % Vor-Filterung der Ergebnisliste
        III_age = reslist_age < settings.results_max_age;
        III_anymatch = reslist_nummatch > 0;
        III = III_age & III_anymatch;
        IIRL = find(III);
        if isempty(IIRL)
          fprintf(['Keiner der %d Ergebnis-Ordner ist brauchbar. Bei %d ', ...
            'passendes Alter, bei %d irgendein passendes Ergebnis\n'], length(II), ...
            sum(III_age), sum(III_anymatch));
          I_reslist = 1; % Dummy-Definition (wird nicht verwendet)
        else
          % Bestimme das am sinnvollsten auszuwählendste (vorgefilterte) Ergebnis:
          [~, I_reslist] = max(reslist_nummatch(IIRL)/length(Whitelist_PKM) ... % Nehme möglichst vollständige Ordner
            - reslist_age(IIRL)*0.05 ... aber ziehe 5% für jeden vergangenen Tag ab, ...
              ... % damit nicht ein sehr altes vollständiges Ergebnis immer genommen wird
            - reslist_rationomatch(IIRL)*0.10); % Bestrafe nicht passende Einträge
          Set.optimization.optname = reslist(IIRL(I_reslist)).name;
          % Erstelle Variablen, die sonst in cds_start entstehen
          csvfile = fullfile(reslist(IIRL(I_reslist)).folder, reslist(IIRL(I_reslist)).name, ...
            sprintf('%s_results_table.csv', reslist(IIRL(I_reslist)).name));
          use_csv = true;
          if ~exist(csvfile, 'file')
            use_csv = false;
          else
            ResData_i = readtable(csvfile, 'HeaderLines', 2);
            if size(ResData_i, 1) == 0 % Leere Tabelle führt zu Fehler
              use_csv = false;
            end
          end
          % Erstelle Cell-Array mit allen Roboter-Strukturen (Platzhalter-Var.)
          if use_csv
            ResData_i_headers = readtable(csvfile, 'ReadVariableNames', true);
            ResData_i.Properties.VariableNames = ResData_i_headers.Properties.VariableNames;
            Structures = cell(1,max(ResData_i.LfdNr),1);
            for i = 1:size(ResData_i,1)
              if isnan(ResData_i.LfdNr(i))
                warning('Datei %s scheint beschädigt zu sein', csvfile);
                continue
              end
              Structures{ResData_i.LfdNr(i)} = struct('Name', ResData_i.Name{i}, 'Type', 2);
            end
          else
            roblist = dir(fullfile(Set.optimization.resdir, Set.optimization.optname, ...
              'Rob*_Endergebnis.mat')); % Die Namen aller Roboter sind in den Ergebnis-Dateien enthalten.
            [tokens, ~] = regexp({roblist(:).name},'Rob(\d+)_([A-Za-z0-9]+)_Endergebnis\.mat','tokens','match');
            % Nach Nummer der Roboter sortieren (für nachträgliche Erzeugung der
            % Ergebnis-Tabelle zum korrekten Laden der mat-Dateien)
            robnum_ges = zeros(length(tokens),1);
            for i = 1:length(tokens)
              robnum_ges(i) = str2double(tokens{i}{1}{1});
            end
            [~,I_sortres] = sort(robnum_ges);
            Structures = cell(1,length(I_sortres)); istr = 0;
            for i = I_sortres(:)'
              istr = istr + 1;
              Structures{istr} = struct('Name', tokens{i}{1}{2}, 'Type', 2); %#ok<SAGROW>
            end
          end
          % Erstelle auch die csv-Tabelle aus den Ergebnissen (falls fehlend)
          if ~exist(csvfile, 'file')
            try
              cds_results_table(Set, Traj, Structures);
            catch
              warning(['Ergebnis-Tabelle konnte nicht erstellt werden. ', ...
                'Vermutlich Daten mit alter Version erzeugt.']);
            end
          end
          % Stelle fest, ob das Ergebnis vollständig ist
          offline_result_complete = false;
          settingsfile=fullfile(Set.optimization.resdir, Set.optimization.optname, ...
            sprintf('%s_settings.mat', Set.optimization.optname));
          if exist(settingsfile, 'file')
            tmpset = load(settingsfile, 'Structures');
            % Vergleiche die Anzahl der geplant durchgeführten Optimierungen
            % mit den tatsächlich durchgeführten (mit Endergebnis.mat). Wenn
            % identisch, dann vollständiger Durchlauf
            complstr = sprintf('Dabei %d/%d Maßsynthesen durchgeführt. ', ...
              length(Structures), length(tmpset.Structures));
            if length(tmpset.Structures) == length(Structures)
              offline_result_complete = true;
              complstr = [complstr,'Der Durchlauf ist vollständig.']; %#ok<AGROW>
            elseif length(tmpset.Structures) < max(ResData_i.LfdNr)
              complstr = [complstr,'Einstellungsdatei und Ergebnisliste nicht konsistent']; %#ok<AGROW>
            else
              complstr = [complstr,'Der Durchlauf ist nicht vollständig.']; %#ok<AGROW>
            end
          else
            complstr = 'Keine Aussage über Vollständigkeit des Ergebnisses möglich.';
          end
          fprintf(['Ergebnis-Ordner %s zur Offline-Auswertung gewählt. Enthält ', ...
            '%d/%d passende Ergebnisse (%1.0f%% unpassende Ergebnisse) und ist %1.1f ', ...
            'Tage alt. %s\n'], Set.optimization.optname, reslist_nummatch(IIRL(I_reslist)), ...
            length(Whitelist_PKM), 100*reslist_rationomatch(IIRL(I_reslist)), reslist_age(IIRL(I_reslist)), complstr);
          resmaindir = fullfile(Set.optimization.resdir, Set.optimization.optname);
        end
      end
    end
    % Ergebnisse der Struktursynthese (bzw. als solcher durchgeführten
    % Maßsynthese zusammenstellen)
    save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', ...
      sprintf('parroblib_add_robots_symact_%s_3.mat', EE_FG_Name)));
    %% LUIS-Cluster vorbereiten
    if settings.comp_cluster && offline_result_complete && ...
        reslist_age(IIRL(I_reslist)) < settings.clustercomp_if_res_olderthan && ...
        reslist_nummatch(IIRL(I_reslist)) == length(Whitelist_PKM)
      % Prüfe Bedingungen, bei denen nicht auf dem Cluster gerechnet werden
      % soll, weil die Ergebnisse lokal schon vorliegen. Kann gemacht
      % werden, wenn die Rechnung auf dem Cluster für manche G-/P-Nummern
      % schon erfolgreich war.
      fprintf(['Das bereits vorhandene Ergebnis ist vollständig und aktuell. ', ...
        'Keine Neuberechnung auf dem Cluster.\n']);
    elseif settings.comp_cluster
      fprintf(['Vorhandener Ergebnis-Ordner ist nicht passend. Starte ', ...
        'Struktursynthese auf Cluster\n']);
      % Eindeutige Bezeichnung für diesen Versuchslauf
      computation_name = sprintf('structsynth_par_%s_G%dP%d_%s_%s%s', EE_FG_Name, ...
        Coupling(1), Coupling(2),  datestr(now,'yyyymmdd_HHMMSS'), genstr, varstr);
      %% Kompiliere zuerst die Funktionen der geforderten Beinketten
      % Sonst teilweise Zugriffskonflikte. Nur die kompilieren, die nocht fehlen.
      % Berücksichtige andere Iterationen der Struktursynthese aus diesem
      % Durchlauf.
      % Zuerst: Ketten bestimmen, die noch nicht kompiliert wurden
      I_missing = true(length(LegChainList_Coupling),1);
      for lll = 1:length(LegChainList_Coupling)
        if any(contains(LegChainListMexUpload, LegChainList_Coupling{lll}))
          I_missing(lll) = false;
        end
      end
      legchain_compilelist = LegChainList_Coupling(I_missing);
      chf_compile = fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
        'dimsynth', 'dimsynth_cluster_header.m');
      if ~exist(chf_compile, 'file')
        error('Datei %s muss aus Vorlage erzeugt werden', chf_compile);
      end
      if ~isempty(legchain_compilelist) && settings.compile_job_on_cluster
        computation_name_compile = [computation_name, '_compile_serrob'];
        jobdir = tmpDirFcn(true);
        % Speichere die noch nicht kompilierten Beinketten als mat-Datei
        save(fullfile(jobdir, 'compilelist.mat'), 'legchain_compilelist');
        targetfile = fullfile(jobdir, 'compile_serrob.m');
        copyfile(chf_compile, targetfile);
        fid = fopen(targetfile, 'a');
        fprintf(fid, 'tmp=load(''compilelist.mat''); \nlegchain_compilelist=tmp.legchain_compilelist;\n');
        fprintf(fid, 'fprintf(''Starte Funktions-Aktualisierung für %%d serielle Roboter\\n'', length(legchain_compilelist));\n');
        fprintf(fid, 'for i = 1:length(legchain_compilelist)\n');
        fprintf(fid, 'fprintf(''Beginne Funktions-Aktualisierung %%d/%%d für %%s\\n'', i, length(legchain_compilelist), legchain_compilelist{i});\n');
        fprintf(fid, 'serroblib_writelock(''lock'', legchain_compilelist{i}, [], 2*60, 0);\n');
        fprintf(fid, 'serroblib_update_template_functions(legchain_compilelist(i));\n');
        fprintf(fid, 'serroblib_writelock(''free'', legchain_compilelist{i}, [], 0, 0);\n');
        fprintf(fid, 'end\n');
        fclose(fid);
        serrob_compile_jobId = [serrob_compile_jobId, jobStart(struct( ...
          'name', computation_name_compile, ...
          ... % Nur so viele Nodes beantragen, wie auch benötigt werden ("ppn")
          'ppn', 1, ... % Es gibt Dateizugriffsprobleme auf dem Cluster ("Datei nicht gefunden"). Daher nicht parallel kompilieren, auch wenn es lange dauert.
          'matFileName', 'compile_serrob.m', ...
          'locUploadFolder', jobdir, ...
          'time',12), startsettings)]; %#ok<AGROW> 
        LegChainListMexUpload = unique([LegChainListMexUpload, LegChainList_Coupling]);
        % Folgenden Struktursynthese-Job erst nach Kompilierung starten
        startsettings.afterok = serrob_compile_jobId;
        startsettings.afternotok = [];
        startsettings.afterany = [];
      end
      %% Kompiliere die Funktionen für die bereits existierenden PKM vorher.
      % Dadurch muss weniger auf den Parallel-Instanzen neu kompiliert und
      % dann gelöscht werden. Es erfolgt zunächst keine Prüfung, ob PKM
      % doppelt kompiliert werden (da die Funktionen unabhängig von der
      % GP-Nummer sind.
      if settings.compile_job_on_cluster
        pkm_list_noGP = Whitelist_PKM;
        for kkk = 1:length(pkm_list_noGP)
          [~, ~, ~, ~, ~, ~, ~, ~, pkm_list_noGP{kkk}, ~] = parroblib_load_robot(Whitelist_PKM{kkk}, 0);
        end
        [~, III_PKM] = unique(pkm_list_noGP);
        pkm_compilelist = Whitelist_PKM(III_PKM);
        pkm_compilelist = pkm_compilelist(randperm(numel(pkm_compilelist)));
        computation_name_compile = [computation_name, '_compile_parrob'];
        jobdir = tmpDirFcn(true);
        save(fullfile(jobdir, 'compilelist.mat'), 'pkm_compilelist', 'EE_FG');
        targetfile = fullfile(jobdir, 'compile_parrob.m');
        copyfile(chf_compile, targetfile);
        fid = fopen(targetfile, 'a');
        fprintf(fid, 'tmp=load(''compilelist.mat''); \npkm_compilelist=tmp.pkm_compilelist;EE_FG=logical(tmp.EE_FG);\n');
        fprintf(fid, 'repopath=fileparts(which(''parroblib_path_init.m''));\n');
        fprintf(fid, 'EEstr = sprintf(''%%dT%%dR'', sum(EE_FG(1:3)), sum(EE_FG(4:6)));\n');
        fprintf(fid, 'acttabfile=fullfile(repopath, [''sym_'', EEstr], [''sym_'',EEstr,''_list_act.mat'']);\n');
        fprintf(fid, 'tmp = load(acttabfile);\n');
        fprintf(fid, 'ActTab = tmp.ActTab;\n');
        fprintf(fid, 'parpool_writelock(''lock'', 180, true);\n');
        fprintf(fid, 'Pool = gcp(''nocreate'');\n');
        fprintf(fid, 'parpool_writelock(''free'', 0, true);\n');
        fprintf(fid, 'fprintf(''Starte Funktions-Aktualisierung für %%d PKM\\n'', length(pkm_compilelist));\n');
        fprintf(fid, 'parfor i = 1:length(pkm_compilelist)\n');
        fprintf(fid, 'fprintf(''Beginne Funktions-Aktualisierung %%d/%%d für %%s\\n'', i, length(pkm_compilelist), pkm_compilelist{i});\n');
        fprintf(fid, 'if ~any(contains(ActTab.Name, pkm_compilelist{i})), fprintf(''Nicht in Datenbank\\n''); continue; end\n');
        fprintf(fid, 'parroblib_writelock(''lock'', pkm_compilelist{i}, EE_FG, 2*60, 0);\n');
        fprintf(fid, 'RP=parroblib_create_robot_class(pkm_compilelist{i},0,0);\n');
        fprintf(fid, 'RP.fill_fcn_handles(true, true);\n');
        fprintf(fid, 'parroblib_update_template_functions(pkm_compilelist(i));\n');
        fprintf(fid, 'parroblib_writelock(''free'', pkm_compilelist{i}, EE_FG, 0, 0);\n');
        fprintf(fid, 'end\n');
        fclose(fid);
        parrob_compile_jobId = [parrob_compile_jobId, jobStart(struct( ...
          'name', computation_name_compile, ...
          'ppn', 12, ... % Paralleles Kompilieren ausprobieren
          'matFileName', 'compile_parrob.m', ...
          'locUploadFolder', jobdir, ...
          'time',24), startsettings)]; %#ok<AGROW> 
        % Wenn zentraler Kompilier-Job abbricht wird stattdessen wieder
        % dezentral kompiliert. Mögliche Ursache ist, dass es zu lange dauert.
        startsettings.afterany = [startsettings.afterany, parrob_compile_jobId];
      end
      %% Führe die Maßsynthese für die Struktursynthese auf dem Cluster durch.
      % Bereite eine Einstellungs-Datei vor
      jobdir = tmpDirFcn(true);
      targetfile = fullfile(jobdir, [computation_name,'.m']);
      settings_cluster = settings;
      % Für jede G-P-Nummer wird ein Cluster-Job erzeugt.
      settings_cluster.base_couplings = Coupling(1);
      settings_cluster.plf_couplings = Coupling(2);
      settings_cluster.comp_cluster = false;
      settings_cluster.offline = false; % sonst versucht die Cluster-Instanz bestehende Ergebnisse zu laden
      settings_cluster.dryrun = false;
      settings_cluster.isoncluster = true;
      settings_cluster.optname = Set.optimization.optname;
      % Struktursynthese auf dem Cluster parallel rechnen
      settings_cluster.parcomp_structsynth = true;
      % Parallele mex-Kompilierung immer auf dem Cluster. Voraussetzung:
      % ParRobLib wird in eigenes Temp-Verzeichnis kopiert. Dann keine
      % Schreibkonflikte mit parallel berechneten G-/P-Nummern
      settings_cluster.parcomp_mexcompile = true;
      save(fullfile(jobdir, [computation_name,'.mat']), 'settings_cluster');
      % Matlab-Skript erzeugen
      chf = fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
        'struktsynth_par', 'structsynth_cluster_header.m');
      if ~exist(chf, 'file')
        error('Datei %s muss aus Vorlage erzeugt werden', chf);
      end
      copyfile(chf, targetfile);
      % Passe Filter für das Kopieren der Datenbank an. Sonst dauert es
      % ewig, wenn die mex-Dateien für alle PKM kopiert werden.
      fid = fopen(fullfile(jobdir, 'parroblib_tar_include.txt'), 'w');
      for f = dir(fullfile(parroblibpath, '*.m'))'
        fprintf(fid, [f.name, newline()]);
      end
      % Listen für alle PKM kopieren. Sonst wird mit gen_bitarrays die mat-
      % Datenbank dafür neu erstellt und es werden Warnungen ausgegeben.
      fprintf(fid, 'sym*/sym_*T*R_list*\n'); % wird rekursiv gesucht
      fprintf(fid, 'sym_%s/*/actuation.csv\n', EE_FG_Name); % Vermeidung von Warnungen beim Laden der DB
      fprintf(fid, 'synthesis_result_lists/*\n');
      for ii = 1:length(Whitelist_Leg) % Ordner für gewählte PKM
        fprintf(fid, 'sym_%s/%s/*\n', EE_FG_Name, Whitelist_Leg{ii});
      end
      fclose(fid);
      fid = fopen(targetfile, 'a');
      fprintf(fid, 'tmp=load(''%s'');\n', [computation_name,'.mat']);
      fprintf(fid, 'settings=tmp.settings_cluster;\n');
      fprintf(fid, 'parroblib_add_robots_symact(settings);\n');
      % Schließen des ParPools auch in Datei hineinschreiben
      fprintf(fid, 'parpool_writelock(''lock'', 300, true);\n');
      fprintf(fid, 'delete(gcp(''nocreate''));\n');
      fprintf(fid, 'parpool_writelock(''free'', 0, true);\n');
      fclose(fid);
      % Matlab-Skript auf Cluster starten.
      % Schätze die Rechenzeit: 30min pro PKM aufgeteilt auf 12 parallele
      % Kerne und 12h Reserve für allgemeine Aufgaben, z.B. Warten. Eher zu 
      % große Einschätzung der Rechenzeit.
      fprintf('Starte die Berechnung der Struktursynthese auf dem Rechencluster: %s\n', computation_name);
      jobid = jobStart(struct('name', computation_name, ...
        ... % Nur so viele Kerne beantragen, wie auch benötigt werden ("ppn")
        'ppn', min(length(Whitelist_PKM),12), ... % 12 Kerne ermöglicht Lauf auf fast allen Cluster-Nodes
        'matFileName', [computation_name, '.m'], ...
        'locUploadFolder', jobdir, ...
        'time', 12+length(Whitelist_PKM)*0.5/min(length(Whitelist_PKM),12)), ... % Zeit in h. Schätze 30min pro PKM im Durchschnitt
        startsettings); % Erst anfangen, wenn Funktionen für serielle Beinketten kompiliert wurden.
      % Starte auch einen Abschluss-Job. Ist notwendig, falls bei Timeout
      % vorzeitig abgebrochen wird. Siehe cds_start.m
      Set.general.computing_cluster = true;
      Set.general.computing_cluster_cores = min(length(Whitelist_PKM),12); % s.o.
      Set.general.only_finish_aborted = true;
      Set.general.cluster_dependjobs.afternotok = jobid;
      pause(2); % Für Sekunden-Zeitstempel im Ordernamen auf Cluster
      cds_start(Set, Traj);
      continue % Nachfolgendes muss nicht gemacht werden
    end % Cluster-Berechnung
    
    %% Nachverarbeitung der Ergebnis-Liste
    if ~exist('IIRL', 'var') || isempty(IIRL)
      num_results = 0; % Es kann kein Ergebnis geladen werden
    else
      % CSV-Tabelle laden (obiges Laden derselben Datei wird nicht bei 
      % jeder Einstellung des Skripts gemacht)
      csvfile = fullfile(resmaindir, [Set.optimization.optname, ...
          '_results_table.csv']); % Muss hier existieren
      ResData = readtable(csvfile, 'HeaderLines', 2);
      ResData_headers = readtable(csvfile, 'ReadVariableNames', true);
      if isempty(ResData)
        fprintf('Keine Ergebnisse vorhanden. Entferne PKM wieder bei Abschluss\n')
        ResData = ResData_headers; % So Übernahme der Überschriften für leere Tabelle.
        ResData = ResData([],:); % Darf keine Zeilen enthalten, sonst unten Fehler
      else
        ResData.Properties.VariableNames = ResData_headers.Properties.VariableNames;
      end
      settingsfile = fullfile(resmaindir, [Set.optimization.optname, ...
          '_settings.mat']);
      num_results = 0;
      if ~exist(settingsfile, 'file')
        warning('Datei %s existiert nicht. Fehler bei Cluster-Berechnung?.', settingsfile);
        tmpset = struct('Structures', {{}}); % Dummy
      else
        tmpset = load(settingsfile);
        % Stelle die Liste der Roboter zusammen. Ist nicht identisch mit Dateiliste,
        % da PKM mehrfach geprüft werden können.
        Structures_Names = cell(1,length(Structures));
        for jjj = 1:length(Structures)
          if isempty(Structures{jjj}) % falls Daten lückenhaft
            Structures_Names{jjj}='missing'; 
            continue; 
          end
          Structures_Names{jjj} = Structures{jjj}.Name;
        end
        Structures_Names = unique(Structures_Names); % Eindeutige Liste der Strukturen erzeugen
        Structures_Names = Structures_Names(~strcmp(Structures_Names, 'missing'));
        % Sortiere die Liste absteigend, damit zuerst hohe Aktuierungs-
        % nummern gelöscht werden. Andernfalls gibt es Logik-Probleme in der DB
        Structures_Names = fliplr(sort(Structures_Names)); %#ok<FLPST>
        num_results = length(Structures_Names);
      end
    end
    if num_results == 0
      % Trage alle vorher ermittelten PKM als Pseudo-Ergebnisliste ein, um
      % sie danach zu löschen
      Structures_Names = fliplr(sort(Whitelist_PKM(:)')); %#ok<FLPST,TRSRT> 
      % Ergebnis-Variable ist damit bedeutungslos
      ResData = {};
    else
      fprintf('Verarbeite die %d Ergebnisse der Struktursynthese (%d PKM)\n', ...
        sum(~isnan(ResData.LfdNr)), num_results);
    end
    parroblib_writelock('lock', 'csv', logical(EE_FG), 30*60, true); % Sperre beim Ändern der csv
    for jjj = 1:length(Structures_Names) % Alle eindeutigen Strukturen durchgehen
      %% Ergebnisse für diese PKM laden
      Name = Structures_Names{jjj};
      % Erneut die Filter-Liste prüfen. Cluster-Ergebnisse können mehr oder
      % teilweise andere PKM enthalten, als hier geprüft werden soll.
      if ~any(strcmp(Whitelist_PKM, Name))
        continue
      end
      % Prüfe ob Struktur in der Ergebnisliste enthalten ist. Jede Struktur
      % kann mehrfach in der Ergebnisliste enthalten sein, wenn
      % verschiedene Fälle für freie Winkelparameter untersucht werden.
      fval_jjj = []; % Zielfunktionswert der Ergebnisse für diese PKM in der Ergebnisliste
      angles_jjj = {}; % gespeicherte Werte für freie alpha- und theta-Parameter (wird variiert)
      parallelity_jjj = [];
      for jj = 1:size(ResData,1)
        if strcmp(ResData.Name{jj},Name)
          if length(tmpset.Structures) < jj
            warning('Struktur-Variable hat die falsche Dimension. Datenfehler?')
            continue
          end
          Structure_jj = tmpset.Structures{ResData.LfdNr(jj)};
          if ~strcmp(Name, Structure_jj.Name)
            warning('Struktur-Variable nicht konsistente Einträge. Datenfehler?')
            continue
          end
          % Ab hier sind die Daten konsistent und können eingetragen werden
          fval_jjj = [fval_jjj, ResData.Fval_Opt(jj)]; %#ok<AGROW>
          if isempty(angles_jjj) % Syntax-Fehler vermeiden bei leerem char als erstem
            angles_jjj = {Structure_jj.angles_values};
          else
            angles_jjj = [angles_jjj, Structure_jj.angles_values]; %#ok<AGROW>
          end
          if ResData.Fval_Opt(jj) < 1e3 % Auswertung der Beingelenk-Parallelität
            % siehe cds_joint_parallelity
            parallelityfile = fullfile(resmaindir, sprintf( ...
              'Rob%d_%s_joint_parallelity.txt', ResData.LfdNr(jj), ResData.Name{jj}));
            if ~exist(parallelityfile, 'file')
              warning('Datei %s existiert nicht', parallelityfile);
            else
              parallelity_jjj = [parallelity_jjj; readmatrix(parallelityfile)]; %#ok<AGROW> 
            end
            parallelityerrorfile = fullfile(resmaindir, sprintf( ...
              'Rob%d_%s_joint_parallelity_error_legs.txt', ResData.LfdNr(jj), ResData.Name{jj}));
            if exist(parallelityerrorfile, 'file') %TODO: Fehlerbehandlung, falls der Fall mal vorkommt
              error('Fehler bei Bestimmung der Beingelenk-Parallelität. Datei %s existiert.', parallelityerrorfile);
            end
          end
        elseif isempty(fval_jjj) && jj == size(ResData,1)
          warning('Ergebnis zu %s nicht in Tabelle gefunden', Name);
          continue; % kann im Offline-Modus passieren, falls unvollständige Ergebnisse geladen werden.
        end
      end % for jj
      if length(fval_jjj) ~= length(angles_jjj)
        save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', ...
          sprintf('parroblib_add_robots_symact_%s_fval_angles_inconsistent.mat', Name)));
        warning('Variablen fval_jjj und angles_jjj sind nicht konsistent');
        continue
      end
      if isempty(fval_jjj)
        if settings.offline
          if num_results > 0 % sonst ist die Warnung nicht sinnvoll
            warning('Keine Ergebnisse zu %s gefunden. Vermutlich unvollständiger Durchlauf geladen', Name);
          end
          % Lösche PKM wieder, falls Aktuierung unbestimmt war
          acttabfile=fullfile(parroblibpath, ['sym_', EE_FG_Name], ['sym_',EE_FG_Name,'_list_act.mat']);
          tmp = load(acttabfile); % siehe parroblib_gen_bitarrays
          ActTab = tmp.ActTab;
          RL_jjj = ActTab.Rankloss_Platform(strcmp(ActTab.Name, Name));
          if isnan(RL_jjj)
            success = parroblib_remove_robot(Name);
            if ~success
              error('Fehler beim Löschen von %s', Name);
            end
            fprintf('PKM %d/%d %s wurde wieder aus der Datenbank gelöscht.\n', ...
              jjj, length(Structures_Names), Name);
          end
          continue
        else
          error('Keine Ergebnisse zu %s gefunden. Darf nicht passieren.', Name);
        end
      end

      %% Daten für freien Winkelparameter bestimmen
      [angles_jjj_u, I] = unique(angles_jjj);
      if length(angles_jjj_u) ~= length(angles_jjj)
        warning(['Min. ein Fall für %s wurde mehrfach überprüft. %d Ergebnisse,', ...
          'aber nur %d eindeutige. Hier stimmt etwas nicht.'], Name, ...
          length(angles_jjj), length(angles_jjj_u));
        II = false(length(angles_jjj),1);
        II(I) = true; % Binär-Indizes der ersten eindeutigen Ergebnisse
        angles_jjj(~II) = '?'; %#ok<SAGROW> % Markiere doppelte, damit die Logik unten noch stimmt
      end
      % Logik zur Reduktion der Fälle: Symbolisches Rechnen.
      % nur Parameter mit gültigem Ergebnis auswählen. Annahme: Keine Unter-
      % scheidung zwischen Rangverlust und voller Rang hier.
      I_valid = fval_jjj < 1e3;
      angles_jjj_valid = angles_jjj(I_valid);
      angles_jjj_vr = structparam_combine(angles_jjj_valid);
      % Zeichenkette erstellen, die in die actuation.csv geschrieben wird.
      % Enthält alle funktionierenden Konfigurationen
      structparamstr = '';
      for iii = 1:length(angles_jjj_vr)
        structparamstr=[structparamstr, angles_jjj_vr{iii}]; %#ok<AGROW>
        if iii < length(angles_jjj_vr), structparamstr=[structparamstr,',']; end %#ok<AGROW>
      end
      if ~isempty(angles_jjj_valid) && length(intersect(angles_jjj_vr,angles_jjj_valid)) ~= length(angles_jjj_valid)
        structparamstr_orig = '';
        for iii = 1:length(angles_jjj_valid)
          structparamstr_orig=[structparamstr_orig, angles_jjj_valid{iii}]; %#ok<AGROW>
          if iii < length(angles_jjj_valid), structparamstr_orig=[structparamstr_orig,',']; end %#ok<AGROW>
        end
        fprintf('Freie Winkelparameter kombiniert: "%s" -> "%s"\n', structparamstr_orig, structparamstr);
      end
      %% Parallelität der Gelenke prüfen
      if size(parallelity_jjj,1) > 1
        test_parellelilty = repmat(parallelity_jjj(1,:), size(parallelity_jjj,1), 1) - parallelity_jjj;
        if any(test_parellelilty(:))
          % Prüfe, ob der zusammengefasste freie Winkel einem der
          % vorherigen Fälle entsprechen und wähle die parallelen Gelenk
          % von diesem Fall aus
          I = strcmp(structparamstr, angles_jjj_valid); % Annahme: Passen zusammen
          if sum(I) == 1
            parallelity_jjj = parallelity_jjj(I,:);
          else
            parallelity_jjj = [];
            save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', ...
              sprintf('parroblib_add_robots_symact_%s_parallelity_changes.mat', Name)));
            warning('Parallelität der Gelenkachsen ändert sich je nach Winkel. Fall nicht vorgesehen');
          end
        end
      end
      %% Prüfe auf Koppelgelenk-Isomorphismen
      plf_isomorph_found = 0; % Nummer für Isomorphismus
      if Coupling(2) == 7 && any(I_valid) % nur bei erfolgreicher IK
        % Für Methode 7 gibt es die Möglichkeit von Koppelgelenk-
        % Isomorphismen. Die Gelenkausrichtung kann identisch mit einer
        % bestehenden Ausrichtung sein. Falls ja, wird diese genommen.
        Ergebnisliste = dir(fullfile(resmaindir,['Rob*_',Name,'_Endergebnis.mat']));
        if isempty(Ergebnisliste)
          warning('Für %s liegen keine Ergebnis-Dateien (.mat) vor. Notwendig für P7.', Name);
          continue
        end
        % Suche aus den verschiedenen Dateien die, die zu den variablen
        % Strukturparametern passt (s.o.)
        ii_Erg = 0;
        for ii = 1:length(Ergebnisliste)
          res_ii = load(fullfile(resmaindir, Ergebnisliste(ii).name));
          if ~any(structparamstr == res_ii.RobotOptRes.Structure.angles_values)
            continue;
          end
          ii_Erg = ii; break;
        end
        if ii_Erg == 0 && ~isempty(structparamstr)
          warning(['Für %s liegen keine passenden Strukturparameter in ', ...
            '%d Ergebnissen vor. Gesucht: "%s"'], Name, length(Ergebnisliste), ...
            structparamstr)
          continue
        end
        % Initialisiere den Roboter und prüfe, ob die Plattformgelenk- 
        % Ausrichtung identisch mit anderer Methode ist
        tmpset.Set.general.use_mex = false; % Sonst dauert es zu lange.
        % Zuerst die Vorlagen-Funktionen für die seriellen Beinketten.
        % Dadurch kann die Schreibsperre hier sichergestellt werden.
        % Sonst Schreibkonflikte, da auf die SerRobLib parallel zugegriffen
        % wird
        % Aktualisiere die M-Funktionen. Annahme: Synthese auf Cluster,
        % Auswertung lokal, also Funktionen noch nicht initialisiert.
        parroblib_update_template_functions({Name}, false, true); % ohne mex
        [R_ii, Structure_ii] = cds_dimsynth_robot(tmpset.Set, tmpset.Traj, ...
          tmpset.Structures{res_ii.RobotOptRes.Structure.Number}, true);
        % Eintragen der Variable xref in die Matlab-Klasse
        try
          % Aktualisiere die Parameter, falls eine veraltete
          % Ergebnisversion geladen wurde
          p_val_ii = cds_parameters_update(res_ii.RobotOptRes.Structure, ...
            Structure_ii, res_ii.RobotOptRes.p_val);
          % Eintragen in Roboter-Klasse
          cds_update_robot_parameters(R_ii, tmpset.Set, Structure_ii, p_val_ii);
        catch err
          save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', ...
            sprintf('parroblib_add_robots_symact_%s_update_robot_parameters_fail.mat', Name)));
          warning(sprintf('Fehler beim Aktualisieren der Parameter. Vermutlich veraltetes Ergebnis: %s', err.message)); %#ok<SPWRN> 
          continue
        end
        % Verschiedene Koppelgelenk-Methoden durchgehen.
        for kkP = [7 1:3 8] % zulässige Methoden für PKM, die P7 unterstützen
          p_plf = R_ii.DesPar.platform_par(1);
          if kkP==8, p_plf = [p_plf;0]; end %#ok<AGROW> 
          R_ii.align_platform_coupling(kkP, p_plf);
          phi_P_B_all_Pkk = R_ii.phi_P_B_all;
          z_P_B_Pkk = NaN(3, R_ii.NLEG);
          for iiL = 1:R_ii.NLEG
            z_P_B_Pkk(:,iiL) = eulxyz2r([phi_P_B_all_Pkk(1:2,iiL);0])*[0;0;1];
          end
          if kkP == 7
            z_P_B_P7 = z_P_B_Pkk;
          else
            % Vergleich die Richtung der z-Achse (und nicht nur die Euler-
            % Winkel). Dadurch Finden von antiparallelen Achsen möglich.
            % Für die Struktur ist die Antiparallelität egal (hängt von
            % IK-Konfiguration ab).
            I_parallel = all(abs(z_P_B_P7 - z_P_B_Pkk) < 1e-6);
            I_antipar = all(abs(z_P_B_P7 - -z_P_B_Pkk) < 1e-6);
            if all(I_parallel | I_antipar)
              plf_isomorph_found = kkP;
              break;
            end
          end
        end % for kkP
      end % if P7
      %% Ergebnis der Maßsynthese auswerten
      remove = false;
      try
        [~, LEG_Names_array, Actuation] = parroblib_load_robot(Name);
        if isempty([Actuation{:}])
          fprintf(['%d/%d: Aktuierung der PKM %s wurde nicht in Datenbank gefunden, ', ...
            'obwohl sie hinzugefügt werden sollte. Fehler.\n'], jjj, length(Structures_Names), Name);
          row = {EE_FG_Name, Coupling(1), Coupling(2), Name, 99, 0, true};
          ResTab = [ResTab; row]; %#ok<AGROW> 
          continue
        end
      catch
        fprintf(['%d/%d: PKM %s wurde nicht in Datenbank gefunden, obwohl sie ', ...
          'hinzugefügt werden sollte. Fehler.\n'], jjj, length(Structures_Names), Name);
        row = {EE_FG_Name, Coupling(1), Coupling(2), Name, 98, 0, true};
        ResTab = [ResTab; row]; %#ok<AGROW> 
        continue
      end
      rescode = NaN; %#ok<NASGU> 
      rank_success = 0;
      if plf_isomorph_found > 0
        fprintf(['%d/%d: Plattform-Koppelgelenkausrichtung von %s aus ', ...
          'Methode P7 ist identisch zu Ausrichtung aus P%d. Überspringe ', ...
          'Isomorphismus.\n'], jjj, length(Structures_Names), Name, plf_isomorph_found);
        remove = true;
        num_isomorph = num_isomorph + 1;
        rescode = 8; % Code für Isomorphismus (nicht nur Kugelgelenk-Iso.)
      elseif all(fval_jjj > 50) % Definition der Werte für fval_jjj, siehe cds_constraints_traj, cds_fitness
        fprintf(['%d/%d: Für PKM %s konnte in der Maßsynthese keine funktio', ...
          'nierende Lösung gefunden werden.\n'], jjj, length(Structures_Names), Name);
        if min(fval_jjj) < 1e3
          fprintf('Rangdefizit der Jacobi für Beispiel-Punkte ist %1.0f\n', min(fval_jjj)/100);
          parroblib_change_properties(Name, 'rankloss', sprintf('%1.0f', min(fval_jjj)/100));
          parroblib_change_properties(Name, 'values_angles', structparamstr);
          if size(parallelity_jjj,1) > 0
            parroblib_change_properties(Name, 'joint_parallelity', parallelity_jjj(1,:));
          end
          rescode = 0;
          num_rankloss = num_rankloss + 1;
        elseif min(fval_jjj) == 9.9e10 || ... % siehe cds_fitness
            all(min(fval_jjj) > 9e9) && all(min(fval_jjj) < 1e10)
          remove = true;
          num_dimsynthfail = num_dimsynthfail + 1;
          rescode = 10;
        elseif min(fval_jjj) > 1e10
          fprintf(['Der Rang der Jacobi konnte gar nicht erst geprüft werden. ', ...
            'Zielfunktion (Einzelpunkt-IK) %1.2e\n'], min(fval_jjj));
          remove = true;
          num_dimsynthfail = num_dimsynthfail + 1;
          rescode = 3;
        elseif min(fval_jjj) > 1e9
          fprintf(['Erweiterte Prüfung (Kollision etc.) fehlgeschlagen. Sollte ', ...
            'eigentlich nicht geprüft werden! Zielfkt. %1.2e\n'], min(fval_jjj));
          remove = true;
          num_dimsynthfail = num_dimsynthfail + 1;
          rescode = 7;
        elseif min(fval_jjj) > 1e8
          fprintf(['Der Rang der Jacobi konnte gar nicht erst geprüft werden. ', ...
            'Zielfunktion (Traj.-IK) %1.2e\n'], min(fval_jjj));
          remove = true;
          num_dimsynthfail = num_dimsynthfail + 1;
          rescode = 4;
        elseif min(fval_jjj) >= 9e7
          fprintf(['Der Rang der Jacobi konnte gar nicht erst geprüft werden. ', ...
            'Zielfunktion (Parasitäre Bewegung) %1.2e\n'], min(fval_jjj));
          remove = true;
          num_dimsynthfail = num_dimsynthfail + 1;
          rescode = 5;
        else
          fprintf(['Der Rang der Jacobi konnte gar nicht erst geprüft werden. ', ...
            'Zielfunktion (Nicht behandelte Ausnahme) %1.2e\n'], min(fval_jjj));
          remove = true;
          num_dimsynthfail = num_dimsynthfail + 1;
          rescode = 7;
        end
      else
        fprintf('%d/%d: PKM %s hat laut Maßsynthese vollen Laufgrad\n', jjj, length(Structures_Names), Name);
        parroblib_change_properties(Name, 'rankloss', '0');
        parroblib_change_properties(Name, 'values_angles', structparamstr);
        if size(parallelity_jjj,1) > 0
          parroblib_change_properties(Name, 'joint_parallelity', parallelity_jjj(1,:));
        end
        rescode = 0;
        rank_success = 1;
        num_fullmobility = num_fullmobility + 1;
        % Mat-Datei muss hier bereits aktualisiert werden. Sonst kann die
        % Anzahl der i.O.-Aktuierungen für csv-Tab. nicht gezählt werden.
        parroblib_gen_bitarrays(logical(EE_FG));
      end
      % Prüfe vor dem Aktualisieren/Löschen, ob das erlaubt ist
      update_db_allowed = true;
      if ~settings.resstatus_downgrade_possible
        % Prüfe vorherigen Wert in der CSV-Liste
        I_name = strcmp(table2cell(synthrestable(:,1)), LEG_Names_array{1});
        I_coupl = table2array(synthrestable(:,3))==Coupling(1) & ...
                  table2array(synthrestable(:,4))==Coupling(2);
        i_restab = find(I_name & I_coupl);
        % aktuellen Status feststellen
        if isempty(i_restab) % Tabelle wurde oben geladen und noch nicht aktualisiert (vermutlich lokaler Durchlauf der Synthese)
          Status_restab = 6; % "noch nicht geprüft"
        elseif length(i_restab) > 1
          error('Doppelter Eintrag in CSV-Tabelle für %sG%dP%d', PName, Coupling(1), Coupling(2));
        else
          Status_restab = table2array(synthrestable(i_restab,5));
        end
        if Status_restab == 0 && rescode > 0
          warning('Eintrag %d in CSV-Synthese-Tabelle ist besser als gefundener neuer Eintrag %d. Keine Aktualisierung in Datenbank.', Status_restab, rescode);
          update_db_allowed = false;
        end
      end
      row = {EE_FG_Name, Coupling(1), Coupling(2), Name, rescode, rank_success, remove};
      ResTab = [ResTab; row]; %#ok<AGROW> 
      if update_db_allowed
        parroblib_update_csv(LEG_Names_array{1}, Coupling, logical(EE_FG), rescode, rank_success);
        if remove && ~settings.isoncluster % Auf Cluster würde das Löschen parallele Instanzen stören.
          fprintf('Entferne PKM %s wieder aus der Datenbank (Name wird wieder frei)\n', Name);
          remsuccess = parroblib_remove_robot(Name);
          if ~remsuccess
            error('Löschen der PKM %s nicht erfolgreich', Name);
          end
        end
      end
    end % for jjj (Structues_Names)
    % Aktualisiere die mat-Dateien (neue Information zu Rangverlust/neue PKM)
    parroblib_gen_bitarrays(logical(EE_FG));
    parroblib_writelock('free', 'csv', logical(EE_FG), 0, true);
    fprintf(['Fertig mit %s-PKM-Synthese für Koppelgelenk G%dP%d.\n', ...
      'Zusammenfassung: %d PKM funktionieren, %d nicht steuerbar, bei %d keine ', ...
      'Lösung der Kinematik. %d Isomorphismen. Insgesamt %d PKM geprüft.\n'], ...
      EE_FG_Name, Coupling(1), Coupling(2), num_fullmobility, num_rankloss, ...
      num_dimsynthfail, num_isomorph, length(Structures_Names));
  end % Koppelpunkte (Variable kk)
  % Ergebnis-Tabelle speichern
  if ~isempty(ResTab)
    structgeompath=fileparts(which('structgeomsynth_path_init.m'));
    restabfile = fullfile(structgeompath, 'results_structsynth', ...
      ['struct_par_', EE_FG_Name, '_', datestr(now,'yyyymmdd_HHMMSS'), '.csv']);
    mkdirs(fileparts(restabfile));
    writetable(ResTab, restabfile, 'Delimiter', ';');
    fprintf('Übersicht gespeichert: %s\n', restabfile);
  end
end % EE-FG (Variable iFG)
