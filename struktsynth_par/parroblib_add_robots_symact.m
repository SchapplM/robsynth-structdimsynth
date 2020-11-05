% Füge PKM dauerhaft zur Bibliothek zu.
% Wähle nur Roboter aus, bei denen die IK und die Jacobi stimmen
% (Überprüfung mit modifizierter Optimierung in Maßsynthese)
% Probiere alle möglichen Beinketten aus der SerRobLib aus
% 
% TODO: Bei Abbruch des Skripts bleiben PKM mit "?"-Eintrag in der DB
% zurück. Abhilfe: remove_invalid_robots.m.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-09
% (C) Institut für Mechatronische Systeme, Universität Hannover

clc
clearvars -except settings

%% Standardeinstellungen für Benutzereingabe
settings_default = struct( ...
  'check_existing', false, ... % Falls true: Prüfe existierende Roboter in Datenbank nochmal
  'check_missing', true, ... % Falls true: Prüfe auch nicht existierende Roboter
  'check_rankdef_existing', true, ... % Falls true: Prüfe existierende Roboter, deren Rang vorher als zu niedrig festgestellt wurde (zusätzlich zu anderen Optionen notwendig)
  'check_resstatus', 1:6, ... % Filter für PKM, die einen bestimmten Status in synthesis_result_lists/xTyR.csv haben
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
  'comp_cluster', false, ... % Rechne auf PBS-Rechen-Cluster. Parallel-Instanz für G-/P-Kombis
  'only_cluster_if_fulldata_olderthan', 2, ... % Falls in den letzten zwei Tagen bereits ein vollständiger Durchlauf gemacht wurde, dann nicht nochmal auf dem Cluster rechnen.
  'isoncluster', false, ... % Marker um festzustellen, dass gerade auf Cluster parallel gerechnet wird
  'dryrun', false, ... % Falls true: Nur Anzeige, was gemacht werden würde
  'offline', false, ... % Falls true: Keine Optimierung durchführen, stattdessen letztes passendes Ergebnis laden
  ... % ... dieser Modus kann genutzt werden, wenn die Optimierung korrekt durchgeführt wurde, aber die Nachverarbeitung fehlerhaft war
  'EE_FG_Nr', 2:3, ... % nur 3T0R, 3T1R
  'parcomp_structsynth', 1, ... % parfor-Struktursynthese (schneller, aber mehr Speicher notwendig)
  'parcomp_mexcompile', 1, ... % parfor-Mex-Kompilierung (schneller, aber Dateikonflikt möglich)
  'max_actuation_idx', 4, ... % Aktuierung bis zum vierten Gelenk-FG zulassen
  'base_couplings', 1:9, ... % siehe ParRob/align_base_coupling
  'plf_couplings', [1:6 8] ... % siehe ParRob/align_platform_coupling
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
    warning('Feld %s kann nicht übergeben werden', f{1});
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
  if settings.dryrun
    error('Option "dryrun" nicht zusammen mit "comp_cluster" möglich.');
  end
  settings.offline = true; % Es werden offline vorhandene Ergebnisse geprüft.
end
%% Initialisierung
EE_FG_ges = [1 1 0 0 0 1; ...
  1 1 1 0 0 0; ...
  1 1 1 0 0 1; ...
  1 1 1 1 1 0; ...
  1 1 1 1 1 1];
EE_FG_Mask = [1 1 1 1 1 1]; % Die FG müssen genauso auch vom Roboter erfüllt werden (0 darf nicht auch 1 sein)
serroblibpath=fileparts(which('serroblib_path_init.m'));
parroblibpath=fileparts(which('parroblib_path_init.m'));
%% Alle PKM generieren
fprintf('Beginne Schleife über %d verschiedene EE-FG\n', length(settings.EE_FG_Nr));
for iFG = settings.EE_FG_Nr % Schleife über EE-FG (der PKM)
  EE_FG = EE_FG_ges(iFG,:);
  EE_FG_Name = sprintf( '%dT%dR', sum(EE_FG(1:3)), sum(EE_FG(4:6)) );
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
  fprintf('Prüfe PKM mit %s Plattform-FG\n', EE_FG_Name);
  % Bestimme Möglichkeiten für Koppelpunkte
  [Cpl1_grid,Cpl2_grid] = ndgrid(settings.base_couplings,settings.plf_couplings);
  % Binär-Matrix zum Entfernen von Koppelpunkt-Kombinationen
  I1del = false(size(Cpl1_grid)); I2del = I1del;
  if iFG==1 % 2T1R: Nur G1P1 ist sinnvoll.
    I1del(Cpl1_grid>1) = true;
    I2del(Cpl2_grid>1) = true;
  end
  if any(iFG==[2 3]) % 3T0R oder 3T1R: keine paarweise Anordnung
    I1del(Cpl1_grid>4&Cpl1_grid<9) = true; % nur Methode 1 bis 4 oder 9 ist sinnvoll
    I2del(Cpl2_grid>3&Cpl2_grid<8) = true; % nur Methode 1 bis 3 oder 8 ist sinnvoll
  end
  if any(iFG==4) % 3T2R: keine paarweise Anordnung
    I1del(Cpl1_grid>4&Cpl1_grid<9) = true; % nur Methode 1 bis 4 oder 9 ist sinnvoll
    I2del(Cpl2_grid>3&Cpl2_grid<8) = true; % nur Methode 1 bis 3 oder 8 ist sinnvoll
  end
  Cpl1_grid_filt = Cpl1_grid(~I1del&~I2del);
  Cpl2_grid_filt = Cpl2_grid(~I1del&~I2del);
  Coupling_all = [Cpl1_grid_filt(:),Cpl2_grid_filt(:)];
  Coupling_all = unique(Coupling_all,'row');
  for kk = 1:size(Coupling_all,1) % Schleife über Koppelpunkt-Möglichkeiten
    Coupling = Coupling_all(kk,:);

    %% Serielle Beinketten auswählen
    N_Legs = sum(EE_FG); % Voll-Parallel: So viele Beine wie EE-FG
    if all(EE_FG == [1 1 1 0 0 0]) || all(EE_FG == [1 1 1 0 0 1])
      % PKM mit reduziertem FG dürfen keine 6FG-Beinketten haben
      % Die Beinketten müssen mindestens so viele FG wie die PKM haben
      % Alles weitere wird weiter unten gefiltert (kinematische Eigenschaften)
      LegDoF_allowed = 5:-1:N_Legs;
    elseif all(EE_FG == [1 1 0 0 0 1]) || all(EE_FG == [1 1 1 1 1 1]) || all(EE_FG == [1 1 1 1 1 0])
      LegDoF_allowed = N_Legs; % Fall 2T1R und 3T3R
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
    num_checked_dimsynth = 0;
    fprintf('G%dP%d: Beginne Schleife über %d (prinzipiell) mögliche Beinketten-Kinematiken\n', ...
      Coupling(1), Coupling(2), length(II));
    
    Whitelist_PKM = {};
    tlm_iFKloop = tic(); % zur Speicherung des Zeitpunkts der letzten Meldung
    % Sperre csv-Dateien während des Hinzufügens. Dieser Abschnitt ist
    % kritisch für parallel arbeitende weitere Instanzen der Synthese
    parroblib_writelock('lock', 'csv', logical(EE_FG), 30*60, true);
    if settings.isoncluster
      pause(10.0); % Warte nach dem Sperren der csv-Tabellen, damit check-Befehl anderer Instanzen auslaufen kann.
    end
    for iFK = II' % Schleife über serielle Führungsketten
      ii_kin = ii_kin + 1;
      SName = l.Names_Ndof{iFK};
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
          error('Doppelter Eintrag in CSV-Tabelle für %sG%dP%d', PName, Coupling(1), Coupling(2));
        else
          Status_restab = table2array(synthrestable(i_restab,5));
        end
        % Vergleichen von Liste zu prüfender Status-Werte
        if ~any(Status_restab == settings.check_resstatus)
          continue
        end
      end

      if sum(SName=='P')>1
        % Hat mehr als ein Schubgelenk. Kommt nicht für PKM in Frage.
        % (es muss dann zwangsläufig ein Schubgelenk passiv sein)
        parroblib_update_csv({SName}, Coupling, logical(EE_FG), 1, 0);
        continue
      end
      N_LegDoF = str2double(SName(2));% Beinkette FHG
      PName = sprintf('P%d%s', N_Legs, SName(3:end));


      fprintf('Kinematik %d/%d: %s, %s\n', ii_kin, length(II), PName, SName);

      % Beinketten-FG aus Datenbank auslesen:
      EE_dof_legchain = dec2bin(l.BitArrays_EEdof0(iFK,:))=='1'; % FG vom Typ [1 1 1 0 0 1 0 0 1]

      % Plausibilitäts-Prüfungen basierend auf Beinketten und Kopplung
      % Beinketten-FG auf Plausibilität prüfen
      leg_success = parrob_structsynth_check_leg_dof(SName, Coupling, EE_FG, EE_dof_legchain);
      if ~leg_success
        fprintf('Beinkette %s mit Koppelpunkt-Nr. %d-%d wird aufgrund geometrischer Überlegungen verworfen.\n', ...
          SName, Coupling(1), Coupling(2));
        parroblib_update_csv({SName}, Coupling, logical(EE_FG), 2, 0);
        continue
      end
      for jj = Actuation_possib % Schleife über mögliche Aktuierungen
        ii = ii + 1;
        if ii < settings.lfdNr_min, continue; end % Starte erst später
        % Prüfe schon hier auf passive Schubgelenke (weniger Rechenaufwand)
        IdxP = (SName(3:3+N_LegDoF-1)=='P'); % Nummer des Schubgelenks finden
        if any(IdxP) && find(IdxP)~=jj
          continue % Es gibt ein Schubgelenk und es ist nicht das aktuierte Gelenk
        end
        % Prüfe, ob ein Teil eines technischen Gelenks (Kardan, Kugel
        % aktuiert werden würde). Das letzte positionsbeeinflussende Gelenk
        % ist das letzte aktuierte Gelenk. Danach kommt nur noch das
        % Koppelgelenk (Kardan/Kugel)
        if jj > l.AdditionalInfo(iFK,1) % siehe serroblib_gen_bitarrays.
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
            [~, PNames_Akt] = parroblib_filter_robots(sum(EE_FG), EE_FG, EE_FG_Mask, 6);
            Icheck = find(strcmp(PNames_Akt, Name));
          else
            fprintf('Der Roboter %s würde zur Datenbank hinzugefügt werden\n', PName);
            Name = '<Neuer Name>';
            % Setze Status 6 ("noch nicht geprüft").
            parroblib_update_csv({SName}, Coupling, logical(EE_FG), 6, 0);
          end
        else
          error('Dieser Fall darf nicht eintreten. Nicht-logische Eingabe');
        end
        Whitelist_PKM = [Whitelist_PKM;{Name}]; %#ok<AGROW>
      end
    end
    parroblib_writelock('free', 'csv', logical(EE_FG), 0, true);
    if isempty(Whitelist_PKM)
      fprintf('Für FG %s und G%dP%d gibt es keine PKM.\n', EE_FG_Name, Coupling(1), Coupling(2));
      continue
    end
    % Zeichenfolge zum Erstellen von eindeutigen Namen
    if settings.selectvariants, varstr = 'v'; else, varstr = ''; end
    if settings.selectgeneral, genstr = 'g'; else, genstr = ''; end
    
    if settings.dryrun, continue; end
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
    fprintf('Generiere Template-Funktionen für %d Roboter und kompiliere anschließend.\n', length(Whitelist_Kin));
    save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', ...
      sprintf('parroblib_add_robots_symact_%s_1.mat', EE_FG_Name)));
    if ~settings.offline
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
          parroblib_writelock('lock', Whitelist_Kin{i}, logical(EE_FG), 60*60, true);
          RP.fill_fcn_handles(true, true);
          parroblib_writelock('free', Whitelist_Kin{i}, logical(EE_FG), 0, true);
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
    Traj = cds_gen_traj(EE_FG, 1, Set.task);
    Set.optimization.objective = 'valid_act';
    rs = ['a':'z', 'A':'Z', '0':'9'];
    Set.optimization.optname = sprintf('add_robots_sym_%s_G%dP%d_tmp_%s_%s%s_%s', ...
      EE_FG_Name, Coupling(1), Coupling(2), datestr(now,'yyyymmdd_HHMMSS'), ...
      genstr, varstr, rs(randi([1 length(rs)], 5, 1))); % zufällige String anhängen, falls Sekundengleicher Start einer Optimierung
    Set.optimization.NumIndividuals = 200;
    Set.optimization.MaxIter = 50;
    Set.optimization.ee_rotation = false;
    Set.optimization.ee_translation = false;
    Set.optimization.movebase = false;
    Set.optimization.base_size = false;
    Set.optimization.platform_size = false;
    Set.optimization.max_range_active_revolute = 2*pi;
    Set.general.max_retry_bestfitness_reconstruction = 1;
    Set.general.plot_details_in_fitness = 0e3;
    Set.general.plot_robot_in_fitness = 0e3;
    Set.general.noprogressfigure = true;
    Set.general.verbosity = 3;
    Set.general.matfile_verbosity = 0;
    Set.general.nosummary = true; % Keine Bilder erstellen.
    Set.structures.whitelist = Whitelist_PKM; % nur diese PKM untersuchen
    Set.structures.use_serial = false; % nur PKM (keine seriellen)
    Set.structures.use_parallel_rankdef = 6*settings.check_rankdef_existing;
    Set.general.save_animation_file_extensions = {'gif'};
    Set.general.parcomp_struct = settings.parcomp_structsynth;
    Set.general.use_mex = true;
    Set.general.compile_missing_functions = false; % wurde schon weiter oben gemacht.
    if ~settings.offline
      fprintf(['Starte Prüfung des Laufgrads der PKM mit Maßsynthese für ', ...
        '%d Roboter\n'], length(Whitelist_PKM));
      cds_start
    else
      offline_result_complete = false;
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
            'passenden Ordner.'], Set.optimization.optname(1:end-15));
          continue % Beendet Prüfung dieser Koppelgelenk-Kombination
        end
      else
        % Suche den Ergebnis-Ordner mit der größten Übereinstimmung
        reslist_nummatch = zeros(length(reslist),1); % Anzahl der Treffer
        reslist_rationomatch = zeros(length(reslist),1); % Anzahl der unpassenden Ergebnisse im Ordner
        reslist_age = zeros(length(reslist),1); % Alter der durchgeführten Optimierungen in Ergebnisordner
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
          reslist_nummatch(i) = sum(Whitelist_PKM_match); % Anzahl der Treffer
          % Verhältnis der gefundenen PKM: Berücksichtige freie alpha-/theta-
          % Parameter, die zu doppelten Ergebnissen für einen Namen führen.
          reslist_rationomatch(i) = 1 - reslist_nummatch(i)/length(unique(reslist_pkm_names));
          % Alter des Ordners bestimmen (aus bekanntem Namensschema)
          [datestr_match, ~] = regexp(reslist(i).name,'[A-Za-z0-9_]*_tmp_(\d+)_(\d+)', 'tokens','match');
          if isempty(datestr_match)
            warning('Ergebnis-Ordner "%s" passt nicht ins Datums-Namensschema. Überspringe.', reslist(i).name);
            continue
          end
          date_i = datenum([datestr_match{1}{1}, ' ', datestr_match{1}{2}], 'yyyymmdd HHMMSS');
          reslist_age(i) = now() - date_i; % Alter in Tagen
        end
        reslist_rationomatch(isnan(reslist_rationomatch)) = 0;
        % Bestimme das am sinnvollsten auszuwählendste Ergebnis:
        [~, I] = max(reslist_nummatch/length(Whitelist_PKM) ... % Nehme möglichst vollständige Ordner
          - reslist_age*0.05 ... aber ziehe 5% für jeden vergangenen Tag ab, ...
            ... % damit nicht ein sehr altes vollständiges Ergebnis immer genommen wird
          - reslist_rationomatch*0.10); % Bestrafe nicht passende Einträge
        Set.optimization.optname = reslist(I).name;
        fprintf(['Ergebnis-Ordner %s zur Offline-Auswertung gewählt. Enthält ', ...
          '%d/%d passende Ergebnisse (%1.0f%% unpassende Ergebnisse) und ist %1.1f ', ...
          'Tage alt\n'], Set.optimization.optname, reslist_nummatch(I), ...
          length(Whitelist_PKM), 100*reslist_rationomatch(I), reslist_age(I));
        % Erstelle Variablen, die sonst in cds_start entstehen
        roblist = dir(fullfile(Set.optimization.resdir, Set.optimization.optname, ...
          'Rob*_*')); % Die Namen aller Roboter sind in den Ordnernamen enthalten.
        [tokens, ~] = regexp({roblist([roblist.isdir]).name},'Rob(\d+)_([A-Za-z0-9]*)','tokens','match');
        % Nach Nummer der Roboter sortieren (für nachträgliche Erzeugung der
        % Ergebnis-Tabelle zum korrekten Laden der mat-Dateien)
        robnum_ges = zeros(length(tokens),1);
        for i = 1:length(tokens)
          robnum_ges(i) = str2double(tokens{i}{1}{1});
        end
        [~,I_sortres] = sort(robnum_ges);
        % Erstelle Cell-Array mit allen Roboter-Strukturen (Platzhalter-Var.)
        Structures = {}; istr = 0;
        for i = I_sortres(:)'
          istr = istr + 1;
          Structures{istr} = struct('Name', tokens{i}{1}{2}, 'Type', 2); %#ok<SAGROW>
        end
        % Erstelle auch die csv-Tabelle aus den Ergebnissen (falls fehlend)
        if ~exist(fullfile(Set.optimization.resdir, Set.optimization.optname, ...
            sprintf('%s_results_table.csv', Set.optimization.optname)), 'file')
          cds_results_table(Set, Traj, Structures);
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
          if length(tmpset.Structures) == length(Structures)
            offline_result_complete = true;
          end
        elseif reslist_nummatch(I) == length(Whitelist_PKM) && reslist_age(I) < 5
          % Provisorisch: Falls Einstellungs-Datei bei alten Ergebnissen
          % noch nicht existiert. TODO: Option wieder entfernen.
          offline_result_complete = true;
        end
      end
    end
    % Ergebnisse der Struktursynthese (bzw. als solcher durchgeführten
    % Maßsynthese zusammenstellen)
    resmaindir = fullfile(Set.optimization.resdir, Set.optimization.optname);
    Ergebnisliste = dir(fullfile(resmaindir,'*_Endergebnis.mat'));
    save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', ...
      sprintf('parroblib_add_robots_symact_%s_3.mat', EE_FG_Name)));
    %% LUIS-Cluster vorbereiten
    if settings.comp_cluster && offline_result_complete && ...
        reslist_age(I) < settings.only_cluster_if_fulldata_olderthan && ...
        reslist_rationomatch(I) == 0.0 && ...
        reslist_nummatch(I) == length(Whitelist_PKM)
      % Prüfe Bedingungen, bei denen nicht auf dem Cluster gerechnet werden
      % soll, weil die Ergebnisse lokal schon vorliegen. Kann gemacht
      % werden, wenn die Rechnung auf dem Cluster für manche G-/P-Nummern
      % schon erfolgreich war.
      fprintf(['Das bereits vorhandene Ergebnis ist vollständig und aktuell. ', ...
        'Keine Neuberechnung auf dem Cluster.\n']);
    elseif settings.comp_cluster
      % Führe die Maßsynthese für die Struktursynthese auf dem Cluster durch.
      % Bereite eine Einstellungs-Datei vor
      cluster_repo_path = computingcluster_repo_path();
      % Eindeutige Bezeichnung für diesen Versuchslauf
      computation_name = sprintf('structsynth_par_%s_G%dP%d_%s_%s%s', EE_FG_Name, ...
        Coupling(1), Coupling(2),  datestr(now,'yyyymmdd_HHMMSS'), genstr, varstr);
      jobdir = fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
        'struktsynth_par', 'cluster_jobs', computation_name);
      mkdirs(fullfile(jobdir, 'results')); % Unterordner notwendig für Cluster-Dateisynchronisation
      targetfile = fullfile(jobdir, [computation_name,'.m']);
      settings_cluster = settings;
      % Für jede G-P-Nummer wird ein Cluster-Job erzeugt.
      settings_cluster.base_couplings = Coupling(1);
      settings_cluster.plf_couplings = Coupling(2);
      settings_cluster.comp_cluster = false;
      settings_cluster.offline = false; % sonst versucht die Cluster-Instanz bestehende Ergebnisse zu laden
      settings_cluster.dryrun = false;
      settings_cluster.isoncluster = true;
      % Struktursynthese auf dem Cluster parallel rechnen
      settings_cluster.parcomp_structsynth = true;
      % Parallele mex-Kompilierung immer auf dem Cluster. Voraussetzung:
      % ParRobLib wird in eigenes Temp-Verzeichnis kopiert. Dann keine
      % Schreibkonflikte mit parallel berechneten G-/P-Nummern
      settings_cluster.parcomp_mexcompile = true;
      save(fullfile(jobdir, [computation_name,'.mat']), 'settings_cluster');
      % Matlab-Skript erzeugen
      chf = fullfile(clean_absolute_path(fullfile(jobdir,'..','..')), ...
        'structsynth_cluster_header.m');
      if ~exist(chf, 'file')
        error('Datei %s muss aus Vorlage erzeugt werden', chf);
      end
      copyfile(chf, targetfile);
      fid = fopen(targetfile, 'a');
      fprintf(fid, 'tmp=load(''%s'');\n', [computation_name,'.mat']);
      fprintf(fid, 'settings=tmp.settings_cluster;\n');
      fprintf(fid, 'parroblib_add_robots_symact;\n');
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
      addpath(cluster_repo_path);
      jobStart(struct('name', computation_name, ...
                      'matFileName', [computation_name, '.m'], ...
                      'locUploadFolder', jobdir, ...
                      'time', 12+length(Whitelist_PKM)*0.5/12));
      rmpath_genpath(cluster_repo_path, false);
      continue % Nachfolgendes muss nicht gemacht werden
    end
    
    %% Nachverarbeitung der Ergebnis-Liste
    % Stelle die Liste der Roboter zusammen. Ist nicht identisch mit Dateiliste,
    % da PKM mehrfach geprüft werden können.
    Structures_Names = cell(1,length(Structures));
    for jjj = 1:length(Structures)
      Structures_Names{jjj} = Structures{jjj}.Name;
    end
    Structures_Names = unique(Structures_Names); % Eindeutige Liste der Strukturen erzeugen
    % Sortiere die Liste absteigend, damit zuerst hohe Aktuierungs-
    % nummern gelöscht werden. Andernfalls gibt es Logik-Probleme in der DB
    Structures_Names = fliplr(sort(Structures_Names)); %#ok<FLPST>
    fprintf('Verarbeite die %d Ergebnisse der Struktursynthese (%d PKM)\n', ...
      length(Ergebnisliste), length(Structures_Names));
    parroblib_writelock('lock', 'csv', logical(EE_FG), 30*60, true); % Sperre beim Ändern der csv
    for jjj = 1:length(Structures_Names) % Alle eindeutigen Strukturen durchgehen
      %% Ergebnisse für diese PKM laden
      Name = Structures_Names{jjj};
      % Prüfe ob Struktur in der Ergebnisliste enthalten ist. Jede Struktur
      % kann mehrfach in der Ergebnisliste enthalten sein, wenn
      % verschiedene Fälle für freie Winkelparameter untersucht werden.
      fval_jjj = []; % Zielfunktionswert der Ergebnisse für diese PKM in der Ergebnisliste
      angles_jjj = {}; % gespeicherte Werte für freie alpha- und theta-Parameter (wird variiert)
      for jj = 1:length(Ergebnisliste)
        if contains(Ergebnisliste(jj).name,Name)
          % Ergebnisse aus Datei laden
          resfile = fullfile(resmaindir, Ergebnisliste(jj).name);
          tmp = load(resfile, 'RobotOptRes');
          RobotOptRes = tmp.RobotOptRes;
          fval_jjj = [fval_jjj; RobotOptRes.fval]; %#ok<AGROW>
          angles_jjj = [angles_jjj; tmp.RobotOptRes.Structure.angles_values]; %#ok<AGROW>
        elseif isempty(fval_jjj) && jj == length(Ergebnisliste)
          warning('Ergebnisdatei zu %s nicht gefunden', Name);
          continue; % kann im Offline-Modus passieren, falls unvollständige Ergebnisse geladen werden.
        end
      end
      if isempty(fval_jjj)
        if settings.offline
          warning('Keine Ergebnisse zu %s gefunden. Vermutlich unvollständiger Durchlauf geladen', Name);
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
      %% Ergebnis der Maßsynthese auswerten
      remove = false;
      try
        [~, LEG_Names_array, Actuation] = parroblib_load_robot(Name);
        if isempty([Actuation{:}])
          fprintf(['%d/%d: Aktuierung der PKM %s wurde nicht in Datenbank gefunden, ', ...
            'obwohl sie hinzugefügt werden sollte. Fehler.\n'], jjj, length(Structures_Names), Name);
          continue
        end
      catch
        fprintf(['%d/%d: PKM %s wurde nicht in Datenbank gefunden, obwohl sie ', ...
          'hinzugefügt werden sollte. Fehler.\n'], jjj, length(Structures_Names), Name);
        continue
      end
      if all(fval_jjj > 50)
        fprintf(['%d/%d: Für PKM %s konnte in der Maßsynthese keine funktio', ...
          'nierende Lösung gefunden werden.\n'], jjj, length(Structures_Names), Name);
        if min(fval_jjj) < 1e3
          fprintf('Rangdefizit der Jacobi für Beispiel-Punkte ist %1.0f\n', min(fval_jjj)/100);
          parroblib_change_properties(Name, 'rankloss', sprintf('%1.0f', min(fval_jjj)/100));
          parroblib_change_properties(Name, 'values_angles', structparamstr);
          parroblib_update_csv(LEG_Names_array(1), Coupling, logical(EE_FG), 0, 0);
          num_rankloss = num_rankloss + 1;
        elseif min(fval_jjj) > 1e10
          fprintf(['Der Rang der Jacobi konnte gar nicht erst geprüft werden. ', ...
            'Zielfunktion (Einzelpunkt-IK) %1.2e\n'], min(fval_jjj));
          remove = true;
          num_dimsynthfail = num_dimsynthfail + 1;
          parroblib_update_csv(LEG_Names_array(1), Coupling, logical(EE_FG), 3);
        elseif min(fval_jjj) > 1e9
          fprintf(['Erweiterte Prüfung (Kollision etc.) fehlgeschlagen. Sollte ', ...
            'eigentlich nicht geprüft werden! Zielfkt. %1.2e\n'], min(fval_jjj));
          remove = true;
          num_dimsynthfail = num_dimsynthfail + 1;
          parroblib_update_csv(LEG_Names_array(1), Coupling, logical(EE_FG), 7);
        elseif min(fval_jjj) > 1e8
          fprintf(['Der Rang der Jacobi konnte gar nicht erst geprüft werden. ', ...
            'Zielfunktion (Traj.-IK) %1.2e\n'], min(fval_jjj));
          remove = true;
          num_dimsynthfail = num_dimsynthfail + 1;
          parroblib_update_csv(LEG_Names_array(1), Coupling, logical(EE_FG), 4);
        elseif min(fval_jjj) >= 9e7
          fprintf(['Der Rang der Jacobi konnte gar nicht erst geprüft werden. ', ...
            'Zielfunktion (Parasitäre Bewegung) %1.2e\n'], min(fval_jjj));
          remove = true;
          num_dimsynthfail = num_dimsynthfail + 1;
          parroblib_update_csv(LEG_Names_array(1), Coupling, logical(EE_FG), 5);
        else
          fprintf(['Der Rang der Jacobi konnte gar nicht erst geprüft werden. ', ...
            'Zielfunktion (Nicht behandelte Ausnahme) %1.2e\n'], min(fval_jjj));
          remove = true;
          num_dimsynthfail = num_dimsynthfail + 1;
          parroblib_update_csv(LEG_Names_array(1), Coupling, logical(EE_FG), 7);
        end
      else
        fprintf('%d/%d: PKM %s hat laut Maßsynthese vollen Laufgrad\n', jjj, length(Structures_Names), Name);
        parroblib_change_properties(Name, 'rankloss', '0');
        parroblib_change_properties(Name, 'values_angles', structparamstr);
        parroblib_update_csv(LEG_Names_array(1), Coupling, logical(EE_FG), 0, 1);
        num_fullmobility = num_fullmobility + 1;
      end

      if remove && ~settings.isoncluster % Auf Cluster würde das Löschen parallele Instanzen stören.
        fprintf('Entferne PKM %s wieder aus der Datenbank (Name wird wieder frei)\n', Name);
        remsuccess = parroblib_remove_robot(Name);
        if ~remsuccess
          error('Löschen der PKM %s nicht erfolgreich', Name);
        end
      end
    end
    parroblib_writelock('free', 'csv', logical(EE_FG), 0, true);
    fprintf(['Fertig mit %s-PKM-Synthese für Koppelgelenk G%dP%d.\n', ...
      'Zusammenfassung: %d PKM funktionieren, %d nicht steuerbar, bei %d keine ', ...
      'Lösung der Kinematik. Insgesamt %d PKM geprüft.\n'], EE_FG_Name, ...
      Coupling(1), Coupling(2), num_fullmobility, num_rankloss, num_dimsynthfail, ...
      length(Structures_Names));
  end % Koppelpunkte (Variable kk)
end % EE-FG (Variable iFG)
