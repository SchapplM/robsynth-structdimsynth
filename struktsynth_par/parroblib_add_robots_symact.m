% Füge PKM dauerhaft zur Bibliothek zu.
% Wähle nur Roboter aus, bei denen die IK und die Jacobi stimmen
% (Überprüfung mit modifizierter Optimierung in Maßsynthese)
% Probiere alle möglichen Beinketten aus der SerRobLib aus

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-09
% (C) Institut für Mechatronische Systeme, Universität Hannover

clc
clearvars -except settings

%% Standardeinstellungen für Benutzereingabe
settings_default = struct( ...
  'check_existing', false, ... % Falls true: Prüfe existierende Roboter in Datenbank nochmal
  'check_missing', true, ... % Falls true: Prüfe auch nicht existierende Roboter
  'check_rankdef_existing', true, ... % Falls true: Prüfe existierende Roboter, deren Rang vorher als zu niedrig festgestellt wurde (zusätzlich zu anderen Optionen notwendig)
  'lfdNr_min', 1, ... % Auslassen der ersten "x" kinematischer Strukturen (zum Debuggen)
  ... % Prüfung ausgewählter Beinketten (zum Debuggen):
  'whitelist_SerialKin', {''}, ... % z.B. 'S6RRPRRR14V2', 'S6RRPRRR14V3' 'S6RRRRRR10V3' 'S6PRRRRR6V2'
  ...% Alternative 1: Nur Beinketten mit Kugelgelenk-Ende
  'onlyspherical', false, ...
  ...% Alternative 2: Allgemeine Beinketten
  'onlygeneral', true, ...
  'dryrun', false, ... % Falls true: Nur Anzeige, was gemacht werden würde
  'EE_FG_Nr', 2:3, ... % nur 3T0R, 3T1R
  'max_actuation_idx', 4, ... % Aktuierung bis zum vierten Gelenk-FG zulassen
  'base_couplings', 1:8, ... % nur Methode 1 bis 4; siehe ParRob/align_base_coupling
  'plf_couplings', 1:6 ... % nur Methode 1 bis 3; siehe ParRob/align_platform_coupling
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

% Coupling_all = [[4 3]; [2 2]; [3 1]; [1 2]];% zum testen

%% Initialisierung
EE_FG_ges = [1 1 0 0 0 1; ...
  1 1 1 0 0 0; ...
  1 1 1 0 0 1; ...
  1 1 1 1 1 1];
EE_FG_Mask = [1 1 1 1 1 1]; % Die FG müssen genauso auch vom Roboter erfüllt werden (0 darf nicht auch 1 sein)
serroblibpath=fileparts(which('serroblib_path_init.m'));

%% Alle PKM generieren
fprintf('Beginne Schleife über %d verschiedene EE-FG\n', length(settings.EE_FG_Nr));
for iFG = settings.EE_FG_Nr % Schleife über EE-FG (der PKM)
  EE_FG = EE_FG_ges(iFG,:);
  EE_FG_Name = sprintf( '%dT%dR', sum(EE_FG(1:3)), sum(EE_FG(4:6)) );
  fprintf('Prüfe PKM mit %s Plattform-FG\n', EE_FG_Name);
  %% Serielle Beinketten auswählen
  N_Legs = sum(EE_FG); % Voll-Parallel: So viele Beine wie EE-FG  
  if all(EE_FG == [1 1 1 0 0 0]) || all(EE_FG == [1 1 1 0 0 1])
    % PKM mit reduziertem FG dürfen keine 6FG-Beinketten haben
    % Die Beinketten müssen mindestens so viele FG wie die PKM haben
    % Alles weitere wird weiter unten gefiltert (kinematische Eigenschaften)
    LegDoF_allowed = 5:-1:N_Legs;
  elseif all(EE_FG == [1 1 0 0 0 1]) || all(EE_FG == [1 1 1 1 1 1])
    LegDoF_allowed = N_Legs; % Fall 2T1R und 3T3R
  else
    error('Fall nicht implementiert');
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
    % nur die ersten drei Gelenke dürfen aktuiert sein. Die letzten drei sind
    % das Kugelgelenk
    Actuation_possib = 1:min(3, settings.max_actuation_idx);
  elseif ~settings.onlyspherical && settings.onlygeneral
    % Eigenschaften kombinieren
    I = I_FG & I_novar;
    % Die Aktuierung ist sinnvollerweise nur (relativ) gestellnah
    Actuation_possib = 1:min(4, settings.max_actuation_idx);
  else
    error('Kombination von Filtern nicht vorgesehen');
  end

  II = find(I); % Umwandlung von Binär-Indizes in Nummern
  ii = 0; % Laufende Nummer für aktuierte PKM
  ii_kin = 0; % Laufende Nummer für Kinematik-Struktur der PKM
  % Variablen zum Erzeugen der Statistik
  num_rankloss = 0;
  num_dimsynthabort = 0;
  num_dimsynthfail = 0;
  num_fullmobility = 0;
  num_checked_dimsynth = 0;
  fprintf('Beginne Schleife über %d (prinzipiell) mögliche Beinketten-Kinematiken\n', length(II));
  for iFK = II' % Schleife über serielle Führungsketten
    ii_kin = ii_kin + 1;
    SName = l.Names_Ndof{iFK};
    if ~isempty(settings.whitelist_SerialKin) && ~any(strcmp(settings.whitelist_SerialKin, SName))
      % Aktuelle Roboterstruktur für Beinketten nicht in Positivliste
      continue
    end
    if sum(SName=='P')>1
      % Hat mehr als ein Schubgelenk. Kommt nicht für PKM in Frage.
      % (es muss dann zwangsläufig ein Schubgelenk passiv sein)
      continue
    end
    
    N_LegDoF = str2double(SName(2));% Beinkette FHG
    PName = sprintf('P%d%s', N_Legs, SName(3:end));
    fprintf('Kinematik %d/%d: %s, %s\n', ii_kin, length(II), PName, SName);
    
    % Beinketten-FG aus Datenbank auslesen:
    EE_dof_legchain = dec2bin(l.BitArrays_EEdof0(iFK,:))=='1'; % FG vom Typ [1 1 1 0 0 1 0 0 1]
    
    % Bestimme Möglichkeiten für Koppelpunkte
    [Cpl1_grid,Cpl2_grid] = ndgrid(settings.base_couplings,settings.plf_couplings);
    if any(settings.EE_FG_Nr==[2 3]) % 3T0R oder 3T1R
      Cpl1_grid(Cpl1_grid>4) = []; % nur Methode 1 bis 4 ist sinnvoll
      Cpl2_grid(Cpl2_grid>3) = []; % nur Methode 1 bis 3 ist sinnvoll
    end
    Coupling_all = [Cpl1_grid(:),Cpl2_grid(:)];
    
    for kk = 1:size(Coupling_all,1) % Schleife über Koppelpunkt-Möglichkeiten
      Coupling = Coupling_all(kk,:);
      
      % Plausibilitäts-Prüfungen basierend auf Beinketten und Kopplung
      % Beinketten-FG auf Plausibilität prüfen
      leg_success = parrob_structsynth_check_leg_dof(SName, Coupling, EE_FG, EE_dof_legchain);
      if ~leg_success
        fprintf('Beinkette %s mit Koppelpunkt-Nr. %d-%d wird aufgrund geometrischer Überlegungen verworfen.\n', ...
          SName, Coupling(1), Coupling(2));
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
          end
        else
          error('Dieser Fall darf nicht eintreten. Nicht-logische Eingabe');
        end
      
        % Mit dem dann eindeutigen Robotermodell sind weitere Berechnungen
        % möglich
        fprintf('Starte Prüfung des Laufgrads der PKM mit Maßsynthese für %s\n', Name);
        if settings.dryrun, continue; end
        [~, ~, ~, ~, ~, ~, ~, ~, ~, AdditionalInfo] = parroblib_load_robot(Name);
        %% Maßsynthese für den Roboter durchführen
        num_checked_dimsynth = num_checked_dimsynth + 1;
        save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', ...
          sprintf('parroblib_add_robots_symact_3T3R_spherical_%d_%s.mat', jj, Name)));
        save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', ...
          'parroblib_add_robots_symact_3T3R_spherical_act.mat'));

        % Damit wird geprüft, ob das System sinnvoll ist
        Set = cds_settings_defaults(struct('DoF', EE_FG));
        Set.task.Ts = 1e-2;
        Set.task.Tv = 1e-1;
        Set.task.profile = 0; % Nur Eckpunkte, kein Zeitverlauf mit Geschwindigkeit
        Set.task.maxangle = 5*pi/180; % Reduzierung der Winkel auf 5 Grad (ist für FG-Untersuchung ausreichend)
        Traj = cds_gen_traj(EE_FG, 1, Set.task);
        Set.optimization.objective = 'valid_act';
        Set.optimization.optname = sprintf('add_robots_sym_%s_%d_%s_tmp', EE_FG_Name, ii, Name);
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
        Set.general.verbosity = 3;
        Set.general.matfile_verbosity = 0;
        Set.general.nosummary = false;
        Set.structures.whitelist = {Name}; % nur diese PKM untersuchen
        Set.structures.use_serial = false; % nur PKM (keine seriellen)
        Set.structures.use_parallel_rankdef = 6*settings.check_rankdef_existing;
        Set.general.save_animation_file_extensions = {'gif'};
        cds_start
        resmaindir = fullfile(Set.optimization.resdir, Set.optimization.optname);
        resfile = fullfile(resmaindir, sprintf('Rob%d_%s_Endergebnis.mat', 1, Name));
        load(resfile, 'RobotOptRes');
        %% Ergebnis der Maßsynthese auswerten
        remove = false;
        if isempty(Structures)
          fprintf('PKM %s wurde in der Maßsynthese aufgrund struktureller Eigenschaften nicht in Erwägung gezogen\n', Name);
          remove = true;
          num_dimsynthabort = num_dimsynthabort + 1;
        elseif RobotOptRes.fval > 50
          fprintf('Für PKM %s konnte in der Maßsynthese keine funktionierende Lösung gefunden werden.\n', Name);
          if RobotOptRes.fval < 1e3
            fprintf('Rangdefizit der Jacobi für Beispiel-Punkte ist %1.0f\n', RobotOptRes.fval/100);
            parroblib_change_properties(Name, 'rankloss', sprintf('%1.0f', RobotOptRes.fval/100));
            num_rankloss = num_rankloss + 1;
          else
            fprintf('Der Rang der Jacobi konnte gar nicht erst geprüft werden. Zielfunktion %1.2e\n', RobotOptRes.fval);
            remove = true;
            num_dimsynthfail = num_dimsynthfail + 1;
          end
        else
          fprintf('PKM %s hat laut Maßsynthese volle Mobilität\n', Name);
          parroblib_change_properties(Name, 'rankloss', '0');
          num_fullmobility = num_fullmobility + 1;
        end

        if remove
          fprintf('Entferne PKM %s wieder aus der Datenbank (Name wird wieder frei)\n', Name);
          remsuccess = parroblib_remove_robot(Name);
          if ~remsuccess
            error('Löschen der PKM %s nicht erfolgreich', Name);
          end
        end
      end % Aktuierungen
    end % Koppelpunkte
    fprintf('Fertig mit PKM-Kinematik %s\n', PName);
  end % Führungsketten
end % EE-FG
