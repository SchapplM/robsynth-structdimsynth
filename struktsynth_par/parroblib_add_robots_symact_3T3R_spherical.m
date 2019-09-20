% Füge PKM dauerhaft zur Bibliothek zu (mit Kugelgelenkende-Beinketten).
% Wähle nur Roboter aus, bei denen die IK und die Jacobi stimmen
% (Überprüfung mit modifizierter Optimierung in Maßsynthese)
% Probiere alle möglichen Beinketten aus der SerRobLib aus

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-09
% (C) Institut für Mechatronische Systeme, Universität Hannover

clc
clear

%% Benutzereingabe / Einstellungen
check_existing = true; % Falls true: Prüfe existierende Roboter in Datenbank nochmal
check_missing = false; % Falls true: Prüfe auch nicht existierende Roboter
check_rankdef_existing = true; % Falls true: Prüfe existierende Roboter, deren Rang vorher als zu niedrig festgestellt wurde (zusätzlich zu anderen Optionen notwendig)
% Auslassen der ersten "x" kinematischer Strukturen (zum Debuggen):
set_lfdNr_min = 1;
% Prüfung ausgewählter Beinketten (zum Debuggen):
set_whitelist_SerialKin = {}; % 'S6RRPRRR14V2', 'S6RRPRRR14V3' 'S6RRRRRR10V3'
% Alternative 1: Nur Beinketten mit Kugelgelenk-Ende
set_onlyspherical = false;
% Alternative 2: Allgemeine 6FG-Beinketten
set_onlygeneral = true;
set_dryrun = false; % Falls true: Nur anzeige, was gemacht werden würde

%% Initialisierung
EE_FG = [1 1 1 1 1 1];
EE_FG_Mask = [1 1 1 1 1 1]; % Die FG müssen genauso auch vom Roboter erfüllt werden (0 darf nicht auch 1 sein)
serroblibpath=fileparts(which('serroblib_path_init.m'));
Coupling = [1 1];

%% Alle PKM generieren
EE_FG_Name = sprintf( '%dT%dR', sum(EE_FG(1:3)), sum(EE_FG(4:6)) );
%% Serielle Beinketten auswählen
N_Legs = sum(EE_FG); % Voll-Parallel: So viele Beine wie EE-FG
N_LegDoF = sum(EE_FG); % Beinketten mit so vielen Gelenk-FG wie EE-FG

mdllistfile_Ndof = fullfile(serroblibpath, sprintf('mdl_%ddof', N_LegDoF), sprintf('S%d_list.mat',N_LegDoF));
l = load(mdllistfile_Ndof, 'Names_Ndof', 'AdditionalInfo');
[~,I_FG] = serroblib_filter_robots(N_LegDoF, EE_FG, EE_FG_Mask);
if set_onlyspherical && ~set_onlygeneral
  % Nur Modell-Varianten, keine Hauptmodelle (Hauptmodelle haben sowieso
  % nicht die Eigenschaft der Koppelgelenk-Position):
  I_var = (l.AdditionalInfo(:,2) == 1);
  % Nur die ersten drei Gelenke beeinflussen die Koppelgelenk-Position:
  I_spherical = (l.AdditionalInfo(:,1) == 3); 
  % Alle Eigenschaften kombinieren und Beinketten auswählen
  I = I_FG & I_var & I_spherical;
  % nur die ersten drei Gelenke dürfen aktuiert sein. Die letzten drei sind
  % das Kugelgelenk
  Actuation_possib = 1:3;
elseif ~set_onlyspherical && set_onlygeneral
  % Nur allgemeine Hauptmodelle, keine abgeleiteten Varianten
  I_novar = (l.AdditionalInfo(:,2) == 0);
  % Eigenschaften kombinieren
  I = I_FG & I_novar;
  % Die Aktuierung ist sinnvollerweise nur (relativ) gestellnah
  Actuation_possib = 1:4;
else
  error('Kombination von Filtern nicht vorgesehen');
end
II = find(I); % Umwandlung von Binär-Indizes in Nummern
ii = 0; % Laufende Nummer für aktuierte PKM
ii_kin = 0; % Laufende Nummer für Kinematik-Struktur der PKM
num_rankloss = 0;
num_dimsynthabort = 0;
num_dimsynthfail = 0;
num_fullmobility = 0;
num_checked_dimsynth = 0;
for iFK = II' % Schleife über serielle Führungsketten
  ii_kin = ii_kin + 1;
  SName = l.Names_Ndof{iFK};
  if ~isempty(set_whitelist_SerialKin) && ~any(strcmp(set_whitelist_SerialKin, SName))
    % Aktuelle Roboterstruktur für Beinketten nicht in Positivliste
    continue
  end
  PName = sprintf('P%d%s', N_Legs, SName(3:3+N_LegDoF));
  fprintf('Kinematik %d/%d: %s, %s\n', ii_kin, length(II), PName, SName);

  for jj = Actuation_possib % Prüfe symmetrische Aktuierung aller Beinketten
    ii = ii + 1;
    if ii < set_lfdNr_min, continue; end % Starte erst später
    fprintf('Untersuchte PKM %d (Gestell %d, Plattform %d): %s mit symmetrischer Aktuierung Gelenk %d\n', ...
      ii, Coupling(1), Coupling(2), PName, jj);
    Actuation = cell(1,N_Legs);
    Actuation(:) = {jj};
    LEG_Names = {SName};
    
    % Prüfe, ob PKM in Datenbank ist
    [found_tmp, Name] = parroblib_find_robot(N_Legs, LEG_Names, Actuation, true);
    found = found_tmp(2); % Marker, dass Aktuierung der PKM gespeichert war
    if ~found && ~check_missing
      % nicht in DB. Soll nicht geprüft werden. Weiter
      fprintf('PKM %s ist nicht in Datenbank. Keine weitere Untersuchung.\n', Name);
      continue
    elseif found && check_existing
      % in Datenbank. Soll auch geprüft werden.
      fprintf('PKM %s ist in Datenbank. Führe Untersuchung fort.\n', Name);
    elseif found && ~check_existing
      fprintf('PKM %s ist in Datenbank. Überspringe nochmalige Prüfung.\n', Name);
      continue
    elseif ~found && check_existing
      % Nicht in DB. Füge hinzu
      [Name, new] = parroblib_add_robot(N_Legs, LEG_Names, Actuation, Coupling, EE_FG);
      if new, fprintf('PKM %s zur Datenbank hinzugefügt. Jetzt weitere Untersuchung\n', Name);
      else,   error('PKM erst angeblich nicht in DB enthalten, jetzt aber doch'); end
    else
      error('Dieser Fall darf nicht eintreten. Nicht-logische Eingabe');
    end
    % Mit dem dann eindeutigen Robotermodell sind weitere Berechnungen
    % möglich
    [~, ~, ~, ~, ~, ~, ~, ~, ~, AdditionalInfo] = parroblib_load_robot(Name);
    %% Maßsynthese für den Roboter durchführen
    num_checked_dimsynth = num_checked_dimsynth + 1;
    fprintf('Starte Mobilitätsprüfung mit Maßsynthese für %s\n', Name);
    if set_dryrun, continue; end
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
    Set.general.plot_details_in_fitness = 0;%1e3;
    Set.general.plot_robot_in_fitness = 0;%1e3;
    Set.general.verbosity = 3;
    Set.general.matfile_verbosity = 0;
    Set.general.nosummary = false;
    Set.structures.whitelist = {Name}; % nur diese PKM untersuchen
    Set.structures.use_serial = false; % nur PKM (keine seriellen)
    Set.structures.use_parallel_rankdef = 6*check_rankdef_existing;
    cds_start
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
    end
  end
  fprintf('Fertig mit PKM-Kinematik %s\n', PName);
end
fprintf(['Insgesamt %d PKM mit Maßsynthese geprüft. ', ...
  'Davon %d strukturell aussortiert, %d nicht prüfbar, %d mit Rangverlust und %d mit voller Mobilität.\n'], ...
  num_checked_dimsynth, num_dimsynthabort, num_dimsynthfail, num_rankloss, num_fullmobility);
