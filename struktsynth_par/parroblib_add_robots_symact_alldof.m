% Füge Roboter dauerhaft zur Bibliothek zu.
% Wähle nur Roboter aus, bei denen die IK und die Jacobi stimmen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-09
% (C) Institut für Mechatronische Systeme, Universität Hannover

clc
clear

%% Initialisierung
EE_FG_ges = [1 1 0 0 0 1; ...
             1 1 1 0 0 0; ...
             1 1 1 0 0 1; ...
             1 1 1 1 1 1];
EE_FG_Mask = [1 1 1 1 1 1]; % Die FG müssen genauso auch vom Roboter erfüllt werden (0 darf nicht auch 1 sein)
serroblibpath=fileparts(which('serroblib_path_init.m'));
%% Benutzereingabe / Einstellungen
check_existing = true; % Falls true: Prüfe existierende Roboter in Datenbank nochmal
lfdNr_min = 1; % Auslassen der ersten "x" kinematischer Strukturen (zum Debuggen)
% EE_FG_ges = EE_FG_ges(4,:);  % Reduktion zum Testen
Coupling = [1 1];
%% Alle PKM generieren
for iFG = 1:size(EE_FG_ges, 1)
  EE_FG = EE_FG_ges(iFG,:);
  EE_FG_Name = sprintf( '%dT%dR', sum(EE_FG(1:3)), sum(EE_FG(4:6)) );
  %% Serielle Beinketten auswählen
  N_Legs = sum(EE_FG); % Voll-Parallel: So viele Beine wie EE-FG
  N_LegDoF = sum(EE_FG); % Beinketten mit so vielen Gelenk-FG wie EE-FG

  mdllistfile_Ndof = fullfile(serroblibpath, sprintf('mdl_%ddof', N_LegDoF), sprintf('S%d_list.mat',N_LegDoF));
  l = load(mdllistfile_Ndof, 'Names_Ndof', 'AdditionalInfo');
  [~,I_FG] = serroblib_filter_robots(N_LegDoF, EE_FG, EE_FG_Mask);
  I_novar = (l.AdditionalInfo(:,2) == 0); % keine Modell-Varianten, nur Hauptmodelle
  I = I_FG & I_novar;
  II = find(I);
  ii = 0;
  ii_kin = 0;
  for iFK = II' % Schleife über serielle Führungsketten
    ii_kin = ii_kin + 1;
    SName = l.Names_Ndof{iFK};
    PName = sprintf('P%d%s', N_Legs, SName(3:3+N_LegDoF));
    fprintf('Kinematik %d/%d: %s, %s\n', ii_kin, length(II), PName, SName);
    for jj = 1:N_Legs-1 % Prüfe symmetrische Aktuierung aller Beinketten
      ii = ii + 1;
      if ii < lfdNr_min, continue; end % Starte erst später
      fprintf('Untersuchte PKM %d: %s mit symmetrischer Aktuierung Gelenk %d\n', ii, PName, jj);
      Actuation = cell(1,N_Legs);
      Actuation(:) = {jj};
      LEG_Names = {SName};

      %% Roboter pauschal zur Datenbank hinzufügen
      % Mit dem dann eindeutigen Robotermodell sind weitere Berechnungen
      % möglich
      [Name, new] = parroblib_add_robot(N_Legs, LEG_Names, Actuation, Coupling, EE_FG);
      if new, fprintf('PKM %s zur Datenbank hinzugefügt.\n', Name);
      else,   fprintf('PKM %s existierte schon in der Datenbank.\n', Name); end
      
      if ~check_existing && ~new
        fprintf('Gehe davon aus, dass bereits in Datenbank existierender Roboter korrekt ist. Überspringe nochmalige Prüfung.\n');
        continue
      end

      %% Maßsynthese für den Roboter durchführen
      % Damit wird geprüft, ob das System sinnvoll ist
      Set = cds_settings_defaults(struct('DoF', EE_FG));
      Set.task.Ts = 1e-2;
      Set.task.Tv = 1e-1;
      Set.task.maxangle = 5*pi/180; % Reduzierung der Winkel auf 5 Grad (ist für FG-Untersuchung ausreichend)
      Traj = cds_gen_traj(EE_FG, 1, Set.task);
      Set.optimization.objective = 'valid_act';
      Set.optimization.optname = sprintf('add_robots_sym_%s_%d_%s_tmp', EE_FG_Name, ii, Name);
      Set.optimization.NumIndividuals = 25;
      Set.optimization.MaxIter = 50;
      Set.optimization.ee_rotation = false;
      Set.general.max_retry_bestfitness_reconstruction = 1;
      Set.general.plot_details_in_fitness = 1e3;
      Set.general.plot_robot_in_fitness = 1e3;
      Set.general.verbosity = 3;
      Set.general.matfile_verbosity = 2;
      Set.structures.whitelist = {Name}; % nur diese PKM untersuchen
      Set.structures.use_serial = false; % nur PKM
      cds_start
      %% Ergebnis der Maßsynthese auswerten
      fail = false;
      if isempty(Structures)
        fprintf('PKM %s wurde in der Maßsynthese aufgrund struktureller Eigenschaften nicht in Erwägung gezogen\n', Name);
        fail = true;
      elseif RobotOptRes.fval > 50
        fprintf('Für PKM %s konnte in der Maßsynthese keine funktionierende Lösung gefunden werden.\n', Name);
        if RobotOptRes.fval < 1e3
          fprintf('Rangdefizit der Jacobi für Beispiel-Punkte ist %1.0f\n', RobotOptRes.fval/100);
        else
          fprintf('Der Rang der Jacobi konnte gar nicht erst geprüft werden. Zielfunktion %1.2e\n', RobotOptRes.fval);
        end
        fail = true;
      end

      if fail
        fprintf('Entferne PKM %s wieder aus der Datenbank (Name wird wieder frei)\n', Name);
        remsuccess = parroblib_remove_robot(Name);
      end
    end
  end
end