% Füge Roboter dauerhaft zur Bibliothek zu.
% Wähle nur Roboter aus, bei denen die IK und die Jacobi stimmen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-09
% (C) Institut für Mechatronische Systeme, Universität Hannover

clc
clear

%% Benutzereingabe / Einstellungen
check_existing = false; % Falls true: Prüfe existierende Roboter in Datenbank nochmal
set_lfdNr_min = 1; % Auslassen der ersten "x" kinematischer Strukturen (zum Debuggen)
set_EE_FG_Nr = 2:3;
Basis_schleife = 1:4;
Plat_schleife = 1:3;
[X,Y] = ndgrid(Basis_schleife,Plat_schleife);
Coupling_all = [X(:),Y(:)];
% Coupling_all = [[4 3]; [2 2]; [3 1]; [1 2]];% f�r testen

%% Initialisierung
EE_FG_ges = [1 1 0 0 0 1; ...
  1 1 1 0 0 0; ...
  1 1 1 0 0 1; ...
  1 1 1 1 1 1];
EE_FG_Mask = [1 1 1 1 1 1]; % Die FG müssen genauso auch vom Roboter erfüllt werden (0 darf nicht auch 1 sein)
serroblibpath=fileparts(which('serroblib_path_init.m'));
EE_FG_ges = EE_FG_ges(set_EE_FG_Nr,:); % Reduktion zum Testen

%% Alle PKM generieren
for iFG = 1:size(EE_FG_ges, 1)
  EE_FG = EE_FG_ges(iFG,:);
  EE_FG_Name = sprintf( '%dT%dR', sum(EE_FG(1:3)), sum(EE_FG(4:6)) );
  %% Serielle Beinketten auswählen
  N_Legs = sum(EE_FG); % Voll-Parallel: So viele Beine wie EE-FG
  N_LegDoF = sum(EE_FG); % Beinketten mit so vielen Gelenk-FG wie EE-FG
  
  if N_Legs == 6
    mdllistfile_Ndof = fullfile(serroblibpath, sprintf('mdl_%ddof', N_LegDoF), sprintf('S%d_list.mat',N_LegDoF));
    l = load(mdllistfile_Ndof, 'Names_Ndof', 'AdditionalInfo');
    [~,I_FG] = serroblib_filter_robots(N_LegDoF, EE_FG, EE_FG_Mask);
    I_novar = (l.AdditionalInfo(:,2) == 0); % keine Modell-Varianten, nur Hauptmodelle
    I = I_FG & I_novar;
    II = find(I);
  else
    l = struct('Names_Ndof', [],'AdditionalInfo',[]);
    I_FG = [];
    for i = 5:-1:N_Legs % PKM mit reduziertem FG d�rfen keine 6FG-Beinketten haben
      mdllistfile_Ndof = fullfile(serroblibpath, sprintf('mdl_%ddof', i), sprintf('S%d_list.mat',i));
      l_tmp = load(mdllistfile_Ndof, 'Names_Ndof', 'AdditionalInfo');
      l.Names_Ndof = [l.Names_Ndof,l_tmp.Names_Ndof];
      l.AdditionalInfo = [l.AdditionalInfo;l_tmp.AdditionalInfo];
      [~,I_FG_tmp] = serroblib_filter_robots(i, EE_FG, EE_FG_Mask);
      I_FG = [I_FG;I_FG_tmp];
      I_novar = (l.AdditionalInfo(:,2) == 0); % keine Modell-Varianten, nur Hauptmodelle
      
    end
    I = I_FG & I_novar;
    II = find(I);
  end
  ii = 0;
  ii_kin = 0;
  for iFK = II' % Schleife über serielle Führungsketten
    ii_kin = ii_kin + 1;
    SName = l.Names_Ndof{iFK};
    if sum(SName=='P')>1
      % Hat mehr als ein Schubgelenk. Kommt nicht f�r PKM in Frage.
      continue
    end
    N_LegDoF = str2double(SName(2));% Beinkette FHG
    
%     if ~strcmp(SName, 'S5PRRRR3'), continue; end
    %     if iFK < 209 , continue; end
    PName = sprintf('P%d%s', N_Legs, SName(3:3+N_LegDoF));
    fprintf('Kinematik %d/%d: %s, %s\n', ii_kin, length(II), PName, SName);
    for kk = 1:size(Coupling_all,1)
      Coupling = Coupling_all(kk,:);
      if any(set_EE_FG_Nr==[2 3]) && ... % 3T0R oder 3T1R
          (~any(Coupling(1)==1:4) || any(~Coupling(2)==1:3)) % Paarweise Anordnung ...
        continue % ... ergibt keinen Sinn. Direkt überspringen.
      end
      
      for jj = 1:N_Legs-1 % Prüfe symmetrische Aktuierung aller Beinketten
        ii = ii + 1;
        if ii < set_lfdNr_min, continue; end % Starte erst später
        % Prüfe schon hier auf passive Schubgelenke (weniger Rechenaufwand)
        IdxP = (SName(3:3+N_LegDoF-1)=='P'); % Nummer des Schubgelenks finden
        if ~isempty(IdxP) && find(IdxP)~=jj
          continue % Es gibt ein Schubgelenk und es ist nicht das aktuierte Gelenk
        end
        
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
        %% Beinketten-FG pr�fen
        status = parrob_structsynth_check_leg_dof(Name);
        if ~status
          fprintf('Beinkette FG wegen der Coupling verloren sind.\n');
          fprintf('Entferne PKM %s wieder aus der Datenbank (Name wird wieder frei)\n', Name);
          remsuccess = parroblib_remove_robot(Name);
        end
        
        %% Maßsynthese für den Roboter durchführen
        % Damit wird geprüft, ob das System sinnvoll ist
        Set = cds_settings_defaults(struct('DoF', EE_FG));
        Set.task.Ts = 1e-2;
        Set.task.Tv = 1e-1;
        Set.task.profile = 0; % Nur Eckpunkte, kein Zeitverlauf mit Geschwindigkeit
        Set.task.maxangle = 5*pi/180; % Reduzierung der Winkel auf 5 Grad (ist für FG-Untersuchung ausreichend)
        Traj = cds_gen_traj(EE_FG, 1, Set.task);
        Set.optimization.objective = 'valid_act';
        Set.optimization.optname = sprintf('add_robots_sym_%s_%d_%s_tmp', EE_FG_Name, ii, Name);
        Set.optimization.NumIndividuals = 25;
        Set.optimization.MaxIter = 5;
        Set.optimization.ee_rotation = false;
        Set.optimization.ee_translation = false;
        Set.optimization.movebase = false;
        Set.optimization.base_size = false;
        Set.optimization.platform_size = false;
        Set.general.max_retry_bestfitness_reconstruction = 1;
        Set.general.plot_details_in_fitness = 1e3;
        Set.general.plot_robot_in_fitness = 1e3;
        Set.general.verbosity = 3;
        Set.general.matfile_verbosity = 2;
        Set.structures.whitelist = {Name}; % nur diese PKM untersuchen
        Set.structures.use_serial = false; % nur PKM
        Set.general.save_animation_file_extensions = {'gif'};
        cds_start
        %% Ergebnis der Maßsynthese auswerten
        remove = false;
        if isempty(Structures)
          fprintf('PKM %s wurde in der Maßsynthese aufgrund struktureller Eigenschaften nicht in Erwägung gezogen\n', Name);
          remove = true;
        else
          load(fullfile(Set.optimization.resdir, Set.optimization.optname, ...
            sprintf('Rob%d_%s_Endergebnis.mat', Structures{1,1}.Number, Structures{1,1}.Name)))
          if RobotOptRes.fval > 50
            fprintf('Für PKM %s konnte in der Maßsynthese keine funktionierende Lösung gefunden werden.\n', Name);
            if RobotOptRes.fval < 1e3
              fprintf('Rangdefizit der Jacobi für Beispiel-Punkte ist %1.0f\n', RobotOptRes.fval/100);
              parroblib_change_properties(Name, 'rankloss', sprintf('%1.0f', RobotOptRes.fval/100));
            else
              fprintf('Der Rang der Jacobi konnte gar nicht erst geprüft werden. Zielfunktion %1.2e\n', RobotOptRes.fval);
              remove = true;
            end
          else
            fprintf('PKM %s hat laut Maßsynthese volle Mobilität\n', Name);
            parroblib_change_properties(Name, 'rankloss', '0');
          end
        end
        if remove
          fprintf('Entferne PKM %s wieder aus der Datenbank (Name wird wieder frei)\n', Name);
          remsuccess = parroblib_remove_robot(Name);
        end
      end
    end
  end
end