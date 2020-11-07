% Erzeuge Listen von Robotern, die für die Optimierung genutzt werden
% 
% Eingabe:
% Set
%   Einstellungen des Optimierungsalgorithmus
% 
% Ausgabe:
% Structures
%   Liste aller Roboterstrukturen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Structures = cds_gen_robot_list(Set)
%% Debug:
if Set.general.matfile_verbosity > 0
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_gen_robot_list1.mat'));
end
% Zum Debuggen
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_gen_robot_list1.mat'));

%% Init
structset = Set.structures;
verblevel = Set.general.verbosity;
serroblibpath=fileparts(which('serroblib_path_init.m'));
% Name der FG für Zugriff auf Listen
if all(structset.DoF == [1 1 0 0 0 1])
  task_str = '2T1R';
elseif all(structset.DoF == [1 1 1 0 0 0])
  task_str = '3T0R';
elseif all(structset.DoF == [1 1 1 0 0 1])
  task_str = '3T1R';
elseif all(structset.DoF == [1 1 1 1 1 0])
  task_str = '3T2R';
elseif all(structset.DoF == [1 1 1 1 1 1])
  task_str = '3T3R';
end

Structures = {};% struct('Name', {}, 'Type', []);

if structset.max_task_redundancy > 0
  error('Aufgabenredundanz noch nicht implementiert');
end
if structset.max_kin_redundancy > 0
  error('Kinematische Redundanz noch nicht implementiert');
end

EE_FG = structset.DoF;
EE_FG_Mask = [1 1 1 1 1 1]; % Die FG müssen genauso auch vom Roboter erfüllt werden (0 darf nicht auch 1 sein)
% if all(structset.DoF == [1 1 1 1 1 0])
%   EE_FG      = [[1 1 1], [1 1 1], [1 1 1]];
%   EE_FG_Mask = [[1 1 1], [1 1 1], [1 1 0]];
% end

ii = 0; % Laufende Nummer für alle Roboterstrukturen (seriell und parallel)

%% Serielle aus Liste Roboter laden
% serroblibpath=fileparts(which('serroblib_path_init.m'));
% mdllistfile = fullfile(serroblibpath, 'lists', sprintf('structsynth_%s.csv',task_str));

% res = xlsread(mdllistfile);
% res = csvread(mdllistfile);

%% Serielle Roboter laden
if structset.use_serial
  N_JointDoF = sum(structset.DoF); % Beinketten ohne irgendeine Redundanz (so viele Gelenke wie EE FG)
  mdllistfile_Ndof = fullfile(serroblibpath, sprintf('mdl_%ddof', N_JointDoF), sprintf('S%d_list.mat',N_JointDoF));
  l = load(mdllistfile_Ndof, 'Names_Ndof', 'AdditionalInfo');
  [~,I_FG] = serroblib_filter_robots(N_JointDoF, EE_FG, EE_FG_Mask);
  I_novar = (l.AdditionalInfo(:,2) == 0);
  I = I_FG;
  % Varianten von Robotern in der Datenbank
  if ~structset.use_kinematic_variants
    I = I & I_novar;
  end
  % Nehme alle Roboter, die explizit genannt wurden
  for i = 1:length(I)
    if any(strcmp(structset.whitelist, l.Names_Ndof{i}))
      I(i) = true; % wieder aktivieren, da explizit gefordert
    end
  end
  II = find(I);
  for j = II'
    SName = l.Names_Ndof{j};
    if ~isempty(structset.whitelist) && ~any(strcmp(structset.whitelist, SName))
      % Es gibt eine Liste von Robotern, dieser ist nicht dabei.
      continue
    end
    
    % Prüfe Anzahl Schubgelenke
    numprismatic = sum(SName == 'P');
    if numprismatic > structset.maxnumprismatic
      if verblevel >= 3, fprintf('%s hat zu viele Schubgelenke (%d>%d). Ignoriere\n', SName, numprismatic, structset.maxnumprismatic); end
      continue
    end
    
    % Prüfe, ob die Gelenkreihenfolge zum Filter passt
    if any(~strcmp(structset.joint_filter, '*'))
      Filter = structset.joint_filter(1:N_JointDoF);
      ChainJoints_filt = SName(3:3+N_JointDoF-1);
      ChainJoints_filt(Filter=='*') = '*';
      if ~strcmp(ChainJoints_filt, structset.joint_filter(1:N_JointDoF))
        if verblevel >= 3, fprintf('%s passt nicht zum Filter %s. Ignoriere\n', SName, structset.joint_filter); end
        continue
      end
    end
    
    ii = ii + 1;
    if verblevel >= 2, fprintf('%d: %s\n', ii, SName); end
    Structures{ii} = struct('Name', SName, 'Type', 0, 'Number', ii); %#ok<AGROW>
  end
end
%% Parallele Roboter aus Liste laden
% parroblibpath=fileparts(which('parroblib_path_init.m'));

%% Parallele Roboter laden
if structset.use_parallel
  % Voll-Parallel: So viele Beinketten wie EE-FG, jede Beinkette einfach aktuiert
  if structset.use_parallel_rankdef
    max_rankdeficit = 6;
  else
    max_rankdeficit = 0;
  end
  [~, PNames_Akt] = parroblib_filter_robots(sum(EE_FG), EE_FG, EE_FG_Mask, max_rankdeficit);
  for j = 1:length(PNames_Akt)
    if ~isempty(structset.whitelist) && ~any(strcmp(structset.whitelist, PNames_Akt{j}))
      % Es gibt eine Liste von Robotern, dieser ist nicht dabei.
      continue
    end
    
    % Lade Detailierte Informationen des Robotermodells
    [~, LEG_Names, Actuation, Coupling, ~, ~, ~, ~, ~, AdditionalInfo_Akt, ...
      StructuralDHParam] = parroblib_load_robot(PNames_Akt{j});
    % Prüfe Koppelpunkt-Eigenschaften
    if ~any(Coupling(1) == [1:9]) || ~any(Coupling(2) == [1:6, 8])
      if verblevel >= 3, fprintf('%s hat eine nicht implementierte Koppelpunkt-Variante\n', PNames_Akt{j}); end
      continue % Robotermodell kann in Optimierung nicht generiert werden.
    end
    if ~any(Coupling(1) == Set.structures.parrob_basejointfilter)
      if verblevel >= 3
        fprintf( '%s hat nicht die gewünschte Gestell-Koppelgelenk-Variante (%s). Ignoriere.\n', ...
          PNames_Akt{j}, disp_array(Set.structures.parrob_basejointfilter,'%d') );
      end
      continue
    end
    % Prüfe, ob Rang ordnungsgemäß in Datenbank steht. Wenn nicht, ist das
    % ein Zeichen dafür, dass die PKM noch ungeprüft ist. Bei Zielfunktion
    % "valid_act" soll in Struktursynthese der Rang geprüft werden. Dann
    % damit weitermachen.
    if isnan(AdditionalInfo_Akt(1)) && ~strcmp(Set.optimization.objective, 'valid_act')
      if verblevel >= 3
        fprintf(['%s hat keine Angabe eines Rangverlusts der Jacobi-Matrix. ', ...
          'Vermutlich ungeprüfte PKM. Ignoriere.\n'], PNames_Akt{j});
      end
      continue
    end
    PassPrisJoint = false;
    TooManyPrisJoints = false;
    LastJointActive = false;
    DistalJointActive = false;
    FilterMatch = true;
    WrongLegChainOrigin = false;
    IsInWhiteList = any(strcmp(structset.whitelist, PNames_Akt{j}));
    for k = 1 % Betrachte erste der symmetrischen Beinketten (für den Fall asymmetrischer PKM auch alle möglich)
      LegChainName = LEG_Names{k};
      NLegDoF = str2double(LegChainName(2));
      ChainJoints = LegChainName(3:3+NLegDoF-1); % enthält nur noch "R" und "P"
      % Prüfe, ob passive Schubgelenke vorliegen
      for l = 1:NLegDoF % Gehe alle Beingelenke
        if strcmp(LegChainName(2+l), 'P') && ~any(Actuation{k} == l)
          PassPrisJoint = true;
        end
      end
      % Prüfe die Anzahl der Schubgelenke
      numprismatic = sum(ChainJoints == 'P');
      if numprismatic > structset.maxnumprismatic
        TooManyPrisJoints = true;
      end
      % Prüfe, ob letztes Gelenk aktiv ist
      if any(Actuation{k} == NLegDoF)
        LastJointActive = true;
      end
      % Prüfe, ob das aktive Gelenk zu nah an der Plattform ist
      if any(Actuation{k} > Set.structures.max_index_active)
        DistalJointActive = true;
      end
      % Prüfe, ob die Gelenkreihenfolge zum Filter passt
      if any(~strcmp(structset.joint_filter, '*'))
        Filter_k = structset.joint_filter(1:NLegDoF);
        ChainJoints_filt = ChainJoints;
        ChainJoints_filt(Filter_k=='*') = '*';
        if ~strcmp(ChainJoints_filt, structset.joint_filter(1:NLegDoF))
          FilterMatch = false;
        end
      end
      % Prüfe, ob die Beinkette nur manuell in die Seriellkinematik-Daten-
      % bank eingetragen wurde und das nicht erwünscht ist
      if structset.onlylegchain_from_synthesis
        mdllistfile_Ndof = fullfile(serroblibpath, sprintf('mdl_%ddof', NLegDoF), sprintf('S%d_list.mat',NLegDoF));
        l = load(mdllistfile_Ndof, 'Names_Ndof', 'BitArrays_Origin', 'AdditionalInfo', 'BitArrays_Ndof');
        % Beinkette in Daten finden
        ilc = find(strcmp(l.Names_Ndof, LegChainName));
        if isempty(ilc) || length(ilc)>1, error('Unerwarteter Eintrag in Datenbank für Beinkette %s', LegChainName); end
        % Erzeuge eine Bit-Maske zur Prüfung, ob die Kinematik aus der
        % Struktursynthese für xTyR-Beinketten kommt
        % Modellherkunft laut Datenbank: dec2bin(l.BitArrays_Origin(ilc,:))
        if all(structset.DoF == [1 1 1 0 0 0])
          Mask_Origin = uint16(bin2dec('00100')); % Dritte Spalte für Modellherkunft
        elseif all(structset.DoF == [1 1 1 0 0 1])
          Mask_Origin = uint16(bin2dec('00010')); % Vierte Spalte (in S5RPRPR.csv o.ä.)
        else
          % Keine Einschränkung (außer manuell eingefügte Ketten)
          Mask_Origin = uint16(bin2dec('01111'));
        end
        % Maske für die Beinkette erstellen. Bei allgemeinen Hauptmodellen
        % direkt ablesen. Bei Varianten die Maske des Hauptmodells nehmen.
        if bitand(Mask_Origin, l.BitArrays_Origin(ilc,:)) ~= 0
          % Beinkette kommt selbst schon aus der richtigen Synthese. Nichts
          % tun. Hierdurch werden auch Varianten aus Beinketten-
          % Struktursynthese genommen (noch ungeklärt, woher die kommen).
        elseif l.AdditionalInfo(ilc,2) == 1 % ist Variante
           % Beinkette ist so direkt nicht richtig
          if ~bitand(uint16(bin2dec('10000')), l.BitArrays_Origin(ilc))
             % Variante soll zumindest aus Generierung der mehrwertigen
             % Gelenke kommen. Keine manuell eingefügten Varianten oder
             % Varianten aus direkter Struktursynthese.
             WrongLegChainOrigin = true;
             break;
          end
          ilc_genmdl = l.AdditionalInfo(ilc,3); % Hauptmodell zu der Variante
          if bitand(Mask_Origin, l.BitArrays_Origin(ilc_genmdl,:)) == 0
            WrongLegChainOrigin = true; % Serielle Kette hat falsche Modellherkunft.
            break;
          end
        else
          % Beinkette ist keine Variante. Obige Prüfung ist fehlgeschlagen.
          % Daher Modellherkunft nicht aus passender Struktursynthese.
          WrongLegChainOrigin = true;
          break;
        end
      end
    end
    SkipRobot = false;
    if structset.nopassiveprismatic && PassPrisJoint % PKM enthält passive Schubgelenke. Nicht auswählen
      if verblevel >= 3, fprintf('%s hat passives Schubgelenk.', PNames_Akt{j}); end
      SkipRobot = true;
    end
    if ~SkipRobot && TooManyPrisJoints
      if verblevel >= 3, fprintf('%s hat zu viele Schubgelenke.', PNames_Akt{j}); end
      SkipRobot = true;
    end
    if ~SkipRobot && structset.activenotlastjoint && LastJointActive
      if verblevel >= 3, fprintf('%s hat aktives letztes Gelenk.', PNames_Akt{j}); end
      SkipRobot = true;
    end
    if ~SkipRobot && DistalJointActive
      if verblevel >= 3, fprintf('%s hat aktives Gelenk nach Position %d.', PNames_Akt{j}, Set.structures.max_index_active); end
      SkipRobot = true;
    end
    if ~SkipRobot && ~FilterMatch
      if verblevel >= 3, fprintf('%s passt nicht zum Filter %s.', PNames_Akt{j}, structset.joint_filter); end
      SkipRobot = true;
    end
    if ~SkipRobot && WrongLegChainOrigin
      if verblevel >= 3, fprintf('%s hat keine Beinkette aus %s-PKM-Synthese (%s).', PNames_Akt{j}, task_str, LegChainName); end
      SkipRobot = true;
    end
    if SkipRobot % Einer der Ausschlussgründe oben wurde getroffen.
      if IsInWhiteList % Positiv-Liste wird trotzdem genommen.
        if verblevel >= 3, fprintf(' Füge trotzdem hinzu, da auf Positiv-Liste.\n'); end
      else
        if verblevel >= 3, fprintf(' Ignoriere.\n'); end
        continue
      end
    end
    % Stelle mögliche Werte für den Strukturparameter theta1 zusammen.
    csvline = serroblib_bits2csvline(l.BitArrays_Ndof(ilc,:)); % DH-Parameter der Beinkette aus csv-Datei
    % Die Indizes beziehen sich auf die MDH-Parameter in der CSV-Datei.
    % Sie sind daher schon passend sortiert (erst Gelenkreihenfolge, dann
    % alpha/theta)
    I_param = contains(csvline, 'theta') | contains(csvline, 'alpha');
    params_str = '';
    II_param = find(I_param);
    for kk = 1:length(II_param)
      params_str = [params_str, csvline{II_param(kk)}]; %#ok<AGROW>
      if kk < length(II_param)
        params_str = [params_str, '/']; %#ok<AGROW>
      end
    end
    if strcmp(Set.optimization.objective, 'valid_act') % Prüfe Laufgrad der PKM (sonst ist die Info schon vorhanden)
      if any(I_param) && ... % es gibt (mindestens) einen freien Struktur-Parameter
          (all(structset.DoF(1:5) == [1 1 1 0 0]) || ... % 3T0R/3T1R
           all(structset.DoF(1:6) == [1 1 1 1 1 0])) % 3T2R (vermutlich hier relevant)
        % Siehe parroblib_load_robot
        % Falls theta ein variabler Parameter ist, werden verschiedene An- 
        % nahmen für theta getroffen und alle einzeln geprüft.
        angles_values = {'p', 'o', 'a'};
        for jj = 1:sum(I_param)-1 % Füge für jeden weiteren Parameter alle Kombinationen hinzu
          [a1,a2] = ndgrid(angles_values,{'p', 'o', 'a'});
          angles_values = {};
          for i1 = 1:length(a1(:))
            angles_values = [angles_values; [a1{i1},a2{i1}]]; %#ok<AGROW>
          end
        end
      else % 3T3R: theta muss nicht betrachtet werden
        % Setze den Fall für alpha/theta auf beliebig
        angles_values = {repmat('a', 1, sum(I_param))};
      end
    else % Normaler Fall der Maßsynthese. Lade Information aus Datenbank
      % Siehe parroblib_load_robot
      if isempty(StructuralDHParam) || isempty(StructuralDHParam{1})
        % Leerer Eintrag. Setze alle Werte auf beliebig
        angles_values = {repmat('a', 1, sum(I_param))};
      elseif length(StructuralDHParam) ~= sum(I_param)
        % Ungültiger Eintrag
        error(['In PKM-Datenbank stimmt für %s die Anzahl der freien ', ...
          'Parameter nicht. Parameter: "%s"; Gegeben: "%s" (erster Eintrag).'], PNames_Akt{j},...
          params_str, StructuralDHParam{1});
      else
        % Gültiger Eintrag in ParRobLib
        angles_values = StructuralDHParam;
      end
    end
    if isempty(angles_values) % Fall darf nicht vorkommen
      error('Es würde kein Eintrag für %s hinzugefügt werden.', PNames_Akt{j});
    end
    for avtmp = angles_values(:)' % Gehe alle möglichen Werte für theta durch und trage als eigene PKM ein.
      av = avtmp{1};
      ii = ii + 1;
      if any(I_param) && ~strcmp(av,'')
        theta_logstr = sprintf('Fall %d/%d; %s = %s', find(strcmp(av,angles_values),1,'first'), ...
          length(angles_values), params_str, av);
      else
        theta_logstr = '';
      end
      if verblevel >= 2, fprintf('%d: %s; %s\n', ii, PNames_Akt{j}, theta_logstr); end
      Structures{ii} = struct('Name', PNames_Akt{j}, 'Type', 2, 'Number', ii, ...
        'Coupling', Coupling, 'angles_values', av); %#ok<AGROW>
    end
  end
end
