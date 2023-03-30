% Erzeuge Listen von Robotern, die für die Optimierung genutzt werden
% 
% Eingabe:
% Set
%   Einstellungen des Optimierungsalgorithmus
% 
% Ausgabe:
% Structures
%   Liste aller Roboterstrukturen
% Structures_Names
%   Namen der Strukturen als Cell-Array

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [Structures, Structures_Names] = cds_gen_robot_list(Set)
%% Debug:
if Set.general.matfile_verbosity > 0
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_gen_robot_list1.mat'));
end
% Zum Debuggen
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_gen_robot_list1.mat'));

%% Init
structset = Set.structures;
verblevel = Set.general.verbosity;
% Datenbank serieller Roboter (und Beinketten) laden
serroblibpath=fileparts(which('serroblib_path_init.m'));
parroblibpath=fileparts(which('parroblib_path_init.m'));
SerRob_DB_all = load(fullfile(serroblibpath, 'serrob_list.mat'));
Structures = {};% struct('Name', {}, 'Type', []);

if structset.max_kin_redundancy > 0
  error('Kinematische Redundanz noch nicht implementiert');
end

% Die FG in der SerRobLib sind anders kodiert: v_xyz, w_xyz, phiD_xyz
% TODO: Vereinheitlichen mit ParRobLib
if all(Set.task.DoF == [1 1 1 1 1 0])
  EE_FG_ser      = [[1 1 1], [1 1 1], [1 1 1]];
  EE_FG_Mask_ser = [[1 1 1], [1 1 1], [1 1 0]];
else
  % Annahme: Muss planare Rotation sein. Daher Wiederholung der
  % Winkelgeschw.-Terme als Euler-Geschw.-Terme (letzte beide Einträge)
  EE_FG_ser = Set.task.DoF([1:3, 4:6, 4:6]);
  EE_FG_Mask_ser = [1 1 1 1 1 1 1 1 1]; % Die FG müssen genauso auch vom Roboter erfüllt werden (0 darf nicht auch 1 sein)
end
% Auch Fall 2T0*R und 3T0*R für serielle Roboter abdecken
if Set.task.pointing_task
  EE_FG_Mask_ser(6) = 0;
  EE_FG_Mask_ser(9) = 0;
end

ii = 0; % Laufende Nummer für alle Roboterstrukturen (seriell und parallel)

%% Serielle Roboter laden
if structset.use_serial
  % Beinketten ohne irgendeine Redundanz (so viele Gelenke wie EE FG)
  if Set.structures.min_task_redundancy == 0
    N_JointDoF_allowed = sum(Set.task.DoF);
  elseif Set.structures.min_task_redundancy == 1
    if sum(Set.task.DoF) == 6 % TODO: Weitere Fallabfragen.
      error('Aufgabenredundanz nicht möglich bei 3T3R');
    end
    N_JointDoF_allowed = sum(Set.task.DoF)+1;
  else % Set.structures.min_task_redundancy > 1
    error('Fall nicht implementiert');
  end
  % Bei Aufgabenredundanz: Benutzte EE-FG der Aufgabe müssen weniger als
  % die tatsächlich steuerbaren (Gelenk-FG) sein
  if Set.structures.max_task_redundancy > 0
    % Die Aufgaben-FG die egal sind, sind mit Null gekennzeichnet.
    % In der Summe daher nicht wirksam. Redundanzgrad aufaddiert.
    % Beispiel Redundanzgrad 1: 2T0*R -> 3 Gelenke, 3T2R -> 6 Gelenke.
    N_JointDoF_max = sum(Set.task.DoF)+Set.structures.max_task_redundancy;
    N_JointDoF_allowed = N_JointDoF_allowed:1:N_JointDoF_max;
  end
else
  N_JointDoF_allowed = [];
end
% Mögliche Anzahl an Gelenken durchgehen
for N_JointDoF = N_JointDoF_allowed
  % Seriellroboter-Datenbank für diese Gelenk-Anzahl vor-filtern
  I_N_in_all = SerRob_DB_all.N == N_JointDoF;
  l = struct('Names_Ndof', {SerRob_DB_all.Names(I_N_in_all)}, ...
             'AdditionalInfo', SerRob_DB_all.AdditionalInfo(I_N_in_all,:));
  % Filterung auf EE-FG (zusätzlich zur Gelenkzahl)
  [~,I_FG] = serroblib_filter_robots(N_JointDoF, EE_FG_ser, EE_FG_Mask_ser);
  I_novar = (l.AdditionalInfo(:,2) == 0);
  I = I_FG;
  % Varianten von Robotern in der Datenbank
  if ~structset.use_kinematic_variants
    I = I & I_novar;
  end
  % Nehme alle Roboter, die explizit genannt wurden
  if ~isempty(structset.whitelist)
    for i = 1:length(I)
      if any(strcmp(structset.whitelist, l.Names_Ndof{i}))
        I(i) = true; % wieder aktivieren, da explizit gefordert
      end
    end
  end
  II = find(I);
  for j = II'
    SName = l.Names_Ndof{j};
    RName = '';
    IsInWhiteList = any(strcmp(structset.whitelist, SName));
    Idx_Rob = contains(structset.whitelist, [SName,'_']);
    IsRobInWhiteList = any(Idx_Rob);
    if ~isempty(structset.whitelist) && IsRobInWhiteList
      % Der gesuchte Begriff ist ein Roboter in der Datenbank (mit
      % Parametern, nicht nur ein Modell)
      % Trenne den Namen auf
      if sum(Idx_Rob) > 1
        error('Mehr als ein Robotermodell eingegeben. Noch nicht implementiert');
      end
      [tokens_rob, ~] = regexp(structset.whitelist{Idx_Rob},[SName,'_(.*)'],'tokens','match');
      if isempty(tokens_rob)
        warning('Unerwartete Eingabe in Positiv-Liste');
      end
      % Erwarte den Roboternamen als "Modellname", Unterstrich, Endung
      % Siehe SerRobLib
      suffix = tokens_rob{1}{1};
      RName = [SName, '_', suffix];
      % Damit die folgenden Abfragen funktionieren
      IsInWhiteList = IsRobInWhiteList;
    elseif ~isempty(structset.whitelist) && ~IsInWhiteList
      % Es gibt eine Liste von Robotern, dieser ist nicht dabei.
      continue
    end
    TooManyPrisJoints = false;
    FilterMatch = true;
    WrongOrigin = false;
    
    % Prüfe Herkunft der kinematischen Kette. Soll keine PKM-Beinkette sein
    % (mit Kardan-Gelenken etc.), sondern als serieller Roboter eingetragen
    if Set.structures.only_serialrobot_from_synthesis
      % Entspricht Spalten "Herkunft Struktursynthese" in S6RRRRRR.csv usw.
      % Wird bei den Bits von rechts nach links gezählt.
      Mask_Origin = uint16(bin2dec(fliplr('01000'))); % Nehme nur Ursprung Struktursynthese-Seriell
    else
      Mask_Origin = uint16(bin2dec(fliplr('01001'))); % Nehme auch Gelenkfolge-Varianten der Hauptmodelle
    end
    ilc = find(strcmp(SerRob_DB_all.Names, SName)); % Index des Roboters in Datenbank
    if isempty(ilc) || length(ilc)>1, error('Unerwarteter Eintrag in Datenbank für Beinkette %s', LegChainName); end
    if ~Set.structures.only_serialrobot_from_synthesis && ... % Modus: Varianten betrachten
        SerRob_DB_all.AdditionalInfo(ilc,2) == 1 % ist Variante
      % Es werden auch Varianten der Hauptmodelle betrachtet, wenn diese
      % aus der Synthese von Gelenkfolgen kommen (Schneidende Gelenkachsen)
      ilc_genmdl = SerRob_DB_all.AdditionalInfo(ilc,3); % Hauptmodell zu der Variante
    else
      ilc_genmdl = ilc;
    end
    if ~bitand(Mask_Origin, SerRob_DB_all.BitArrays_Origin(ilc_genmdl,:)) ~= 0
      % Es gibt keine Übereinstimmung der Modellherkunft. Wird ignoriert.
      WrongOrigin = true;
    end
    % Prüfe Anzahl Schubgelenke
    numprismatic = sum(SName == 'P');
    if numprismatic > structset.maxnumprismatic
      TooManyPrisJoints = true;
    end

    % Prüfe, ob die Gelenkreihenfolge zum Filter passt
    for i = 1:length(structset.joint_filter)
      FilterMatch = true;
      joint_filter_i = structset.joint_filter{i};
      if any(~strcmp(joint_filter_i, '*'))
        Filter = joint_filter_i(1:N_JointDoF);
        ChainJoints_filt = SName(3:3+N_JointDoF-1);
        ChainJoints_filt(Filter=='*') = '*';
        if ~strcmp(ChainJoints_filt, joint_filter_i(1:N_JointDoF))
          FilterMatch = false;
        end
      end
      if FilterMatch
        break; % Ein Filter wurde erfüllt. Damit in Ordnung.
      end
    end
    
    SkipRobot = false;
    if WrongOrigin
      if verblevel > 3
        fprintf(['%s kommt nicht aus der gewünschten Struktursynthese. ', ...
          'Herkunft-Bits: [%s], Maske: [%s]'], SName, dec2bin( ...
          SerRob_DB_all.BitArrays_Origin(ilc_genmdl,:), 5), dec2bin(Mask_Origin, 5));
      end
      SkipRobot = true;
    end
    if ~SkipRobot && TooManyPrisJoints
      if verblevel > 3
        fprintf('%s hat zu viele Schubgelenke (%d>%d).', ...
          SName, numprismatic, structset.maxnumprismatic);
      end
      SkipRobot = true;
    end
    if ~SkipRobot && ~FilterMatch
      if verblevel > 3
        fprintf('%s passt nicht zum Filter {%s}.', ...
          SName, disp_array(structset.joint_filter, '%s'));
      end
      SkipRobot = true;
    end
    if SkipRobot % Einer der Ausschlussgründe oben wurde getroffen.
      if IsInWhiteList % Positiv-Liste wird trotzdem genommen.
        if verblevel > 3, fprintf(' Füge trotzdem hinzu, da auf Positiv-Liste.\n'); end
      else
        if verblevel > 3, fprintf(' Ignoriere.\n'); end
        continue
      end
    end
    % Bestimme den Typ der Antriebe
    if all(strcmp(SName(3:3+N_JointDoF-1), 'P'))
      acttype_i = 'prismatic';
    elseif all(strcmp(SName(3:3+N_JointDoF-1), 'R'))
      acttype_i = 'revolute';
    else
     acttype_i = 'mixed';
    end
    ii = ii + 1;
    if verblevel >= 2, fprintf('%d: %s\n', ii, SName); end
    Structures{ii} = struct('Name', SName, 'Type', 0, 'Number', ii, ...
      'RobName', RName, ... % Falls ein konkreter Roboter mit Parametern gewählt ist
      'act_type', acttype_i, 'deactivated', false, ...
      ... % Platzhalter, Angleichung an PKM (Erkennung altes Dateiformat)
      'angles_values', []); %#ok<AGROW> 
  end
end

%% Parallele Roboter laden
if structset.use_parallel
  % Voll-Parallel: So viele Beinketten wie EE-FG, jede Beinkette einfach aktuiert
  if structset.use_parallel_rankdef
    max_rankdeficit = 6;
  else
    max_rankdeficit = 0;
  end
  if Set.structures.min_task_redundancy == 0
    EE_FG_allowed = logical(Set.task.DoF);
  elseif Set.structures.min_task_redundancy > 1
    error('Fall nicht implementiert');
  else
    EE_FG_allowed = logical([]);
  end
  if Set.structures.max_task_redundancy > 0
    if all(Set.task.DoF == [1 1 1 1 1 0])
      % Bei 3T2R-Aufgabe sind 3T3R-PKM aufgabenredundant mit Grad 1
      EE_FG_allowed = [EE_FG_allowed; logical([1 1 1 1 1 1])];
    elseif all(Set.task.DoF == [1 1 0 0 0 0]) && Set.task.pointing_task
      % Bei 2T0*R-Aufgabe sind 2T1R-PKM aufgabenredundant mit Grad 1
      EE_FG_allowed = [EE_FG_allowed; logical([1 1 0 0 0 1])];
    elseif all(Set.task.DoF == [1 1 1 0 0 0]) && Set.task.pointing_task
      % Bei 3T0*R-Aufgabe sind 3T1R-PKM aufgabenredundant mit Grad 1
      EE_FG_allowed = [EE_FG_allowed; logical([1 1 1 0 0 1])];
    else
      if verblevel >= 2
        fprintf(['Aufgabenredundanz gefordert aber Aufgaben-FG %dT%dR nicht ', ...
          'dafür vorgesehen\n'], sum(Set.task.DoF(1:3)), sum(Set.task.DoF(4:6)));
      end
    end
  end
else
  EE_FG_allowed = logical([]);
end
for kkk = 1:size(EE_FG_allowed,1)
  EEstr = sprintf('%dT%dR', sum(EE_FG_allowed(kkk,1:3)), sum(EE_FG_allowed(kkk,4:6)));
  acttabfile=fullfile(parroblibpath, ['sym_', EEstr], ['sym_',EEstr,'_list_act.mat']);
  tmp = load(acttabfile);
  ActTab = tmp.ActTab;
  [~, PNames_Akt, AdditionalInfo_Akt] = parroblib_filter_robots(EE_FG_allowed(kkk,:), max_rankdeficit);
  for j = 1:length(PNames_Akt)
    if ~isempty(structset.whitelist) && ~any(strcmp(structset.whitelist, PNames_Akt{j}))
      % Es gibt eine Liste von Robotern, dieser ist nicht dabei.
      continue
    end
    IsInWhiteList = any(strcmp(structset.whitelist, PNames_Akt{j}));
    % Lade vereinfachte Informationen des Robotermodells
    [NLEG, LEG_Names, ~, Coupling] = parroblib_load_robot(PNames_Akt{j}, 0);
    Ij = strcmp(ActTab.Name, PNames_Akt{j});
    Actuation = cell(1, NLEG);
    for kk = 1:NLEG
      ActLeg_kk = ActTab.(sprintf('Act_Leg%d',kk));
      Actuation{kk} = find(ActLeg_kk(Ij,:));
    end
    StructuralDHParam = ActTab.Values_Angle_Parameters(Ij);
    % Prüfe Koppelpunkt-Eigenschaften
    if ~any(Coupling(1) == 1:10) || ~any(Coupling(2) == 1:9)
      if verblevel > 3 || IsInWhiteList
        fprintf('%s hat eine nicht implementierte Koppelpunkt-Variante\n', PNames_Akt{j});
      end
      continue % Robotermodell kann in Optimierung nicht generiert werden.
    end
    if ~any(Coupling(1) == Set.structures.parrob_basejointfilter)
      if verblevel > 3 || IsInWhiteList
        fprintf( '%s hat nicht die gewünschte Gestell-Koppelgelenk-Variante (%s). Ignoriere.\n', ...
          PNames_Akt{j}, disp_array(Set.structures.parrob_basejointfilter,'%d') );
      end
      continue
    end
    if PNames_Akt{j}(3) == 'P' && ~any(Coupling(1) == Set.structures.parrob_basejointfilter_prismatic)
      if verblevel > 3 || IsInWhiteList
        fprintf( '%s hat nicht die gewünschte Gestell-Koppelgelenk-Variante (%s) für ein Schubgelenk. Ignoriere.\n', ...
          PNames_Akt{j}, disp_array(Set.structures.parrob_basejointfilter_prismatic,'%d') );
      end
      continue
    end
    if PNames_Akt{j}(3) == 'R' && ~any(Coupling(1) == Set.structures.parrob_basejointfilter_revolute)
      if verblevel > 3 || IsInWhiteList
        fprintf( '%s hat nicht die gewünschte Gestell-Koppelgelenk-Variante (%s) für ein Drehgelenk. Ignoriere.\n', ...
          PNames_Akt{j}, disp_array(Set.structures.parrob_basejointfilter_revolute,'%d') );
      end
      continue
    end
    if ~any(Coupling(2) == Set.structures.parrob_platformjointfilter)
      if verblevel > 3 || IsInWhiteList
        fprintf( '%s hat nicht die gewünschte Plattform-Koppelgelenk-Variante (%s). Ignoriere.\n', ...
          PNames_Akt{j}, disp_array(Set.structures.parrob_platformjointfilter,'%d') );
      end
      continue
    end
    
    % Prüfe, ob Rang ordnungsgemäß in Datenbank steht. Wenn nicht, ist das
    % ein Zeichen dafür, dass die PKM noch ungeprüft ist. Bei Zielfunktion
    % "valid_act" soll in Struktursynthese der Rang geprüft werden. Dann
    % damit weitermachen.
    if isnan(AdditionalInfo_Akt(j,1)) && ~any(strcmp(Set.optimization.objective, 'valid_act'))
      if verblevel > 3 || IsInWhiteList
        fprintf(['%s hat keine Angabe eines Rangverlusts der Jacobi-Matrix. ', ...
          'Vermutlich ungeprüfte PKM. Ignoriere.\n'], PNames_Akt{j});
      end
      continue
    end
    PassPrisJoint = false;
    TooManyPrisJoints = false;
    LastJointActive = false;
    DistalJointActive = false;
    DistalPrisJointActive = false;
    DistalRevJointActive = false;
    FilterMatch = true;
    NumTechJointsDontMatch = false;
    SphericalJointInChain = false;
    WrongLegChainOrigin = false;
    
    for k = 1 % Betrachte erste der symmetrischen Beinketten (für den Fall asymmetrischer PKM auch alle möglich)
      LegChainName = LEG_Names{k};
      NLegDoF = str2double(LegChainName(2));
      ChainJoints = LegChainName(3:3+NLegDoF-1); % enthält nur noch "R" und "P"
      % Prüfe, ob passive Schubgelenke vorliegen
      for l = 1:NLegDoF % Gehe alle Beingelenke durch
        if strcmp(LegChainName(2+l), 'P') && ~any(Actuation{k} == l)
          PassPrisJoint = true; break;
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
      for l = 1:NLegDoF % Gehe alle Beingelenke durch
        if strcmp(LegChainName(2+l), 'P') && any(Actuation{k} == l) && ...
            l > Set.structures.max_index_active_prismatic
          DistalPrisJointActive = true; break;
        end
        if strcmp(LegChainName(2+l), 'R') && any(Actuation{k} == l) && ...
            l > Set.structures.max_index_active_revolute
          DistalRevJointActive = true; break;
        end
      end
      
      % Prüfe, ob die Gelenkreihenfolge zum Filter passt
      for i = 1:length(structset.joint_filter)
        FilterMatch = true;
        joint_filter_i = structset.joint_filter{i};
        if any(~strcmp(joint_filter_i, '*'))
          Filter_k = joint_filter_i(1:NLegDoF);
          ChainJoints_filt = ChainJoints;
          ChainJoints_filt(Filter_k=='*') = '*';
          if ~strcmp(ChainJoints_filt, joint_filter_i(1:NLegDoF))
            FilterMatch = false;
          end
        end
        if FilterMatch
          break; % Ein Filter wurde erfüllt. Damit in Ordnung.
        end
      end
      % Beinkette in Daten finden
      ilc = find(strcmp(SerRob_DB_all.Names, LegChainName));
      if isempty(ilc) || length(ilc)>1, error('Unerwarteter Eintrag in Datenbank für Beinkette %s', LegChainName); end

      % Finde die Anzahl der technischen Gelenke der Beinkette heraus und
      % filtere danach
      SName_TechJoint = fliplr(regexprep(num2str(SerRob_DB_all.AdditionalInfo(ilc,7)), ...
        {'1','2','3','4','5'}, {'R','P','C','U','S'}));
      if ~any(length(SName_TechJoint) == structset.num_tech_joints)
        NumTechJointsDontMatch = true;
      end
      % Ignoriere PKM-Beinketten, die das Kugelgelenk nicht am Ende haben
      % oder am Anfang (ist in der Mitte nicht sinnvoll konstruierbar)
      if any(SName_TechJoint(2:end-1) == 'S')
        SphericalJointInChain = true;
      end
      % Ignoriere PKM-Beinketten, die bei Schubzylindern einen Hebel vom
      % Zylinder auf die vorherigen und folgenden Gelenke haben (optional)
      UnwantedCylinderLever = false;
      if Set.structures.prismatic_cylinder_no_lever
        [~, csvbits] = serroblib_bits2csvline(SerRob_DB_all.BitArrays_Ndof(ilc,:));
        PS = serroblib_csvindex2paramstruct(NLegDoF, csvbits);
        % Erkenne an welcher Stelle ein Schubgelenk liegt (Annahme:
        % Zylinder, falls nicht am Gestell). Finde zugehörige a-/d-Param.
        % Siehe: cds_dimsynth_robot (Code für prismatic_cylinder_no_lever)
        I_cyl = find(PS.sigma == 1 & [0; ones(NLegDoF-1,1)]);
        for ii_cyl = I_cyl' % kann mehrere Gelenke geben
          if isnan(PS.a(ii_cyl)) || ii_cyl < NLegDoF && ...
              ( isnan(PS.a(ii_cyl+1)) || isnan(PS.d(ii_cyl+1)) )
            UnwantedCylinderLever = true;
            break;
          end
        end
      end
      % Prüfe, ob die Beinkette nur manuell in die Seriellkinematik-Daten-
      % bank eingetragen wurde und das nicht erwünscht ist
      if structset.onlylegchain_from_synthesis
        % Erzeuge eine Bit-Maske zur Prüfung, ob die Kinematik aus der
        % Struktursynthese für xTyR-Beinketten kommt
        % Modellherkunft laut Datenbank: dec2bin(l.BitArrays_Origin(ilc,:))
        % Prüfe nicht die Einstellungsvariable Set.task.DoF, da bei
        % Aufgabenredundanz mehrere EE-FG in Frage kommen.
        if all(EE_FG_allowed(kkk,:) == [1 1 1 0 0 0])
          % Eigentlich nur dritte Spalte für Modellherkunft für 3T0R-PKM
          % Benutze auch Beinketten für 3T1R-PKM. TODO: Seriell-Synthese
          % passt eventuell nicht komplett
          Mask_Origin = uint16(bin2dec(fliplr('00110'))); % siehe oben bei SerRob-Filter
        elseif all(EE_FG_allowed(kkk,:) == [1 1 1 0 0 1])
          % Eigentlich nur vierte Spalte (in S5RPRPR.csv o.ä.) für 3T1R-PKM
          % Benutze auch Beinketten für 3T0R-PKM
          Mask_Origin = uint16(bin2dec(fliplr('00110')));
        else
          % Keine Einschränkung (außer manuell eingefügte Ketten)
          Mask_Origin = uint16(bin2dec(fliplr('01111')));
        end
        % Maske für die Beinkette erstellen. Bei allgemeinen Hauptmodellen
        % direkt ablesen. Bei Varianten die Maske des Hauptmodells nehmen.
        if bitand(Mask_Origin, SerRob_DB_all.BitArrays_Origin(ilc,:)) ~= 0
          % Beinkette kommt selbst schon aus der richtigen Synthese. Nichts
          % tun. Hierdurch werden auch Varianten aus Beinketten-
          % Struktursynthese genommen (noch ungeklärt, woher die kommen).
        elseif SerRob_DB_all.AdditionalInfo(ilc,2) == 1 % ist Variante
           % Beinkette ist so direkt nicht richtig
          if bitand(uint16(bin2dec(fliplr('10000'))), SerRob_DB_all.BitArrays_Origin(ilc))
             % Variante soll zumindest aus Generierung der mehrwertigen
             % Gelenke kommen. Keine manuell eingefügten Varianten oder
             % Varianten aus direkter Struktursynthese.
             WrongLegChainOrigin = true;
             break;
          end
          ilc_genmdl = SerRob_DB_all.AdditionalInfo(ilc,3); % Hauptmodell zu der Variante
          if bitand(Mask_Origin, SerRob_DB_all.BitArrays_Origin(ilc_genmdl,:)) == 0
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
      if verblevel > 3 || IsInWhiteList
        fprintf('%s hat passives Schubgelenk.', PNames_Akt{j});
      end
      SkipRobot = true;
    end
    if ~SkipRobot && TooManyPrisJoints
      if verblevel > 3 || IsInWhiteList
        fprintf('%s hat zu viele Schubgelenke.', PNames_Akt{j});
      end
      SkipRobot = true;
    end
    if ~SkipRobot && structset.activenotlastjoint && LastJointActive
      if verblevel > 3 || IsInWhiteList
        fprintf('%s hat aktives letztes Gelenk.', PNames_Akt{j});
      end
      SkipRobot = true;
    end
    if ~SkipRobot && DistalJointActive
      if verblevel > 3 || IsInWhiteList
        fprintf('%s hat aktives Gelenk nach Position %d.', PNames_Akt{j}, ...
          Set.structures.max_index_active);
      end
      SkipRobot = true;
    end
    if ~SkipRobot && DistalPrisJointActive
      if verblevel > 3 || IsInWhiteList
        fprintf('%s hat aktives Schubgelenk nach Position %d.', PNames_Akt{j}, ...
          Set.structures.max_index_active_prismatic);
      end
      SkipRobot = true;
    end
    if ~SkipRobot && DistalRevJointActive
      if verblevel > 3 || IsInWhiteList
        fprintf('%s hat aktives Drehgelenk nach Position %d.', PNames_Akt{j}, ...
          Set.structures.max_index_active_revolute);
      end
      SkipRobot = true;
    end
    if ~SkipRobot && ~FilterMatch
      if verblevel > 3 || IsInWhiteList
        fprintf('%s passt nicht zum Filter {%s}.', PNames_Akt{j}, ...
          disp_array(structset.joint_filter, '%s'));
      end
      SkipRobot = true;
    end
    if ~SkipRobot && NumTechJointsDontMatch
      if verblevel > 3 || IsInWhiteList
        fprintf(['%s hat nicht die passende Gelenkzahl in Beinkette (%s). ', ...
          'Ist: %d. Erlaubt: [%s]'], PNames_Akt{j}, SName_TechJoint, ...
          length(SName_TechJoint), disp_array(structset.num_tech_joints, '%d'));
      end
      SkipRobot = true;
    end
    if ~SkipRobot && SphericalJointInChain
      if verblevel > 3 || IsInWhiteList
        fprintf('%s hat ein Kugelgelenk am Anfang oder in der Mitte der Beinkette (%s)', ...
          PNames_Akt{j}, SName_TechJoint);
      end
      SkipRobot = true;
    end
    if ~SkipRobot && UnwantedCylinderLever
      if verblevel > 3 || IsInWhiteList
        fprintf('%s hat einen Hebel auf einen Schubzylinder in der Beinkette (%s)', ...
          PNames_Akt{j}, SName_TechJoint);
      end
      SkipRobot = true;
    end
    if ~SkipRobot && WrongLegChainOrigin
      if verblevel > 3 || IsInWhiteList
        fprintf('%s hat keine Beinkette aus %s-PKM-Synthese (%s).', ...
          PNames_Akt{j}, EEstr, LegChainName);
      end
      SkipRobot = true;
    end
    if SkipRobot % Einer der Ausschlussgründe oben wurde getroffen.
      if IsInWhiteList % Positiv-Liste wird trotzdem genommen.
        fprintf(' Füge trotzdem hinzu, da auf Positiv-Liste.\n');
      else
        if verblevel > 3, fprintf(' Ignoriere.\n'); end
        continue
      end
    end
    % Stelle mögliche Werte für den Strukturparameter theta1 zusammen.
    csvline = serroblib_bits2csvline(SerRob_DB_all.BitArrays_Ndof(ilc,:)); % DH-Parameter der Beinkette aus csv-Datei
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
          (all(Set.task.DoF(1:5) == [1 1 1 0 0]) || ... % 3T0R/3T1R
           all(Set.task.DoF(1:6) == [1 1 1 1 1 0])) % 3T2R (vermutlich hier relevant)
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
      elseif length(StructuralDHParam{1}) ~= sum(I_param)
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
    % Typ der Antriebe herausfinden
    for k = 1 % Annahme: Symmetrische PKM (so wie oben)
      NlegJ = str2double(LEG_Names{k}(2));
      Chain = LEG_Names{k}(3:3+NlegJ-1);
      if all(strcmp(Chain(Actuation{k}), 'P'))
        acttype_i = 'prismatic';
      elseif all(strcmp(Chain(Actuation{k}), 'R'))
       acttype_i = 'revolute';
      else
        acttype_i = 'mixed';
      end
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
        'Coupling', Coupling, 'angles_values', av, 'DoF', EE_FG_allowed(kkk,:), ...
        'act_type', acttype_i, 'deactivated', false, ...
        'RobName', ''); %#ok<AGROW> % Zur Angleichung an SerRob.
    end
  end
end
%% Einträge auf der Liste verdoppeln
% Wenn nur ein Roboter optimiert wird, können durch parallele Berechnung 
% mehr Parameter ausprobiert werden.
if ~isempty(Set.structures.repeatlist)
  for i = 1:length(Set.structures.repeatlist)
    Name_i = Set.structures.repeatlist{i}{1};
    Rep_i = Set.structures.repeatlist{i}{2};
    found = false;
    for j = 1:length(Structures)
      Structure_j = Structures{j};
      Name_j = Structure_j.Name;
      if strcmp(Name_i, Name_j)
        found = true;
        % Hänge diese Einstellung mehrfach ans Ende an.
        for k = 1:Rep_i-1 % einmal ist sie schon drin. also n-1
          Structure_k = Structure_j;
          Structure_k.Number = length(Structures)+1;
          Structures{end+1} = Structure_k; %#ok<AGROW>
        end
        break;
      end
    end
    if ~found
      warning(['Roboter %s soll mehrfach optimiert werden, ist aber gar ', ...
        'nicht in ursprünglicher Lister der Roboter enthalten.'], Name_i);
    end
  end
end
if nargout == 2
  Structures_Names = cell(length(Structures), 1);
  for i = 1:length(Structures)
    Structures_Names{i} = Structures{i}.Name;
  end
end
