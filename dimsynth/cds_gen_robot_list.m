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
if all(structset.DoF == [1 1 1 1 1 0])
  EE_FG      = [[1 1 1], [1 1 1], [1 1 1]];
  EE_FG_Mask = [[1 1 1], [1 1 1], [1 1 0]];
end

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
  % Varianten von Robotern in der Datenbank werden nicht verwendet, außer
  % es wird explizit eine Variante in der Positiv-Liste genannt
  if ~any(contains(structset.whitelist, 'V'))
    I = I & I_novar;
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
    fprintf('%d: %s\n', ii, SName);
    Structures{ii} = struct('Name', SName, 'Type', 0, 'Number', ii);
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
    [NLEG, LEG_Names, Actuation, Coupling, ~, ~, ~] = parroblib_load_robot(PNames_Akt{j});
    % Prüfe Koppelpunkt-Eigenschaften
    if ~any(Coupling(1) == [1:8]) || ~any(Coupling(2) == [1:6])
      if verblevel >= 3, fprintf('%s hat eine nicht implementierte Koppelpunkt-Variante\n', PNames_Akt{j}); end
      continue % Robotermodell kann in Optimierung nicht generiert werden.
    end
    
    PassPrisJoint = false;
    TooManyPrisJoints = false;
    LastJointActive = false;
    DistalJointActive = false;
    FilterMatch = true;
    WrongLegChainOrigin = false;
    for k = 1:NLEG % Gehe alle Beinketten durch (für den Fall asymmetrischer PKM)
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
        l = load(mdllistfile_Ndof, 'Names_Ndof', 'BitArrays_Origin');
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
          % Keine Einschränkung
        end
        if bitand(Mask_Origin, l.BitArrays_Origin(ilc,:)) == 0
          WrongLegChainOrigin = true;
          break;
        end
      end
    end
    if structset.nopassiveprismatic && PassPrisJoint % PKM enthält passive Schubgelenke. Nicht auswählen
      if verblevel >= 3, fprintf('%s hat passives Schubgelenk. Ignoriere\n', PNames_Akt{j}); end
      continue
    end
    if TooManyPrisJoints
      if verblevel >= 3, fprintf('%s hat zu viele Schubgelenke. Ignoriere\n', PNames_Akt{j}); end
      continue
    end
    if structset.activenotlastjoint && LastJointActive
      if verblevel >= 3, fprintf('%s hat aktives letztes Gelenk. Ignoriere\n', PNames_Akt{j}); end
      continue
    end
    if DistalJointActive
      if verblevel >= 3, fprintf('%s hat aktives Gelenk nach Position %d. Ignoriere\n', PNames_Akt{j}, Set.structures.max_index_active); end
      continue
    end
    if ~FilterMatch
      if verblevel >= 3, fprintf('%s passt nicht zum Filter %s. Ignoriere\n', PNames_Akt{j}, structset.joint_filter); end
      continue
    end
    if WrongLegChainOrigin
      if verblevel >= 3, fprintf('%s hat keine Beinkette aus %s-PKM-Synthese (%s). Ignoriere\n', PNames_Akt{j}, task_str, LegChainName); end
      continue
    end
    % TODO: Mögliche Basis-Anordnungen von PKM hier generieren und hinzufügen

    ii = ii + 1;
    if verblevel >= 2, fprintf('%d: %s\n', ii, PNames_Akt{j}); end
    Structures{ii} = struct('Name', PNames_Akt{j}, 'Type', 2, 'Number', ii, 'Coupling', Coupling);

  end
end
