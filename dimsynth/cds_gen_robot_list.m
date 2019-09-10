% Erzeuge Listen von Robotern, die für die Optimierung genutzt werden

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

% Name der FG für Zugriff auf Listen
if all(structset.DoF == [1 1 0 0 0 1])
  task_str = '2T1R';
elseif all(structset.DoF == [1 1 1 0 0 1])
  task_str = '3T1R';
elseif all(structset.DoF == [1 1 1 0 0 0])
  task_str = '3T0R';
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
ii = 0; % Laufende Nummer für alle Roboterstrukturen (seriell und parallel)

%% Serielle aus Liste Roboter laden
% serroblibpath=fileparts(which('serroblib_path_init.m'));
% mdllistfile = fullfile(serroblibpath, 'lists', sprintf('structsynth_%s.csv',task_str));

% res = xlsread(mdllistfile);
% res = csvread(mdllistfile);

%% Serielle Roboter laden
if structset.use_serial
  N_JointDoF = sum(EE_FG); % Beinketten ohne irgendeine Redundanz (so viele Gelenke wie EE FG)
  serroblibpath=fileparts(which('serroblib_path_init.m'));
  mdllistfile_Ndof = fullfile(serroblibpath, sprintf('mdl_%ddof', N_JointDoF), sprintf('S%d_list.mat',N_JointDoF));
  l = load(mdllistfile_Ndof, 'Names_Ndof', 'AdditionalInfo');
  [~,I_FG] = serroblib_filter_robots(N_JointDoF, EE_FG, EE_FG_Mask);
  I_novar = (l.AdditionalInfo(:,2) == 0);
  I = I_FG & I_novar;
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
  [PNames_Kin, PNames_Akt] = parroblib_filter_robots(sum(EE_FG), EE_FG, EE_FG_Mask);
  for j = 1:length(PNames_Akt)
    if ~isempty(structset.whitelist) && ~any(strcmp(structset.whitelist, PNames_Akt{j}))
      % Es gibt eine Liste von Robotern, dieser ist nicht dabei.
      continue
    end
    
    % Lade Detailierte Informationen des Robotermodells
    [NLEG, LEG_Names, Actuation, ActNr, symrob, EE_dof0, PName_Kin] = parroblib_load_robot(PNames_Akt{j});

    PassPrisJoint = false;
    TooManyPrisJoints = false;
    LastJointActive = false;
    for k = 1:NLEG % Gehe alle Beinketten durch (für den Fall asymmetrischer PKM)
      LegChainName = LEG_Names{k};
      NLegDoF = str2double(LegChainName(2));
      % Prüfe, ob passive Schubgelenke vorliegen
      for l = 1:NLegDoF % Gehe alle Beingelenke
        if strcmp(LegChainName(2+l), 'P') && ~any(Actuation{k} == l)
          PassPrisJoint = true;
        end
      end
      % Prüfe die Anzahl der Schubgelenke
      numprismatic = sum(LegChainName == 'P');
      if numprismatic > structset.maxnumprismatic
        TooManyPrisJoints = true;
      end
      % Prüfe, ob letztes Gelenk aktiv ist
      if any(Actuation{k} == NLegDoF)
        LastJointActive = true;
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

    % TODO: Mögliche Basis-Anordnungen von PKM hier generieren und hinzufügen

    ii = ii + 1;
    if verblevel >= 2, fprintf('%d: %s\n', ii, PNames_Akt{j}); end
    Structures{ii} = struct('Name', PNames_Akt{j}, 'Type', 2, 'Number', ii);

  end
end
