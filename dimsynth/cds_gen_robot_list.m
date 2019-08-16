% Erzeuge Listen von Robotern, die für die Optimierung genutzt werden

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Structures = cds_gen_robot_list(structset)

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
%% Serielle aus Liste Roboter laden
% serroblibpath=fileparts(which('serroblib_path_init.m'));
% mdllistfile = fullfile(serroblibpath, 'lists', sprintf('structsynth_%s.csv',task_str));

% res = xlsread(mdllistfile);
% res = csvread(mdllistfile);

%% Serielle Roboter laden
EE_FG = [1 1 0 0 0 1]; % 110001=planare Bewegung
EE_FG_Mask = [1 1 1 1 1 1]; % Die FG müssen genauso auch vom Roboter erfüllt werden (0 darf nicht auch 1 sein)
N_JointDoF = 3; % Beinketten mit 3 Gelenk-FG
serroblibpath=fileparts(which('serroblib_path_init.m'));
mdllistfile_Ndof = fullfile(serroblibpath, sprintf('mdl_%ddof', N_JointDoF), sprintf('S%d_list.mat',N_JointDoF));
l = load(mdllistfile_Ndof, 'Names_Ndof', 'AdditionalInfo');
[~,I_FG] = serroblib_filter_robots(N_JointDoF, EE_FG, EE_FG_Mask);
I_novar = (l.AdditionalInfo(:,2) == 0);
I = I_FG & I_novar;
II = find(I);
ii = 0;
for j = II'
  SName = l.Names_Ndof{j};
  % Prüfe Anzahl Schubgelenke
  numprismatic = sum(SName == 'P');
  if numprismatic > structset.maxnumprismatic
    continue
  end
  
  ii = ii + 1;

  fprintf('%s\n', SName);
  Structures{ii} = struct('Name', SName, 'Type', 0);
end
%% Parallele Roboter aus Liste laden
% parroblibpath=fileparts(which('parroblib_path_init.m'));

%% Parallele Roboter laden
[PNames_Kin, PNames_Akt] = parroblib_filter_robots(sum(EE_FG), EE_FG, EE_FG_Mask);
for j = 1:length(PNames_Akt)
  
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
%     fprintf('%s hat passives Schubgelenk. Ignoriere\n', PNames_Akt{j});
    continue
  end
  if TooManyPrisJoints
%     fprintf('%s hat zu viele Schubgelenke. Ignoriere\n', PNames_Akt{j});
    continue
  end
  if structset.activenotlastjoint && LastJointActive
%     fprintf('%s hat aktives letztes Gelenk. Ignoriere\n', PNames_Akt{j});
    continue
  end
  
  % TODO: Mögliche Basis-Anordnungen von PKM hier generieren und hinzufügen
  
  
  fprintf('%s\n', PNames_Akt{j});
  ii = ii + 1;
  Structures{ii} = struct('Name', PNames_Akt{j}, 'Type', 2);
  
end

