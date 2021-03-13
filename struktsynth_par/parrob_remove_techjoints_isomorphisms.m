% Entferne Isomorphismen, die aus einem Kugelgelenk als Koppelgelenk resultieren.
% Hier sind verschiedene Koppelgelenk-Modi gleichwertig, da die
% Achsrichtung des letzten virtuellen Einzelgelenks relativ zur Plattform
% keine Rolle spielt, wenn es ein Kugelgelenk ist.
% Entferne zusätzlich noch PKM, bei denen mehrwertige Gelenke aktuiert sind

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-03
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

parroblib_gen_bitarrays
serroblibpath=fileparts(which('serroblib_path_init.m'));

% Suche alle PKM und bearbeite die Datenbank
EEFG_Ges = [1 1 0 0 0 1; ...
            1 1 1 0 0 0; ...
            1 1 1 0 0 1; ...
            1 1 1 1 1 0; ...
            1 1 1 1 1 1];
PKM_deleted = zeros(1,5);
PKM_missing = {};
for i_FG = 1:size(EEFG_Ges,1)
  EE_FG = logical(EEFG_Ges(i_FG,:));
  fprintf('Prüfe PKM mit EE-FG %dT%dR\n', sum(EE_FG(1:3)), sum(EE_FG(4:6)));
  [PNames_Kin, PNames_Akt] = parroblib_filter_robots(EE_FG, 6);
  for ii = 1:length(PNames_Akt)
    % Lade Daten zu der PKM
    PName = PNames_Akt{ii};
    fprintf('Prüfe PKM %d/%d: %s\n', ii, length(PNames_Akt), PName);
    [~, LEG_Names, Actuation, Coupling, ~, ~, ~, ~, PName_Legs, ~] = parroblib_load_robot(PName);
    LegJointDoF = str2double(LEG_Names{1}(2));
    mdllistfile_Ndof = fullfile(serroblibpath, sprintf('mdl_%ddof', LegJointDoF), ...
      sprintf('S%d_list.mat', LegJointDoF));
    l = load(mdllistfile_Ndof, 'Names_Ndof', 'BitArrays_Ndof', 'BitArrays_EEdof0', 'AdditionalInfo');
    IIleg = strcmp(l.Names_Ndof, LEG_Names{1});
    SName_TechJoint = fliplr(regexprep(num2str(l.AdditionalInfo(IIleg,7)), ...
        {'1','2','3','4','5'}, {'R','P','C','U','S'}));
    % Prüfe, welche Gelenke aktuiert werden dürfen
    Joints_Actuation_Possible = [];
    for iii = 1:length(SName_TechJoint)
      switch SName_TechJoint(iii)
        case 'R'
          Joints_Actuation_Possible = [Joints_Actuation_Possible, 1]; %#ok<AGROW>
        case 'P'
          Joints_Actuation_Possible = [Joints_Actuation_Possible, 1]; %#ok<AGROW>
        case 'U'
          Joints_Actuation_Possible = [Joints_Actuation_Possible, [0 0]]; %#ok<AGROW>
        case 'S'
          Joints_Actuation_Possible = [Joints_Actuation_Possible, [0 0 0]]; %#ok<AGROW>
        case 'C'
          Joints_Actuation_Possible = [Joints_Actuation_Possible, [0 0]]; %#ok<AGROW>
      end
    end
    delete_robot = false;
    if ~Joints_Actuation_Possible(Actuation{1})
      delete_robot = true;
      fprintf('Beinkette %s (%s) kann nicht %d als aktuiertes Gelenk haben.\n', ...
        LEG_Names{1}, SName_TechJoint, Actuation{1});
    end
    % Prüfe Gestell-Koppelgelenk
    if SName_TechJoint(1)=='S' && all(Coupling(1) ~= [1 5])
      % Prüfe, ob die dafür vorgesehene PKM existiert, bevor gelöscht wird
      if any(Coupling(1) == [2 3 4 9]) % Im Kreis
        BaseCoupling_Soll = 1;
      elseif any(Coupling(1) == [6 7 8]) % Paarweise
        BaseCoupling_Soll = 5;
      else
        error('Fall nicht vorgesehen');
      end
      PName_Iso = strrep(PName, sprintf('G%dP%d', Coupling(1), Coupling(2)), ...
        sprintf('G%dP%d', BaseCoupling_Soll, Coupling(2)));
      if ~any(strcmp(PNames_Akt, PName_Iso))
        warning(['Der Kugelgelenk-Isomorphismus zu %s ist %s. Dieser existiert ', ...
          'aber gar nicht'], PName, PName_Iso);
        PKM_missing = [PKM_missing(:)', {PName_Iso}];
        continue
      end
      delete_robot = true;
      fprintf(['Letztes Gelenk ist Kugelgelenk. Plattform-Koppelgelenk-', ...
        'Stellung %d dafür nicht vorgesehen, sondern %d (doppelt)\n'], ...
        Coupling(1), BaseCoupling_Soll);
    end
    % Prüfe Plattform-Koppelgelenk
    if SName_TechJoint(end)=='S' && all(Coupling(2)~=[1 4])
      % Prüfe, ob die dafür vorgesehene PKM existiert, bevor gelöscht wird
      if any(Coupling(2) == [2 3 8]) % Kreisförmig
        PlfCoupling_Soll = 1;
      elseif any(Coupling(2) == [5 6]) % Paarweise
        PlfCoupling_Soll = 4;
      else
        error('Fall nicht vorgesehen');
      end
      PName_Iso = strrep(PName, sprintf('G%dP%d', Coupling(1), Coupling(2)), ...
        sprintf('G%dP%d', Coupling(1), PlfCoupling_Soll));
      if ~any(strcmp(PNames_Akt, PName_Iso))
        warning('Der Kugelgelenk-Isomorphismus zu %s ist %s. Dieser existiert aber gar nicht', ...
          PName, PName_Iso);
        PKM_missing = [PKM_missing(:)', {PName_Iso}];
        continue
      end
      delete_robot = true;
      fprintf(['Letztes Gelenk ist Kugelgelenk. Plattform-Koppelgelenk-', ...
        'Stellung %d dafür nicht vorgesehen, sondern %d (doppelt)\n'], ...
        Coupling(2), PlfCoupling_Soll);
    end
    if delete_robot
      fprintf('Lösche PKM %s\n', PName);
      deletesuccess = parroblib_remove_robot(PName);
      if ~deletesuccess
        error('Etwas ist beim Löschen schief gelaufen');
      end
      PKM_deleted(i_FG) = PKM_deleted(i_FG) + 1;
    end
  end
end
% Zusammenfassung ausgeben
for i_FG = 1:size(EEFG_Ges,1)
  EE_FG = logical(EEFG_Ges(i_FG,:));
  fprintf('Für EE-FG %dT%dR wurden %d PKM gelöscht.\n', ...
    sum(EE_FG(1:3)), sum(EE_FG(4:6)), PKM_deleted(i_FG));
end