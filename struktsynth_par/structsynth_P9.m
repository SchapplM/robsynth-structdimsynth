% Erzeuge PKM mit EE-FG 3T0R mit Plattform-Koppelgelenk-Ausrichtung P9

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-03
% (C) Institut für Mechatronische Systeme, Universität Hannover
clc
clear
for ii = 2%1:3
  %% EE-FG definieren
  if ii == 1
    EE_FG = logical([1 1 1 0 0 0]);
  elseif ii == 2
    EE_FG = logical([1 1 1 0 0 1]);
  else
    EE_FG = logical([1 1 1 1 1 0]);
  end
  %% Datenbank filtern zur Übersicht
  [PNames_Kin, PNames_Akt] = parroblib_filter_robots(EE_FG, 6);
  PNames_Kin_G4 = PNames_Kin(contains(PNames_Kin, 'G4')); % konische Anordnung der Gestell-Gelenke
  PNames_Akt_G4 = PNames_Akt(contains(PNames_Akt, 'G4'));
  fprintf(['Es gibt %d Parallelkinematiken mit %d Aktuierungen mit ', ...
    'G4-Methode (konische Gestellgelenke): %s\n'], length(PNames_Kin_G4), ...
    length(PNames_Akt_G4), disp_array(PNames_Akt_G4(:)', '%s'));
  PNames_Kin_P9 = PNames_Kin(contains(PNames_Kin, 'P9')); % konische Anordnung der Gestell-Gelenke
  PNames_Akt_P9 = PNames_Akt(contains(PNames_Akt, 'P9'));
  fprintf(['Es gibt %d Parallelkinematiken mit %d Aktuierungen mit ', ...
    'P9-Methode (konische Plattformgelenke): %s\n'], length(PNames_Kin_P9), ...
    length(PNames_Akt_P9), disp_array(PNames_Akt_P9(:)', '%s'));

  %% Start der Struktursynthese
%   P4PRRRR6G9P1A1 P4PRRRR6G3P8A1
  settings = struct( ...
    'EE_FG', EE_FG, ... % 3T0R
    'dryrun', false, ...
    'base_couplings', 3, ... % siehe ParRob/align_base_coupling
    'plf_couplings', 8, ... % siehe ParRob/align_platform_coupling
    'parcomp_structsynth', 0, ... % parfor-Struktursynthese (schneller, aber mehr Speicher notwendig)
    'parcomp_mexcompile', 0, ... % parfor-Mex-Kompilierung (schneller, aber Dateikonflikt möglich)
    'whitelist_SerialKin', 'S5PRRRR6', ...
    'check_existing', true, ...
    'max_actuation_idx', 4); 
  
  % Modifikation der Einstellungen: Auf Cluster rechnen.
  settings.comp_cluster = false;
  settings.offline = false;
  settings.clustercomp_if_res_olderthan = 7; % bis zu eine Woche alte Ergebnisse nutzen
  
  parroblib_add_robots_symact(settings);
end