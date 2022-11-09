% Erzeuge PKM mit EE-FG 3T0R mit Plattform-Koppelgelenk-Ausrichtung P9

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-03
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
EE_FG = logical([1 1 1 0 0 0]);
%% Prüfung der PKM-Datenbank
[PNames_Kin, PNames_Akt] = parroblib_filter_robots(EE_FG, 6);
PNames_Kin = PNames_Kin{contains(PNames_Kin, 'G4')}; % konische Anordnung der Gestell-Gelenke
PNames_Akt = PNames_Akt{contains(PNames_Akt, 'G4')};

%% Start der Struktursynthese
settings = struct( ...
  'EE_FG', EE_FG, ... % 3T0R
  'dryrun', false, ...
  'base_couplings', 1:10, ... % siehe ParRob/align_base_coupling
  'plf_couplings', 9, ... % siehe ParRob/align_platform_coupling
  'parcomp_structsynth', 0, ... % parfor-Struktursynthese (schneller, aber mehr Speicher notwendig)
  'parcomp_mexcompile', 0, ... % parfor-Mex-Kompilierung (schneller, aber Dateikonflikt möglich)
  ... 'whitelist_SerialKin', 'S5RRRRR6', ...
  'check_existing', true, ...
  'max_actuation_idx', 4); 

% Modifikation der Einstellungen: Auf Cluster rechnen.
settings.comp_cluster = true;
  
parroblib_add_robots_symact(settings);