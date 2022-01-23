% Führe PKM-Struktursynthese für aktualisierte Beinketten neu durch
% 
% Vorher: Liste der aktualisierten Beinketten erstellen, DB aufräumen:
% * serrob_mdlbib/scripts/remove_base_alignment_from_MDH.m
% * serrob_mdlbib/scripts/remove_template_files_updated_chains.m
% * parrob_mdlbib/scripts/remove_pkm_updated_legs.m
% * parrob_mdlbib/scripts/clean_directories_missing_pkm.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc

papath = fileparts(which('robsynth_projektablage_path.m'));
listfile = fullfile(papath, '03_Entwicklung', 'Struktursynthese', ...
  '20220121_Aktualisierung_Beinketten_Basisausrichtung', ...
  'base_alignment_changed_list20220120_091120.txt');
LegNames_updated = readlines(listfile);
% LegNames_updated = {'S5RRRRR5'}; % Debug

% Bestimme die Varianten, die von den Beinketten abhängen
roblibpath=fileparts(which('serroblib_path_init.m'));
mdllistfile_Ndof = fullfile(roblibpath, 'serrob_list.mat');
l = load(mdllistfile_Ndof);
LegNames_updated_var = {};
LegNames_updated_cell = {};
for i = 1:length(LegNames_updated)
  j = find(strcmp(l.Names, LegNames_updated{i}));
  if isempty(j), continue; end % ungültiger Eintrag
  LegNames_updated_cell = [LegNames_updated_cell, LegNames_updated{i}]; %#ok<AGROW> 
  j_variants = find((l.AdditionalInfo(:,3) == j & l.AdditionalInfo(:,2) == 1));
  if isempty(j_variants), continue; end
 LegNames_updated_var = [LegNames_updated_var, l.Names(j_variants)]; %#ok<AGROW> 
end
LegNames_updated_all = [LegNames_updated_cell, LegNames_updated_var];
EE_FG_ges = [ ...
  1 1 1 0 0 0; ...
  1 1 1 0 0 1; ...
  1 1 1 1 1 0];
for iDoF = 1:3 % 3T0R, 3T1R, 3T2R
  for select_variants = [false, true]
    settings = struct( ...
      'EE_FG', EE_FG_ges(iDoF,:), ...
      'check_existing', true, ... % alle existierenden nochmal prüfen
      'check_missing', true, ...  % fehlende hinzufügen
      'whitelist_SerialKin', {LegNames_updated_all}, ...
      'comp_cluster', true, ... % auf Cluster rechnen
      'offline', false, ... % Offline-Auswertung nach Cluster-Berechnung
      'check_resstatus', 0:9, ... % Alle testen
      'selectgeneral', ~select_variants, ... % entweder nur allgemeine Beinketten
      'selectvariants', select_variants, ... % oder nur Varianten (dadurch stärker parallelisiert)
      'max_actuation_idx', 4); % Alle möglichen Aktuierungen prüfen
    if ~settings.comp_cluster
    	settings.parcomp_structsynth = false;
      settings.parcomp_mexcompile = false;
    end
    if all(settings.EE_FG == [1 1 1 1 1 0])
      settings.base_couplings = [1 9 10];
      settings.plf_couplings = 8;
    end
    parroblib_add_robots_symact(settings);
    pause(1.0); % damit nicht zwei gleiche Zeitstempel entstehen.
  end
end