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
  'base_alignment_changed_list20220120_091120_update.txt');
LegNames_updated = readlines(listfile);
% LegNames_updated = {'S5RRRRR5'}; % Debug

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
      'whitelist_SerialKin', LegNames_updated, ...
      'comp_cluster', true, ... % auf Cluster rechnen
      'offline', true, ... % Offline-Auswertung nach Cluster-Berechnung
      'check_resstatus', 0:9, ... % Alle testen
      'selectgeneral', ~select_variants, ... % entweder nur allgemeine Beinketten
      'selectvariants', select_variants, ... % oder nur Varianten (dadurch stärker parallelisiert)
      'max_actuation_idx', 4); % Alle möglichen Aktuierungen prüfen
    if ~settings.comp_cluster
    	settings.parcomp_structsynth = false;
      settings.parcomp_mexcompile = false;
    end
    if all(settings.EE_FG == [1 1 1 1 1 0]) % 3T2R: Nur diese funktionieren
      settings.base_couplings = [1 9 10];
      settings.plf_couplings = 8;
    end
    % Debug:
%     settings.base_couplings = 3;
%     settings.plf_couplings = 8;
%     settings.use_mex = false; % Debug: Damit es schneller startet
    parroblib_add_robots_symact(settings);
    pause(1.0); % damit nicht zwei gleiche Zeitstempel entstehen.
  end
end
