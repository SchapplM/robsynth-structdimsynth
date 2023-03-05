% Führe die Struktursynthese neu durch für Beinketten, die ein Schubgelenk
% ohne Hebel haben. Wurden Anfang März 2023 eingefügt.
% Prüfe auch direkt alle Beinketten, die ein Schubgelenk haben

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-03
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

for EE_FG_Nr = 3:6 % 1=2T0R, 2=2T1R, 3=3T0R, ...
  %% Start der Struktursynthese
  settings = struct( ...
    'EE_FG_Nr', EE_FG_Nr, ...
    'dryrun', false, ...
    'fixed_number_prismatic', 1, ... % Für diese PKM gab es eine Aktualisierung mit den neuen Beinketten
    'parcomp_structsynth', 1, ... % parfor-Struktursynthese (schneller, aber mehr Speicher notwendig)
    'parcomp_mexcompile', 0, ... % parfor-Mex-Kompilierung (schneller, aber Dateikonflikt möglich)
    'check_existing', true, ...
    'check_missing', true, ...
    'check_resstatus', 3:10, ... % siehe parroblib_update_csv; prüfe solche, die noch kein positives Ergebnis haben
    'max_actuation_idx', 5); 
    
  % Debug:
%   settings.base_couplings = 2;
%   settings.plf_couplings = 2;
%   settings.whitelist_SerialKin = 'S6RRPRRR13V5';

  settings.clusterjobdepend = 1236582;

  % Modifikation der Einstellungen: Auf Cluster rechnen.
  settings.comp_cluster = true;
  settings.offline = false;
  settings.clustercomp_if_res_olderthan = 2; % 0=Immer neu generieren
  % Bestehende vorher i.O. erzeugte PKM in der Datenbank nicht aktualisieren
  settings.resstatus_downgrade_possible = false;
  % Nur falls die Eigenschaft der Gelenkgruppen fehlt: true
  settings.check_only_missing_joint_parallelity = false;
  % Bei mehrfachem Aufruf muss der Compile-Job irgendwann nicht mehr
  % gestartet werden.
  settings.compile_job_on_cluster = true;
  parroblib_add_robots_symact(settings);
end