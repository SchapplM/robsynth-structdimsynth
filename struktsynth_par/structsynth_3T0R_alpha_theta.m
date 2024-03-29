% Erzeuge beispielhafte Roboter mit Struktursynthese, die Beinketten
% mit freien alpha- und theta-Parameter enthalten

% Ketten zum Testen:
% * S5RRRRR10V1 - Delta-Roboter (P3RRPRR10V1G2P2A1) - ein Parameter (alpha2)
% * S5PRRRR8V1 - ein Parameter (theta1)
% * S5PRRRR8 - zwei Parameter (theta1,alpha2)
% * S5RRRRR12 - zwei Parameter (alpha2,alpha3)
% * S5PRRRR10V1 - drei Parameter (theta1,alpha2,alpha3)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-03
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
structgeomsynthpath=fileparts(which('structgeomsynth_path_init.m'));
addpath(fullfile(structgeomsynthpath, 'struktsynth_par'));
% Stelle alle Beinketten mit alpha und theta zusammen (zur Neu-Generierung)

settings = struct( ...
  'EE_FG', logical([1 1 1 0 0 0]), ... % 3T0R
  'dryrun', false, ...
  'check_existing', true, ...
  'base_couplings', 2, ...
  'plf_couplings', 2, ...
  'parcomp_structsynth', 0, ...
  'parcomp_mexcompile', 0, ...
  'whitelist_SerialKin', 'S5PRRRR8V1', ...
  'max_actuation_idx', 1); 
  
parroblib_add_robots_symact(settings);

return

%% Alle PKM generieren
% Suche serielle Beinketten, die einen freien alpha- oder theta- Parameter
% haben
serroblibpath=fileparts(which('serroblib_path_init.m'));
whitelist = {};
for i = 3:5
  mdllistfile_Ndof = fullfile(serroblibpath, sprintf('mdl_%ddof', i), sprintf('S%d_list.mat',i));
  l = load(mdllistfile_Ndof);
  for j = 1:length(l.Names_Ndof)
    % Debug: Nehme nur Beinketten mit P-Gelenk oder R-Gelenk am Anfang
    % (Auskommentieren für Deaktivierung des Filters)
    % if ~strcmp(l.Names_Ndof{j}(3), 'P'), continue; end % nur P-Gelenk am Anfang
    % if ~strcmp(l.Names_Ndof{j}(3), 'R'), continue; end % nur R-Gelenk am Anfang
    csvline = serroblib_bits2csvline(l.BitArrays_Ndof(j,:));
    if any(contains(csvline, 'alpha')) || any(contains(csvline, 'theta'))
      whitelist = [whitelist, l.Names_Ndof{j}];
    end
  end
end
fprintf('%d Strukturen in Positiv-Liste\n', length(whitelist));
% Starte die Struktursynthese für alle PKM neu
settings = struct('whitelist_SerialKin', {whitelist});
% settings.whitelist_SerialKin = {'S5RRRRR10V1'}; % Zum Debuggen
for select_variants = [false, true]
  settings = struct( ...
    'EE_FG', logical([1 1 1 0 0 0]), ... % 3T0R
    'dryrun', false, ...
    'check_existing', true, ...
    ... 'base_couplings', 2, ... % Zum nachträglichen Testen einzelner PKM
    ... 'plf_couplings', 3, ... % zum Debuggen
    'clustercomp_if_res_olderthan', 2, ... % Nicht die alten Ergebnisse anschauen zur Vermeidung der Cluster-Berechnung
    'check_missing', true, ...
    'comp_cluster', false, ... % auf Cluster rechnen: true; Offline-Auswertung: false
    'offline', true, ... % Nach dem Herunterladen der Ergebnisse vom Cluster auf true; für Rechnen auf Cluster: false
    'check_resstatus', 0:8, ... % Alle testen
    'selectgeneral', ~select_variants, ... % entweder nur allgemeine Beinketten
    'selectvariants', select_variants, ... % oder nur Varianten (dadurch stärker parallelisiert)
    'whitelist_SerialKin', {settings.whitelist_SerialKin}); % wegen clear Befehl im Skript
  pause(1.0); % damit nicht zwei gleiche Zeitstempel entstehen.
  parroblib_add_robots_symact(settings);
end
%% Debug: Nutze Roboter direkt in der Maßsynthese (zum Testen)
DoF = [1 1 1 0 0 0];
Set = cds_settings_defaults(struct('DoF', DoF));
Set.optimization.NumIndividuals = 40;
Set.optimization.MaxIter = 15;
Set.general.matfile_verbosity = 1;
% Set.structures.whitelist = {'P3RRRRR10V1G2P2A1'};
% Set.structures.joint_filter = 'P****';
Set.optimization.objective = 'condition';
Set.optimization.obj_limit = 1e3; % Gültige Lösung führt zum Abbruch.
Set.general.parcomp_struct = true;
Set.optimization.optname = 'synth_alphatheta_test';
Traj = cds_gen_traj(DoF, 1, Set.task);
cds_start(Set,Traj);