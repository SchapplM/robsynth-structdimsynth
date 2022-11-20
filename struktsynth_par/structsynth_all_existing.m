% Prüfe erneut alle PKM in der Datenbank. Erweitere die Datenbank um die
% Gelenkgruppen, die für die Bestimmung des Namens für Publikationen
% notwendig sind. Wurde am 15.11.2022 hinzugefügt.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-11
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover
clc
clear

for EE_FG_Nr = 3:6 % 1=2T0R, 2=2T1R, 3=3T0R, ...
  %% Start der Struktursynthese
  settings = struct( ...
    'EE_FG_Nr', EE_FG_Nr, ...
    'dryrun', false, ...
    'parcomp_structsynth', 1, ... % parfor-Struktursynthese (schneller, aber mehr Speicher notwendig)
    'parcomp_mexcompile', 0, ... % parfor-Mex-Kompilierung (schneller, aber Dateikonflikt möglich)
    'check_existing', true, ...
    'check_missing', false, ...
    'check_resstatus', [0:10], ... % 0=nur falls Rang-Prüfung schon mal erfolgreich war
    'max_actuation_idx', 5); 
    
  % Debug:
%   settings.base_couplings = 2;
%   settings.plf_couplings = 1;
%   settings.whitelist_SerialKin = 'S5RPRRR8';

%   settings.clusterjobdepend = 890999;

  % Modifikation der Einstellungen: Auf Cluster rechnen.
  settings.comp_cluster = false;
  settings.offline = false;
  settings.clustercomp_if_res_olderthan = 0.5; % 0=Immer neu generieren
  % Bestehende vorher i.O. erzeugte PKM in der Datenbank nicht aktualisieren
  settings.resstatus_downgrade_possible = false;
  % Nur falls die Eigenschaft der Gelenkgruppen fehlt.
  settings.check_only_missing_joint_parallelity = true;
  % Bei mehrfachem Aufruf muss der Compile-Job irgendwann nicht mehr
  % gestartet werden.
  settings.compile_job_on_cluster = false;
  parroblib_add_robots_symact(settings);
end