% Gesamttest für dieses Struktur- und Maßsynthese- Repo
% 
% Führt alle verfügbaren Modultests aus um die Funktionalität sicherzustellen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-09
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

this_repo_path = fileparts(which('structgeomsynth_path_init.m'));
% Füge die Testfunktionen temporär zum Pfad hinzu. Normalerweise werden sie
% nicht gebraucht und sind nicht im Pfad (structgeomsynth_path_init)
addpath(fullfile(this_repo_path, 'dimsynth', 'test'));
addpath(fullfile(this_repo_path, 'dimsynth', 'config'));

%% Testfälle zum Aufruf aller Zielfunktionen einmal
cds_testcase_2T1R; close all;
cds_testcase_3T0R; close all;
cds_testcase_3T1R; close all;
cds_testcase_3T2R; close all;
cds_testcase_3T3R; close all;

%% Testfälle zum Reproduzieren eines definierten Ergebnisses
cds_testcase_PKM_G1G4; close all;

%% Diverse weitere Tests
% Einige andere einzelne Funktionen werden getestet
cds_example_2T1R_RRR_multiobj; close all;
cds_example_collisioncheck; close all;
cds_example_installationspace; close all;
cds_example_desopt_linkstrength; close all;
cds_example_desopt_test; close all;
cds_example_taskred_desopt; close all;
test_cds_obj_dependencies; close all;
cds_example_external_force; close all;
%% Ende
clc
close all
fprintf('Alle Testfunktionen des Struktur- und Maßsynthese-Repos ausgeführt\n');
