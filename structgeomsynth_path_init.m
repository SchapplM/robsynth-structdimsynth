% Pfad-Initialisierung für dieses Repo

% Moritz Schappler, schappler@imes.uni-hannover.de, 2018-04
% (C) Institut für Mechatronische Systeme, Universität Hannover

this_tb_path = fileparts( mfilename('fullpath') );
addpath(this_tb_path);

addpath(fullfile(this_tb_path, 'dimsynth'));
addpath(fullfile(this_tb_path, 'dimsynth', 'desopt'));
addpath(fullfile(this_tb_path, 'dimsynth', 'fitness'));
addpath(fullfile(this_tb_path, 'dimsynth', 'misc'));
mkdirs(fullfile(this_tb_path, 'results'));
mkdirs(fullfile(this_tb_path, 'tmp'));
