% Definitionen für die Maßsynthese: Konstanten und Variablen, die im Code
% an mehreren Stellen verwendet werden
% 
% Ausgabe:
% defstruct
%   Struktur mit Einträgen, die jeweils einer Variable im Code entsprechen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function defstruct = cds_definitions()

defstruct = struct();
% Nebenbedingungen der Optimierung
defstruct.objconstr_names_all = {'mass', 'energy', 'actforce', 'condition', ...
  'stiffness', 'materialstress', 'positionerror'};
% Zielkriterien der Optimierung (für Ausgabe aller Zielkriterien in Tabelle)
defstruct.obj_names_all = {'mass', 'energy', 'power', 'actforce', 'materialstress', 'condition', ...
  'manipulability', 'minjacsingval', 'positionerror', 'jointrange', 'jointlimit', ...
  'actvelo','chainlength', 'installspace', 'footprint', 'colldist', 'stiffness'}; % konsistent zu fval_obj_all und physval_obj_all
% Zielkriterien der inversen Kinematik
defstruct.objective_ik_names_all = {'default', 'ikjac_cond', 'jac_cond', ...
  'coll_par', 'instspc_par', 'poserr_ee', 'none', 'constant'};