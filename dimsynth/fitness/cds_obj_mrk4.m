% Zielfunktion ("objective function") für Optimierung in der Maßsynthese
% für MRK-Kennzahl (effektive Plattform-Masse).
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Structure
%   Eigenschaften der Roboterstruktur (aus cds_gen_robot_list.m)
% Traj_0
%   Endeffektor-Trajektorie (bezogen auf Basis-KS)
% Q
%   Gelenkwinkel-Trajektorie
% Jinvges
%   Zeilenweise (inverse) Jacobi-Matrizen des Roboters (für PKM). Bezogen
%   auf vollständige Gelenkgeschwindigkeiten und EE-Geschw.
% 
% Ausgabe:
% fval [1x1]
%   Zielfunktionswert, der im PSO-Algorithmus minimiert wird.
% fval_debugtext [char]
%   Zeile mit Hinweistext, der bei PSO nach Fitness-Berechnung ausgegeben wird
% debug_info [cell]
%   Zusatz-Informationen, die im Debug-Bild des Roboters angezeigt werden
% f_phys [1x1]
%   Physikalischer Wert, der dem Zielfunktionswert zugrunde liegt
%   Hier: größte Effektive Masse der EE-Plattform
% 
% Siehe auch: cds_obj_mass.m, cds_obj_dependencies.m
% ParRob/invdyn2_platform.m (Code-Variante 2, unten)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval, fval_debugtext, debug_info, f_phys] = cds_obj_mrk4(R, Set, Structure, Traj_0, Q, Jinvges)
% Debug-Code:
if Set.general.matfile_verbosity > 2 % Debug
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_obj_mrk4.mat'));
end
% clear
% clc
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_obj_mrk4.mat'));

debug_info = '';

%% Berechne die Massenmatrix
XP = R.xE2xP_traj(Traj_0.X); % Plattform-Koordinaten benötigt
M_all = NaN(size(Q,1), 1);
for i = 1:size(Q,1)
  Jinv = reshape(Jinvges(i,:), R.NJ, sum(R.I_EE));
  M_full = R.inertia2_platform_full(Q(i,:)', XP(i,:)'); % Subsystem-Methode
  M_i  = R.inertia2_platform(Q(i,:)', XP(i,:)', Jinv, M_full); % Projektion auf Plattform
  M_ev = eig(M_i(1:3,1:3));
  M_all(i) = max(M_ev);
  continue
  % Debug: Prüfe, welche Anteile die einzelnen Subsysteme haben
  M_ev_dbg_jj = NaN(1,R.NLEG+1); %#ok<UNRCH> 
  for jj = 1:R.NLEG+1
    M_full_jj = zeros(size(M_full));
    if jj < R.NLEG+1 % Beinkette
      II = (jj-1)*R.Leg(jj).NJ+1:R.Leg(jj).NJ*jj; % Siehe Indizierung in inertia2_platform_full
    else % Plattform
      II = R.NJ+1:size(M_full_jj,1);
    end
    M_full_jj(II,II) = M_full(II,II);
    M_jj = R.inertia2_platform(Q(i,:)', XP(i,:)', Jinv, M_full_jj);
    M_ev_dbg_jj(jj) = max(eig(M_jj(1:3,1:3)));
  end
  % Auswertung: Größte Eigenwerte (bzgl. translatorische Plattform-MM) der
  % Beinketten und der Plattform im Vergleich. Keine absolute Aussage
  % möglich, da Addition der EW keine Aussage über EW der Summen-Matrix
end
%% Zielfunktion
f_phys = max(M_all);
f_mass_norm = 2/pi*atan(f_phys/10); % Normierung auf 0 bis 1; 62 ist 0.9.
fval = 1e3*f_mass_norm; % Normiert auf 0 bis 1e3
fval_debugtext = sprintf('Größte effektive Plattformmasse %1.1fkg.', f_phys);