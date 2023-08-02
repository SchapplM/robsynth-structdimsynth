% Zielfunktion ("objective function") für Optimierung in der Maßsynthese
% für MRK-Kennzahl (Platzhalter).
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Structure
%   Eigenschaften der Roboterstruktur
% Traj_0
%   Endeffektor-Trajektorie (bezogen auf Basis-KS)
% Q
%   Gelenkwinkel-Trajektorie
% JP
%   Gelenkpositionen aller Gelenke des Roboters
%   (Zeilen: Zeitschritte der Trajektorie)
% 
% Ausgabe:
% fval [1x1]
%   Zielfunktionswert, der im PSO-Algorithmus minimiert wird. Benutze den
% fval_debugtext [char]
%   Zeile mit Hinweistext, der bei PSO nach Fitness-Berechnung ausgegeben wird
% debug_info [cell]
%   Zusatz-Informationen, die im Debug-Bild des Roboters angezeigt werden
% f_mrk [1x1]
%   Physikalischer Wert, der dem Zielfunktionswert zugrunde liegt

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval, fval_debugtext, debug_info, f_mrk] = cds_obj_mrk2(R, Set, Structure, Traj_0, Q, JP)
% Platzhalter:
debug_info = '';
fval = 0;
fval_debugtext = '';
f_mrk = 0;