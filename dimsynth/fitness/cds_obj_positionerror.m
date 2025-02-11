% Zielfunktion ("objective function") für Optimierung in der Maßsynthese
% basierend auf Lagegenauigkeit des Roboters.
% Der Positionsfehler wird in einen normierten Zielfunktionswert übersetzt
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Jinvges
%   Zeilenweise (inverse) Jacobi-Matrizen des Roboters (für PKM). Bezogen
%   auf vollständige Gelenkgeschwindigkeiten und EE-Geschw.
% Q
%   Gelenkpositionen des Roboters (für PKM auch passive Gelenke)
% 
% Ausgabe:
% fval [1x1]
%   Zielfunktionswert, der im PSO-Algorithmus minimiert wird
% fval_debugtext [char]
%   Zeile mit Hinweistext, der bei PSO nach Fitness-Berechnung ausgegeben wird
% debug_info [cell]
%   Zusatz-Informationen, die im Debug-Bild des Roboters angezeigt werden
% f_poserr [1x1]
%   Physikalischer Wert, der dem Zielfunktionswert zugrunde liegt
%   Hier: Maximaler Positionsfehler des Endeffektors (in Trajektorie)
% 
% Quelle:
% * [Rob2LUH] Skript Robotik II (Prof. Ortmaier, Uni Hannover), Kap. 1.6.3

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-10
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval, fval_debugtext, debug_info, f_poserr] = cds_obj_positionerror(R, Set, Jinvges, Q)
debug_info = {};

% Berechne Positionsfehler über Trajektorie
deltapges = NaN(size(Q,1), 1);
if any(R.Type == [0 1]) % Seriell
  delta_qa = R.update_q_poserr();
  % Berechne Manipulierbarkeit für alle Punkte der Bahn
  for i = 1:size(Q,1)
    J_3T = R.jacobit(Q(i,:)'); % nur translatorisch
    J_transl = J_3T(Set.task.DoF(1:3),:);
    % Berechne Positionsfehler (siehe [Rob2LUH])
    deltapges(i,:) = norm(abs(J_transl)*delta_qa);
  end
else % PKM
  [~, delta_qa] = R.update_q_poserr();
  % Berechne Positionsfehler für alle Punkte der Bahn
  for i = 1:size(Q,1)
    Jinv_IK = reshape(Jinvges(i,:), R.NJ, sum(R.I_EE));
    J = inv(Jinv_IK(R.I_qa,:));
    % Wähle translatorischen Teil der Jacobi aus
    J_transl = J(Set.task.DoF(1:3),:);
    deltapges(i,:) = norm(abs(J_transl)*delta_qa); % siehe [Rob2LUH]
  end
end
% Maximaler Fehler (über Trajektorie) ist Kennzahl
f_poserr = max(deltapges);
f_poserr_norm = 2/pi*atan(f_poserr/1.5e-4); % Normierung auf 0 bis 1; 1e-3 ist 0.9.
fval = 1e3*f_poserr_norm; % Normiert auf 0 bis 1e3
fval_debugtext = sprintf('Schlechtester Positionsfehler %1.1fµm.', 1e6*f_poserr);