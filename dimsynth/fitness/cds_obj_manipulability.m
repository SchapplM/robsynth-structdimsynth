% Zielfunktion ("objective function") für Optimierung in der Maßsynthese
% basierend auf der Manipulierbarkeit des Roboters.
% Die Manipulierbarkeit wird in einen normierten Zielfunktionswert übersetzt
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Jinvges
%   Zeilenweise (inverse) Jacobi-Matrizen des Roboters (für PKM). Bezogen
%   auf vollständige Gelenkgeschwindigkeiten und EE-Geschw.
% Traj_0
%   Endeffektor-Trajektorie (bezogen auf Basis-KS)
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
% f_mani1 [1x1]
%   Physikalischer Wert, der dem Zielfunktionswert zugrunde liegt
%   Hier: Kehrwert der minimale Manipulierbarkeit der Jacobi-Matrix
%   (Kehrwert, da die Größe minimiert werden soll und größere
%   Manipulierbarkeit besser ist)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-10
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval, fval_debugtext, debug_info, f_mani1] = cds_obj_manipulability(R, Set, Jinvges, Traj_0, Q)
debug_info = {};

% Berechne Konditionszahl über Trajektorie
muges = NaN(length(Traj_0.t), 1);
if R.Type == 0 % Seriell
  % Berechne Manipulierbarkeit für alle Punkte der Bahn
  for i = 1:length(Traj_0.t)
    J_3T3R = R.jacobia(Q(i,:)'); % analytisch wegen 3T2R; für alle anderen geht auch jacobig
    J_task = J_3T3R(Set.task.DoF,:);
    if size(J_task,1) < size(J_task,2) % Redundanz; allgemeiner Fall
      muges(i,:) = sqrt(det(J_task*J_task'));
    else % nicht-redundant
      muges(i,:) = abs(det(J_task));
    end
  end
else % PKM
  % Berechne Manipulierbarkeit für alle Punkte der Bahn
  for i = 1:length(Traj_0.t)
    Jinv_IK = reshape(Jinvges(i,:), R.NJ, sum(R.I_EE));
    if sum(R.I_EE_Task) < sum(R.I_EE) % Aufgabenredundanz;
      % Doppelt-Invertiert zum seriellen Fall, da nur die inverse Jacobi
      % für PKM bestimmbar ist. TODO: Formel bestätigen und Quelle.
      muges(i) = 1/sqrt(det( ...
        Jinv_IK(R.I_qa,R.I_EE_Task)'*Jinv_IK(R.I_qa,R.I_EE_Task)));
    else % nicht-redundant
      % Determinante der Jacobi ist Kehrwert der Determinante der J.-Inversen
      muges(i) = abs(1/det(Jinv_IK(R.I_qa,:)));
    end
  end
end
% Schlechtester Wert der Manipulierbarkeit ist Kennzahl
% Nehme Kehrwert, da Manipulierbarkeit oft sehr groß ist und große Werte besser sind.
f_mani1 = 1/min(muges);
f_mani_norm = 2/pi*atan(f_mani1/10); % Normierung auf 0 bis 1; 70 ist 0.9.
fval = 1e3*f_mani_norm; % Normiert auf 0 bis 1e3
fval_debugtext = sprintf('Kleinste Manipulierbarkeit %1.3e (Kehrwert %1.3e).', 1/f_mani1, f_mani1);
