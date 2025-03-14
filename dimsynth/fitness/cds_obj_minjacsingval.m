% Zielfunktion ("objective function") für Optimierung in der Maßsynthese
% basierend auf der kleinstem Singulärwert der Jacobi-Matrix des Roboters.
% Der Singulärwert wird in einen normierten Zielfunktionswert übersetzt
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
% f_msv1 [1x1]
%   Physikalischer Wert, der dem Zielfunktionswert zugrunde liegt
%   Hier: Kehrwert des minimaler Singulärwerts der Jacobi-Matrix (über die
%   Trajektorie). Kehrwert, da große Singulärwerte besser sind, hier aber
%   ein Minimierungsproblem formuliert wird.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-10
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval, fval_debugtext, debug_info, f_msv1] = cds_obj_minjacsingval(R, Set, Jinvges, Traj_0, Q)
debug_info = {};

% Berechne Singulärwerte über Trajektorie
msvges = NaN(length(Traj_0.t), 1);
if any(R.Type == [0 1]) % Seriell
  % Berechne Manipulierbarkeit für alle Punkte der Bahn
  for i = 1:length(Traj_0.t)
    J_3T3R = R.jacobia(Q(i,:)'); % analytisch wegen 3T2R; für alle anderen geht auch jacobig
    J_task = J_3T3R(Set.task.DoF,:);
    msvges(i,:) = min(svd(J_task));
  end
else % PKM
  % Berechne Manipulierbarkeit für alle Punkte der Bahn
  for i = 1:length(Traj_0.t)
    Jinv_IK = reshape(Jinvges(i,:), R.NJ, sum(R.I_EE));
    % Kleinster Singulärwert der Jacobi ist größter SW der inversen Jacobi
    msvges(i) = 1/max(svd(Jinv_IK(R.I_qa,:)));
    % Testen: Kleinster SW: min(svd(inv(Jinv_IK(R.I_qa,:))))
  end
end
% Schlechtester (kleinster) Wert des (kleinsten Jacobi-) Singulärwerts ist Kennzahl
% Nehme Kehrwert, da Wert nicht begrenzt ist und große Werte besser sind.
f_msv1 = 1/min(msvges);
f_msv_norm = 2/pi*atan(f_msv1/10); % Normierung auf 0 bis 1; 70 ist 0.9.
fval = 1e3*f_msv_norm; % Normiert auf 0 bis 1e3
fval_debugtext = sprintf('Kleinster (Jacobi-)Singulärwert %1.3e (Kehrwert %1.3e).', 1/f_msv1, f_msv1);
