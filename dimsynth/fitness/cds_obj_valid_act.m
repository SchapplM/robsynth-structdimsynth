% Zielfunktion ("objective function") für Optimierung in der Maßsynthese
% basierend auf der Gültigkeit der gewählten Aktuierung des (Parallel-)Roboters.
% Die Korrektheit der Aktuierung wird über den Laufgrad (eng. "Mobility")
% des Mechanismus in der Zielfunktion dargestellt.
% Diese Zielfunktion dient hauptsächlich der Struktursynthese, da keine
% kontinuierliche Bewertung der Strukturen möglich ist.
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Jinvges
%   Zeilenweise (inverse) Jacobi-Matrizen des Roboters (für PKM).
% 
% Ausgabe:
% fval [1x1]
%   Zielfunktionswert, der im PSO-Algorithmus minimiert wird. Zustände:
%   10: Der Roboter hat volle Freiheitsgrade
%   100*RD: Kodierung des Rangdefizits ("RD") der Jacobi-Matrix
% fval_debugtext [char]
%   Zeile mit Hinweistext, der bei PSO nach Fitness-Berechnung ausgegeben wird
% debug_info [cell]
%   Zusatz-Informationen, die im Debug-Bild des Roboters angezeigt werden

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [fval, fval_debugtext, debug_info] = cds_obj_valid_act(R, Set, Jinvges)
debug_info = {};

if R.Type == 0
  error('Diese Funktion ergibt nur für parallele Roboter Sinn.');
end

% Zielfunktion ist der Rang. Bei vollem Rang "funktioniert" der Roboter
% und der entsprechende Schwellwert wird unterschritten
% Berechne Rang der Jacobi nur für ersten Bahnpunkt. Annahme: Keine
% Singularität, da Trajektorie und IK lösbar ist.
n_qa = sum(R.I_qa);
for i = 1
  % Vollständige inverse Jacobi-Matrix aus Ergebnissen der Traj.-IK holen
  Jinv_IK = reshape(Jinvges(i,:), R.NJ, sum(R.I_EE));
  % Reduktion auf aktive Gelenke (vorher auch passive Gelenke enthalten)
  Jinv_xred = Jinv_IK(R.I_qa,:);
  % Inverse Jacobi-Matrix nochmal neu aus vollständiger Matrix aufbauen.
  % TODO: Prüfen, ob dieser Schritt noch notwendig ist.
  Jinv_3T3R = zeros(6, n_qa);
  Jinv_3T3R(R.I_EE,:) = Jinv_xred;
  Jinv_task = Jinv_3T3R(Set.structures.DoF,:);
  % Rangprüfung. Sehr schlecht konditionierte Matrizen sollen auch als
  % Rangverlust gekennzeichnet werden.
  % Mit Toleranz 1e-4 werden Matrizen mit Kond. 1e7 teilweise noch als
  % voller Rang gewertet. Schlechtere Matrizen werden als Rangverlust
  % gewertet.
  tol = 5e10*max(size(Jinv_task)) * eps(norm(Jinv_task)); % ca. 1e-4 bei "normaler" Matrix mit Werten ungefähr 0 bis 1
  rankJ = rank(Jinv_task, tol);
end
RD = sum(R.I_EE) - rankJ; % Rangdefizit
if RD == 0
  fval = 10; % Reicht zum aufhören
else
  fval = 100*RD; % Kodiere Rangdefizit in Zielfunktion
end
fval_debugtext = sprintf('Rangdefizit %d (Konditionszahl %e, Determinante %e).', ...
  RD, cond(Jinv_task), det(Jinv_task));