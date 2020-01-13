% Zielfunktion ("objective function") für Optimierung in der Maßsynthese
% basierend auf der Steifigkeit des Roboters.
% Die Steifigkeit wird in einen normierten Zielfunktionswert übersetzt
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
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
% fval_phys [1x1]
%   Physikalischer Wert, der dem Zielfunktionswert zugrunde liegt
%   Hier: Summe Schlechtester Wert für die (translatorische) Steifigkeit
%
% Quellen:
% [GoerguelueDed2019] A New Stiffness Performance Index: Volumetric
% Isotropy Index
% [Zhao2020] Modellierung und Maßsynthese serieller und paralleler Roboter
% hinsichtlich der strukturellen Steifigkeit (Masterarbeit)
% [Klimchik2011] Enhanced stiffness modeling of serial and parallel
% manipulators for robotic-based processing of high performance materials

% Masterarbeit Yuqi ZHAO, zhaoyuqi.nicolas@gmail.com, 2019-12
% Betreuer: Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [fval, fval_debugtext, debug_info, fval_phys] = cds_obj_stiffness(R, Q)
debug_info = {};


% Berechne Steifigkeit über Trajektorie
Vges = NaN(size(Q,1), 1);
% Berechne Steifigkeit für alle Punkte der Bahn
for i = 1:size(Q,1)
  % Kartesische Steifigkeitsmatrix (6x6)
  K_ges = R.stiffness(Q(i,:)');
  % Auswahl der translatorischen Submatrix (3x3), Normierung auf N/mm
  % (damit Zahlenwerte eher im Bereich 1 liegen)
  K_trans_norm = 1e-3*K_ges(1:3,1:3);
  % [GoerguelueDed2019], Gl. 21
  % Entspricht Volumen des Ellipsoids der Nachgiebigkeits-Matrix
  Vges(i,:) = 1/det(K_trans_norm);%prod(N_eig);
end
if any(isnan(Vges))
  warning('Hier stimmt etwas nicht');
end
% Schlechtester Wert des Volumens vom Ellipsoid ist Kennzahl
f_sti = max(Vges)^(1/3); % Umrechnung vom Volumen auf den Radius mit ^(1/3)
f_sti_norm = 2/pi*atan(f_sti); % Nomierung: 1e3 mm/N -> 0.5; 5e3 mm/N -> 0.87
fval = 1e3*f_sti_norm; % Normiert auf 0 bis 1e3
fval_debugtext = sprintf('Nachgiebigkeit %1.3e mm/N', f_sti);
% Entsprechung des physikalischen Wertes: Eine gleichförmige Nachgiebigkeit
% mit diesem Wert in alle drei Raumrichtungen (für Translation) würde eine
% Hyper-Kugel mit dem gleichen Volumen wie das Hyper-Ellipsoid erzeugen,
% das den aktuell schlechtesten Fall darstellt
fval_phys = 1e3 * f_sti; % Umrechnung in äquivalenten physikalischen Wert

debug_info = {sprintf('max. Nachgiebigkeit: %1.3fs', f_sti)};