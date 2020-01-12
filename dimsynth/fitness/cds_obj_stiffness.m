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
  K_ges = R.stiffness(Q(i,:)');
  K_trans_norm = 1e-3*K_ges(1:3,1:3);
%   N_trans_norm = 1e3*inv(K_trans); %#ok<MINV>
%   N_eig = eig(N_trans);
%   if R.Type == 0  %Serial Robot
%     N_norm = log(N_eig/1e-10+1);
%   else %Parallel Robot
%     N_norm = log(N_eig/1e-9+1);
%   end
  % [GoerguelueDed2019], Gl. 21
  % Volumen des Ellipsoids der Nachgiebigkeits-Matrix
  Vges(i,:) = 1/det(K_trans_norm);%prod(N_eig);
end

% Schlechtester Wert des Volumens vom Ellipsoid ist Kennzahl
f_sti = max(Vges);
f_sti_norm = 2/pi*atan(f_sti/1000); % Nomierung auf 0 bis 1;
fval = 1e3*f_sti_norm; % Normiert auf 0 bis 1e3
fval_debugtext = sprintf('Steifigkeit %1.2f', f_sti);
% Entsprechung des physikalischen Wertes: Eine gleichförmige Nachgiebigkeit
% mit diesem Wert in alle drei Raumrichtungen (für Translation) würde eine
% Hyper-Kugel mit dem gleichen Volumen wie das Hyper-Ellipsoid erzeugen,
% das den aktuell schlechtesten Fall darstellt
fval_phys = 1e3 * f_sti^(1/3); % TODO: Ist das wirklich ein physikalischer Wert?