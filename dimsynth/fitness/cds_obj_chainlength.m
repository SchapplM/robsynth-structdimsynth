% Zielfunktion ("objective function") für Optimierung in der Maßsynthese
% basierend auf der Länge der kinematischen Kette(n) des Roboters.
% Die kinematische Länge wird in einen normierten Zielfunktionswert übersetzt
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% 
% Ausgabe:
% fval [1x1]
%   Zielfunktionswert, der im PSO-Algorithmus minimiert wird
% fval_debugtext [char]
%   Zeile mit Hinweistext, der bei PSO nach Fitness-Berechnung ausgegeben wird
% debug_info [cell]
%   Zusatz-Informationen, die im Debug-Bild des Roboters angezeigt werden
% fphys [1x1]
%   Physikalischer Wert, der dem Zielfunktionswert zugrunde liegt
%   Hier: Länge aller Robotersegmente in m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval, fval_debugtext, debug_info, fphys] = cds_obj_chainlength(R)
debug_info = {};

% Gesamtlänge berechnen. Benutze den gleichen Ansatz wie die SerRob/reach.
% Benutze die vorab in der Fitness-Funktion eingestellten Grenzen qlim.
% Mögliche Alternative: Aus Gelenkkoordinaten Q der Schubgelenke den
% Bewegungsbereich und damit die nötige Länge selbst ableiten.
if R.Type == 0
  lref = R.reach();
else
  % Symmetrische PKM
  lref = 0;
  for i = 1:R.NLEG
    lref = lref + R.Leg(i).reach();
  end
end

fval_debugtext = sprintf('Gesamtlänge %1.1f mm.', 1e3*lref);
f_lref_norm = 2/pi*atan(lref*6); % Normierung auf 0 bis 1; 1m ist 0.9.
fval = 1e3*f_lref_norm; % Normiert auf 0 bis 1e3
fphys = lref;