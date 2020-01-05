% Zielfunktion ("objective function") für Optimierung in der Maßsynthese
% basierend auf der Gesamtmasse des Roboters.
% Die Masse wird in einen normierten Zielfunktionswert übersetzt
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
%   Hier: Summe aller bewegten Massen in kg

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [fval, fval_debugtext, debug_info, fphys] = cds_obj_mass(R)
debug_info = {};

% Gesamtmasse berechnen
if R.Type == 0
  m_sum = sum(R.DynPar.mges(2:end));
  debug_info = {debug_info{:}; sprintf('masses: total %1.2fkg, Segments [%s] kg', ...
    m_sum, disp_array(R.DynPar.mges(2:end)', '%1.2f'))};
else
  m_sum = sum(R.DynPar.mges(1:end-1))*R.NLEG + R.DynPar.mges(end);
  debug_info = {debug_info{:}; sprintf('masses: total %1.2fkg, 1Leg %1.2fkg, Pf %1.2fkg', ...
    m_sum, sum(sum(R.DynPar.mges(1:end-1))), R.DynPar.mges(end))};
end
fval_debugtext = sprintf('Gesamtmasse %1.1f kg', m_sum);
f_mass_norm = 2/pi*atan((m_sum)/100); % Normierung auf 0 bis 1; 620 ist 0.9. TODO: Skalierung ändern
fval = 1e3*f_mass_norm; % Normiert auf 0 bis 1e3
fphys = m_sum;