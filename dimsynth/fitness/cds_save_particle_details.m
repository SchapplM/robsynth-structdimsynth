% Speichere zusätzliche Informationen für jedes Partikel des PSO-Alg.
% Diese Funktion wird beim Verlassen der Fitness-Funktion aufgerufen
% 
% Eingabe:
% comptime
%   Rechenzeit in s nach Beginn des Fitnessfunktion-Aufrufs
% fval
%   Güte-Wert für aktuellen Parametersatz

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-01
% (C) Institut für Mechatronische Systeme, Universität Hannover

function cds_save_particle_details(comptime, fval)
global PSO_Detail_Data
% Suche erste freie Stelle in Ergebnis-Variable
data_transp = PSO_Detail_Data.comptime'; % Transp., da spaltenweise gesucht wird
k=find(isnan(data_transp(:)), 1, 'first'); % 1D-Index in Matrix
[j,i] = ind2sub(size(PSO_Detail_Data.comptime),k); % Umrechnung in 2D-Indizes
PSO_Detail_Data.comptime(i,j) = comptime;
PSO_Detail_Data.fval(i,j) = fval;