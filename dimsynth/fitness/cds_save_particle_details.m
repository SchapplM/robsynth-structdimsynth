% Speichere zusätzliche Informationen für jedes Partikel des PSO-Alg.
% Diese Funktion wird beim Verlassen der Fitness-Funktion aufgerufen
% 
% Eingabe:
% Set
%   Struktur mit Einstellungen für die Optimierung
% comptime
%   Rechenzeit in s nach Beginn des Fitnessfunktion-Aufrufs
% fval
%   Güte-Wert für aktuellen Parametersatz
% Jcond
%   Schlechteste Konditionszahl des Roboters. Siehe cds_obj_condition.
% option
%   Steuerungsparameter für das Verhalten der Funktion
%   output: Nur Ausgabe der gespeicherten persistenten Variable
%   reset: Zurücksetzen der persistenten Variable
%   <leer lassen>: Einfügen der Daten in die persistente Variable bei jedem
%     Aufruf
% 
% Ausgabe:
% PSO_Detail_Data_output
%   Struktur mit Detailsauswertung für die einzelnen PSO-Partikel

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-01
% (C) Institut für Mechatronische Systeme, Universität Hannover

function PSO_Detail_Data_output = cds_save_particle_details(Set, comptime, fval, Jcond, option)
if isnan(comptime) || isnan(fval)
  error('Rechenzeit darf nicht NaN sein');
end
% Variable zum Speichern der Ergebnisse
persistent PSO_Detail_Data
PSO_Detail_Data_output = [];
% Eingabe verarbeiten
if nargin < 5
  option = 'iter';
end
if strcmp(option, 'output')
  PSO_Detail_Data_output = PSO_Detail_Data;
  return
end
% Persistente Variable belegen
size_data = [Set.optimization.MaxIter+1, Set.optimization.NumIndividuals];
if isempty(PSO_Detail_Data) || strcmp(option, 'reset')
  PSO_Detail_Data = struct('comptime', NaN(size_data), 'fval', NaN(size_data), 'Jcond', NaN(size_data));
  if strcmp(option, 'reset')
    return
  end
end
%% Speichervariable belegen
% Suche erste freie Stelle in Ergebnis-Variable (erst Zeilen=Generationen,
% dann Spalten=Partikel)
data_transp = PSO_Detail_Data.comptime'; % Transp., da spaltenweise gesucht wird
k=find(isnan(data_transp(:)), 1, 'first'); % 1D-Index in Matrix
[j,i] = ind2sub(fliplr(size_data),k); % Umrechnung in 2D-Indizes
% Eintragen der eingegebenen Daten für aktuelles PSO-Partikel
PSO_Detail_Data.comptime(i,j) = comptime;
PSO_Detail_Data.fval(i,j) = fval;
PSO_Detail_Data.Jcond(i,j) = Jcond;