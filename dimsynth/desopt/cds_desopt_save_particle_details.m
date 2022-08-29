% Speichere zusätzliche Informationen für jedes Partikel des DesOpt-PSO-Alg.
% Diese Funktion wird beim Verlassen der Fitness-Funktion aufgerufen
% 
% Eingabe:
% comptime
%   Rechenzeit in s nach Beginn des Fitnessfunktion-Aufrufs
% fval
%   Güte-Wert für aktuellen Parametersatz. Kann skalar oder vektoriell sein.
% pval
%   Vektor der Optimierungsvariablen für PSO
% physval
%   Physikalischer Wert für Zielfunktionswert(e) aus fval.
% fval_main, physval_main
%   Güte-Wert und physikalischer Wert für Zielfunktion der Maßsynthese
%   (Entspricht Ausgabe von cds_fitness bei Erfolg)
% option
%   Steuerungsparameter für das Verhalten der Funktion
%   output: Nur Ausgabe der gespeicherten persistenten Variable
%   reset: Zurücksetzen der persistenten Variable
%   <leer lassen>: Einfügen der Daten in die persistente Variable bei jedem
%     Aufruf
% PSO_Detail_Data_in
%   Eingabewert der persistenten Variable (z.B. gelesen aus Datei) zum
%   Überschreiben bestehender Werte
% 
% Ausgabe:
% PSO_Detail_Data_output
%   Struktur mit Detailsauswertung für die einzelnen PSO-Partikel
% 
% Siehe auch: cds_save_particle_details

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-10
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [PSO_Detail_Data_output, i_gen, i_ind] = cds_desopt_save_particle_details( ...
  comptime, fval, pval, physval, fval_main, physval_main, option, PSO_Detail_Data_in)
% Variable zum Speichern der Ergebnisse
persistent PSO_Detail_Data
if nargin == 0
  PSO_Detail_Data = [];
  return
end
if isnan(comptime) || any(isnan(fval))
  error('Rechenzeit darf nicht NaN sein');
end
i_gen = 0; i_ind = 0;
PSO_Detail_Data_output = [];
% Eingabe verarbeiten
if nargin < 7
  option = 'iter';
end
if nargin < 5 % Werte nicht belegt. NaN mit gespeicherter Dimension
  fval_main = NaN(size(PSO_Detail_Data.fval_main, 2), 1);
  physval_main = fval_main;
end
if strcmp(option, 'output')
  PSO_Detail_Data_output = PSO_Detail_Data;
  return
end
% Persistente Variable initialisieren. Eine zusätzliche Generation für Initial-
% population, eine für Berechnungen nach Ende der eigentlichen Optimierung.
if isempty(PSO_Detail_Data) || strcmp(option, 'reset')
  size_data = size(PSO_Detail_Data_in.comptime);
  PSO_Detail_Data = struct( ...
    'comptime', NaN(size_data), ...
    'fval', NaN(size_data), ...
    'physval', NaN(size_data), ...
    'fval_main', NaN(size_data(2), length(fval_main), size_data(1)), ...
    'physval_main', NaN(size_data(2), length(fval_main), size_data(1)), ...
    'pval', NaN(size_data(2), length(pval), size_data(1)));
  if strcmp(option, 'reset')
    return
  end
else
  size_data = size(PSO_Detail_Data.comptime);
end
%% Speichervariable belegen
% Suche erste freie Stelle in Ergebnis-Variable (erst Zeilen=Generationen,
% dann Spalten=Partikel)
data_transp = PSO_Detail_Data.comptime'; % Transp., da spaltenweise gesucht wird
k=find(isnan(data_transp(:)), 1, 'first'); % 1D-Index in Matrix
if isempty(k)
  % Aufruf der Fitness-Funktion nach Ende der Optimierung. Daher kein Platz
  % mehr in Variable. Ignoriere den Aufruf
  return
end
[i_ind, i_gen] = ind2sub(fliplr(size_data),k); % Umrechnung in 2D-Indizes.
% Eintragen der eingegebenen Daten für aktuelles PSO-Partikel
PSO_Detail_Data.comptime(i_gen,i_ind) = comptime; % Rechenzeit
PSO_Detail_Data.fval(i_gen,i_ind) = fval;
PSO_Detail_Data.physval(i_gen,i_ind) = physval; % Physikalische Werte zu fval
PSO_Detail_Data.pval(i_ind,:,i_gen) = pval; % Parametersatz zu fval
PSO_Detail_Data.fval_main(i_ind,:,i_gen) = fval_main; % vollständiger Vektor bei mehrkriteriell
PSO_Detail_Data.physval_main(i_ind,:,i_gen) = physval_main;
