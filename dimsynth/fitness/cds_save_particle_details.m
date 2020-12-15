% Speichere zusätzliche Informationen für jedes Partikel des PSO-Alg.
% Diese Funktion wird beim Verlassen der Fitness-Funktion aufgerufen
% 
% Eingabe:
% Set
%   Struktur mit Einstellungen für die Optimierung
% R
%   Matlab-Klasse des aktuell optimierten Roboters
% comptime
%   Rechenzeit in s nach Beginn des Fitnessfunktion-Aufrufs
% fval
%   Güte-Wert für aktuellen Parametersatz. Kann skalar oder vektoriell sein.
% pval
%   Vektor der Optimierungsvariablen für PSO
% physval
%   Physikalischer Wert für alle Zielfunktionswerte aus fval. Dient der
%   späteren vereinfachten Auswertung.
% constraint_obj_val
%   Enthält alle physikalischen Werte zu als Nebenbedingung möglichen Güte-
%   kriterien. Siehe cds_settings_defaults. Reihenfolge: 
%   1=Mass, 2=Energy, 3=Actforce, 4=Condition, 5=Stiffness;
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

function PSO_Detail_Data_output = cds_save_particle_details(Set, R, comptime, ...
  fval, pval, physval, constraint_obj_val, option)
if isnan(comptime) || any(isnan(fval))
  error('Rechenzeit darf nicht NaN sein');
end
% Variable zum Speichern der Ergebnisse
persistent PSO_Detail_Data
PSO_Detail_Data_output = [];
% Eingabe verarbeiten
if nargin < 8
  option = 'iter';
end
if strcmp(option, 'output')
  PSO_Detail_Data_output = PSO_Detail_Data;
  return
end
if R.Type == 0, q0 = R.qref;
else,           q0 = cat(1,R.Leg.qref); end

% Persistente Variable initialisieren. Eine zusätzliche Generation für Initial-
% population, eine für Berechnungen nach Ende der eigentlichen Optimierung.
size_data = [Set.optimization.MaxIter+2, Set.optimization.NumIndividuals];
if isempty(PSO_Detail_Data) || strcmp(option, 'reset')
  PSO_Detail_Data = struct( ...
    'comptime', NaN(size_data), ...
    'fval_mean', NaN(size_data), ...
    'fval', NaN(size_data(2), length(fval), size_data(1)), ...
    'pval', NaN(size_data(2), length(pval), size_data(1)), ...
    'physval', NaN(size_data(2), length(fval), size_data(1)), ...
    'constraint_obj_val', NaN(size_data(2), length(constraint_obj_val), size_data(1)), ...
    'q0_ik', NaN(size_data(2), length(q0), size_data(1)) );
  if strcmp(option, 'reset')
    return
  end
end
%% Speichervariable belegen
% Suche erste freie Stelle in Ergebnis-Variable (erst Zeilen=Generationen,
% dann Spalten=Partikel)
data_transp = PSO_Detail_Data.comptime'; % Transp., da spaltenweise gesucht wird
k=find(isnan(data_transp(:)), 1, 'first'); % 1D-Index in Matrix
[j,i] = ind2sub(fliplr(size_data),k); % Umrechnung in 2D-Indizes. i=Generation, j=Individuum
% Eintragen der eingegebenen Daten für aktuelles PSO-Partikel
PSO_Detail_Data.comptime(i,j) = comptime; % Rechenzeit
PSO_Detail_Data.fval_mean(i,j) = mean(fval); % Mittelwert für mehrkriterielle Optimierung
PSO_Detail_Data.fval(j,:,i) = fval; % vollständiger Vektor bei mehrkriteriell
PSO_Detail_Data.pval(j,:,i) = pval; % Parametersatz zu fval
PSO_Detail_Data.physval(j,:,i) = physval; % physikalische Werte zu fval (ohne Sättigung/Normierung)
PSO_Detail_Data.constraint_obj_val(j,:,i) = constraint_obj_val; % physikalische Werte zu Set.optimization.constraint_obj
PSO_Detail_Data.q0_ik(j,:,i) = q0; % IK-Anfangswerte
