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
% desopt_pval
%   Enthält Parameter aus der Entwurfsoptimierung (innere Optimierungskaskade)
%   Reihenfolge, siehe cds_dimsynth_robot
% option
%   Steuerungsparameter für das Verhalten der Funktion
%   output: Nur Ausgabe der gespeicherten persistenten Variable
%   reset: Zurücksetzen der persistenten Variable
%   overwrite: Überschreiben der persistenten Variable aus Eingabe
%   <leer lassen>: Einfügen der Daten in die persistente Variable bei jedem
%     Aufruf
% PSO_Detail_Data_in
%   Eingabewert der persistenten Variable (z.B. gelesen aus Datei) zum
%   Überschreiben bestehender Werte
% 
% Ausgabe:
% PSO_Detail_Data_output
%   Struktur mit Detailsauswertung für die einzelnen PSO-Partikel

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-01
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [PSO_Detail_Data_output, i_gen, i_ind] = cds_save_particle_details( ...
  Set, R, comptime, fval, pval, physval, constraint_obj_val, desopt_pval, option, PSO_Detail_Data_in)
assert(~isnan(comptime), 'Rechenzeit darf nicht NaN sein');
assert(~any(isnan(fval)), 'fval darf nicht NaN sein');

i_gen = 0; i_ind = 0;
% Variable zum Speichern der Ergebnisse
persistent PSO_Detail_Data
PSO_Detail_Data_output = [];
% Eingabe verarbeiten
if nargin < 9
  option = 'iter';
end
if strcmp(option, 'overwrite')
  PSO_Detail_Data = PSO_Detail_Data_in;
  PSO_Detail_Data_output = PSO_Detail_Data;
  if nargout > 1 && ~isempty(PSO_Detail_Data)
    % Bestimme auch die aktuelle Generation/Indiviuums-Nr der Optimierung
    [i_gen, i_ind] = cds_load_particle_details(PSO_Detail_Data, ...
      NaN(size(PSO_Detail_Data.fval,2),1));
  end
  return
end
if strcmp(option, 'output')
  PSO_Detail_Data_output = PSO_Detail_Data;
  if nargout > 1 && ~isempty(PSO_Detail_Data)
    [i_gen, i_ind] = cds_load_particle_details(PSO_Detail_Data, ...
      NaN(size(PSO_Detail_Data.fval,2),1));
  end
  return
end
if R.Type == 0, q0 = R.qref;
else,           q0 = cat(1,R.Leg.qref); end
if R.Type == 0, qlim = R.qlim;
else,           qlim = cat(1,R.Leg(:).qlim); end
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
    'desopt_pval', NaN(size_data(2), length(desopt_pval), size_data(1)), ...
    'constraint_obj_val', NaN(size_data(2), length(constraint_obj_val), size_data(1)), ...
    'q_min', NaN(size_data(2), length(q0), size_data(1)), ...
    'q_max', NaN(size_data(2), length(q0), size_data(1)), ...
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
[i_ind, i_gen] = ind2sub(fliplr(size_data),k); % Umrechnung in 2D-Indizes.
% Eintragen der eingegebenen Daten für aktuelles PSO-Partikel
PSO_Detail_Data.comptime(i_gen,i_ind) = comptime; % Rechenzeit
PSO_Detail_Data.fval_mean(i_gen,i_ind) = mean(fval); % Mittelwert für mehrkriterielle Optimierung
PSO_Detail_Data.fval(i_ind,:,i_gen) = fval; % vollständiger Vektor bei mehrkriteriell
PSO_Detail_Data.pval(i_ind,:,i_gen) = pval; % Parametersatz zu fval
PSO_Detail_Data.physval(i_ind,:,i_gen) = physval; % physikalische Werte zu fval (ohne Sättigung/Normierung)
PSO_Detail_Data.desopt_pval(i_ind,:,i_gen) = desopt_pval; % Ergebnisse der Entwurfsoptimierung
PSO_Detail_Data.constraint_obj_val(i_ind,:,i_gen) = constraint_obj_val; % physikalische Werte zu Set.optimization.constraint_obj
PSO_Detail_Data.q_min(i_ind,:,i_gen) = qlim(:,1); % untere Grenze der Gelenkkoord.
PSO_Detail_Data.q_max(i_ind,:,i_gen) = qlim(:,2); % obere Grenze
PSO_Detail_Data.q0_ik(i_ind,:,i_gen) = q0; % IK-Anfangswerte
