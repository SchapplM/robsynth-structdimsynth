% OutputFcn für den Multi-Objective Particle-Swarm-Algorithmus (MOPSO). 
% Speichere den aktuellen Zwischenstand.
% 
% Die Funktion kann mit folgendem Befehl im MOPSO-Algorithmus benutzt werden:
% mopso_outputfun = @(MS)cds_save_all_results_mopso(MS,Set,Structure);
% options.OutputFcn = {mopso_outputfun};
% 
% Eingabe:
% MOPSO_struct
%   Standard-Eingabe für MOPSO-Output-Fcn. Siehe MOPSO.m, OutputFcn
%   Felder: POS, POS_fit, REP
% Set
%   Eingabe zusätzlich zu MOPSO-Standard.
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Structure
%   Eingabe zusätzlich zu MOPSO-Standard.
%   Eigenschaften der Roboterstruktur (aus cds_gen_robot_list.m)
% 
% Ausgabe:
% stop
%   Standard-Ausgabe für MOPSO-Output-Fcn
% 
% Manueller Aufruf: `cds_save_all_results_mopso([], Set, Structure)`
% (kann beim Debuggen benutzt werden um vor einem erzwungenen Abbruch zu
% speichern)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function stop = cds_save_all_results_mopso(MOPSO_struct, Set, Structure)

stop = false;
if ~isempty(MOPSO_struct)
  if any(isinf(MOPSO_struct.POS_fit(:)))
    % Der Wert inf wird in cds_fitness als Marker benutzt, um die Optimierung
    % abzubrechen. Das kann hier an den PSO übergeben werden, da in MOPSO
    % keine Abbruchbedingungen definierbar sind.
    stop = true;
  end
  REP = MOPSO_struct.REP;
else
  % Dummy-Variable zum Speichern auch ohne Aufruf aus MOPSO.m
  REP = [];
end
% Detail-Ergebnisse extrahieren (persistente Variable in Funktion)
PSO_Detail_Data = cds_save_particle_details(Set, [], 0, 0, NaN, NaN, NaN, NaN, 'output');
% Nummer der aktuellen Generation herausfinden
currgen = find(any(~isnan(PSO_Detail_Data.comptime),2),1,'last')-1;
filename = sprintf('MOPSO_Gen%02d_AllInd.mat', currgen);

% Pareto-Front bestimmen (bei manuellem Aufruf, s.o.)
if isempty(REP)
  % Initialisieren: Entspricht p_val_pareto und fval_pareto
  REP = struct('pos', [], 'pos_fit', []);
  for i = 1:currgen
    REP.pos = [REP.pos; PSO_Detail_Data.pval(:,:,i)];
    REP.pos_fit = [REP.pos_fit; PSO_Detail_Data.fval(:,:,i)];
  end
  % Reduziere die Daten, damit weniger gespeichert wird
  Idom = pareto_dominance(REP.pos_fit);
  REP.pos = REP.pos(~Idom,:);
  REP.pos_fit = REP.pos_fit(~Idom,:);
end
% Aktuelle Pareto-Front und Details abspeichern
resdir = fullfile(Set.optimization.resdir, Set.optimization.optname, ...
  'tmp', sprintf('%d_%s', Structure.Number, Structure.Name));
save(fullfile(resdir, filename), 'PSO_Detail_Data', 'REP');
% Datei der vorherigen Iteration löschen (wird nicht mehr benötigt)
if currgen > 1
  filename_previous = sprintf('MOPSO_Gen%02d_AllInd.mat', currgen-1);
  delete(fullfile(resdir, filename_previous));
end

cds_log(1,sprintf('[output] Zwischenergebnisse gespeichert: %s', ...
  fullfile(resdir, filename)));
