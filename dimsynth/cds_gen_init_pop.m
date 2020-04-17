% Anfangspopulation der zu optimierenden Parameter bestimmen
% 
% Eingabe:
% NumIndividuals
%   Anzahl Parameter für jede Generation
% nvars
%   Anzahl der Parameter
% varlim [nvars x 2]
%   Grenzen für alle Parameter (erste Spalte Untergrenze, zweite Obergrenze)
% varnames {nvars x 1} cell
%   Namen aller Optimierungsparameter.
% vartypes [nvars x 1]
%   Kodierung des Parametertyps als Zahl. Siehe cds_dimsynth_robot
%   0=Skalierung, 1=pkin, 2=Basis-Position, ...
% 
% Ausgabe:
% InitPop [NumIndividuals x nvars]
%   Anfangspopulation für PSO-Optimierung

% Junnan Li, Hiwi bei Moritz Schappler
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-01
% (C) Institut für Mechatronische Systeme, Universität Hannover


function InitPop = cds_gen_init_pop(NumIndividuals,nvars,varlim,varnames,vartypes)

% Zufällige Werte für alle Optimierungsparameter (normiert mit varlim)
InitPop = repmat(varlim(:,1)', NumIndividuals,1) + rand(NumIndividuals, nvars) .* ...
                        repmat(varlim(:,2)'-varlim(:,1)',NumIndividuals,1);

% Alle PSO-Parameter durchgehen
for i = 1:nvars
  if vartypes(i) ~= 1
    % Nur Betrachtung von Kinematikparametern der Beinkette.
    % Alle anderen werden auf obige Zufallswerte gesetzt
    continue % nicht pkin parameter
  end
  if contains(varnames(i),{'theta'}) % theta parameter
    % Setze nur auf 0 oder pi/2. Das verspricht eine bessere Lösbarkeit der
    % Kinematik
    InitPop(:,i) = pi/2 * round(rand(NumIndividuals,1)); 
    continue
  end
  if contains(varnames(i),{'alpha'}) % alpha parameter
    % nur 0 oder pi/2
    InitPop(:,i) = pi/2 * round(rand(NumIndividuals,1)); 
    continue
  end
end