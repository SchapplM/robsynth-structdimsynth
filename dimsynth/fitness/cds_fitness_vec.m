% Gütefunktion für Maßsynthese von Robotern (allgemein); vektorisierte Form
% 
% Eingabe:
% R
%   Matlab-Klasse für Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus
% Traj_W
%   Trajektorie (bezogen auf Welt-KS)
% Structure
%   Eigenschaften der Roboterstruktur
% P [Nf x Np]
%   Matrix mit Vektoren der Optimierungsvariablen für PSO
%   Zeilen: Nf verschiedene zu berechnende Parametervektoren
%   Spalten: Np Einträge des Parametervektors
% 
% Ausgabe:
% fval_vec
%   Fitness-Werte für die Einträge der Parameter-Matrix P.
%   Zeilen entsprechend zu P. Spalten entsprechend zu cds_fitness.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function fval_vec = cds_fitness_vec(R, Set, Traj_W, Structure, P)

fval_vec = NaN(size(P,1), length(Set.optimization.objective));
for i = 1:size(P,1)
  fval_vec(i,:) = cds_fitness(R, Set, Traj_W, Structure, P(i,:)');
end
