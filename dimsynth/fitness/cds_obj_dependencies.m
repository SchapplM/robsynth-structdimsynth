% Berechne Abhängigkeiten für die Zielfunktionen (Auslagerung von
% Vorberechnungen)
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Q, QD, QDD
%   Gelenkpositionen und -geschwindigkeiten des Roboters (für PKM auch
%   passive Gelenke)
% Jinvges
%   Zeilenweise (inverse) Jacobi-Matrizen des Roboters (für PKM).
% 
% Ausgabe:
% output
%   Struktur mit Diversen zu berechnenden Größen des Roboters im
%   Zeitverlauf. Felder:
%   TAU
%     Alle Antriebsmomente (in den aktiven Gelenken)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function output = cds_obj_dependencies(R, Traj_0, Set, Q, QD, QDD, Jinvges)
output = struct('content', 'cds_obj_dependencies');
if any(strcmp(Set.optimization.objective, {'energy', 'minactforce'}))
  if R.Type == 0 % Serieller Roboter
    % Antriebskräfte berechnen
    TAU = R.invdyn2_traj(Q, QD, QDD);
  else % PKM
    % Trajektorie in Plattform-KS umrechnen
    [XP,XPD,XPDD] = R.xE2xP_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD);
    % Antriebskräfte berechnen
    Fx_red_traj = invdyn_platform_traj(R, Q, XP, XPD, XPDD);
    TAU = NaN(length(Traj_0.t), sum(R.I_qa));
    for i = 1:length(Traj_0.t)
      Jinv_IK = reshape(Jinvges(i,:), sum(R.I_EE), sum(R.I_qa));
      TAU(i,:) = (Jinv_IK') \ Fx_red_traj(i,:)';
    end
  end
  output.TAU = TAU;
end