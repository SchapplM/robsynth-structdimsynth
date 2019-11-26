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
%   Zeilenweise (inverse) Jacobi-Matrizen des Roboters (für PKM). Bezogen
%   auf vollständige Gelenkgeschwindigkeiten und Plattform-Geschw.
% JinvD_ges
%   Zeitableitung von Jinvges
% 
% Ausgabe:
% output
%   Struktur mit Diversen zu berechnenden Größen des Roboters im
%   Zeitverlauf. Felder:
%   TAU
%     Alle Antriebsmomente (in den aktiven Gelenken)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function output = cds_obj_dependencies(R, Traj_0, Set, Q, QD, QDD, Jinv_ges, JinvD_ges)
output = struct('content', 'cds_obj_dependencies');
if any(strcmp(Set.optimization.objective, {'energy', 'minactforce'}))
  if R.Type == 0 % Serieller Roboter
    % Antriebskräfte berechnen
    TAU = R.invdyn2_traj(Q, QD, QDD);
  else % PKM
    % Trajektorie in Plattform-KS umrechnen
    XE = Traj_0.X;
    XED = Traj_0.XD;
    XEDD = Traj_0.XDD;
    % Antriebskräfte berechnen
    Fx_red_traj = R.invdyn2_platform_traj(Q, QD, XE, XED, XEDD, Jinv_ges, JinvD_ges);
    % Teste Dynamik-Berechnung
    if all(R.DynPar.mges(R.NQJ_LEG_bc+1:end-1) == 0)
      % Bedingung für Zulässigkeit der symbolischen Form erfüllt. Teste.
      [XP,XPD,XPDD] = R.xE2xP_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD);
      Fx_red_traj_test = R.invdyn_platform_traj(Q, XP, XPD, XPDD);
      test_Fx = Fx_red_traj(:) - Fx_red_traj_test(:);
      if any(abs(test_Fx) > 1e-5)
        error('Dynamik symbolisch/numerisch stimmt nicht. Max. Fehler %1.3e', max(abs(test_Fx)));
      end
    end
    TAU = NaN(length(Traj_0.t), sum(R.I_qa));
    for i = 1:length(Traj_0.t)
      Jinv_IK = reshape(Jinv_ges(i,:), R.NJ, sum(R.I_EE));
      Jinv_IK_a = Jinv_IK(R.I_qa,:);
      TAU(i,:) = (Jinv_IK_a') \ Fx_red_traj(i,:)';
    end
  end
  output.TAU = TAU;
end