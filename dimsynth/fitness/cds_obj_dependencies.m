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
%   auf vollständige Gelenkgeschwindigkeiten und Plattform-Geschw. (in
%   x-Koordinaten, nicht: Winkelgeschwindigkeit)
% JinvD_ges
%   Zeitableitung von Jinvges
% 
% Ausgabe:
% output
%   Struktur mit Diversen zu berechnenden Größen des Roboters im
%   Zeitverlauf. Felder:
%   TAU
%     Alle Antriebsmomente (in den aktiven Gelenken)
%   Wges
%     Alle Schnittkräfte (in allen Gelenken; bei PKM für alle Beinketten)
%     Indizes: Siehe SerRob/internforce_traj
%     (1: Zeit, 2:Kraft/Moment)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function output = cds_obj_dependencies(R, Traj_0, Set, Q, QD, QDD, Jinv_ges, JinvD_ges)
output = struct('content', 'cds_obj_dependencies');
if any(strcmp(Set.optimization.objective, {'energy', 'minactforce'})) ...
    || Set.optimization.desopt_link_yieldstrength && R.Type == 2
  if R.Type == 0 % Serieller Roboter
    % Antriebskräfte berechnen
    TAU = R.invdyn2_traj(Q, QD, QDD);
  else % PKM
    % Trajektorie in Plattform-KS umrechnen
    XE = Traj_0.X;
    XED = Traj_0.XD;
    XEDD = Traj_0.XDD;
    % Antriebskräfte berechnen (Momente im Basis-KS, nicht x-Koord.)
    Fx_red_traj = R.invdyn2_platform_traj(Q, QD, XE, XED, XEDD, Jinv_ges, JinvD_ges);
    % Teste Dynamik-Berechnung
    if all(R.DynPar.mges(R.NQJ_LEG_bc+1:end-1) == 0) && ~isempty(R.DynPar.mpv_sym)
      % Bedingung für Zulässigkeit der symbolischen Form erfüllt. Teste.
      [XP,XPD,XPDD] = R.xE2xP_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD);
      Fx_red_traj_test = R.invdyn_platform_traj(Q, XP, XPD, XPDD);
      test_Fx_abs = Fx_red_traj(:) - Fx_red_traj_test(:);
      % TODO: Test sollte optional mit Debug-Schalter sein
      if any(abs(test_Fx_abs) > 1e-5) && ~any(abs(Jinv_ges(:))<1e6) % Nur Fehler erkennbar, wenn Jacobi nicht Singulär
        figure(999);clf; tt = {'fx','fy','fz','mx','my','mz'};
        for ii = 1:6
          subplot(4,2,ii);hold on; grid on; plot(Traj_0.t, Fx_red_traj_test(:,ii)); plot(Traj_0.t, Fx_red_traj(:,ii), '--')
          ylabel(tt{ii});
        end
        legend({'sym', 'num'});
        subplot(4,2,7);plot(Traj_0.t, Traj_0.X(:,1:3));ylabel('x pos'); grid on;
        subplot(4,2,8);plot(Traj_0.t, Traj_0.X(:,4:6));ylabel('x ori'); grid on;
        linkxaxes
        error('Dynamik symbolisch/numerisch stimmt nicht. Max. Fehler %1.3e', max(abs(test_Fx_abs)));
      end
    end
    TAU = NaN(length(Traj_0.t), sum(R.I_qa));
    for i = 1:length(Traj_0.t)
      Jinv_qD_xD = reshape(Jinv_ges(i,:), R.NJ, sum(R.I_EE));
      Jinv_qaD_xD = Jinv_qD_xD(R.I_qa,:);
      % Jacobi-Matrix auf Winkelgeschwindigkeiten beziehen. Siehe ParRob/jacobi_qa_x
      if size(Jinv_qaD_xD,2) == 6
        T = [eye(3,3), zeros(3,3); zeros(3,3), euljac(XE(i,4:6)', R.phiconv_W_E)];
        Jinv_qD_sD = Jinv_qaD_xD / T;
      end
      TAU(i,:) = (Jinv_qD_sD') \ Fx_red_traj(i,:)';
    end
  end
  output.TAU = TAU;
end

if Set.optimization.desopt_link_yieldstrength
  % Berechne die Schnittkräfte in allen Segmenten
  if R.Type == 0 % Seriell
    Wges = R.internforce_traj(Q, QD, QDD);
  else % PKM
    Wges = R.internforce_traj(Q, QD, QDD, TAU);
  end
  output.Wges = Wges;
end