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
%   TAU_reg
%     Regressormatrix für TAU
%   Wges
%     Alle Schnittkräfte (in allen Gelenken; bei PKM für alle Beinketten)
%     Indizes: Siehe SerRob/internforce_traj
%     (1: Zeit, 2:Kraft/Moment)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-10
% (C) Institut für Mechatronische Systeme, Universität Hannover

function output = cds_obj_dependencies(R, Traj_0, Set, Q, QD, QDD, Jinv_ges)
output = struct('content', 'cds_obj_dependencies');
if any(strcmp(Set.optimization.objective, {'energy', 'minactforce'})) ...
    || Set.optimization.desopt_link_yieldstrength && R.Type == 2
  if R.Type == 0 % Serieller Roboter
    % Antriebskräfte berechnen
    TAU = R.invdyn2_traj(Q, QD, QDD);
    % Regressormatrix für Antriebskräfte berechnen (falls
    % Entwurfsoptimierung durchgeführt wird)
    if Set.general.debug_calc || Set.optimization.use_desopt
      TAU_reg = R.invdynregmat_traj(Q, QD, QDD);
      if Set.general.debug_calc
        TAU_test = R.invdyn3_traj(TAU_reg);
      end
    end
  else % PKM
    % Trajektorie in Plattform-KS umrechnen
    XE = Traj_0.X;
    XED = Traj_0.XD;
    XEDD = Traj_0.XDD;
    % Antriebskräfte berechnen (Momente im Basis-KS, nicht x-Koord.)
    if Set.general.debug_calc || Set.optimization.use_desopt
      % Regressorform nur berechnen, falls später benötigt
      [Fa_red_traj, Fa_red_traj_reg] = R.invdyn2_actjoint_traj(Q, QD, QDD, XE, XED, XEDD, Jinv_ges);
    else
      Fa_red_traj = R.invdyn2_actjoint_traj(Q, QD, QDD, XE, XED, XEDD, Jinv_ges);
      Fa_red_traj_reg = NaN;
    end
    TAU = Fa_red_traj;
    TAU_reg = Fa_red_traj_reg;
    % Antriebskräfte noch auf andere Wege berechnen. Diese Rechenwege sind
    % aber mittlerweile in der ParRob-Funktion mit eingebunden. Dient nur
    % zur Kontrolle, ob die Rechenwege übereinstimmen.
    if Set.general.debug_calc
      [Fx_red_traj, Fx_red_traj_reg] = R.invdyn2_platform_traj(Q, QD, QDD, XE, XED, XEDD, Jinv_ges);
      % Teste Regressorform in Plattform-Koordinaten
      Fx_red_traj_test = R.invdyn3_platform_traj(Fx_red_traj_reg);
      test_Fx_red_traj = Fx_red_traj_test - Fx_red_traj;
      if any(abs(test_Fx_red_traj(:)) > 1e-6)
        error('Regressorform Plattform-Dynamik stimmt nicht. Fehler: max %1.2e', max(abs(test_Fx_red_traj(:))));
      end
      % Teste Regressorform in Antriebs-Koordinaten
      Fa_red_traj_test = R.invdyn3_actjoint_traj(Fa_red_traj_reg);
      test_Fa_red_traj = Fa_red_traj_test - Fa_red_traj;
      if any(abs(test_Fa_red_traj(:)) > 1e-6)
        error('Regressorform Antriebs-Dynamik stimmt nicht. Fehler: max %1.2e', max(abs(test_Fa_red_traj(:))));
      end
      % Teste Dynamik-Berechnung gegen symbolische Berechnung
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
      TAU_test = NaN(length(Traj_0.t), sum(R.I_qa));
      for i = 1:length(Traj_0.t)
        Jinv_qD_xD = reshape(Jinv_ges(i,:), R.NJ, sum(R.I_EE));
        Jinv_qaD_xD = Jinv_qD_xD(R.I_qa,:);
        % Jacobi-Matrix auf Winkelgeschwindigkeiten beziehen. Siehe ParRob/jacobi_qa_x
        if size(Jinv_qaD_xD,2) == 6
          T = [eye(3,3), zeros(3,3); zeros(3,3), euljac(XE(i,4:6)', R.phiconv_W_E)];
          Jinv_qaD_sD = Jinv_qaD_xD / T;
        else
          % Nehme an, dass keine räumliche Drehung vorliegt. TODO: Fall 3T2R
          % genauer prüfen, wenn Roboter verfügbar sind.
          Jinv_qaD_sD = Jinv_qaD_xD;
        end
        TAU_test(i,:) = (Jinv_qaD_sD') \ Fx_red_traj(i,:)';
      end
    end
  end
  % Prüfe Regressorform gegen direkte Berechnung (optional)
  if Set.general.debug_calc
    test_TAU = TAU - TAU_test;
    if any(abs(test_TAU(:)) > 1e-8)
      error('Regressorform stimmt nicht. Fehler: max %1.2e', max(abs(test_TAU(:))));
    end
  end
  output.TAU = TAU;
  output.TAU_reg = TAU_reg;
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
