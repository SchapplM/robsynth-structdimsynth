% Zielfunktion ("objective function") für Optimierung in der Maßsynthese
% für MRK-Kennzahl (Platzhalter).
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Structure
%   Eigenschaften der Roboterstruktur
% Traj_0
%   Endeffektor-Trajektorie (bezogen auf Basis-KS)
% Q
%   Gelenkwinkel-Trajektorie
% JinvE_ges
%   Zeilenweise (inverse) Jacobi-Matrizen des Roboters (für PKM). Bezogen
%   auf vollständige Gelenkgeschwindigkeiten und EE-Geschw.
% JP
%   Gelenkpositionen aller Gelenke des Roboters
%   (Zeilen: Zeitschritte der Trajektorie)
% 
% Ausgabe:
% fval [1x1]
%   Zielfunktionswert, der im PSO-Algorithmus minimiert wird. Benutze den
% fval_debugtext [char]
%   Zeile mit Hinweistext, der bei PSO nach Fitness-Berechnung ausgegeben wird
% debug_info [cell]
%   Zusatz-Informationen, die im Debug-Bild des Roboters angezeigt werden
% f_mrk [1x1]
%   Physikalischer Wert, der dem Zielfunktionswert zugrunde liegt
% 
% ToDos:
% * Kraftangriff senkrecht zur Stabachse ist bei Ellenbogenkontakt eine
%   stark einschränkende Annahme

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval, fval_debugtext, debug_info, f_mrk] = cds_obj_mrk2(R, Set, Structure, Traj_0, Q, JinvE_ges, JP)

% Debug-Code:
% if Set.general.matfile_verbosity > 2 % Debug
%   save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_obj_mrk2.mat'));
% end
% clear
% clc
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_obj_mrk2.mat'));

%% Initialisierung
% Platzhalter:
debug_info = '';
f_mrk = 0;
f_norm = 140; % Referenzkraft 140N nach MRK-Richtlinie
debug_plot = false;

% Umrechnung der Jacobi-Matrix auf Plattform-Koordinaten (Dynamik ist nicht
% in EE-Koordinaten definiert, da die Trafo sich ändern kann).
XE = Traj_0.X; % EE-Pose
XP = R.xE2xP_traj(XE);
JinvP_ges = R.jacobi_q_xE_2_jacobi_q_xP_traj(JinvE_ges, XE, XP);
%% Bestimme Antriebskräfte aus beispielhafter Kontaktkraft für Segmente
% Variablen zur Speicherung des schlechtesten Falls
tau_a_min = inf;
i_traj_min = NaN;
i_leg_min = NaN;
i_link_min = NaN;
i_force_min = NaN;

n_reflength = 3; % Anzahl der Prüfpunkte über jedes Segment (Anfang, Mitte, Ende)
n_force = 360/15; % Alle 15 Grad den Kraftangriff prüfen
for i_traj = 1:length(Traj_0.t) % Alle Zeitschritte durchgehen
  q = Q(i_traj,:)';
%   i_pt = 1; % Index für Gelenkpunkte JP (könnte prinzipiell anstatt von fkine benutzt werden)
%   jp_i = reshape(JP(i_traj,:), 3, size(JP,2)/3);
  for i_leg = 1:R.NLEG % Alle Beinketten durchgehen
    T_0_L0 = R.Leg(i_leg).T_W_0; % Basis-Trafo der Beinkette
    q_leg = q(R.I1J_LEG(i_leg):R.I2J_LEG(i_leg)); % Gelenkwinkel der BK
    Tc_Leg_i = R.Leg(i_leg).fkine(q_leg); % Beinketten-Trafos im BK-Basis-KS
%     i_pt = i_pt + 1; % Basis-KS überspringen
    for i_link = 1:R.Leg(i_leg).NJ
%       i_pt = i_pt + 1;
      if i_link == 1, continue; end % Oberarm ignorieren (Erkennung unproblematisch)
      if i_link == R.Leg(i_leg).NJ, continue; end % EE-KS (kein nachfolgendes Segment)
      % Vektor zum nächsten Gelenk (Gelenkpunkt liegt bei i_link+1)
      link2_L0 = Tc_Leg_i(1:3,4,i_link+2) - Tc_Leg_i(1:3,4,i_link+1);
      if norm(link2_L0) < 1e-6
        % Das vom Gelenk bewegte Segment hat keine Ausdehnung. Vermutlich
        % Kugelgelenk oder Kardangelenk.
        continue
      end
      if debug_plot
        fhdl = change_current_figure(3); clf; hold all; %#ok<UNRCH> 
        view(3); axis auto; hold on; grid on;
        xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
        if R.Type == 0 % Seriell
          s_plot = struct( 'ks', [], 'straight', 1, 'mode', 1, 'nojoints', 1);
          R.plot( Q(i_reflength,:)', s_plot);
        else % PKM
          s_plot = struct( 'ks_legs', [], 'ks_platform', [], ...
            'straight', 1, 'mode', 1, 'nojoints', 1);
          R.plot( q, Traj_0.X(i_traj,:)', s_plot);
        end
        % Betroffenes Segment
        joint_W = R.T_W_0 * T_0_L0 * Tc_Leg_i(:,:,i_link+1) * [0;0;0;1];
        link2_W = R.T_W_0 * T_0_L0 * [link2_L0; 0];
        plot3(joint_W(1)+[0;link2_W(1)], joint_W(2)+[0;link2_W(2)], ...
          joint_W(3)+[0;link2_W(3)], 'c--', 'LineWidth', 5);
      end
      % Bestimme Jacobi-Matrix der seriellen Beinkette zum Segment
      % (Körper-KS-Ursprung)
      Jg_Link = R.Leg(i_leg).jacobig_refpoint(q_leg, i_link, zeros(3,1));
      % Diskretisiere Länge des Segments
      for i_reflength = 1:n_reflength
        l_rel = (i_reflength-1)/(n_reflength-1); % von 0 bis 1 für Strecke auf Segment
        % Angriffspunkt der externen Kraft (relativ zu Gelenk-KS)
        r_L0_i_C = link2_L0*l_rel;
        % Umrechnung des Referenzpunktes in Segment-KS
        T_Li_L0 = invtr(Tc_Leg_i(:,:,i_link+1));
        rh_i_i_C = T_Li_L0 * [r_L0_i_C;0];
        % Beziehe Jacobi-Matrix auf diesen Punkt
        A_C_i = adjoint_jacobian(rh_i_i_C(1:3)); % Übergabe: Vektor von i nach C.
        J_C = A_C_i * Jg_Link; % Jg_Link entspricht J_i
        % Kraftangriff aus verschiedenen Richtungen prüfen
        % Definiere Koordinatensystem in Stabachse (nicht genau aus a/d
        % Parameter der DH Notation ableitbar)
        x_S = link2_L0/norm(link2_L0); % Stabrichtung ist x-Achse des KS S
        y_S = cross(rand(3,1), x_S); 
        y_S = y_S / norm(y_S);
%         z_S = cross(x_S, y_S);
        f_L0_all = NaN(3, n_force-1); % Im BK-Basis-KS
        for i_force = 1:n_force-1 % 2*pi entspricht 0. Letzten weglassen
          % Winkel mit dem die Kraft verdreht angreift (Drehung um Stab)
          theta_force = (i_force-1)/(n_force-1) * 2*pi;
          % Vektor rechtwinklig zur Stabachse link2_L0
          f_L0 = angvec2r(theta_force, x_S) * y_S * f_norm; % in y-Richtung`, dann Drehen
          f_L0_all(:,i_force) = f_L0;
          if debug_plot % Debug: Zeichnen
            % Kraft mit Angriffspunkt
            r_W_W_C = R.T_W_0 * T_0_L0 * Tc_Leg_i(:,:,i_link+1) * [rh_i_i_C(1:3);1];
            plot3(r_W_W_C(1), r_W_W_C(2), r_W_W_C(3), 'rx', 'MarkerSize', 12);
            plotratio = 0.15 / 140; % 140N entsprechen 150mm.
            f_W = R.T_W_0(1:3,1:3) * T_0_L0(1:3,1:3) * f_L0;
            quiver3(r_W_W_C(1), r_W_W_C(2), r_W_W_C(3), ...
              plotratio*f_W(1), plotratio*f_W(2), plotratio*f_W(3));
          end
        end
        % Bestimme Gelenkmoment aus dieser Kraft
        tau_all_Leg = J_C(1:3,:)' * f_L0_all; % freigeschnittene Beinkette
        % Gelenkmoment der PKM (für alle verschiedenen Kraftangriffe
        % zusammenfassen für kompaktere Berechnung)
        tau_all = zeros(size(tau_all_Leg,2),R.NJ);
        tau_all(:,R.I1J_LEG(i_leg):R.I2J_LEG(i_leg)) = tau_all_Leg';
        % Rechne Gelenkmoment der seriellen Kette in Antriebsmoment der PKM
        % um. Siehe Dissertation Schappler, Formel 2.130.
        JinvP = reshape(JinvP_ges(i_traj,:), R.NJ, sum(R.I_EE));
        tau_a_all = R.jointtorque_actjoint(q, XP(i_traj,:)', tau_all', JinvP)';
        % Speichere kleinsten Maximalwert ab. Der Fall würde dazu führen,
        % dass die Erkennung der externen Kraft schwer wäre
        [tau_a_worstcase, i_force_min_tmp] = min(max(abs(tau_a_all),[],2));
        if tau_a_worstcase < tau_a_min
          tau_a_min = tau_a_worstcase;
          i_traj_min = i_traj;
          i_leg_min = i_leg;
          i_link_min = i_link;
          i_force_min = i_force_min_tmp;
          f_W_min = R.T_W_0(1:3,1:3) * T_0_L0(1:3,1:3) * f_L0_all(:,i_force_min);
          r_W_W_Cmin = R.T_W_0 * T_0_L0 * Tc_Leg_i(:,:,i_link+1) * [rh_i_i_C(1:3);1];
        end
      end
    end
  end
end
%% Kontaktkraft auch für die Plattform-Kontakt bestimmen
for i_traj = 1:length(Traj_0.t)
  % Stelle Kraftrichtungen für Kontakt am Plattform-KS mit Kugelkoordinaten
  % zusammen. TODO: Noch ungeprüft
  k = 1;
  f_0_all = NaN(3,10*20);
  for i_phi = 1:10 % nur die erste Hälfte (sonst doppelt)
    for i_psi = 1:20 % in 18°-Schritten
      f_0 = rotx((i_phi-1)*2*pi/21) * roty((i_psi-1)*2*pi/21) * [f_norm;0;0];
      f_0_all(:,k) = f_0;
      k = k + 1;
    end
  end
  JinvE = reshape(JinvE_ges(i_traj,:), R.NJ, sum(R.I_EE));
  tau_a_all = (- (JinvE(R.I_qa,:))' \ [f_0_all;zeros(3,size(f_0_all,2))])';
  % Speichere kleinsten Maximalwert ab.
  [tau_a_worstcase, i_force_min_tmp] = min(max(abs(tau_a_all),[],2));
  if tau_a_worstcase < tau_a_min
    tau_a_min = tau_a_worstcase;
    i_traj_min = i_traj;
    i_leg_min = 0;
    i_link_min = 0;
    i_force_min = i_force_min_tmp;
    f_W_min = R.T_W_0(1:3,1:3) * f_0_all(:,i_force_min);
    r_W_W_Cmin = XE(i_traj_min,1:3)';
  end
end

%% Abschluss
f_mrk = -tau_a_min; % Werte von Null (am schlechtesten) bis beliebiger Größe. Zähle negativ, damit hohe Zahlenwerte beim Minimierungsproblem gut ist.
fval = 1e3 * 2/pi*atan(10/tau_a_min); % Wert 874/1000 bei Grenzfall von 2Nm. Bei höheren Momenten kleinerer Ergebniswert
fval_debugtext = sprintf('MRK-Sensitivität %1.1f N/Nm', tau_a_min);
%% Aktivierung des Debug-Plots prüfen
if Set.general.plot_details_in_fitness < 0 && 1e4*fval > abs(Set.general.plot_details_in_fitness) || ... % Gütefunktion ist schlechter als Schwellwert: Zeichne
   Set.general.plot_details_in_fitness > 0 && 1e4*fval < abs(Set.general.plot_details_in_fitness) % Gütefunktion ist besser als Schwellwert: Zeichne
  % Zeichnen/Debuggen (s.u.)
else
  return
end
%% Debug-Plot zeichnen
fhdl = change_current_figure(906); clf; hold all;
view(3); axis auto; hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m');

plotmode = 1; % Strichmodell
if R.Type == 0 % Seriell
  s_plot = struct( 'ks', [], 'straight', 1, 'mode', plotmode, 'nojoints', 1);
  R.plot( Q(i_traj_min,:)', s_plot);
else % PKM
  s_plot = struct( 'ks_legs', [], 'ks_platform', [], ...
    'straight', 1, 'mode', plotmode, 'nojoints', 1);
  R.plot( Q(i_traj_min,:)', Traj_0.X(i_traj_min,:)', s_plot);
end

% Kraft mit Angriffspunkt
plot3(r_W_W_Cmin(1), r_W_W_Cmin(2), r_W_W_Cmin(3), 'rx', 'MarkerSize', 12);
plotratio = 0.15 / 140; % 140N entsprechen 150mm.
quiver3(r_W_W_Cmin(1), r_W_W_Cmin(2), r_W_W_Cmin(3), ...
  plotratio*f_W_min(1), plotratio*f_W_min(2), plotratio*f_W_min(3), ...
  'LineWidth', 5, 'Color', 'r');

title(sprintf(['Kontaktkrafterkennung schlechtester Fall. ', ...
  'min(tau\\_a)=%1.e, I=%d/%d. Beinkette %d, Segment %d'], ...
  tau_a_min, i_traj_min, size(Q,1), i_leg_min, i_link_min));
drawnow();
% Bild speichern
[currgen,currind,currimg,resdir] = cds_get_new_figure_filenumber(Set, Structure,'ObjHRCForceDetection');
for fileext=Set.general.save_robot_details_plot_fitness_file_extensions
  if strcmp(fileext{1}, 'fig')
    saveas(fhdl, fullfile(resdir, sprintf('Gen%02d_Ind%02d_Eval%d_ObjHRCForceDetection.fig', currgen, currind, currimg)));
  else
    export_fig(fhdl, fullfile(resdir, sprintf('Gen%02d_Ind%02d_Eval%d_ObjHRCForceDetection.%s', currgen, currind, currimg, fileext{1})));
  end
end
