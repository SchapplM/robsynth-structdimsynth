% Zielfunktion ("objective function") für Optimierung in der Maßsynthese
% für MRK-Kennzahl (basierend auf Klemmwinkeln).
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
% JP
%   Gelenkpositionen aller Gelenke des Roboters
%   (Zeilen: Zeitschritte der Trajektorie)
% 
% Ausgabe:
% fval [1x1]
%   Zielfunktionswert, der im PSO-Algorithmus minimiert wird. Benutze den
%   kleinsten Klemmwinkel (negativ gezählter Klemmwinkel)
% fval_debugtext [char]
%   Zeile mit Hinweistext, der bei PSO nach Fitness-Berechnung ausgegeben wird
% debug_info [cell]
%   Zusatz-Informationen, die im Debug-Bild des Roboters angezeigt werden
% f_clamp [1x1]
%   Physikalischer Wert, der dem Zielfunktionswert zugrunde liegt.
%   Winkel wird negativ gezählt, da es ein Minimierungsproblem ist und
%   große Beträge besser sind.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval, fval_debugtext, debug_info, f_clamp] = cds_obj_mrk1(R, Set, Structure, Traj_0, Q, JP)
% Debug-Code:
if Set.general.matfile_verbosity > 2 % Debug
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_obj_mrk1.mat'));
end
% clear
% clc
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_obj_mrk1.mat'));
debug_info = '';

%% Bestimme Zugehörigkeit zu den Interaktionsvolumen
% Interaktions-Arbeitsraum in das KS 0 rotieren
collbodies_iaspc = cds_update_interactionspace(R, Set);
n_cb_robot = size(JP,2)/3;
collbodies_iaspc2 = struct(...
  'link', uint16([ collbodies_iaspc.link; ...
    repmat((1:n_cb_robot)'-1, 1,2) ]), ... % Der erste Eintrag in JP ist auch bzgl. Link 0
  'type', [ collbodies_iaspc.type; ...
    repmat(uint8(9), n_cb_robot, 1) ], ...
  'params', [ collbodies_iaspc.params; ...
    NaN(n_cb_robot, 10) ]);
% Virtuelle Kollisionsprüfung mit den Punkten und Interaktionsraum
collchecks_iaspc2 = [];
n_cb_iaspc = size(Set.task.interactionspace.type,1);
for i = 1:n_cb_iaspc
  % Prüfe, ob in diesem Interaktionsarbeitsraum die Gelenkklemmung
  % betrachtet wird
  if ~Set.task.interactionspace.check_clamp_angle, continue; end
  collchecks_iaspc2 = [collchecks_iaspc2; ...
    [n_cb_iaspc+(1:n_cb_robot)', repmat(uint8(i), n_cb_robot, 1)]]; %#ok<AGROW> 
end
% Eigentliche Prüfung auf Überschneidung
CollSet = struct('collsearch', false);
[coll_iaspc, ~] = check_collisionset_simplegeom_mex(collbodies_iaspc2, ...
  collchecks_iaspc2, JP, CollSet);
if n_cb_iaspc>1
  error('Mehr als ein Interaktionsraum nicht vollständig implementiert');
  % TODO: Die Zuordnung von coll_iaspc ist nicht mehr 1:1 mit JP
end

%% Winkel in passiven Gelenken bestimmen
Q_clamp_all = NaN(size(Q));
for k = 1:size(Q,1) % Alle Zeitschritte durchgehen
  i_pt = 1; % Index für Gelenkpunkte JP
  jp_k = reshape(JP(k,:), 3, size(JP,2)/3);
  % Debug: Zeichne Roboter und alle Gelenkpunkte mit Info, ob innen/außen
  if false
    fhdl = change_current_figure(5); clf; hold all; %#ok<UNRCH> 
    view(3); axis auto; hold on; grid on;
    xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
    if R.Type == 0 % Seriell
      s_plot = struct( 'ks', [], 'straight', 1, 'mode', 1, 'nojoints', 1);
      R.plot( Q(k,:)', s_plot);
    else % PKM
      s_plot = struct( 'ks_legs', [], 'ks_platform', [], ...
        'straight', 1, 'mode', 1, 'nojoints', 1);
      R.plot( Q(k,:)', Traj_0.X(k,:)', s_plot);
    end
    for jj = 1:size(jp_k,2)
      JP_jj_W = R.T_W_0 * [jp_k(:,jj);1];
      hdljj = plot3(JP_jj_W(1), JP_jj_W(2), JP_jj_W(3), 'kx', 'MarkerSize', 10);
      if ~coll_iaspc(k, jj)
        set(hdljj, 'Color', 'r');
      else
        set(hdljj, 'Color', 'g');
      end
    end
    % Zeichne Interaktionsbereich
    for kk = 1:length(collbodies_iaspc.type)
      if collbodies_iaspc.type(kk) == 10
        params_W = Set.task.interactionspace.params(kk,:); % ist schon im Welt-KS
        q_W = eye(3,4)*[params_W(1:3)';1];
        u1_W = params_W(4:6)';
        u2_W = params_W(7:9)';
        % letzte Kante per Definition senkrecht auf anderen beiden.
        u3_W = cross(u1_W,u2_W); u3_W = u3_W/norm(u3_W)*params_W(10);
        % Umrechnen in Format der plot-Funktion
        cubpar_c = q_W(:)+(u1_W(:)+u2_W(:)+u3_W(:))/2; % Mittelpunkt des Quaders
        cubpar_l = [norm(u1_W); norm(u2_W); norm(u3_W)]; % Dimension des Quaders
        cubpar_a = 180/pi*tr2rpy([u1_W(:)/norm(u1_W), u2_W(:)/norm(u2_W), u3_W(:)/norm(u3_W)],'zyx')'; % Orientierung des Quaders
        drawCuboid([cubpar_c', cubpar_l', cubpar_a'], ...
          'FaceColor', 'm', 'FaceAlpha', 0.03);
      end
    end
  end

  % Gehe alle Beinketten durch
  for i = 1:R.NLEG % Alle Beinketten durchgehen
    i_pt = i_pt + 1; % Basis-KS überspringen
    % Gehe alle technische Gelenke durch (nicht: Gelenk-FG)
    % Direkte Kinematik für Beinkette. 
    % TODO: Könnte theoretisch auch mit
    % Eingabe-Variable JP gelöst werden. Dort ist schon die direkte
    % Kinematik (für die Punkte, nicht die Rotationen) enthalten.
    Tc_Leg_i = R.Leg(i).fkine(Q(k,R.I1J_LEG(i):R.I2J_LEG(i))');
    % Transformation von Beinketten-Basis zum Plattform-KS zur Bestimmung
    % des Klemmwinkels des Plattform-Koppelgelenks.
    % Annahme: Plattform ist eine Platte in xy-Ebene des KS P.
    R_P_Bi = eulxyz2r(R.phi_P_B_all(:,i)); % Schnitt-KS auf Seite der Plattform
    R_L0_Ci = Tc_Leg_i(1:3,1:3,end); % Zum Schnitt-KS des Plattform-Koppelgelenks auf Seite der Beinkette
    R_L0_P = R_L0_Ci * R_P_Bi'; % Von Beinkette-Basis (L0) zu Plattform (P)
    % Vektor vom Plattform-Koppelgelenk zur Plattform-Mitte
    r_L0_Bi_P = -R_L0_P * R.r_P_B_all(:,i);
    clamp_check_type = 0; % Debug-Marker für Art der Klemmung
    j = 0; % Laufvariable für Gelenk-FG
    while j < R.Leg(i).NJ % Keine For-Schleife, damit Indizes übersprungen werden können
      j = j + 1;
      i_pt = 2 + j + R.I1L_LEG(i) - i; % Bestimme Gelenkpunkt-Index direkt (Alternative: i_pt hochzählen)
      % Prüfe, ob Gelenkpunkt im Interaktionsarbeitsraum ist
      if ~coll_iaspc(k, i_pt), continue; end
      % Abgleich des neu mit direkter Kinematik berechneten KS mit den
      % eingegebenen Gelenkpunkten (Konsistenzprüfung), im Basis-KS
      test_jp = R.Leg(i).T_W_0 * Tc_Leg_i(:,:,j+1) * [0;0;0;1] - [jp_k(:,i_pt); 1];
      if ~all(abs(test_jp) < 1e-6)
        save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', ...
          sprintf('cds_obj_mrk1_error_testjp_%s_%s.mat', Set.optimization.optname, Structure.Name)));
        error('Fehler bei Indizes der Gelenkpunkte');
      end
      if R.Leg(i).MDH.sigma(j) == 1, continue; end % keine Schubgelenke betrachten
      if j == 1, continue; end % Keinen Klemmwinkel für gestellfestes Gelenk betrachten
      % Bestimme Vektor des Segments zum Gelenk hin (im Basis-KS der Beinkette)
      link1_L0 = Tc_Leg_i(1:3,4,j+1)-Tc_Leg_i(1:3,4,j);
      % Klemmwinkel für Gelenk-FG j der Beinkette i zu Zeitschritt k
      if R.Leg(i).DesPar.joint_type(j) == 0 % Drehgelenk
        if j < R.Leg(i).NJ % Drehgelenk in der Kette
          % Vektor zum nächsten Gelenk (Gelenkpunkt liegt bei j+1)
          link2_L0 = Tc_Leg_i(1:3,4,j+2)-Tc_Leg_i(1:3,4,j+1);
          clamp_check_type = 1;
          Q_clamp_all(k,R.I1J_LEG(i)-1+j) = acos(-link1_L0' * link2_L0 / ...
                                                (norm(link1_L0)*norm(link2_L0)));
          % Alter Ansatz: Direkt den Gelenkwinkel nehmen. Das ignoriert den
          % MDH-Parameter alpha
%           % Winkel ist direkt ablesbar
%           Q_clamp_all(k,R.I1J_LEG(i)-1+j) = Q(k,R.I1J_LEG(i)-1+j);
%           % Probe: Winkel auch aus KS-Trafo holen (Prüfung der Indizes)
%           T_joint_j = Tc_Leg_i(:,:,j) \ Tc_Leg_i(:,:,j+1);
%           phi_joint_j = r2eulxyz(T_joint_j(1:3,1:3));
%           q_ik_test = phi_joint_j(3);
%           if abs(angleDiff(Q_clamp_all(k,R.I1J_LEG(i)-1+j), q_ik_test))>1e-3
%             save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', ...
%               sprintf('cds_obj_mrk1_error_testqdiff_%s_%s.mat', Set.optimization.optname, Structure.Name)));
%             error('Berechnung des Winkels des Drehgelenks inkonsistent: Q vs Tc_Leg_i');
%           end
        else % Letztes Gelenk der Kette
          clamp_check_type = 2;
          % TODO: Siehe andere Fälle weiter unten
%           Q_clamp_all(k,R.I1J_LEG(i)-1+j) = asin(R_L0_P(1:3,3)' * link1_L0);
        end
      elseif R.Leg(1).DesPar.joint_type(j) == 2 % Kardan-Gelenk
        % Der Gelenkpunkt liegt bei Transformationsmatrix Index j+1 und j+2
        if j < R.Leg(i).NJ-1 % Gelenk liegt in der Kette, nicht am Ende
          clamp_check_type = 1;
          % Vektor zum nächsten Gelenk
          link2_L0 = Tc_Leg_i(1:3,4,j+3)-Tc_Leg_i(1:3,4,j+2);
          Q_clamp_all(k,R.I1J_LEG(i)-1+j) = acos(-link1_L0' * link2_L0 / ...
                                                (norm(link1_L0)*norm(link2_L0)));
          % Probe: Winkel auch aus Gelenkkoordinaten bestimmen
%           T_joint_j = Tc_Leg_i(:,:,j) \ Tc_Leg_i(:,:,j+2);
%           R_joint_j_test = rotx(R.Leg(i).MDH.alpha(j)) * rotz(Q(k,R.I1J_LEG(i)-1+j)) * ...
%                            rotx(R.Leg(i).MDH.alpha(j+1))*rotz(Q(k,R.I1J_LEG(i)-1+j+1));
%           % TODO ...
        else % Gelenk am Ende der Kette. Klemmung mit Plattform
          % Winkel zwischen Vektor und Ebene (90° Minus der Winkel zwischen
          % der Normalen (z-Achse) und dem Vektor). Daher asin statt acos.
          % TODO: So inkonsistent mit Kugelgelenk. Noch nicht plausibel.
%           Q_clamp_all(k,R.I1J_LEG(i)-1+j) = asin(R_L0_P(1:3,3)' * link1_L0);
        end
      elseif R.Leg(1).DesPar.joint_type(j) == 3 % Kugel-Gelenk
        if j < R.Leg(i).NJ-2 % Gelenk liegt in der Kette (nicht in Datenbank vorgesehen)
          warning('TODO: Noch nicht implementiert');
        else % Klemmwinkel zwischen vorherigem Segment und Plattform
          clamp_check_type = 2;
          % TODO: Hier fehlt noch ein plausibler Ansatz
          % Winkel zwischen Oberflächennormale der Plattform und des Stabs
%           Q_clamp_all(k,R.I1J_LEG(i)-1+j) = asin(R_L0_P(1:3,3)' * link1); % s.o.
          % Winkel zwischen Vektor vom Gelenk zur Plattform-Mitte
%           Q_clamp_all(k,R.I1J_LEG(i)-1+j) = acos(-link1_L0' * r_L0_Bi_P);
        end
      end
      if false && ~isnan(Q_clamp_all(k,R.I1J_LEG(i)-1+j)) % Debuggen
        fhdl = change_current_figure(3); clf; hold all; %#ok<UNRCH> 
        view(3); axis auto; hold on; grid on;
        xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
        if R.Type == 0 % Seriell
          s_plot = struct( 'ks', [], 'straight', 1, 'mode', 1, 'nojoints', 1);
          R.plot( Q(k,:)', s_plot);
        else % PKM
          s_plot = struct( 'ks_legs', [], 'ks_platform', [], ...
            'straight', 1, 'mode', 1, 'nojoints', 1);
          R.plot( Q(k,:)', Traj_0.X(k,:)', s_plot);
        end
        % Aktuell betrachtete Vektoren einzeichnen: Segment zum Gelenk
        joint_W = R.T_W_0 * R.Leg(i).T_W_0 * Tc_Leg_i(:,:,j+1) * [0;0;0;1];
        link1_W = R.T_W_0 * R.Leg(i).T_W_0 * [link1_L0; 0];
        plot3(joint_W(1)-[0;link1_W(1)], joint_W(2)-[0;link1_W(2)], ...
          joint_W(3)-[0;link1_W(3)], 'r--', 'LineWidth', 5);
        if clamp_check_type == 1 % In Kette: Zeichne nächstes Segment
          link2_W = R.T_W_0 * R.Leg(i).T_W_0 * [link2_L0; 0];
          plot3(joint_W(1)+[0;link2_W(1)], joint_W(2)+[0;link2_W(2)], ...
            joint_W(3)+[0;link2_W(3)], 'c--', 'LineWidth', 5);
        elseif clamp_check_type == 2 % Mit Plattform: Zeichne Plf.-Vektor (muss konsistent zu oben sein)
          plfcpl_W = R.T_W_0 * R.Leg(i).T_W_0 * Tc_Leg_i(:,:,end) * [0;0;0;1];
          plfvec_W = R.T_W_0 * R.Leg(i).T_W_0 * [r_L0_Bi_P;0];
          plot3(plfcpl_W(1)+[0;plfvec_W(1)],plfcpl_W(2)+[0;plfvec_W(2)], ...
            plfcpl_W (3)+[0;plfvec_W(3)], 'b--', 'LineWidth', 5);
        elseif clamp_check_type == 0
          save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', ...
            sprintf('cds_obj_mrk1_error_plotundefinedcase_%s_%s.mat', Set.optimization.optname, Structure.Name)));
          error('Fall undefiniert')
        end
        % Aktuellen Winkel eintragen
        jp_w = R.T_W_0 * [jp_k(:,i_pt);1];
        text(jp_w(1), jp_w(2), jp_w(3), sprintf('%1.0f°', ...
          180/pi*Q_clamp_all(k,R.I1J_LEG(i)-1+j)), 'BackgroundColor', 'y');
        % Zeichne Interaktionsbereich
        for kk = 1:length(collbodies_iaspc.type)
          if collbodies_iaspc.type(kk) == 10
            params_0 = collbodies_iaspc.params(kk,:); % Testweise nochmal hier transformieren
            q_W = eye(3,4)*R.T_W_0*[params_0(1:3)';1]; % Ortsvektor
            u1_W = eye(3,4)*R.T_W_0*[params_0(4:6)';0]; % Richtungsvektor
            u2_W = eye(3,4)*R.T_W_0*[params_0(7:9)';0];
            % letzte Kante per Definition senkrecht auf anderen beiden.
            u3_W = cross(u1_W,u2_W); u3_W = u3_W/norm(u3_W)*params_0(10);
            % Umrechnen in Format der plot-Funktion
            cubpar_c = q_W(:)+(u1_W(:)+u2_W(:)+u3_W(:))/2; % Mittelpunkt des Quaders
            cubpar_l = [norm(u1_W); norm(u2_W); norm(u3_W)]; % Dimension des Quaders
            cubpar_a = 180/pi*tr2rpy([u1_W(:)/norm(u1_W), u2_W(:)/norm(u2_W), u3_W(:)/norm(u3_W)],'zyx')'; % Orientierung des Quaders
            drawCuboid([cubpar_c', cubpar_l', cubpar_a'], ...
              'FaceColor', 'm', 'FaceAlpha', 0.05);
          end
        end
        titleaddstr='';
        if ~coll_iaspc(k, i_pt), titleaddstr=' (nicht in Interaktionsraum)'; end
        title(sprintf('Beinkette %d, Gelenk-FG %d%s', i, j, titleaddstr));
        drawnow();
      end
      if R.Leg(i).DesPar.joint_type(j) == 2 % Kardan-Gelenk
        j = j + 1; % Überspringe nächsten Gelenk-FG
        continue
      end
      if R.Leg(i).DesPar.joint_type(j) == 3 % Kugel-Gelenk
        j = j + 2; % Überspringe nächste beiden Gelenk-FG
        continue
      end
    end
  end
end

%% Kleinsten Klemmwinkel bestimmen
[minclampangle, IIcmin] = min(abs(Q_clamp_all),[],2); % kleinster jeweils alle Zeitschritte
[min2clampangle, IItmin] = min(minclampangle); % Zeitschritt für kleinsten Winkel von allen

if all(isnan(min2clampangle)) % Falls kein Gelenk von der MRK-Kennzahl erfasst wird
  fval = 0;
  f_clamp = -pi; % Bestmöglicher Wert
  fval_debugtext = 'Kein Klemmwinkel im Interaktionsbereich.';
  return
end

% Kennzahl berechnen: Bester Wert ist 180°, schlechtester Wert ist 0°. Es
% gibt keine negativen Winkel
f_clamp_norm = (pi - min2clampangle)/pi; % 0 bis 1 (Minimierungsproblem)
fval = 1e3*f_clamp_norm; % 0 bis 1e3 (Wertebereich für Toolbox)
f_clamp = -min2clampangle; % Physikalischer Wert (Winkel in Grad), negativ, da kleine Beträge schlecht sind
fval_debugtext = sprintf('Klemmwinkel %1.1f°.', 180/pi*min2clampangle);

%% Aktivierung des Debug-Plots prüfen
if Set.general.plot_details_in_fitness < 0 && 1e4*fval > abs(Set.general.plot_details_in_fitness) || ... % Gütefunktion ist schlechter als Schwellwert: Zeichne
   Set.general.plot_details_in_fitness > 0 && 1e4*fval < abs(Set.general.plot_details_in_fitness) % Gütefunktion ist besser als Schwellwert: Zeichne
  % Zeichnen/Debuggen (s.u.)
else
  return
end
%% Debug-Plot zeichnen
fhdl = change_current_figure(901); clf; hold all;
view(3); axis auto; hold on; grid on;
xlabel('x in m'); ylabel('y in m'); zlabel('z in m');

plotmode = 1; % Strichmodell
if R.Type == 0 % Seriell
  s_plot = struct( 'ks', [], 'straight', 1, 'mode', plotmode, 'nojoints', 1);
  R.plot( Q(IItmin,:)', s_plot);
else % PKM
  s_plot = struct( 'ks_legs', [], 'ks_platform', [], ...
    'straight', 1, 'mode', plotmode, 'nojoints', 1);
  R.plot( Q(IItmin,:)', Traj_0.X(IItmin,:)', s_plot);
end
% Zeichne Beträge aller Klemmwinkel in das Bild ein
JP_tmin = reshape(JP(IItmin,:), 3, size(JP,2)/3); % KS-/Gelenkpunkte zum Zeitpunkt des Bildes
i_pt = 1;
fprintf('Klemmwinkel für Zeitschritt %d/%d: %s\n', IItmin, size(Q_clamp_all,1), ...
  disp_array(180/pi*Q_clamp_all(IItmin,~isnan(Q_clamp_all(IItmin,:))), '%1.0f'));
for i = 1:R.NLEG
  % Basis-KS überspringen
  i_pt = i_pt + 1;
  for j = 1:R.Leg(i).NJ
    i_pt = i_pt + 1;
    idx_q = R.I1J_LEG(i)-1+j;
    if isnan(Q_clamp_all(IItmin,idx_q)), continue; end % Kein Klemmwinkel bestimmt (s.o.)
    % In Welt-KS umrechnen
    jp_w = R.T_W_0 * [JP_tmin(:,i_pt);1];
    if IIcmin(IItmin) == idx_q
      plot3(jp_w(1), jp_w(2), jp_w(3), 'rx', 'MarkerSize', 20);
    else
      plot3(jp_w(1), jp_w(2), jp_w(3), 'bs', 'MarkerSize', 15);
    end
    text(jp_w(1), jp_w(2), jp_w(3), sprintf('%1.1f°', ...
      180/pi*Q_clamp_all(IItmin,idx_q)), 'BackgroundColor', 'y');
  end
end
% Zeichne Interaktionsbereich
for kk = 1:length(collbodies_iaspc.type)
  if collbodies_iaspc.type(kk) == 10
    params_W = Set.task.interactionspace.params(kk,:); % ist schon im Welt-KS
    q_W = eye(3,4)*[params_W(1:3)';1];
    u1_W = params_W(4:6)';
    u2_W = params_W(7:9)';
    % letzte Kante per Definition senkrecht auf anderen beiden.
    u3_W = cross(u1_W,u2_W); u3_W = u3_W/norm(u3_W)*params_W(10);
    % Umrechnen in Format der plot-Funktion
    cubpar_c = q_W(:)+(u1_W(:)+u2_W(:)+u3_W(:))/2; % Mittelpunkt des Quaders
    cubpar_l = [norm(u1_W); norm(u2_W); norm(u3_W)]; % Dimension des Quaders
    cubpar_a = 180/pi*tr2rpy([u1_W(:)/norm(u1_W), u2_W(:)/norm(u2_W), u3_W(:)/norm(u3_W)],'zyx')'; % Orientierung des Quaders
    drawCuboid([cubpar_c', cubpar_l', cubpar_a'], ...
      'FaceColor', 'm', 'FaceAlpha', 0.1);
  end
end
title(sprintf(['Klemmwinkel schlechtester Fall. ', ...
  'Winkel=%1.1f°, I=%d/%d'], 180/pi*min2clampangle, IItmin, size(Q,1)));
drawnow();
% Bild speichern
[currgen,currind,currimg,resdir] = cds_get_new_figure_filenumber(Set, Structure,'ObjClampAngle');
for fileext=Set.general.save_robot_details_plot_fitness_file_extensions
  if strcmp(fileext{1}, 'fig')
    saveas(fhdl, fullfile(resdir, sprintf('Gen%02d_Ind%02d_Eval%d_ObjClampAngle.fig', currgen, currind, currimg)));
  else
    export_fig(fhdl, fullfile(resdir, sprintf('Gen%02d_Ind%02d_Eval%d_ObjClampAngle.%s', currgen, currind, currimg, fileext{1})));
  end
end
