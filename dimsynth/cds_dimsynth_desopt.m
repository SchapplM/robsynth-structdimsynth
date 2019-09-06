% Entwurfsoptimierung des Robotermodells: Bestimmung von Dynamikparametern
% 
% Unterscheidung von Koordinatensystemen:
% Körper: Den einzelnen Starrkörpern des Robotermodells zugeordnet
% Geometrie: Einzelnen Ersatzkörpern (z.B. Zylindern) zugeordnet
% 
% Zum Debuggen:
% * Bilder einkommentieren
% * Koordinatensysteme von zu prüfenden KS/Vektoren zeichnen (trplot)
% * Trägheitsellipsen, Schwerpunkte auf Plausibilität prüfen
% 
% 
% Quelle:
% [A] Aufzeichnungen Schappler, 23.08.2019
% [B] Aufzeichnungen Schappler, 06.09.2019

% Siehe auch: SerRob/plot (bezüglich Anfangssegment für Schubgelenke)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function R=cds_dimsynth_desopt(R, Q, Traj_0, Set, Structure)
save(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_desopt.mat'));
desopt_debug = false;
% Debug:
% function R=cds_dimsynth_desopt()
% desopt_debug = true;
% load(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_desopt.mat'));

%% Definitionen, Konstanten
density = 2.7E3; %[kg/m^3] Aluminium

%% Debug: Roboter zeichnen; in dieses Bild werden Debug-KS eingezeichnet
if desopt_debug
  figure(3000);clf;
  s = struct('mode', 4, 'ks', 1:R.NJ, 'straight', false);
  R.plot(Q(1,:)', s); xlabel('x');ylabel('y');zlabel('z');
  view([0,90])
end

%% Entwurfsparameter ändern
% TODO: Hier Optimierung durchführen
% Vorläufig: Segmenteigenschaften direkt belegen
if R.Type == 0
  R.DesPar.seg_par = repmat([5e-3, 300e-3], R.NL, 1); % dicke Struktur
  % R.DesPar.seg_par(end,:) = [5e-3, 300e-3]; % EE anders zum Testen der Visu.
elseif R.Type == 2  % Parallel (symmetrisch)
  for i = 1:R.NLEG
    % Belege alle Beinketten, damit in Plot-Funktion nutzbar. Es werden
    % eigentlich nur die Werte der ersten Beinkette für Berechnungen
    % genutzt
    R.Leg(i).DesPar.seg_par = repmat([2e-3, 50e-3], R.Leg(1).NL, 1); % dünne Struktur
  end
  R.DesPar.platform_par(2) = 10e-3; % Dünne Platte als Plattform
end

%% Dynamikparameter belegen
% Parameter des Struktursegmentes
% Roboterklasse für Parametermodell heraussuchen
if R.Type == 0 % Seriell
  R_pkin = R;
else  % Parallel (symmetrisch)
  R_pkin = R.Leg(1);
end
%% EE-Zusatzlast

if any(Set.task.payload.Ic(4:6)~=0) || any(diff(Set.task.payload.Ic(1:3)))
  error('Nicht symmetrische EE-Last noch nicht definiert');
end
% Dynamikparameter für Robotermodell belegen
m_ges_Zus = zeros(R_pkin.NL,1);
mrS_ges_Zus = zeros(R_pkin.NL,3);
If_ges_Zus = zeros(R_pkin.NL,6);
m_ges_Zus(end) = Set.task.payload.m;
if R.Type == 0 % Seriell
  r_N_N_E = R.T_N_E(1:3,4);
  [mrS_ges_Zus(end,:), If_ges_Zus(end,:)] = inertial_parameters_convert_par1_par2( ...
    r_N_N_E(:)', Set.task.payload.Ic(:)', Set.task.payload.m);
else
  r_P_P_E = R.T_P_E(1:3,4);
  [mrS_ges_Zus(end,:), If_ges_Zus(end,:)] = inertial_parameters_convert_par1_par2( ...
    r_P_P_E(:)', Set.task.payload.Ic(:)', Set.task.payload.m);
end
%% Strukturteile
% Länge von Schubgelenken herausfinden
if Structure.Type == 0 % Seriell
  q_range = diff(minmax2(Q')')';
else  % Parallel (symmetrisch)
  q_range = diff(minmax2(Q(:,R.I_qa)')')';
end
% Dynamikparameter initialisieren:
% Segmente des Roboters
m_ges_Link = NaN(R_pkin.NL,1);
mrS_ges_Link = NaN(R_pkin.NL,3);
If_ges_Link = NaN(R_pkin.NL,6);
% P-Stator: Fester Teil der Linear-Antriebseinheit (Außenzyliner, Linearführung)
m_ges_PStator = zeros(R_pkin.NL,1);
mrS_ges_PStator = zeros(R_pkin.NL,3);
If_ges_PStator = zeros(R_pkin.NL,6);
% P-Abtrieb: Beweglicher Teil der Linear-Antriebseinheit (Innenzylinder, Schlitten)
m_ges_PAbtrieb = zeros(R_pkin.NL,1);
mrS_ges_PAbtrieb = zeros(R_pkin.NL,3);
If_ges_PAbtrieb = zeros(R_pkin.NL,6);
%% Alle Körper des Roboters durchgehen
for i = 1:length(m_ges_Link)
  % Wandstärke und Radius der Hohlzylinder aus Robotereigenschaften
  if R_pkin.DesPar.seg_type(i) == 1 % Hohlzylinder
    e_i = R_pkin.DesPar.seg_par(i,1);
    R_i = 0.5*R_pkin.DesPar.seg_par(i,2);
  else
    error('Strukturmodell nicht definiert');
  end
  if i < size(m_ges_Link,1) % Segmente zu Gelenken
    %% Länge und Orientierung des Segments bestimmen
    % a- und d-Parameter werden zu Segment vor dem Gelenk gezählt (wg.
    % MDH-Notation) -> a1/d1 zählen zum Basis-Segment;
    if R.MDH.sigma(i) == 0 % Drehgelenk
      % Nehme nur die konstanten a- und d-Parameter als "schräge"
      % Verbindung der Segmente
      r_i_i_D = [R_pkin.MDH.a(i); 0; R_pkin.MDH.d(i)]; % [A]/(6)
      % [A]/(4); Drehe das Segment-KS so, dass die Längsachse des
      % Segments in x-Richtung dieses KS zeigt
      R_i_Si = rotx(R_pkin.MDH.alpha(i)) * roty(atan2(R_pkin.MDH.d(i), R_pkin.MDH.a(i)));
    elseif R_pkin.DesPar.joint_type(i) == 1 % Schubgelenk ohne spezielles Modell
      % Dieses Modell ist sehr ungenau und berücksichtigt
      % nicht den Anfangspunkt des Schubgelenks
      r_i_i_D = [R_pkin.MDH.a(i); 0; q_range(i)];
      R_i_Si = rotx(R_pkin.MDH.alpha(i)) * roty(atan2(q_range(i), R_pkin.MDH.a(i)));
    else % Schubgelenk mit Modell
      % Betrachte nur die Verschiebung des a-Parameters als Segment. Der
      % d-Parameter wird durch ein spezielles Schubgelenkmodell
      % berücksichtigt. Zum Start von Linearachse oder Hubzylinder wird
      % eine spezielle Verbindung modelliert. Diese entspricht dem Segment.
      if R_pkin.DesPar.joint_type(i) ==  4 % Schubgelenk ist Linearführung
        if R_pkin.qlim(i,2)*R_pkin.qlim(i,2)< 0 % Führung liegt auf der Höhe; [B]/(4)
          r_i_i_D = [R_pkin.MDH.a(i); 0; 0];
          % Nur die a-Verschiebung entspricht dem Segment. Daher nur
          % alpha-Rotation. Der a-Parameter wird für das VZ der Rotation benutzt.
          R_i_Si = rotx(R_pkin.MDH.alpha(i)) * roty(atan2(0, R_pkin.MDH.a(i)));
        else % Start oder Ende der Führung liegen am nächsten an vorherigem Gelenk
          % Keine senkrechte Verbindung (Schiene fängt woanders an)
          if R_pkin.qlim(i,2) < 0 % Schiene liegt komplett "links" vom KS. Gehe zum Endpunkt (qmax)
            % [B]/(6)
            r_i_i_D = [R_pkin.MDH.a(i);0;0] + rotx(R_pkin.MDH.alpha(i))*[0;0;R_pkin.qlim(i,2)];
            R_i_Si = rotx(R_pkin.MDH.alpha(i)) * roty(-atan2(R_pkin.qlim(i,2), R_pkin.MDH.a(i)));
          else % Schiene liegt "rechts" vom KS. Gehe zum Startpunkt (qmin)
            % [B]/(5)
            r_i_i_D = [R_pkin.MDH.a(i);0;0] + rotx(R_pkin.MDH.alpha(i))*[0;0;R_pkin.qlim(i,1)];
            R_i_Si = rotx(R_pkin.MDH.alpha(i)) * roty(-atan2(R_pkin.qlim(i,1), R_pkin.MDH.a(i)));
          end
        end
      else % Schubgelenk ist Hubzylinder
        if R_pkin.qlim(i,2) > 2*R_pkin.qlim(i,1) && R_pkin.qlim(i,2) > 0 % Zylinder liegt auf der Höhe des KS; [B]/(1)
          % Als senkrechte Verbindung unter Benutzung des a-Parameters der
          % DH-Notation für Verschiebung und für Vorzeichen der Rotation.
          r_i_i_D = [R_pkin.MDH.a(i); 0; 0];
          R_i_Si = rotx(-R_pkin.MDH.alpha(i)) * roty(atan2(0, R_pkin.MDH.a(i)));
        else
          if R_pkin.qlim(i,2) < 0 % Großer Zylinder liegt komplett "links" (von x-Achse schauend); [B]/(3)
            % Rechte Seite des Hubzylinders entspricht qmin
            r_i_i_D = [R_pkin.MDH.a(i); 0; 0] + rotx(R_pkin.MDH.alpha(i))*[0;0;R_pkin.qlim(i,1)];
            R_i_Si = rotx(R_pkin.MDH.alpha(i)) * roty(atan2(-R_pkin.qlim(i,1), R_pkin.MDH.a(i)));
          else % Großer Zylinder liegt komplett "rechts"
            % Linke Seite des Zylinders indirekt aus rechter Seite und
            % Länge berechnen
            r_i_i_D = [R_pkin.MDH.a(i); 0; 0] + rotx(R_pkin.MDH.alpha(i))*[0;0;2*R_pkin.qlim(i,1)-R_pkin.qlim(i,2)];
            R_i_Si = rotx(R_pkin.MDH.alpha(i)) * roty(-atan2(2*R_pkin.qlim(i,1)-R_pkin.qlim(i,2), R_pkin.MDH.a(i)));
          end
        end
      end
    end
    % Probe, ob Orientierung (R_i_Si) und berechnetes Ende des Segments (r_i_i_D)stimmen
    if any( abs(r_i_i_D - R_i_Si*[norm(r_i_i_D);0;0] ) > 1e-10)
      error('Richtung des Segments stimmt nicht');
    end
    % Probe: [A]/(10) (nur für Drehgelenke; wegen Maximallänge bei
    % Schubgelenk
    r_i_i_ip1_test = R_i_Si*[norm(r_i_i_D);0;0];
    Tges=R_pkin.jtraf(zeros(R_pkin.NJ,1));
    r_i_i_ip1 = Tges(1:3,4,i);
    if R_pkin.MDH.sigma(i) == 0 && any(abs(r_i_i_ip1_test-r_i_i_ip1) > 1e-10)
      error('Segment-Darstellung stimmt nicht');
    end
    %% Bestimme zusätzliche Dynamikparameter für Schubgelenke:
    if R_pkin.MDH.sigma(i) == 1
      if R_pkin.DesPar.joint_type(i) ==  5
        % Bei ausfahrbaren Zylindern gibt es zusätzlich noch einen
        % statischen Teil. Dieser Teil wird dem vorherigen Segment
        % zugeordnet. Ähnlich wie ein Stator beim Motor.
        % Berechne Anfangs- und Endpunkt des äußeren Zylinders für Geometrie
        T_i_Si1 = trotx(R_pkin.MDH.alpha(i))*transl(R_pkin.MDH.a(i),0,0)*...
          transl(0,0,R_pkin.qlim(i,1)-diff(R_pkin.qlim(i,:)));
        T_i_Si2 = T_i_Si1*transl(0,0,diff(R_pkin.qlim(i,:)));
        l_outercyl = norm(T_i_Si2(1:3,4)-T_i_Si1(1:3,4));
        [m_s, J_B_C] = data_hollow_cylinder(R_i, e_i, l_outercyl, density);
        % Umrechnen auf Körper-KS: rotx(alpha) damit z-Achse entlang
        % Zylinder zeigt. roty, weil Längsachse im Geometrie-KS x ist
        % anstatt z
        R_i_B = rotx(R_pkin.MDH.alpha(i))*roty(-pi/2);
        J_i_C = R_i_B * J_B_C * R_i_B'; % Rotation ins Körper-KS
        r_i_Oi_C = T_i_Si1(1:3,4) + T_i_Si1(1:3,1:3)*[0;0;l_outercyl/2];
        % Eintragen in Dynamik-Parameter (bezogen auf Ursprung)
        m_ges_PStator(i) = m_s;
        [mrS_ges_PStator(i,:), If_ges_PStator(i,:)] = inertial_parameters_convert_par1_par2( ...
          r_i_Oi_C', inertiamatrix2vector(J_i_C), m_s);
        % Der ausfahrbare Teil des Zylinders wird dem nächsten Segment
        % zugeordnet
        l_innercyl = q_range(i);
        [m_s, J_S_C] = data_hollow_cylinder(R_i-e_i, e_i*1.5, l_innercyl, density);
        R_i_L = roty(pi/2); % Hier Längsachse z; in Geometrie-KS x-Achse
        J_ip1_C = R_i_L * J_S_C * R_i_L';
        r_ip1_Oip1_C = [0;0;-(q_range(i))/2];
        m_ges_PAbtrieb(i+1) = m_s;
        [mrS_ges_PAbtrieb(i+1,:), If_ges_PAbtrieb(i+1,:)] = inertial_parameters_convert_par1_par2( ...
          r_ip1_Oip1_C', inertiamatrix2vector(J_ip1_C), m_s);
      elseif R_pkin.DesPar.joint_type(i) ==  4 % Linearführung
        % Benutze Gleiche Hohlzylinderannahme für Dynamikparameter der
        % Führung. Speichere als "Stator" der Antriebseinheit.
        l_rail = q_range(i); % Länge der Führung aus Gelenkgrenzen
        [m_s, J_B_C] = data_hollow_cylinder(R_i, e_i, l_rail, density);
        % Orientierung der x-Achse des Geometrie-KS längs zur
        % Bewegungsrichtung der Linearachse
        R_i_B = rotx(R_pkin.MDH.alpha(i))*roty(-pi/2);
        J_i_C = R_i_B * J_B_C * R_i_B'; % Rotation ins Körper-KS
        % Schwerpunkt in der Mitte der Führung
        r_i_Oi_C = [R_pkin.MDH.a(i);0;0] + rotx(R_pkin.MDH.alpha(i))*[0;0;R_pkin.qlim(i,1)+0.5*q_range(i)];
        % Eintragen in Dynamik-Parameter (bezogen auf Ursprung)
        m_ges_PStator(i) = m_s;
        [mrS_ges_PStator(i,:), If_ges_PStator(i,:)] = inertial_parameters_convert_par1_par2( ...
          r_i_Oi_C', inertiamatrix2vector(J_i_C), m_s);
        % Benutze Gleiche Hohlzylinderannahme für Dynamikparameter des
        % Schlittens. Diese Parameter müssen zum nächsten Segment gezählt
        % werden. TODO: Modellierung erstellen und implementieren.
        m_ges_PAbtrieb(i+1) = 0;
        mrS_ges_PAbtrieb(i+1,:) = 0;
        If_ges_PAbtrieb(i+1,:) = 0;
      end
    end
  else
    % Letzter Körper: Flansch->EE bei Seriell; Plattform->EE bei Parallel
    if Structure.Type == 0 % Serieller Roboter
      r_i_i_D = R_pkin.T_N_E(1:3,4);
      % Rotationsmatrix für das letzte Segment-KS
      % Die x-Achse muss zum EE zeigen. Die anderen beiden Achsen sind
      % willkürlich senkrecht dazu.
      if ~all(R_pkin.T_N_E(1:3,4) == 0)
        x_Si = R_pkin.T_N_E(1:3,4) / norm(R_pkin.T_N_E(1:3,4));
        y_Si = cross(x_Si, R_pkin.T_N_E(1:3,1));
        y_Si = y_Si/norm(y_Si);
        z_Si = cross(x_Si, y_Si);
        R_i_Si = [x_Si, y_Si, z_Si];
      else
        R_i_Si = eye(3); % Es gibt keinen Abstand zum letzten KS. EE hat keine eigene Ausdehnung
      end
      if abs(det(R_i_Si)-1) > 1e-6 || any(isnan(R_i_Si(:)))
        error('Letztes Segment-KS stimmt nicht');
      end
    else % Paralleler Roboter
      r_i_i_D = R.T_P_E(1:3,4);
      % Annahme: Da die Plattform in der Mitte des Roboters ist, muss keine besondere
      % EE-Transformation berücksichtigt werden.
      R_i_Si = eye(3);
      if any(R.T_P_E(1:3,4))
        error('Der Vergleich zwischen Seriell und Parallel bei vorhandener Trafo P-E ist aktuell nicht möglich');
      end
    end
  end
  %% Segment-Geometrie als Dynamikparameter berechnen
  if Structure.Type == 0 || Structure.Type == 2 && i < size(m_ges_Link,1)
    % Hohlzylinder-Segment für Serielle Roboter und PKM
    L_i = norm(r_i_i_D);
    % Modellierung Hohlzylinder. Siehe TM-Formelsammlung
    [m_s, J_S_C] = data_hollow_cylinder(R_i, e_i, L_i, density);
    % Umrechnen auf Körper-KS;
    J_i_C = R_i_Si * J_S_C * R_i_Si'; % [A]/(8)
    r_i_Oi_C = 0.5*r_i_i_D;
    % Eintragen in Dynamik-Parameter (bezogen auf Ursprung)
    J_i_C_vec= inertiamatrix2vector(J_i_C);
    m_ges_Link(i) = m_s;
    [mrS_ges_Link(i,:), If_ges_Link(i,:)] = inertial_parameters_convert_par1_par2(r_i_Oi_C', J_i_C_vec, m_s);
  elseif Structure.Type == 2 && i == size(m_ges_Link,1)
    % Plattform-Segment bei PKM
    % Annahme: Kreisscheibe
    if R.DesPar.platform_method == 1
      R_P = R.DesPar.platform_par(1) / 2;
      e_P = R.DesPar.platform_par(2);

      % Modellierung Kreisscheibe (in Schwerpunkts-KS). Siehe TM-Formelsammlung
      m_P = density*pi*R_P^2*e_P;
      J_P_P_zz = 0.5 * m_P * R_P^2; % Normalenrichtung der Kreisscheibe
      J_P_P_xx = 0.5*J_P_P_zz; J_P_P_yy = 0.5*J_P_P_zz;
      r_P_P_C = zeros(3,1);
      J_P_C = diag([J_P_P_xx; J_P_P_yy; J_P_P_zz]);
      % Umrechnen auf Körper-KS nicht notwendig.
      J_P_C_vec= inertiamatrix2vector(J_P_C);
      m_ges_Link(i) = m_P;
      [mrS_ges_Link(i,:), If_ges_Link(i,:)] = inertial_parameters_convert_par1_par2(r_P_P_C', J_P_C_vec, m_P);
    else
      error('Dieser Fall darf nicht eintreten');
    end
  end

end
% Addition der Dynamikparameter
m_ges = m_ges_Zus + m_ges_Link + m_ges_PStator + m_ges_PAbtrieb;
mrS_ges = mrS_ges_Zus + mrS_ges_Link + mrS_ges_PStator + mrS_ges_PAbtrieb;
If_ges = If_ges_Zus + If_ges_Link + If_ges_PStator + If_ges_PAbtrieb;
if any(m_ges<0)
  error('Eine Masse ist kleiner Null. Vorher war ein Fehler');
end
if any(isnan([If_ges(:);mrS_ges(:)]))
  error('Irgendein Dynamik-Parameter ist NaN. Das stört spätere Berechnungen!');
end
if R.Type == 0 
  % Seriell: Parameter direkt eintragen
  save(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_desopt_saveparamserial.mat'));
  R.update_dynpar2(m_ges, mrS_ges, If_ges)
else
  % PKM (symmetrisch): Parameter ohne Basis und ohne Segmente nach
  % Schnittgelenk; mit Plattform
  m_ges_pkm = m_ges([2:R.NQJ_LEG_bc+1, end]);
  mrS_ges_pkm = mrS_ges([2:R.NQJ_LEG_bc+1, end],:);
  If_ges_pkm = If_ges([2:R.NQJ_LEG_bc+1, end],:);
  R.update_dynpar2(m_ges_pkm, mrS_ges_pkm, If_ges_pkm);
end
%% Debug: Bilder der verschiedenen Dynamikparameter
if desopt_debug
  figure(2000);clf;
  sphdl = NaN(2,3);
  for i = 1:6
    sphdl(i) = subplot(2,3,i);
    if R.Type == 0
      I_pi = 1:R.NL;
    else
      I_pi = [2:R.NQJ_LEG_bc+1,length(m_ges)];
    end
    s = struct('mode', 3, 'ks', 1:R.NJ, 'straight', false);
    switch i
      case 1, R.update_dynpar2(m_ges_Link(I_pi,:), mrS_ges_Link(I_pi,:), If_ges_Link(I_pi,:)); t='Link';
      case 2, R.update_dynpar2(m_ges_PStator(I_pi,:), mrS_ges_PStator(I_pi,:), If_ges_PStator(I_pi,:)); t='PStator';
      case 3, R.update_dynpar2(m_ges_PAbtrieb(I_pi,:), mrS_ges_PAbtrieb(I_pi,:), If_ges_PAbtrieb(I_pi,:)); t='PAbtrieb';
      case 4, R.update_dynpar2(m_ges_Zus(I_pi,:), mrS_ges_Zus(I_pi,:), If_ges_Zus(I_pi,:)); t='Zus';
      case 5, R.update_dynpar2(m_ges(I_pi,:), mrS_ges(I_pi,:), If_ges(I_pi,:)); t='Gesamt';
      case 6, t='Ersatzgeometrie'; s=struct('mode', 4);
    end
    if R.Type == 0
      R.plot(Q(1,:)', s);
    else
      R.plot(Q(1,:)', Traj_0.X(1,:)', s);
    end
    title(t); xlabel('x');ylabel('y');zlabel('z'); view([0,90]);
  end
  linkaxes(sphdl);
end
end

function [m_s, J_S_C] = data_hollow_cylinder(R_i, e_i, L_i, density)
  % Berechne die mechanischen Daten des Hohlzylinders (entlang der x-Achse
  % des Geometrie-KS). Bezogen auf Schwerpunkt des Zylinders.
  % Eingabe:
  % R_i: Radius (außen)
  % e_i: Wandstärke
  % L_i: Länge
  % density: Dichte
  %
  % Ausgabe:
  % m_s: Masse
  % J_S_C: Trägheitstensor im Geometrie-KS
  
  % Modellierung Hohlzylinder. Siehe TM-Formelsammlung
  m_s = ((pi*(R_i^2)-(pi*((R_i-e_i)^2)))*abs(L_i)*density);
  % Trägheitstensor im Segment-KS (nicht: Körper-KS)
  J_S_C_xx=m_s/2*(R_i^2 + (R_i-e_i)^2); % Längsrichtung des Zylinders
  J_S_C_yy=m_s/4*(R_i^2 + (R_i-e_i)^2 + (L_i^2)/3);
  J_S_C_zz=J_S_C_yy;
  J_S_C = diag([J_S_C_xx; J_S_C_yy; J_S_C_zz]); % [A]/(7)
end