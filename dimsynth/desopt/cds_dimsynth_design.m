% Entwurfsparameter in Robotermodell einsetzen: Bestimmung von Dynamikparametern
% 
% Unterscheidung von Koordinatensystemen:
% Körper: Den einzelnen Starrkörpern des Robotermodells zugeordnet
% Geometrie: Einzelnen Ersatzkörpern (z.B. Zylindern) zugeordnet
% 
% Eingabe:
% Q
%   Gelenkwinkel-Trajektorie
% Set
%   Einstellungen des Optimierungsalgorithmus
% Structure
%   Eigenschaften der Roboterstruktur
% p_desopt
%   Parameter der Entwurfsvariablen: Wandstärke und Durchmesser der
%   Segmente (modelliert als Hohlzylinder)
% 
% Ausgabe:
% R
%   Aktualisiertes Klassenobjekt des Roboters (nicht zwangsläufig
%   benötigt), da die Parameter auch per Handle im Eingabe-Objekt
%   aktualisiert werden.
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

function R=cds_dimsynth_design(R, Q, Set, Structure, p_desopt)
desopt_debug = false;
if nargin < 5 || ~any(strcmp(Set.optimization.desopt_vars, 'linkstrength'))
  use_default_link_param = true;
else
  use_default_link_param = false;
end
if Set.general.matfile_verbosity > 2 + (~use_default_link_param) % weniger oft speichern, wenn Aufruf in desopt_fitness
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_design.mat'));
end
% Debug (dafür obiges auskommentieren und folgendes einkommentieren):
% function R=cds_dimsynth_design()
% desopt_debug = true;
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_design.mat'));

%% Definitionen, Konstanten
density = 2.7E3; %[kg/m^3] Aluminium

%% Entwurfsparameter ändern
% Entweder aus Optimierungsvariable oder konstante Standard-Werte
if R.Type == 0
  if use_default_link_param
    % Keine Entwurfsoptimierung, nehme die Standardwerte für
    % Segmentparameter (dicke Struktur für seriell)
    % Sollte konsistent mit cds_dimsynth_desopt sein.
    R.DesPar.seg_par = repmat([5e-3, 80e-3], R.NL, 1);
  else
    % Nehme die Segmentparameter aus Eingabeargument
    R.DesPar.seg_par = repmat([p_desopt(1), p_desopt(2)], R.NL, 1);
  end
  % R.DesPar.seg_par(end,:) = [5e-3, 300e-3]; % EE anders zum Testen der Visu.
elseif R.Type == 2  % Parallel (symmetrisch)
  for i = 1:R.NLEG
    % Belege alle Beinketten, damit in Plot-Funktion nutzbar. Es werden
    % eigentlich nur die Werte der ersten Beinkette für Berechnungen
    % genutzt
    if use_default_link_param
      % Standardwerte: Dünne Struktur für PKM-Beine. Je mehr Beine, desto
      % mehr wird die Last aufgeteilt. Siehe cds_dimsynth_desopt
      R.Leg(i).DesPar.seg_par = repmat([5e-3, 80e-3]/(R.NLEG/2), R.Leg(i).NL, 1);
    else
      R.Leg(i).DesPar.seg_par = repmat([p_desopt(1), p_desopt(2)], R.Leg(i).NL, 1);
    end
  end
  if     any(Structure.Coupling(2) == [1:3 7]), i_plfthickness = 2; %#ok<ALIGN>
  elseif any(Structure.Coupling(2) == [4:6 8]), i_plfthickness = 3;
  else,  error('Plattform-Koppelgelenk-Methode %d nicht implementiert', Structure.Coupling(2)); end
  R.DesPar.platform_par(i_plfthickness) = 10e-3; % Dünne Platte als Plattform
end

%% Dynamikparameter belegen
% Parameter des Struktursegmentes
% Roboterklasse für Parametermodell heraussuchen
if R.Type == 0 % Seriell
  R_pkin = R;
else  % Parallel (symmetrisch)
  R_pkin = R.Leg(1);
end

%% Debug: Roboter zeichnen; in dieses Bild werden Debug-KS eingezeichnet
if desopt_debug
  change_current_figure(3000);clf;
  subplot(1,2,1); hold on; view(3); axis auto; grid on; view([0,90])
  s = struct('mode', 1, 'ks', 1:R.NJ, 'straight', false);
  R_pkin.plot(Q(1,1:R_pkin.NJ)', s); xlabel('x in m');ylabel('y in m');zlabel('z in m');
  title('winklig (a/d erkennbar)');
  subplot(1,2,2); hold on; view(3); axis auto; grid on; view([0,90])
  s = struct('mode', 1, 'ks', 1:R.NJ, 'straight', true);
  R_pkin.plot(Q(1,1:R_pkin.NJ)', s); xlabel('x in m');ylabel('y in m');zlabel('z in m');
  title('direkt (Modell)');
end

%% EE-Zusatzlast
% Dynamikparameter für Robotermodell belegen
m_ges_Zus = zeros(R_pkin.NL,1);
mrS_ges_Zus = zeros(R_pkin.NL,3);
If_ges_Zus = zeros(R_pkin.NL,6);
m_ges_Zus(end) = Set.task.payload.m;
r_E_E_S = Set.task.payload.rS; % Schwerpunkt bzgl. EE-KS
% Trägheitstensor in Plattform-KS rotieren
Ic_E = inertiavector2matrix(Set.task.payload.Ic');
if R.Type == 0 % Seriell (Benennung P=N für Konsistenz mit PKM)
  r_P_P_S = R.T_N_E(1:3,4) + R.T_N_E(1:3,1:3) * r_E_E_S;
  Ic_P = R.T_N_E(1:3,1:3)' * Ic_E * R.T_N_E(1:3,1:3);
else
  r_P_P_S = R.T_P_E(1:3,4) + R.T_P_E(1:3,1:3) * r_E_E_S;
  Ic_P = R.T_P_E(1:3,1:3)' * Ic_E * R.T_P_E(1:3,1:3);
end
[mrS_ges_Zus(end,:), If_ges_Zus(end,:)] = inertial_parameters_convert_par1_par2( ...
  r_P_P_S(:)', inertiamatrix2vector(Ic_P), Set.task.payload.m);
%% Strukturteile
% Länge von Schubgelenken herausfinden
q_minmax = NaN(R.NJ, 2);
q_minmax(R.MDH.sigma==1,:) = minmax2(Q(:,R.MDH.sigma==1)');
if Structure.Type == 0 % Seriell
  q_range = diff(q_minmax')';
else  % Parallel (symmetrisch)
  % Siehe cds_update_collbodies; nehme eine identische Führungsschiene
  % aller PKM-Beinketten an, auch wenn die Bewegung nicht symmetrisch ist.
  q_minmax_sym = q_minmax(R.I1J_LEG(1):R.I2J_LEG(1),:);
  % Grenzen durch Min-/Max-Werte aller Beinketten finden
  for i = 2:R.NLEG
    q_minmax_sym = minmax2([q_minmax_sym, q_minmax(R.I1J_LEG(i):R.I2J_LEG(i),:)]);
  end
  q_range = diff(q_minmax_sym')';
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
    if R_pkin.MDH.sigma(i) == 0 % Drehgelenk
      % Nehme nur die konstanten a- und d-Parameter als "schräge"
      % Verbindung der Segmente
      r_i_i_D = [R_pkin.MDH.a(i);0;0] + rotx(R_pkin.MDH.alpha(i))*[0;0;R_pkin.MDH.d(i)]; % [B]
      % [A]/(4); Drehe das Segment-KS so, dass die Längsachse des
      % Segments in x-Richtung dieses KS zeigt
      R_i_Si = rotx(R_pkin.MDH.alpha(i)) * roty(-atan2(R_pkin.MDH.d(i), R_pkin.MDH.a(i)));
    elseif R_pkin.DesPar.joint_type(i) == 1 % Schubgelenk ohne spezielles Modell
      % Dieses Modell ist sehr ungenau und berücksichtigt
      % nicht den Anfangspunkt des Schubgelenks
      r_i_i_D = [R_pkin.MDH.a(i); 0; q_range(i)];
      R_i_Si = rotx(R_pkin.MDH.alpha(i)) * roty(atan2(q_range(i), R_pkin.MDH.a(i)));
    elseif any(isinf(R_pkin.qlim(i,:))) % Schubgelenk mit unendlichen Grenzen
      % Für unendliche Grenzen kann kein physikalisches Modell des Schub-
      % gelenks aufgestellt werden. Untersuchung ist rein kinematisch.
      r_i_i_D = zeros(3,1); % Damit Masse zu Null setzen
      R_i_Si = eye(3); % spielt keine Rolle
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
          % TODO: Variable joint_offset noch nicht korrekt implementiert.
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
    % Berücksichtige den beta-Parameter (MDH-Notation nach Khalil). Das
    % vorher benutzte KS i ist das KS vor Berücksichtigung der beta-Trafo
    if R_pkin.MDH.beta(i) ~= 0
      R_i_Si = rotz(R_pkin.MDH.beta(i))*R_i_Si;
      r_i_i_D = rotz(R_pkin.MDH.beta(i))*r_i_i_D;
    end
    % Probe, ob Orientierung (R_i_Si) und berechnetes Ende des Segments (r_i_i_D) stimmen
    if any( abs(r_i_i_D - R_i_Si*[norm(r_i_i_D);0;0] ) > 1e-10)
      save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_design_error.mat'));
      error('Richtung des Segments stimmt nicht');
    end
    % Probe: [A]/(10) (nur für Drehgelenke; wegen Maximallänge bei Schubgelenk)
    r_i_i_ip1_test = R_i_Si*[norm(r_i_i_D);0;0];
    Tges=R_pkin.jtraf(zeros(R_pkin.NJ,1));
    r_i_i_ip1 = Tges(1:3,4,i);
    if R_pkin.MDH.sigma(i) == 0 && any(abs(r_i_i_ip1_test-r_i_i_ip1) > 1e-10)
      save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_design_error.mat'));
      error('Segment-Darstellung stimmt nicht');
    end
    %% Bestimme zusätzliche Dynamikparameter für Schubgelenke:
    % (nur, falls Grenzen auf endliche Werte gesetzt sind)
    if R_pkin.MDH.sigma(i) == 1 && ~any(isinf(R_pkin.qlim(i,:)))
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
        % zugeordnet. Maße müssen konsistent mit Außenzylinder sein.
        l_innercyl = q_range(i);
        [m_s, J_S_C] = data_hollow_cylinder(R_i-e_i, min(e_i*1.5,R_i-e_i), l_innercyl, density);
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
        % Schwerpunkt in der Mitte der Führung. Berücksichtige die Verschiebung
        % der Führungsschiene durch den Offset.
        r_i_Oi_C = [R_pkin.MDH.a(i);0;0] + rotx(R_pkin.MDH.alpha(i))*...
          [0;0;R_pkin.qlim(i,1)+R_pkin.DesPar.joint_offset(i)+0.5*q_range(i)];
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
      %% Berücksichtige den Gelenkwinkel-Offset als eigenes Segment
      % Zwischen Schubgelenk und nachfolgendem Gelenk. Die Verbindung
      % entspricht einer mitbewegten Masse.
      if R_pkin.DesPar.joint_offset(i)
        % Länge entspricht dem Offset (Verlängerungsstange zum nächsten
        % Gelenk)
        l = R_pkin.DesPar.joint_offset(i);
        % Hohlzylinder-Annahme wie restlicher Roboter
        [m_o, J_B_C] = data_hollow_cylinder(R_i, e_i, l, density);
        % Gleiche Richtung wie Schubachse (starre Verbindung damit)
        % (siehe Code zu Führungsschiene)
        R_i_B = rotx(R_pkin.MDH.alpha(i))*roty(-pi/2);
        J_i_C = R_i_B * J_B_C * R_i_B'; % Rotation ins Körper-KS
        % Der Schwerpunkt wird nach der Trafo des Schubgelenks gezählt. Der
        % Offset muss also wieder rückgängig gemacht werden.
        r_i_Oi_C = [R_pkin.MDH.a(i);0;0] + rotx(R_pkin.MDH.alpha(i))*...
          [0;0;-0.5*R_pkin.DesPar.joint_offset(i)];
        % Zähle das Segment als Abtrieb (ähnlich wie ausfahrbaren Zylinder)
        % Daher zum nächsten Segment hinzuzählen
        m_ges_PAbtrieb(i+1) = m_o;
        [mrS_ges_PAbtrieb(i+1,:), If_ges_PAbtrieb(i+1,:)] = inertial_parameters_convert_par1_par2( ...
          r_i_Oi_C', inertiamatrix2vector(J_i_C), m_o);
      end
    end
  else
    % Letzter Körper: Flansch->EE bei Seriell; Plattform->EE bei Parallel
    if Structure.Type == 0 % Serieller Roboter
      r_i_i_D = R_pkin.T_N_E(1:3,4);
      T_P_E = R_pkin.T_N_E; % Für gleiche Benennung seriell/parallel
    else
      r_i_i_D = R.T_P_E(1:3,4);
      T_P_E = R.T_P_E;
    end
    % Rotationsmatrix für das letzte Segment-KS
    % Die x-Achse muss zum EE zeigen. Die anderen beiden Achsen sind
    % willkürlich senkrecht dazu.
    if ~all(r_i_i_D == 0)
      x_Si = r_i_i_D / norm(r_i_i_D);
      if dot(x_Si, T_P_E(1:3,1)) > 0.99 % Verlängerung zeigt nur in x-Richtung
        y_Si = T_P_E(1:3,2); % wähle y-Achse so wie in EE-KS
      else
        y_Si = cross(x_Si, T_P_E(1:3,1)); % y-Achse neu wählen
        y_Si = y_Si/norm(y_Si);
      end
      z_Si = cross(x_Si, y_Si); % senkrecht dazu
      R_i_Si = [x_Si, y_Si, z_Si];
    else
      R_i_Si = eye(3); % Es gibt keinen Abstand zum letzten KS. EE hat keine eigene Ausdehnung
    end
    if abs(det(R_i_Si)-1) > 1e-6 || any(isnan(R_i_Si(:)))
      save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_design_error.mat'));
      error('Letztes Segment-KS stimmt nicht');
    end
  end
  %% Segment-Geometrie als Dynamikparameter berechnen
  % Hohlzylinder-Segment für Serielle Roboter und PKM-Beinketten sowie
  % für EE-Verbindungen (vom Plattform- oder Flansch-KS zum EE)
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
  %% Dynamik-Parameter für Plattform
  if Structure.Type == 2 && i == size(m_ges_Link,1)
    % Plattform-Segment bei PKM
    % Annahme: Kreisscheibe. Plattform-Parameter siehe align_platform_coupling
    % (ist nur eine Näherung bei paarweiser Anordnung)
    if any(R.DesPar.platform_method == [1:6 7 8])
      R_P = R.DesPar.platform_par(1); % Parameter 1 ist Radius der Plattform
      e_P = R.DesPar.platform_par(i_plfthickness); % Dicke der Plattform

      % Modellierung Kreisscheibe (in Schwerpunkts-KS). Siehe TM-Formelsammlung
      m_P = density*pi*R_P^2*e_P;
      J_P_P_zz = 0.5 * m_P * R_P^2; % Normalenrichtung der Kreisscheibe
      J_P_P_xx = 0.5*J_P_P_zz; J_P_P_yy = 0.5*J_P_P_zz;
      r_P_P_C = zeros(3,1);
      J_P_C = diag([J_P_P_xx; J_P_P_yy; J_P_P_zz]);
      % Umrechnen auf Körper-KS nicht notwendig.
      J_P_C_vec= inertiamatrix2vector(J_P_C);
      [mrS_P, If_P] = inertial_parameters_convert_par1_par2(r_P_P_C', J_P_C_vec, m_P);
      % Addieren zur bisherigen Masse der EE-Trafo (P-E)
      m_ges_Link(i) = m_ges_Link(i) + m_P;
      mrS_ges_Link(i,:) = mrS_ges_Link(i,:) + mrS_P;
      If_ges_Link(i,:) = If_ges_Link(i,:) + If_P;
    else
      error('platform_method %d ist nicht implementiert', R.DesPar.platform_method);
    end
  end

end
% Addition der Dynamikparameter
m_ges   =   m_ges_Link +   m_ges_PStator +   m_ges_PAbtrieb;
mrS_ges = mrS_ges_Link + mrS_ges_PStator + mrS_ges_PAbtrieb;
If_ges  =  If_ges_Link +  If_ges_PStator +  If_ges_PAbtrieb;
% Parameter zu Null setzen (außer die aus Set.task.payload).
if R.Type == 0
  if Set.optimization.nolinkmass
    m_ges(:) = 0; mrS_ges(:) = 0; If_ges(:) = 0;
  end
else
  if Set.optimization.nolinkmass
    m_ges(1:end-1) = 0; mrS_ges(1:end-1,:) = 0; If_ges(1:end-1,:) = 0;
  end
  if Set.optimization.noplatformmass
    m_ges(end) = 0; mrS_ges(end,:) = 0; If_ges(end,:) = 0;
  end
end
% Zusätzliche Masse der Traglast erst hier einsetzen. Falls der Wert Null
% gewünscht wäre, würde man das direkt in Set.task.payload machen
m_ges = m_ges + m_ges_Zus;
mrS_ges = mrS_ges + mrS_ges_Zus;
If_ges = If_ges + If_ges_Zus;

if any(m_ges<0)
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_design_error.mat'));
  error('Eine Masse ist kleiner Null. Vorher war ein Fehler');
end
if any(isnan([If_ges(:);mrS_ges(:);m_ges(:)]))
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_design_error.mat'));
  error('Irgendein Dynamik-Parameter ist NaN. Das stört spätere Berechnungen!');
end
if Set.general.matfile_verbosity > 2 + (~use_default_link_param)
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_design_saveparam.mat'));
end
if R.Type == 0 
  % Seriell: Parameter direkt eintragen
  R.update_dynpar2(m_ges, mrS_ges, If_ges)
else
  % PKM (symmetrisch): Parameter ohne Basis und ohne Segmente nach
  % Schnittgelenk; mit Plattform
  % Als Schnittgelenk wird nur noch das letzte (Dreh-)Gelenk aufgefasst.
  % Falls das letzte Gelenk ein Kugelgelenk ist, wären die Massenparameter
  % trotzdem Null (da Segment ohne Länge)
  m_ges_pkm = [m_ges(2:end-1);0;m_ges(end)]; % m_ges([2:R.NQJ_LEG_bc+1, end])
  mrS_ges_pkm = [mrS_ges(2:end-1,:);zeros(1,3);mrS_ges(end,:)]; % ([2:R.NQJ_LEG_bc+1, end],:)
  If_ges_pkm = [If_ges(2:end-1,:);zeros(1,6);If_ges(end,:)]; % ([2:R.NQJ_LEG_bc+1, end],:)
  R.update_dynpar2(m_ges_pkm, mrS_ges_pkm, If_ges_pkm);
end
%% Debug: Bilder der verschiedenen Dynamikparameter
if desopt_debug
  change_current_figure(2000);clf;
  sphdl = NaN(2,3);
  for i = 1:6
    sphdl(i) = subplot(2,3,i); hold on;
    s = struct('mode', 3, 'ks', 1:R.NJ, 'straight', true);
    % Setze in die Roboterklasse nach und nach die einzelnen Komponenten
    % der Dynamikparameter ein und plotte jeweils. Am Ende Gesamt-Parameter
    % einsetzen (damit keine Beeinflussung folgender Berechnungen)
    switch i % Spalten für ID_ges_i: Masse, Schwerpunkt, Trägheitstensor
      case 1, ID_ges_i = [m_ges_Link,    mrS_ges_Link,    If_ges_Link];    t='Link';
      case 2, ID_ges_i = [m_ges_PStator, mrS_ges_PStator, If_ges_PStator]; t='PStator';
      case 3, ID_ges_i = [m_ges_PAbtrieb,mrS_ges_PAbtrieb,If_ges_PAbtrieb];t='PAbtrieb';
      case 4, ID_ges_i = [m_ges_Zus,     mrS_ges_Zus,     If_ges_Zus];     t='Zus';
      case 5, ID_ges_i = [m_ges,         mrS_ges,         If_ges];         t='Gesamt';
      case 6, t='Ersatzgeometrie'; s=struct('mode', 4);
    end
    if R.Type == 0 % Seriell
      R.update_dynpar2(ID_ges_i(:,1), ID_ges_i(:,2:4), ID_ges_i(:,5:10));
    else % PKM
      ID_ges_i_pkm = [ID_ges_i(2:end-1,:); zeros(1,10); ID_ges_i(end,:)];
      R.update_dynpar2(ID_ges_i_pkm(:,1), ID_ges_i_pkm(:,2:4), ID_ges_i_pkm(:,5:10));
    end
    if R.Type == 0 % Seriell
      R.plot(Q(1,:)', s);
    else % PKM
      % Pose der Plattform berechnen (steht in dieser Funktion nicht zur
      % Verfügung, da nicht zur Berechnung benötigt).
      T_0A1 = R.Leg(1).T_W_0;
      T_A1E1 = R.Leg(1).fkineEE(Q(1,R.I1J_LEG(1):R.I2J_LEG(1))');
      T_P_B1 = rt2tr(eulxyz2r(R.phi_P_B_all(:,1)), R.r_P_B_all(:,1));
      x = R.t2x(T_0A1*T_A1E1*invtr(T_P_B1)*R.T_P_E);
      R.plot(Q(1,:)', x, s);
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
