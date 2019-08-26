% Entwurfsoptimierung des Robotermodells: Bestimmung von Dynamikparametern
% 
% Quelle:
% [A] Aufzeichnungen Schappler, 23.08.2019

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function R=cds_dimsynth_desopt(R, Q, Set, Structure)
save(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_desopt.mat'));
% Debug:
% load(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_desopt.mat'));

%% Definitionen, Konstanten
density = 2.7E3; %[kg/m^3] Aluminium

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
if Structure.Type == 0 || Structure.Type == 2
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
  mges_Zus = zeros(R_pkin.NL,1);
  mrS_ges_Zus = zeros(R_pkin.NL,3);
  If_ges_Zus = zeros(R_pkin.NL,6);
  mges_Zus(end) = Set.task.payload.m;
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
  % Dynamikparameter initialisieren
  mges_Link = NaN(R_pkin.NL,1);
  mrS_ges_Link = NaN(R_pkin.NL,3);
  If_ges_Link = NaN(R_pkin.NL,6);

  for i = 1:length(mges_Link)
    % Wandstärke und Radius der Hohlzylinder aus Robotereigenschaften
    if R_pkin.DesPar.seg_type(i) == 1 % Hohlzylinder
      e_i = R_pkin.DesPar.seg_par(i,1);
      R_i = 0.5*R_pkin.DesPar.seg_par(i,2);
    else
      error('Strukturmodell nicht definiert');
    end
    % Länge des Segments
    if i < size(mges_Link,1)
      % a- und d-Parameter werden zu Segment vor dem Gelenk gezählt (wg.
      % MDH-Notation) -> a1/d1 zählen zum Basis-Segment
      % [A]/(6)
      % Berechnet wird die maximale Länge des Segmentes mit Schubgelenk
      L_i = sqrt(R_pkin.MDH.a(i)^2+(R_pkin.MDH.d(i)+R.MDH.sigma(i)*q_range(i))^2);
      
      % Drehung des Segment-KS gegen das Körper-KS
      % [A]/(4)
      R_i_Si = rotx(R_pkin.MDH.alpha(i)) * roty(atan2(R_pkin.MDH.d(i), R_pkin.MDH.a(i)));
      
      % Probe: [A]/(10) (nur für Drehgelenke; wegen Maximallänge bei
      % Schubgelenk
      r_i_i_ip1_test = R_i_Si*[L_i;0;0];
      Tges=R_pkin.jtraf(zeros(R_pkin.NJ,1));
      r_i_i_ip1 = Tges(1:3,4,i);
      if R.MDH.sigma(i) == 0 && any(abs(r_i_i_ip1_test-r_i_i_ip1) > 1e-10)
        error('Segment-Darstellung stimmt nicht');
      end
    else
      % Letzter Körper: Flansch->EE bei Seriell; Plattform->EE bei Parallel
      if Structure.Type == 0 % Seriell
        L_i = norm(R_pkin.T_N_E(1:3,4));
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
      else % Parallel
        L_i = norm(R.T_P_E(1:3,4));
        % Annahme: Da die Plattform in der Mitte des Roboters ist, muss keine besondere
        % EE-Transformation berücksichtigt werden.
        R_i_Si = eye(3);
        if any(R.T_P_E(1:3,4))
          error('Der Vergleich zwischen Seriell und Parallel bei vorhandener Trafo P-E ist aktuell nicht möglich');
        end
      end
    end
    
    if Structure.Type == 0 || Structure.Type == 2 && i < size(mges_Link,1)
      % Hohlzylinder-Segment für Serielle Roboter und PKM
      
      % Modellierung Hohlzylinder. Siehe TM-Formelsammlung
      m_s = ((pi*(R_i^2)-(pi*((R_i-e_i)^2)))*L_i*density);
      % Schwerpunktskoordinate  im Segment-KS (nicht: Körper-KS)
      r_S_Oi_C = [L_i/2;0;0]; % % [A]/(5)
      % Trägheitstensor im Segment-KS (nicht: Körper-KS)
      J_S_C_xx=m_s/2*(R_i^2 + (R_i-e_i)^2); % Längsrichtung des Zylinders
      J_S_C_yy=m_s/4*(R_i^2 + (R_i-e_i)^2 + (L_i^2)/3);
      J_S_C_zz=J_S_C_yy;
      J_S_C = diag([J_S_C_xx; J_S_C_yy; J_S_C_zz]); % % [A]/(7)
      % Umrechnen auf Körper-KS;
      J_i_C = R_i_Si * J_S_C * R_i_Si'; % [A]/(8)
      r_i_Oi_C = R_i_Si*r_S_Oi_C; % [A]/(3)
      % Eintragen in Dynamik-Parameter (bezogen auf Ursprung)
      J_i_C_vec= inertiamatrix2vector(J_i_C);
      mges_Link(i) = m_s;
      [mrS_ges_Link(i,:), If_ges_Link(i,:)] = inertial_parameters_convert_par1_par2(r_i_Oi_C', J_i_C_vec, m_s);
    elseif Structure.Type == 2 && i == size(mges_Link,1)
      % Plattform-Segment bei PKM
      % Annahme: Kreisscheibe
      if R.DesPar.platform_method == 1
        R_P = R.DesPar.platform_par(1) / 2;
        e_P = R.DesPar.platform_par(2);
        
        % Modellierung Kreisscheibe (in Schwerpunkts-KS). Siehe TM-Formelsammlung
        m_P = density*pi*R_P^2*e_P;
        J_P_P_zz = 0.5 * m_P * R_P^2; % Normalenrichtung der Kreisscheibe
        J_P_P_xx = 0.5*J_P_P_zz;
        J_P_P_yy = 0.5*J_P_P_zz;
        r_P_P_C = zeros(3,1);
        J_P_C = diag([J_P_P_xx; J_P_P_yy; J_P_P_zz]);
        % Umrechnen auf Körper-KS nicht notwendig.
        J_P_C_vec= inertiamatrix2vector(J_P_C);
        mges_Link(i) = m_P;
        [mrS_ges_Link(i,:), If_ges_Link(i,:)] = inertial_parameters_convert_par1_par2(r_P_P_C', J_P_C_vec, m_P);
      else
        error('Dieser Fall darf nicht eintreten');
      end
    end
    
  end
  % Addition der Dynamikparameter
  mges = mges_Zus + mges_Link;
  mrS_ges = mrS_ges_Zus + mrS_ges_Link;
  If_ges = If_ges_Zus + If_ges_Link;
  if any(isnan([If_ges(:);mrS_ges(:)]))
    error('Irgendein Dynamik-Parameter ist NaN. Das stört spätere Berechnungen!');
  end
  if R.Type == 0 
    % Seriell: Parameter direkt eintragen
    R.update_dynpar2(mges, mrS_ges, If_ges)
  else
    % PKM (symmetrisch): Parameter ohne Basis und ohne Segmente nach
    % Schnittgelenk; mit Plattform
    mges_pkm = mges([2:R.NQJ_LEG_bc+1, end]);
    mrS_ges_pkm = mrS_ges([2:R.NQJ_LEG_bc+1, end],:);
    If_ges_pkm = If_ges([2:R.NQJ_LEG_bc+1, end],:);
    R.update_dynpar2(mges_pkm, mrS_ges_pkm, If_ges_pkm);
  end
end