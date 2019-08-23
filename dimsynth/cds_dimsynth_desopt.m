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
elseif R.Type == 2  % Parallel (symmetrisch)
  R.Leg(1).DesPar.seg_par = repmat([2e-3, 50e-3], R.NL, 1); % dünne Struktur
end

%% Dynamikparameter belegen
% Parameter des Struktursegmentes
if R.Type == 0 || R.Type == 2
  % Roboterklasse für Parametermodell heraussuchen
  if R.Type == 0 % Seriell
    R_pkin = R;
  else  % Parallel (symmetrisch)
    R_pkin = R.Leg(1);
  end
  %% EE-Zusatzlast
  mges_Zus = zeros(R_pkin.NL,1);
  mrS_ges_Zus = zeros(R_pkin.NL,3);
  If_ges_Zus = zeros(R_pkin.NL,6);
  if any(Set.task.payload.Ic(4:6)~=0) || any(diff(Set.task.payload.Ic(1:3)))
    error('Nicht symmetrische EE-Last noch nicht definiert');
  end
  if R.Type == 0 % Seriell
    mges_Zus(end) = Set.task.payload.m;
    r_N_N_E = R.T_N_E(1:3,4);

    [mrS_ges_Zus(end,:), If_ges_Zus(end,:)] = inertial_parameters_convert_par1_par2( ...
      r_N_N_E(:)', Set.task.payload.Ic(:)', Set.task.payload.m);
  else
    error('Zusatzlast für PKM noch nicht definiert');
  end
  %% Strukturteile
  % Länge von Schubgelenken herausfinden
  if R.Type == 0 % Seriell
    q_range = diff(minmax2(Q')')';
  else  % Parallel (symmetrisch)
    error('Noch nicht implementiert');
  end
  mges_Link = NaN(R_pkin.NL,1);
  mrS_ges_Link = NaN(R_pkin.NL,3);
  If_ges_Link = NaN(R_pkin.NL,6);
  for i = 1:R_pkin.NL
    % Wandstärke und Radius der Hohlzylinder aus Robotereigenschaften
    if R_pkin.DesPar.seg_type(i) == 1 % Hohlzylinder
      e_i = R_pkin.DesPar.seg_par(i,1);
      R_i = 0.5*R_pkin.DesPar.seg_par(i,2);
    else
      error('Strukturmodell nicht definiert');
    end
    % Länge des Segments
    if i < R_pkin.NL
      % a- und d-Parameter werden zu Segment vor dem Gelenk gezählt (wg.
      % MDH-Notation) -> a1/d1 zählen zum Basis-Segment
      % [A]/(6)
      L_i = sqrt(R_pkin.MDH.a(i)^2+(R_pkin.MDH.d(i)+R.MDH.sigma(i)*q_range(i))^2);
      
      % Drehung des Segment-KS gegen das Körper-KS
      % [A]/(4)
      R_i_Si = rotx(R_pkin.MDH.alpha(i)) * roty(atan2(R_pkin.MDH.d(i), R_pkin.MDH.a(i)));
      
      % Probe: [A]/(10)
      r_i_i_ip1_test = R_i_Si*[L_i;0;0];
      Tges=R_pkin.jtraf(zeros(R.NJ,1));
      r_i_i_ip1 = Tges(1:3,4,i);
      if any(abs(r_i_i_ip1_test-r_i_i_ip1) > 1e-10)
        error('Segment-Darstellung stimmt nicht');
      end
    else
      L_i = norm(R_pkin.T_N_E(1:3,4));
      % Rotationsmatrix muss noch bestimmt werden. z-Achse muss zum EE
      % zeigen
      if any(R_pkin.T_N_E(1:3,4))
        error('Noch nicht definiert');
      end
    end
    
    % Modellierung Hohlzylinder. Siehe TM-Formelsammlung
    m_s = ((pi*(R_i^2)-(pi*((R_i-e_i)^2)))*L_i*density);
    % Schwerpunktskoordinate  im Segment-KS (nicht: Körper-KS)
    r_S_Oi_C = [L_i/2;0;0]; % % [A]/(5)
    % Trägheitstensor im Segment-KS (nicht: Körper-KS)
    J_S_C_xx=(m_s/2)*((R_i^2 + (R_i-e_i)^2));
    J_S_C_yy=(m_s/4)*((R_i^2 + (R_i-e_i)^2)+((L_i^2)/3));
    J_S_C_zz=J_S_C_yy;
    J_S_C = diag([J_S_C_xx; J_S_C_yy; J_S_C_zz]); % % [A]/(7)
    % Umrechnen auf Körper-KS;
    J_i_C = R_i_Si * J_S_C * R_i_Si'; % [A]/(8)
    r_i_Oi_C = R_i_Si*r_S_Oi_C; % [A]/(3)
    
    % Eintragen in Dynamik-Parameter (bezogen auf Ursprung)
    J_i_C_vec= inertiamatrix2vector(J_i_C);
    mges_Link(i) = m_s;
    [mrS_ges_Link(i,:), If_ges_Link(i,:)] = inertial_parameters_convert_par1_par2(r_i_Oi_C', J_i_C_vec, m_s);
  end
  if R.Type == 0
    % Addition der Dynamikparameter
    mges = mges_Zus + mges_Link;
    mrS_ges = mrS_ges_Zus + mrS_ges_Link;
    If_ges = If_ges_Zus + If_ges_Link;
    R.update_dynpar2(mges, mrS_ges, If_ges)
  else
    error('Noch nicht definiert');
  end
end