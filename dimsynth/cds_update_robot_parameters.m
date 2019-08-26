% Belegen der Kinematikparameter für die Optimierung
% Die Dynamikparameter werden in der Entwurfsoptimierung bestimmt.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function R_neu = cds_update_robot_parameters(R, Set, Structure, p)

R_neu = R; % ohne copy-Befehl: Parameter werden direkt in Eingang geschrieben

%% Parameter prüfen
if p(1) == 0
  error('Roboterskalierung kann nicht Null werden');
end
if length(Structure.vartypes) ~= length(p)
  error('Nicht für alle Optimierungsvariablen ist ein Typ definiert');
end
%% Strukturparameter der Kinematik
if R_neu.Type == 0 || R_neu.Type == 2
  % Relevante Parameter sind die, die auch in Opt.Var. sind. TODO: Abspeichern
  if R_neu.Type == 0 % Seriell
    R_pkin = R_neu;
  else  % Parallel
    R_pkin = R_neu.Leg(1);
  end
  Ipkinrel = R_pkin.get_relevant_pkin(Set.structures.DoF);
  pkin_voll = R_pkin.pkin;
  j = 0;
  pkin_optvar = p(Structure.vartypes==1);
  for i = 1:length(pkin_voll)
    if ~Ipkinrel(i)
      continue
    end
    j = j + 1; % Index für Kinematikparameter in den Optimierungsvariablen
    if R_pkin.pkin_types(i) == 1 || R_pkin.pkin_types(i) == 3 || R_pkin.pkin_types(i) == 5
      pkin_voll(i) = pkin_optvar(j);
    else
      % Längenparameter skaliert mit Roboter-Skalierungsfaktor
      pkin_voll(i) = pkin_optvar(j)*p(1);
    end
  end
  if R_neu.Type == 0 % Seriell
    R_neu.update_mdh(pkin_voll);
  else  % Parallel
    for i = 1:R.NLEG
      R_neu.Leg(i).update_mdh(pkin_voll);
    end
  end
else
  error('Noch nicht implementiert');
end

%% Basis-Position
if Set.optimization.movebase
  p_basepos = p(Structure.vartypes == 2);
  r_W_0_neu = R.T_W_0(1:3,4);
  % xy-Punktkoordinaten der Basis skaliert mit Referenzlänge
  r_W_0_neu(Set.structures.DoF(1:2)) = p_basepos(Set.structures.DoF(1:2))*Structure.Lref;
  % z-Punktkoordinaten der Basis skaliert mit Roboter-Skalierungsfaktor
  r_W_0_neu(Set.structures.DoF(3)) = p_basepos(Set.structures.DoF(3))*p(1);
  
  R_neu.update_base(r_W_0_neu);
end

%% EE-Verschiebung
if Set.optimization.ee_translation
  p_eepos = p(Structure.vartypes == 3);
  r_N_E_neu = zeros(3,1);
  % EE-Versatz skaliert mit Roboter-Skalierungsfaktor
  r_N_E_neu(Set.structures.DoF(1:3)) = p_eepos.*p(1);
  R_neu.update_EE(r_N_E_neu);
end

%% EE-Rotation
if Set.optimization.ee_rotation
  error('Noch nicht implementiert');
end

%% Basis-Rotation
if Set.optimization.rotate_base
  error('Noch nicht implementiert');
end

%% Basis-Koppelpunkt Positionsparameter (z.B. Gestelldurchmesser)
if R_neu.Type == 2 && any(Structure.vartypes == 6)
  p_baseradius = p(Structure.vartypes == 6);
  if length(p_baseradius) ~= 1
    error('Mehr als ein Fußpunkt-Positionsparameter nicht vorgesehen');
  end
  if p_baseradius == 0
    error('Basis-Radius darf nicht Null werden');
  end
  % Setze den Fußpunkt-Radius skaliert mit Referenzlänge
  R_neu.align_base_coupling(1, p_baseradius*p(1));
end

%% Plattform-Koppelpunkt Positionsparameter (z.B. Plattformdurchmesser)
if R_neu.Type == 2 && any(Structure.vartypes == 7)
  p_pfradius = p(Structure.vartypes == 7);
  if length(p_pfradius) ~= 1
    error('Mehr als ein Plattform-Positionsparameter nicht vorgesehen');
  end
  if p_pfradius == 0
    error('Plattform-Radius darf nicht Null werden');
  end
  if ~any(Structure.vartypes == 6)
    error('Basis-Koppelpunkt muss zusammen mit Plattformkoppelpunkt optimiert werden');
  end
  % Setze den Plattform-Radius skaliert mit Gestell-Radius
  R_neu.align_platform_coupling(1, p_pfradius*p_baseradius);
end
