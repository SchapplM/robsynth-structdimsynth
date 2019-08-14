% Belegen der Roboterparameter für die Optimierung

function R_neu = cds_update_robot_parameters(R, Set, p)

R_neu = copy(R);

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
  types = R_pkin.get_pkin_parameter_type();
  j = 0;
  pkin_optvar = p(Set.optimization.vartypes==1);
  for i = 1:length(pkin_voll)
    if ~Ipkinrel(i)
      continue
    end
    j = j + 1; % Index für Kinematikparameter in den Optimierungsvariablen
    if types(i) == 1 || types(i) == 3 || types(i) == 5
      pkin_voll(i) = pkin_optvar(j);
    else
      % Längenparameter skaliert mit Referenzlänge
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
  p_basepos = p(Set.optimization.vartypes == 2);
  r_W_0_neu = zeros(3,1);
  % Punktkoordinaten der Basis skaliert mit Referenzlänge
  r_W_0_neu(Set.structures.DoF(1:3)) = p_basepos.*p(1);
  R_neu.update_base(r_W_0_neu);
end

%% EE-Verschiebung
if Set.optimization.ee_translation
  p_eepos = p(Set.optimization.vartypes == 3);
  r_N_E_neu = zeros(3,1);
  % EE-Versatz skaliert mit Referenzlänge
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
