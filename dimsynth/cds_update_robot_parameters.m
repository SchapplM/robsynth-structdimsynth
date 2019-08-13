% Belegen der Roboterparameter für die Optimierung

function R_neu = cds_update_robot_parameters(R, Set, p)

R_neu = copy(R);

%% Strukturparameter der Kinematik
if R.Type == 0 % Seriell
  % Relevante Parameter sind die, die auch in Opt.Var. sind. TODO: Abspeichern
  Ipkinrel = R.get_relevant_pkin(Set.structures.DoF);
  IIpkinrel = find(Ipkinrel);
  pkin_voll = R.pkin;
  types = R.get_pkin_parameter_type();
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
  R_neu.update_mdh(pkin_voll);
elseif R.Type == 2 % Parallel
  error('TODO: Noch nicht implementiert');
end

%% Basis-Position

%% EE-Verschiebung

%% EE-Rotation
