% Gebe Details zu den Zielfunktionen aus. Wird in Plot-Funktionen genutzt.
% 
% Eingabe
% Set
%   Struktur mit Einstellungen.
% Structure
%   Cell-Array mit Kenngrößen der zu optimierenden Roboterstrukturen
% Ausgabe:
% obj_units
%   Einheiten der physikalischen Werte der Zielfunktionen
% objscale
%   Skalierung der physikalischen Werte im Plot (z.B. Grad/Bogenmaß)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [obj_units, objscale] = cds_objective_plotdetails(Set, Structures)
% Initialisierung
obj_units = cell(1,length(Set.optimization.objective));
objscale = ones(length(Set.optimization.objective),1);
% Prüfe für alle untersuchten Strukturen die Art der Antriebe
acttype = ''; % 'revolute', 'prismatic' oder 'mixed'. Siehe cds_gen_robot_list
if nargin == 2
  for i = 1:length(Structures)
    % Belege mit erstem Vorkommnis
    if isempty(acttype), acttype = Structures{i}.act_type; end
    if strcmp(acttype, Structures{i}.act_type)
      continue
    else
      acttype = 'mixed';
      break; % Die Roboter haben gemischte Antriebsgelenke
    end % nichts tun
  end
end
% Zielkriterien durchgehen
for jj = 1:length(Set.optimization.objective)
  if strcmp(Set.optimization.objective{jj}, 'valid_act')
    obj_units{jj} = 'unitless'; % Rangverlust ist nur eine Zahl. Plot nicht vorgesehen.
  elseif strcmp(Set.optimization.objective{jj}, 'mass')
    obj_units{jj} = 'kg';
  elseif strcmp(Set.optimization.objective{jj}, 'condition')
    obj_units{jj} = 'units of cond(J)';
  elseif strcmp(Set.optimization.objective{jj}, 'energy')
    obj_units{jj} = 'J';
  elseif strcmp(Set.optimization.objective{jj}, 'power')
    obj_units{jj} = 'W';
  elseif strcmp(Set.optimization.objective{jj}, 'actforce')
    if strcmp(acttype, 'prismatic')
      obj_units{jj} = 'N';
    elseif strcmp(acttype, 'revolute')
      obj_units{jj} = 'Nm';
    else
      obj_units{jj} = 'N or Nm';
    end
  elseif strcmp(Set.optimization.objective{jj}, 'materialstress')
    obj_units{jj} = '%';
    objscale(jj) = 100;
  elseif strcmp(Set.optimization.objective{jj}, 'stiffness')
    obj_units{jj} = 'm/N';
  elseif strcmp(Set.optimization.objective{jj}, 'jointrange')
    if Set.optimization.obj_jointrange.only_revolute || ...
        Set.optimization.obj_jointrange.only_passive
      % Annahme: Es gibt keine passiven Schubgelenke.
      obj_units{jj} = 'deg';
      objscale(jj) = 180/pi;
    else
      obj_units{jj} = 'rad or m'; % Einheit nicht bestimmbar und evtl gemischt
    end
  elseif strcmp(Set.optimization.objective{jj}, 'jointlimit')
    obj_units{jj} = 'rad or m'; % Einheit hier nicht bestimmbar und evtl gemischt
  elseif strcmp(Set.optimization.objective{jj}, 'manipulability')
    obj_units{jj} = 'units of cond(J)';
  elseif strcmp(Set.optimization.objective{jj}, 'minjacsingval')
    obj_units{jj} = 'units of cond(J)';
  elseif strcmp(Set.optimization.objective{jj}, 'positionerror')
    obj_units{jj} = 'µm';
    objscale(jj) = 1e6;
  elseif strcmp(Set.optimization.objective{jj}, 'chainlength')
    obj_units{jj} = 'mm';
    objscale(jj) = 1e3;
  elseif strcmp(Set.optimization.objective{jj}, 'actvelo')
    if strcmp(acttype, 'prismatic')
      obj_units{jj} = 'm/s';
    elseif strcmp(acttype, 'revolute')
      obj_units{jj} = 'rad/s';
    else
      obj_units{jj} = 'rad/s or m/s';
    end
    objscale(jj) = 1;
  elseif strcmp(Set.optimization.objective{jj}, 'installspace')
    obj_units{jj} = 'm³';
  elseif strcmp(Set.optimization.objective{jj}, 'footprint')
    obj_units{jj} = 'm²';
  elseif strcmp(Set.optimization.objective{jj}, 'colldist')
    obj_units{jj} = 'mm';
    objscale(jj) = 1e3;
  else
    error('Zielfunktion %s nicht vorgesehen', Set.optimization.objective{jj});
  end
end
