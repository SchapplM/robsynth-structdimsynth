% Gebe Details zu den Zielfunktionen aus. Wird in Plot-Funktionen genutzt.
% 
% Eingabe
% Set
%   Struktur mit Einstellungen.
% Ausgabe:
% obj_units
%   Einheiten der physikalischen Werte der Zielfunktionen
% objscale
%   Skalierung der physikalischen Werte im Plot (z.B. Grad/Bogenmaß)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [obj_units, objscale] = cds_objective_plotdetails(Set)

obj_units = cell(1,length(Set.optimization.objective));
objscale = ones(length(Set.optimization.objective),1);
for jj = 1:length(Set.optimization.objective)
  if strcmp(Set.optimization.objective{jj}, 'valid_act')
    obj_units{jj} = 'unitless'; % Rangverlust ist nur eine Zahl. Plot nicht vorgesehen.
  elseif strcmp(Set.optimization.objective{jj}, 'mass')
    obj_units{jj} = 'kg';
  elseif strcmp(Set.optimization.objective{jj}, 'condition')
    obj_units{jj} = 'units of cond(J)';
  elseif strcmp(Set.optimization.objective{jj}, 'energy')
    obj_units{jj} = 'J';
  elseif strcmp(Set.optimization.objective{jj}, 'actforce')
    obj_units{jj} = 'N or Nm';
  elseif strcmp(Set.optimization.objective{jj}, 'materialstress')
    obj_units{jj} = 'in %';
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
    obj_units{jj} = '{rad/m}/s';
    objscale(jj) = 1;
  elseif strcmp(Set.optimization.objective{jj}, 'installspace')
    obj_units{jj} = 'm³';
  else
    error('Zielfunktion %s nicht vorgesehen', Set.optimization.objective{jj});
  end
end