% Aktualisierung der Informationen zum Interaktionsraum
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Ausgabe:
% collbodies_iaspc
%   Struktur mit Interaktionsarbeitsraum definiert im Basis-KS
% 
% Siehe auch: cds_update_collbodies.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function collbodies_iaspc = cds_update_interactionspace(R, Set)

  n_cb_iaspc = size(Set.task.interactionspace.type,1);

  collbodies_iaspc = struct('link', zeros(n_cb_iaspc,2), ...
    'type', Set.task.interactionspace.type, ...
    'params', Set.task.interactionspace.params);

  % Transformiere Körper ins Basis-KS
  T_0_W = R.T_0_W;
  for i = 1:n_cb_iaspc
    type_i = Set.task.interactionspace.type(i);
    % Umrechnung der Parameter ins Basis-KS
    params_W = Set.task.interactionspace.params(i,:);
    if type_i == 1 % Quader
      params_0 = transform_box(params_W, T_0_W);
      % Setze Typ auf "Quader im Basis-KS". Information ist notwendig für
      % automatische Verarbeitung (im Gegensatz zu "körperfester Quader").
      type_i = uint8(10);
    elseif type_i == 2 % Zylinder
      params_0 = transform_cylinder(params_W, T_0_W);
      % Setze Typ auf "Zylinder im Basis-KS". Information ist notwendig für
      % automatische Verarbeitung (im Gegensatz zu "körperfester Zylinder").
      type_i = uint8(12);
    elseif type_i == 3 % Kapsel
      params_0 = transform_capsule(params_W, T_0_W);
      % Setze Typ auf "Kapsel im Basis-KS". Information ist notwendig für
      % automatische Verarbeitung (im Gegensatz zu "körperfeste Kapsel").
      type_i = uint8(13);
    else
      error('Fall %d nicht definiert', type_i);
    end
    % Eintragen Liste, in der auch schon die Roboter-Objekte stehen
    collbodies_iaspc.params(i,:) = params_0;
    collbodies_iaspc.type(i,1) = type_i;
    % Bauraum wird zur Basis (=0) gezählt (ortsfest)
    collbodies_iaspc.link(i,:) = uint16([0,0]);
  end
end

% Umrechnung von Welt- in Basis-KS. Siehe cds_update_collbodies.m
function params_0 = transform_box(params_W, T_0_W)
params_0 = [eye(3,4)*T_0_W*[params_W(1:3)';1]; ... % Aufpunkt
            T_0_W(1:3,1:3)*params_W(4:6)'; ... % Richtungsvektoren
            T_0_W(1:3,1:3)*params_W(7:9)'; params_W(10)]';
end

function params_0 = transform_cylinder(params_W, T_0_W)
params_0 = [eye(3,4)*T_0_W*[params_W(1:3)';1]; ... % Punkt 1
            eye(3,4)*T_0_W*[params_W(4:6)';1]; ... % Punkt 2
            params_W(7); NaN(3,1)]'; % Radius, auffüllen auf Array-Größe
end
function params_0 = transform_capsule(params_W, T_0_W)
% identische Funktion wie für Zylinder
params_0 = transform_cylinder(params_W, T_0_W);
end