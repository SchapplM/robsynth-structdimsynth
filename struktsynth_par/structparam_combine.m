% Kombiniere die Ausdrücke für freie Winkelparameter wie alpha und theta
% Gebe eine minimale Form des Ausdrucks zurück.
% 
% Eingabe:
% angles_valid (1xnc cell array)
%   Enthält Einträge mit Kombinationen möglicher Parameter in der Form
%   {'pp', 'pa'}. Reihenfolge entsprechend `params_names`. Bedeutung, siehe
%   parroblib_load_robot
%   p - parallel
%   o - orthogonal
%   b - beide (parallel und orthogonal)
%   a - "arbitrary" (beliebige Parameterwerte)
% 
% Ausgabe:
% angles_valid_red (1xnr cell array)
%   Gleiches Format wie `angles_valid`, aber reduzierte Einträge.
%   Wenn bspw. ein Parameter mit 'a' für "beliebig" gekennzeichnet ist,
%   wird der zugehörige Eintrag 'p' (Parallel -> 0°) entfernt.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-11
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function angles_valid_red = structparam_combine(angles_valid)
% Dummy-Namen der Parameter vorbereiten
if isempty(angles_valid)
  angles_valid_red = angles_valid;
  return
end
params_names = cell(1,length(angles_valid{1}));
for i = 1:length(params_names)
  % Laufende Nummer des Parameters. Wahl des Namens auch in der Form
  % "alpha1" oder "theta1" möglich, aber nicht notwendig.
  params_names{i} = sprintf('%d', i);
end

% Erzeuge einen symbolischen Ausdruck aus den möglichen Kombinationen der
% einzelnen Parameter.
angles_expr = sym(zeros(length(angles_valid),1));
for j = 1:length(angles_valid)
  tmp = 1;
  for k = 1:length(params_names)
    % Kombinationen entsprechen Multiplikationen. Erzeuge eindeutige temp.
    % Variablen wie "p1" oder "o2". Die Nummer bezieht sich nur auf die
    % laufende Nummer des Parameters
    tmp = tmp * sym([angles_valid{j}(k),params_names{k}]);
  end
  angles_expr(j) = tmp;
end
% Ausdrücke addieren
aes = sum(angles_expr);
% Vereinfachung des Ausdrucks. Durch das Ausklammern wird die
% Vereinfachungslogik vorbereitet
aes = simplify(aes);
% Vereinfachungsregeln anwenden
for ii = 1:length(params_names) % alle Parameter durchgehen
  % Ersetzungsausdrücke definieren (entsprechend den tmp-Var. oben.
  a=sym(['a',params_names{ii}]);
  b=sym(['b',params_names{ii}]);
  o=sym(['o',params_names{ii}]);
  p=sym(['p',params_names{ii}]);
  % b kombiniert p und o
  lhs = p+o;
  rhs = b;
  aes = subs(aes, lhs, rhs);
  % b enthält bereits o
  lhs = b+o;
  rhs = b;
  aes = subs(aes, lhs, rhs);
  % b enthält bereits p
  lhs = b+p;
  rhs = b;
  aes = subs(aes, lhs, rhs);
  % a kombiniert alle anderen
  lhs = p+a;
  rhs = a;
  aes = subs(aes, lhs, rhs);
  lhs = o+a;
  rhs = a;
  aes = subs(aes, lhs, rhs);
  lhs = b+a;
  rhs = a;
  aes = subs(aes, lhs, rhs);
end
%% Rücksubstitution
% Summanden extrahieren
aes = expand(aes); % Aufteilen des Ausdrucks in seine Summanden
aesc = children(aes+1);
% Entferne addierte +1 wieder
aesc = aesc(~(logical(aesc==1)));

% Erzeuge den Ausgabe-String
angles_valid_red = cell(1,length(aesc));
for i = 1:length(aesc)
  angles_valid_red{i} = '';
  for ii = 1:length(params_names)
    states = sym(zeros(4,2));
    states(1,:)=[sym(['a',params_names{ii}]),sym('a')];
    states(2,:)=[sym(['b',params_names{ii}]),sym('b')];
    states(3,:)=[sym(['o',params_names{ii}]),sym('o')];
    states(4,:)=[sym(['p',params_names{ii}]),sym('p')];
    for jj = 1:4
      if has(aesc(i), states(jj,1))
        angles_valid_red{i} = [angles_valid_red{i}, char(states(jj,2))];
      end
    end
  end
end
