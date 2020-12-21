% Entwurfsoptimierung für Offset der Schubgelenke (SerRob/DesPar.joint_offset)
% Notwendig, falls Bauraumbeschränkungen für die Führungsschienen der
% Schubgelenke vorliegen.
% 
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% X
%   Trajektorie im Arbeitsraum (Basis-KS)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Structure
%   Eigenschaften der Roboterstruktur (aus cds_gen_robot_list.m)
% JP
%   Gelenkpositionen aller Gelenke des Roboters
%   (Zeilen: Zeitschritte der Trajektorie)
% Q
%   Gelenkpositionen(für PKM auch passive Gelenke)
% 
% Ausgabe:
% fval_instspc
%   Strafterm für Bauraumverletzung. 0 falls keine Überschreitung. Sonst <1
% 
% Siehe auch: cds_constr_installspace.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function fval_instspc = cds_desopt_prismaticoffset(R, X, Set, Structure, JP, Q)
%% Initialisierung
% Prüfe, ob die Optimierung überhaupt notwendig ist.
if Structure.Type == 0 % Serieller Roboter
  nvars = sum(R.MDH.sigma==1);
else % symmetrische PKM
  nvars = sum(R.Leg(1).MDH.sigma==1);
end
if nvars == 0
  % Keine Schubgelenke, also keine Offsets zu optimieren
  return
end
% Deaktiviere die Debug-Ausgaben (Bilder) in den Funktionsaufrufen
Set_tmp = Set;
Set_tmp.general.plot_details_in_fitness = 0;
Set_tmp.general.matfile_verbosity = 0;
%% Optimierung durchführen
% Optimierungsfunktionen definieren
nonlcon2 = @(x)nonlcon(x, R, X, Set_tmp, Structure, JP, Q);
optfun_comb2 = @(x)optfun_comb(x, R, X, Set_tmp, Structure, JP, Q);
% Startwert für Optimierung: Offsets sind Null. Teste Gradienten am Start.
x1 = 1e-6*ones(nvars,1);
x0 = zeros(nvars,1); 
c1 = nonlcon2(x1);
c0 = nonlcon2(x0);
if c0 == c1
  % Die Bauraum-Nebenbedingung ist unabhängig von den Offsets.
  return
end
% Benutze fsolve um eine gültige Lösung zu finden (Beschränkung ist Null).
% Die Länge der Offsets ist damit erstmal egal und kann auch viel zu groß
% werden.
options_fsolve = optimoptions('fsolve', 'Display', 'none');
jo_opt_fsolve = fsolve(nonlcon2, x0, options_fsolve);

% Führe die Minimierung der Offsets mit hierarchischer Optimierung durch.
% Ein Verlassen der gültigen Lösung muss nicht gesondert betrachtet werden,
% da eine stetige Gütefunktion angenommen werden kann (je kleiner die
% Offsets, desto besser, bis sie irgendwann so klein sind, dass der Bauraum
% verletzt wird). Benutze nicht die (virtuell) unbeschränkte fminunc,
% sondern fminsearch (gradientenfrei), um weniger Funktionen auszuwerten
% options_fminunc = optimoptions('fminunc','Display', 'none');
% jo_opt_fminunc = fminunc(optfun_comb2, jo_opt_fsolve, options_fminunc);
[jo_opt_fminsearch, fval] = fminsearch(optfun_comb2, jo_opt_fsolve);
if fval > 1 % NB verletzt
  fval_instspc = fval;
else % alles i.O.
  fval_instspc = 0;
end
%% Eintragen in Roboter-Klasse
if Structure.Type == 0
  R.DesPar.joint_offset(R.MDH.sigma==1) = jo_opt_fminsearch;
else
  for kk = 1:R.NLEG
    R.Leg(kk).DesPar.joint_offset(R.Leg(kk).MDH.sigma==1) = jo_opt_fminsearch;
  end
end
end

function y = optfun(x) % Optimierungsfunktion
  y = 2/pi*atan(sum(abs(x))); % je betragskleiner, desto besser
  % fprintf('f([%s])=%1.3e\n', disp_array(x(:)', '%1.4f'), y);
end

function y = optfun_comb(x, R, X, Set, Structure, JP, Q) % Optimierungsfunktion
  % hierarchische Optimierung: Erst Nebenbedingung, dann Zielfunktion.
  y1 = nonlcon(x, R, X, Set, Structure, JP, Q);
  y2 = optfun(x);
  if y1 > 0 % NB wird verletzt.
    y = 1+y1; % Bereich 1 bis 2
  else % Zielfunktion
    y = y2; % Bereich 0 bis 1
  end
end

function [c, ceq] = nonlcon(x, R, X, Set, Structure, JP, Q)
  % Nebenbedingung in Optimierung: Einhaltung der Bauraumbeschränkung.
  % Parameter in Roboterklasse einstellen
  if Structure.Type == 0
    R.DesPar.joint_offset(R.MDH.sigma==1) = x;
  else
    for kk = 1:R.NLEG
      R.Leg(kk).DesPar.joint_offset(R.Leg(kk).MDH.sigma==1) = x;
    end
  end
  % Kollisionskörper damit aktualisieren
  [Structure.collbodies_robot, Structure.installspace_collbodies] = ...
    cds_update_collbodies(R, Set, Structure, Q);
  % Bauraumprüfung durchführen. Nebenbedingung ungleich Null, falls Bauraum
  % verletzt wird.
  c = cds_constr_installspace(R, X, Set, ...
    Structure, JP, Q, [0 1]);
  ceq = [];
  % fprintf('c([%s])=%1.3e\n', disp_array(x(:)', '%1.4f'), c);
end