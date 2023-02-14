% Nebenbedingungen für Roboter-Maßsynthese berechnen (Teil 0: Parameter-
% ebene). Ermöglicht komplexere Parameter-Nebenbedingungen als
% Box-Constraints aus PSO-Algorithmus
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob).
%   Darf leer sein. Dann werden manche Prüfungen übersprungen.
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Structure
%   Eigenschaften der Roboterstruktur (aus cds_gen_robot_list.m)
% p, p_phys
%   Vektor der Optimierungsvariablen (siehe cds_update_robot_parameters.m).
%   (normierte Werte und physikalische Werte)
% 
% Ausgabe:
% fval
%   Zielfunktionswert, der im PSO-Algorithmus minimiert wird. Entspricht
%   Strafterm in der Fitnessfunktion bei Verletzung der Nebenbedingungen
%   Werte:
%   1e13...2e13: Winkel für konische Gestellgelenke zu nah an waagerecht/senkrecht
%   2e13...3e13: Das gleiche für konische Plattformgelenke
%   3e13...4e13: Effektiver Radius für paarweise Gestellgelenke zu groß
%   4e13...5e13: Das gleiche für effektiven Radius paarweiser Plattformgelenke
%   5e13...6e13: Die Gelenkabstände sind zu klein

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-02
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval, constrvioltext] = cds_constraints_parameters(R, Set, Structure, p, p_phys)
fval = 0;
constrvioltext = '';
if nargin < 5, p_phys = p; end
%% Prüfe Neigungswinkel konischer Gestellgelenke
if Structure.Type == 2 && any(Structure.Coupling(1) == [4 8]) && ...
    Set.optimization.min_inclination_conic_base_joint > 0
  p_basemorph = p_phys(Structure.vartypes == 8);
  gamma_b = p_basemorph(end);
  % Bestrafe Abweichungen von der senkrechten (0 bzw. 180°) oder
  % waagerechten (90° oder 270°)
  delta_gamma1 = angleDiff(gamma_b, 0);
  delta_gamma2 = angleDiff(gamma_b, pi/2);
  delta_gamma3 = angleDiff(gamma_b, pi);
  delta_gamma4 = angleDiff(gamma_b, 3*pi/2);
  if     abs(delta_gamma1) < Set.optimization.min_inclination_conic_base_joint
    delta_gamma = abs(delta_gamma1);
  elseif abs(delta_gamma2) < Set.optimization.min_inclination_conic_base_joint
    delta_gamma = abs(delta_gamma2);
  elseif abs(delta_gamma3) < Set.optimization.min_inclination_conic_base_joint
    delta_gamma = abs(delta_gamma3);
  elseif abs(delta_gamma4) < Set.optimization.min_inclination_conic_base_joint
    delta_gamma = abs(delta_gamma4);
  else
    delta_gamma = NaN;
  end
  % Normierten Strafterm bilden
  if ~isnan(delta_gamma)
    fval_rel = (Set.optimization.min_inclination_conic_base_joint-delta_gamma)/...
                Set.optimization.min_inclination_conic_base_joint;
    assert(fval_rel <=1 & fval_rel >=0, 'Relative Abweichung der Gelenkneigung muss <1 sein');
    fval = 1e13 * (1 + fval_rel); % normiere auf 1e13 bis 2e13
    constrvioltext = sprintf('Neigung des Gestellgelenks ist mit %1.1f° nicht groß genug', ...
      180/pi*gamma_b);
    return
  end
end

%% Prüfe Neigungswinkel konischer Plattformgelenke
if Structure.Type == 2 && any(Structure.Coupling(2) == 9) && ...
    Set.optimization.min_inclination_conic_platform_joint > 0
  gamma_p = p_phys(Structure.vartypes == 9);
  % Bestrafe Abweichungen von der senkrechten (0 bzw. 180°) oder
  % waagerechten (90° oder 270°)
  delta_gamma1 = angleDiff(gamma_p, 0);
  delta_gamma2 = angleDiff(gamma_p, pi/2);
  delta_gamma3 = angleDiff(gamma_p, pi);
  delta_gamma4 = angleDiff(gamma_p, 3*pi/2);
  if     abs(delta_gamma1) < Set.optimization.min_inclination_conic_platform_joint
    delta_gamma = abs(delta_gamma1);
  elseif abs(delta_gamma2) < Set.optimization.min_inclination_conic_platform_joint
    delta_gamma = abs(delta_gamma2);
  elseif abs(delta_gamma3) < Set.optimization.min_inclination_conic_platform_joint
    delta_gamma = abs(delta_gamma3);
  elseif abs(delta_gamma4) < Set.optimization.min_inclination_conic_platform_joint
    delta_gamma = abs(delta_gamma4);
  else
    delta_gamma = NaN;
  end
  % Normierten Strafterm bilden
  if ~isnan(delta_gamma)
    fval_rel = (Set.optimization.min_inclination_conic_platform_joint-delta_gamma)/...
                Set.optimization.min_inclination_conic_platform_joint;
    assert(fval_rel <=1 & fval_rel >=0, 'Relative Abweichung der Gelenkneigung muss <1 sein');
    fval = 1e13 * (2 + fval_rel); % normiere auf 2e13 bis 3e13
    constrvioltext = sprintf('Neigung des Plattformgelenks ist mit %1.1f° nicht groß genug', ...
      180/pi*gamma_p);
    return
  end
end
%% Prüfe effektiven Radius für paarweise Gestell-Gelenke
if ~isempty(R) && Structure.Type == 2 && any(Structure.Coupling(1) == [5 6 7 8]) && ... % PKM, Gestell paarweise
    all(~isnan(Set.optimization.base_size_limits)) && ... % Gestell-Grenzen gegeben
    any(Structure.vartypes == 8) && ... % Morphologie wird optimiert
    Set.optimization.base_size_limits(1)~=Set.optimization.base_size_limits(2) % Grenzen nicht gleich
  r_eff = norm(R.r_0_A_all(:,1)); % Effektiver Radius (Abstand des Gelenks zur Mitte)
  r_eff_exc_rel = (r_eff - Set.optimization.base_size_limits(1)) / ...
    (Set.optimization.base_size_limits(2)-Set.optimization.base_size_limits(1));
  if r_eff_exc_rel > 1 % obere Grenze überschritten durch zu großen Paarabstand
    fval = 1e13 * (3 + 1 * 2/pi*atan(r_eff_exc_rel-1)); % normiere auf 3e13 bis 4e13
    constrvioltext = sprintf(['G=%d;I=%d. Fitness-Evaluation in %1.2fs. ', ...
      'fval=%1.3e. Gestell-Paarabstand ist um %1.1f%% zu groß. Effektiver Basis-', ...
      'Radius damit %1.1fmm (erlaubt: %1.1fmm ... %1.1fmm)'], i_gen, i_ind, ...
      toc(t1), fval(1), 100*(r_eff_exc_rel-1), 1e3*r_eff, 1e3*Set.optimization.base_size_limits(1), ...
      1e3*Set.optimization.base_size_limits(2));
    return
  elseif r_eff_exc_rel < 0
    error('Logik-Fehler. Untere Grenze für Gestell-Radius wurde unterschritten');
  end
end
%% Prüfe effektiven Radius für paarweise Plattform-Gelenke
if ~isempty(R) && Structure.Type == 2 && any(Structure.Coupling(2) == [4 5 6]) && ... % PKM, Plattform paarweise
    all(~isnan(Set.optimization.platform_size_limits)) && ... % Plattform-Grenzen gegeben
    any(Structure.vartypes == 9) && ... % Morphologie wird optimiert
    Set.optimization.platform_size_limits(1)~=Set.optimization.platform_size_limits(2) % Grenzen nicht gleich
  r_eff = norm(R.r_P_B_all(:,1)); % Effektiver Radius (Abstand des Gelenks zur Mitte)
  r_eff_exc_rel = (r_eff - Set.optimization.platform_size_limits(1)) / ...
    (Set.optimization.platform_size_limits(2)-Set.optimization.platform_size_limits(1));
  if r_eff_exc_rel > 1 % obere Grenze überschritten durch zu großen Paarabstand
    fval = 1e13 * (4 + 1 * 2/pi*atan(r_eff_exc_rel-1)); % normiere auf 4e13 bis 5e13
    constrvioltext = sprintf(['G=%d;I=%d. Fitness-Evaluation in %1.2fs. ', ...
      'fval=%1.3e. Plattform-Paarabstand ist um %1.1f%% zu groß. Effektiver Plattform-', ...
      'Radius damit %1.1fmm (erlaubt: %1.1fmm ... %1.1fmm)'], i_gen, i_ind, ...
      toc(t1), fval(1), 100*(r_eff_exc_rel-1), 1e3*r_eff, 1e3*Set.optimization.platform_size_limits(1), ...
      1e3*Set.optimization.platform_size_limits(2));
    return
  elseif r_eff_exc_rel < 0
    error('Logik-Fehler. Untere Grenze für Plattform-Radius wurde unterschritten');
  end
end
%% Prüfe minimalen Gelenkabstand
if Set.optimization.min_joint_distance > 0
  % Gehe alle a-/d-Variablen durch und bestimme daraus den Abstand
  [tokens, ~] = regexp(Structure.varnames, 'pkin \d+: ([ad])(\d)', 'tokens', 'match');
  DHtable = NaN(7,3); % Spalten: a-Param., d-Param., Norm. Zeile: Gelenke
  for i = 1:length(tokens)
    if isempty(tokens{i}), continue; end
      if tokens{i}{1}{1} == 'a', j = 1;
      elseif tokens{i}{1}{1} == 'd', j = 2; % d-Param.
      else, error('Unerwartetes Muster in Structure.varnames');
      end
      k = str2double(tokens{i}{1}{2}); % Gelenknummer
      DHtable(k,j) = p(1) * p(i); % Skaliere auf physikalischen Wert
  end
  % Berechne den Abstand zum vorherigen Gelenk. Keine Beachtung von U/S
  % Gelenken notwendig, da diese sowieso keine Optimierungsparameter haben
  for i = 1:size(DHtable,1)
    if all(isnan(DHtable(i,:))), continue; end
    % Setze NaN-Felder auf Null. Der DH-Parameter ist nicht gesetzt
    DHtable(i,isnan(DHtable(i,:))) = 0;
    % Gelenkabstand aus a- und d-Parameter berechnen
    DHtable(i,3) = sqrt(DHtable(i,1)^2 + DHtable(i,2)^2);
  end
  RelNormViol = DHtable(:,3) / Set.optimization.min_joint_distance;
  if any(RelNormViol < 1)
    [minRelNormViol, IminRelNormViol] = min(RelNormViol);
    fval_rel = 1-min(minRelNormViol);
    assert(fval_rel<=1, 'Relative Gelenkabstandsunterschreitung muss <1 sein');
    fval = 1e13 * (5 + fval_rel); % normiere auf 5e13 bis 6e13
    constrvioltext = sprintf('Gelenkabstand ist zu klein (Gelenk %d Abstand %1.1fmm, also %1.0f%% von %1.1fmm)', ...
      IminRelNormViol, 1e3*DHtable(IminRelNormViol,3), minRelNormViol*100, 1e3*Set.optimization.min_joint_distance);
    return
  end
end