% Belegen der Kinematikparameter für die Optimierung
% Die Dynamikparameter werden in der Entwurfsoptimierung bestimmt.
% 
% Eingabe:
% R
%   Roboter-Klasse (SerRob oder ParRob) mit allen Eigenschaften des zu
%   optimierenden Roboters
% Set
%   Einstellungen des Optimierungsalgorithmus
% Structure
%   Eigenschaften der Roboterstruktur
% p
%   Vektor der Optimierungsvariablen für PSO. Werden in Roboter-Klasse ge-
%   schrieben
% 
% Ausgabe:
%   Roboter-Klasse mit aktualisierten Parametern

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function R_neu = cds_update_robot_parameters(R, Set, Structure, p)

R_neu = R; % ohne copy-Befehl: Parameter werden direkt in Eingang geschrieben
scale = p(1);
%% Parameter prüfen
if scale == 0
  error('Roboterskalierung kann nicht Null werden');
end
if length(Structure.vartypes) ~= length(p)
  error('Nicht für alle Optimierungsvariablen ist ein Typ definiert');
end

%% Gelenkgrenzen
% Müssen neu hineingeschrieben werden, da die Variable im späteren Verlauf
% der Optimierung überschrieben wird (z.B. zwecks Plotten)
if R_neu.Type == 0
  R_neu.qlim = Structure.qlim;
  % Schubgelenkgrenzen mit Skalierungsfaktor setzen. Ansonsten passt die
  % Proportion zwischen Segmentlängen und Schubgelenkslängen nicht
  R_neu.qlim(R_neu.MDH.sigma==1,:) = scale * Structure.qlim(R_neu.MDH.sigma==1,:);
else
  for i = 1:R.NLEG
    R_neu.Leg(i).qlim = Structure.qlim(R_neu.I1J_LEG(i):R_neu.I2J_LEG(i),:);
    % Skalierungsfaktor, damit Verhältnis zwischen Plattformgröße und Bein-
    % längen bei Schubgelenken noch stimmt
    R_neu.Leg(i).qlim(R_neu.Leg(i).MDH.sigma==1,:) = scale * R_neu.Leg(i).qlim(R_neu.Leg(i).MDH.sigma==1,:);
  end
end

%% Strukturparameter der Kinematik
if R_neu.Type == 0 || R_neu.Type == 2
  % Relevante Parameter sind die, die auch in Opt.Var. sind.
  if R_neu.Type == 0 % Seriell
    R_pkin = R_neu;
  else  % Parallel
    R_pkin = R_neu.Leg(1);
  end
  Ipkinrel = Structure.Ipkinrel; % Lade für Optimierung relevante Parameter aus Eingabe-Struktur
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
      pkin_voll(i) = pkin_optvar(j)*scale;
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
  % Die Parameter geben den skalierten Abstand des Roboters vom
  % Aufgabenmittelpunkt an
  p_basepos = p(Structure.vartypes == 2);
  r_W_0_neu = R.T_W_0(1:3,4);
  % xy-Punktkoordinaten der Basis skaliert mit Referenzlänge
  r_W_0_neu(Set.structures.DoF(1:2)) = Structure.xT_mean(Set.structures.DoF(1:2)) + ...
    p_basepos(Set.structures.DoF(1:2))*Structure.Lref;
  % z-Punktkoordinaten der Basis skaliert mit Roboter-Skalierungsfaktor
  if Set.structures.DoF(3)
    r_W_0_neu(3) = Structure.xT_mean(Set.structures.DoF(3)) + ...
      p_basepos(Set.structures.DoF(3))*scale;
  end
  R_neu.update_base(r_W_0_neu);
end

%% EE-Verschiebung
if any(Structure.vartypes == 3) % Set.optimization.ee_translation
  p_eepos = p(Structure.vartypes == 3);
  r_N_E_neu = zeros(3,1);
  % EE-Versatz skaliert mit Roboter-Skalierungsfaktor
  r_N_E_neu(Set.structures.DoF(1:3)) = p_eepos.*scale;
  R_neu.update_EE(r_N_E_neu);
end

%% EE-Rotation
if Set.optimization.ee_rotation && any(Structure.vartypes == 4)
  p_eerot = p(Structure.vartypes == 4);
  if sum(Set.structures.DoF(4:6)) == 1 % 2T1R -> Drehung um z-Achse
    % Die Drehung in Structure.R_N_E richtet die z-Achse nach oben
    if Structure.R_N_E_isset
      phi_N_E = r2eulxyz(Structure.R_N_E*rotz(p_eerot));
    else
      phi_N_E = [0;0;p_eerot];
    end
  elseif sum(Set.structures.DoF(4:6)) == 2
    % 3T2R. Nehme an, dass die EE-Transformation mit XYZ-Notation
    % durchgeführt wird. Daher keine Drehung um die letzte z-Achse
    phi_N_E = [p_eerot(1:2);0];
  else % muss 3T3R sein. Andere Fälle können hier nicht vorkommen
    % Annahme: R_N_E/R_P_E wird von diesem Typ nicht verwendet.
    phi_N_E = p_eerot(:);
  end
  R.update_EE([], phi_N_E);
end

%% Basis-Rotation
if Set.optimization.rotate_base
  error('Noch nicht implementiert');
end

%% Basis-Koppelpunkt Positionsparameter (z.B. Gestelldurchmesser)
if R_neu.Type == 2
  p_basepar = R.DesPar.base_par;
  changed_base = false;
end
if R_neu.Type == 2 && Set.optimization.base_size && any(Structure.vartypes == 6)
  p_baseradius = p(Structure.vartypes == 6);
  if length(p_baseradius) ~= 1
    error('Mehr als ein Fußpunkt-Positionsparameter nicht vorgesehen');
  end
  if p_baseradius == 0
    error('Basis-Radius darf nicht Null werden');
  end
  if all(~isnan(Set.optimization.base_size_limits))
    % Manuell gesetzte Grenzen: Absolute Größenangaben
    p_basepar(1) = p_baseradius;
  else
    % Setze den Fußpunkt-Radius skaliert mit Referenzlänge
    p_basepar(1) = p_baseradius*scale;
  end
  changed_base = true;
end

%% Gestell-Morphologie-Parameter (z.B. Gelenkpaarabstand)
if R_neu.Type == 2 && Set.optimization.base_morphology && any(Structure.vartypes == 8)
  % Methoden und Parameter, siehe align_base_coupling
  if R.DesPar.base_method == 4
    % Skalierung mit Basis-Radius und Winkel ohne Skalierung
    p_basepar(2:3) = p(Structure.vartypes == 8).*[p_basepar(1);1];
    changed_base = true;
  end
end

if R_neu.Type == 2 && changed_base
  R_neu.align_base_coupling(R.DesPar.base_method, p_basepar);
end
%% Plattform-Koppelpunkt Positionsparameter (z.B. Plattformdurchmesser)
if R_neu.Type == 2
  p_plfpar = R.DesPar.platform_par(1:end-1);
  changed_plf = false;
end
if R_neu.Type == 2 && Set.optimization.platform_size && any(Structure.vartypes == 7)
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
  if all(~isnan(Set.optimization.platform_size_limits))
    % Manuell gesetzte Grenzen: Absolute Größenangaben
    p_plfpar(1) = p_pfradius;
  else
    % Setze den Plattform-Radius skaliert mit Absolutwert des Gestell-Radius
    p_plfpar(1) = R_neu.DesPar.base_par(1)*p_pfradius;
  end
  changed_plf = true;
end

%% Plattform-Morphologie-Parameter (z.B. Gelenkpaarabstand)

if R_neu.Type == 2 && Set.optimization.platform_morphology && any(Structure.vartypes == 9)
  % Methoden und Parameter, siehe align_platform_coupling
  if R.DesPar.platform_method == 4
    p_plfpar(2) = p(Structure.vartypes == 9)*p_plfpar(1);
    changed_plf = true;
  end
end
if R_neu.Type == 2 && changed_plf
  R_neu.align_platform_coupling(R.DesPar.platform_method, p_plfpar);
end