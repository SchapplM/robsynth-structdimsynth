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
% p_phys
%   Parameter ohne Skalierung, so wie sie physikalisch eingetragen werden

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function p_phys = cds_update_robot_parameters(R, Set, Structure, p)

R_neu = R; % ohne copy-Befehl: Parameter werden direkt in Eingang geschrieben
scale = p(1);
p_phys = NaN(length(p),1);
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
  Ipkin = find(Structure.vartypes==1);
  for i = 1:length(pkin_voll)
    if ~Ipkinrel(i)
      continue
    end
    j = j + 1; % Index für Kinematikparameter in den Optimierungsvariablen
    if R_pkin.pkin_types(i) == 1 || R_pkin.pkin_types(i) == 3 || R_pkin.pkin_types(i) == 5
      pkin_voll(i) = pkin_optvar(j);
      p_phys(Ipkin(j)) = pkin_optvar(j);
    else
      % Längenparameter skaliert mit Roboter-Skalierungsfaktor
      pkin_voll(i) = pkin_optvar(j)*scale;
      p_phys(Ipkin(j)) = pkin_optvar(j)*scale;
    end
  end
  if R_neu.Type == 0, R_neu.update_mdh(pkin_voll);  % Seriell
  else,               R_neu.update_mdh_legs(pkin_voll); end % Parallel
else
  error('Noch nicht implementiert');
end

%% Basis-Position
changed_basepose = false;
if Set.optimization.movebase
  % Die Parameter sind entweder relativ zur Aufgabe oder absolut definiert.
  p_basepos = p(Structure.vartypes == 2);
  I_bp = find(Structure.vartypes == 2);
  r_W_0_neu = R.T_W_0(1:3,4);
  if Structure.Type == 0, mounting = Set.structures.mounting_serial;
  else, mounting = Set.structures.mounting_parallel;
  end
  if strcmp(mounting, 'wall') && all(Set.task.DoF(1:5) == [1 1 0 0 0])
    task_transl_DoF_rot0 = [1 0 1]; % Sonderfall: Basis-Position in z0-Richtung soll nicht optimiert werden
  else
    task_transl_DoF_rot0 = Set.task.DoF(1:3);
  end
  % xyz-Punktkoordinaten der Basis skaliert mit Referenzlänge
  % TODO: Klären, ob Roboter-Skalierungsfaktor für z-Koordinate doch besser wäre
  p_idx = 0;
  for i_xyz = 1:3 % bezogen auf Basis-KS des Roboters
    if task_transl_DoF_rot0(i_xyz) == 0 || ...  % FG ist nicht Teil der Aufgabe (z.B. bei 2T1R). Ignorieren.
        Set.optimization.basepos_limits(i_xyz,1)==Set.optimization.basepos_limits(i_xyz,2) % nur ein Wert vorgegeben
      continue
    end
    p_idx = p_idx + 1;
    if all(~isnan(Set.optimization.basepos_limits(1,:)))
      % Grenzen explizit vorgegeben. Nehme den Wert direkt (ohne Skalierung)
      r_W_0_neu(i_xyz) = p_basepos(p_idx);
    else
      % Skaliere den Wert und rechne eine Position relativ zum Mittelpunkt
      % der Aufgabe aus.
      r_W_0_neu(i_xyz) = Structure.xT_mean(i_xyz) + ...
        p_basepos(p_idx)*Structure.Lref;
    end
    p_phys(I_bp(p_idx)) = r_W_0_neu(i_xyz);
  end
  R_neu.update_base(r_W_0_neu);
  changed_basepose = true;
end

%% EE-Verschiebung
if any(Structure.vartypes == 3) % Set.optimization.ee_translation
  p_eepos = p(Structure.vartypes == 3);
  % Koordinaten auswählen (bezogen auf KS N, nicht KS E).
  % Stellt für planare serielle Roboter einen Unterschied dar
  if Structure.R_N_E_isset && R.Type == 0
    % hier nicht R.T_N_E(1:3,1:3)' benutzen, da die Rotation dann nicht passt
    task_transl_DoF_rotE = Structure.R_N_E' * double(Set.task.DoF(1:3)');
  else
    task_transl_DoF_rotE = double(Set.task.DoF(1:3)');
  end
  r_N_E_neu = zeros(3,1);
  % EE-Versatz skaliert mit Roboter-Skalierungsfaktor
  r_N_E_neu(abs(task_transl_DoF_rotE(:))>1e-10) = p_eepos .* scale;
  p_phys(Structure.vartypes == 3) = r_N_E_neu(abs(task_transl_DoF_rotE(:))>1e-10);
  R_neu.update_EE(r_N_E_neu);
end

%% EE-Rotation
phi_N_E = NaN(3,1);
if Set.optimization.ee_rotation && any(Structure.vartypes == 4)
  p_eerot = p(Structure.vartypes == 4);
  p_phys(Structure.vartypes == 4) = p_eerot;
  if sum(Set.task.DoF(4:6)) == 1 % 2T1R oder 3T1R -> Drehung um z-Achse
    % Die Drehung in Structure.R_N_E richtet die z-Achse nach oben
    if Structure.R_N_E_isset
      phi_N_E = r2eulxyz(Structure.R_N_E*rotz(p_eerot));
    else
      phi_N_E = [0;0;p_eerot];
    end
  elseif sum(Set.task.DoF(4:6)) == 2
    % 3T2R. Nehme an, dass die EE-Transformation mit XYZ-Notation
    % durchgeführt wird. Daher keine Drehung um die letzte z-Achse
    phi_N_E = Set.optimization.ee_rotation_fixed(:); % NaN für optimierte Werte
    phi_N_E(isnan(phi_N_E)) = p_eerot;
    phi_N_E(3) = 0;
    if Structure.R_N_E_isset % Berücksichtige Drehung z.B. für Deckenmontage
      phi_N_E = r2eulxyz(Structure.R_N_E*eulxyz2r(phi_N_E));
    end
  else % muss 3T3R sein. Andere Fälle können hier nicht vorkommen
    % Annahme: R_N_E/R_P_E wird von diesem Typ nicht verwendet.
    phi_N_E = Set.optimization.ee_rotation_fixed(:); % NaN für optimierte Werte
    phi_N_E(isnan(phi_N_E)) = p_eerot;
    if Structure.R_N_E_isset % Berücksichtige Drehung z.B. für Deckenmontage
      phi_N_E = r2eulxyz(Structure.R_N_E*eulxyz2r(phi_N_E));
    end
  end
  R.update_EE([], phi_N_E);
end

%% Basis-Rotation (um die z-Achse nach der Drehung in Boden-/Deckenmontage)
if Set.optimization.rotate_base && any(Structure.vartypes == 5)
  p_baserot = p(Structure.vartypes == 5);
  p_phys(Structure.vartypes == 5) = p_baserot;
  if Structure.R_W_0_isset
    phi_W_0 = r2eulxyz(Structure.R_W_0*rotz(p_baserot));
  else
    phi_W_0 = [0;0;p_baserot];
  end
  R.update_base([], phi_W_0);
  if all(R.Type == 2) && (all(R.I_EE == [1 1 1 0 0 0]) || all(R.I_EE == [1 1 0 0 0 0]))
    % Bei 3T0R-PKM kann die Basis nicht einfach gedreht werden. Dann ist
    % das EE-KS verdreht gegenüber der Aufgabe. Daher muss das EE-KS um den 
    % Winkel der Basis-Drehung zurückgedreht werden. Annahme: z-Achsen von
    % Basis und Plattform (nicht: EE) zeigen in die gleiche Richtung.
    if any(isnan(phi_N_E)) % Wurde noch nicht oben belegt
      if Structure.R_N_E_isset
        phi_N_E = r2eulxyz(Structure.R_N_E); % Kann durch Deckenmontage bereits gesetzt sein
      else
        phi_N_E = zeros(3,1);
      end
    end
    R.update_EE([], phi_N_E+[0;0;p_baserot]);
  end
  changed_basepose = true;
end
if changed_basepose && R.Type == 2
  % Aktualisiere die Referenzpose der Plattform (notwendig für
  % Aktualisierung der Plattform-Gestellgelenke Nr. 7)
  R.xref = R.t2x(R.T_0_W * R.x2t(Structure.xref_W));
end
%% Basis-Koppelpunkt Positionsparameter (z.B. Gestell-Radius)
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
  p_phys(Structure.vartypes == 6) = p_basepar(1);
  changed_base = true;
end

%% Gestell-Morphologie-Parameter (z.B. Gelenkpaarabstand)
if R_neu.Type == 2 && Set.optimization.base_morphology && any(Structure.vartypes == 8)
  % Methoden und Parameter, siehe align_base_coupling
  if any(R.DesPar.base_method == 1:3)
    % Modi haben keine Morphologieparameter
  elseif R.DesPar.base_method == 4
    % Nur der Winkel der Kegel-Steigung ist der Morphologieparameter.
    p_basepar(2) = p(Structure.vartypes == 8);
    p_phys(Structure.vartypes == 8) = p_basepar(2);
  elseif any(R.DesPar.base_method == 5:7)
    % Parameter ist der Paarabstand. Skaliert mit Robotergröße.
    p_basepar(2) = p(Structure.vartypes == 8).*p_basepar(1);
    p_phys(Structure.vartypes == 8) = p_basepar(2);
  elseif R.DesPar.base_method == 8
    % Skalierung mit Basis-Radius und Winkel ohne Skalierung
    p_basepar(2:3) = p(Structure.vartypes == 8).*[p_basepar(1);1];
    p_phys(Structure.vartypes == 8) = p_basepar(2:3);
  end
  changed_base = true;
end

if R_neu.Type == 2 && changed_base
  R_neu.align_base_coupling(R.DesPar.base_method, p_basepar);
  if R.DesPar.base_method == 4 || R.DesPar.base_method == 8 % Kegel, Pyramide
    % Falls schräge Anordnung kann sich der Winkel ändern. Sonst bleibt er
    % konstant (wie in Initialisierung bereits gesetzt).
    R.update_gravity(); % Die Gravitations-Vektoren im Beinketten-Basis-KS aktualisieren
  end
end
%% Plattform-Koppelpunkt Positionsparameter (z.B. Plattform-Radius)
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
  p_phys(Structure.vartypes == 7) = p_plfpar(1);
  changed_plf = true;
end

%% Plattform-Morphologie-Parameter (z.B. Gelenkpaarabstand)
% Methoden und Parameter, siehe align_platform_coupling
if R_neu.Type == 2 && any(R.DesPar.platform_method == [4 5 6 8])
  if Set.optimization.platform_morphology && any(Structure.vartypes == 9) 
    if R.DesPar.platform_method == 8
      % Offset-Parameter (Winkel) platform_morph_axoffset. Direkte Übernahme.
      p_plfpar(2) = p(Structure.vartypes == 9);
    else
      % Paar-Abstand-Parameter platform_morph_pairdist. Skalierung mit Plattform-Größe
      p_plfpar(2) = p(Structure.vartypes == 9)*p_plfpar(1);
    end
    p_phys(Structure.vartypes == 9) = p_plfpar(2);
    changed_plf = true;
  elseif any(R.DesPar.platform_method == [4 5 6])
    % Aktualisiere die Proportionen (gleicher Wert wie in cds_dimsynth_robot). 
    % Sonst würde sich mit der Größe die Proportion ändern).
    p_plfpar(2) = 0.5*p_plfpar(1);
  end
elseif R_neu.Type == 2 && R.DesPar.platform_method == 7 && changed_basepose
  % Die veränderte Basis-Position des Roboters erfordert eine Aktuali-
  % sierung der Koppelgelenk-Ausrichtung (da diese per IK bestimmt wird)
  changed_plf = true;
end
if R_neu.Type == 2 && changed_plf
  R_neu.align_platform_coupling(R.DesPar.platform_method, p_plfpar);
end

%% Vorab festgelegte Gelenkwinkel für bestimmte Parameter
% Hiermit werden Gelenkwinkel aus der Initialpopulation vorgegeben. Hilf-
% reich, wenn IK-Ergebnisse nicht exakt reproduziert werden können.
if isfield(Structure, 'dict_param_q')
  % Prüfe, ob der Parametersatz in der Tabelle gelistet ist
  I = all(Structure.dict_param_q.p - repmat(p(:)',size(Structure.dict_param_q.p,1),1)==0,2);
  % Belege die Referenzpose des Roboters mit diesen Werten. Dadurch IK-Startwert
  if any(I) && sum(I) == 1
    qref = Structure.dict_param_q.q(I,:)';
    if R.Type == 0 % Seriell
      R.qref = qref;
    else % Parallel
      for i = 1:R.NLEG, R.Leg(i).qref = qref(R.I1J_LEG(i):R.I2J_LEG(i)); end
    end
  end
end
