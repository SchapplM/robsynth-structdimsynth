% Aktualisiere die Kollisionskörper des Roboters
% Diese können sich durch die inverse Kinematik ändern (für Schubgelenke)
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Q (optional)
%   Gelenkpositionen (für PKM auch passive Gelenke). Hiermit werden die
%   Kollisionskörper aktualisiert (wegen Länge der Schiene der Schubgelenke).
%   Optional. Falls nicht gesetzt: Nur Bestimmung der Ausgabe
% 
% Ausgabe:
% collbodies_robot
%   Struktur mit allen Kollisionskörpern des Roboters. Für PKM müssen die
%   Indizes umgerechnet werden. Daher hier ausgelagert.
% collbodies_instspc
%   Struktur mit Ersatz-Kollisionskörpern für die Prüfungs des Bauraums.
%   Enthält Punkte für jedes Gelenk und Anfangs- und Endpunkt von
%   Linearführungen von Schubachsen.
% init
%   Schalter, ob der Aufruf der Funktion für die Initialisierung oder vor
%   einer Kollisionsprüfung geschieht.
%   Vor der Initialisierung werden gestellfeste Kollisionskörper von PKM-
%   Beinketten noch nicht zusammengefasst (siehe cds_dimsynth_robot).
% 
% Siehe auch: cds_dimsynth_robot.m (Code teilweise identisch), 
% cds_constr_collisions_self

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [collbodies_robot, collbodies_instspc] = cds_update_collbodies(R, Set, Structure, Q, init)
if nargin < 5
  init = false;
end
if Set.general.matfile_verbosity > 2
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_update_collbodies_0.mat'));
  % load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_update_collbodies_0.mat'));
end
if R.Type == 0 % Seriell
  NLEG = 1;
elseif R.Type == 2 % Parallel
  NLEG = R.NLEG;
end
if nargin >= 4 % Eingabe Q ist gegeben
  update_collbodies = true;
else
  update_collbodies = false;
end
if isfield(Structure, 'installspace_collbodies')
  collbodies_instspc = Structure.installspace_collbodies;
  update_installspcbodies = true;
else
  collbodies_instspc = struct('link', [], 'type', [], 'params', []); % Platzhalter. Wird nicht nachher verarbeitet.
  update_installspcbodies = false;
end
%% Kollisionskörper aktualisieren
if update_collbodies
  % Grenzen der Schubgelenk-Koordinaten bestimmen die Position der
  % Führungsschiene bzw. des Führungszylinders
  q_minmax = NaN(R.NJ, 2);
  q_minmax(R.MDH.sigma==1,:) = minmax2(Q(:,R.MDH.sigma==1)');
  % Wähle die symmetrischen Gelenkgrenzen aus: Bei symmetrischem Roboter
  % sind die Führungsschienen von Schubgelenken der Beinketten gleich.
  % Wenn eine Schubache eine große Auslenkung hat, müssen bei allen Schub-
  % achsen die Schienen lang gewählt werden.
  if R.Type == 0  % Seriell 
    q_minmax_sym = q_minmax;
  elseif R.Type == 2  % Symmetrische PKM
    q_minmax_sym = q_minmax(R.I1J_LEG(1):R.I2J_LEG(1),:);
    % Grenzen durch Min-/Max-Werte aller Beinketten finden
    for i = 2:R.NLEG
      q_minmax_sym = minmax2([q_minmax_sym, q_minmax(R.I1J_LEG(i):R.I2J_LEG(i),:)]);
    end
  else % noch undefinierte Fälle (z.B Asymmetrisch, passive Zwangsführung)
    error('Fall %d für Robotertyp explizit noch nicht vorgesehen');
  end
  for k = 1:NLEG % Siehe cds_dimsynth_robot.m
    if R.Type == 0  % Seriell 
      R_cc = R;
    else % PKM-Beinkette
      R_cc = R.Leg(k);
    end
    q_minmax_k = q_minmax_sym;
    % Alle Schubgelenke der seriellen Kette durchgehen
    cbidx = 0; % Index für R_cc.collbodies
    for i = find(R_cc.MDH.sigma'==1)
      if i > 1, continue; end % Führungsschiene hier noch nicht modellierbar. Konsistent mit Initialisierung.
      cbidx = cbidx + 1;
      % Bestimme Anfangs- und Endposition der Führungsschiene (entsprechend
      % der MDH-Notation der Kinematik)
      T_mdh1 = trotz(R_cc.MDH.beta(i))*transl([0;0;R_cc.MDH.b(i)]) * ...
               trotx(R_cc.MDH.alpha(i))*transl([R_cc.MDH.a(i);0;0]);
      T_qmin = T_mdh1 * transl([0;0;-R_cc.DesPar.joint_offset(i)+q_minmax_k(i,1)]);
      T_qmax = T_qmin * transl([0;0;q_minmax_k(i,2)-q_minmax_k(i,1)]);
      % Prüfe Art des Schubgelenks
      if R_cc.DesPar.joint_type(i) == 4 % Führungsschiene
        % Füge die Führungsschiene der Linearachse als Körper hinzu.
        % Wird als Kapsel durch Anfang und Ende gekennzeichnet.
        % Bilde die MDH-Transformation nach. Das führt zu min-max für q
        cbi_par = [T_qmin(1:3,4)', T_qmax(1:3,4)', Set.optimization.collision_bodies_size/2]; % Radius
      elseif R_cc.DesPar.joint_type(i) == 5 % Hubzylinder
        % Der äußere Zylinder muss so lang sein wie der innere (bzw. der
        % innere Zylinder muss so lang sein wie der Hub).
        T_grozyl_start = T_qmin * transl([0;0;-(q_minmax_k(i,2)-q_minmax_k(i,1))]);
        T_grozyl_end = T_qmax;
        cbi_par = [T_grozyl_start(1:3,4)', T_grozyl_end(1:3,4)', Set.optimization.collision_bodies_size/2];
      else
        error('Fall %d für Schubgelenk nicht vorgesehen', R_cc.DesPar.joint_type(i));
      end
      % Ändere den Eintrag in der Liste der Kollisionsobjekte
      % (Stator-Teile der Schubgelenke sind immer am Anfang der Liste)
      R_cc.collbodies.params(cbidx,:) = [cbi_par, NaN(1,3)];
    end
    % Aktualisiere Kollisionskörper der Segmente (für alle). Betrifft nicht
    % die Plattform- und Gestell-Körper, da diese hiernach gesetzt werden.
    I_update_link = R_cc.collbodies.type == 6 & all(R_cc.collbodies.link ~= 0,2);
    R_cc.collbodies.params(I_update_link,1) = Set.optimization.collision_bodies_size/2; % Radius vs Durchmesser
  end
end
%% 
if nargout == 0
  if R.Type == 2
    R.update_collbodies([1 2], init); % sonst nicht wirksam für weitere Nutzung bei PKM
  end
  return
end
%% Debug: Kollisionskörper zeichnen
if false
  change_current_figure(2301); clf; hold all %#ok<UNRCH>
  view(3); axis auto; grid on;
  xlabel('x in m');ylabel('y in m');zlabel('z in m');
  q_plot = Q(1,:)';
  q_plot(isinf(q_plot)) = 0;
  if R.Type == 0 % Seriell
    s_plot = struct( 'ks', 1:R.NJ+2, 'straight', 1, 'mode', 5);
    R.plot( q_plot, s_plot);
  else % PKM
    s_plot = struct( 'ks_legs', [], 'ks_platform', [], 'straight', 1, 'mode', 5);
    R.plot( q_plot, NaN(6,1), s_plot); % TODO: EE-Trafo fehlt noch
  end
end

%% Bauraum-Geometrie als Kollisionsobjekte eintragen
n_cb_instspc = size(Set.task.installspace.type,1);
if update_installspcbodies && n_cb_instspc > 0
  T_0_W = invtr(R.T_W_0);
  % Füge Geometrien des Bauraums als virtuelle Kollisionsobjekte hinzu.
  % Transformiere ins Basis-KS des Roboters
  for i = 1:n_cb_instspc
    type_i = Set.task.installspace.type(i);
    % Umrechnung der Parameter ins Basis-KS
    params_W = Set.task.installspace.params(i,:);
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
    collbodies_instspc.params(i,:) = params_0;
    collbodies_instspc.type(i,1) = type_i;
    % Bauraum wird zur Basis (=0) gezählt (ortsfest)
    collbodies_instspc.link(i,:) = uint16([0,0]);
  end

  % Aktualisiere die Bauraum-Objekte in der Roboter-Klasse
  if Structure.Type ~= 0 % PKM
    % Aufteilung der Objekte in allgemein und Beinketten zugeordnet.
    % Hier nur Aktualisierung des allgemeinen Teils (zu PKM-Basis gehörig)
    collbodies_instspc_fix = struct( ...
      'link', collbodies_instspc.link(1:n_cb_instspc,:), ...
      'type', collbodies_instspc.type(1:n_cb_instspc,:), ...
      'params', collbodies_instspc.params(1:n_cb_instspc,:));
    R.collbodies_instspc_nonleg = collbodies_instspc_fix;
  else
    R.collbodies_instspc = collbodies_instspc;
  end
end

%% Gebe Kollisionskörper des Roboters heraus

% Liste für gesamte PKM. 0=PKM-Basis, 1=Beinkette1-Basis, 2=Beinkette1-
% erster Körper, usw. Wird für serielle Roboter auch benutzt. Dort aber
% nur eine Basis (=0).
collbodies_robot = struct('link', [], 'type', [], 'params', []);

% Erzeuge Kollisionsobjekte für Gestell und Plattform einer PKM. Muss vor
% der Definition der Kollisionsobjekte von Beinketten passieren, da die
% Variablen sonst nicht konsistent zur Roboter-Klasse sind. Reihenfolge
% dort ist festelegt durch Funktion ParRob/update_collbodies
if Structure.Type ~= 0 % PKM
  cbbpidx1 = size(collbodies_robot.link,1)+1;
  % Indizes der jeweiligen vorherigen benachbarten Beinkette
  I1 = (1:NLEG)'; I2 = [NLEG, 1:NLEG-1]';
  % Kollisionsobjekte für das Gestell
  if any(strcmp(Set.optimization.collshape_base, 'default'))
    if Set.task.DoF(3) ~= 0 % räumliche PKM
      collshape_base = {'star', 'ring'};
    else % planare PKM
      % Kein ausgedehntes Gestell, da 2T1R nur akademisches Beispiel ist. 
      % Konstruktiv sowieso lösbar durch Verschiebung des Gestells aus der 
      % Bewegungsebene heraus.
      collshape_base = {'joint'};
    end
  else
    collshape_base = Set.optimization.collshape_base;
  end
  if any(strcmp(collshape_base, 'star'))
    % Sternförmige Basis mit Kollisionskörpern: Nummer für PKM-Basis=0; Setze
    % den Vorgänger auf die jeweiligen Basiskörper der einzelnen Beinketten.
    % Kapseln verbinden die Koppelgelenke.
    collbodies_robot.link = [collbodies_robot.link; ...
      uint16([zeros(NLEG,1), R.I1L_LEG-(I1-1)])];
    collbodies_robot.type = [collbodies_robot.type; repmat(uint8(6),NLEG,1)];
    collbodies_robot.params = [collbodies_robot.params; ...
      [repmat(10e-3, NLEG, 1), NaN(NLEG, 9)]];
  end
  if any(strcmp(collshape_base, 'ring'))
    % Ringförmige Basis; verbindet die Basis der Beinketten mit der jeweils
    % vorherigen
    collbodies_robot.link = [collbodies_robot.link; ...
      uint16([R.I1L_LEG(I1)-(I1-1), R.I1L_LEG(I2)-(I2-1)])];
    collbodies_robot.type = [collbodies_robot.type; repmat(uint8(6),NLEG,1)];
    collbodies_robot.params = [collbodies_robot.params; ...
      [repmat(10e-3, NLEG, 1), NaN(NLEG, 9)]];
  end
  if any(strcmp(collshape_base, 'joint'))
    % Nur Gestellgelenke als Kugel, damit eine Kollision anderer Beinketten
    % damit verhindert wird.
    collbodies_robot.link = [collbodies_robot.link; ... % Beinketten-Basis
      uint16(repmat(R.I1L_LEG(I1)-(I1-1), 1, 2))];
%     T_base_stack = [R.Leg(:).T_W_0]; % Alle Beinketten-Basis-KS als Trafo spaltenweise
    collbodies_robot.type = [collbodies_robot.type; repmat(uint8(16),NLEG,1)]; % Kugel im Basis-KS der Beinketten
    collbodies_robot.params = [collbodies_robot.params; ... % Koordinaten der Gestell-Koppelgelenke eintragen
      [zeros(NLEG,3), repmat(0.05, NLEG, 1), NaN(NLEG, 6)]]; % Geringer Radius 50mm
  end
  
  % Kollisionsobjekte für die Plattform
  if any(strcmp(Set.optimization.collshape_platform, 'default'))
    if Set.task.DoF(3) ~= 0 %#ok<IFBDUP> % räumliche PKM
      % Kugel und Stern sind größtenteils redundant. Kugel verhindert
      % Durchdringen der Beinketten durch eine große Plattform
      collshape_platform = {'ring', 'sphere'};
    else % planare PKM
      % Der Ring verhindert ein Eindringen der Beinketten von außen in die
      % Plattform. Die Kugel verhindert, dass die Plattform außen um den
      % Roboter rum geht und die Beinketten komplett innen liegen.
      collshape_platform = {'ring', 'sphere'};
    end
  else
    collshape_platform = Set.optimization.collshape_platform;
  end
  if any(strcmp(collshape_platform, 'ring'))
    % Kapseln für jeden virtuellen Körper der Plattform-Koppelgelenke (auf
    % Plattform-Seite). Kapsel als Verbindung zum jeweils vorherigen
    % Koppelgelenk. Erzeugt Ring an der Plattform
    collbodies_robot.link = [collbodies_robot.link; ...
      uint16([R.I2L_LEG(I1)-(I1-1)-1, R.I2L_LEG(I2)-(I2-1)-1])];
    collbodies_robot.type = [collbodies_robot.type; repmat(uint8(6),NLEG,1)];
    collbodies_robot.params = [collbodies_robot.params; ...
      [repmat(10e-3, NLEG, 1), NaN(NLEG, 9)]];
  end
  if any(strcmp(collshape_platform, 'sphere'))
    % Kugel in der Mitte der Plattform platzieren. Damit kein Durchgriff
    % durch den Plattform-Ring möglich.
    % Bestimmung des Plattform-Radius vorerst bei Ignorieren der paarweisen
    % Anordnung. Dort sind die Gelenke noch weiter entfernt.
    % Bei sehr kleinen Plattform-Durchmessern wird nur ein Punkt als Kol-
    % lisionskörper benutzt (mit gleicher Implementierung)
    platform_radius = R.DesPar.platform_par(1);
    collbodies_robot.link = [collbodies_robot.link; ...
      repmat(uint16(R.I2L_LEG(end)-I1(end)+1),1,2)]; % zugeordnet (nur) zur Plattform
    collbodies_robot.type = [collbodies_robot.type; uint8(16)]; % Kugel im Körper-KS-Ursprung
    % Ziehe Durchmesser ab, damit die Enden der Beinketten nicht permanent
    % in Kollision stehen und mache die Kugel etwas kleiner. Nicht negativ.
    % Dadurch ist auch der maximale Winkel (großzügig) definiert, mit dem die
    % Beinkette über der Plattform geneigt zur Hochachse stehen kann.
    collbodies_robot.params = [collbodies_robot.params; ...
      [max(3/4*platform_radius-Set.optimization.collision_bodies_size,0), NaN(1,9)]]; % nur Radius als Parameter
  end
  if any(strcmp(collshape_platform, 'star'))
    % Sternförmige Kapseln für die Plattform. Verbindung zwischen
    % Endpunkten der Beinketten und Plattform-KS
    collbodies_robot.link = [collbodies_robot.link; ...
      uint16([R.I2L_LEG(I1)-(I1-1)-1, repmat(R.I2L_LEG(end)-I1(end)+1, NLEG, 1)])];
    collbodies_robot.type = [collbodies_robot.type; repmat(uint8(6),NLEG,1)];
    collbodies_robot.params = [collbodies_robot.params; ...
      [repmat(10e-3, NLEG, 1), NaN(NLEG, 9)]];
  end
  % Kollisionsobjekt für Verbindung von Plattform zu TCP (falls zutreffend)
  if Set.optimization.ee_translation_only_serial && ...
      ~Set.optimization.ee_translation_only_serial || ...
      all(~isnan(Set.optimization.ee_translation_fixed))
    % Direkte Verbindung als Kapsel
    collbodies_robot.link = [collbodies_robot.link; ...
      uint16(R.I2L_LEG(end)-I1(end)+[1 2])]; % Plattform+TCP
    collbodies_robot.type = [collbodies_robot.type; uint8(6)]; % Kapsel
    collbodies_robot.params = [collbodies_robot.params; ...
      [10e-3, NaN(1, 9)]]; % Radius 10mm
  end

  cbbpidx2 = size(collbodies_robot.link,1);
  % Eintragen in Klassen-Variable
  R.collbodies_nonleg =struct( ...
    'link',   collbodies_robot.link(cbbpidx1:cbbpidx2,:), ...
    'type',   collbodies_robot.type(cbbpidx1:cbbpidx2,:), ...
    'params', collbodies_robot.params(cbbpidx1:cbbpidx2,:));
  R.update_collbodies([1 2], init);
end
% Struktur der Kollisionskörper aus Roboterklasse nehmen
collbodies_robot = R.collbodies;

%% Bauraumprüfung für Führungsschienen der PKM-Beinketten oder serielle Roboter
if nargin < 4
  % Vereinfachter Aufruf um nur die Basis-Bauraumgeometrien zu initialisieren
  update_installspcbodies = false;
end
isidx = n_cb_instspc+1; % Nächster Index für collbodies_instspc

for k = 1:NLEG
  if R.Type == 0  % Seriell 
    R_cc = R;
  else % PKM-Beinkette
    R_cc = R.Leg(k);
  end
  if R_cc.MDH.sigma(1) == 1
    if any(R_cc.collbodies.type(1) == [3 13])
      % 3=Kapsel mitbewegt, 13=Kapsel basisfest. Muss noch vereinheitlicht werden in Initialisierung
      if R_cc.islegchain
        % Transformation der Parameter ins Basis-KS der PKM. Das ist für
        % die Kollisionsprüfung einfacher.
        T_0_0i = R_cc.T_W_0; % Trafo von PKM-Basis zu Beinketten-Basis
        pts_0i = R_cc.collbodies.params(1,1:6);
        pts_0 = [eye(3,4)*T_0_0i*[pts_0i(1:3)';1]; ...   % Punkt 1
                 eye(3,4)*T_0_0i*[pts_0i(4:6)';1]]'; ... % Punkt 2
      else
        % Keine Anpassung notwendig. Führungsschiene ist bereits bezogen auf
        % Roboter-Basis-KS
        pts_0 = R_cc.collbodies.params(1,1:6);
      end
    else
      error('Fall noch nicht vorhergesehen');
    end
    if update_installspcbodies
      % Ändere den Eintrag in der Liste der Bauraum-Objekte: Trage Anfangs-
      % und Endpunkt des Kapsel-Objekts als zwei Punkte ein.
      % Dadurch wird geprüft, ob Anfang und Ende der Schiene im Bauraum
      % liegen. Reicht als Prüfung (bei konvexem Bauraum).
      collbodies_instspc.params(isidx:isidx+1,1:3) = reshape(pts_0(1:6), 3, 2)';
      isidx = isidx + 2;
    end
  end
  % Überspringe die Indizes der Bauraum-Kollisionsobjekte für die Gelenke
  % Die übrigen Punktkoordinaten wurden schon vorher auf Null gesetzt.
  % (Entspricht Ursprung des jeweiligen Körper-KS)
  isidx = isidx + R_cc.NJ;
end

% Ausgabe der aktualisierten Liste der Bauraum-Kollisionsprüfungen
return

%% Debug: Vorbereitung
% Debug: Bei Fehler bezüglich falscher Indizierung
if isfield(Structure, 'collbodies_robot') %#ok<UNRCH> 
  n_cb_1 = size(Structure.collbodies_robot.type,1);
  n_cb_2 = size(collbodies_robot.type,1);
  if n_cb_1~=n_cb_2
    error('Anzahl der Kollisionskörper hat sich geändert');
  end
end

% Vorbereitung für Plot
Q(isinf(Q)) = 0;% Grenzen können inf sein bei Drehgelenken
%% Debug: Bauraum-Ersatzpunkte zeichnen
% Hiermit kann geprüft werden, ob die Punkt-Transformation korrekt ist.
% Die Führungsschienen sind im Plot länger, da sie aus qlim bestimmt werden
% und nicht aus minmax(q).
if false
  change_current_figure(2302); clf; hold all
  view(3); axis auto; grid on;
  xlabel('x in m');ylabel('y in m');zlabel('z in m');
  if R.Type == 0 % Seriell
    s_plot = struct( 'ks', 1:R.NJ+2, 'straight', 1, 'mode', 4);
    R.plot( Q(1,:)', s_plot);
  else % PKM
    s_plot = struct( 'ks_legs', [], 'ks_platform', [], 'straight', 1, 'mode', 4);
    R.plot( Q(1,:)', NaN(6,1), s_plot);
  end
  % Bauraum-Prüfpunkte einzeichnen (nur Führungsschienen)
  for i = 1:size(collbodies_instspc.type, 1)
    if collbodies_instspc.type(i) == 14
      p_0 = collbodies_instspc.params(i,1:3)';
      p_W = R.T_W_0 * [p_0;1];
      plot3(p_W(1), p_W(2), p_W(3), 'mx', 'MarkerSize', 20);
    end
  end
end

%% Debug: Roboter mit Kollisionskörpern zeichnen
% Benutze dafür die Klassenvariable. Die Variablen dort werden für die
% Kollisionsvermeidung mit der IK benutzt.
if false
  change_current_figure(2303); clf; hold all
  view(3); axis auto; grid on;
  xlabel('x in m');ylabel('y in m');zlabel('z in m');
  if R.Type == 0 % Seriell
    s_plot = struct( 'ks', 1:R.NJ+2, 'straight', 1, 'mode', 5);
    R.plot( Q(1,:)', s_plot);
  else % PKM
    s_plot = struct( 'ks_legs', [], 'ks_platform', [], 'straight', 1, 'mode', 5);
    R.plot( Q(1,:)', NaN(6,1), s_plot);
  end
end
end

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
