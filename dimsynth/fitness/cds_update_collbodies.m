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
% 
% Siehe auch: cds_dimsynth_robot.m (Code teilweise identisch), 
% cds_constr_collisions_self

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [collbodies_robot, collbodies_instspc] = cds_update_collbodies(R, Set, Structure, Q)
if Set.general.matfile_verbosity > 2
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_update_collbodies_0.mat'));
  % load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_update_collbodies_0.mat'));
end
if R.Type == 0 % Seriell
  NLEG = 1;
elseif R.Type == 2 % Parallel
  NLEG = R.NLEG;
end
if nargin == 4
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
        cbi_par = [T_qmin(1:3,4)', T_qmax(1:3,4)', 20e-3]; % Radius 20mm
      elseif R_cc.DesPar.joint_type(i) == 5 % Hubzylinder
        % Der äußere Zylinder muss so lang sein wie der innere (bzw. der
        % innere Zylinder muss so lang sein wie der Hub).
        T_grozyl_start = T_qmin * transl([0;0;-(q_minmax_k(i,2)-q_minmax_k(i,1))]);
        T_grozyl_end = T_qmax;
        cbi_par = [T_grozyl_start(1:3,4)', T_grozyl_end(1:3,4)', 20e-3];
      else
        error('Fall %d für Schubgelenk nicht vorgesehen', R_cc.DesPar.joint_type(i));
      end
      % Ändere den Eintrag in der Liste der Kollisionsobjekte
      % (Stator-Teile der Schubgelenke sind immer am Anfang der Liste)
      R_cc.collbodies.params(cbidx,:) = [cbi_par, NaN(1,3)];
    end
  end
end
%% 
if nargout == 0
  return
end
%% Debug: Kollisionskörper zeichnen
if false
  change_current_figure(2301); clf; hold all %#ok<UNRCH>
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
%% Gebe Kollisionskörper des Roboters heraus

% Liste für gesamte PKM. 0=PKM-Basis, 1=Beinkette1-Basis, 2=Beinkette1-
% erster Körper, usw. Wird für serielle Roboter auch benutzt. Dort aber
% nur eine Basis (=0).
collbodies_robot = struct('link', [], 'type', [], 'params', []);
isidx = 1; % Index für collbodies_instspc
for k = 1:NLEG
  if R.Type == 0  % Seriell 
    NLoffset = 0;
    R_cc = R;
  else % PKM-Beinkette
    % Alle bewegten Körper der Beinketten werden als Kollisionskörper
    % gezählt, aber theoretisch auch (in der Zählung) alle Beinketten- 
    % Basis-KS. Damit wird die spätere Kollisionsprüfung vereinfacht.
    % Zusätzlich wird die PKM-Basis selbst gezählt (als erster Eintrag).
    NLoffset = 1; % Für Basis der Beinkette
    R_cc = R.Leg(k);
    if k > 1
      NLoffset = 1+R.I2L_LEG(k-1)-(k-1); % in I1L wird auch Basis und EE-Link noch mitgezählt. Hier nicht.
    end
  end
  % Passe den Typ der Parameter an. Kollisionsobjekte der Führungsschiene
  % an der PKM-Basis werden genauso zum Welt-KS gezählt.
  collbodies_type_mod = R_cc.collbodies.type;
  collbodies_params_mod = R_cc.collbodies.params;
  if R_cc.MDH.sigma(1) == 1
    if R_cc.collbodies.type(1) == 3
      collbodies_type_mod(1) = uint8(13);
      if R_cc.islegchain
        % Transformation der Parameter ins Basis-KS der PKM. Das ist für
        % die Kollisionsprüfung einfacher.
        T_0_0i = R_cc.T_W_0; % Trafo von PKM-Basis zu Beinketten-Basis
        pts_0i = R_cc.collbodies.params(1,1:6);
        pts_0 = [eye(3,4)*T_0_0i*[pts_0i(1:3)';1]; ...   % Punkt 1
                 eye(3,4)*T_0_0i*[pts_0i(4:6)';1]]'; ... % Punkt 2
        collbodies_params_mod(1,1:6) = pts_0;
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
  % Trage in PKM-weite Variable ein
  collbodies_robot.link = [collbodies_robot.link; [R_cc.collbodies.link+...
    NLoffset,Structure.MDH_ante_collcheck(1+R_cc.collbodies.link+NLoffset)]];
  collbodies_robot.type = [collbodies_robot.type; collbodies_type_mod];
  collbodies_robot.params = [collbodies_robot.params; collbodies_params_mod];
  % Überspringe die Indizes der Bauraum-Kollisionsobjekte für die Gelenke
  % Die übrigen Punktkoordinaten wurden schon vorher auf Null gesetzt.
  % (Entspricht Ursprung des jeweiligen Körper-KS)
  isidx = isidx + R_cc.NJ;
end
% Ausgabe der aktualisierten Liste der Bauraum-Kollisionsprüfungen
if update_installspcbodies
  Structure.installspace_collbodies = collbodies_instspc;
end

%% Debug: Bauraum-Ersatzpunkte zeichnen
% Hiermit kann geprüft werden, ob die Punkt-Transformation korrekt ist.
% Die Führungsschienen sind im Plot länger, da sie aus qlim bestimmt werden
% und nicht aus minmax(q).
if false
  change_current_figure(2302); clf; hold all %#ok<UNRCH>
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
