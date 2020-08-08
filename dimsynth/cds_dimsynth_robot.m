% Maßsynthese für eine Roboterstruktur
% 
% Eingabe:
% Set
%   Einstellungen des Optimierungsalgorithmus
% Traj
%   Trajektorie (bezogen auf Welt-KS)
% Structure
%   Eigenschaften der Roboterstruktur
% 
% Ausgabe:
% RobotOptRes
%   Ergebnis-Struktur

% Quellen:
% [SchapplerTapOrt2019] Schappler, M. and Tappe, S., Ortmaier, T.:
% Exploiting Dynamics Parameter Linearity for Design Optimization in
% Combined Structural and Dimensional Robot Synthesis (2019)

function RobotOptRes = cds_dimsynth_robot(Set, Traj, Structure)
t1 = tic();
t_start = now(); % Anfangs-Zeitstempel der Optimierung dieses Roboters
%% Debug: 
if Set.general.matfile_verbosity > 0
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_robot1.mat'));
end
% Zum Debuggen:
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_robot1.mat'));

%% Initialisierung
% Log-Datei initialisieren
resdir_rob = fullfile(Set.optimization.resdir, Set.optimization.optname, ...
  sprintf('Rob%d_%s', Structure.Number, Structure.Name));
mkdirs(resdir_rob); % Ergebnis-Ordner für diesen Roboter erstellen
cds_log(1, sprintf('[dimsynth] Start der Maßsynthese für %s',  Structure.Name), 'init', Set, Structure)
% Zurücksetzen der Detail-Speicherfunktion
clear cds_save_particle_details;
% Mittelpunkt der Aufgabe
Structure.xT_mean = mean(minmax2(Traj.X(:,1:3)'), 2);
% Charakteristische Länge der Aufgabe (empirisch ermittelt aus der Größe
% des notwendigen Arbeitsraums)
Lref = norm(diff(minmax2(Traj.X(:,1:3)')'));
% Experimentell: Abstand der Aufgabe vom Roboter-Basis-KS
% Lref = Lref + mean(Structure.xT_mean) / 2;
Structure.Lref = Lref;
%% Roboter-Klasse initialisieren
if Structure.Type == 0 % Seriell
  R = serroblib_create_robot_class(Structure.Name);
  NLEG = 1;
elseif Structure.Type == 2 % Parallel
  % Parameter für Basis-Kopplung einstellen
  p_base = 1.5*Lref;
  if Structure.Coupling(1) == 4
    p_base(2) = pi/3;
  elseif any(Structure.Coupling(1) == [5,6,7])
    p_base(2) = 0.4*p_base(1);
  elseif Structure.Coupling(1) == 8
    p_base(2) = 0.4*p_base(1);
    p_base(3) = pi/3;
  end
  % Parameter für Plattform-Kopplung einstellen
  p_platform = 0.75*Lref;
  if any(Structure.Coupling(2) == [4,5,6])
    p_platform(2) = 0.5*p_platform(1);
  end
  R = parroblib_create_robot_class(Structure.Name, p_base(:), p_platform(:));
  NLEG = R.NLEG;
  if Set.optimization.use_desopt && Set.optimization.constraint_link_yieldstrength > 0
    R.DynPar.mode = 3; % Benutze Inertialparameter-Dynamik, weil auch Schnittkräfte in Regressorform berechnet werden
  else
    R.DynPar.mode = 4; % Benutze Minimalparameter-Dynamikfunktionen für die PKM
  end
  R.update_dynpar1(R.DynPar.mges, R.DynPar.rSges, R.DynPar.Icges); % Nochmal initialisieren, damit MPV definiert ist
else
  error('Typ-Nummer nicht definiert');
end
R.fill_fcn_handles(Set.general.use_mex, true);

% Aufgaben-FG des Roboters setzen
if Structure.Type == 0 % Seriell (nur hier notwendig. TODO: Prüfen ob obsolet)
  R.I_EE_Task = Set.structures.DoF;
end

for i = 1:NLEG
  if Structure.Type == 0
    R_init = R;
  else
    R_init = R.Leg(i);
  end
  R_init.gen_testsettings(false, true); % Setze Kinematik-Parameter auf Zufallswerte
  % Gelenkgrenzen setzen: Schubgelenke (Verfahrlänge nicht mehr als "vier
  % mal schräg durch Arbeitsraum" (char. Länge))
  % Muss so hoch gesetzt sein, damit UPS-Kette (ohne sonstige
  % Kinematikparameter auch funktioniert)
  R_init.qlim(R_init.MDH.sigma==1,:) = repmat([-0.5*Lref, 5*Lref],sum(R_init.MDH.sigma==1),1);
  % Gelenkgrenzen setzen: Drehgelenke
  if Structure.Type == 0 % Serieller Roboter
    % Grenzen für Drehgelenke: Alle sind aktiv
    R_init.qlim(R.MDH.sigma==0,:) = repmat([-0.5, 0.5]*Set.optimization.max_range_active_revolute, sum(R.MDH.sigma==0),1); % Drehgelenk
    Structure.qlim = R_init.qlim;
  else % Paralleler Roboter
    % Grenzen für passive Drehgelenke (aktive erstmal mit setzen)
    R_init.qlim(R_init.MDH.sigma==0,:) = repmat([-0.5, 0.5]*Set.optimization.max_range_passive_revolute, sum(R_init.MDH.sigma==0),1); % Drehgelenk
    % Grenzen für aktives Drehgelenk setzen
    I_actrevol = R_init.MDH.mu == 2 & R_init.MDH.sigma==0;
    R_init.qlim(I_actrevol,:) = repmat([-0.5, 0.5]*Set.optimization.max_range_active_revolute, sum(I_actrevol),1);
    if i == NLEG % Grenzen aller Gelenke aller Beinketten eintragen
      Structure.qlim = cat(1, R.Leg.qlim);
    end
  end
  % Gelenkgeschwindigkeiten setzen
  R_init.qDlim = repmat([-1,1]*Set.optimization.max_velocity_active_revolute, R_init.NJ, 1);
  R_init.qDlim(R_init.MDH.sigma==1,:) = repmat([-1,1]*Set.optimization.max_velocity_active_prismatic, sum(R_init.MDH.sigma==1), 1);
  if Structure.Type == 2 % Paralleler Roboter
    I_passrevol = R_init.MDH.mu == 1 & R_init.MDH.sigma==0;
    R_init.qDlim(I_passrevol,:) = repmat([-1,1]*Set.optimization.max_velocity_passive_revolute,sum(I_passrevol),1);
  end
  if Structure.Type == 0 % Serieller Roboter
    Structure.qDlim = R_init.qDlim;
  else % Paralleler Roboter
    if i == NLEG % Grenzen aller Gelenke aller Beinketten eintragen
      Structure.qDlim = cat(1, R.Leg.qDlim);
    end
  end
  
  % Dynamikparameter setzen
  if Set.optimization.use_desopt && Set.optimization.constraint_link_yieldstrength > 0
    R_init.DynPar.mode = 3; % Benutze Inertialparameter-Dynamik, weil auch Schnittkräfte in Regressorform berechnet werden
  else
    R_init.DynPar.mode = 4; % Benutze Minimalparameter-Dynamikfunktionen
  end
  R_init.DesPar.joint_type((1:R_init.NJ)'==1&R_init.MDH.sigma==1) = 4; % Linearführung erste Achse
  R_init.DesPar.joint_type((1:R_init.NJ)'~=1&R_init.MDH.sigma==1) = 5; % Schubzylinder weitere Achse
  R_init.update_dynpar1(); % Nochmal initialisieren, damit MPV definiert ist
  
  % Platzhalter-Werte für Segment-Parameter setzen (nur für Plotten)
  R_init.DesPar.seg_par(:,1) = 50e-3;
  R_init.DesPar.seg_par(:,2) = 5e-3;
end
% Merke die ursprünglich aus der Datenbank geladene EE-Rotation. Die in der
% Optimierung ergänzte Rotation ist zusätzlich dazu. (Bei 2T1R-Robotern
% wird teilweise die EE-Rotation notwendig, damit das letzte KS planar ist)
if Structure.Type == 0
  Structure.R_N_E = R.T_N_E(1:3,1:3);
else
  Structure.R_N_E = R.T_P_E(1:3,1:3);
end
if any(abs(r2eulxyz(Structure.R_N_E(1:3,1:3))) > 1e-10)
  Structure.R_N_E_isset = true;
else
  Structure.R_N_E_isset = false;
end
% Falls planerer Roboter: Definiere Verschiebung, damit der Roboter von
% oben angreift. Sieht besser aus, macht die Optimierung aber schwieriger.
% if all(Set.structures.DoF(1:3) == [1 1 0])
%   R.update_base([0;0;0.5*Lref]);
%   R.update_EE([0;0;-0.5*Lref]);
% end
%% Umfang der Berechnungen prüfen: Schnittkraft / Regressorform / Dynamik
% Schalter zum Berechnen der Dynamik bezogen auf Antriebe
calc_dyn_act = false;
% Schalter zum Berechnen der vollständigen Schnittkräfte
calc_dyn_cut = false;
% Schalter zur Berechnung der Regressorform der Dynamik; [SchapplerTapOrt2019]
calc_reg = false;

if any(strcmp(Set.optimization.objective, {'energy', 'actforce'}))
  calc_dyn_act = true; % Antriebskraft für Zielfunktion benötigt
end
if any(Set.optimization.constraint_obj(2:3)) % Energie oder Antriebskraft
  calc_dyn_act = true; % Antriebskraft für Nebenbedingung benötigt
end
if Set.optimization.use_desopt
  calc_reg = true; % Entwurfsoptimierung besser mit Regressor
end
if Set.optimization.constraint_link_yieldstrength > 0
  calc_dyn_cut = true; % Schnittkraft für Segmentauslegung benötigt
end
Structure.calc_dyn_act = calc_dyn_act;
Structure.calc_reg = calc_reg;
Structure.calc_dyn_cut = calc_dyn_cut;
%% Optimierungsparameter festlegen
nvars = 0; vartypes = []; varlim = [];

% Roboterskalierung
% Skalierung für Optimierung ist immer positiv und im Verhältnis zur
% Aufgaben-/Arbeitsraumgröße. Darf nicht Null werden.
nvars = nvars + 1;
vartypes = [vartypes; 0];
varlim = [varlim; [1e-3*Lref, 3*Lref]];
varnames = {'scale'};
% Strukturparameter der Kinematik
if Structure.Type == 0 || Structure.Type == 2
  if Structure.Type == 0 % Seriell
    R_pkin = R;
  else  % Parallel
    R_pkin = R.Leg(1);
  end
  Ipkinrel = R_pkin.get_relevant_pkin(Set.structures.DoF);
  % Setze die a1/d1-Parameter für PKM-Beinketten auf Null. diese sind
  % redundant zur Einstellung der Basis-Position oder -Größe
  % (die Parameter werden dann auch nicht optimiert)
  if Structure.Type == 2 % PKM
    I_firstpospar = R_pkin.pkin_jointnumber==1 & (R_pkin.pkin_types==4 | R_pkin.pkin_types==6);
    Ipkinrel = Ipkinrel & ~I_firstpospar; % Nehme die "1" bei d1/a1 weg.
  end
  % Setzen den a2-Parameter zu Null, wenn das erste Gelenk ein Schubgelenk
  % ist. Aus kinematischer Sicht ist dieser Parameter redundant zur PKM-Platt-
  % formgröße (bei senkrechter Anbringung) bzw. auch der Basis-Position
  % (bei schräger Anbringung der Schubgelenke). Der Parameter entspricht
  % keinem Hebelarm eines Drehgelenks.
  if ((Structure.Type == 0 && Set.optimization.movebase) || ... % Seriell mit verschieblicher Basis
      (Structure.Type == 2 && Set.optimization.base_size)) && ... % PKM mit variabler Gestellgröße
     R_pkin.MDH.sigma(1) == 1 % erstes Gelenk ist Schubgelenk
    I_a2 = R_pkin.pkin_jointnumber==2 & R_pkin.pkin_types == 4;
    Ipkinrel = Ipkinrel & ~I_a2; % Nehme die "1" bei a2 weg.
    % Setze den d2-Parameter zu Null, wenn a2 Null ist. Der Parameter ist
    % dann redundant zum vorherigen d1-Parameter (Schubgelenk).
    I_d2 = R_pkin.pkin_jointnumber==2 & R_pkin.pkin_types == 6;
    Ipkinrel = Ipkinrel & ~I_d2; % Nehme die "1" bei d2 weg.
  end
  % Setze den letzten d-Parameter für PKM-Beinketten auf Null. Dieser ist
  % redundant zur Plattform-Größe
  if Structure.Type == 2 % PKM
    I_lastdpar = ((R_pkin.pkin_jointnumber==R_pkin.NJ) & (R_pkin.pkin_types==6));
    Ipkinrel = Ipkinrel & ~I_lastdpar; % Nehme die "1" bei d6 weg.
  end

  pkin_init = R_pkin.pkin;
  pkin_init(~Ipkinrel) = 0; % nicht relevante Parameter Null setzen
  if Structure.Type == 0
    R.update_mdh(pkin_init);
  else
    for i = 1:R.NLEG
      R.Leg(i).update_mdh(pkin_init);
    end
  end
  nvars = nvars + sum(Ipkinrel);
  vartypes = [vartypes; 1*ones(sum(Ipkinrel),1)];
  % Grenzen für Kinematikparameter anhand der Typen bestimmen
  % Nummern, siehe SerRob/get_pkin_parameter_type
  plim = NaN(length(R_pkin.pkin),2);
  for i = 1:size(plim,1)
    if R_pkin.pkin_types(i) == 1
      % Winkel-Parameter beta. Darf eigentlich gar nicht auftreten bei
      % seriellen Robotern. Ginge aber auch mit 0 bis pi/2.
      error('Die Optimierung des Parameters beta ist nicht vorgesehen');
    elseif R_pkin.pkin_types(i) == 3
      % Winkel-Parameter alpha. Nur Begrenzung auf [0,pi/2]. Ansonsten sind
      % negative DH-Längen und negative Winkel redundant. Es wird nur die
      % Parallelität der Gelenke eingestellt
      plim(i,:) = [0, pi/2];
    elseif R_pkin.pkin_types(i) == 5
      % Winkel-Parameter theta. Nur Begrenzung auf [-pi/2,pi/2].
      % Durch Möglichkeit negativer DH-Längen ist jede beliebige
      % Ausrichtung des folgenden Gelenks möglich.
      plim(i,:) = [-pi/2, pi/2];
    elseif R_pkin.pkin_types(i) == 2 || R_pkin.pkin_types(i) == 4 || R_pkin.pkin_types(i) == 6
      % Maximale Länge der einzelnen Segmente
      plim(i,:) = [-1, 1]; % in Optimierung bezogen auf Lref
    else
      error('Parametertyp nicht definiert');
    end
    if Ipkinrel(i)
      varnames = {varnames{:}, sprintf('pkin %d: %s', i, R_pkin.pkin_names{i})}; %#ok<CCAT>
    end
  end
  varlim = [varlim; plim(Ipkinrel,:)];
else
  error('Noch nicht definiert');
end
Structure.Ipkinrel = Ipkinrel;

% Basis-Position. Die Komponenten in der Optimierungsvariablen sind nicht
% bezogen auf die Skalierung. Die Position des Roboters ist nur in einigen
% Fällen in Bezug zur Roboterskalierung (z.B. z-Komponente bei hängendem
% Roboter, Entfernung bei seriellem Roboter)

% Berechne Mittelpunkt der Aufgabe
if Set.optimization.movebase
  % Auswahl der Indizes für die zu optimierenden Basis-Koordinaten
  I_DoF_basepos = Set.structures.DoF(1:3); % nur die FG der Aufgabe nehmen
  I_DoF_setfix = Set.optimization.basepos_limits(:,1)==...
                 Set.optimization.basepos_limits(:,2); 
  for i = 1:3
    % Wenn identische Grenzen vorgegeben werden, diesen FG nicht optimieren
    I_DoF_basepos(i) = I_DoF_basepos(i) & ~I_DoF_setfix(i);
  end
  nvars = nvars + sum(I_DoF_basepos); % Verschiebung um translatorische FG der Aufgabe
  vartypes = [vartypes; 2*ones(sum(I_DoF_basepos),1)];
  if Structure.Type == 0 % Seriell
    % TODO: Stelle den seriellen Roboter vor die Aufgabe
    varlim = [varlim; repmat([-1, 1], sum(I_DoF_basepos), 1)];
  else % Parallel
    % Stelle den parallelen Roboter in/über die Aufgabe
    % Bei Parallelen Robotern ist der Arbeitsraum typischerweise in der
    % Mitte des Gestells (bezogen auf x-y-Ebene). Daher müssen die Grenzen
    % nicht so weit definiert werden: 20% der Referenzlänge um Mittelpunkt
    % der Aufgabe. Annahme: x-/y-Komponente werden immer optimiert.
    varlim = [varlim; repmat([-0.2, 0.2],sum(I_DoF_basepos(1:2)),1)]; % xy-Komponenten
    % Die z-Komponente der Basis kann mehr variieren (hängender Roboter)
    varlim = [varlim; repmat([-1, 1], sum(I_DoF_basepos(3)), 1)];
  end
  % Überschreibe die Grenzen, falls sie explizit als Einstellung gesetzt sind
  bplim = NaN(3,2); % Grenzen für xyz-Koordinaten
  % Nur Koordinaten von oben eintragen, die auch optimiert werden
  bplim(I_DoF_basepos,:) = varlim(end-(sum(I_DoF_basepos)-1):end,:);
  % Einstellungen eintragen, sobald Einstellungen gesetzt sind
  bplim(~isnan(Set.optimization.basepos_limits)) = ...
    Set.optimization.basepos_limits(~isnan(Set.optimization.basepos_limits));
  % Grenzen in Variable für PSO-Parametergrenzen eintragen
  varlim(end-(sum(I_DoF_basepos)-1):end,:) = bplim(I_DoF_basepos,:);
  % Benenne die Variablen
  for i = find(I_DoF_basepos)
    varnames = {varnames{:}, sprintf('base %s', char(119+i))}; %#ok<CCAT>
  end
  % Trage konstante Variablen schon ein (werden später nicht mehr geändert.
  r_W_0 = zeros(3,1);
  r_W_0(I_DoF_setfix) = Set.optimization.basepos_limits(I_DoF_setfix);
else
  % Setze Standard-Werte für Basis-Position fest
  if Structure.Type == 0 % Seriell
    % Stelle den seriellen Roboter vor die Aufgabe
    r_W_0 = Structure.xT_mean + [-0.4*Lref;-0.4*Lref;0];
    if Set.structures.DoF(3) == 1
      r_W_0(3) = -0.7*Lref; % Setze Roboter-Basis etwas unter die Aufgabe
    end
  else % Parallel
    r_W_0 = zeros(3,1);
    if Set.structures.DoF(3) == 1
      r_W_0(3) = Structure.xT_mean(3)-0.7*Lref; % Setze Roboter mittig unter die Aufgabe (besser sichtbar)
    end
    % In der xy-Ebene liegt der Roboter in der Mitte der Aufgabe
    r_W_0(1:2) = Structure.xT_mean(1:2);
  end
end
R.update_base(r_W_0);
% EE-Verschiebung
if Set.optimization.ee_translation && ...
    (Structure.Type == 0 || Structure.Type == 2 && ~Set.optimization.ee_translation_only_serial)
  % (bei PKM keine EE-Verschiebung durchführen. Dort soll das EE-KS bei
  % gesetzter Option immer in der Mitte sein)
  nvars = nvars + sum(Set.structures.DoF(1:3)); % Verschiebung des EE um translatorische FG der Aufgabe
  vartypes = [vartypes; 3*ones(sum(Set.structures.DoF(1:3)),1)];
  varlim = [varlim; repmat([-1, 1], sum(Set.structures.DoF(1:3)), 1)]; % bezogen auf Lref
  for i = find(Set.structures.DoF(1:3))
    varnames = {varnames{:}, sprintf('ee pos %s', char(119+i))}; %#ok<CCAT>
  end
end

% EE-Rotation
if Set.optimization.ee_rotation
  if sum(Set.structures.DoF(4:6)) == 1
    neerot = 1;
  elseif sum(Set.structures.DoF(4:6)) == 0
    neerot = 0;
  elseif sum(Set.structures.DoF(4:6)) == 2
    % Bei 3T2R wird die Rotation um die Werkzeugachse nicht optimiert.
    neerot = 2;
  else
    neerot = 3;
  end
  nvars = nvars + neerot; % Verschiebung des EE um translatorische FG der Aufgabe
  vartypes = [vartypes; 4*ones(neerot,1)];
  varlim = [varlim; repmat([0, pi], neerot, 1)];
  for i = find(Set.structures.DoF(4:6))
    varnames = [varnames(:)', {sprintf('ee rot %d', i)}];
  end
end

% Basis-Koppelpunkt Positionsparameter (z.B. Gestell-Radius)
if Structure.Type == 2 && Set.optimization.base_size
  % TODO: Die Anzahl der Positionsparameter könnte sich evtl ändern
  % Eventuell ist eine Abgrenzung verschiedener Basis-Anordnungen sinnvoll
  nvars = nvars + 1;
  vartypes = [vartypes; 6];
  % TODO: Untergrenze muss noch sinnvoll gewählt werden (darf nicht Null
  % sein)
  if all(~isnan(Set.optimization.base_size_limits))
    % Nehme absolute Werte (vorgegeben durch Benutzer)
    varlim = [varlim; Set.optimization.base_size_limits];
  else
    % Automatische Einstellung: fünf-fache spezifische Länge als Basis-Radius
    varlim = [varlim; [0.1,5]];
  end
  varnames = {varnames{:}, 'base radius'}; %#ok<CCAT>
end

% Plattform-Koppelpunkt Positionsparameter (z.B. Plattform-Radius)
if Structure.Type == 2 && Set.optimization.platform_size
  nvars = nvars + 1;
  vartypes = [vartypes; 7];
  if all(~isnan(Set.optimization.platform_size_limits))
    % Nehme absolute Werte (vorgegeben durch Benutzer)
    varlim = [varlim; Set.optimization.platform_size_limits];
  else
    % Automatische Einstellung: Bezogen auf Gestell-Radius
    % max. zwei-facher Gestell-Radius als Plattform-Radius
    varlim = [varlim; [0.1,2]]; 
  end
  varnames = {varnames{:}, 'platform radius'}; %#ok<CCAT>
end

% Gestell-Morphologie-Parameter (z.B. Gelenkpaarabstand).
% Siehe align_base_coupling.m
if Structure.Type == 2 && Set.optimization.base_morphology
  if any(R.DesPar.base_method == 5:8) % Paarweise Anordnung der Beinketten
    nvars = nvars + 1;
    vartypes = [vartypes; 8];
    varlim = [varlim; [0.2,0.8]]; % Gelenkpaarabstand. Relativ zu Gestell-Radius.
    varnames = {varnames{:}, 'base_morph_pairdist'}; %#ok<CCAT>
  end
  if any(R.DesPar.base_method == [4 8]) % Erste Achse hat eine Steigung gegen die Mitte
    nvars = nvars + 1;
    vartypes = [vartypes; 8];
    % Die Steigung wird gegen die Senkrechte "nach innen kippend" gezählt. 
    % Damit die erste Achse nach unten zeigt, muss der Winkel größer 90° sein. 
    % Als Sonderfall ist Steigung 90° (bleibt in der Ebene) und Steigung 0°
    % (senkrechte Anordnung nach oben) und 180° (senkrecht nach unten) ent- 
    % halten. Das ist der Übergang zu anderen Gestell-Varianten.
    % Der Bereich 0-180° ist notwendig, damit bei Schubachsen ein zusammen-
    % laufen unter- oder oberhalb des Gestells möglich ist.
    varlim = [varlim; [0, pi]]; % Steigung; Winkel in rad
    if R.DesPar.base_method == 4
      % Pyramide
      varnames = {varnames{:}, 'base_morph_coneelev'}; %#ok<CCAT>
    else
      % Kegel
      varnames = {varnames{:}, 'base_morph_pyrelev'}; %#ok<CCAT>
    end
  end
end

% Plattform-Morphologie-Parameter (z.B. Gelenkpaarabstand).
% Siehe align_platform_coupling.m
if Structure.Type == 2 && Set.optimization.platform_morphology
  if R.DesPar.platform_method == 1 % keine Parameter bei Kreis
  elseif R.DesPar.platform_method == 4
    nvars = nvars + 1;
    vartypes = [vartypes; 9];
    varlim = [varlim; [0.2,0.8]]; % Gelenkpaarabstand. Relativ zu Plattform-Radius.
    varnames = {varnames{:}, 'platform_morph'}; %#ok<CCAT>
  else
    error('platform_morphology Nicht implementiert');
  end
end
% Variablen-Typen speichern
Structure.vartypes = vartypes;
Structure.varnames = varnames;
assert(length(vartypes)==size(varlim,1), 'Anzahl der Variablen muss konsistent zur Anzahl der gesetzten Grenzen sein');
assert(length(vartypes)==length(varnames), 'Abgespeicherte Variablennamen stimmen scheinbar nicht');
assert(all(varlim(:,1)~=varlim(:,2)), 'Obere und untere Parametergrenze identisch. Optimierung nicht sinnvoll.')
%% Weitere Struktureigenschaften abspeichern
% Bestimme die Indizes der ersten Schubgelenke. Das kann benutzt werden, um
% Gelenkgrenzen für das erste Schubgelenk anders zu bewerten.
if R.Type == 0 % Seriell
  I_firstprismatic = false(R.NJ,1);
  if R.MDH.sigma(1) == 1, I_firstprismatic(1) = true; end
else % PKM
  I_first = false(R.NJ,1);
  I_first(R.I1J_LEG) = true;
  I_prismatic = (R.MDH.sigma == 1);
  I_firstprismatic = I_first & I_prismatic;
end
Structure.I_firstprismatic = I_firstprismatic;

%% Initialisierung der Kollisionsprüfung
if Set.optimization.constraint_collisions || ~isempty(Set.task.obstacles.type) || ...
    ~isempty(Set.task.installspace.type)
  % Lege die Starrkörper-Indizes fest, für die Kollisionen geprüft werden
  selfcollchecks_bodies = [];
  % Prüfe Selbstkollisionen einer kinematischen Kette.
  for k = 1:NLEG
    % Erneute Initialisierung (sonst eventuell doppelte Eintragungen)
    collbodies = struct('link', [], 'type', [], 'params', []); % Liste für Beinkette
    if Structure.Type == 0  % Seriell 
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
    % Erzeuge Kollisionskörper für den statischen Teil von Schub- 
    % gelenken (z.B. Linearachsen). Siehe: SerRob/plot
    for i = find(R_cc.MDH.sigma'==1)
      % MDH-Trafo (konstanter Teil; bezogen auf Basis-KS)
      T_mdh1 = trotz(R_cc.MDH.beta(i))*transl([0;0;R_cc.MDH.b(i)]) * ...
               trotx(R_cc.MDH.alpha(i))*transl([R_cc.MDH.a(i);0;0]);
      T_qmin = T_mdh1 * transl([0;0;R_cc.qlim(i,1)]);
      T_qmax = T_qmin * transl([0;0;R_cc.qlim(i,2)-R_cc.qlim(i,1)]);
      % Prüfe Art des Schubgelenks
      if R_cc.DesPar.joint_type(i) == 4 % Führungsschiene
        % Füge die Führungsschiene der Linearachse als Körper hinzu.
        % Wird als Kapsel durch Anfang und Ende gekennzeichnet.
        % Bilde die MDH-Transformation nach. Das führt zu min-max für q
        cbi_par = [T_qmin(1:3,4)', T_qmax(1:3,4)', 20e-3]; % Radius 20mm
      elseif R_cc.DesPar.joint_type(i) == 5 % Hubzylinder
        % Der äußere Zylinder muss so lang sein wie der innere (bzw. der
        % innere Zylinder muss so lang sein wie der Hub).
        T_grozyl_start = T_qmin * transl([0;0;-(R_cc.qlim(i,2)-R_cc.qlim(i,1))]);
        T_grozyl_end = T_qmax;
        cbi_par = [T_grozyl_start(1:3,4)', T_grozyl_end(1:3,4)', 20e-3];
      else
        error('Fall %d für Schubgelenk nicht vorgesehen', R_cc.DesPar.joint_type(i));
      end
      if i == 1 % erstes Gelenk der Kette ist Schubgelenk. Führungsschiene basisfest.
        % Kapsel, zwei Punkte (im weltfesten Basis-KS der Kette)
        collbodies.type = [collbodies.type; uint8(3)];
        % Umrechnung der Parameter ins Welt-KS: Notwendig. Aber hier
        % ignoriert.
      else
        warning(['Die Kollisionsprüfung für die Führungsschiene des P-Gelenks ', ...
          'an Stelle %d ist nicht definiert. Wird vorerst ignoriert.'], i);
        continue
        % Kapsel, zwei Punkte (im mitbewegten Körper-KS)
        collbodies.type = [collbodies.type; uint8(3)]; %#ok<UNRCH>
      end
      collbodies.params = [collbodies.params; cbi_par, NaN(1,3)];
      % Führungsschiene/Führungszylinder ist vorherigem Segment zugeordnet
      collbodies.link = [collbodies.link; uint8(i-1)];
    end
    
    % Erzeuge Ersatzkörper für die kinematische Kette (aus Gelenk-Trafo)
    for i = 1:R_cc.NJ
      if R_cc.MDH.a(i) ~= 0 || R_cc.MDH.d(i) ~= 0 || R_cc.MDH.sigma(i) == 1
        % Es gibt eine Verschiebung in der Koordinatentransformation i
        % Definiere einen Ersatzkörper dafür
        collbodies.link =   [collbodies.link; uint8(i)];
        collbodies.type =   [collbodies.type; uint8(6)]; % Kapsel, direkte Verbindung
        % Wähle Kapseln mit Radius 20mm. R.DesPar.seg_par ist noch nicht belegt
        % (passiert erst in Entwurfsoptimierung).
        collbodies.params = [collbodies.params; 20e-3, NaN(1,9)];
      end
    end
    R_cc.collbodies = collbodies;
    % Trage die Kollisionsprüfungen ein
    for i = 3:R_cc.NJ
      % Füge Prüfung mit allen vorherigen hinzu (außer direktem Vorgänger)
      % Diese Kollision wird nicht geprüft, da dort keine Kollision
      % stattfinden können sollte (direkt gegeneinander drehbare Teile
      % können konstruktiv kollisionsfrei gestaltet werden).
      % Der Vorgänger bezieht sich auf den Kollisionskörper, nicht auf
      % die Nummer des Starrkörpers. Ansonsten würden zwei durch Kugel-
      % oder Kardan-Gelenk verbundene Körper in Kollision stehen.
      j_hascollbody = collbodies.link(collbodies.link<i)';
      % Sonderfall Portal-System: Abstand zwischen Kollisionskörpern noch
      % um eins vergrößern. TODO: Ist so noch nicht allgemeingültig.
      % Dadurch wird die Kollisionsprüfung effektiv deaktiviert.
      if sum(R_cc.MDH.sigma(i-2:i) == 1) == 3
        cbdist = 3;
      else
        if j_hascollbody(end-1) == 0
          % Der übernächste Kollisionskörper wäre die Basis (und damit
          % vermutlich eine Führungsschiene).
          % Überspringe diese Prüfung. Nehme die Führungsschiene nur, wenn
          % es zwei weiter ist. Ansonsten wird bei PUU-Ketten immer eine
          % Selbstkollision erkannt.
          cbdist = 2;
        else
          % Standardfall: Ab dem übernächsten Körper wird geprüft
          cbdist = 1;
        end
      end
      % Bestimme die Starrkörper-Nummer bezogen auf die PKM mit NLoffset
      for j = j_hascollbody(1:end-cbdist) % Kollisionskörper mehr als zwei vorher
        % Füge zur Prüfliste hinzu. Durch obige Erstellung der Indizes j
        % wird sichergestellt, dass es hierzu einen Koll.-körper gibt.  
        selfcollchecks_bodies = [selfcollchecks_bodies; ...
          uint8(NLoffset+[i, j])]; %#ok<AGROW>
        % fprintf('Kollisionsprüfung (%d): Beinkette %d Seg. %d Seg. %d. Zeile [%d,%d]\n', ...
        %   size(selfcollchecks_bodies,1), k, i, j, selfcollchecks_bodies(end,1), selfcollchecks_bodies(end,2));
      end
    end
  end % k-loop (NLEG)
  % Roboter-Kollisionsobjekte in Struktur abspeichern (zum Abruf in den
  % Funktionen cds_constr_collisions_... und cds_constr_installspace
  % Ist erstmal nur Platzhalter. Wird zur Laufzeit noch aktualisiert.
  Structure.collbodies_robot = cds_update_collbodies(R, Set, Structure, Structure.qlim');
  % Probe: Sind Daten konsistent? Inkonsistenz durch obigem Aufruf möglich.
  if any(any(~isnan(Structure.collbodies_robot.params(Structure.collbodies_robot.type==6,2:end))))
    error('Inkonsistente Kollisionsdaten: Kapsel-Direktverbindung hat zu viele Parameter');
  end
  % Vorgänger-Indizes für Segmente für die Kollisionsprüfung abspeichern.
  % Unterscheidet sich von normaler MDH-Notation dadurch, dass alle
  % Beinketten-Basis-KS enthalten sind und die Basis ihr eigener Vorgänger
  % ist (vereinfacht spätere Implementierung)
  if Structure.Type == 0  % Seriell 
    v = uint8([0;R.MDH.v]); % zusätzlicher Dummy-Eintrag für Basis
  else % PKM
    % Jede Beinkette hat zusätzliches Basis-KS
    v = uint8(zeros(1+R.NJ+R.NLEG,1));
    for k = 1:R.NLEG
      if k > 1, NLoffset = 1+R.I2L_LEG(k-1)-(k-1);
      else, NLoffset = 1; end
      v(R.I1J_LEG(k)+k:R.I2J_LEG(k)+k+1) = [0; NLoffset+R.Leg(k).MDH.v];
    end
  end
  Structure.MDH_ante_collcheck = v;
  
  % Starrkörper-Kollisionsprüfung für PKM erweitern
  if Structure.Type == 2  % PKM
    % Auch Kollisionen aller Beinsegmente mit allen anderen Beinketten
    % prüfen. Einschränkungen: Nur direkt benachbarte Beinketten prüfen
    for k = 1:NLEG
      if k > 1, NLoffset_k = 1+R.I2L_LEG(k-1)-(k-1);
      else,     NLoffset_k = 1; end
      for j = k-1 % Beinketten müssen benachbart sein. Prüfe nur linken Nachbarn
        % Durch Prüfung aller Beinketten sind auch alle Nachbarn abgedeckt
        % Rechne Beinkette 0-7 um in 1-6 (zur einfachereren Zählung)
        i = j; % Periodizität der Nummern
        if i == 0, i = NLEG; end
        if i == NLEG+1, i = 1; end
        % Offset der Beinketten-Körper in den PKM-Körpern
        if i > 1, NLoffset_i = 1+R.I2L_LEG(i-1)-(i-1);
        else,     NLoffset_i = 1; end
        % Alle Segmente von Beinkette k können mit allen von Kette j
        % kollidieren
        for cb_k = R.Leg(k).collbodies.link'
          for cb_i = R.Leg(i).collbodies.link'
            selfcollchecks_bodies = [selfcollchecks_bodies; ...
              uint8([NLoffset_k+cb_k, NLoffset_i+cb_i])]; %#ok<AGROW>
            % fprintf(['Kollisionsprüfung (%d): Bein %d Seg. %d vs Bein %d Seg. %d. ', ...
            %   'Zeile [%d,%d]\n'], size(selfcollchecks_bodies,1), k, cb_k, i, ...
            %   cb_i, selfcollchecks_bodies(end,1), selfcollchecks_bodies(end,2));
          end
        end
      end
    end
    % TODO: Kollision Beinketten mit Plattform und Gestell
  end
  if isempty(selfcollchecks_bodies)
    warning('Es sind keine Kollisionskörpern eingetragen, obwohl Kollisionen geprüft werden sollen.');
  elseif any(selfcollchecks_bodies(:,1)==selfcollchecks_bodies(:,2))
    error('Prüfung eines Körpers mit sich selbst ergibt keinen Sinn');
  end
  % Lege die Kollisionskörper-Indizes fest, für die Kollisionen geprüft werden
  % Der Inhalt sind direkt die Indizes von collbodies. Das muss nicht
  % online in der Optimierung gemacht werden.
  % Abgrenzung von "bodies" (oben) und "collbodies" (ab hier) beachten.
  Structure.selfcollchecks_collbodies = [];
  for i = 1:size(selfcollchecks_bodies,1)
    % Finde die Indizes aller Ersatzkörper der zu prüfenden Starrkörper
    % (es kann auch mehrere Ersatzkörper für einen Starrkörper geben)
    I1 = selfcollchecks_bodies(i,1) == Structure.collbodies_robot.link;
    I2 = selfcollchecks_bodies(i,2) == Structure.collbodies_robot.link;
    CheckCombinations = NaN(sum(I1)*sum(I2),2);
    if sum(I1) == 0 || sum(I2) == 0
      % Die Kollision der Starrkörper soll geprüft werden, es sind aber gar
      % keine Ersatzkörper definiert.
      continue
    end
    kk = 0;
    for ii1 = find(I1)'
      for ii2 = find(I2)'
        kk = kk + 1;
        CheckCombinations(kk,:) = [ii1,ii2];
        % fprintf('Kollisionsprüfung (%d): Koll.-körper %d (Seg. %d) vs Koll.-körper %d (Seg. %d).\n', ...
        %   i, ii1, Structure.collbodies_robot.link(ii1), ii2, Structure.collbodies_robot.link(ii2));
      end
    end
    if any(CheckCombinations(:,1)==CheckCombinations(:,2))
      error('CheckCombinations: Prüfung eines Körpers mit sich selbst ergibt keinen Sinn');
    end
    % Eintragen in Gesamt-Liste
    Structure.selfcollchecks_collbodies = ...
      uint8([Structure.selfcollchecks_collbodies; CheckCombinations]);
  end
  if isempty(Structure.selfcollchecks_collbodies)
    warning('Es sind keine Prüfungen von Kollisionskörpern vorgesehen');
    % Deaktiviere die Kollisionsprüfungen wieder
    Set.optimization.constraint_collisions = false;
    Set.task.obstacles.type = [];
  elseif any(Structure.selfcollchecks_collbodies(:,1)==Structure.selfcollchecks_collbodies(:,2))
    error('Prüfung eines Körpers mit sich selbst ergibt keinen Sinn');
  end
end
%% Initialisierung der Bauraumprüfung
% Erstelle Liste der Kollisionsprüfungen für cds_constr_installspace.m
% Die Geometrie-Objekte werden erst dort ins Basis-KS des Roboters trans-
% formiert.
if ~isempty(Set.task.installspace.type)
  % Liste für alle Kollisionskörper des Roboters bei Bauraumprüfung.
  % Es werden nur Punkte anstatt der Ersatz-Volumen benutzt.
  % Das Format der Struktur ist genauso wie oben.
  collbodies_instspc = struct('link', [], 'type', [], 'params', []);
  instspc_collchecks_collbodies = []; % Liste der Bauraum-Kollisionsprüfungen (bezogen auf obige Variable)
  % Stelle Ersatz-Punkte für alle Robotergelenke zusammen
  for k = 1:NLEG
    % Offset für PKM-Beinketten bestimmen, s.o.
    if Structure.Type == 0  % Seriell 
      NLoffset = 0;
      R_cc = R;
    else % PKM-Beinkette
      NLoffset = 1;
      R_cc = R.Leg(k);
      if k > 1
        NLoffset = 1+R.I2L_LEG(k-1)-(k-1); % in I1L wird auch Basis und EE-Link noch mitgezählt. Hier nicht.
      end
    end
    % Erzeuge Kollisionskörper für den statischen Teil von Schub- 
    % gelenken (z.B. Linearachsen). Siehe: SerRob/plot
    for i = find(R_cc.MDH.sigma'==1)
      % Hänge zwei Punkte für Anfang und Ende jeder Linearführung an
      % Setze als Körper Nr. 14 (Punkt im Basis-KS)
      collbodies_instspc.type = [collbodies_instspc.type; repmat(uint8(14),2,1)];
      collbodies_instspc.params = [collbodies_instspc.params; NaN(2,10)];
      % Führungsschiene/Führungszylinder ist vorherigem Segment zugeordnet
      collbodies_instspc.link = [collbodies_instspc.link; repmat(NLoffset+uint8(i-1),2,1)];
    end
    % Hänge einen Punkt (Nr. 9) für jedes Gelenk an. Unabhängig, ob 3D-Körper dafür
    collbodies_instspc.type = [collbodies_instspc.type; repmat(uint8(9),R_cc.NJ,1)];
    collbodies_instspc.params = [collbodies_instspc.params; NaN(R_cc.NJ,10)];
    collbodies_instspc.link = [collbodies_instspc.link; uint8(NLoffset+(1:R_cc.NJ)')];
  end
  % Stelle äquivalente Gelenknummer von PKM zusammen (als  Übersetzungs- 
  % tabelle). Zeile 1 ursprünglich, Zeile 2 Übersetzung. Siehe oben.
  % Seriell: 0=Basis, 1=erstes bewegtes Segment, ...
  equiv_link = repmat(0:collbodies_instspc.link(end),2,1);
  if Structure.Type == 2 % PKM
    % Kollisionskörper-Zählung: 0=Basis, 1=Beinkette1-Basis, 2=Beinkette1-1.Seg., ...
    % Zählung für äquivalente Indizes: PKM-Basis und Beinketten-Basis ist
    % alles Körper 0. Ansonsten ist der erste bewegte Körper nach der Basis
    % 1, usw.
    for k = 1:R.NLEG
      NLoffset = 1; % Für Basis der Beinkette
      if k > 1
        NLoffset = 1+R.I2L_LEG(k-1)-(k-1); % siehe andere Vorkommnisse oben
      end
      for jj = 0:R.Leg(1).NL-1
        % Segment-Nummer in Gesamt-PKM nach Kollisionskörper-Zählung
        iil = NLoffset+jj;
        equiv_link(2,iil+1) = jj;
      end
    end
  end
  n_cb_robot = size(collbodies_instspc.type,1);
  for i = 1:size(Set.task.installspace.type,1)
    % Prüfe, für welches Robotersegment das Bauraum-Geometrieobjekt i
    % definiert ist
    links_i = Set.task.installspace.links{i}; % erlaubte Segmente
    % Gehe alle Kollisionskörper j des Roboters durch und prüfe, ob passend
    for j = 1:size(collbodies_instspc.type,1)
      % Nehme nicht die Segmentnummer selbst, sondern die des äquivalenten
      % Segments (jew. von der Basis aus gesehen, egal ob PKM oder seriell)
      iii = equiv_link(1,:) == collbodies_instspc.link(j);
      equiv_link_j = equiv_link(2,iii); % 0=Basis, 1=erstes bewegtes,...
      if any(links_i == equiv_link_j)
        instspc_collchecks_collbodies = [instspc_collchecks_collbodies; ...
          uint8([j, n_cb_robot+i])]; %#ok<AGROW> % Prüfe "Kollision" Roboter-Segment j mit Bauraum-Begrenzung i
      end
    end
  end
  Structure.installspace_collbodies = collbodies_instspc; % Eingabe vorbereiten
  [~,collbodies_instspc] = cds_update_collbodies(R, Set, Structure, Structure.qlim');
  Structure.installspace_collbodies = collbodies_instspc; % modifizerte Struktur abspeichern
  Structure.installspace_collchecks_collbodies = instspc_collchecks_collbodies;
end

%% Anfangs-Population generieren
% TODO: Existierende Roboter einfügen

NumIndividuals = Set.optimization.NumIndividuals;
% Generiere Anfangspopulation aus Funktion mit Annahmen bezüglich Winkel
InitPop = cds_gen_init_pop(NumIndividuals,nvars,varlim,varnames,vartypes);
%% PSO-Einstellungen festlegen
options = optimoptions('particleswarm');
options.Display='iter';
options.MaxIter = Set.optimization.MaxIter; %70 100 % in GeneralConfig
% options.StallIterLimit = 2;
options.SwarmSize = NumIndividuals;
if any(strcmp(Set.optimization.objective, {'valid_act', 'valid_kin'}))
  % Es soll nur geprüft werden, ob es eine zulässige Lösung gibt.
  % Breche bei einer erfolgreichen Berechnung der Zulässigkeit ab.
  options.ObjectiveLimit = 999;
end
options.InitialSwarmMatrix = InitPop;
if ~Set.general.noprogressfigure
  options.PlotFcn = {@pswplotbestf};
end
cds_save_all_results_anonym = @(optimValues,state)cds_psw_save_all_results(optimValues,state,Set,Structure);
options.OutputFcn = {cds_save_all_results_anonym};
%% Tmp-Ordner leeren
resdir = fullfile(Set.optimization.resdir, Set.optimization.optname, ...
  'tmp', sprintf('%d_%s', Structure.Number, Structure.Name));
if exist(resdir, 'file')
  % Leere Verzeichnis
  rmdir(resdir, 's')
end
mkdirs(resdir);
%% Fitness-Funktion initialisieren (Strukturunabhängig)
% Zurücksetzen der gespeicherten Werte (aus vorheriger Maßsynthese)
clear cds_fitness
fitnessfcn=@(p)cds_fitness(R, Set, Traj, Structure, p(:));
f_test = fitnessfcn(InitPop(1,:)'); %#ok<NASGU> % Testweise ausführen
% Zurücksetzen der Detail-Speicherfunktion
cds_save_particle_details(Set, R, 0, 0, 0, 0, 'reset');
% Zurücksetzen der gespeicherten Werte der Fitness-Funktion
clear cds_fitness
%% PSO-Aufruf starten
if Set.general.matfile_verbosity > 0
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_robot2.mat'));
end
if false
  % Falls der PSO abbricht: Zwischenergebnisse laden und daraus Endergebnis
  % erzeugen. Dafür ist ein manuelles Eingreifen mit den Befehlen in diesem
  % Block erforderlich. Danach kann die Funktion zu Ende ausgeführt werden.
  load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_robot2.mat')); %#ok<LOAD,UNRCH>
  filelist_tmpres = dir(fullfile(resdir, 'PSO_Gen*_AllInd_iter.mat'));
  lastres = load(fullfile(resdir, filelist_tmpres(end).name));
  p_val = lastres.optimValues.bestx;
  fval = lastres.optimValues.bestfval;
  exitflag = -6;
else
  % PSO wird ganz normal ausgeführt.
  [p_val,fval,exitflag,output] = particleswarm(fitnessfcn,nvars,varlim(:,1),varlim(:,2),options);
  cds_log(1, sprintf('[dimsynth] Optimierung beendet. iterations=%d, funccount=%d, message: %s', ...
    output.iterations, output.funccount, output.message));
end
% Detail-Ergebnisse extrahieren (persistente Variable in Funktion)
PSO_Detail_Data = cds_save_particle_details(Set, R, 0, 0, 0, 0, 'output');
% Zurücksetzen, damit Neuberechnungen der Fitness-Funktion nicht fehlschlagen
cds_save_particle_details(Set, R, 0, 0, 0, 0, 'reset');
clear cds_fitness
% Debug:
if Set.general.matfile_verbosity > 0
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_robot3.mat'));
end
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_robot3.mat'));
%% Nachverarbeitung der Ergebnisse

% Fitness-Funktion nochmal mit besten Parametern aufrufen. Dadurch werden
% die Klassenvariablen (R.pkin, R.DesPar.seg_par, ...) aktualisiert
for i = 1:Set.general.max_retry_bestfitness_reconstruction
  % Mehrere Versuche vornehmen, da beim Umklappen der Roboterkonfiguration
  % andere Ergebnisse entstehen können.
  % Eigentlich darf sich das Ergebnis aber nicht ändern (wegen der
  % Zufallszahlen-Initialisierung in cds_fitness).
  fval_test = fitnessfcn(p_val');
  if fval_test~=fval
    if fval_test < fval
      t = sprintf('Der neue Wert (%1.1f) ist um %1.1e besser als der alte (%1.1f).', ...
        fval_test, fval-fval_test, fval);
    else
      t = sprintf('Der alte Wert (%1.1f) ist um %1.1e besser als der neue (%1.1f).', ...
        fval, fval_test-fval, fval_test);
    end
    cds_log(-1, sprintf('[dimsynth] Bei nochmaligem Aufruf der Fitness-Funktion kommt nicht der gleiche Wert heraus (Versuch %d). %s', i, t));
    if fval_test < fval
      fval = fval_test;
      cds_log(1,sprintf('[dimsynth] Nehme den besseren neuen Wert als Ergebnis ...'));
      break;
    end
  else
    if i > 1, cds_log(1,sprintf('[dimsynth] Zielfunktion konnte nach %d Versuchen rekonstruiert werden', i)); end
    break;
  end
end
% Schreibe die Anfangswerte der Gelenkwinkel für das beste Individuum in
% die Roboterklasse. Suche dafür den besten Funktionswert in den zusätzlich
% gespeicherten Daten für die Position des Partikels in dem
% Optimierungsverfahren
k = find(fval == PSO_Detail_Data.fval', 1, 'first');
[dd_optind,dd_optgen] = ind2sub(fliplr(size(PSO_Detail_Data.fval)),k); % Umrechnung in 2D-Indizes: Generation und Individuum
q0_ik = PSO_Detail_Data.q0_ik(dd_optind,:,dd_optgen)';

% Prüfen, ob diese mit den im Optimierungsprozess gespeicherten IK-Anfangs-
% winkeln übereinstimmen
if Structure.Type == 0, q0_ik2 = R.qref;
else,                   q0_ik2 = cat(1,R.Leg.qref); end
test_q0 = q0_ik - q0_ik2;
test_q0(abs(abs(test_q0)-2*pi)<1e-6) = 0; % entferne 2pi-Fehler
if any(test_q0~=0) && fval<1e9 % nur bei erfolgreicher Berechnung der IK ist der gespeicherte Wert sinnvoll
  cds_log(-1, sprintf('[dimsynth] IK-Anfangswinkeln sind bei erneuter Berechnung anders. Darf nicht passieren. max. Abweichung: %1.2e.', max(abs(test_q0))));
end
% Berechne Inverse Kinematik zu erstem Bahnpunkt
Traj_0 = cds_rotate_traj(Traj, R.T_W_0);
if Structure.Type == 0 % Seriell
  % Benutze Referenzpose die bei obigen Zielfunktionsaufruf gespeichert wurde
  [q, Phi] = R.invkin2(Traj_0.XE(1,:)', R.qref);
else % Parallel
  [q, Phi] = R.invkin_ser(Traj_0.XE(1,:)', cat(1,R.Leg.qref));
end
if ~strcmp(Set.optimization.objective, 'valid_act') && any(abs(Phi)>1e-8)
  cds_log(-1, '[dimsynth] PSO-Ergebnis für Startpunkt nicht reproduzierbar (ZB-Verletzung)');
end
% Berechne IK der Bahn (für spätere Visualisierung und Neuberechnung der Zielfunktionen)
% Benutze ähnliche Einstellungen wie in cds_constraints.m (aber feinere
% Toleranz und mehr Rechenaufwand bei der eigentlichen IK-Berechnung)
% Hier auch Weglassen der Beachtung der Winkelgrenzen (führt teilweise zu
% Abbruch, obwohl die Spannweite in Ordnung ist.)
if Structure.Type == 0 % Seriell
  s = struct('normalize', false, 'retry_limit', 0, 'Phit_tol', 1e-12, ...
    'Phir_tol', 1e-12, 'n_max', 5000, 'scale_lim', 0);
  [Q, QD, QDD, PHI] = R.invkin2_traj(Traj_0.X, Traj.XD, Traj.XDD, Traj.t, q, s);
  Jinv_ges = [];
else % Parallel
  s = struct('retry_limit', 0, 'Phit_tol', 1e-12, ...
    'Phir_tol', 1e-12, 'n_max', 5000, 'scale_lim', 0);
  [Q, QD, QDD, PHI, Jinv_ges] = R.invkin2_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD, Traj_0.t, q, s);
end
test_q = abs(Q(1,:)'-q0_ik);
test_q(abs(abs(test_q)-2*pi)<1e-2) = 0; % entferne 2pi-Fehler, großzügige Toleranz
if any(test_q > 1e-6)
  cds_log(-1, sprintf(['[dimsynth] Die Neu berechneten IK-Werte (q0) der Trajektorie stimmen nicht ', ...
    'mehr mit den ursprünglich berechneten überein. Max diff.: %1.4e'], max(test_q)));
end
result_invalid = false;
if ~strcmp(Set.optimization.objective, 'valid_act') && ...
    (any(abs(PHI(:))>1e-6) || any(isnan(Q(:)))) % Toleranz wie in cds_constraints
  cds_log(-1, sprintf(['[dimsynth] PSO-Ergebnis für Trajektorie nicht reproduzierbar ', ...
    'oder nicht gültig (ZB-Verletzung). Max IK-Fehler: %1.1e. %d Fehler > 1e-6. %d mal NaN.'], ...
    max(abs(PHI(:))), sum(sum(abs(PHI)>1e-6,2)>0), sum(isnan(Q(:))) ));
  result_invalid = true;
end
%% Berechne andere Zielfunktionen
Structure_tmp = Structure; % Eingabe um Berechnung der Antriebskräfte zu erzwingen
Structure_tmp.calc_dyn_act = true;
Structure_tmp.calc_dyn_cut = true; % ... und der Schnittkräfte
Structure_tmp.calc_reg = false;
if R.Type ~= 0 % für PKM
  % Berechne Dynamik in diesem Abschnitt mit Inertialparametern. Sonst
  % keine Berechnung der Schnittkräfte möglich (mit Minimalparametern)
	R.DynPar.mode = 3; 
  for i = 1:R.NLEG, R.Leg(i).DynPar.mode = 3; end
end
if ~result_invalid && ~strcmp(Set.optimization.objective, 'valid_act')
  % Masseparameter belegen, falls das nicht vorher passiert ist.
  if fval > 1e3 ...% irgendeine Nebenbedingung wurde immer verletzt. ...
      || any(strcmp(Set.optimization.objective, {'condition', 'jointrange'})) % ... oder rein kinematische Zielfunktion ...
    cds_dimsynth_design(R, Q, Set, Structure); % ...  Daher nie bis zu diesem Funktionsaufruf gekommen.
  end
  data_dyn = cds_obj_dependencies(R, Traj_0, Set, Structure_tmp, Q, QD, QDD, Jinv_ges);
  % Einzelne Zielfunktionen aufrufen
  [fval_energy,~, ~, physval_energy] = cds_obj_energy(R, Set, Structure, Traj_0, data_dyn.TAU, QD);
  [fval_cond,~, ~, physval_cond] = cds_obj_condition(R, Set, Structure, Jinv_ges, Traj_0, Q, QD);
  [fval_mass,~, ~, physval_mass] = cds_obj_mass(R);
  [fval_actforce,~, ~, physval_actforce] = cds_obj_actforce(data_dyn.TAU);
  [fval_jrange,~, ~, physval_jrange] = cds_obj_jointrange(R, Set, Structure, Q);
  [fval_stiff,~, ~, physval_stiff] = cds_obj_stiffness(R, Set, Q);
  [~, ~, f_maxstrengthviol] = cds_constr_yieldstrength(R, Set, data_dyn, Jinv_ges, Q, Traj_0);
  % Reihenfolge siehe Variable Set.optimization.constraint_obj aus cds_settings_defaults
  fval_obj_all = [fval_mass; fval_energy; fval_actforce; fval_cond; fval_jrange; fval_stiff];
  fval_constr_all = f_maxstrengthviol;
  physval_obj_all = [physval_mass; physval_energy; physval_actforce; physval_cond; physval_jrange; physval_stiff];
  % Vergleiche neu berechnete Werte mit den zuvor abgespeicherten (müssen
  % übereinstimmen)
  test_Jcond = PSO_Detail_Data.Jcond(dd_optgen, dd_optind) - physval_cond;
  if abs(test_Jcond) > 1e-6
    save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', ...
      sprintf('%s_Rob%d_%s_cds_dimsynth_robot_condreprowarning.mat', Set.optimization.optname, ...
      Structure.Number, Structure.Name)));
    cds_log(-1, sprintf('[dimsynth] Während Optimierung gespeicherte Konditionszahl (%1.5e) stimmt nicht mit erneuter Berechnung (%1.5e) überein. Differenz %1.5e.', ...
      PSO_Detail_Data.Jcond(dd_optgen, dd_optind), physval_cond, test_Jcond));
  end
  test_sv = PSO_Detail_Data.f_maxstrengthviol(dd_optgen, dd_optind) - f_maxstrengthviol;
  if abs(test_sv) > 1e-5
    save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', ...
      sprintf('%s_Rob%d_%s_cds_dimsynth_robot_svreprowarning.mat', Set.optimization.optname, ...
      Structure.Number, Structure.Name)));
    cds_log(-1, sprintf('[dimsynth] Während Optimierung gespeicherte Materialbelastung (%1.5e) stimmt nicht mit erneuter Berechnung (%1.5e) überein. Differenz %1.5e.', ...
      PSO_Detail_Data.f_maxstrengthviol(dd_optgen, dd_optind), f_maxstrengthviol, test_sv));
  end
else
  % Keine Berechnung der Zielfunktionen möglich, da keine zulässige Lösung
  % gefunden wurde.
  fval_obj_all = NaN(6,1);
  fval_constr_all = NaN(1,1);
  physval_obj_all = NaN(6,1);
end
I_fobj_set = Set.optimization.constraint_obj ~= 0;
if any( physval_obj_all(I_fobj_set) > Set.optimization.constraint_obj(I_fobj_set) )
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', ...
    sprintf('%s_Rob%d_%s_cds_dimsynth_robot_objconstrwarning.mat', Set.optimization.optname, ...
    Structure.Number, Structure.Name)));
  cds_log(1, sprintf('[dimsynth] Zielfunktions-Nebenbedingung verletzt trotz Berücksichtigung in Optimierung. Keine Lösung gefunden.'));
end
if Set.optimization.constraint_link_yieldstrength~= 0 && ...
    fval_constr_all(1) > Set.optimization.constraint_link_yieldstrength
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', ...
    sprintf('%s_Rob%d_%s_cds_dimsynth_robot_strengthconstrwarning.mat', Set.optimization.optname, ...
    Structure.Number, Structure.Name)));
  cds_log(1, sprintf('[dimsynth] Materialbelastungs-Nebenbedingung verletzt trotz Berücksichtigung in Optimierung. Keine Lösung gefunden.'));
end
%% Ausgabe der Ergebnisse
t_end = now(); % End-Zeitstempel der Optimierung dieses Roboters
RobotOptRes = struct( ...
  'fval', fval, ... % Zielfunktionswert (nach dem optimiert wurde)
  'fval_obj_all', fval_obj_all, ... % Werte aller möglicher einzelner Zielf.
  'physval_obj_all', physval_obj_all, ... % Physikalische Werte aller Zielf.
  'fval_constr_all', fval_constr_all, ... % Werte der Nebenbedingungen
  'R', R, ... % Roboter-Klasse
  'p_val', p_val, ... % Parametervektor der Optimierung
  'p_limits', varlim, ... % Grenzen für die Parameterwerte
  'options', options, ... % Optionen des Optimierungs-Algorithmus
  'q0', q, ... % Anfangs-Gelenkwinkel für Lösung der IK
  'Traj_Q', Q, ...
  'Traj_QD', QD, ...
  'Traj_QDD', QDD, ...
  'Traj_PHI', PHI, ...
  'timestamps_start_end', [t_start, t_end, toc(t1)], ...
  'exitflag', exitflag, ...
  'Structure', Structure, ...
  'fitnessfcn', fitnessfcn);
% Debug: Durch laden dieser Ergebnisse kann nach Abbruch des PSO das
% Ergebnis trotzdem geladen werden
if Set.general.matfile_verbosity > 0
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_robot4.mat'));
end
% Gesamtergebnis der Optimierung speichern
save(fullfile(Set.optimization.resdir, Set.optimization.optname, ...
  sprintf('Rob%d_%s_Endergebnis.mat', Structure.Number, Structure.Name)), ...
  'RobotOptRes', 'Set', 'Traj', 'PSO_Detail_Data');
cds_log(1,sprintf('[dimsynth] Optimierung von Rob. %d (%s) abgeschlossen. Dauer: %1.1fs', ...
  Structure.Number, Structure.Name, toc(t1)));
