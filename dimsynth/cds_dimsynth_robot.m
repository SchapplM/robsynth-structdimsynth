% Maßsynthese für eine Roboterstruktur
% 
% Eingabe:
% Set
%   Einstellungen des Optimierungsalgorithmus
% Traj
%   Trajektorie (bezogen auf Welt-KS)
% Structure
%   Eigenschaften der Roboterstruktur
% init_only
%   Nur Roboter-Klasse initialisieren. Keine Optimierung durchführen.

% Quellen:
% [SchapplerTapOrt2019] Schappler, M. and Tappe, S., Ortmaier, T.:
% Exploiting Dynamics Parameter Linearity for Design Optimization in
% Combined Structural and Dimensional Robot Synthesis (2019)
% [SierraCoe2005] Improving PSO-based multi-objective optimization 
% using crowding, mutation and ϵ-dominance (2005)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [R, Structure] = cds_dimsynth_robot(Set, Traj, Structure, init_only)
if nargin < 4
  init_only = false;
end
t1 = tic();
t_start = now(); % Anfangs-Zeitstempel der Optimierung dieses Roboters
%% Debug: 
if Set.general.matfile_verbosity > 0
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_robot1.mat'));
end
% Zum Debuggen:
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_robot1.mat'));

% Prüfe, ob die Optimierung bereits erfolgreich war
if ~Set.general.overwrite_existing_results
  if exist(fullfile(Set.optimization.resdir, Set.optimization.optname, ...
    sprintf('Rob%d_%s_Endergebnis.mat', Structure.Number, Structure.Name)), 'file')
    fprintf('[dimsynth] Ergebnis für Rob %d (%s) liegt bereits vor. Abbruch.\n', ...
      Structure.Number, Structure.Name);
    return
  end
end
%% Initialisierung
% Log-Datei initialisieren
if ~init_only && ~Set.general.only_finish_aborted
  resdir_rob = fullfile(Set.optimization.resdir, Set.optimization.optname, ...
    sprintf('Rob%d_%s', Structure.Number, Structure.Name));
  mkdirs(resdir_rob); % Ergebnis-Ordner für diesen Roboter erstellen
  fpfile = cds_log(1, sprintf('[dimsynth] Start der Maßsynthese für %s',  Structure.Name), 'init', Set, Structure);
  % Fingerabdruck der relevanten Repos in Log-Datei speichern
  repo_deps = {
    {'structgeomsynth', 'structgeomsynth_path_init.m'}, ...
    {'parroblib', 'parroblib_path_init.m'}, ...
    {'serroblib', 'serroblib_path_init.m'}, ...
    {'robotics', 'robotics_toolbox_path_init.m'}, ...
    {'hybriddyn', 'hybrdyn_path_init.m'}, ...
    {'matlab-tools', 'matlab_tools_path_init.m'}, ...
    {'matlab-ext', 'matlab_ext_path_init.m'}, ...
    {'trajectory-toolbox', 'trajectory_toolbox_path_init.m'}, ...
    {'geometry-toolbox', 'geometry_toolbox_path_init.m'}};
  olddir = pwd();
  fid = fopen(fpfile, 'a');
  fprintf(fid, 'Fingerabdruck aller Abhängigkeiten:\n');
  for i = 1:length(repo_deps)
    cd(fileparts(which(repo_deps{i}{2})));
    [~,rev]=system('git rev-parse HEAD');
    if ispc() % Windows
      [~,revdatum]=system('git log -1 --date=short --pretty=format:%cd');
    else % Linux
      [~,revdatum]=system('export TERM=ansi; git log -1 --date=short --pretty=format:%cd');
      revdatum = revdatum(2:11); % Entferne Zeilenumbruch und nicht lesbare Zeichen
    end
    [~,branch]=system('git rev-parse --abbrev-ref HEAD');
    fprintf(fid, '%s: Branch %s, Rev. %s (%s)\n', repo_deps{i}{1}, branch(1:end-1), rev(1:8), revdatum);
  end
  fclose(fid);
  if ~ispc() % lspcu funktioniert nur unter Linux
    system(sprintf('echo "Eigenschaften des Rechners (lspcu):" >> %s', fpfile));
    system(sprintf('lscpu >> %s', fpfile));
  end
  cd(olddir);
end
% Zurücksetzen der Detail-Speicherfunktion
clear cds_save_particle_details;

%% Referenzlänge ermitteln
% Mittelpunkt der Aufgabe
Structure.xT_mean = mean(minmax2(Traj.X(:,1:3)'), 2);
% Charakteristische Länge der Aufgabe (empirisch ermittelt aus der Größe
% des notwendigen Arbeitsraums)
Lref = norm(diff(minmax2(Traj.X(:,1:3)')'));
% Abstand der Aufgabe vom Roboter-Basis-KS
if any(~isnan(Set.optimization.basepos_limits(:)))
  maxdist_xyz = NaN(3,1);
  for i = 1:3
    mmt = minmax2(Traj.X(:,i)');
    mmb = Set.optimization.basepos_limits(i,:);
    distmat = reshape(mmt,2,1)-reshape(mmb',1,2);
    maxdist_xyz(i) = max(abs(distmat(:)));
  end
  % Vergrößere den Wert, damit bei weit entfernter Basis der Roboter lang
  % genug werden kann.
  Lref = max(Lref, max(maxdist_xyz));
end
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
  % Bei paralleler Rechnung der Struktursynthese auf Cluster Konflikte vermeiden
  parroblib_writelock('check', 'csv', logical(Set.structures.DoF), 5*60, false);
  % Klasse initialisierung (liest auch die csv-Dateien aus).
  R = parroblib_create_robot_class(Structure.Name, p_base(:), p_platform(:));
  NLEG = R.NLEG;
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
  % Gelenkgrenzen setzen: Schubgelenke (Verfahrlänge nicht mehr als "fünf
  % mal schräg durch Arbeitsraum" (char. Länge))
  % Muss so hoch gesetzt sein, damit UPS-Kette (ohne sonstige
  % Kinematikparameter auch funktioniert)
  R_init.qlim(R_init.MDH.sigma==1,:) = repmat([-5*Lref, 5*Lref],sum(R_init.MDH.sigma==1),1);
  % Gelenkgrenzen setzen: Drehgelenke
  if Structure.Type == 0 % Serieller Roboter
    % Grenzen für Drehgelenke: Alle sind aktiv
    R_init.qlim(R.MDH.sigma==0,:) = repmat([-0.5, 0.5]*... % Drehgelenk
      Set.optimization.max_range_active_revolute, sum(R.MDH.sigma==0),1);
    Structure.qlim = R_init.qlim;
  else % Paralleler Roboter
    % Grenzen für passive Drehgelenke (aktive erstmal mit setzen)
    R_init.qlim(R_init.MDH.sigma==0,:) = repmat([-0.5, 0.5]*... % Drehgelenk
      Set.optimization.max_range_passive_revolute, sum(R_init.MDH.sigma==0),1);
    % Grenzen für technische Gelenke gesondert setzen
    R_init.qlim(R_init.DesPar.joint_type==2,:) = repmat([-0.5, 0.5]*... % Kardan-Gelenk
      Set.optimization.max_range_passive_universal, sum(R_init.DesPar.joint_type==2),1);
    R_init.qlim(R_init.DesPar.joint_type==3,:) = repmat([-0.5, 0.5]*... % Kugelgelenk
      Set.optimization.max_range_passive_spherical, sum(R_init.DesPar.joint_type==3),1);
    % Grenzen für aktives Drehgelenk setzen
    I_actrevol = R_init.MDH.mu == 2 & R_init.MDH.sigma==0;
    R_init.qlim(I_actrevol,:) = repmat([-0.5, 0.5]*Set.optimization.max_range_active_revolute, sum(I_actrevol),1);
    if i == NLEG % Grenzen aller Gelenke aller Beinketten eintragen
      Structure.qlim = cat(1, R.Leg.qlim);
    end
  end
  % Grenzen für Gelenkgeschwindigkeiten setzen
  R_init.qDlim = repmat([-1,1]*Set.optimization.max_velocity_active_revolute, R_init.NJ, 1);
  R_init.qDlim(R_init.MDH.sigma==1,:) = repmat([-1,1]*... % Schubgelenk
    Set.optimization.max_velocity_active_prismatic, sum(R_init.MDH.sigma==1), 1);
  if Structure.Type == 2 % Paralleler Roboter
    I_passrevolute = R_init.MDH.mu == 1 & R_init.MDH.sigma==0;
    I_passuniversal = R_init.MDH.mu == 1 & R_init.DesPar.joint_type==2;
    I_passspherical = R_init.MDH.mu == 1 & R_init.DesPar.joint_type==3;
    R_init.qDlim(I_passrevolute,:) = repmat([-1,1]*... % Drehgelenk
      Set.optimization.max_velocity_passive_revolute,sum(I_passrevolute),1);
    R_init.qDlim(I_passuniversal,:) = repmat([-1,1]*... % Kardan-Gelenk
      Set.optimization.max_velocity_passive_universal,sum(I_passuniversal),1);
    R_init.qDlim(I_passspherical,:) = repmat([-1,1]*... % Kugelgelenk
      Set.optimization.max_velocity_passive_spherical,sum(I_passspherical),1);
  end
  % Grenze für Gelenkbeschleunigung setzen (keine Betrachtung Kardan/Kugel)
  R_init.qDDlim = repmat([-1,1]*Set.optimization.max_acceleration_revolute, R_init.NJ, 1);
  R_init.qDDlim(R_init.MDH.sigma==1,:) = repmat([-1,1]*... % Schubgelenk
    Set.optimization.max_acceleration_prismatic, sum(R_init.MDH.sigma==1), 1);
  
  if Structure.Type == 0 % Serieller Roboter
    Structure.qDlim = R_init.qDlim;
    Structure.qDDlim = R_init.qDDlim;
  else % Paralleler Roboter
    if i == NLEG % Grenzen aller Gelenke aller Beinketten eintragen
      Structure.qDlim = cat(1, R.Leg.qDlim);
      Structure.qDDlim = cat(1, R.Leg.qDDlim);
    end
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
% Richte den Roboter entsprechend der Montagekonfiguration aus
if Structure.Type == 0 % Seriell
  mounting = Set.structures.mounting_serial;
else % PKM
  mounting = Set.structures.mounting_parallel;
end
if ~Set.optimization.rotate_base
  if strcmp(mounting, 'floor')
    % Nichts ändern. Basis-KS zeigt (mit z-Achse) nach oben
    R.update_base([], zeros(3,1));
  elseif strcmp(mounting, 'ceiling')
    % Roboter zeigt nach unten. x-Achse bleibt gleich
    R.update_base([], [pi;0;0]); % xyz-Euler-Winkel
    R.update_gravity([0;0;-9.81]); % Gravitation wird für Dynamik im Basis-KS definiert.
    % Drehe End-Effektor auch um. Die Aufgaben sind so definiert, dass die
    % z-Achse standardmäßig (im Welt-KS) nach oben zeigt. Sonst ist bei
    % 2T1R, 3T0R und 3T1R die IK nicht lösbar.
    % Vorher sind die KS schon so definiert, dass die z-Achse im Basis-KS
    % nach oben zeigt. Jetzt zeigt sie im Basis-KS nach unten.
    Structure.R_N_E = Structure.R_N_E*rotx(pi); % wird damit in Opt. der EE-Rotation berücksichtigt
    R.update_EE([], r2eulxyz(Structure.R_N_E));
  else
    error('Fall %s noch nicht implementiert', mounting);
  end
else
  error('Rotation der Basis ist noch nicht implementiert');
end
if any(any(abs(Structure.R_N_E-eye(3)) > 1e-10))
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

% Gelenk-Steifigkeit einsetzen (Sonderfall für Starrkörpergelenke)
if R.Type ~= 0 && Set.optimization.joint_stiffness_passive_revolute
  for k = 1:R.NLEG
    % Ruhelage der Feder muss erst später eingestellt werden (nach IK)
    % Federsteifigkeit auf vorgegebenen Wert setzen.
    R.Leg(k).DesPar.joint_stiffness(R.Leg(k).MDH.sigma==0) = ...
      Set.optimization.joint_stiffness_passive_revolute;
  end
end
%% Umfang der Berechnungen prüfen: Schnittkraft / Regressorform / Dynamik
% Schalter zum Berechnen der inversen Dynamik bezogen auf Antriebe
calc_dyn_act = false;
% Schalter zur Berechnung der Antriebskräfte für Gelenkelastizitäten
calc_spring_act = false;
% Schalter zum Berechnen der vollständigen Schnittkräfte. Die Zusammensetzung
% (Dynamik/Federkraft, direkt oder Regressor) wird passend gewählt.
calc_cut = false;
% Schalter zur Berechnung der Regressorform der Dynamik; [SchapplerTapOrt2019]
calc_dyn_reg = false;
% Schalter zur Berechnung der Regressorform für Gelenkelastizität
calc_spring_reg = false;

if ~isempty(intersect(Set.optimization.objective, {'energy', 'actforce'}))
  calc_dyn_act = true; % Antriebskraft für Zielfunktion benötigt
  if Set.optimization.joint_stiffness_passive_revolute
    calc_spring_act = true;
  end
end
if any(Set.optimization.constraint_obj(2:3)) % Energie oder Antriebskraft
  calc_dyn_act = true; % Antriebskraft für Nebenbedingung benötigt
  if Set.optimization.joint_stiffness_passive_revolute
    calc_spring_act = true;
  end
end
if any(strcmp(Set.optimization.desopt_vars, 'linkstrength'))
  calc_dyn_reg = true; % Entwurfsoptimierung schneller mit Regressor
end
if any(strcmp(Set.optimization.desopt_vars, 'joint_stiffness_qref'))
  calc_spring_reg = true; % Entwurfsoptimierung schneller mit Regressor
end
if Set.optimization.constraint_obj(6) > 0 || ... % Schnittkraft als Nebenbedingung ...
    any(strcmp(Set.optimization.objective, {'materialstress'})) % ... oder Zielfunktion
  calc_cut = true;
end
if Structure.Type == 2 && calc_cut
  calc_dyn_act = true;
  if Set.optimization.joint_stiffness_passive_revolute
    calc_spring_act = true;
  end
end
Structure.calc_dyn_act = calc_dyn_act;
Structure.calc_dyn_reg = calc_dyn_reg;
Structure.calc_cut = calc_cut;
Structure.calc_spring_reg = calc_spring_reg;
Structure.calc_spring_act = calc_spring_act;
%% Art der Dynamikparameter in der Roboter-Klasse einstellen
if Structure.Type == 2 % Parallel
  if calc_cut % Benutze Inertialparameter-Dynamik, weil auch Schnitt- ...
    R.DynPar.mode = 3; % ... kräfte in Regressorform berechnet werden
  else % Benutze Minimalparameter-Dynamikfunktionen für die PKM
    R.DynPar.mode = 4;
  end
end
for i = 1:NLEG % Das Gleiche für die seriellen Beinketten ...
  if Structure.Type == 0
    R_init = R; % ... oder den seriellen Roboter
  else
    R_init = R.Leg(i);
  end
  % Dynamikparameter setzen
  if calc_cut
    R_init.DynPar.mode = 3;
  else
    R_init.DynPar.mode = 4;
  end
end
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
  % Nummern zur Indizierung der pkin, siehe SerRob/get_pkin_parameter_type
  Ipkinrel = R_pkin.get_relevant_pkin(Set.structures.DoF);
  % Setzen den theta-Parameter für PKM-Beinketten auf einen konstant Wert,
  % falls das durch die Struktursynthese vorgegeben ist (z.B. auf 0).
  % Bei 3T0R- und 3T1R-PKM ist die Parallelität der Gelenke in den Beinketten
  % besonders wichtig. Bei 3T3R darf es eigentlich keinen Einfluss haben.
  if Structure.Type == 2 && ~isempty(Structure.angles_values)
    % Sortiere pkin entsprechend der Reihenfolge der freien Parameter:
    % Gelenkweise, erst alpha, dann theta
    [~,pkin_jointnumber] = R_pkin.get_pkin_parameter_type();
    sortstr = cell(length(R_pkin.pkin_names),1);
    for k = 1:length(sortstr)
      sortstr{k} = sprintf('%d%s', pkin_jointnumber(k),R_pkin.pkin_names{k});
    end
    [~,III] = sort(sortstr); % Indizes zum Sortieren
    I_alphatheta = R_pkin.pkin_types==3 | R_pkin.pkin_types==5;
    II_alphatheta = find(I_alphatheta);
    if length(II_alphatheta) ~= length(Structure.angles_values)
      error(['Anzahl der freien alpha-/theta-Parameter der Beinkette stimmt ', ...
        'nicht (%d in Beinkette, %d in Datenbank: "%s")'], length(II_alphatheta), ...
        length(Structure.angles_values), Structure.angles_values)
    end
    iik = 0; % Zähler für Variable Structure.angles_values
    for kk = III' % Gehe alle Kinematikparameter durch (in veränderter REihenfolge)
      if ~I_alphatheta(kk)
        continue % kein freier alpha-/theta-Parameter
      end
      iik = iik + 1; % Index in den gespeicherten Konfigurationen für alpha und theta in der ParRobLib
      if ~strcmp(Structure.angles_values(iik), 'a') % "a"="arbitrary"
        % Winkel alpha/theta ist fest auf 0 oder pi/2 gestellt.
        Ipkinrel(II_alphatheta(iik)) = false; % Nehme die "1" bei einstellbarem alpha/theta weg.
      end
    end
    if iik ~= length(Structure.angles_values)
      error('Nicht alle Elemente von Structure.angles_values wurden ausgelesen.');
    end
  end
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
  % Nicht relevanten alpha- oder theta-Parameter auf 0 oder pi/2 setzen.
  if Structure.Type == 2 && ~isempty(Structure.angles_values)
    iik = 0; % Zähler für Variable Structure.angles_values
    for kk = III' % Gehe alle Kinematikparameter durch (in veränderter REihenfolge)
      if ~I_alphatheta(kk)
        continue % kein freier alpha-/theta-Parameter
      end
      iik = iik + 1;
      if strcmp(Structure.angles_values(iik), 'p') % nur Wert 0 ist zulässig
        pkin_init(II_alphatheta(iik)) = 0;
        % Winkel alpha/theta ist fest auf 0 oder pi/2 gestellt.
      elseif strcmp(Structure.angles_values(iik), 'o') % nur Wert +/- 90 ist zulässig
        pkin_init(II_alphatheta(iik)) = pi/2;
      elseif strcmp(Structure.angles_values(iik), 'b') % nur Wert 0 oder 90 ist zulässig
        cds_log(-1, sprintf(['[dimsynth] Winkel %s als 0 und 90 zulässig ', ...
          'gegeben. Wähle eine Alternative (0). Das sollte eigentlich nicht mehr vorkommen.'], R_pkin.pkin_names{kk}));
        pkin_init(II_alphatheta(iik)) = 0; % Nehme die 0
      else % Entweder 0 (nicht definiert) oder 4 (alles erlaubt)
        % Mache gar nichts. Parameter wird ganz normal optimiert.
      end
    end
  end
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
      % Parallelität der Gelenke eingestellt.
      plim(i,:) = [0, pi/2];
      % Sonderfall Struktursynthese: Die Fälle 0° und 90° sollen ausge- 
      % schlossen werden, da diese eine strukturelle Eigenschaft sind.
      % Dafür werden theta-Parameter separat optimiert.
      if any(strcmp(Set.optimization.objective, 'valid_act'))
        plim(i,:) = [5, 85]*pi/180; % 5° Abstand von den rechten Winkeln
      end
    elseif R_pkin.pkin_types(i) == 5
      % Winkel-Parameter theta. Nur Begrenzung auf [-pi/2,pi/2].
      % Durch Möglichkeit negativer DH-Längen ist jede beliebige
      % Ausrichtung des folgenden Gelenks möglich.
      plim(i,:) = [-pi/2, pi/2];
      % Sonderfall Struktursynthese: Siehe oben bei alpha
      if any(strcmp(Set.optimization.objective, 'valid_act'))
        plim(i,:) = [5, 85]*pi/180; % 5° Abstand von den rechten Winkeln
      end
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
    varlim = [varlim; repmat([-1, 1], sum(I_DoF_basepos(1:2)), 1)];
    % TODO: Durchdachte Behandlung des Montageortes. Prinzipiell kann ein
    % am Boden befestigter Roboter auch auf eine Säule gestellt werden und
    % die Basis kann dann oberhalb der Basis sein. Aktuell keine
    % Einschränkung.
    varlim = [varlim; repmat([-1, 1], sum(I_DoF_basepos(3)), 1)];
  else % Parallel
    % Stelle den parallelen Roboter in/über die Aufgabe
    % Bei Parallelen Robotern ist der Arbeitsraum typischerweise in der
    % Mitte des Gestells (bezogen auf x-y-Ebene). Daher müssen die Grenzen
    % nicht so weit definiert werden: 20% der Referenzlänge um Mittelpunkt
    % der Aufgabe. Annahme: x-/y-Komponente werden immer optimiert.
    varlim = [varlim; repmat([-0.2, 0.2],sum(I_DoF_basepos(1:2)),1)]; % xy-Komponenten
    % Die z-Komponente der Basis kann mehr variieren. Einstellung je nach-
    % dem, ob unter- oder oberhalb der Aufgabe.
    % Siehe cds_update_robot_parameters.m
    if strcmp(mounting, 'floor')
      % Roboterbasis ist unterhalb der Aufgabe
      varlim = [varlim; repmat([-1, 0], sum(I_DoF_basepos(3)), 1)];
    elseif strcmp(mounting, 'ceiling')
      varlim = [varlim; repmat([0, 1], sum(I_DoF_basepos(3)), 1)];
    else
      error('Noch nicht implementiert');
    end
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
    if Set.structures.DoF(3) == 1 % nicht für 2T1R
      if strcmp(mounting, 'floor')
        r_W_0(3) = -0.7*Lref; % Setze Roboter-Basis etwas unter die Aufgabe
      elseif strcmp(mounting, 'ceiling')
        r_W_0(3) =  0.7*Lref;
      else
        error('Fall nicht definiert');
      end
    end
  else % Parallel
    r_W_0 = zeros(3,1);
    if Set.structures.DoF(3) == 1 % nicht für 2T1R
      if strcmp(mounting, 'floor')
        % Setze Roboter mittig unter die Aufgabe
        r_W_0(3) = Structure.xT_mean(3)-0.7*Lref;
      elseif strcmp(mounting, 'ceiling')
        r_W_0(3) = Structure.xT_mean(3)+0.7*Lref;
      else
        error('Fall nicht definiert');
      end
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
  if any(R.DesPar.platform_method == [1:3, 8]) % keine Parameter bei Kreis
  elseif any(R.DesPar.platform_method == 4:6) % Parameter ist Gelenkpaarabstand (6FG-PKM)
    nvars = nvars + 1;
    vartypes = [vartypes; 9];
    varlim = [varlim; [0.2,0.8]]; % Gelenkpaarabstand. Relativ zu Plattform-Radius.
    varnames = {varnames{:}, 'platform_morph'}; %#ok<CCAT>
  else
    error('Parameter "platform_morphology" für Platform-Methode %d nicht implementiert', R.DesPar.platform_method);
  end
end
% Variablen-Typen speichern
Structure.vartypes = vartypes;
Structure.varnames = varnames;
Structure.varlim = varlim;
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
% Offsetparameter für Schubgelenke (Verschiebung der Führungsschiene)
Structure.desopt_prismaticoffset = false;
%% Initialisierung der Kollisionsprüfung
if Set.optimization.constraint_collisions || ~isempty(Set.task.obstacles.type) || ...
    ~isempty(Set.task.installspace.type) || ...
    ~isnan(Set.optimization.base_size_limits(2)) && any(Structure.I_firstprismatic)
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
        % Falls eine Führungsschiene existiert, muss immer der Offset- 
        % Parameter zum nachfolgenden Segment optimiert werden, da die
        % Schiene als Kollisionskörper mit den nachfolgenden Segmenten
        % kollidieren kann. Durch den Offset wird dies vermieden.
        Structure.desopt_prismaticoffset = true;
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
        cds_log(-1, sprintf(['[dimsynth] Die Kollisionsprüfung für die Führungsschiene des P-Gelenks ', ...
          'an Stelle %d ist nicht definiert. Wird vorerst ignoriert.'], i));
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
        if length(j_hascollbody) <= 1
          % Es gibt keinen oder nur einen direkten Vorgänger-Kollisionskörper
          cbdist = 1; % dieser wird hiermit nicht genommen
        elseif j_hascollbody(end-1) == 0
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
  
  % Roboter-Kollisionsobjekte in Struktur abspeichern (zum Abruf in den
  % Funktionen cds_constr_collisions_... und cds_constr_installspace
  % Ist erstmal nur Platzhalter. Wird zur Laufzeit noch aktualisiert.
  Structure.collbodies_robot = cds_update_collbodies(R, Set, Structure, Structure.qlim');
  % Probe: Sind Daten konsistent? Inkonsistenz durch obigem Aufruf möglich.
  if any(any(~isnan(Structure.collbodies_robot.params(Structure.collbodies_robot.type==6,2:end))))
    error('Inkonsistente Kollisionsdaten: Kapsel-Direktverbindung hat zu viele Parameter');
  end
  
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
    % Kollisionsprüfung der Körper der Beinketten mit dem Gestell
    for k = 1:NLEG
      if k > 1, NLoffset_k = 1+R.I2L_LEG(k-1)-(k-1);
      else,     NLoffset_k = 1; end
      % Prüfe keine Kollision des ersten Kollisionskörpers, da dieser
      % Körper direkt nach dem Gestell kommt. Annahme. Kollision
      % konstruktiv vermeidbar
      for cb_k = R.Leg(k).collbodies.link(2:end)'
        % Kollision mit Kollisionskörpern fest bezüglich PKM-Basis
        selfcollchecks_bodies = [selfcollchecks_bodies; ...
          uint8([NLoffset_k+cb_k, 0])]; %#ok<AGROW>
        % fprintf(['Kollisionsprüfung (%d): Bein %d Seg. %d vs Gestell. ', ...
        %   'Zeile [%d,%d]\n'], size(selfcollchecks_bodies,1), k, cb_k, ...
        %   selfcollchecks_bodies(end,1), selfcollchecks_bodies(end,2));
        % TODO: Prüfe Kollision mit Gestellfesten Kollisionskörpern, die dem
        % Basis-KS von anderen Beinketten zugeordnet sind.
        % Wird aktuell nicht geprüft (unsicher, ob sinnvoll).
      end
    end
    % Kollision Beinketten mit Plattform und Gestell
    for k = 1:NLEG
      for j = 1:NLEG
        k_plf = R.I2L_LEG(k)-(k-1)-1;
        j_base = R.I1L_LEG(j)-(j-1);
        selfcollchecks_bodies = [selfcollchecks_bodies; ...
          uint8([k_plf, j_base])]; %#ok<AGROW>
        % fprintf(['Kollisionsprüfung (%d): Plattform-Körper zugeordnet zu ', ...
        %   'Bein %d vs Gestell-Körper zugeordnet zu Bein %d. ', ...
        %   'Zeile [%d,%d]\n'], size(selfcollchecks_bodies,1), k, j, ...
        %   selfcollchecks_bodies(end,1), selfcollchecks_bodies(end,2));
      end
    end
  end
  
  % Debug: Namen der Körper für die Kollisionsprüfung anzeigen
  % (unterscheidet sich von den Kollisionskörpern, den Körpern zugeordnet)
  if false
    if Structure.Type == 0   %#ok<UNRCH>
      names_collbodies = cell(R.NL,1);
      names_collbodies{1} = 'Base';
      for k = 2:R.NL
        names_collbodies{k} = sprintf('Link %d', k-1);
      end
    else % PKM
      names_collbodies = cell(R.I2L_LEG(end)-R.NLEG,1);
      names_collbodies{1} = 'Base';
      i = 1;
      for k = 1:NLEG
        i = i + 1;
        names_collbodies{i} = sprintf('Leg %d Base', k);
        for j = 1:R.Leg(1).NL-1
          i = i + 1;
          names_collbodies{i} = sprintf('Leg %d Link %d', k, j);
        end
      end
    end
    fprintf('Liste der Körper:\n');
    for i = 1:length(names_collbodies)
      fprintf('%d - %s\n', i-1, names_collbodies{i});
    end
    % Debug: Liste der Kollisionskörper anzeigen
    fprintf('Liste der Kollisionskörper:\n');
    for i = 1:size(Structure.collbodies_robot.link,1)
      fprintf('%d - links %d+%d (%s + %s)\n', i, Structure.collbodies_robot.link(i,1), ...
        Structure.collbodies_robot.link(i,2), names_collbodies{1+Structure.collbodies_robot.link(i,1)}, ...
        names_collbodies{1+Structure.collbodies_robot.link(i,2)});
    end
  end
  % Prüfe die zusammengestellten Kollisionskörper. Die höchste Nummer
  % (Null-indiziert) ist durch die Roboterstruktur vorgegeben.
  if Structure.Type == 0
    Nmax = R.NL-1;
  else
    Nmax = R.I2L_LEG(end)-(R.NLEG-1)-1;
  end
  if any(Structure.collbodies_robot.link(:) > Nmax)
    error(['Ungültige Kollisionskörper. Maximaler Index %d in Structure.', ...
      'collbodies_robot.link überschritten'], Nmax);
  end
  
  if isempty(selfcollchecks_bodies)
    cds_log(-1, sprintf(['[dimsynth] Es sind keine Kollisionskörpern eingetragen, ', ...
      'obwohl Kollisionen geprüft werden sollen.']));
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
    I1 = selfcollchecks_bodies(i,1) == Structure.collbodies_robot.link(:,1);
    I2 = selfcollchecks_bodies(i,2) == Structure.collbodies_robot.link(:,1);
    CheckCombinations = NaN(sum(I1)*sum(I2),2);
    if sum(I1) == 0 || sum(I2) == 0
      % Die Kollision der Starrkörper soll geprüft werden, es sind aber gar
      % keine Ersatzkörper definiert.
      continue
    end
    kk = 0;
    for ii1 = find(I1)'
      for ii2 = find(I2)'
        % Prüfe, ob diese Kombination sinnvoll zu prüfen ist. Keine Prüfung
        % von Gestellteilen gegen die Beinketten.
        % TODO: Weitere Differenzierung sinnvoll, aber nicht akut notwendig.
        if Structure.Type == 2
          % Indizes der Körper, die zu Gestell (bzw. Beinketten-Basis-KS)
          % oder Plattform (zugehörig zu letztem Beinketten-KS) gehören
          I_base = R.I1L_LEG(1:NLEG)-((1:NLEG)'-1);
          I_pl = R.I2L_LEG(1:NLEG)-((1:NLEG)'-1)-1;
          % Finde heraus, ob einer der Kollisionskörper ein Gestellteil ist.
          % Basis-Körper haben entweder Zugehörigkeit zu zwei Beinketten-
          % Basis-KS (in I_base) oder einen Eintrag mit 0 (sternförmit für
          % PKM-Basis).
          % (siehe cds_update_collbodies zur Definition der Gestellteile)
          c1_is_base = false;
          if ( length(intersect(Structure.collbodies_robot.link(ii1,:)', I_base)) == 2 || ... % beide Beinketten-Basis zugeordnet (Kreis)
              length(intersect(Structure.collbodies_robot.link(ii1,:)', I_base)) == 1 && ... % einer zur PKM-Basis, einer zur Beinketten-Basis (Stern)
              any(Structure.collbodies_robot.link(ii1,:)==0) ) && ...
              Structure.collbodies_robot.link(ii1,1) ~= Structure.collbodies_robot.link(ii1,2)
            c1_is_base = true;
          end
          c2_is_base = false;
          if ( length(intersect(Structure.collbodies_robot.link(ii2,:)', I_base)) == 2 || ...
              length(intersect(Structure.collbodies_robot.link(ii2,:)', I_base)) == 1 && ...
              any(Structure.collbodies_robot.link(ii2,:)==0) ) && ...
              Structure.collbodies_robot.link(ii2,1) ~= Structure.collbodies_robot.link(ii2,2)
            c2_is_base = true;
          end
          % Finde heraus, ob einer der Kollisionskörper ein Plattformteil ist
          % (siehe cds_update_collbodies zur Definition der Plattformteile)
          c1_is_platform = false;
          if length(intersect(Structure.collbodies_robot.link(ii1,:)', I_pl)) == 2 && ...
              Structure.collbodies_robot.link(ii1,1) ~= Structure.collbodies_robot.link(ii1,2)
            c1_is_platform = true;
          end
          c2_is_platform = false;
          if length(intersect(Structure.collbodies_robot.link(ii2,:)', I_pl)) == 2 && ...
              Structure.collbodies_robot.link(ii2,1) ~= Structure.collbodies_robot.link(ii2,2)
            c2_is_platform = true;
          end
          if c1_is_platform && c2_is_platform
            % Beide Körper sind Teil der Plattform, aber zwecks Implemen-
            % tierung verschiedenen Beinketten zugeordnet. Keine Kollisions-
            % prüfung notwendig.
            continue
          elseif c1_is_base && ~c2_is_platform || c2_is_base && ~c1_is_platform
            % Ein Körper ist Teil des Gestells, der andere aber nicht Teil
            % der Plattform. Überspringe die Prüfung. Hiermit vermiedene
            % Fälle:
            % * Gestell gegen Gestell: Kann keine Kollision sein, wird aber
            %   als solche erkannt (z.B. da die Kapseln sich überlappen)
            % * Beinkette gegen Gestell: Eine Kapsel verbindet zwei Gestell-
            %   Koppelgelenke. Daher dabei immer Überlappung. Weitere
            %   Prüfung wäre notwendig (z.B. ob Kollisionssegment nicht das
            %   erste ist.
            % * Gestell-Teil, dass der Basis-Zugeordnet ist (z.B. Führungs-
            %   schiene einer Linearachse. Hier auch keine Prüfung gegen
            %   kreisförmig modelliertes Gestell.
            continue
          elseif c1_is_platform && ~c2_is_base || c2_is_platform && ~c1_is_base
            % Ein Körper ist Teil der Plattform, der andere aber nicht Teil
            % des Gestells. Der hier vorliegende Fall Beinkette+Plattform
            % wird vorerst nicht geprüft.
            continue
          end
        end
        kk = kk + 1;
        CheckCombinations(kk,:) = [ii1,ii2];
        % fprintf('Kollisionsprüfung (%d): Koll.-körper %d (Seg. %d) vs Koll.-körper %d (Seg. %d).\n', ...
        %   i, ii1, Structure.collbodies_robot.link(ii1,1), ii2, Structure.collbodies_robot.link(ii2,1));
      end
    end
    CheckCombinations = CheckCombinations(1:kk,:);
    if any(CheckCombinations(:,1)==CheckCombinations(:,2))
      error('CheckCombinations: Prüfung eines Körpers mit sich selbst ergibt keinen Sinn');
    end
    % Eintragen in Gesamt-Liste
    Structure.selfcollchecks_collbodies = ...
      uint8([Structure.selfcollchecks_collbodies; CheckCombinations]);
  end
  % Debug: Liste der Kollisionsprüfungen anzeigen
  if false
    fprintf('Liste der Kollisionsprüfungen (der Kollisionskörper):\n');   %#ok<UNRCH>
    for i = 1:size(Structure.selfcollchecks_collbodies,1)
      fprintf('%d - collbodies %d vs %d (links %d vs %d)\n', i, ...
        Structure.selfcollchecks_collbodies(i,1), Structure.selfcollchecks_collbodies(i,2), ...
        Structure.collbodies_robot.link(Structure.selfcollchecks_collbodies(i,1),1), ...
        Structure.collbodies_robot.link(Structure.selfcollchecks_collbodies(i,2),1));
    end
  end
  if isempty(Structure.selfcollchecks_collbodies)
    cds_log(-1, sprintf(['[dimsynth] Es sind keine Prüfungen von Kollisions', ...
      'körpern vorgesehen']));
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
      links_i = repmat(NLoffset+uint8(i-1),2,1);
      collbodies_instspc.link = [collbodies_instspc.link; [links_i, v(1+links_i)]];
    end
    % Hänge einen Punkt (Nr. 9) für jedes Gelenk an. Unabhängig, ob 3D-Körper dafür
    collbodies_instspc.type = [collbodies_instspc.type; repmat(uint8(9),R_cc.NJ,1)];
    collbodies_instspc.params = [collbodies_instspc.params; NaN(R_cc.NJ,10)];
    collbodies_instspc.link = [collbodies_instspc.link; ...
      repmat(uint8(NLoffset+(1:R_cc.NJ)'),1,2)];
    % Prüfe den Offset-Parameter für das Schubgelenk
    for i = find(R_cc.MDH.sigma'==1)
      for j = 1:length(Set.task.installspace.links)
        if ~any(Set.task.installspace.links{j} == i-1)
          % Der dem Schubgelenk i zugeordnete Körper i-1 darf sich nicht in
          % einem Teil des Bauraums befinden. Dafür muss ein Offset-Parameter
          % optimiert werden.
          Structure.desopt_prismaticoffset = true;
          break;
        end
      end
    end
  end
  % Stelle äquivalente Gelenknummer von PKM zusammen (als  Übersetzungs- 
  % tabelle). Zeile 1 ursprünglich, Zeile 2 Übersetzung. Siehe oben.
  % Seriell: 0=Basis, 1=erstes bewegtes Segment, ...
  equiv_link = repmat(0:collbodies_instspc.link(end,1),2,1);
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
      iii = equiv_link(1,:) == collbodies_instspc.link(j,1);
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

%% Parameter der Entwurfsoptimierung festlegen
% Dies enthält alle Parameter, die zusätzlich gespeichert werden sollen.
% Typen von Parametern in der Entwurfsoptimierung: 1=Gelenk-Offset, 
% 2=Segmentstärke, 3=Nullstellung von Gelenkfedern
desopt_ptypes = [];
if Structure.desopt_prismaticoffset
  % Gelenk-Offsets. Siehe cds_desopt_prismaticoffset.m
  if Structure.Type == 0 % Serieller Roboter
    desopt_nvars_po = sum(R.MDH.sigma==1);
  else % symmetrische PKM
    desopt_nvars_po = sum(R.Leg(1).MDH.sigma==1);
  end
  desopt_ptypes = [desopt_ptypes; 1*ones(desopt_nvars_po, 1)];
end

if any(strcmp(Set.optimization.desopt_vars, 'linkstrength'))
  % Siehe cds_dimsynth_desopt
  desopt_nvars_ls = 2; % Annahme: Alle Segmente gleich.
  desopt_ptypes = [desopt_ptypes; 2*ones(desopt_nvars_ls, 1)];
end

if any(strcmp(Set.optimization.desopt_vars, 'joint_stiffness_qref'))
  if ~Set.optimization.joint_stiffness_passive_revolute
    error(['Nullstellung der Gelenksteifigkeit soll optimiert werden, ', ...
      'aber es ist keine Steifigkeit definiert']);
  end
  % Siehe cds_dimsynth_desopt
  if Structure.Type == 0 % Serieller Roboter
    error('Gelenkfedern für serielle Roboter noch nicht implementiert');
  else % symmetrische PKM
    desopt_nvars_js = sum(R.Leg(1).MDH.sigma==0);
  end
  desopt_ptypes = [desopt_ptypes; 3*ones(desopt_nvars_js, 1)];
end
Structure.desopt_ptypes = desopt_ptypes;

if nargin == 4 && init_only
  % Keine Optimierung durchführen. Damit kann nachträglich die
  % initialisierte Roboterklasse basierend auf Ergebnissen der Maßsynthese
  % erzeugt werden, ohne dass diese gespeichert werden muss.
  return
end

%% Anfangs-Population generieren
% TODO: Existierende Roboter einfügen
NumIndividuals = Set.optimization.NumIndividuals;
if ~Set.general.only_finish_aborted
  % Generiere Anfangspopulation aus Funktion mit Annahmen bezüglich Winkel.
  % Lädt bisherige Ergebnisse, um schneller i.O.-Werte zu bekommen.
  InitPop = cds_gen_init_pop(Set, Structure);
end
%% Tmp-Ordner leeren
resdir = fullfile(Set.optimization.resdir, Set.optimization.optname, ...
  'tmp', sprintf('%d_%s', Structure.Number, Structure.Name));
if ~Set.general.only_finish_aborted
  if exist(resdir, 'file')
    % Leere Verzeichnis
    rmdir(resdir, 's')
  end
  mkdirs(resdir);
end
%% Fitness-Funktion initialisieren (Strukturunabhängig)
% Zurücksetzen der gespeicherten Werte (aus vorheriger Maßsynthese)
clear cds_fitness
% Initialisierung der Speicher-Funktion (damit testweises Ausführen funk- 
% tioniert; sonst teilw. Fehler im Debug-Modus durch Zugriff auf Variablen)
cds_save_particle_details(Set, R, 0, zeros(length(Set.optimization.objective),1), ...
  zeros(nvars,1), zeros(length(Set.optimization.objective),1), ...
  zeros(length(Set.optimization.constraint_obj),1), ...
  zeros(length(Structure.desopt_ptypes),1), 'reset');
fitnessfcn=@(p)cds_fitness(R, Set, Traj, Structure, p(:)); % Definition der Funktion
if length(Set.optimization.objective) > 1 % Mehrkriteriell (MOPSO geht nur mit vektorieller Fitness-Funktion)
  fitnessfcn_vec=@(P)cds_fitness_vec(R, Set, Traj, Structure, P);
end
if false % Debug: Fitness-Funktion testweise ausführen
  f_test = fitnessfcn(InitPop(1,:)'); %#ok<UNRCH> % Testweise ausführen
  if length(Set.optimization.objective) > 1
    f_test_vec = fitnessfcn_vec(InitPop(1:3,:)); % Testweise ausführen
  end
  % Zurücksetzen der Detail-Speicherfunktion
  cds_save_particle_details(Set, R, 0, zeros(size(f_test)), zeros(nvars,1), ...
    zeros(size(f_test)), zeros(length(Set.optimization.constraint_obj),1), ...
    zeros(length(Structure.desopt_ptypes),1), 'reset');
  % Zurücksetzen der gespeicherten Werte der Fitness-Funktion
  clear cds_fitness
end
%% PSO-Aufruf starten
if Set.general.matfile_verbosity > 0
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_robot2.mat'));
end
if length(Set.optimization.objective) > 1 % Mehrkriteriell: GA-MO oder MOPSO
  if strcmp(Set.optimization.algorithm, 'mopso')
    % Durchführung mit MOPSO; Einstellungen siehe [SierraCoe2005]
    % Und Beispiele aus Matlab File Exchange
    MOPSO_set1 = struct('Np', NumIndividuals, 'Nr', NumIndividuals, ...
      'maxgen', Set.optimization.MaxIter, 'W', 0.4, 'C1', 2, 'C2', 2, 'ngrid', 20, ...
      'maxvel', 5, 'u_mut', 1/nvars); % [SierraCoe2005] S. 4
    options = struct('fun', fitnessfcn_vec, 'nVar', nvars, ...
      'var_min', varlim(:,1), 'var_max', varlim(:,2));
    if Set.general.matfile_verbosity > 2 || Set.general.isoncluster
      mopso_outputfun = @(MS)cds_save_all_results_mopso(MS,Set,Structure);
      options.OutputFcn = {mopso_outputfun};
    end
  elseif strcmp(Set.optimization.algorithm, 'gamultiobj')
    options = optimoptions('gamultiobj');
    options.MaxGenerations = Set.optimization.MaxIter;
    options.PopulationSize = NumIndividuals;
    if ~Set.general.noprogressfigure
      options.PlotFcn = {@gaplotpareto,@gaplotscorediversity};
    end
    if Set.general.matfile_verbosity > 2 || Set.general.isoncluster
      gamo_outputfun = @(options, state, flag) ...
        cds_save_all_results_gamultiobj(options,state,flag,Set,Structure);
      options.OutputFcn = {gamo_outputfun};
    end
  else
    error('Algorithmus %s nicht definiert', Set.optimization.algorithm);
  end
  if ~Set.general.only_finish_aborted % Führe Optimierung durch
    if strcmp(Set.optimization.algorithm, 'mopso')
      options.P0 = InitPop;
      output = MOPSO(MOPSO_set1, options);
      p_val_pareto = output.pos;
      fval_pareto = output.pos_fit;
      cds_log(1, sprintf(['[dimsynth] MOPSO-Optimierung für Rob. %d (%s) ', ...
        'beendet. %d Punkte auf Pareto-Front.'], Structure.Number, ...
        Structure.Name, size(p_val_pareto,1)));
      exitflag = -1; % Nehme an, dass kein vorzeitiger Abbruch erfolgte
    elseif strcmp(Set.optimization.algorithm, 'gamultiobj')
      % Alternative: Durchführung mit gamultiobj (konvergiert schlechter)
      options.InitialPopulationMatrix = InitPop;
      [p_val_pareto,fval_pareto,exitflag,output] = gamultiobj(fitnessfcn, nvars, [], [], [], [],varlim(:,1),varlim(:,2), options);
      cds_log(1, sprintf(['[dimsynth] Optimierung mit GAMULTIOBJ beendet. ', ...
        'generations=%d, funccount=%d, message: %s'], ...
        output.generations, output.funccount, output.message));
    else
      error('Algorithmus %s nicht definiert', Set.optimization.algorithm);
    end
  else
    % Keine Durchführung der Optimierung. Lade Daten der unfertigen
    % Optimierung und speichere sie so ab, als ob die Optimierung
    % durchgeführt wurde.
    if strcmp(Set.optimization.algorithm, 'mopso')
      filelist_tmpres = dir(fullfile(resdir, 'MOPSO_Gen*_AllInd.mat'));
    elseif strcmp(Set.optimization.algorithm, 'gamultiobj')
      filelist_tmpres = dir(fullfile(resdir, 'GAMO_Gen*_AllInd.mat'));
    else
      error('Algorithmus %s nicht definiert', Set.optimization.algorithm);
    end
    if isempty(filelist_tmpres)
      cds_log(1, sprintf(['[dimsynth] Laden des letzten abgebrochenen Durch', ...
        'laufs wurde angefordert. Aber keine Daten in %s vorliegend. Ende.'], resdir));
      return
    else
      cds_log(1, sprintf(['[dimsynth] Laden des letzten abgebrochenen Durch', ...
        'laufs aus gespeicherten Daten erfolgreich.'], resdir));
    end
    [~,I_newest] = max([filelist_tmpres.datenum]);
    d = load(fullfile(resdir, filelist_tmpres(I_newest).name));
    PSO_Detail_Data = d.PSO_Detail_Data;
    if strcmp(Set.optimization.algorithm, 'mopso')
      p_val_pareto = d.REP.pos;
      fval_pareto = d.REP.pos_fit;
      options.P0 = PSO_Detail_Data.pval(:,:,1);
      exitflag = -6;
    elseif strcmp(Set.optimization.algorithm, 'gamultiobj')
      p_val_pareto = d.state.Population;
      fval_pareto = d.state.Score;
      options.InitialPopulationMatrix = PSO_Detail_Data.pval(:,:,1);
      exitflag = 0; % Maximum number of generations exceeded.
    else
      error('Algorithmus %s nicht definiert', Set.optimization.algorithm);
    end
    cds_log(1, sprintf(['[dimsynth] Ergebnis des letzten abgebrochenen ', ...
      'Durchlaufs aus %s geladen.'], filelist_tmpres(I_newest).name));
  end
  % Entferne doppelte Partikel aus der Pareto-Front (bei MOPSO beobachtet)
  [~,I_unique] = unique(p_val_pareto, 'rows');
  if sum(I_unique) ~= size(p_val_pareto,1)
    cds_log(-1, sprintf(['[dimsynth] Pareto-Front bestand aus %d Partikeln, ', ...
      'aber nur %d davon nicht doppelt'], size(p_val_pareto,1), sum(I_unique)));
    fval_pareto = fval_pareto(I_unique,:);
    p_val_pareto = p_val_pareto(I_unique,:);
  end
  % Prüfe, ob die Partikel auf der Pareto-Front wirklich dominant sind.
  % Bei Laden alter Ergebnisse aus MO-GA auch nicht-dominante möglich.
  Idom = pareto_dominance(fval_pareto);
  if any(Idom)
    cds_log(-1, sprintf(['[dimsynth] %d Partikel aus der Pareto-Front werden ', ...
      'dominiert (und sind damit eigentlich nicht Teil der Pareto-Front)'], sum(Idom)));
    fval_pareto = fval_pareto(~Idom,:);
    p_val_pareto = p_val_pareto(~Idom,:);
  end
  % Sortiere die Pareto-Front nach dem ersten Optimierungskriterium. Dann
  % sind die Partikel der Pareto-Front im Diagramm von links nach rechts
  % nummeriert. Das hilft bei der späteren Auswertung (dort Angabe der Nr.)
  [~,Isort] = sort(fval_pareto(:,1));
  fval_pareto = fval_pareto(Isort,:);
  p_val_pareto = p_val_pareto(Isort,:);
  % Gleiche das Rückgabeformat zwischen MO und SO Optimierung an. Es muss
  % immer ein einzelnes Endergebnis geben (nicht nur Pareto-Front)
  % Nehme ein Partikel aus der Mitte (kann unten noch überschrieben werden)
  Iselect = ceil(size(fval_pareto,1)/2);
  p_val = p_val_pareto(Iselect,:)';
  fval = fval_pareto(Iselect,:)';
else % Einkriteriell: PSO
  options = optimoptions('particleswarm');
  options.MaxIter = Set.optimization.MaxIter; %70 100 % in GeneralConfig
  options.SwarmSize = NumIndividuals;
  if any(strcmp(Set.optimization.objective, {'valid_act', 'valid_kin'}))
    % Es soll nur geprüft werden, ob es eine zulässige Lösung gibt.
    % Breche bei einer erfolgreichen Berechnung der Zulässigkeit ab.
    options.ObjectiveLimit = 999;
  elseif Set.optimization.obj_limit > 0
    % Die Grenze zum Abbbruch wurde vom Nutzer gesetzt
    options.ObjectiveLimit = Set.optimization.obj_limit;
  end
  if ~Set.general.noprogressfigure
    options.PlotFcn = {@pswplotbestf};
  end
  % Speichere mat-Dateien nach jeder Iteration des PSO zum Debuggen bei
  % Abbruch.
  if Set.general.matfile_verbosity > 2 || Set.general.isoncluster
    pso_outputfun = @(optimValues,state)cds_save_all_results_pso(optimValues,state,Set,Structure);
    options.OutputFcn = {pso_outputfun};
  end
  options.Display='iter';
  if ~Set.general.only_finish_aborted % Führe Optimierung durch
    options.InitialSwarmMatrix = InitPop;
    [p_val,fval,exitflag,output] = particleswarm(fitnessfcn,nvars,varlim(:,1),varlim(:,2),options);
    cds_log(1, sprintf(['[dimsynth] PSO-Optimierung für Rob. %d (%s) beendet. ', ...
      'iterations=%d, funccount=%d, message: %s'], Structure.Number, Structure.Name, ...
      output.iterations, output.funccount, output.message));
    p_val = p_val(:); % stehender Vektor
  else % Lade Ergebnis einer unfertigen Optimierung
    filelist_tmpres = dir(fullfile(resdir, 'PSO_Gen*_AllInd.mat'));
    if isempty(filelist_tmpres)
      cds_log(1, sprintf(['[dimsynth] Laden des letzten abgebrochenen ', ...
        'Durchlaufs wurde angefordert. Aber keine Daten vorliegend. Ende.']));
      return
    end
    [~,I_newest] = max([filelist_tmpres.datenum]);
    d = load(fullfile(resdir, filelist_tmpres(I_newest).name));
    p_val = d.optimValues.bestx(:);
    fval = d.optimValues.bestfval;
    exitflag = -6;
  end
  p_val_pareto = [];
  fval_pareto = [];
end
% Detail-Ergebnisse künstlich in die Funktion hineinschreiben
if ~Set.general.only_finish_aborted
  % Detail-Ergebnisse extrahieren (persistente Variable in Funktion)
  PSO_Detail_Data = cds_save_particle_details(Set, R, 0, 0, NaN, NaN, NaN, NaN, 'output');
else
  % Nachträgliche Reproduktion eines Ergebnisses evtl problematisch.
  % Ergebnis wird nicht in der persistenten Variable PSO_Detail_Data gespeichert.
  PSO_Detail_Data = d.PSO_Detail_Data;
  if length(Set.optimization.objective) > 1
    if strcmp(Set.optimization.algorithm, 'mopso')
      options.P0 = PSO_Detail_Data.pval(:,:,1);
    elseif strcmp(Set.optimization.algorithm, 'gamultiobj')
      options.InitialPopulationMatrix = PSO_Detail_Data.pval(:,:,1);
    else
      error('Algorithmus %s nicht definiert', Set.optimization.algorithm);
    end
  else
    options.InitialSwarmMatrix = PSO_Detail_Data.pval(:,:,1);
  end
end

if Set.general.matfile_verbosity > 0
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_robot3.mat'));
end
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_robot3.mat'));
%% Nachverarbeitung der Ergebnisse
% Finde die physikalischen Parameter der Pareto-Front (dort ansonsten nur
% normierte und gesättigte Werte gespeichert). Suche in den während der
% Optimierung gespeicherten Werten (persistente Variablen)
if length(Set.optimization.objective) > 1 % Mehrkriteriell
  physval_pareto = NaN(size(fval_pareto));
  for i = 1:size(fval_pareto,1) % Pareto-Front durchgehen
    [k_gen, k_ind] = cds_load_particle_details(PSO_Detail_Data, fval_pareto(i,:)');
    physval_pareto(i,:) = PSO_Detail_Data.physval(k_ind,:,k_gen);
  end
  % Falls Nebenbedingungen gesetzt sind: Wähle von Pareto-Front dazu
  % passende Partikel als Rückgabe aus (für Plausibilität).
  % Das Überschreibt das oben gewählte erste Individuum. Dieser Wert ist
  % aber sowieso nicht kritisch, da meistens die Pareto-Front komplett aus-
  % gewertet wird.
  I_physvalmatch = all(physval_pareto < ...
    repmat(Set.optimization.obj_limit_physval', size(physval_pareto,1), 1), 2);
  I_fvalmatch = all(fval_pareto < ...
    repmat(Set.optimization.obj_limit', size(physval_pareto,1), 1), 2);
  if sum(I_physvalmatch|I_fvalmatch) > 0
    II_match = find(I_physvalmatch|I_fvalmatch, 1, 'first');
    p_val = p_val_pareto(II_match,:)';
    fval = fval_pareto(II_match,:)';
  end
else % Einkriteriell
  physval_pareto = [];
end

% Fitness-Funktion nochmal mit besten Parametern aufrufen. Dadurch werden
% die Klassenvariablen (R.pkin, R.DesPar.seg_par, ...) aktualisiert
if any(strcmp(Set.optimization.objective, 'valid_act'))
  % Keine nochmalige Berechnung in diesem Fall sinnvoll
  max_retry = 0;
else
  max_retry = Set.general.max_retry_bestfitness_reconstruction;
end
if max_retry > Set.optimization.NumIndividuals
  % Reduziere, damit Speicherung in virtueller Post-Final-Generation läuft.
  cds_log(-1, sprintf(['[dimsynth] Anzahl der Fitness-Neuversuche begrenzt ', ...
    'auf %d. Einstellung max_retry_bestfitness_reconstruction passt nicht.'], ...
    Set.optimization.NumIndividuals));
  max_retry = Set.optimization.NumIndividuals;
end

% Für Reproduktion Ergebnis der Entwurfsoptimierung laden.
[k_gen, k_ind] = cds_load_particle_details(PSO_Detail_Data, fval);
desopt_pval = PSO_Detail_Data.desopt_pval(k_ind, :, k_gen)';

for i = 1:max_retry
  % Struktur-Variable neu erstellen um Schalter für Dynamik-Berechnung
  % richtig zu setzen, wenn die Fitness-Funktion neu ausgeführt wird.
	if i == 1
    Structure_tmp = Structure;
    if ~isempty(desopt_pval)
      % Keine erneute Entwurfsoptimierung, also auch keine Regressorform notwendig.
      % Direkte Berechnung der Dynamik, falls für Zielfunktion notwendig.
      Structure_tmp.calc_dyn_act = Structure.calc_dyn_act | Structure.calc_dyn_reg;
      Structure_tmp.calc_spring_act = Structure.calc_spring_act | Structure.calc_spring_reg;
      Structure_tmp.calc_spring_reg = false;
      Structure_tmp.calc_dyn_reg = false;
    end
  end
  
  % Mehrere Versuche vornehmen, da beim Umklappen der Roboterkonfiguration
  % andere Ergebnisse entstehen können.
  % Eigentlich darf sich das Ergebnis aber nicht ändern (wegen der
  % Zufallszahlen-Initialisierung in cds_fitness). Es kann rundungsbedingte
  % Aenderungen des Ergebnisses geben.
  clear cds_fitness % persistente Variable in fitnessfcn löschen (falls Grenzwert erreicht wurde wird sonst inf zurückgegeben)
  % Aufruf nicht über anonmye Funktion, sondern vollständig, damit Param.
  % der Entwurfsoptimierung übergeben werden können.
  [fval_test, ~, Q_test] = cds_fitness(R, Set, Traj, Structure_tmp, p_val, desopt_pval);
  if any(abs(fval_test-fval)>1e-8)
    if all(fval_test < fval)
      t = sprintf('Der neue Wert (%s) ist um [%s] besser als der alte (%s).', ...
        disp_array(fval_test','%1.1f'), disp_array(fval'-fval_test','%1.1e'), disp_array(fval','%1.1f'));
    else
      t = sprintf('Der alte Wert (%s) ist um [%s] besser als der neue (%s).', ...
        disp_array(fval','%1.1f'), disp_array(fval_test'-fval','%1.1e'), disp_array(fval_test','%1.1f'));
    end
    if fval < 5e8 % Warnung ergibt nur Sinn, wenn IK erfolgreich. Sonst immer Abweichung.
      cds_log(-1, sprintf('[dimsynth] Bei nochmaligem Aufruf der Fitness-Funktion kommt nicht der gleiche Wert heraus (Versuch %d). %s', i, t));
    end
    if all(fval_test < fval)
      fval = fval_test;
      cds_log(1,sprintf('[dimsynth] Nehme den besseren neuen Wert als Ergebnis ...'));
      break;
    end
  else
    if i > 1, cds_log(1,sprintf('[dimsynth] Zielfunktion konnte nach %d Versuchen rekonstruiert werden', i)); end
    break;
  end
end
% Detail-Ergebnisse extrahieren (persistente Variable in Funktion).
% (Nochmal, da Neuberechnung oben eventuell anderes Ergebnis bringt)
% Kein Zurücksetzen der persistenten Variablen notwendig.
if ~Set.general.only_finish_aborted
  PSO_Detail_Data = cds_save_particle_details(Set, R, 0, 0, NaN, NaN, NaN, NaN, 'output');
end
% Schreibe die Anfangswerte der Gelenkwinkel für das beste Individuum in
% die Roboterklasse. Suche dafür den besten Funktionswert in den zusätzlich
% gespeicherten Daten für die Position des Partikels in dem Optimierungsverfahren
[dd_optgen, dd_optind] = cds_load_particle_details(PSO_Detail_Data, fval);
q0_ik = PSO_Detail_Data.q0_ik(dd_optind,:,dd_optgen)';
qlim_pso = NaN(length(q0_ik),2);
qlim_pso(:,1) = PSO_Detail_Data.q_min(dd_optind,:,dd_optgen)';
qlim_pso(:,2) = PSO_Detail_Data.q_max(dd_optind,:,dd_optgen)';
% Prüfen, ob diese mit der Klassenvariable (aus dem letzten Fitness-Aufruf)
% übereinstimmen. Muss nicht sein, da Zufallskomponente bei IK-Anfangswerten
if max_retry > 0 % nur sinnvoll, falls Fitness nach Optimierungs-Ende neu berechnet
  if Structure.Type == 0, q0_ik2 = R.qref;
  else,                   q0_ik2 = cat(1,R.Leg.qref); end
  test_q0 = q0_ik - q0_ik2;
  test_q0(abs(abs(test_q0)-2*pi)<1e-6) = 0; % entferne 2pi-Fehler
  if any(abs(test_q0)>1e-10) && all(fval<1e10) % nur bei erfolgreicher Berechnung der IK ist der gespeicherte Wert sinnvoll
    cds_log(-1, sprintf(['[dimsynth] IK-Anfangswinkel sind bei erneuter ', ...
      'Berechnung anders. Kann passieren, aber nachteilig für Reproduzierbar', ...
      'keit des Ergebnisses. max. Abweichung: %1.2e.'], max(abs(test_q0))));
  end
end
% Schreibe die während der Optimierung gespeicherten Anfangswerte in der Klasse
% Annahme: Damit ist immer eine Reproduktion des Endergebnisses (Optimum) möglich
if Structure.Type == 0 % Seriell
  R.qref = q0_ik;
else % Parallel
  for i = 1:R.NLEG, R.Leg(i).qref = q0_ik(R.I1J_LEG(i):R.I2J_LEG(i)); end
end
% Vergrößere die Grenzen für Drehgelenke entsprechend der vorherigen
% Vorgaben. Dadurch Vermeidung von Problemen, wenn ein Gelenk kaum oder
% keine Auslenkung hat und später die Grenzen der ursprünglichen
% Trajektorie nicht eingehalten werden. Nicht für Schubgelenke machen, da
% dies die Masse beeinflusst (wegen Führungsschiene)
qlim_neu = qlim_pso; % aus PSO gespeicherte Grenzen (dort aus Gelenk-Traj.)
qlim_mitte = mean(qlim_pso,2); % Mittelwert als Ausgangspunkt für neue Grenzen
q_range = diff(Structure.qlim')'; % ursprüngliche max. Spannweite aus Einstellungen
qlim_neu(R.MDH.sigma==0,:) = repmat(qlim_mitte(R.MDH.sigma==0),1,2) + ...
  [-q_range(R.MDH.sigma==0)/2, q_range(R.MDH.sigma==0)/2];
if R.Type == 0 % Seriell
  R.qlim(R.MDH.sigma==0,:) = qlim_neu(R.MDH.sigma==0,:);
else % PKM
  for i = 1:R.NLEG
    qlim_neu_i = qlim_neu(R.I1J_LEG(i):R.I2J_LEG(i),:);
    R.Leg(i).qlim(R.Leg(i).MDH.sigma==0,:) = qlim_neu_i(R.Leg(i).MDH.sigma==0,:);
  end
end

% Berechne Inverse Kinematik zu erstem Bahnpunkt
Traj_0 = cds_transform_traj(R, Traj);
% Einstellung für Positions-IK: Keine Normalisierung mehr, da Sprung bei pi
% ungünstig bei vorab festgelegten Grenzen für Koordinaten
s_ik = struct('normalize', false);
if Structure.Type == 0 % Seriell
  % Benutze Referenzpose die bei obigen Zielfunktionsaufruf gespeichert wurde
  [q, Phi] = R.invkin2(R.x2tr(Traj_0.XE(1,:)'), R.qref, s_ik);
else % Parallel
  [q, Phi] = R.invkin_ser(Traj_0.XE(1,:)', cat(1,R.Leg.qref), s_ik);
end
if ~any(strcmp(Set.optimization.objective, 'valid_act')) && any(abs(Phi)>1e-8)
  cds_log(-1, '[dimsynth] PSO-Ergebnis für Startpunkt nicht reproduzierbar (ZB-Verletzung)');
end
% Berechne IK der Bahn (für spätere Visualisierung und Neuberechnung der Leistungsmerkmale)
% Benutze ähnliche Einstellungen wie in cds_constraints.m (aber feinere
% Toleranz und mehr Rechenaufwand bei der eigentlichen IK-Berechnung)
% Hier auch Weglassen der Beachtung der Winkelgrenzen (führt teilweise zu
% Abbruch, obwohl die Spannweite in Ordnung ist.)
if Structure.Type == 0 % Seriell
  s = struct('normalize', false, 'Phit_tol', 1e-12, ...
    'Phir_tol', 1e-12, 'n_max', 5000);
  [Q, QD, QDD, PHI] = R.invkin2_traj(Traj_0.X, Traj.XD, Traj.XDD, Traj.t, q, s);
  Jinv_ges = [];
else % Parallel
  s = struct('normalize', false,  'Phit_tol', 1e-12, ...
    'Phir_tol', 1e-12, 'n_max', 5000);
  [Q, QD, QDD, PHI, Jinv_ges] = R.invkin2_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD, Traj_0.t, q, s);
end
test_q = abs(Q(1,:)'-q0_ik);
test_q(abs(abs(test_q)-2*pi)<1e-2) = 0; % entferne 2pi-Fehler, großzügige Toleranz
if any(test_q > 1e-6) && all(fval<1e10) % nur wenn IK erfolgreich war testen
  cds_log(-1, sprintf(['[dimsynth] Die Neu berechneten IK-Werte (q0) der Trajektorie stimmen nicht ', ...
    'mehr mit den ursprünglich berechneten überein. Max diff.: %1.4e'], max(test_q)));
end
if any(q<qlim_neu(:,1) | q>qlim_neu(:,2))
  cds_log(-1, sprintf(['[dimsynth] Startwert für Gelenkwinkel liegt außer', ...
    'halb des erlaubten Bereichs.']));
  save(fullfile(resdir, 'jointlimitviolationwarning.mat'));
end
result_invalid = false;
if ~any(strcmp(Set.optimization.objective, 'valid_act')) && ...
    (any(abs(PHI(:))>1e-6) || any(isnan(Q(:)))) % Toleranz wie in cds_constraints
  % Berechnung der Trajektorie ist fehlgeschlagen
  if any(fval<1e4*1e4) % nur bemerkenswert, falls vorher überhaupt soweit gekommen.
    save(fullfile(resdir, 'trajikreprowarning.mat'));
    cds_log(-1, sprintf(['[dimsynth] PSO-Ergebnis für Trajektorie nicht reproduzierbar ', ...
      'oder nicht gültig (ZB-Verletzung). Max IK-Fehler: %1.1e. %d Fehler > 1e-6. %d mal NaN.'], ...
      max(abs(PHI(:))), sum(sum(abs(PHI)>1e-6,2)>0), sum(isnan(Q(:))) ));
    % Vergleiche die neu berechnete Trajektorie und die aus der Fitness-Funktion
    if Set.general.max_retry_bestfitness_reconstruction > 0 && ~isempty(Q_test) && ...
        ~Set.general.isoncluster
      change_current_figure(654); clf;
      for i = 1:R.NJ
        subplot(ceil(sqrt(R.NJ)), ceil(R.NJ/ceil(sqrt(R.NJ))), i); hold on;
        plot(Traj_0.t, Q(:,i));
        plot(Traj_0.t, Q_test(:,i));
        ylabel(sprintf('q%d', i));
      end
      legend({'invkin_traj', 'fitnessfcn'});
      sgtitle('Neuberechnung Gelenk-Traj. Debug');
    end
  end
  result_invalid = true;
end
%% Berechne andere Leistungsmerkmale
Structure_tmp = Structure; % Eingabe um Berechnung der Antriebskräfte zu erzwingen
Structure_tmp.calc_dyn_act = true;
Structure_tmp.calc_cut = true; % ... und der Schnittkräfte
Structure_tmp.calc_dyn_reg = false;
if Set.optimization.joint_stiffness_passive_revolute
  Structure_tmp.calc_spring_act = true;
  Structure_tmp.calc_spring_reg = false;
end
if R.Type ~= 0 % für PKM
  % Berechne Dynamik in diesem Abschnitt mit Inertialparametern. Sonst
  % keine Berechnung der Schnittkräfte möglich (mit Minimalparametern)
	R.DynPar.mode = 3; 
  for i = 1:R.NLEG, R.Leg(i).DynPar.mode = 3; end
end
if ~result_invalid && ~any(strcmp(Set.optimization.objective, 'valid_act'))
  % Masseparameter belegen, falls das nicht vorher passiert ist.
  % Nachbildung der Bedingungen für Belegung der Masseparameter in cds_fitness.m
  % Stelle fest, ob die Zielfunktion rein kinematisch ist; dann werden die
  % Dynamikparameter nicht in der Fitness-Funktion belegt:
  % ... mehrkriteriell und nur kinematische Zielfunktionen ...
  only_kinematic_objective = length(intersect(Set.optimization.objective, ...
    {'condition', 'jointrange', 'manipulability', 'minjacsingval', ...
    'positionerror', 'chainlength'})) == 6;
  % einkriteriell und kinematische ZF
  only_kinematic_objective = only_kinematic_objective || ...
    length(Set.optimization.objective) == 1 && any(strcmp(Set.optimization.objective, ...
    {'condition', 'jointrange', 'manipulability', 'minjacsingval', ...
    'positionerror', 'chainlength'}));
  if any(fval > 1e3) ...% irgendeine Nebenbedingung wurde immer verletzt. ...
      || only_kinematic_objective % ... oder nur kinematische Zielfunktion ...
    cds_dimsynth_design(R, Q, Set, Structure); % ...  Daher nie bis zu diesem Funktionsaufruf gekommen.
  end
  data_dyn = cds_obj_dependencies(R, Traj_0, Set, Structure_tmp, Q, QD, QDD, Jinv_ges);
  % Einzelne Zielfunktionen aufrufen
  [fval_mass,~, ~, physval_mass] = cds_obj_mass(R);
  [fval_energy,~, ~, physval_energy] = cds_obj_energy(R, Set, Structure, Traj_0, data_dyn.TAU, QD);
  [fval_actforce,~, ~, physval_actforce] = cds_obj_actforce(data_dyn.TAU);
  [fval_ms, ~, ~, physval_ms] = cds_obj_materialstress(R, Set, data_dyn, Jinv_ges, Q, Traj_0);
  [fval_cond,~, ~, physval_cond] = cds_obj_condition(R, Set, Structure, Jinv_ges, Traj_0, Q, QD);
  [fval_mani,~, ~, physval_mani] = cds_obj_manipulability(R, Set, Jinv_ges, Traj_0, Q);
  [fval_msv,~, ~, physval_msv] = cds_obj_minjacsingval(R, Set, Jinv_ges, Traj_0, Q);
  [fval_pe,~, ~, physval_pe] = cds_obj_positionerror(R, Set, Jinv_ges, Traj_0, Q);
  [fval_jrange,~, ~, physval_jrange] = cds_obj_jointrange(R, Set, Structure, Q);
  [fval_chainlength,~, ~, physval_chainlength] = cds_obj_chainlength(R);
  [fval_stiff,~, ~, physval_stiff] = cds_obj_stiffness(R, Set, Q);
  % Reihenfolge siehe Variable Set.optimization.constraint_obj aus cds_settings_defaults
  fval_obj_all = [fval_mass; fval_energy; fval_actforce; fval_ms; fval_cond; ...
    fval_mani; fval_msv; fval_pe; fval_jrange; fval_chainlength; fval_stiff];
  physval_obj_all = [physval_mass; physval_energy; physval_actforce; ...
    physval_ms; physval_cond; physval_mani; physval_msv; physval_pe; ...
    physval_jrange; physval_chainlength; physval_stiff];
  % Vergleiche neu berechnete Werte mit den zuvor abgespeicherten (müssen
  % übereinstimmen)
  test_Jcond_abs = PSO_Detail_Data.constraint_obj_val(dd_optind, 4, dd_optgen) - physval_cond;
  test_Jcond_rel = test_Jcond_abs / physval_cond;
  if abs(test_Jcond_abs) > 1e-6 && test_Jcond_rel > 1e-3
    save(fullfile(resdir, 'condreprowarning.mat'));
    cds_log(-1, sprintf(['[dimsynth] Während Optimierung gespeicherte ', ...
      'Konditionszahl (%1.5e) stimmt nicht mit erneuter Berechnung (%1.5e) ', ...
      'überein. Differenz %1.5e (%1.2f%%)'], ...
      PSO_Detail_Data.constraint_obj_val(dd_optind, 4, dd_optgen), ...
      physval_cond, test_Jcond_abs, test_Jcond_rel));
  end
  test_sv = PSO_Detail_Data.constraint_obj_val(dd_optind, 6, dd_optgen) - physval_ms;
  if abs(test_sv) > 1e-5
    save(fullfile(resdir, 'svreprowarning.mat'));
    cds_log(-1, sprintf(['[dimsynth] Während Optimierung gespeicherte ', ...
      'Materialbelastung (%1.5e) stimmt nicht mit erneuter Berechnung (%1.5e) ', ...
      'überein. Differenz %1.5e.'], ...
      PSO_Detail_Data.constraint_obj_val(dd_optind, 6, dd_optgen), physval_ms, test_sv));
  end
  test_actforce = PSO_Detail_Data.constraint_obj_val(dd_optind, 3, dd_optgen) - physval_actforce;
  if abs(test_actforce) > 1e-5
    save(fullfile(resdir, 'actforcereprowarning.mat'));
    cds_log(-1, sprintf(['[dimsynth] Während Optimierung gespeicherte ', ...
      'Antriebskraft (%1.5e) stimmt nicht mit erneuter Berechnung (%1.5e) ', ...
      'überein. Differenz %1.5e.'], ...
      PSO_Detail_Data.constraint_obj_val(dd_optind, 3, dd_optgen), physval_actforce, test_actforce));
  end
  test_mass = PSO_Detail_Data.constraint_obj_val(dd_optind, 1, dd_optgen) - physval_mass;
  if abs(test_mass) > 1e-5
    save(fullfile(resdir, 'massreprowarning.mat'));
    cds_log(-1, sprintf('[dimsynth] Während Optimierung gespeicherte Masse (%1.5e) stimmt nicht mit erneuter Berechnung (%1.5e) überein. Differenz %1.5e.', ...
      PSO_Detail_Data.constraint_obj_val(dd_optind, 1, dd_optgen), physval_mass, test_mass));
  end
else
  % Keine Berechnung der Leistungsmerkmale möglich, da keine zulässige Lösung
  % gefunden wurde.
  fval_obj_all = NaN(11,1);
  physval_obj_all = NaN(11,1);
end
% Prüfe auf Plausibilität, ob die Optimierungsziele erreicht wurden. Neben-
% bedingungen nur prüfen, falls überhaupt gültige Lösung erreicht wurde.
I_fobj_set = Set.optimization.constraint_obj ~= 0;
% Die Reihenfolge der Zielfunktionen insgesamt und die der Zielfunktionen
% als Grenze sind unterschiedlich. Finde Indizes der einen in den anderen.
objconstr_names_all = {'mass', 'energy', 'actforce', 'condition', ...
  'stiffness', 'materialstress'};
obj_names_all = {'mass', 'energy', 'actforce', 'materialstress', 'condition', ...
  'manipulability', 'minjacsingval', 'positionerror', 'jointrange', ...
  'chainlength', 'stiffness'};
I_constr = zeros(length(objconstr_names_all),1);
for i = 1:length(objconstr_names_all)
  I_constr(i) = find(strcmp(objconstr_names_all{i}, obj_names_all));
end
% Indizes der verletzten Nebenbedingungen. Wert Null heißt inaktiv.
I_viol = physval_obj_all(I_constr) > Set.optimization.constraint_obj & I_fobj_set;
if any(fval<1e3) && any(I_viol)
  save(fullfile(resdir, 'objconstrwarning.mat'));
  for i = find(I_viol)'
    cds_log(-1,sprintf(['[dimsynth] Zielfunktions-Nebenbedingung %d (%s) verletzt ', ...
      'trotz Berücksichtigung in Optimierung: %1.4e > %1.4e. Keine Lösung gefunden.'], ...
      i, objconstr_names_all{i}, physval_obj_all(I_constr(i)), Set.optimization.constraint_obj(i)));
  end
end
if any(fval<1e3) && Set.optimization.constraint_obj(6) ~= 0 && ...
    physval_ms > Set.optimization.constraint_obj(6) 
  save(fullfile(resdir, 'strengthconstrwarning.mat'));
  cds_log(-1, sprintf('[dimsynth] Materialbelastungs-Nebenbedingung verletzt trotz Berücksichtigung in Optimierung. Keine Lösung gefunden.'));
end
% Bestimme Zuordnung der Fitness-Vektor-Einträge zu den Leistungsmerkmalen
% (Erleichtert spätere Auswertung bei Pareto-Optimierung)
I_fval_obj_all = zeros(length(Set.optimization.objective),1);
for i = 1:length(Set.optimization.objective)
  II_i = strcmp(Set.optimization.objective{i},obj_names_all);
  if any(II_i) % Bei Zielfunktion valid_act wird kein Leistungsmerkmal berechnet.
    I_fval_obj_all(i) = find(II_i);
  end
end
% Prüfe ob die Zielfunktion durch Neuberechnung der Leistungsmerkmale
% reproduziert werden konnte (Plausibilität der Berechnungen)
for i = 1:length(Set.optimization.objective)
  if I_fval_obj_all(i) == 0, continue; end % keine Prüfung möglich.
  if fval(i) > 1e3, continue; end % Leistungsmerkmal steht nicht in Zielfunktion (NB-Verletzung)
  test_fval_i = fval(i) - fval_obj_all(I_fval_obj_all(i));
  if abs(test_fval_i) > 1e-6
    save(fullfile(resdir, 'fvalreprowarning.mat'));
    cds_log(-1, sprintf(['[dimsynth] Zielfunktionswert %d (%s) nicht aus Neu', ...
      'berechnung der Leistungsmerkmale reproduzierbar. Aus PSO: %1.5f, neu: %1.5f.'], ...
      i, obj_names_all{I_fval_obj_all(i)}, fval(i), fval_obj_all(I_fval_obj_all(i))));
  end
end
%% Speichere Ergebnisse der Entwurfsoptimierung
desopt_pval_pareto = NaN(size(fval_pareto,1),length(Structure.desopt_ptypes));
for i = 1:size(fval_pareto,1)
  [k_gen, k_ind] = cds_load_particle_details(PSO_Detail_Data, fval_pareto(i,:)');
  desopt_pval_pareto(i,:) = PSO_Detail_Data.desopt_pval(k_ind, :, k_gen);
end
[k_gen, k_ind] = cds_load_particle_details(PSO_Detail_Data, fval);
desopt_pval = PSO_Detail_Data.desopt_pval(k_ind, :, k_gen)';

%% Speichere die Anfangswerte der IK der Pareto-Front
% Sind notwendig, um sicher die Ergebnisse reproduzieren zu können
q0_pareto = NaN(size(fval_pareto,1), size(Q,2));
for i = 1:size(fval_pareto,1)
  [k_gen, k_ind] = cds_load_particle_details(PSO_Detail_Data, fval_pareto(i,:)');
  q0_pareto(i,:) = PSO_Detail_Data.q0_ik(k_ind,:,k_gen)';
end

%% Ausgabe der Ergebnisse
t_end = now(); % End-Zeitstempel der Optimierung dieses Roboters
% Allgemeine Ergebnis-Struktur. Enthält die wichtisten Informationen
% um das Endergebnis reproduzieren zu können.
RobotOptRes = struct( ...
  'fval', fval, ... % Zielfunktionswert (nach dem optimiert wurde)
  'fval_obj_all', fval_obj_all, ... % Werte aller möglicher einzelner Zielf.
  'physval_obj_all', physval_obj_all, ... % Physikalische Werte aller Zielf.
  'p_val', p_val, ... % Parametervektor der Optimierung
  'desopt_pval', desopt_pval, ... % Entwurfsparameter zum Ergebnis
  'fval_pareto', fval_pareto, ... % Alle Fitness-Werte der Pareto-Front
  'physval_pareto', physval_pareto, ... % physikalische Werte dazu
  'p_val_pareto', p_val_pareto, ... % Alle Parametervektoren der P.-Front
  'desopt_pval_pareto', desopt_pval_pareto, ... % Alle Entwurfsparameter zu den Pareto-Punkten
  'q0_pareto', q0_pareto, ... % Alle IK-Anfangswerte aller Pareto-Partikel
  'q0', q, ... % Anfangs-Gelenkwinkel für Lösung der IK
  'I_fval_obj_all', I_fval_obj_all, ... % Zuordnung der Fitness-Einträge zu fval_obj_all
  'p_limits', varlim, ... % Grenzen für die Parameterwerte
  'timestamps_start_end', [t_start, t_end, toc(t1)], ...
  'exitflag', exitflag, ...
  'Structure', Structure);
% Detail-Informationen. Damit lässt sich die Bestimmung des Fitness-Werts
% detailliert nachvollziehen
RobotOptDetails = struct( ...
  'R', R, ... % Roboter-Klasse
  'options', options, ... % Optionen des Optimierungs-Algorithmus
  'Traj_Q', Q, ...
  'Traj_QD', QD, ...
  'Traj_QDD', QDD, ...
  'Traj_PHI', PHI, ...
  'timestamps_start_end', [t_start, t_end, toc(t1)], ...
  'fitnessfcn', fitnessfcn);
% Debug: Durch laden dieser Ergebnisse kann nach Abbruch des PSO das
% Ergebnis trotzdem geladen werden
if Set.general.matfile_verbosity > 0
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_robot4.mat'));
end
% Gesamtergebnis der Optimierung speichern
save(fullfile(Set.optimization.resdir, Set.optimization.optname, ...
  sprintf('Rob%d_%s_Endergebnis.mat', Structure.Number, Structure.Name)), ...
  'RobotOptRes');
save(fullfile(Set.optimization.resdir, Set.optimization.optname, ...
  sprintf('Rob%d_%s_Details.mat', Structure.Number, Structure.Name)), ...
  'RobotOptDetails', 'PSO_Detail_Data');
lfp = cds_log(1,sprintf(['[dimsynth] Optimierung von Rob. %d (%s) abgeschlossen. ', ...
  'Dauer: %1.1fs'], Structure.Number, Structure.Name, toc(t1)));
if isempty(lfp) % Aufruf mit Set.general.only_finish_aborted. Log nicht initialisiert.
  robstr = sprintf('Rob%d_%s', Structure.Number, Structure.Name);
  lfp = fullfile(resdir, robstr, sprintf('%s.log', robstr));
end
% Log-Datei komprimieren und Textdatei löschen
if exist(lfp, 'file')
  gzip(lfp); delete(lfp);
else
  % Dieser Fall kann eintreten, wenn der nachträgliche Abschluss der
  % abgebrochenen Optimierung mehrfach durchgeführt wurde.
  warning(['[dimsynth] Log-Datei %s existiert nicht (mehr). Kein ', ...
    'zippen+löschen mehr notwendig.'], lfp);
end
% Lösche temporäre Ergebnisse. Nicht mehr benötigt, da Endergebnis da.
filelist_tmpres = [dir(fullfile(resdir, '*PSO_Gen*_AllInd.mat')); ...
  dir(fullfile(resdir, '*GAMO_Gen*_AllInd.mat'));];
for i = 1:length(filelist_tmpres)
  delete(fullfile(resdir, filelist_tmpres(i).name));
end
