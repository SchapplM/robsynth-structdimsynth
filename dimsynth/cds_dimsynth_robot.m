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
resultfile = fullfile(Set.optimization.resdir, Set.optimization.optname, ...
    sprintf('Rob%d_%s_Endergebnis.mat', Structure.Number, Structure.Name));
if ~Set.general.overwrite_existing_results
  if exist(resultfile, 'file')
    fprintf('[dimsynth] Ergebnis für Rob %d (%s) liegt bereits vor. Abbruch.\n', ...
      Structure.Number, Structure.Name);
    return
  end
end
if Set.general.only_finish_aborted && exist(resultfile, 'file')
  fprintf(['[dimsynth] Ergebnis für Rob %d (%s) liegt bereits vor. Kein ', ...
    'erneuter Abschluss notwendig.\n'], Structure.Number, Structure.Name);
  return
end
%% Initialisierung
% Log-Datei initialisieren
clear cds_log % Falls Durchlauf direkt hiervor und jetzt mit only_finish_aborted
if ~init_only && ~Set.general.only_finish_aborted
  resdir_rob = fullfile(Set.optimization.resdir, Set.optimization.optname, ...
    sprintf('Rob%d_%s', Structure.Number, Structure.Name));
  mkdirs(resdir_rob); % Ergebnis-Ordner für diesen Roboter erstellen
  cds_log(1, sprintf('[dimsynth] Start der Maßsynthese für %s',  ...
    Structure.Name), 'init', Set, Structure);
end
% Zurücksetzen der Detail-Speicherfunktion
clear cds_save_particle_details;
% Anpassung der eingegebenen Struktur-Variable an Aktualisierungen
if ~isfield(Structure, 'RobName'), Structure.RobName = ''; end  
%% Referenzlänge ermitteln
% Mittelpunkt der Aufgabe
Structure.xT_mean = mean(minmax2(Traj.X(:,1:3)'), 2);
% Charakteristische Länge der Aufgabe (empirisch ermittelt aus der Größe
% des notwendigen Arbeitsraums)
Lref = norm(diff(minmax2(Traj.X(:,1:3)')'));
% Bei vorgegebener Basis-Position: Zähle Abstand von Basis zu
% Aufgaben-Mitte zu Referenz-Länge hinzu
if ~Set.optimization.movebase
  r_W_0 = zeros(3,1);
  for k = 1:3
    if all(~isnan(Set.optimization.basepos_limits(k,:)))
      r_W_0(k) = mean(Set.optimization.basepos_limits(k,:));
    end
  end
  Lref = Lref + norm(Structure.xT_mean-r_W_0);
end
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
if ~isempty(Structure.RobName) && ~Set.optimization.fix_joint_limits
  cds_log(-1, sprintf(['[dimsynth] Gelenkwinkelgrenzen nicht fixiert ', ...
    'aber konkreter Roboter gegeben. Nicht sinnvoll.']));
end
if ~Set.optimization.fix_joint_limits && any(strcmp(Set.optimization.objective, 'jointlimit'))
  cds_log(-1, ['Optimierung mit Ziel Gelenkwinkelgrenzen bei nicht-festen ', ...
    'Grenzen nicht sinnvoll']);
end
if Structure.Type == 0 % Seriell
  R = serroblib_create_robot_class(Structure.Name, Structure.RobName);
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
  if all(~isnan(Set.optimization.base_size_limits))
    p_base(1) = mean(Set.optimization.base_size_limits);
  end
  % Parameter für Plattform-Kopplung einstellen
  p_platform = 0.75*Lref;
  if all(~isnan(Set.optimization.platform_size_limits))
    p_platform(1) = mean(Set.optimization.platform_size_limits);
  end
  if any(Structure.Coupling(2) == [4,5,6])
    p_platform(2) = 0.5*p_platform(1); % Paar-Abstand halb so groß wie Radius
  elseif Structure.Coupling(2) == 8
    p_platform(2) = 0; % Kein Offset-Winkel für Gelenkachsen
  end
  % Bei paralleler Rechnung der Struktursynthese auf Cluster Konflikte vermeiden
  parroblib_writelock('check', 'csv', logical(Set.task.DoF), 5*60, false);
  % Klasse initialisierung (liest auch die csv-Dateien aus).
  R = parroblib_create_robot_class(Structure.Name, p_base(:), p_platform(:));
  NLEG = R.NLEG;
  R.update_dynpar1(R.DynPar.mges, R.DynPar.rSges, R.DynPar.Icges); % Nochmal initialisieren, damit MPV definiert ist
else
  error('Typ-Nummer nicht definiert');
end
% Initialisieren der Funktionsdatei-Verknüpfungen. Keine Synchronisation
% mit parroblib_writelock notwendig, da bereits zu Beginn geprüft.
R.fill_fcn_handles(Set.general.use_mex, true);
% Aufgaben-FG des Roboters setzen
if Structure.Type == 0 % Seriell
  R.I_EE_Task = Set.task.DoF;
else % Parallel
  R.update_EE_FG(R.I_EE, Set.task.DoF);
end
if all(Set.task.DoF == [1 1 1 1 1 0])
  Set.task.pointing_task = true;
end
% Speichere die Eigenschaft der Aufgabenredundanz
Structure.task_red = ...
  R.Type == 0 && sum(R.I_EE_Task) < R.NJ || ... % Seriell: Redundant wenn mehr Gelenke als Aufgaben-FG
  R.Type == 2 && sum(R.I_EE_Task) < sum(R.I_EE); % Parallel: Redundant wenn mehr Plattform-FG als Aufgaben-FG

% Platzhalter für Vorgabe der Traj-IK-Anfangswerte
Structure.q0_traj = NaN(R.NJ, 1);
for i = 1:NLEG
  if Structure.Type == 0
    R_init = R;
  else
    R_init = R.Leg(i);
  end
  if isempty(Structure.RobName) % nur machen, wenn Kinematikparameter frei wählbar
    R_init.gen_testsettings(false, true); % Setze Kinematik-Parameter auf Zufallswerte
  end
  if ~Set.optimization.fix_joint_limits
  % Gelenkgrenzen setzen: Schubgelenke (Verfahrlänge nicht mehr als "fünf
  % mal schräg durch Arbeitsraum" (char. Länge))
  % Muss so hoch gesetzt sein, damit UPS-Kette (ohne sonstige
  % Kinematikparameter auch funktioniert)
  R_init.qlim(R_init.MDH.sigma==1,:) = repmat([-5*Lref, 5*Lref],sum(R_init.MDH.sigma==1),1);
  if ~isnan(Set.optimization.max_range_prismatic)
    % Schätzwert für Begrenzung der Schubgelenke wird durch Benutzervorgabe
    % überschrieben. Z.B. Mit unendlich, um komplett frei zu lassen.
    % Größe des Roboters wird dann durch andere Kennzahlen begrenzt
    R_init.qlim(R_init.MDH.sigma==1,:) = repmat([-0.5, 0.5]*...
      Set.optimization.max_range_prismatic,sum(R_init.MDH.sigma==1),1);
  end
  % Gelenkgrenzen setzen: Drehgelenke
  if Structure.Type == 0 % Serieller Roboter
    % Grenzen für Drehgelenke: Alle sind aktiv
    R_init.qlim(R.MDH.sigma==0,:) = repmat([-0.5, 0.5]*... % Drehgelenk
      Set.optimization.max_range_active_revolute, sum(R.MDH.sigma==0),1);
  else % Paralleler Roboter
    % Grenzen für passive Drehgelenke (aktive erstmal mit setzen)
    R_init.qlim(R_init.MDH.sigma==0,:) = repmat([-0.5, 0.5]*... % Drehgelenk
      Set.optimization.max_range_passive_revolute, sum(R_init.MDH.sigma==0),1);
    % Grenzen für technische Gelenke gesondert setzen
    R_init.qlim(R_init.DesPar.joint_type==2,:) = repmat([-0.5, 0.5]*... % Kardan-Gelenk
      Set.optimization.max_range_passive_universal, sum(R_init.DesPar.joint_type==2),1);
    R_init.qlim(R_init.DesPar.joint_type==3,:) = repmat([-0.5, 0.5]*... % Kugelgelenk
      Set.optimization.max_range_passive_spherical, sum(R_init.DesPar.joint_type==3),1);
    % Behandle den letzten Rotations-FG eines Kugelgelenks wie ein
    % Drehgelenk. Ursache: Kugel kann freie Längsdrehung machen.
    % TODO: Einbaulage des Kugelgelenks sollte frei definierbar sein (z.B.
    % längs zur Stabrichtung)
    I_spherical = find(R_init.DesPar.joint_type==3);
    if ~isempty(I_spherical) % Annahme: Es gibt ein Kugelgelenk in der Kette
      R_init.qlim(I_spherical(end),:) = [-0.5, 0.5] * ... % wie Drehgelenk
        Set.optimization.max_range_passive_revolute;
    end
    % Grenzen für aktives Drehgelenk setzen
    I_actrevol = R_init.MDH.mu == 2 & R_init.MDH.sigma==0;
    R_init.qlim(I_actrevol,:) = repmat([-0.5, 0.5]*Set.optimization.max_range_active_revolute, sum(I_actrevol),1);
  end
  end
  % Eintragen in Strukturvariable
  if Structure.Type == 0 % Serieller Roboter
    Structure.qlim = R_init.qlim;
  else % Paralleler Roboter
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
  % Eintragen der Grenzen in die Strukturvariable. Maßgeblich für die
  % Prüfung der Grenzen in der Maßsynthese
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
% Reduziere die Grenzen in der Klassenvariable. Diese Grenzen werden für
% die inverse (Trajektorien-)Kinematik benutzt. Aufgrund von numerischen
% Ungenauigkeiten können die Grenzen dort teilweise überschritten werden
for i = 1:NLEG
  if Structure.Type == 0
    R_init = R;
  else
    R_init = R.Leg(i);
  end
  R_init.qDlim =  0.99*R_init.qDlim;
  R_init.qDDlim = 0.98*R_init.qDDlim;
end
% Trage Grenzen für die Endeffektor-Bewegung ein. Das ist nur wirksam,
% falls Aufgabenredundanz vorliegt.
if Set.task.pointing_task
  R.xDlim = [NaN(5,2); [-1,1]*Set.optimization.max_velocity_ee_rotation];
  R.xDDlim = [NaN(5,2); [-1,1]*Set.optimization.max_acceleration_ee_rotation];
end
% Erste Pose der Trajektorie merken (für darauf aufbauende Anpassung der
% Plattform-Gelenke)
Structure.xref_W = Traj.X(1,:)';

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
if strcmp(mounting, 'floor')
  % Nichts ändern. Basis-KS zeigt (mit z-Achse) nach oben
  R.update_base([], zeros(3,1));
elseif strcmp(mounting, 'ceiling')
  % Roboter zeigt nach unten. x-Achse bleibt gleich
  R.update_base([], [pi;0;0]); % xyz-Euler-Winkel
  % Drehe End-Effektor auch um. Die Aufgaben sind so definiert, dass die
  % z-Achse standardmäßig (im Welt-KS) nach oben zeigt. Sonst ist bei
  % 2T1R, 3T0R und 3T1R die IK nicht lösbar.
  % Vorher sind die KS schon so definiert, dass die z-Achse im Basis-KS
  % nach oben zeigt. Jetzt zeigt sie im Basis-KS nach unten.
  Structure.R_N_E = Structure.R_N_E*rotx(pi); % wird damit in Opt. der EE-Rotation berücksichtigt
  R.update_EE([], r2eulxyz(Structure.R_N_E));
elseif strcmp(mounting, 'wall')
  % Roboter zeigt zur Seite (y-Achse). Bei 2T1R also Bewegung in xz-Ebene.
  R.update_base([], [-pi/2;0;0]);
else
  error('Fall %s noch nicht implementiert', mounting);
end
R.update_gravity([0;0;-9.81]); % Gravitation wird für Dynamik im Basis-KS definiert.
if any(any(abs(Structure.R_N_E-eye(3)) > 1e-10))
  Structure.R_N_E_isset = true;
else
  Structure.R_N_E_isset = false;
end
% Merke diese ursprünglich gesetzte Basis-Rotation. Weitere Rotation kann
% dazukommen, falls in Optimierung gewünscht.
Structure.R_W_0 = R.T_W_0(1:3,1:3);
if any(any(abs(Structure.R_W_0-eye(3)) > 1e-10))
  Structure.R_W_0_isset = true;
else
  Structure.R_W_0_isset = false;
end

% Falls planerer Roboter: Definiere Verschiebung, damit der Roboter von
% oben angreift. Sieht besser aus, macht die Optimierung aber schwieriger.
% if all(Set.task.DoF(1:3) == [1 1 0])
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
if ~isempty(Structure.RobName)
  % Nichts machen. Es ist ein Roboter, kein allgemeines Modell zur
  % Optimierung vorgegeben. Die DH-Parameter sind also fix.
  if Structure.Type == 0 % Seriell
    Ipkinrel = false(size(R.pkin));
  else  % Parallel
    Ipkinrel = false(size(R.Leg(1).pkin));
  end
elseif Structure.Type == 0 || Structure.Type == 2
  if Structure.Type == 0 % Seriell
    R_pkin = R;
  else  % Parallel
    R_pkin = R.Leg(1);
  end
  % Nummern zur Indizierung der pkin, siehe SerRob/get_pkin_parameter_type
  Ipkinrel = R_pkin.get_relevant_pkin(Set.task.DoF);
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
  % Setze den a-Parameter für Schubylinder zu Null. Ist technisch
  % sinnvoller. Sonst wäre der Schubzylinder mit einem Hebel zum vorherigen
  % oder nächsten Gelenk angebracht. Widerspricht dem Konzept der Stabkinematik
  % Hierdurch wird die Vielfalt möglicher Roboter zugunsten einer besseren
  % Plausibilität der Ergebnisse eingeschränkt. TODO: Zurücknehmen, sobald
  % Kollisionskörper für Führungsschiene/Zylinder besser implementiert sind
  I_cyl = find(R_pkin.MDH.sigma == 1 & R_init.DesPar.joint_type == 5);
  % Nur bei räumlichen Systemen mit Drehung aus der Ebene heraus machen.
  % Betrifft die serielle Kette und nicht die Aufgabe (im Fall von PKM)
  if all(R_pkin.I_EE(4:5)==0), I_cyl = []; end
  for ii_cyl = I_cyl' % Alle Gelenke mit Schubzylinder durchgehen (falls mehrere)
    % Setze den a-Parameter vor einem Schubylinder zu Null. Dadurch drückt
    % der Zylinder direkt auf das vorhergehende Gelenk
    I_aprecyl = R_pkin.pkin_jointnumber==ii_cyl & R_pkin.pkin_types == 4;
    Ipkinrel = Ipkinrel & ~I_aprecyl; % Nehme die "1" bei dem a-Parameter weg
    % Setze den d- und a-Parameter nach einem Schubylinder zu Null. Ist technisch
    % sinnvoller. Dadurch drückt der Zylinder direkt auf das folgende Gelenk
    I_apostcyl = R_pkin.pkin_jointnumber==(ii_cyl+1) & R_pkin.pkin_types == 4;
    Ipkinrel = Ipkinrel & ~I_apostcyl; % Nehme die "1" bei dem a-Parameter weg
    I_dpostcyl = R_pkin.pkin_jointnumber==(ii_cyl+1) & R_pkin.pkin_types == 6;
    Ipkinrel = Ipkinrel & ~I_dpostcyl; % Nehme die "1" bei dem d-Parameter weg
  end
  
  % Deaktiviert:
  % Setze den letzten d-Parameter für PKM-Beinketten auf Null. Dieser ist
  % kinematisch redundant zur Plattform-Größe. Für die Auslegung spielt der
  % Parameter allerdings trotzdem eine Rolle (Kollision, Masse Plattform).
%   if Structure.Type == 2 % PKM
%     I_lastdpar = ((R_pkin.pkin_jointnumber==R_pkin.NJ) & (R_pkin.pkin_types==6));
%     Ipkinrel = Ipkinrel & ~I_lastdpar; % Nehme die "1" bei d6 weg.
%   end

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
    R.update_mdh_legs(pkin_init);
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
  I_DoF_basepos = Set.task.DoF(1:3); % nur die FG der Aufgabe nehmen
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
    elseif strcmp(mounting, 'wall') % Annahme: Basis über der Aufgabe, Roboter arbeitet von der Wand weg
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
    if Set.task.DoF(3) == 1 % nicht für 2T1R
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
    if Set.task.DoF(3) == 1 % nicht für 2T1R
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
% Für planare Aufgabe die Z-Position belegen (nicht durch Roboter änderbar)
if all(Set.task.DoF(1:5) == [1 1 0 0 0]) % egal ob 2T0R, 2T0*R oder 2T1R
  r_W_0(3) = Traj.XE(1,3);
end
R.update_base(r_W_0);
% EE-Verschiebung
if all(~isnan(Set.optimization.ee_translation_fixed))
  % EE-Verschiebung wird in Einstellung vorgegeben. Nicht optimieren
  R.update_EE(Set.optimization.ee_translation_fixed(:));
elseif Set.optimization.ee_translation && ...
    (Structure.Type == 0 || Structure.Type == 2 && ~Set.optimization.ee_translation_only_serial)
  % (bei PKM keine EE-Verschiebung durchführen. Dort soll das EE-KS bei
  % gesetzter Option immer in der Mitte sein)
  nvars = nvars + sum(Set.task.DoF(1:3)); % Verschiebung des EE um translatorische FG der Aufgabe
  vartypes = [vartypes; 3*ones(sum(Set.task.DoF(1:3)),1)];
  varlim = [varlim; repmat([-1, 1], sum(Set.task.DoF(1:3)), 1)]; % bezogen auf Lref
  % Bei planaren seriellen Robotern muss eine Rotation durchgeführt werden,
  % falls es eine Transformation N-E gibt. Sonst wird die falsche Richtung
  % des N-KS benutzt statt wie gewünscht des E-KS.
  if Structure.R_N_E_isset && R.Type == 0
    task_transl_DoF_rotE = R.T_N_E(1:3,1:3)' * double(Set.task.DoF(1:3)');
  else
    task_transl_DoF_rotE = double(Set.task.DoF(1:3)');
  end
  for i = find(abs(task_transl_DoF_rotE(:))>1e-10)'
    varnames = {varnames{:}, sprintf('ee pos %s', char(119+i))}; %#ok<CCAT>
  end
end

% EE-Rotation
if all(~isnan(Set.optimization.ee_rotation_fixed))
  % EE-Rotation wird in Einstellung vorgegeben. Nicht optimieren. Boden- 
  % oder Deckenmontage über zusätzliche Rotation berücksichtigen (s.o.)
  R_N_E_fix = Structure.R_N_E * eulxyz2r(Set.optimization.ee_rotation_fixed(:));
  R.update_EE([], r2eulxyz(R_N_E_fix));
elseif Set.optimization.ee_rotation
  % Gehe die verschiedenen Fälle von EE-FG durch. Falls einzelne
  % Euler-Winkel fix vorgegeben werden, werden diese nicht optimiert
  if sum(Set.task.DoF(4:6)) == 1 % 2T1R oder 3T1R
    neerot = 1;
    assert(neerot~=0, ['Bei 2T1R/3T1R und ee_rotation=true darf z-Winkel ', ...
      'nicht vorgegeben werden']);
  elseif sum(Set.task.DoF(4:6)) == 0 % 3T0R
    neerot = 0;
  elseif sum(Set.task.DoF(4:6)) == 2 % 3T2R
    % Bei 3T2R wird die Rotation um die Werkzeugachse nicht optimiert.
    neerot = 2 - sum(~isnan(Set.optimization.ee_rotation_fixed(1:2)));
    assert(neerot~=0, ['Bei 3T2R und ee_rotation=true dürfen nicht x- ', ...
      'und y-Winkel vorgegeben werden']);
  else % 3T3R
    neerot = 3 - sum(~isnan(Set.optimization.ee_rotation_fixed(1:3)));
    assert(neerot~=0, ['Bei 3T3R und ee_rotation=true dürfen nicht x- ', ...
      'y- und z-Winkel vorgegeben werden']);
  end
  nvars = nvars + neerot; % Verdrehung des EE um rotatorische FG der Aufgabe
  vartypes = [vartypes; 4*ones(neerot,1)];
  varlim = [varlim; repmat([0, pi], neerot, 1)];
  for i = find(Set.task.DoF(4:6))
    if ~isnan(Set.optimization.ee_rotation_fixed(i)), continue; end
    varnames = [varnames(:)', {sprintf('ee rot %d', i)}];
  end
end

% Gestell-Rotation: Besonders für PKM relevant. Für Serielle Roboter mit
% erstem Drehgelenk in z-Richtung irrelevant.
if Set.optimization.rotate_base && ...
    ~(Structure.Type == 0 && R.MDH.sigma(1)==0 && R.MDH.alpha(1)==0 && ...
    ... % Wenn Gelenkgrenzen fix sind, auch Drehung des Roboters ...
      ~Set.optimization.fix_joint_limits) % statt erstem Gelenk sinnvoll
  nvars = nvars + 1;
  vartypes = [vartypes; 5];
  varlim = [varlim; repmat([-pi, pi], 1, 1)];
  varnames = [varnames(:)', {'baserotation z'}];
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
    % Grenzfall: Koppelgelenke benachbarter Paare fallen zusammen. Dann
    % klassische Hexapod-Koppelgelenke aus Literatur
    % Geometrische Berechnung des Grenzfalls für l/r: `(cos(4*pi/3)-1) / sin(4*pi/3)`
    % Dabei ist `l` der halbe Paar-Abstand
    varlim = [varlim; [0.2,1.7321*2]]; % Gelenkpaarabstand. Relativ zu Gestell-Radius.
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
  if any(R.DesPar.platform_method == [1:3,7]) % keine Parameter bei Kreis
  elseif any(R.DesPar.platform_method == 4:6) % Parameter ist Gelenkpaarabstand (6FG-PKM)
    nvars = nvars + 1;
    vartypes = [vartypes; 9];
    varlim = [varlim; [0.2,1.7321*2]]; % Gelenkpaarabstand. Relativ zu Plattform-Radius. Grenzfall, siehe oben für Gestell.
    varnames = {varnames{:}, 'platform_morph_pairdist'}; %#ok<CCAT>
  elseif R.DesPar.platform_method == 8
    nvars = nvars + 1;
    vartypes = [vartypes; 9];
    varlim = [varlim; [-pi,pi]]; % Offset für Gelenkrichtung auf Plattform
    varnames = {varnames{:}, 'platform_morph_axoffset'}; %#ok<CCAT>
  else
    error('Parameter "platform_morphology" für Plattform-Methode %d nicht implementiert', R.DesPar.platform_method);
  end
end
% Variablen-Typen speichern
if nvars == 1 && strcmp(varnames{1}, 'scale')
  cds_log(1, sprintf(['[dimsynth] Es gibt nur einen einzigen Optimierungs', ...
    'parameter %s. Voraussichtlich keine Optimierung sinnvoll.'], varnames{1}));
end
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
% Index für Schubzylinder die direkt durch das vorhergehende Gelenk gehen.
% Kann benutzt werden, um die Länge der Zylinder zu begrenzen.
I_cylinder = false(R.NJ,1); % Indizes der Zylinder-Gelenke (Schubgelenk)
I_adzero = false(R.NJ,1); % Index für a/d Null. Dann Zylinder durch vorheriges Gelenk
if R.Type == 0 % Seriell
  I_adzero = (R.MDH.a==0 & R.MDH.d==0);
  I_cylinder = (R.DesPar.joint_type==5);
else % Parallel
  for i = 1:R.NLEG
    I_adzero(R.I1J_LEG(i):R.I2J_LEG(i)) = ...
      (R.Leg(i).MDH.a==0 & R.Leg(i).MDH.d==0);
    I_cylinder(R.I1J_LEG(i):R.I2J_LEG(i)) = ...
      (R.Leg(i).DesPar.joint_type==5);
  end
end
Structure.I_straightcylinder = I_cylinder & I_adzero;
%% Initialisierung der Kollisionsprüfung
if Set.optimization.constraint_collisions || ~isempty(Set.task.obstacles.type) || ...
    ~isempty(Set.task.installspace.type) || ...
    ~isnan(Set.optimization.base_size_limits(2)) && any(Structure.I_firstprismatic)
  % Lege die Starrkörper-Indizes fest, für die Kollisionen geprüft werden
  selfcollchecks_bodies = uint8(zeros(0,2));
  % Prüfe Selbstkollisionen einer kinematischen Kette.
  for k = 1:NLEG
    % Erneute Initialisierung (sonst eventuell doppelte Eintragungen)
    collbodies = struct('link', uint8([]), 'type', uint8(zeros(0,2)), ...
      'params', zeros(0,10)); % Liste für Beinkette
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
        collbodies.type = [collbodies.type; uint8(13)];
        % Umrechnung der Parameter ins Welt-KS: Notwendig. Aber hier
        % ignoriert.
      elseif i < R_cc.NJ
        if ~(R_cc.MDH.a(i+1)==0 && R_cc.MDH.d(i+1)==0 && R_cc.DesPar.joint_type(i)==5)
          % Bei Schubzylinder ohne a-/d-Versatz passen die Kollisionskörper
          cds_log(-1, sprintf(['[dimsynth] Die Kollisionsprüfung für die ', ...
            'Führungsschiene des P-Gelenks an Stelle %d ist nicht definiert. ', ...
            'Wird vorerst ignoriert.'], i));
        end
        continue
        % Kapsel, zwei Punkte (im mitbewegten Körper-KS)
        collbodies.type = [collbodies.type; uint8(3)]; %#ok<UNRCH>
      else
        cds_log(-1, sprintf(['[dimsynth] Kollisionsprüfung für Schubgelenk ', ...
          'als letztes Gelenk der Kette noch nicht implementiert']));
        continue; % TODO: Prüfen, ob type=6 hier funktionieren würde
      end
      collbodies.params = [collbodies.params; cbi_par, NaN(1,3)];
      % Führungsschiene/Führungszylinder ist vorherigem Segment zugeordnet
      collbodies.link = [collbodies.link; [uint8(i-1), uint8(i-1)]];
    end
    
    % Erzeuge Ersatzkörper für die kinematische Kette (aus Gelenk-Trafo)
    for i = 1:R_cc.NJ
      if ~isempty(Structure.RobName) && norm([R_cc.MDH.a(i);R_cc.MDH.d(i)]) < 20e-3
        % Sonderfall: Roboter mit festen Parametern wird optimiert und
        % Abstand zwischen Segmenten ist kleiner als Kollisionskörper.
        % Dann Körper weglassen (für Denso-Roboter)
        continue
      end
      if R_cc.MDH.a(i) ~= 0 || R_cc.MDH.d(i) ~= 0 || R_cc.MDH.sigma(i) == 1
        % Es gibt eine Verschiebung in der Koordinatentransformation i
        % Definiere einen Ersatzkörper dafür
        collbodies.link =   [collbodies.link; [uint8(i),uint8(i-1)]];
        collbodies.type =   [collbodies.type; uint8(6)]; % Kapsel, direkte Verbindung
        % Wähle Kapseln mit Radius 20mm. R.DesPar.seg_par ist noch nicht belegt
        % (passiert erst in Entwurfsoptimierung).
        collbodies.params = [collbodies.params; 20e-3, NaN(1,9)];
      end
    end
    % Trage eine EE-Transformation ein: Kapsel von letzten Roboter-Segment
    % zu TCP. Direkte Verbindung.
    if Structure.Type == 0 && Set.optimization.ee_translation
      collbodies.link =   [collbodies.link; [uint8(R_cc.NJ+1),uint8(R_cc.NJ)]];
      collbodies.type =   [collbodies.type; uint8(6)]; % Kapsel, direkte Verbindung
      collbodies.params = [collbodies.params; 10e-3, NaN(1,9)]; % Radius 10mm
    end
    R_cc.collbodies = collbodies;
    % Trage die Kollisionsprüfungen ein
    for i = 3:R_cc.NJ+1 % für letztes Segment und zusätzlich für virt. EE-Segment
      % Füge Prüfung mit allen vorherigen hinzu (außer direktem Vorgänger)
      % Diese Kollision wird nicht geprüft, da dort keine Kollision
      % stattfinden können sollte (direkt gegeneinander drehbare Teile
      % können konstruktiv kollisionsfrei gestaltet werden).
      % Der Vorgänger bezieht sich auf den Kollisionskörper, nicht auf
      % die Nummer des Starrkörpers. Ansonsten würden zwei durch Kugel-
      % oder Kardan-Gelenk verbundene Körper in Kollision stehen.
      j_hascollbody = collbodies.link(collbodies.link(:,1)<i,1)';
      % Sonderfall Portal-System: Abstand zwischen Kollisionskörpern noch
      % um eins vergrößern. TODO: Ist so noch nicht allgemeingültig.
      % Dadurch wird die Kollisionsprüfung effektiv deaktiviert.
      if i<=R_cc.NJ && sum(R_cc.MDH.sigma(i-2:i) == 1) == 3
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
        % fprintf('Kollisionsprüfung (%d): Beinkette %d Seg. %d vs Seg. %d. Zeile [%d,%d]\n', ...
        %   size(selfcollchecks_bodies,1), k, i, j, selfcollchecks_bodies(end,1), selfcollchecks_bodies(end,2));
      end
    end % i-loop (NJ)
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
  % zusätzliche Vorgänger-Index für EE-TCP zu EE-Segment
  v = [v; max(v)+1];
  Structure.MDH_ante_collcheck = v; % wird nicht mehr benötigt.
  
  % Roboter-Kollisionsobjekte in Struktur abspeichern (zum Abruf in den
  % Funktionen cds_constr_collisions_... und cds_constr_installspace
  % Ist erstmal nur Platzhalter. Wird zur Laufzeit noch aktualisiert.
  Structure.collbodies_robot = cds_update_collbodies(R, Set, Structure, Structure.qlim', true);
  % Probe: Sind Daten konsistent? Inkonsistenz durch obigen Aufruf möglich.
  assert(size(Structure.collbodies_robot.params,1)==length(Structure.collbodies_robot.type), ...
    'Felder params und type haben keine konsistente Dimension in Structure.collbodies_robot.');
  assert(size(Structure.collbodies_robot.link,1)==length(Structure.collbodies_robot.type), ...
    'Felder params und type haben keine konsistente Dimension in Structure.collbodies_robot.');
  if any(any(~isnan(Structure.collbodies_robot.params(Structure.collbodies_robot.type==6,2:end))))
    save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_robot_cb_error.mat'));
    error('Inkonsistente Kollisionsdaten: Kapsel-Direktverbindung hat zu viele Parameter');
  end
  if size(Structure.collbodies_robot.link,2)~=2
    error('Structure.collbodies_robot.link muss 2 Spalten haben');
  end
  
  % Starrkörper-Kollisionsprüfung für PKM erweitern
  if Structure.Type == 2  % PKM
    % Prüfe Kollisionen aller Beinsegmente mit allen anderen Beinketten
    for k = 1:NLEG % Index erste Beinkette für Koll.-Prüfung
      if k > 1, NLoffset_k = 1+R.I2L_LEG(k-1)-(k-1);
      else,     NLoffset_k = 1; end
      for i = [1:(k-1), (k+1):NLEG] % Index zweite Beinkette für Koll.-Prüfung
        if i > k, continue; end % Nur in eine Richtung prüfen, sonst doppelt
        % Offset der Beinketten-Körper in den PKM-Körpern
        if i > 1, NLoffset_i = 1+R.I2L_LEG(i-1)-(i-1);
        else,     NLoffset_i = 1; end
        % Alle Segmente von Beinkette k können mit allen von Kette i
        % kollidieren. Nehme die vorgesehenen Kollisionssegmente und ein
        % letztes Segment für die Plattform-Körper. Überzählige Prüfungen
        % werden weiter unten wieder entfernt
        for cb_k = unique([R.Leg(k).collbodies.link(:,1)',R.Leg(k).NL-1])
          for cb_i = unique([R.Leg(i).collbodies.link(:,1)',R.Leg(i).NL-1])
            % Mögliche Ausnahmen hier definieren per `continue`:
            % [Deaktiviert] Bei paarweiser Anordnung der Beinketten (zur Plattform)
            % wird das letzte Segment der Beinketten-Paare nicht geprüft.
            % Dadurch können konstruktive Anpassungen berücksichtigt werden
            if any(R.DesPar.platform_method == [4 5 6]) && ...% paarweise Anordnung
                any(all(repmat(sort([k,i]),3,1)==[1 2; 3 4; 5 6],2)) && ... % betrifft Paar-Kombination
                cb_k == R.Leg(k).collbodies.link(end,1) && cb_i == R.Leg(i).collbodies.link(end,1) % letztes Segment
              % Ausnahme deaktiviert, erzeugt zu viele sichtbare Kollisionen
              % continue % Nicht prüfen
            end
            % Bei paarweiser Anordnung des Gestells sind die Drehachsen
            % der Beinketten-Paare immer parallel. Daher kann sich der
            % Kollisionsabstand der ersten Segmente nicht ändern
            if any(R.DesPar.base_method == [4 5 6]) && ...% paarweise Anordnung
                R.Leg(k).MDH.sigma(1) == 0 && R.Leg(i).MDH.sigma(1) == 0 && ... % Beides Drehachsen
                any(all(repmat(sort([k,i]),3,1)==[1 2; 3 4; 5 6],2)) && ... % betrifft Paar-Kombination
                cb_k == R.Leg(k).collbodies.link(1,1) && cb_i == R.Leg(i).collbodies.link(1,1) % erstes Segment
              continue
            end
            row_ki = uint8([NLoffset_k+cb_k, NLoffset_i+cb_i]);
            if any(all(selfcollchecks_bodies==...
                repmat(row_ki,size(selfcollchecks_bodies,1),1),2))
              % Eintrag ist schon vorhanden. Betrifft voraussichtlich
              % Kollisionsprüfungen der Beinkette mit der eigenen Basis
              continue
            end
            % Eintragen
            selfcollchecks_bodies = [selfcollchecks_bodies; row_ki]; %#ok<AGROW>
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
      % Prüfe keine Kollision des ersten Körpers, da dieser
      % Körper direkt nach dem Gestell kommt. Annahme. Kollision
      % konstruktiv vermeidbar
      for cb_k = R.Leg(k).collbodies.link(2:end,1)'
        if isempty(cb_k), continue; end % passiert, falls nur ein Körper in Beinkette
        % Kollision mit Kollisionskörpern fest bezüglich PKM-Basis
        selfcollchecks_bodies = [selfcollchecks_bodies; ...
          uint8([NLoffset_k+cb_k, 0])]; %#ok<AGROW>
        % fprintf(['Kollisionsprüfung (%d): Bein %d Seg. %d vs Gestell. ', ...
        %   'Zeile [%d,%d]\n'], size(selfcollchecks_bodies,1), k, cb_k, ...
        %   selfcollchecks_bodies(end,1), selfcollchecks_bodies(end,2));
        % Prüfe Kollision mit gestellfesten Körpern, die dem
        % Basis-KS von anderen Beinketten zugeordnet sind. Prüfe nicht die
        % Körper dieser Beinkette selbst, da dies schon weiter oben
        % geschehen ist. Gestell-Körper, die den Basis-KS von zwei Bein-
        % ketten zugeordnet sind, werden weiterhin geprüft.
        for j = [1:(k-1), (k+1):NLEG] % gehe nur andere Beinketten durch
          if j > 1, NLoffset_j = 1+R.I2L_LEG(j-1)-(j-1);
          else,     NLoffset_j = 1; end
          row_kj = uint8([NLoffset_k+cb_k, NLoffset_j+0]);
          if any(all(selfcollchecks_bodies==...
              repmat(row_kj,size(selfcollchecks_bodies,1),1),2))
            % Eintrag ist schon vorhanden. Liegt voraussichtlich daran,
            % dass gestellfeste Kollisionskörper bereits den Beinketten
            % zugeordnet waren und nicht der PKM (bei Führungsschienen)
            continue
          end
          selfcollchecks_bodies = [selfcollchecks_bodies; row_kj]; %#ok<AGROW>
          % fprintf(['Kollisionsprüfung (%d): Bein %d Seg. %d vs Gestell-Körper ', ...
          %   'Bein %d. Zeile [%d,%d]\n'], size(selfcollchecks_bodies,1), k, cb_k, ...
          %   j, selfcollchecks_bodies(end,1), selfcollchecks_bodies(end,2));
        end
      end
    end
    % Kollision von Plattform und Gestell
    for j = 1:NLEG % Über Beinketten bzgl. Gestell
      j_base = R.I1L_LEG(j)-(j-1);
      for k = 1:NLEG % Über Beinketten bzgl. Plattform
        k_plf = R.I2L_LEG(k)-(k-1)-1;
        row_kj = uint8([k_plf, j_base]);
        if any(all(selfcollchecks_bodies==...
            repmat(row_kj,size(selfcollchecks_bodies,1),1),2))
          continue % Eintrag ist schon vorhanden. Siehe oben
        end
        selfcollchecks_bodies = [selfcollchecks_bodies; row_kj]; %#ok<AGROW>
        % fprintf(['Kollisionsprüfung (%d): Plattform-Körper zugeordnet zu ', ...
        %   'Bein %d vs Gestell-Körper zugeordnet zu Bein %d. ', ...
        %   'Zeile [%d,%d]\n'], size(selfcollchecks_bodies,1), k, j, ...
        %   selfcollchecks_bodies(end,1), selfcollchecks_bodies(end,2));
      end
      % Plattform-Modell zugeordnet zum Plattform-KS (Stern oder Kugel)
      if ~any(all(selfcollchecks_bodies==...
          repmat([j_base, R.NL+R.NLEG-1],size(selfcollchecks_bodies,1),1),2))
        selfcollchecks_bodies = [selfcollchecks_bodies; ...
          uint8( [j_base, R.NL+R.NLEG-1] )]; %#ok<AGROW>
      end
      % Kollisionsprüfung für EE-Segment zu EE-TCP
      if ~any(all(selfcollchecks_bodies==...
          repmat([j_base, R.NL+R.NLEG],size(selfcollchecks_bodies,1),1),2))
        selfcollchecks_bodies = [selfcollchecks_bodies; ...
          uint8( [j_base, R.NL+R.NLEG] )]; %#ok<AGROW>
      end
    end
    % Plattform und TCP gegen Roboter-Basis
    if ~any(all(selfcollchecks_bodies==...
        repmat([0, R.NL+R.NLEG-1],size(selfcollchecks_bodies,1),1),2))
      selfcollchecks_bodies = [selfcollchecks_bodies; ...
        uint8( [0, R.NL+R.NLEG-1] )];
    end
    % Kollisionsprüfung für EE-Segment zu EE-TCP
    if ~any(all(selfcollchecks_bodies==...
        repmat([0, R.NL+R.NLEG],size(selfcollchecks_bodies,1),1),2))
      selfcollchecks_bodies = [selfcollchecks_bodies; ...
        uint8( [0, R.NL+R.NLEG] )];
    end
    % Kollisionen mit der Plattform (beinhaltet teilweise auch die
    % Gestell-Körper, daher Prüfen auf Duplikat vor Hinzufügen).
    for k = 1:NLEG
      if k > 1, NLoffset_k = 1+R.I2L_LEG(k-1)-(k-1);
      else,     NLoffset_k = 1; end
      % Nehme alle Kollisionskörper der Beinketten. Bei paralleler Bewegung
      % der Beinketten auslassen des letzten. Annahme: Konstruktive
      % Vermeidung möglich. Sonst zu viele Kollisionen.
      Ilast = size(R.Leg(k).collbodies.link,1);
      if any(R.DesPar.platform_method == [1 4 7])
        Ilast = Ilast-1;
      end
      for cb_k = R.Leg(k).collbodies.link(1:Ilast,1)'
        if ~any(all(selfcollchecks_bodies==...
            repmat([NLoffset_k+cb_k, R.NL+R.NLEG-1],size(selfcollchecks_bodies,1),1),2))
          selfcollchecks_bodies = [selfcollchecks_bodies; ...
            uint8( [NLoffset_k+cb_k, R.NL+R.NLEG-1] )]; %#ok<AGROW>
          % fprintf(['Kollisionsprüfung (%d): Bein %d Seg. %d vs Plattform. ', ...
          %   'Zeile [%d,%d]\n'], size(selfcollchecks_bodies,1), k, cb_k, ...
          %   selfcollchecks_bodies(end,1), selfcollchecks_bodies(end,2));
        end
        % Kollisionsprüfung für EE-Segment zu EE-TCP
        if ~any(all(selfcollchecks_bodies==...
            repmat([NLoffset_k+cb_k, R.NL+R.NLEG],size(selfcollchecks_bodies,1),1),2))
          selfcollchecks_bodies = [selfcollchecks_bodies; ...
            uint8( [NLoffset_k+cb_k, R.NL+R.NLEG] )]; %#ok<AGROW>
          % fprintf(['Kollisionsprüfung (%d): Bein %d Seg. %d vs TCP. ', ...
          %   'Zeile [%d,%d]\n'], size(selfcollchecks_bodies,1), k, cb_k, ...
          %   selfcollchecks_bodies(end,1), selfcollchecks_bodies(end,2));
        end
      end
    end
  end
  % Debug: Namen der Körper für die Kollisionsprüfung anzeigen
  % (unterscheidet sich von den Kollisionskörpern, den Körpern zugeordnet)
  if Structure.Type == 0
    names_bodies = cell(R.NL+1,1);
    names_bodies{1} = 'Base';
    for k = 2:R.NL
      names_bodies{k} = sprintf('Link %d', k-1);
    end
  else % PKM
    names_bodies = cell(R.I2L_LEG(end)-R.NLEG+1+1+1,1);
    names_bodies{1} = 'Base';
    i = 1;
    for k = 1:NLEG
      i = i + 1;
      names_bodies{i} = sprintf('Leg %d Base', k);
      for j = 1:R.Leg(1).NL-2
        i = i + 1;
        names_bodies{i} = sprintf('Leg %d Link %d', k, j);
      end
      i = i + 1;
      names_bodies{i} = sprintf('Leg %d Link %d (EE)', k, R.Leg(1).NL-1);
    end
    names_bodies{end-1} = 'Platform';
  end
  names_bodies{end} = 'TCP'; % Pseudo-Körper, damit Verbindung zum TCP mit einfacher Geometrie möglich ist.
  names_collbodies = cell(size(Structure.collbodies_robot.link,1),1);
  for i = 1:size(Structure.collbodies_robot.link,1)
    names_collbodies{i} = sprintf('%s + %s', names_bodies{1+Structure.collbodies_robot.link(i,1)}, ...
      names_bodies{1+Structure.collbodies_robot.link(i,2)});
  end
  if false
    fprintf('Liste der Körper:\n'); %#ok<UNRCH>
    for i = 1:length(names_bodies)
      fprintf('%d - %s\n', i-1, names_bodies{i});
    end
    % Debug: Liste der Kollisionskörper anzeigen
    fprintf('Liste der Kollisionskörper:\n');
    for i = 1:size(Structure.collbodies_robot.link,1)
      fprintf('%d - links %d+%d (%s) (Typ: %d)\n', i, Structure.collbodies_robot.link(i,1), ...
        Structure.collbodies_robot.link(i,2), names_collbodies{i}, Structure.collbodies_robot.type(i));
    end
    fprintf('Liste der zu prüfenden Körper:\n');
    for i = 1:size(selfcollchecks_bodies,1)
      fprintf('%03d - %02d vs %02d (%s vs %s)\n', i, selfcollchecks_bodies(i,1), ...
        selfcollchecks_bodies(i,2), names_bodies{1+selfcollchecks_bodies(i,1)}, ...
        names_bodies{1+selfcollchecks_bodies(i,2)});
    end
  end
  % Prüfe die Liste der Kollisionsprüfungen aus selfcollchecks_bodies
  selfcollchecks_bodies_sort = sortrows(selfcollchecks_bodies')';
  [selfcollchecks_bodies_unique, I_unique] = unique(selfcollchecks_bodies_sort, 'rows');
  if size(selfcollchecks_bodies_unique,1) ~= size(selfcollchecks_bodies_sort,1)
    I_nonunique = []; % Indizes der doppelten Kollisionsprüfungen
    for jj = 1:size(selfcollchecks_bodies_sort,1)
      if ~any(I_unique==jj), I_nonunique = [I_nonunique, jj]; end %#ok<AGROW>
    end
    I_exist = []; % Indizes der bereits vorhandenen zu den doppelten
    for kk = I_nonunique
      I_exist = [I_exist, find(all(selfcollchecks_bodies==repmat( ...
        selfcollchecks_bodies(kk,:),size(selfcollchecks_bodies,1),1),2))']; %#ok<AGROW>
    end
    error('In selfcollchecks_bodies sind doppelte Einträge. Doppelt: [%s], vorhanden: [%s]', ...
      disp_array(I_nonunique,'%d'), disp_array(setxor(I_nonunique,I_exist),'%d'));
  end
  % Prüfe die zusammengestellten Kollisionskörper. Die höchste Nummer
  % (Null-indiziert) ist durch die Roboterstruktur vorgegeben.
  if Structure.Type == 0
    Nmax = R.NL-1+1;
  else
    Nmax = R.I2L_LEG(end)-(R.NLEG-1)+1;
  end
  if any(Structure.collbodies_robot.link(:) > Nmax)
    error(['Ungültige Kollisionskörper. Maximaler Index %d in Structure.', ...
      'collbodies_robot.link überschritten'], Nmax);
  end
  assert(size(Structure.collbodies_robot.link,2)==2, ['collbodies_robot.link ', ...
    'muss 2 Spalten haben']);
  assert(size(Structure.collbodies_robot.params,2)==10, ['collbodies_robot.params ', ...
    'muss 10 Spalten haben']);
  assert(size(Structure.collbodies_robot.type,2)==1, ['collbodies_robot.type ', ...
    'muss 1 Spalte haben']);
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
  Structure.selfcollchecks_collbodies = uint8(zeros(0,2));
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
        % fprintf('Kollisionsprüfung für Körper %d: Koll.-körper %d (%s) vs Koll.-körper %d (%s).\n', ...
        %   size(Structure.selfcollchecks_collbodies,1), ii1, names_collbodies{ii1}, ii2, names_collbodies{ii2});
        if Structure.Type == 2
          % Indizes der Körper zum Gestell (bzw. Beinketten-Basis-KS)
          I_base = R.I1L_LEG(1:NLEG)-((1:NLEG)'-1);
          % Indizes der Körper zur Plattform (zugehörig zu letztem
          % Beinketten-KS)
          I_pl = R.I2L_LEG(1:NLEG)-((1:NLEG)'-1)-1;
          % Plattform-Körper zusätzlich einfügen (zu keiner Beinkette).
          I_pl = [I_pl; I_pl(end)+1; I_pl(end)+2]; %#ok<AGROW> 
          % Segment-Nummern der beteiligten Körper (jeder Kollisionskörper
          % hat zwei Roboter-Körper zugewiesen)
          ii_links = [Structure.collbodies_robot.link(ii1,:), ...
                      Structure.collbodies_robot.link(ii2,:)];
          % Ein Körper darf nicht zwei mal auftreten (bei beiden
          % Kollisionsparteien). Dann sind es direkt benachbarte Objekte.
          ii_links_test = ii_links;
          if ii_links_test(3) == ii_links_test(4)
            ii_links_test = ii_links_test(1:3); % letzter Eintrag trägt keine Information
          end
          if ii_links_test(1) == ii_links_test(2)
            ii_links_test = ii_links_test(2:end); % erster Eintrag trägt keine Information
          end
          if length(ii_links_test)>length(unique(ii_links_test))
            continue
          end
          % Bestimme die Nummern der zugehörigen Beinketten (NaN für Plf)
          ii_leg = NaN(1,4);
          for kkk = 1:4
            if ii_links(kkk) == 0, continue; end
            ii_leg(kkk) = find(ii_links(kkk) >= I_base, 1, 'last');
            if any(ii_links(kkk) == I_pl(end-1:end)), ii_leg(kkk) = NaN; end
          end
          
          % Finde heraus, ob einer der Kollisionskörper ein Gestellteil ist.
          % Basis-Körper haben entweder Zugehörigkeit zu zwei Beinketten-
          % Basis-KS (in I_base) oder einen Eintrag mit 0 (sternförmig für
          % PKM-Basis) oder zwei Einträge mit 0 (Kugeln um Gestellgelenk)
          % (siehe cds_update_collbodies zur Definition der Gestellteile).
          % Hier müssen die Führungsschienen von gestellfesten Schub-
          % gelenken noch der jeweiligen Beinketten-Basis zugeordnet sein
          c1_is_base = false;
          if (length(intersect(ii_links(1:2)', I_base)) == 2 || ... % beide Beinketten-Basis zugeordnet (Kreis)
              length(intersect(ii_links(1:2)', I_base)) == 1 && ... % einer zur PKM-Basis, einer zur Beinketten-Basis (Stern)
              any(ii_links(1:2)==0) || ...
              all(ii_links(1:2)==0) ) && ... % Beide zur PKM-Basis gezählt (Gestellgelenk-Kugel)
              ii_links(1) ~= ii_links(2)
            c1_is_base = true;
          end
          c2_is_base = false; % das gleiche nochmal
          if (length(intersect(ii_links(3:4)', I_base)) == 2 || ...
              length(intersect(ii_links(3:4)', I_base)) == 1 && ...
              any(ii_links(3:4)==0) || ...
              all(ii_links(3:4)==0) ) && ...
              ii_links(3) ~= ii_links(4)
            c2_is_base = true;
          end
          % Finde heraus, ob einer der Kollisionskörper ein Plattformteil ist
          % (siehe cds_update_collbodies zur Definition der Plattformteile)
          c1_is_platform = false;
          if length(intersect(ii_links(1:2)', I_pl)) == 2 && ...
              ii_links(1) ~= ii_links(2) || ... % Jeweils dem Ende der Beinketten zugeordnet
             length(intersect(ii_links(1:2)', I_pl)) == 1 && ...
              ii_links(1) == ii_links(2) % Plattform-Körper
            c1_is_platform = true;
          end
          c2_is_platform = false;
          if length(intersect(ii_links(3:4)', I_pl)) == 2 && ...
              ii_links(3) ~= ii_links(4) || ...
             length(intersect(ii_links(3:4)', I_pl)) == 1 && ...
              ii_links(3) == ii_links(4)
            c2_is_platform = true;
          end
          % Finde heraus, ob beide Körper zu der gleichen Beinkette gehören
          is_same_leg = ~isempty(intersect(ii_leg(1:2), ii_leg(3:4)));
          % Prüfe auf mögliche Ausschlüssgründe für die Kollisionsprüfung
          if c1_is_platform && c2_is_platform
            % Beide Körper sind Teil der Plattform, aber zwecks Implemen-
            % tierung verschiedenen Beinketten zugeordnet. Keine Kollisions-
            % prüfung notwendig.
            continue
          elseif c1_is_base && c2_is_base
            % Beide Körper sind Teil des Gestells. Wird weitestgehend
            % vorher ausgeschlossen, außer, wenn Führungsschienen von
            % Linearachsen betrachtet werden. Werden nicht geprüft.
            continue
          elseif c1_is_platform || c2_is_platform
            % Ein Körper ist Teil der Plattform, der andere aber nicht.
            % Zähle keine Kollisionen, wenn die Körper zu den gleichen
            % Beinketten gehören. Es wird dadurch nur die Kollision der
            % Beinkette mit nicht daran angrenzenden Kanten der Plattform
            % gebildet. Teilweise ist dann aber eine Kollision mit dem
            % Anfang der Beinkette und der eigenen Plattform-Seite möglich
            if is_same_leg
              continue
            end
          end
          % Prüfe, ob ein an das Gestell angrenzender Kollisionskörper mit dem
          % Gestell auf Kollision geprüft wird. Hier liegt immer eine
          % Überschneidung vor, daher keine Prüfung möglich. Wird durch
          % obige Initialisierung nicht ausgeschlossen, da Beinketten-Basis- 
          % Kollisionskörper mehreren Körpern zugeordnet sind. Es muss
          % geprüft werden, ob es der erste Kollisionskörper der Beinkette
          % ist - nicht der erste Körper (falls z.B. U-Gelenk zuerst kommt).
          for j = 1:R.Leg(1).NJ % Gehe alle Körper der Beinkette durch
            I_leglink_first = I_base + j; % Annahme: Symmetrische PKM
            % Prüfe, ob diesem Körper j ein Kollisionskörper zugeordnet ist
            if ~isempty(intersect(Structure.collbodies_robot.link(:), I_leglink_first))
              break; % Körper j ist ein KK zugeordnet
            end
          end
          if is_same_leg && (...
              c1_is_base && ... % Erster Körper ist Basis
              ~isempty(intersect(I_leglink_first, ii_links(3:4))) || ... % zweiter Körper ist erster Bein-Kollisionskörper
              c2_is_base && ... % Zweiter Körper ist Basis
              ~isempty(intersect(I_leglink_first, ii_links(1:2)))) % erster Körper ist erster Bein-Kollisionskörper
            continue
          end
        end
        kk = kk + 1;
        CheckCombinations(kk,:) = [ii1,ii2];
        % fprintf('Kollisionsprüfung (%d+x): Koll.-körper %d (%s) vs Koll.-körper %d (%s).\n', ...
        %   size(Structure.selfcollchecks_collbodies,1), ii1, names_collbodies{ii1}, ii2, names_collbodies{ii2});
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
  % Eintragen in Roboter-Klasse
  R.collchecks = Structure.selfcollchecks_collbodies;
  if ~isempty(Structure.selfcollchecks_collbodies)
    % Jetzt werden die Kollisionen nochmal darauf geprüft, ob einige Ab-
    % stände sich nie ändern. Diese müssen dann nicht mehr geprüft werden.
    % (Die Prüfungen werden aber trotzdem nicht entfernt, sondern nur gewarnt)
    % Siehe z.B. cds_obj_footprint.m, cds_obj_colldist.m
    % Mit bereits oben gesetzten Zufallswerten für Kinematikparameter und
    % Gelenkwinkel werden Kollisionsabstände bestimmt.
    % Nutze den vollständigen Winkelbereich. Sonst falsch-positiv.
    Q_test = -pi+2*pi*rand(100,R.NJ); JP_test = [];
    for jj = 1:size(Q_test,1)
      % Variieren der Kinematikparameter, damit nicht die oben zufällig
      % gewählten Parameter immer zu einer Kollision führen (bswp. durch
      % Kollision der letzten Beinkette mit einem Plattform-Kollisionskörper)
      p_rand = varlim(:,1)+rand(size(varlim,1),1).*(varlim(:,2)-varlim(:,1));
      cds_update_robot_parameters(R, Set, Structure, p_rand);
      % Trage neue Grenzen mit Null für die Gelenkkoordinaten ein, damit
      % Ersatzkörper für Schubgelenke in dieser Prüfung hier ignoriert werden
      qmin_rand = Structure.qlim(:,1)+rand(size(Structure.qlim,1),1).* ...
        diff(Structure.qlim')';
      qmin_rand(isnan(qmin_rand)) = Structure.qlim(isnan(qmin_rand),1);
      % Übernehme Werte der ersten Beinkette für alle anderen
      if R.Type == 2
        for k = 1:R.NLEG
          qmin_rand(R.I1J_LEG(k):R.I2J_LEG(k)) = qmin_rand(R.I1J_LEG(1):R.I2J_LEG(1));
        end
      end
      % minimal kurze Schubwege damit Kollisionskörper so klein wie möglich werden
      qmax_rand = qmin_rand + eps;
      qmax_rand(isinf(qmin_rand)) = +inf;
      % Aktualisiere die Kollisionskörper der Schubgelenke. Verschiebung der
      % Zuordnung von Kollisionskörpern von Beinketten-Basis zu PKM-Basis.
      % TODO: Das funktioniert noch nicht so gut. Bei einigen PKM gibt es
      % immer Kollisionen. Bei anderen nicht. Unklar, warum.
      cds_update_collbodies(R, Set, Structure, [qmin_rand,qmax_rand]', false);
      [~, JP_jj] = R.fkine_coll2(Q_test(jj,:)');
      JP_test = [JP_test; JP_jj(:)']; %#ok<AGROW>
    end
    [colldet_test, colldist_test] = check_collisionset_simplegeom_mex(R.collbodies, ...
      R.collchecks, JP_test, struct('collsearch', false));
    % Prüfe, welche Abstände sich bei diesen Zufallswerten nicht ändern.
    % Die kinematischen Zwangsbedingungen für PKM sind nicht berücksichtigt.
    % Es kann später also noch mehr Prüfungen geben, die gleich sind. Hier
    % sollte aber eigentlich keine Prüfung immer gleich bleiben.
    colldist_minmax = minmax2(colldist_test')';
    colldist_range = diff(colldist_minmax);
    I_ccnc = abs(colldist_range(:)) < 1e-10; % "ccnc": "collcheck nochange"
    % Schließe Kollisionen der Schubgelenk-Führungsschienen von der Prüfung
    % aus. Zuordnung von Zufallswerten oben funktioniert nicht gut (TODO).
    I_bb = contains(names_collbodies(R.collchecks(:,1)), 'Base') & ...
           contains(names_collbodies(R.collchecks(:,2)), 'Base');
    if any(I_ccnc & ~I_bb)
      cds_log(2, sprintf(['[dimsynth] Die Kollisionsabstände für %d ', ...
        'Prüfungen sind immer gleich.'], sum(I_ccnc)));
      for kkk = 1:2
        if kkk == 1
          I_kkk = I_ccnc;
          str_kkk = 'ohne';
        else
          I_kkk = ~I_ccnc;
          str_kkk = 'mit';
        end
        logstr=sprintf('%d/%d Kollisionsprüfungen %s Änderung der Abstände:\n', ...
          sum(I_kkk), length(I_kkk), str_kkk);
        for i = find(I_kkk(:))'
          logstr=[logstr,sprintf(['%03d (dist range. %1.1e: %1.2e...%1.2e): ', ...
            '[%02d %02d], "%30s" vs "%30s"\n'], i, colldist_range(i), ...
            colldist_minmax(1,i), colldist_minmax(2,i), R.collchecks(i,1), R.collchecks(i,2), ...
            names_collbodies{R.collchecks(i,1)}, names_collbodies{R.collchecks(i,2)})]; %#ok<AGROW> 
        end
        cds_log(3, logstr);
      end
    end
    % Prüfe, ob permanent unbeeinflussbare Kollisionen erkannt werden.
    % Schließe die Kollision mit der Führungsschiene aus, da die
    % Zufallsmenge der Kollisionen oben nicht gut gebildet wird (TODO).
    I_alwayscoll = all(colldet_test)' & I_ccnc & ~I_bb;
    if any(I_alwayscoll)
      logstr=sprintf('%d/%d Kollisionsprüfungen mit permanenter Kollision:\n', ...
        sum(I_alwayscoll), length(I_alwayscoll));
      for i = find(I_alwayscoll(:))'
        logstr=[logstr,sprintf('%03d: [%02d %02d], "%s" vs "%s"\n', ...
          i, R.collchecks(i,1), R.collchecks(i,2), ...
          names_collbodies{R.collchecks(i,1)}, names_collbodies{R.collchecks(i,2)})]; %#ok<AGROW> 
      end
      cds_log(-1, logstr);
      save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_robot_collcheck_error.mat'));
      % Debug: Unterschiedliche Fälle untersuchen
%       Set.general.plot_details_in_fitness = inf;
%       Traj_0 = cds_transform_traj(R, Traj);
%       for k = 1:3
%         cds_constr_collisions_self(R, Traj_0.X(k,:), Set, Structure, ...
%           JP_test(k,:), Q_test(k,:), [1 2]);
%         saveas(867, fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
%           'tmp', sprintf('cds_dimsynth_robot_collision_case_%d.fig', k)));
%       end
      error('Logik-Fehler bei Initialisierung der Kollisionsprüfungen.')
    end
    Structure.I_collcheck_nochange = I_ccnc;
    % Untersuche, welche Kollisionskörper gar nicht geprüft werden. Muss nicht
    % unbedingt ein Fehler sein, deutet aber auf nicht berücksichtigten Fall
    for i = 1:length(R.collbodies.type)
      if ~any(R.collchecks(:) == i)
        cds_log(-1, sprintf(['[dimsynth] Kollisionskörper %d (%s) wird ', ...
          'nie geprüft.'], i, names_collbodies{i}));
      end
    end
    
    assert(max(R.collchecks(:))<=size(R.collbodies.link,1), ['Matlab-Klasse ', ...
      'muss gleiche Anzahl in collchecks und collbodies haben']);
    % Prüfe, ob die Typen der zu prüfenden Körper übereinstimmen. Wenn die
    % Reihenfolge durcheinanderkommt und verschiedene Körpertypen benutzt
    % werden, führt das hier zum Fehler.
    colltypes_check_class = [R.collbodies.type(R.collchecks(:,1)), ...
       R.collbodies.type(R.collchecks(:,2))];
    colltypes_check_struct = [...
       Structure.collbodies_robot.type(Structure.selfcollchecks_collbodies(:,1)), ...
       Structure.collbodies_robot.type(Structure.selfcollchecks_collbodies(:,2))];
    colltypes_check_struct(colltypes_check_struct(:)==13)=3; % Gleichwertige Definitionen (Kapsel)
    colltypes_check_class (colltypes_check_class (:)==13)=3;
    assert(all(colltypes_check_class(:)==colltypes_check_struct(:)), ...
      'Typen der Objekte zu den Kollisionsprüfungen nicht konsistent nach Matlab-Klasse');
    assert(all(Structure.selfcollchecks_collbodies(:,1)~=Structure.selfcollchecks_collbodies(:,2)), ...
      'Prüfung eines Kollisionskörpers mit sich selbst ergibt keinen Sinn');
    assert(max(Structure.selfcollchecks_collbodies(:))<=size(Structure.collbodies_robot.type,1), ...
      'collchecks indiziert mehr Kollisionskörper, als vorhanden')
    for i = 1:size(Structure.selfcollchecks_collbodies,1)
      collbodies_i = Structure.selfcollchecks_collbodies(i,:);
      bodies_i = [Structure.collbodies_robot.link(collbodies_i(1),:), ...
        Structure.collbodies_robot.link(collbodies_i(2),:)];
      if bodies_i(3) == bodies_i(4)
        bodies_i = bodies_i(1:3); % letzter Eintrag trägt keine Information
      end
      if bodies_i(1) == bodies_i(2)
        bodies_i = bodies_i(2:end); % erster Eintrag trägt keine Information
      end
      assert(length(bodies_i)==length(unique(bodies_i)), sprintf(['Ein Körper ist ', ...
        'mehrfach an Kollisionsprüfung %d beteiligt. Nicht sinnvoll.'], i));
    end
  end
  % Debug: Liste der Kollisionsprüfungen anzeigen
  if false
    fprintf('Liste der Kollisionsprüfungen (der Kollisionskörper):\n');   %#ok<UNRCH>
    for i = 1:size(Structure.selfcollchecks_collbodies,1)
      collbodies_i = Structure.selfcollchecks_collbodies(i,:);
      bodies_i = [Structure.collbodies_robot.link(collbodies_i(1),:), ...
        Structure.collbodies_robot.link(collbodies_i(2),:)];
      fprintf('%03d - collbodies %02d vs %02d (links %02d vs %02d; type %02d vs %02d; "%s" vs "%s")\n', i, ...
        Structure.selfcollchecks_collbodies(i,1), Structure.selfcollchecks_collbodies(i,2), ...
        Structure.collbodies_robot.link(collbodies_i(1),1), ...
        Structure.collbodies_robot.link(collbodies_i(2),1), ...
        Structure.collbodies_robot.type(collbodies_i(1)), ...
        Structure.collbodies_robot.type(collbodies_i(2)), ...
        names_collbodies{collbodies_i(1)}, ...
        names_collbodies{collbodies_i(2)});
    end
  end
  if isempty(Structure.selfcollchecks_collbodies)
    cds_log(-1, sprintf(['[dimsynth] Es sind keine Prüfungen von Kollisions', ...
      'körpern vorgesehen, obwohl verlangt. Liegt an Roboter-Kinematik.']));
    % Deaktiviere die Kollisionsprüfungen wieder
    Set.optimization.constraint_collisions = false;
    Set.task.obstacles.type = [];
  elseif any(Structure.selfcollchecks_collbodies(:,1)==Structure.selfcollchecks_collbodies(:,2))
    error('Prüfung eines Körpers mit sich selbst ergibt keinen Sinn');
  end
  % Bestimme Indizes für die Kollisionsprüfungen mit bestimmten Eigen-
  % schaften. Dadurch kann die Anzahl der Prüfungen in bestimmten Fällen
  % reduziert werden
  % Kollisionsprüfungen mit gestellfesten Führungsschienen.
  % Siehe cds_desopt_prismaticoffset.m
  % Stelle Körper-Nummern dazu fest (s.o.)
  if R.Type == 0 % Seriell
    Ib_baserail = 0;
  else % Parallel
    Ib_baserail = NaN(1,R.NLEG);
    for j = 1:R.NLEG % gehe Beinketten durch
      % Umrechnung von Körper an Beinketten-Basis auf PKM-KS-Indizes
      if j > 1, NLoffset_j = 1+R.I2L_LEG(j-1)-(j-1);
      else,     NLoffset_j = 1; end
      Ib_baserail(j) = NLoffset_j+0;
    end
  end
  % Finde Kollisionsprüfungen dazu
  I_collcheck_baserail = false(size(Structure.selfcollchecks_collbodies,1),1);
  for i = 1:size(Structure.selfcollchecks_collbodies,1)
    collbodies_i = Structure.selfcollchecks_collbodies(i,:);
    bodies1 = Structure.collbodies_robot.link(collbodies_i(1),:);
    bodies2 = Structure.collbodies_robot.link(collbodies_i(2),:);
    brcand1 = ~isempty(intersect(bodies1,Ib_baserail)) && bodies1(1)==bodies1(2);
    brcand2 = ~isempty(intersect(bodies2,Ib_baserail)) && bodies2(1)==bodies2(2);
    if ~brcand1 && ~brcand2
      % Führungsschiene ist nur einem einzigen Körper zugeordnet
      continue % Diese Kollisionsprüfung ist ein anderer Basis-Körper
    end
    if any(bodies1(1) == Ib_baserail) || any(bodies2(1) == Ib_baserail)
      I_collcheck_baserail(i) = true;
    end
  end
  if false
    fprintf('Liste der %d Kollisionsprüfungen mit Führungsschienen:\n', sum(I_collcheck_baserail));   %#ok<UNRCH>
    for i = find(I_collcheck_baserail)'
      collbodies_i = Structure.selfcollchecks_collbodies(i,:);
      bodies_i = [Structure.collbodies_robot.link(collbodies_i(1),:), ...
        Structure.collbodies_robot.link(collbodies_i(2),:)];
      fprintf('%03d - collbodies %02d vs %02d (links %02d vs %02d; type %02d vs %02d; "%s" vs "%s")\n', i, ...
        Structure.selfcollchecks_collbodies(i,1), Structure.selfcollchecks_collbodies(i,2), ...
        Structure.collbodies_robot.link(collbodies_i(1),1), ...
        Structure.collbodies_robot.link(collbodies_i(2),1), ...
        Structure.collbodies_robot.type(collbodies_i(1)), ...
        Structure.collbodies_robot.type(collbodies_i(2)), ...
        names_collbodies{collbodies_i(1)}, ...
        names_collbodies{collbodies_i(2)});
    end
  end
  Structure.I_collcheck_baserail = I_collcheck_baserail;
end
%% Initialisierung der Bauraumprüfung
% Erstelle Liste der Kollisionsprüfungen für cds_constr_installspace.m
if ~isempty(Set.task.installspace.type)
  % Liste für alle Kollisionskörper des Roboters bei Bauraumprüfung.
  % Es werden nur Punkte anstatt der Ersatz-Volumen für den Roboter benutzt.
  % Das Format der Struktur ist genauso wie oben für die Kollisionen.
  % Initialisiere die Struktur mit den Objekten, die zur Basis gehören.
  % (aus Set.task.installspace.type)
  % Diese stehen am Anfang und werden auch durch die update-Funktion später
  % aktualisiert (Transformation Welt -> Basis)
  % Die Reihenfolge der Definition der Ersatzkörper ist konsistent mit
  % cds_update_collbodies.
  Structure.installspace_collbodies = struct('link',[], 'type',[], 'params',[]);
  % Zuerst nur Definition der Ersatzkörper aus Set.task.installspace.type
  [~, collbodies_instspc_fix] = cds_update_collbodies(R, Set, Structure);

  % Stelle Ersatz-Punkte für alle Robotergelenke zusammen
  for k = 1:NLEG
    % Ersatzkörper nur bezogen auf die Beinkette oder den seriellen Roboter
    collbodies_instspc_k = struct('link',[], 'type',[], 'params',[]);
    % Offset für PKM-Beinketten wird in Funktion ParRob/update_collbodies
    % durchgeführt. Hier nur Definition relativ zu PKM-Beinkette / SerRob
    if Structure.Type == 0  % Seriell 
      R_cc = R;
    else % PKM-Beinkette
      R_cc = R.Leg(k);
    end
    % Erzeuge Kollisionskörper für den statischen Teil von Schub- 
    % gelenken (z.B. Linearachsen). Siehe: SerRob/plot
    for i = find(R_cc.MDH.sigma'==1)
      if i > 1
        if i == R_cc.NJ
          warning(['Kollisionsprüfung (für Bauraum) für Schubgelenk als ', ...
            'letztes Gelenk der Kette noch nicht implementiert']);
        elseif ~(R_cc.MDH.a(i+1)==0 && R_cc.MDH.d(i+1)==0 && R_cc.DesPar.joint_type(i)==5)
          % Bei Schubzylinder ohne a-/d-Versatz passen die Kollisionskörper
          warning(['Kollisionsprüfung für Führungsschiene/-Zylinder für ', ...
            'nicht gestellfeste Linearantriebe nicht implementiert (bzgl. Bauraum)']);
        end
        continue
      end
      % Hänge zwei Punkte für Anfang und Ende jeder Linearführung an
      % Setze als Körper Nr. 14 (Punkt im Basis-KS).
      collbodies_instspc_k.type = [collbodies_instspc_k.type; repmat(uint8(14),2,1)];
      collbodies_instspc_k.params = [collbodies_instspc_k.params; NaN(2,10)];
      % Führungsschiene/Führungszylinder ist vorherigem Segment zugeordnet
      % Bei gestellfestem Schubgelenk wird ein Punkt bzgl Beinketten-Basis
      % und ein Punkt bzgl. PKM-Basis eingetragen
      links_i = repmat(uint8(i-1),2,1);
      collbodies_instspc_k.link = [collbodies_instspc_k.link; [links_i, v(1+links_i)]]; % TODO: Unklar welche Bedeutung die zweite Spalte hat
    end
    % Hänge einen Punkt (Nr. 9) für jedes Gelenk an. Unabhängig, ob 3D-Körper dafür
    collbodies_instspc_k.type = [collbodies_instspc_k.type; repmat(uint8(9),R_cc.NJ,1)];
    collbodies_instspc_k.params = [collbodies_instspc_k.params; NaN(R_cc.NJ,10)];
    collbodies_instspc_k.link = [collbodies_instspc_k.link; ...
      repmat(uint8((1:R_cc.NJ)'),1,2)];
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
    % Trage in Roboter-Klasse der PKM-Beinkette ein
    R_cc.collbodies_instspc = collbodies_instspc_k;
  end
  % Trage Bauraum-Objekte in die Klassen ein (Gesamt-PKM bzw. SerRob)
  if Structure.Type == 0  % Seriell 
    R.collbodies_instspc.type = [collbodies_instspc_fix.type; collbodies_instspc_k.type];
    R.collbodies_instspc.link = [collbodies_instspc_fix.link; collbodies_instspc_k.link];
    R.collbodies_instspc.params = [collbodies_instspc_fix.params; collbodies_instspc_k.params];
  else % PKM
    R.update_collbodies(2); % Aktualisiert R.collbodies_instspc
  end
  collbodies_instspc = R.collbodies_instspc;
  % Definiere die virtuellen Kollisionsprüfungen zur Bauraumüberwachung
  instspc_collchecks_collbodies = []; % Liste der Bauraum-Kollisionsprüfungen (bezogen auf obige Variable)
  % Stelle äquivalente Gelenknummer von PKM zusammen (als  Übersetzungs- 
  % tabelle). Zeile 1 ursprünglich, Zeile 2 Übersetzung. Siehe oben.
  % Seriell: 0=Basis, 1=erstes bewegtes Segment, ...
  equiv_link = repmat(0:max(collbodies_instspc.link(:,1)),2,1);
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
  n_cb_task = length(Set.task.installspace.type); 
  for i = 1:n_cb_task
    % Prüfe, für welches Robotersegment das Bauraum-Geometrieobjekt i
    % definiert ist
    links_i = Set.task.installspace.links{i}; % erlaubte Segmente
    % Gehe alle Kollisionskörper j des Roboters durch und prüfe, ob passend
    % (in collbodies_instspc zuerst Bauraumgrenzen, dann Roboter-Körper)
    for j = (n_cb_task+1):size(collbodies_instspc.type,1)
      % Nehme nicht die Segmentnummer selbst, sondern die des äquivalenten
      % Segments (jew. von der Basis aus gesehen, egal ob PKM oder seriell)
      iii = equiv_link(1,:) == collbodies_instspc.link(j,1);
      assert(~isempty(iii), 'Zuordnungsfehler bei equiv_link');
      equiv_link_j = equiv_link(2,iii); % 0=Basis, 1=erstes bewegtes,...
      if any(links_i == equiv_link_j)
        % Notation: Zuerst die Bauraumobjekte, dann die Roboterobjekte
        instspc_collchecks_collbodies = [instspc_collchecks_collbodies; ...
          uint8([j, i])]; %#ok<AGROW> % Prüfe "Kollision" Roboter-Segment j mit Bauraum-Begrenzung i
      end
    end
  end
  Structure.installspace_collbodies = collbodies_instspc; % Eingabe vorbereiten
  [~,collbodies_instspc] = cds_update_collbodies(R, Set, Structure, Structure.qlim');
  Structure.installspace_collbodies = collbodies_instspc; % modifizerte Struktur abspeichern
  Structure.installspace_collchecks_collbodies = instspc_collchecks_collbodies;
  assert(size(collbodies_instspc.link,2)==2, ['collbodies_instspc.link ', ...
    'muss 2 Spalten haben']);
  assert(size(collbodies_instspc.params,2)==10, ['collbodies_instspc.params ', ...
    'muss 10 Spalten haben']);
  assert(size(collbodies_instspc.type,2)==1, ['collbodies_instspc.type ', ...
    'muss 1 Spalte haben']);
  assert(size(collbodies_instspc.link,1)==size(collbodies_instspc.type,1), ...
    'collbodies_instspc.link muss so viele Zeilen wie type haben');
  assert(size(collbodies_instspc.params,1)==size(collbodies_instspc.type,1), ...
    'collbodies_instspc.params muss so viele Zeilen wie type haben');
  % Entferne Bauraumprüfungen von statischen Objekten aus der Roboterklasse
  % Diese können mit der IK nicht verbessert werden (z.B. Führungsschienen)
  % Dazu Nummer der Kollisionskörper aus Prüf-Liste holen
  links_collchecks = [collbodies_instspc.link(instspc_collchecks_collbodies(:,1),1), ...
                      collbodies_instspc.link(instspc_collchecks_collbodies(:,2),1)];
  % Aus der laufenden Nummer eine Zuordnung zur Nummer bzgl. Roboterbasis:
  equlinks_collchecks = NaN(size(links_collchecks)); % Zuordnung von oben
  for i = 1:length(equlinks_collchecks(:)) % Ordne den Prüfungen das äquivalente Segment zu
    equlinks_collchecks(i) = equiv_link(2,equiv_link(1,:)==links_collchecks(i));
  end
  % Filtern und eintragen in Klassenvariable
  I_basecheck = all(equlinks_collchecks==0,2); % 0 in equlinks_collchecks ist (PKM-)Basis
  R.collchecks_instspc = instspc_collchecks_collbodies(~I_basecheck,:);
  assert(size(R.collbodies_instspc.link,2)==2, ['R.collbodies_instspc.link ', ...
    'muss 2 Spalten haben']);
  assert(size(R.collbodies_instspc.params,2)==10, ['R.collbodies_instspc.params ', ...
    'muss 10 Spalten haben']);
  assert(size(R.collbodies_instspc.type,2)==1, ['R.collbodies_instspc.type ', ...
    'muss 1 Spalte haben']);
  assert(size(R.collbodies_instspc.link,1)==size(collbodies_instspc.type,1), ...
    'R.collbodies_instspc.link muss so viele Zeilen wie lokale Variable haben');
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

% Belege die Dynamik-Parameter mit Platzhalter-Werten. Wichtig, damit die
% Plots in jedem Fall ordentlich ausehen. Wird später überschrieben.
% Wenn immer vor Trajektorien-Berechnung abgebrochen wird, sind die Werte
% sonst nicht gesetzt.
cds_dimsynth_design(R, zeros(2,R.NJ), Set, Structure);
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
  % Lädt zusätzlich bisherige Ergebnisse, um schneller i.O.-Werte zu bekommen.
  [InitPop, QPop] = cds_gen_init_pop(Set, Structure);
  % Speichere die Gelenkwinkel der Anfangspopulation, um sie später wieder
  % abzurufen (betrifft die aus alten Ergebnissen geladenen).
  if ~isempty(QPop)
    I_dict = all(~isnan(QPop),2);
    Structure.dict_param_q = struct('p', InitPop(I_dict,:), 'q', QPop(I_dict,:));
  end
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
cds_log(3, sprintf('[dimsynth] Starte Optimierung mit %d Parametern: %s', ...
  length(varnames), disp_array(varnames, '%s')));
if Set.general.matfile_verbosity > 0
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_robot2.mat'));
end
if Set.optimization.NumIndividuals == 1 && Set.optimization.MaxIter == 0
  % Nur ein Dummy-Aufruf durchführen
  p_val = InitPop(1,:)';
  fval = fitnessfcn(p_val);
  exitflag = 0;
  p_val_pareto = [];
  fval_pareto = [];
  options = [];
elseif length(Set.optimization.objective) > 1 % Mehrkriteriell: GA-MO oder MOPSO
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
    d = load_checkpoint_file(Set, resdir);
    if isempty(d), return; end
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
  end
  % Entferne doppelte Partikel aus der Pareto-Front (bei MOPSO beobachtet)
  [~,I_unique] = unique(p_val_pareto, 'rows'); % Ausgabe sind Zahlen-Indizes (nicht: Binär)
  if length(I_unique) ~= size(p_val_pareto,1)
    cds_log(-1, sprintf(['[dimsynth] Pareto-Front bestand aus %d Partikeln, ', ...
      'aber nur %d davon nicht doppelt'], size(p_val_pareto,1), length(I_unique)));
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
    d = load_checkpoint_file(Set, resdir);
    if isempty(d), return; end
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
  % Trage in die persistente Variable der Speicher-Funktion ein für
  % Reproduktion der Ergebnisse
  PSO_Detail_Data = cds_save_particle_details([],[],0, 0, NaN, NaN, NaN, NaN,...
    'overwrite', d.PSO_Detail_Data);
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
% Ein mal muss immer nochmal neu berechnet werden.
max_retry = min(1,Set.general.max_retry_bestfitness_reconstruction);
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
q0_ik = PSO_Detail_Data.q0_ik(k_ind,:,k_gen)';
% Struktur-Variable neu erstellen um Schalter für Dynamik-Berechnung
% richtig zu setzen, wenn die Fitness-Funktion neu ausgeführt wird.
Structure_tmp = Structure;
if ~isempty(desopt_pval) && ... % Es gibt eine Entwurfsoptimierung
    ~any(isnan(desopt_pval)) % Vorher überhaupt bis Entwurfsoptimierung gekommen
  % Keine erneute Entwurfsoptimierung, also auch keine Regressorform notwendig.
  % Direkte Berechnung der Dynamik, falls für Zielfunktion notwendig.
  Structure_tmp.calc_dyn_act = Structure.calc_dyn_act | Structure.calc_dyn_reg;
  Structure_tmp.calc_spring_act = Structure.calc_spring_act | Structure.calc_spring_reg;
  Structure_tmp.calc_spring_reg = false;
  Structure_tmp.calc_dyn_reg = false;
end
% Gebe IK-Anfangswerte aus bekannter Lösung vor, falls einmal die Trajek-
% torie berechnet werden konnte. Falls nicht, würde die Vorgabe hier die 
% Traj.-Berechnung erzwingen und automatisch ein verbessertes Ergebnis zeigen.
if any(fval < 1e9)
  Structure_tmp.q0_traj = q0_ik;
end
% Erneuter Aufruf der Fitness-Funktion. Hauptsächlich, um die Q-Trajektorie
% extrahieren zu können. Rekonstruktion im Fall von Aufgabenredundanz sonst
% nicht so einfach möglich.
% Bei erneutem Aufruf keine Debug-Bilder. Dauert sonst bei Abschluss von
% Berechnung zu lange.
Set.general.debug_taskred_perfmap = false;
Set.general.plot_details_in_fitness = 0;
Set.general.plot_robot_in_fitness = 0;
for i = 1:max_retry 
  % Mehrere Versuche vornehmen, da beim Umklappen der Roboterkonfiguration
  % andere Ergebnisse entstehen können.
  % Eigentlich darf sich das Ergebnis aber nicht ändern (wegen der
  % Zufallszahlen-Initialisierung in cds_fitness). Es kann rundungsbedingte
  % Aenderungen des Ergebnisses geben.
  clear cds_fitness % persistente Variable in fitnessfcn löschen (falls Grenzwert erreicht wurde wird sonst inf zurückgegeben)
  % Aufruf nicht über anonmye Funktion, sondern vollständig, damit Param.
  % der Entwurfsoptimierung übergeben werden können.
  [fval_test, ~, Q, QD, QDD, TAU, JP, ~, X6Traj] = cds_fitness(R, Set, Traj, Structure_tmp, p_val, desopt_pval);
  if any(abs(fval_test-fval)>1e-8)
    if all(fval_test < fval)
      t = sprintf('Der neue Wert (%s) ist um [%s] besser als der alte (%s).', ...
        disp_array(fval_test','%1.1f'), disp_array(fval'-fval_test','%1.1e'), disp_array(fval','%1.1f'));
    else
      t = sprintf('Der alte Wert (%s) ist um [%s] besser als der neue (%s).', ...
        disp_array(fval','%1.1f'), disp_array(fval_test'-fval','%1.1e'), disp_array(fval_test','%1.1f'));
    end
    if fval < 1e4*5e4 % Warnung ergibt nur Sinn, wenn IK erfolgreich. Sonst immer Abweichung.
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
PSO_Detail_Data = cds_save_particle_details(Set, R, 0, 0, NaN, NaN, NaN, NaN, 'output');
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
  if any(abs(test_q0)>1e-6) && all(fval<1e10) % nur bei erfolgreicher Berechnung der IK ist der gespeicherte Wert sinnvoll
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
if ~Set.optimization.fix_joint_limits
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
end

result_invalid = false;
if ~any(strcmp(Set.optimization.objective, 'valid_act')) && ...
    (any(isnan(Q(:))) || isempty(QD) && Set.task.profile~=0)
  % Berechnung der Trajektorie ist fehlgeschlagen. Toleranz wie in cds_constraints
  if any(fval<1e4*1e4) % nur bemerkenswert, falls vorher überhaupt soweit gekommen.
    cds_log(-1, sprintf(['[dimsynth] PSO-Ergebnis für Trajektorie nicht ', ...
      'reproduzierbar oder nicht gültig (ZB-Verletzung).'] ));
    try % Auf Cluster teilweise Probleme beim Dateisystemzugriff
      save(fullfile(resdir, 'trajikreprowarning.mat'));
    catch err
      cds_log(-1, sprintf('[dimsynth] Fehler beim Speichern von mat-Datei: %s', err.message));
    end
  end
  result_invalid = true;
end

% Transformation der Trajektorie ins Basis-KS. Beachte geänderten dritten
% Euler-Winkel bei Aufgabenredundanz oder 3T2R-PKM
Traj_0 = cds_transform_traj(R, Traj);
if ~isempty(X6Traj) && (Structure.task_red || all(R.I_EE_Task == [1 1 1 1 1 0]))
  Traj_0.X(:,6) = X6Traj(:,1);
  Traj_0.XD(:,6) = X6Traj(:,2);
  Traj_0.XDD(:,6) = X6Traj(:,3);
end
% Berechne die Jacobi-Matrix für PKM neu. Keine Neuberechnung der inversen
% Kinematik (wird bereits in erneutem Aufruf der Fitness-Funktion gemacht)
Jinv_ges = []; % Platzhalter
if R.Type ~= 0 && ~result_invalid && ~isempty(QD) % nur machen, wenn Traj.-IK erfolgreich
  Jinv_ges = NaN(size(Q,1), sum(R.I_EE)*R.NJ);
  test_xD_fromJ_max = 0; % Fehler dabei prüfen
  i_maxerr = 0;
  for i = 1:size(Q,1)
    [~,Jinv_x] = R.jacobi_qa_x(Q(i,:)', Traj_0.X(i,:)'); % Jacobi-Matrix
    Jinv_ges(i,:) = Jinv_x(:);
    % Prüfe, ob differentieller Zusammenhang mit Jacobi-Matrix korrekt ist.
    % Berücksichtigung von 2T1R vs 3T3R Plattform-Koordinaten
    test_xD_fromJ_abs = zeros(6,1);
    test_xD_fromJ_abs(R.I_EE) =  Traj_0.XD(i,R.I_EE)' - Jinv_x(R.I_qa,:) \ QD(i,R.I_qa)';
    if max(abs(test_xD_fromJ_abs(R.I_EE_Task))) > test_xD_fromJ_max
      test_xD_fromJ_max = max(abs(test_xD_fromJ_abs(R.I_EE_Task)));
      i_maxerr = i;
    end
  end
  if test_xD_fromJ_max > 1e-6
    cds_log(-1, sprintf(['[dimsynth] Neuberechnung der Jacobi-Matrix ', ...
      'falsch. Zuerst Schritt %d/%d. Fehler in EE-Geschw.: max %1.1e'], ...
      i_maxerr, size(Q,1), test_xD_fromJ_max));
    try % Auf Cluster teilweise Probleme beim Dateisystemzugriff
      save(fullfile(resdir, 'jacobireprowarning.mat'));
    catch err
      cds_log(-1, sprintf('[dimsynth] Fehler beim Speichern von mat-Datei: %s', err.message));
    end
  end
end

if any(q0_ik<qlim_neu(:,1) | q0_ik>qlim_neu(:,2))
  cds_log(-1, sprintf(['[dimsynth] Startwert für Gelenkwinkel liegt außer', ...
    'halb des erlaubten Bereichs.']));
  try % Auf Cluster teilweise Probleme beim Dateisystemzugriff
    save(fullfile(resdir, 'jointlimitviolationwarning.mat'));
  catch err
    cds_log(-1, sprintf('[dimsynth] Fehler beim Speichern von mat-Datei: %s', err.message));
  end
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
if ~result_invalid && ~any(strcmp(Set.optimization.objective, 'valid_act')) && ~isempty(QD)
  % Masseparameter belegen, falls das nicht vorher passiert ist.
  % Nachbildung der Bedingungen für Belegung der Masseparameter in cds_fitness.m
  % Stelle fest, ob die Zielfunktion rein kinematisch ist; dann werden die
  % Dynamikparameter nicht in der Fitness-Funktion belegt
  only_kinematic_objective = length(intersect(Set.optimization.objective, ...
    {'condition','jointrange','jointlimit','manipulability','minjacsingval','positionerror', ...
    'actvelo','chainlength','installspace','footprint','colldist'})) == length(Set.optimization.objective);
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
  [fval_jlimit,~, ~, physval_jlimit] = cds_obj_jointlimit(R, Set, Structure, Q);
  [fval_actvelo,~, ~, physval_actvelo] = cds_obj_actvelo(R, QD);
  [fval_chainlength,~, ~, physval_chainlength] = cds_obj_chainlength(R);
  [fval_instspc,~, ~, physval_instspc] = cds_obj_installspace(R, Set, Structure, Traj_0, Q, JP);
  [fval_footprint,~, ~, physval_footprint] = cds_obj_footprint(R, Set, Structure, Traj_0, Q, JP);
  [fval_colldist,~, ~, physval_colldist] = cds_obj_colldist(R, Set, Structure, Traj_0, Q, JP);
  if Set.optimization.nolinkmass
    % Die Formel für die Berechnung der Steifigkeit benutzt als Hilfsgröße
    % die Masse der Beinketten. TODO: Ansatz überarbeiten.
    fval_stiff = NaN; physval_stiff = NaN;
    cds_log(2, sprintf(['[dimsynth] Keine Berechnung der Steifigkeit möglich, ', ...
      'da masselose Beinketten.']));
  else
    [fval_stiff,~, ~, physval_stiff] = cds_obj_stiffness(R, Set, Q);
  end
  % Reihenfolge siehe Variable Set.optimization.constraint_obj aus cds_settings_defaults
  fval_obj_all = [fval_mass; fval_energy; fval_actforce; fval_ms; fval_cond; ...
    fval_mani; fval_msv; fval_pe; fval_jrange; fval_jlimit; fval_actvelo; fval_chainlength; ...
    fval_instspc; fval_footprint; fval_colldist; fval_stiff];
  physval_obj_all = [physval_mass; physval_energy; physval_actforce; ...
    physval_ms; physval_cond; physval_mani; physval_msv; physval_pe; ...
    physval_jrange; physval_jlimit; physval_actvelo; physval_chainlength; ...
    physval_instspc; physval_footprint; physval_colldist; physval_stiff];
  if length(fval_obj_all)~=16 || length(physval_obj_all)~=16
    % Dimension ist falsch, wenn eine Zielfunktion nicht skalar ist (z.B. leer)
    cds_log(-1, sprintf(['[dimsynth] Dimension der Zielfunktionen falsch ', ...
      'berechnet. dim(fval_obj_all)=%d, dim(physval_obj_all)=%d'], ...
      length(fval_obj_all), length(physval_obj_all)));
    try % Auf Cluster teilweise Probleme beim Dateisystemzugriff
      save(fullfile(resdir, 'fvaldimensionerror.mat'));
    catch err
      cds_log(-1, sprintf('[dimsynth] Fehler beim Speichern von mat-Datei: %s', err.message));
    end
  end
  % Vergleiche neu berechnete Werte mit den zuvor abgespeicherten (müssen
  % übereinstimmen)
  test_Jcond_abs = PSO_Detail_Data.constraint_obj_val(dd_optind, 4, dd_optgen) - physval_cond;
  test_Jcond_rel = test_Jcond_abs / physval_cond;
  if abs(test_Jcond_abs) > 1e-6 && test_Jcond_rel > 1e-3 && ...
      physval_cond < 1e6 % Abweichung nicht in Singularität bestimmbar
    cds_log(-1, sprintf(['[dimsynth] Während Optimierung gespeicherte ', ...
      'Konditionszahl (%1.5e) stimmt nicht mit erneuter Berechnung (%1.5e) ', ...
      'überein. Differenz %1.5e (%1.2f%%)'], ...
      PSO_Detail_Data.constraint_obj_val(dd_optind, 4, dd_optgen), ...
      physval_cond, test_Jcond_abs, 100*test_Jcond_rel));
    try % Auf Cluster teilweise Probleme beim Dateisystemzugriff
      save(fullfile(resdir, 'condreprowarning.mat'));
    catch err
      cds_log(-1, sprintf('[dimsynth] Fehler beim Speichern von mat-Datei: %s', err.message));
    end
  end
  test_sv = PSO_Detail_Data.constraint_obj_val(dd_optind, 6, dd_optgen) - physval_ms;
  if abs(test_sv) > 1e-5
    cds_log(-1, sprintf(['[dimsynth] Während Optimierung gespeicherte ', ...
      'Materialbelastung (%1.5e) stimmt nicht mit erneuter Berechnung (%1.5e) ', ...
      'überein. Differenz %1.5e.'], ...
      PSO_Detail_Data.constraint_obj_val(dd_optind, 6, dd_optgen), physval_ms, test_sv));
    try % Auf Cluster teilweise Probleme beim Dateisystemzugriff
      save(fullfile(resdir, 'svreprowarning.mat'));
    catch err
      cds_log(-1, sprintf('[dimsynth] Fehler beim Speichern von mat-Datei: %s', err.message));
    end
  end
  test_actforce = PSO_Detail_Data.constraint_obj_val(dd_optind, 3, dd_optgen) - physval_actforce;
  if abs(test_actforce) > 1e-5
    cds_log(-1, sprintf(['[dimsynth] Während Optimierung gespeicherte ', ...
      'Antriebskraft (%1.5e) stimmt nicht mit erneuter Berechnung (%1.5e) ', ...
      'überein. Differenz %1.5e.'], ...
      PSO_Detail_Data.constraint_obj_val(dd_optind, 3, dd_optgen), physval_actforce, test_actforce));
    try % Auf Cluster teilweise Probleme beim Dateisystemzugriff
      save(fullfile(resdir, 'actforcereprowarning.mat'));
    catch err
      cds_log(-1, sprintf('[dimsynth] Fehler beim Speichern von mat-Datei: %s', err.message));
    end
  end
  test_mass = PSO_Detail_Data.constraint_obj_val(dd_optind, 1, dd_optgen) - physval_mass;
  if abs(test_mass) > 1e-5
    cds_log(-1, sprintf('[dimsynth] Während Optimierung gespeicherte Masse (%1.5e) stimmt nicht mit erneuter Berechnung (%1.5e) überein. Differenz %1.5e.', ...
      PSO_Detail_Data.constraint_obj_val(dd_optind, 1, dd_optgen), physval_mass, test_mass));
    try % Auf Cluster teilweise Probleme beim Dateisystemzugriff
      save(fullfile(resdir, 'massreprowarning.mat'));
    catch err
      cds_log(-1, sprintf('[dimsynth] Fehler beim Speichern von mat-Datei: %s', err.message));
    end
  end
else
  % Keine Berechnung der Leistungsmerkmale möglich, da keine zulässige Lösung
  % gefunden wurde.
  fval_obj_all = NaN(16,1);
  physval_obj_all = NaN(16,1);
  physval_cond = inf;
end
% Prüfe auf Plausibilität, ob die Optimierungsziele erreicht wurden. Neben-
% bedingungen nur prüfen, falls überhaupt gültige Lösung erreicht wurde.
I_fobj_set = Set.optimization.constraint_obj ~= 0;
% Die Reihenfolge der Zielfunktionen insgesamt und die der Zielfunktionen
% als Grenze sind unterschiedlich. Finde Indizes der einen in den anderen.
objconstr_names_all = {'mass', 'energy', 'actforce', 'condition', ...
  'stiffness', 'materialstress'};
obj_names_all = {'mass', 'energy', 'actforce', 'materialstress', 'condition', ...
  'manipulability', 'minjacsingval', 'positionerror', 'jointrange', 'jointlimit', ...
  'actvelo','chainlength', 'installspace', 'footprint', 'colldist', 'stiffness'}; % konsistent zu fval_obj_all und physval_obj_all
I_constr = zeros(length(objconstr_names_all),1);
for i = 1:length(objconstr_names_all)
  I_constr(i) = find(strcmp(objconstr_names_all{i}, obj_names_all));
end
% Indizes der verletzten Nebenbedingungen. Wert Null heißt inaktiv.
I_viol = physval_obj_all(I_constr) > Set.optimization.constraint_obj & I_fobj_set;
if any(fval<1e3) && any(I_viol)
  for i = find(I_viol)'
    cds_log(-1,sprintf(['[dimsynth] Zielfunktions-Nebenbedingung %d (%s) verletzt ', ...
      'trotz Berücksichtigung in Optimierung: %1.4e > %1.4e. Keine Lösung gefunden.'], ...
      i, objconstr_names_all{i}, physval_obj_all(I_constr(i)), Set.optimization.constraint_obj(i)));
  end
  try % Auf Cluster teilweise Probleme beim Dateisystemzugriff
    save(fullfile(resdir, 'objconstrwarning.mat'));
  catch err
    cds_log(-1, sprintf('[dimsynth] Fehler beim Speichern von mat-Datei: %s', err.message));
  end
end
if any(fval<1e3) && Set.optimization.constraint_obj(6) ~= 0 && ...
    physval_ms > Set.optimization.constraint_obj(6) 
  cds_log(-1, sprintf('[dimsynth] Materialbelastungs-Nebenbedingung verletzt trotz Berücksichtigung in Optimierung. Keine Lösung gefunden.'));
  try % Auf Cluster teilweise Probleme beim Dateisystemzugriff
    save(fullfile(resdir, 'strengthconstrwarning.mat'));
  catch err
    cds_log(-1, sprintf('[dimsynth] Fehler beim Speichern von mat-Datei: %s', err.message));
  end
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
  if strcmp(Set.optimization.objective{i}, 'condition') && physval_cond > 1e6 
    continue % Abweichung in Singularität numerisch unsicher
  end
  test_fval_i = fval(i) - fval_obj_all(I_fval_obj_all(i));
  if abs(test_fval_i) > 1e-5
    cds_log(-1, sprintf(['[dimsynth] Zielfunktionswert %d (%s) nicht aus Neu', ...
      'berechnung der Leistungsmerkmale reproduzierbar. Aus PSO: %1.5f, neu: %1.5f.'], ...
      i, obj_names_all{I_fval_obj_all(i)}, fval(i), fval_obj_all(I_fval_obj_all(i))));
    try % Auf Cluster teilweise Probleme beim Dateisystemzugriff
      save(fullfile(resdir, 'fvalreprowarning.mat'));
    catch err
      cds_log(-1, sprintf('[dimsynth] Fehler beim Speichern von mat-Datei: %s', err.message));
    end
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
lfp = cds_log(1,sprintf(['[dimsynth] Optimierung von Rob. %d (%s) abgeschlossen. ', ...
  'Dauer: %1.1fs'], Structure.Number, Structure.Name, toc(t1)));
if isempty(lfp) % Aufruf mit Set.general.only_finish_aborted. Log nicht initialisiert.
  robstr = sprintf('Rob%d_%s', Structure.Number, Structure.Name);
  lfp = fullfile(Set.optimization.resdir, Set.optimization.optname, ...
    robstr, sprintf('%s.log', robstr));
end
if Set.general.only_finish_aborted
  % Zeitstempel von Start und Ende der Optimierung aus Log-Datei auslesen
  if exist(lfp, 'file') % Findet alle Datums-Zeit-Stempel in eckigen Klammern (sollten nur am Zeilenanfang stehen)
    matches = regexp(fileread(lfp), '\[(\d+)-(\d+)-(\d+) (\d+):(\d+):(\d+)\]', 'match');
    t_start = datenum(matches{1});
    t_end = datenum(matches{end});
  else % Überschreibe Werte mit NaN, damit zumindest keine falschen Werte in die Tabelle kommen
    t_start = NaN;
    t_end = NaN;
  end
end

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
  'q0', q0_ik, ... % Anfangs-Gelenkwinkel für Lösung der IK
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
  'Traj_Tau', TAU, ...
  'timestamps_start_end', [t_start, t_end, toc(t1)], ...
  'fitnessfcn', fitnessfcn);
% Debug: Durch laden dieser Ergebnisse kann nach Abbruch des PSO das
% Ergebnis trotzdem geladen werden
if Set.general.matfile_verbosity > 0
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_robot4.mat'));
end
% Gesamtergebnis der Optimierung speichern
save(resultfile, 'RobotOptRes');
save(fullfile(Set.optimization.resdir, Set.optimization.optname, ...
  sprintf('Rob%d_%s_Details.mat', Structure.Number, Structure.Name)), ...
  'RobotOptDetails', 'PSO_Detail_Data');
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
end


function d = load_checkpoint_file(Set, resdir)
% Lade Daten zu letzter erfolgreicher Generation der Optimierung
% Eingabe:
%   Set, resdir: Siehe Definition bei Aufruf der Funktion
% Ausgabe:
%   d: Inhalt der Sicherungs-Datei
d = [];
if strcmp(Set.optimization.algorithm, 'mopso')
  filelist_tmpres = dir(fullfile(resdir, 'MOPSO_Gen*_AllInd.mat'));
elseif strcmp(Set.optimization.algorithm, 'gamultiobj')
  filelist_tmpres = dir(fullfile(resdir, 'GAMO_Gen*_AllInd.mat'));
elseif strcmp(Set.optimization.algorithm, 'pso')
  filelist_tmpres = dir(fullfile(resdir, 'PSO_Gen*_AllInd.mat'));
else
  error('Algorithmus %s nicht definiert', Set.optimization.algorithm);
end
if isempty(filelist_tmpres)
  cds_log(1, sprintf(['[dimsynth] Laden des letzten abgebrochenen Durch', ...
    'laufs wurde angefordert. Aber keine Daten in %s vorliegend. Ende.'], resdir));
  return
end
% Bestimme die Reihenfolge der Checkpoint-Dateien. Normalerweise
% chronologisch, aber nochmal Prüfung anhand der Gen.-Nummer.
[tmp,~] = regexp({filelist_tmpres.name},'_Gen(\d+)_','tokens','match');
filelist_gen = cellfun(@str2double, cellfun(@(v)v{1},tmp) );
[~,I_genasc] = sort(filelist_gen);

for ii = fliplr(I_genasc)
  try
    d = load(fullfile(resdir, filelist_tmpres(ii).name));
    cds_log(1, sprintf(['[dimsynth] Laden des letzten abgebrochenen Durch', ...
      'laufs aus gespeicherten Daten erfolgreich aus %s.'], ...
      fullfile(resdir,filelist_tmpres(ii).name)));
    break;
  catch err
    cds_log(-1, sprintf(['[dimsynth] Fehler beim Laden von Wiederauf', ...
      'nahme-Datei: %s.'], err.message));
    continue
  end
end
if isempty(d)
  cds_log(-1, sprintf(['[dimsynth] Keine der %d Wiederaufnahme-Dateien ', ...
    'erfolgreich geladen.'], length(I_genasc)));
  return
end
end
