% Start der komb. Struktur und Maßsynthese für alle Roboter
% 
% Eingabe:
%   Set (Globale Einstellungen). Siehe cds_settings_defaults.m
%   Traj (Eigenschaften der Trajektorie). Siehe cds_gen_traj.m
%   Structures (Liste der zu optimierenden Strukturen, s. cds_gen_robot_list.m)
% 
% Ausgabe:
% jobID_out
%   Nummer des letzten Teil-Jobs auf dem Rechencluster. Kann für
%   Abhängigkeiten beim Start mehrerer Synthese-Durchläufe benutzt werden.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function jobID_out = cds_start(Set, Traj, Structures)

% Warnungen unterdrücken, die bei der Maßsynthese typischerweise auftreten
warning('off', 'MATLAB:singularMatrix');
warning('off', 'MATLAB:nearlySingularMatrix');
warning('off', 'MATLAB:illConditionedMatrix');
warning('off', 'MATLAB:rankDeficientMatrix');
warning('off', 'Coder:MATLAB:singularMatrix');
warning('off', 'Coder:MATLAB:nearlySingularMatrix');
warning('off', 'Coder:MATLAB:illConditionedMatrix');
warning('off', 'Coder:MATLAB:rankDeficientMatrix');
% Falls Figure manuell angedockt wurde und gespeichert werden soll
warning('off', 'MATLAB:Figure:SetPosition');
if nargin < 3
  Structures = [];
end
jobID_out = [];
%% Log Vorbereiten
resdir_main = fullfile(Set.optimization.resdir, Set.optimization.optname);
if Set.general.isoncluster && isfolder(resdir_main) && ...
    ~Set.general.regenerate_summary_only && ~Set.general.only_finish_aborted
  % Nur Abbruch, wenn Maßsynthese auf Cluster durchgeführt werden soll.
  % Auch nur dann, wenn schon Ergebnisse vorliegen. Im Fall von Abbruch
  % eines Jobs auf dem Cluster (z.B. wegen NODE FAIL), wird dieser automatisch neu gestartet.
  if exist(fullfile(resdir_main, [Set.optimization.optname, '_results_table.csv']), 'file')
    error('Ergebnis-Tabelle existiert bereits in %s. Kein Überschreiben auf Cluster', resdir_main)
  end
  fprintf('Verzeichnis %s existiert bereits, aber ohne Endergebnisse. Überschreibe.\n', resdir_main)
end
if ~Set.general.computing_cluster
  % Ergebnis-Ordner für diese Optimierung erstellen (für Log benötigt).
  % Falls nur hochgeladen wird, wird keine Log-Datei angelegt.
  % Wenn auf Cluster gerechnet wird, ist die Option deaktiviert.
  mkdirs(resdir_main);
end
if ~Set.general.only_finish_aborted
  msg = sprintf('Starte Maßsynthese %s.', Set.optimization.optname);
else
  msg = sprintf('Schließe die abgebrochene Maßsynthese %s ab.', Set.optimization.optname);
end
cds_log(); % persistente Variablen zurücksetzen
fpfile = cds_log(1, msg, 'init', Set, struct('Number', 0));
% Fingerabdruck der relevanten Repos in Log-Datei speichern
if ~isempty(fpfile) % (nur wenn Log-Datei erstellt wird, nicht bei Hochladen)
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
  fprintf(fid, 'Matlab-Konfigurationsverzeichnis: %s\n', prefdir());
  fclose(fid);
  if ~ispc() % lscpu funktioniert nur unter Linux
    system(sprintf('echo "Rechnung läuft auf: $(whoami)@$(hostname)" >> %s', fpfile));
    % Cluster-Umgebungsvariablen
    system(sprintf('echo "Umgebungsvariable HOME: `echo $HOME`" >> %s', fpfile));
    system(sprintf('echo "Umgebungsvariable TMPDIR: `echo $TMPDIR`" >> %s', fpfile));
    system(sprintf('echo "Umgebungsvariable JOBID: `echo $JOBID`" >> %s', fpfile));
    system(sprintf('echo "Eigenschaften des Rechners (lscpu):" >> %s', fpfile));
    system(sprintf('lscpu >> %s', fpfile));
  end
  cd(olddir);
end
%% Eingaben prüfen 
if Set.general.only_finish_aborted && Set.general.regenerate_summary_only
  error('Option only_finish_aborted zusammen mit regenerate_summary_only nicht sinnvoll');
end
Set_default = cds_settings_defaults(struct('DoF', Set.task.DoF));
for subconf = fields(Set_default)'
  for ftmp = fields(Set.(subconf{1}))'
    if ~isfield(Set_default.(subconf{1}), ftmp{1})
      warning('Feld %s.%s in der Eingabestruktur ist nicht vorgesehen', subconf{1}, ftmp{1})
    end
  end
end
for ftmp = fields(Set)'
  if ~isfield(Set_default, ftmp{1})
    warning('Feld %s in der Eingabestruktur ist nicht vorgesehen', ftmp{1})
  end
end
if isa(Set.optimization.objective, 'char')
  % Bei mehrkriterieller Optimierung mehrere Zielfunktionsnamen als cell.
  % Einheitliches Format für Ein- und Mehrkriteriell.
  Set.optimization.objective = {Set.optimization.objective};
end
if length(intersect(Set.optimization.objective, {'mass', 'energy', 'power', ...
    'condition', 'valid_kin', 'valid_act', 'actforce', 'materialstress', ...
    'stiffness', 'jointrange', 'jointlimit', 'manipulability', 'minjacsingval', ...
    'positionerror', 'actvelo', 'chainlength', 'installspace', 'footprint', 'colldist', 'mrk1', 'mrk2'})) ~= ...
    length(Set.optimization.objective)
  error('Set.optimization.objective enthält unerwarteten Wert');
end
if length(Set.optimization.objective) > 1 && strcmp(Set.optimization.algorithm, 'pso')
  Set.optimization.algorithm = 'mopso';
elseif length(Set.optimization.objective) == 1 && strcmp(Set.optimization.algorithm, 'mopso')
  Set.optimization.algorithm = 'pso';
end
if isa(Set.optimization.objective_ik, 'char')
  Set.optimization.objective_ik = {Set.optimization.objective_ik};
end
if length(Set.optimization.objective_ik) > 1
  error('Nur eine Zielfunktion in Set.optimization.objective_ik möglich');
end
if Set.task.profile == 0 && any(strcmp(Set.optimization.objective, 'energy'))
  error('Energieberechnung ohne Zeitverlauf der Trajektorie nicht sinnvoll');
end
if Set.task.profile == 0 && any(strcmp(Set.optimization.objective, 'actvelo'))
  error('Antriebsgeschwindigkeit als Optimierungsziel ohne Zeitverlauf der Trajektorie nicht sinnvoll');
end
if Set.optimization.nolinkmass && any(strcmp(Set.optimization.objective, 'stiffness'))
  error('Berechnung der Steifigkeit zusammen mit nolinkmass aktuell nicht möglich');
end
only_opts = [Set.optimization.obj_jointrange.only_prismatic; ...
             Set.optimization.obj_jointrange.only_revolute];
if sum(only_opts) > 1
  error('Es dürfen nicht mehr als eine Option vom Typ only_prismatic in Set.optimization.obj_jointrange gleichzeitig aktiv sein.');
end
only_opts = [Set.optimization.obj_jointrange.only_active; ...
             Set.optimization.obj_jointrange.only_passive];
if sum(only_opts) > 1
  error('Es dürfen nicht mehr als eine Option vom Typ only_active in Set.optimization.obj_jointrange gleichzeitig aktiv sein.');
end
% Prüfe Plausibilität von Abbruchbedingungen und Wahl mehrkriterieller Ziele
if length(Set.optimization.obj_limit_physval) == 1 && length(Set.optimization.objective) > 1 && ...
    isnan(Set.optimization.obj_limit_physval) % nur, falls NaN nicht bereits überschrieben wurde
  % Korrigiere auf Dimension der Fitness-Funktion. Skalare Grenze von
  % NaN ist Standard (kein Fehler)
  Set.optimization.obj_limit_physval = repmat(Set.optimization.obj_limit_physval, ...
    length(Set.optimization.objective), 1);
end
% Jetzt muss die Anzahl der Zielfunktionen mit der Abbruchbedingung stimmen
if length(Set.optimization.objective) ~= length(Set.optimization.obj_limit_physval)
  error('%d Zielfunktionen gesetzt und %d Abbruchbedingungen in obj_limit_physval. Passt nicht.', ...
    length(Set.optimization.objective), length(Set.optimization.obj_limit_physval));
end
if length(Set.optimization.obj_limit) == 1 && length(Set.optimization.objective) > 1 && ...
    Set.optimization.obj_limit == 0 % nur, falls Null nicht bereits überschrieben wurde
  Set.optimization.obj_limit = repmat(Set.optimization.obj_limit, ...
    length(Set.optimization.objective), 1);
end
if ~any(strcmp(Set.optimization.algorithm, {'pso', 'mopso', 'gamultiobj'}))
  error('Unerwarteter Wert für Feld "algorithm": %s', Set.optimization.algorithm);
end
if length(Set.optimization.objective) ~= length(Set.optimization.obj_limit)
  error('%d Zielfunktionen gesetzt und %d Abbruchbedingungen in obj_limit. Passt nicht.', ...
    length(Set.optimization.objective), length(Set.optimization.obj_limit));
end
if size(Set.optimization.obj_limit_physval,2) > 1
  error('obj_limit_physval muss %d x 1 Vektor sein', length(Set.optimization.objective));
end
if size(Set.optimization.obj_limit,2) > 1
  error('obj_limit muss %d x 1 Vektor sein', length(Set.optimization.objective));
end
if ~all(size(Set.optimization.base_size_limits)==[1 2])
  error('base_size_limits muss 1x2 sein min/max Radius');
end
if ~all(size(Set.optimization.platform_size_limits)==[1 2])
  error('platform_size_limits muss 1x2 sein min/max Radius');
end
if all(~isnan(Set.optimization.base_size_limits)) && ...
    diff(Set.optimization.base_size_limits)==0 && Set.optimization.base_size
  error('Optimierung von base_size_limits aktiv, aber Grenzen sind gleich');
end
if all(~isnan(Set.optimization.platform_size_limits)) && ...
    diff(Set.optimization.platform_size_limits)==0 && Set.optimization.platform_size
  error('Optimierung von platform_size aktiv, aber Grenzen sind gleich');
end
if ~all(size(Set.optimization.basepos_limits)==[3 2])
  error('basepos_limits muss 3x2 sein (xyz Koordinate, min/max)');
end
if Set.optimization.rotate_base_only_orthogonal && ~Set.optimization.rotate_base
  error('rotate_base_only_orthogonal ohne rotate_base ergibt keinen Sinn.');
end
if Set.optimization.tilt_base_only_orthogonal && ~Set.optimization.tilt_base
  error('tilt_base_only_orthogonal ohne tilt_base ergibt keinen Sinn.');
end
if ~all(size(Set.optimization.ee_translation_fixed)==[1 3])
  error('ee_translation_fixed muss 1x3 sein (xyz Koordinate)');
end
if ~all(size(Set.optimization.ee_rotation_fixed)==[1 3])
  error('ee_rotation_fixed muss 1x3 sein (xyz Koordinate, Winkel)');
end
if size(Set.task.installspace.params,1) ~= length(Set.task.installspace.type)
  error('Set.task.installspace: Länge von Feldern "params" und "type" stimmt nicht überein');
end
if length(Set.task.installspace.links) ~= length(Set.task.installspace.type)
  error('Set.task.installspace: Länge von Feldern "links" und "type" stimmt nicht überein');
end
if isa(Set.task.installspace.links, 'uint8')
  warning('Set.task.installspace.links: Typ wurde auf uint16 geändert.');
  Set.task.installspace.links = uint16(Set.task.installspace.links);
end
if ~isa(Set.task.installspace.links, 'cell')
  error('Set.task.installspace: Feld "links" muss cell Array sein');
end
if ~isempty(Set.task.installspace.links)
  if ~isa(Set.task.installspace.links{1}, 'double')
    error('Set.task.installspace: Feld "links" muss cell Array sein, das double Arrays enthält');
  end
end
if size(Set.task.installspace.type,2) > 1
  error('Set.task.installspace: Feld "type" hat mehr als eine Spalte.');
end
if size(Set.task.obstacles.params,1) ~= length(Set.task.obstacles.type)
  error('Set.task.obstacles: Länge von Feldern params und type stimmt nicht überein');
end
if size(Set.task.obstacles.type,2) > 1
  error('Set.task.obstacles: Feld "type" hat mehr als eine Spalte.');
end
if any(Set.task.obstacles.type==1) || any(Set.task.obstacles.type==2)
  warning(['Arbeitsraum-Hindernis als Quader oder Zylinder definiert. ' ...
    'Kollisionsprüfung nur als Ersatzpunkt definiert'])
end
assert(isa(Set.optimization.collshape_base, 'cell'), ...
  'Set.optimization.collshape_base muss cell array sein');
if length(intersect(Set.optimization.collshape_base, {'default', 'ring', ...
    'star', 'joint'})) ~= length(Set.optimization.collshape_base)
  error('Set.optimization.collshape_base enthält unerwarteten Wert');
end
if length(union(Set.optimization.desopt_vars, {'joint_stiffness_qref', ...
    'joint_stiffness', 'linkstrength'})) ~= 3
  error('Unerwarteter Wert in Set.optimization.desopt_vars');
end
if Set.optimization.constraint_collisions_desopt && ~Set.optimization.constraint_collisions
  error('constraint_collisions_desopt=1, aber constraint_collisions=0. Unlogisch.')
end
defstruct = cds_definitions();
if length(intersect(Set.optimization.objective_ik, ...
    defstruct.objective_ik_names_all)) ~= length(Set.optimization.objective_ik)
  error(['Set.optimization.objective_ik enthält unerwarteten Wert. Ist: %s. ' ...
    'Erlaubt: {%s}'], disp_array(Set.optimization.objective_ik, '%s'), ...
    disp_array(defstruct.objective_ik_names_all, '%s'));
end
if ~isempty(intersect(Set.general.eval_figures, {'pareto_desopt', ...
    'pareto_dimsynth_desopt'})) && ~Set.general.debug_desopt
  warning(['Set.general.debug_desopt wurde auf true gesetzt, da damit ', ...
    'erzeugte Daten notwendig für Auswertung sind']);
  Set.general.debug_desopt = true;
end
% Prüfe das Namensformat von Robotern auf der Positiv-Liste
for i = 1:length(Set.structures.whitelist)
  Name_i = Set.structures.whitelist{i};
  expression_P = 'P[\d][RP]+[\d]+[V]?[\d]*G[\d]+P[\d]+A[\d]+';
  expression_S = 'S[\d][RP]+[\d]+[V]?[\d]*';
  match_P = regexp(Name_i,expression_P,'match');
  match_S = regexp(Name_i,expression_S,'match');
  if isempty(match_P) && isempty(match_S)
    error('Roboter %s auf Positiv-Liste entspricht nicht dem Namensformat', Name_i);
  end
end
if ~isempty(Set.structures.repeatlist)
  for j = 1:length(Set.structures.repeatlist)
    assert(isa(Set.structures.repeatlist{j}, 'cell'), ...
      'Eintrag in Set.structures.repeatlist ist kein Cell-Array');
    assert(length(Set.structures.repeatlist{j}) == 2, ...
      'Eintrag in Set.structures.repeatlist hat nicht Dimension 2');
  end
end
if length(Set.structures.whitelist) ~= length(unique(Set.structures.whitelist))
  warning('Die Positiv-Liste enthält %d doppelte Einträge. Werden entfernt.', ...
    length(Set.structures.whitelist)-length(unique(Set.structures.whitelist)));
  Set.structures.whitelist = unique(Set.structures.whitelist);
end
if ~isa(Set.structures.joint_filter, 'cell') % Altes Format kompatibel machen
  Set.structures.joint_filter = {Set.structures.joint_filter};
end
% Bei PKM-Struktursynthese darf nicht frühzeitig bei Singularität abbrechen
if length(Set.optimization.objective) == 1 && ... % ist immer einkriteriell
    any(strcmp(Set.optimization.objective, {'valid_act', 'valid_kin'}))
  Set.optimization.condition_limit_sing = inf; % Überschreibe Standardwert
end
assert(~all(Set.general.taskred_dynprog_numstates == 0), ['Es dürfen nicht', ...
  'alle Einträge in taskred_dynprog_numstates Null sein']);
if strcmp(Set.optimization.objective_ik, 'constant') && Set.general.taskred_dynprog
  cds_log(1, ['Einstellung objective_ik=constant und taskred_dynprog ' ...
    'widersprüchlich. Deaktiviere Dynamische Programmierung']);
  Set.general.taskred_dynprog = false;
  Set.general.taskred_dynprog_and_gradproj = false;
  Set.general.taskred_dynprog_only = false;
end
% Einige eher kosmetische Einstellungen sollten bei Struktursynthese
% deaktiviert bleiben
if length(Set.optimization.objective) == 1 && ... % ist immer einkriteriell
    any(strcmp(Set.optimization.objective, {'valid_act', 'valid_kin'}))
  Set.optimization.prismatic_cylinder_allow_overlength = false;
  Set.optimization.joint_limits_symmetric_prismatic = false;
end
assert(isa(Set.task.DoF, 'logical') && all(size(Set.task.DoF)==[1 6]), ...
  'Set.task.DoF muss 1x6 logical sein');
if ~isempty(Traj.X) && any(abs(Traj.X(1,:)-Traj.XE(1,:))>1e-6)
  error(['Erster Eckpunkt Traj.XE sollte identisch zum Trajektorienstart ', ...
    'Traj.X sein: [%s] vs [%s]'], disp_array(Traj.X(1,:), '%1.1f'), ...
    disp_array(Traj.XE(1,:), '%1.1f'));
end
if isfield(Set.task.payload, 'If') && any(~isnan(Set.task.payload.If)) && any(~isnan(Set.task.payload.Ic))
  warning(['In Set.task.payload ist der Trägheitstensor mit Ic und If ', ...
    'definiert. Sollte nur ein Eintrag sein, damit es nicht widersprüchlich sein kann.'])
end
if Set.general.save_evolution_video
  % Für dieses Bild müssen Bilder des Roboters im Verlauf der Optimierung
  % gespeichert werden
  Set.general.save_robot_details_plot_fitness_file_extensions = unique( ...
    [Set.general.save_robot_details_plot_fitness_file_extensions(:)', {'fig'}]);
  if Set.general.plot_robot_in_fitness < 1e3
    Set.general.plot_robot_in_fitness = 1e3;
  end
end
if isempty(Set.general.save_animation_file_extensions) && ~isempty(Set.general.animation_styles)
  warning('Eingabefeld save_animation_file_extensions ist leer. Keine Animation');
  Set.general.animation_styles = {};
end
if any(strcmp(Set.general.animation_styles, ''))
  error('Einstellung animation_styles enthält leeren Eintrag. Ungültig. Muss leeres Cell sein.');
end
if any(strcmp(Set.general.save_animation_file_extensions, ''))
  error('Einstellung save_animation_file_extensions enthält leeren Eintrag. Ungültig. Muss leeres Cell sein.');
end
if ~isempty(Set.general.save_animation_file_extensions) && isempty(Set.general.animation_styles)
  warning('Eingabefeld animation_styles ist leer. Keine Animation');
  Set.general.save_animation_file_extensions = {};
end
eval_figures_allowed = {'robvisu', 'robvisuanim', 'animation', 'jointtraj', ...
  'pareto', 'pareto_dimsynth_desopt', 'pareto_desopt', 'dynamics', 'dynparvisu', 'optpar', 'springrestpos', ... % cds_vis_results_figures
  'pareto_all_phys', 'pareto_all_fval', 'histogram', 'fitness_various'}; % cds_vis_results
if length(intersect(eval_figures_allowed, Set.general.eval_figures)) ~= ...
    length(Set.general.eval_figures)
  error(['Eingabefeld eval_figures enthält unerwartete Einträge. Ist: ', ...
    '{%s}, Erlaubt: {%s}'], disp_array(Set.general.eval_figures, '%s'), ...
    disp_array(eval_figures_allowed, '%s'));
end
if isnan(Set.general.cluster_maxrobotspernode)
  Set.general.cluster_maxrobotspernode = Set.general.computing_cluster_cores;
end
assert(isa(Set.general.cluster_dependjobs, 'struct'), 'cluster_dependjobs muss Struktur sein');
assert(~isempty(intersect(fields(Set.general.cluster_dependjobs), ...
  {'afterok', 'afternotok', 'afterany'})), 'cluster_dependjobs muss min. ein Feld afterok, afternotok oder afterany haben');
if Set.task.profile ~= 0 % Trajektorie prüfen
  % De-Normalisiere die Trajektorie. Dadurch springen die Euler-Winkel nicht
  % (Auswirkung hauptsächlich optisch in Auswertungen). Winkel größer pi.
  Traj.X(:,4:6) = denormalize_angle_traj(Traj.X(:,4:6));
  % Prüfe Konsistenz des Positionsverlaufs
  if Set.task.profile == 1 % Variante 1: Trapezregel (besser für kontinuierliche Funktionen)
    X_numint = repmat(Traj.X(1,:),size(Traj.X,1),1)+cumtrapz(Traj.t, Traj.XD);
  elseif Set.task.profile == 2 % Variante 2: Kumulierte Summe (besser, falls aus Differenzenquotient)
    X_numint = repmat(Traj.X(1,:),size(Traj.X,1),1) + ...
      cumsum(Traj.XD.*repmat([diff(Traj.t); diff(Traj.t(end-1:end))],1,size(Traj.XD,2)) );
  else
    error('Fall Set.task.profile=%d nicht definiert', Set.task.profile);
  end
  corrX = diag(corr(X_numint, Traj.X));
  corrX(all(abs(X_numint-Traj.X)<1e-6)) = 1;
  assert(all(corrX>0.98), 'eingegebene Trajektorie ist nicht konsistent (X-XD)');
  testX = X_numint - Traj.X; % Korrelation reicht nicht, falls Skalierungsfehler
  if any(abs(testX(:))>1e-2)
    warning('Abweichung zwischen int(XD) und X zu groß: [%s]', disp_array(max(abs(testX)), '%1.1e'));
  end
  if Set.task.profile == 1
    XD_numint = repmat(Traj.XD(1,:),size(Traj.XD,1),1)+cumtrapz(Traj.t, Traj.XDD);
  else
    XD_numint = repmat(Traj.XD(1,:),size(Traj.XD,1),1) + ...
      cumsum(Traj.XDD.*repmat([diff(Traj.t); diff(Traj.t(end-1:end))],1,size(Traj.XD,2)) );
  end
  corrXD = diag(corr(XD_numint, Traj.XD));
  corrXD(all(abs(Traj.XD)<1e-3)) = 1;
  assert(all(corrXD>0.98), 'eingegebene Trajektorie ist nicht konsistent (XD-XDD)');
  testXD = XD_numint - Traj.XD;
  if any(abs(testXD(:))>1e-2)
    warning('Abweichung zwischen int(XDD) und XD zu groß: [%s]', disp_array(max(abs(testXD)), '%1.1f'));
  end
else % Kein Trajektorienprofil gegeben. Prüfe Datenformat
  % Trajektorie in X muss identisch zu der in XE sein (für Implementierung)
  assert(all(size(Traj.X)==size(Traj.XE)), 'Für profile=0 muss Traj.X und Traj.XE gleich sein (Dimension)');
  assert(all(abs(Traj.X(:)-Traj.XE(:))<1e-10), 'Für profile=0 muss Traj.X und Traj.XE gleich sein (Werte)'); 
end
assert(all(size(Traj.X)==size(Traj.XD)), 'Dimension von X und XD nicht gleich');
assert(all(size(Traj.X)==size(Traj.XDD)), 'Dimension von X und XDD nicht gleich');
if ~isfield(Traj, 'IE')
  warning('Eingabe Traj enthält keinen Index-Vektor IE. Annahme: Keine Übereinstimmung XE und X');
  Traj.IE = zeros(size(Traj.XE,1),1);
end
if Set.task.profile ~= 2
  assert(length(Traj.IE)==size(Traj.XE,1), 'IE und XE muss gleiche Dimension haben in Traj.-Var.');
end
assert(max(Traj.IE)<=size(Traj.X,1), 'Index-Vektor IE darf Bereich aus X nicht überschreiten');
test_XEfromIE = Traj.X(Traj.IE(Traj.IE~=0),:) - Traj.XE(Traj.IE~=0,:);
if Set.task.profile ~= 2
  assert(all(abs(test_XEfromIE(:))<1e-10), 'Eckpunkte der Trajektorie X müssen in XE mit IE indiziert werden können');
end
IE_firstzero = find(Traj.IE==0, 1, 'first');
if ~isempty(IE_firstzero)
  assert(all(Traj.IE(IE_firstzero:end)==0), 'die letzten Werte in XE dürfen nicht mehr der Trajektorie zugeordnet sein');
end
IE_lastentry = find(Traj.IE~=0, 1, 'last');
assert(all(diff(Traj.IE(1:IE_lastentry)) > 0), 'Werte in IE müssen monoton steigend sein');
if all(Set.task.DoF(1:5) == [1 1 0 0 0]) % planare Aufgabe: 2T0R, 2T0*R oder 2T1R
  if ~strcmp(Set.structures.mounting_parallel, 'wall')
    assert(all(abs(Traj.X(1,3)- Traj.X(:,3))  < 1e-10) && ...
           all(abs(Traj.XE(1,3)-Traj.XE(:,3)) < 1e-10), ...
      'Planare Aufgabe: z-Position darf sich nicht verändern');
  else
    assert(all(abs(Traj.X(1,2)- Traj.X(:,2))  < 1e-10) && ...
           all(abs(Traj.XE(1,2)-Traj.XE(:,2)) < 1e-10), ...
      'Planare Aufgabe in xz-Wandmontage: y-Position darf sich nicht verändern');
  end
end
% Rast-zu-Rast-Verhalten auch für Nullraumbewegung, wenn gefordert.
% Bestimme die Zeitschritte
if isnan(Set.task.T_dec_ns)
  % Zeit zum Abbremsen der Nullraumbewegung bestimmen.
  Set.task.T_dec_ns = Set.task.vmax / Set.task.amax;
end
if Set.task.profile == 1 && Set.task.T_dec_ns > 0
  nullspace_maxvel_interp = nullspace_maxvel_from_tasktraj(Traj.t, ...
    Traj.IE(Traj.IE~=0), Set.task.vmax/Set.task.amax, Set.task.T_dec_ns , Set.task.Ts);
  Traj.nullspace_maxvel_interp = nullspace_maxvel_interp;
else % Deaktiviere Begrenzung der Geschwindigkeit
  Traj.nullspace_maxvel_interp = zeros(2,0);
end
if ~isfield(Traj, 'Fext'), Traj.Fext = zeros(length(Traj.t), 6); end

if Set.general.create_template_functions && ~Set.general.check_missing_template_functions
  Set.general.check_missing_template_functions = true;
  warning('check_missing_template_functions wurde auf true gesetzt, da create_template_functions gesetzt')
end
%% Menge der Roboter laden
% Nur neu erzeugen, wenn die Liste nicht schon vorgegeben wurde
if ~(Set.general.only_finish_aborted && Set.general.isoncluster) && ... % Abschluss auf Cluster
    ~Set.general.regenerate_summary_only || ... % Nur Bilder (ohne Abschluss)
    (Set.general.only_finish_aborted && Set.general.computing_cluster) % Abschluss lokal. Kein Aufruf zum Hochladen des Abschluss-Auftrags auf das Cluster
  % Bei Fortsetzen der abgebrochenen Berechnung auf dem Cluster nicht
  % notwendig. Sonst schon (lokal oder Hochladen des Abschluss-Jobs).
  % Auch nicht notwendig bei reiner Neu-Erzeugung der Ergebnis-Bilder.
  % Der Fall des Hochladens des Auftrags zum Abschluss auf das Cluster wird
  % mit obiger Logik berücksichtigt (zum Aufteilen auf parallele Jobs)
  t1 = tic();
  if isempty(Structures)
    Structures = cds_gen_robot_list(Set);
  end
  cds_log(1, sprintf('Insgesamt %d Roboter ausgewählt. Dauer: %1.1fs', ...
    length(Structures), toc(t1)));
  if isempty(Structures)
    cds_log(1, 'Keine Strukturen entsprechen den Filterkriterien');
    if ~isempty(Set.structures.whitelist)
      cds_log(1, ['Es wurde eine Positiv-Liste übergeben, aber keine ', ...
        'Strukturen entsprachen den Kriterien. Filter-Liste passt nicht']);
    end
    return
  end
end
%% Ergebnis-Speicherort vorbereiten. Einstellungen speichern.
settingsfile = fullfile(resdir_main, sprintf('%s_settings.mat', ...
  Set.optimization.optname));
if Set.general.only_finish_aborted && (Set.general.isoncluster || ...
    ~Set.general.computing_cluster) % nicht bei hochladen des Jobs aufs Cluster
  % Alte Einstellungsdatei laden. Damit muss nicht die Menge der Strukturen
  % neu erzeugt werden (geht schneller und ist robuster).
  if ~exist(resdir_main, 'file')
    cds_log(1, sprintf(['Nachträglicher Abschluss von %s nicht möglich. ', ...
      'Verzeichnis %s fehlt.'], Set.optimization.optname, resdir_main));
    return
  end
  if ~exist(settingsfile, 'file')
    cds_log(1, sprintf(['Nachträglicher Abschluss von %s nicht möglich. ', ...
      'Datei %s fehlt.'], Set.optimization.optname, settingsfile));
    return
  end
  d = load(settingsfile, 'Set', 'Traj', 'Structures');
  Traj = d.Traj;
  Structures = d.Structures;
  Set_tmp = Set;
  Set = cds_settings_update(d.Set);
  % Überschreibe geladene Einstellungen, damit ein kopierter Ordner vom
  % Cluster auch lokal abgeschlossen werden kann.
  Set.general.only_finish_aborted = true; % Überschreibe geladene Einstellung
  Set.general.parcomp_plot = Set_tmp.general.parcomp_plot;
  Set.general.parcomp_struct = Set_tmp.general.parcomp_struct;
  Set.general.parcomp_maxworkers = Set_tmp.general.parcomp_maxworkers;
  Set.optimization.resdir = Set_tmp.optimization.resdir;
  Set.general.compile_missing_functions = Set_tmp.general.compile_missing_functions;
  Set.general.create_template_functions = Set_tmp.general.create_template_functions;
  Set.general.update_template_functions = Set_tmp.general.update_template_functions;
  Set.general.check_missing_template_functions = Set_tmp.general.check_missing_template_functions;
  cds_log(1, sprintf('Einstellungsdatei %s für Abschluss geladen.', settingsfile));
  % Prüfe, ob der Abschluss noch notwendig ist. Annahme: Ist die Ergebnis-
  % Tabelle einmal erstellt, sind alle einzelnen Roboter abgeschlossen.
  restabfile = fullfile(resdir_main, sprintf('%s_results_table.csv', ...
    Set.optimization.optname));
  if exist(restabfile, 'file') % Prüfe, ob die Tabelle auch vollständig ist
    opts = detectImportOptions(restabfile,'NumHeaderLines',2);
    opts.VariableNamesLine = 1;
    opts.VariableDescriptionsLine = 2;
    ResTab = readtable(restabfile, opts);
    if size(ResTab, 1) == length(Structures)
      cds_log(1, sprintf(['Ergebnis-Tabelle existiert schon mit %d Einträgen. ', ...
        'Kein Abschluss der abgebrochenen Berechnung notwendig.'], size(ResTab, 1)));
      return
    else
      cds_log(1, sprintf(['Ergebnis-Tabelle existiert schon mit %d/%d Einträgen. ', ...
        'Führe Abschluss der Berechnung durch.'], size(ResTab, 1), length(Structures)));
    end
  end
elseif Set.general.regenerate_summary_only && (Set.general.isoncluster || ...
    ~Set.general.computing_cluster)
  % Es sollen nur die Bilder neu generiert werden. Lade die alten Ein-
  % stellungen, damit die Nummern der Roboter nicht geändert werden.
  d = load(settingsfile, 'Set', 'Traj', 'Structures');
  Traj = d.Traj;
  % Übernehme Optionen zur Steuerung der Bildgenerierung aus der Eingabe
  Set_tmp = Set;
  Set = d.Set;
  Set.general.eval_figures = Set_tmp.general.eval_figures;
  Set.general.animation_styles = Set_tmp.general.animation_styles;
  Set.general.parcomp_plot = Set_tmp.general.parcomp_plot;
  Set.general.parcomp_struct = false; % keine Struktursynthese.
  Set.general.regenerate_summary_only = true;
  Set.general.nosummary = Set_tmp.general.nosummary; % für nur Tabelle ohne Bilder
  Set.optimization.resdir = Set_tmp.optimization.resdir; % anders auf Cluster
  Structures = d.Structures;
  cds_log(1, sprintf('Einstellungsdatei %s für Bild-Generierung geladen.', settingsfile));
elseif ~Set.general.computing_cluster % nicht bei Hochladen des Jobs
  % Einstellungen dieser kombinierten Synthese speichern. Damit ist im
  % Nachhinein nachvollziehbar, welche Roboter eventuell fehlen. Bereits hier
  % oben machen. Dann passt die Variable Structures auch für den Fall, dass
  % die Maßsynthese im folgenden Schritt aufgeteilt wird.
  save(settingsfile, 'Set', 'Traj', 'Structures');
end
% Verzeichnisse zum Laden alter Ergebnisse vorbereiten
if Set.optimization.InitPopRatioOldResults > 0
  % Bei Hochladen der Berechnung auf das Cluster darf nicht das lokale
  % Verzeichnis eingetragen werden. Sonst immer das lokale Verzeichnis.
  if ~(Set.general.computing_cluster && ~Set.general.isoncluster)
    Set.optimization.result_dirs_for_init_pop = unique( ... % bei mehrfacher Durchführung von cds_start sonst doppelt
      [Set.optimization.result_dirs_for_init_pop, Set.optimization.resdir]);
  end
else
  Set.optimization.result_dirs_for_init_pop = {};
end
I_keep = true(length(Set.optimization.result_dirs_for_init_pop), 1);
assert(isa(Set.optimization.result_dirs_for_init_pop, 'cell'), ...
  'result_dirs_for_init_pop muss cell-Array sein')
for i = 1:length(Set.optimization.result_dirs_for_init_pop)
  I_keep(i) = exist(Set.optimization.result_dirs_for_init_pop{i}, 'file');
end
if ~all(I_keep)
  warning('Verzeichnis aus result_dirs_for_init_pop existiert nicht: {%s}', ...
    disp_array(Set.optimization.result_dirs_for_init_pop(~I_keep), '%s'));
  Set.optimization.result_dirs_for_init_pop = Set.optimization.result_dirs_for_init_pop(I_keep);
end
if Set.optimization.InitPopRatioOldResults == 0
  % Es wird kein Index alter Ergebnisse benötigt
  Set.optimization.InitPopFromGlobalIndex = false;
end
if length(Set.optimization.result_dirs_for_init_pop) > 1 % min. das Standard-Verzeichnis ist hier aktiv
  % Der globale Index kann aktuell nur benutzt werden, wenn keine zusätz- 
  % lichen Verzeichnisse mit eingelesen werden sollen (Implementierung)
  warning(['InitPopFromGlobalIndex wird deaktiviert, da zusätzliche ', ...
    'Verzeichnisse in result_dirs_for_init_pop angegeben wurden']);
  Set.optimization.InitPopFromGlobalIndex = false;
end
%% Berechnung auf PBS-Cluster vorbereiten und durchführen
if Set.general.computing_cluster
  % Bereite eine Einstellungs-Datei vor
  clusterheaderfile=fullfile(fileparts(which('structgeomsynth_path_init.m')),...
    'dimsynth', 'dimsynth_cluster_header.m');
  if ~exist(clusterheaderfile, 'file')
    error('Datei %s existiert nicht. Muss manuell aus template-Datei erstellt werden.', clusterheaderfile);
  end
  % Erzeuge Liste aller oben zur Optimierung gefundener Roboter
  Names = {};
  for k = 1:length(Structures)
    if ~isempty(Structures{k}.RobName)
      Names = [Names, Structures{k}.RobName]; %#ok<AGROW>
    else
      Names = [Names, Structures{k}.Name]; %#ok<AGROW>
    end
  end
  % Erzeuge Positiv-Liste für Cluster aus bereits ausgelesener Roboter-DB
  % Teile diese Liste so auf, dass mehrere Cluster-Instanzen parallel
  % gestartet werden können.
  I1_Struct = 1:Set.general.cluster_maxrobotspernode:length(Structures);
  if length(I1_Struct) > 1
    cds_log(1, sprintf(['Teile die Optimierung von %d Robotern auf %d parallele Cluster-', ...
      'Instanzen auf'], length(Structures), length(I1_Struct)));
  else
    cds_log(1, sprintf('Lade die Optimierung von %d Robotern auf das Cluster hoch', length(Structures)));
  end
  jobIDs = NaN(2,length(I1_Struct)); % erste Zeile: Produktiv-Job; zweite Zeile: Aufräum-Job
  for kk = 1:length(I1_Struct)
    I1_kk = I1_Struct(kk); % Anfangs-Index in allen Roboter-Namen
    if kk < length(I1_Struct) % Bestimme End-Index
      I2_kk = I1_Struct(kk+1)-1;
    else
      I2_kk = length(Structures);
    end
    if length(I1_Struct) == 1 % Die Rechnung wird nicht aufgeteilt. Lasse den Namen wie er ist
      suffix = '';
    else % Rechnung aufgeteilt auf die verschiedenen Roboter. Suffix für Part-Nummer.
      suffix = sprintf('_p%d',kk);
    end
    computation_name = sprintf('dimsynth_%s_%s%s', ...
      datestr(now,'yyyymmdd_HHMMSS'), Set.optimization.optname, suffix);
    if Set.general.only_finish_aborted
      % Es wird keine Optimierung durchgeführt. Kennzeichnung im Namen.
      computation_name = [computation_name, '_finish']; %#ok<AGROW>
    end
    if kk < Set.general.continue_with_part
      continue; % Falls dieser Job-Teil vorher schon hochgeladen wurde überspringen
    end
    jobdir = tmpDirFcn(true);
    targetfile = fullfile(jobdir, 'dimsynth_start.m'); % Benutze nicht Namen der Optimierung als Dateiname, da Matlab nur bis 64 Zeichen unterstützt
    Set_cluster = Set;
    Set_cluster.optimization.optname = [Set.optimization.optname, suffix]; % sonst wird bei Zerlegung mehrfach der gleiche Name benutzt.
    Set_cluster.general.noprogressfigure = true; % Fortschritts-Bild des PSO auf Cluster nicht notwendig
    Set_cluster.general.computing_cluster = false; % auf Cluster muss "lokal" gerechnet werden
    Set_cluster.general.isoncluster = true; % Damit einige Debug-Bilder reduziert werden können
    Set_cluster.general.parcomp_struct = true; % parallele Berechnung auf Cluster (sonst sinnlos)
    Set_cluster.general.parcomp_plot = true; % paralleles Plotten auf Cluster (ist dort gleichwertig und schneller)
    % Wähle nur einen Bereich aller möglicher Roboter aus für diesen Lauf.
    Structures_cluster = Structures(I1_kk:I2_kk);
    save(fullfile(jobdir, [computation_name,'.mat']), 'Set_cluster', 'Traj', 'Structures_cluster');
    % Schätze die Rechenzeit: Im Mittel 2s pro Parametersatz aufgeteilt auf
    % 12 parallele Kerne, 30min für Bilderstellung und 6h Reserve/Allgemeines
    comptime_est = (Set.optimization.NumIndividuals*(1+Set.optimization.MaxIter)*2 + ...
      30*60)*ceil(length(I1_kk:I2_kk)/12) + 6*3600;
    % Falls Entwurfsoptimierung durchgeführt wird, rechne dort auch noch
    % mit 1s pro Partikel, durchschnittlich 20 Iterationen bei 10% aller
    % Partikel aus der Maßsynthese.
    if ~isempty(Set.optimization.desopt_vars)
      npart = Set.optimization.NumIndividuals*(1+Set.optimization.MaxIter);
      comptime_est = comptime_est + 0.1*npart*1*20;
    end
    if Set.general.only_finish_aborted || Set.general.regenerate_summary_only
      % Es wird keine Optimierung durchgeführt. Überschreibe die vorher
      % berechnete Zeit (nur Bilderstellung).
      comptime_est = ceil(length(I1_kk:I2_kk)/12)*30*60;
    end
    if ~isnan(Set.general.computing_cluster_max_time)
      if length(I1_kk:I2_kk) > Set.general.computing_cluster_cores
        warning(['Keine Nutzung von computing_cluster_max_time möglich ', ...
          '(Jobs nicht voll parallel)']);
      else
        comptime_est = Set.general.computing_cluster_max_time;
      end
    end
    % Matlab-Skript erzeugen
    copyfile(clusterheaderfile, targetfile);
    fid = fopen(targetfile, 'a');
    fprintf(fid, 'tmp=load(''%s'');\n', [computation_name,'.mat']);
    fprintf(fid, 'Set=tmp.Set_cluster;\nTraj=tmp.Traj;\nStructures=tmp.Structures_cluster;\n');
    % Ergebnis-Ordner neu setzen. Ansonsten ist der Pfad des Rechners
    % gesetzt, von dem der Job gestartet wird.
    fprintf(fid, ['Set.optimization.resdir=fullfile(fileparts(', ...
      'which(''structgeomsynth_path_init.m'')),''results'');\n']);
    % Maximale Rechenzeit auf dem Cluster eintragen (in Sekunden).
    % Kann in Optimierung genutzt werden um baldigen Abbruch zu erkennen
    fprintf(fid, 'Set.general.computing_cluster_max_time=%1.0f;\n', comptime_est);
    fprintf(fid, 'Set.general.computing_cluster_start_time=now();\n');
    % Platzhalter-Eintrag, der weiter unten ersetzt wird:
    fprintf(fid, '%% Set.general.only_finish_aborted = true;\n');
    fprintf(fid, 'cds_start(Set, Traj, Structures);\n');
    % Schließen des ParPools auch in Datei hineinschreiben
    fprintf(fid, 'parpool_writelock(''lock'', 300, true);\n');
    fprintf(fid, 'delete(gcp(''nocreate''));\n');
    fprintf(fid, 'parpool_writelock(''free'', 0, true);\n');
    fclose(fid);

    % Matlab-Skript auf Cluster starten (Toolbox muss im Pfad sein).
    ppn = min(length(I1_kk:I2_kk),Set.general.computing_cluster_cores);
    startsettings = Set.general.cluster_dependjobs;
    % zwei Tage lang in 5min-Abständen versuchen (falls Cluster voll und
    % die Jobs nach und nach erst gestartet werden dürfen 
    startsettings.waittime_max = 3600*24*2; %  2 Tage
    startsettings.retry_interval = 60*5; % 5 Minuten
    jobIDs(1,kk) = jobStart(struct( ...
      'name', computation_name, ...
      'nodes', 1, ... Nur einen Knoten pro Job (Synthese profitiert nich von mehr Knoten, dafür mehr parallele Jobs)
      ... % Nur so viele Kerne beantragen, wie auch benötigt werden ("ppn")
      'ppn', ppn, ... % 32 ist max. auf Cluster
      'mem', ceil(16+5*ppn), ... % Hergeleitet aus MaxRSS-Wert aus diversen Job-Status-Emails. 2GB Aufschlag aufgrund wiederholter Speicher-Probleme
      'matFileName', 'dimsynth_start.m', ...
      'locUploadFolder', jobdir, ...
      'time',comptime_est/3600), ... % Angabe in h
      startsettings); % Mögliche Abhängigkeiten (optional)
    assert(jobIDs(1,kk)~=0, 'Fehler beim Starten des Produktiv-Jobs auf Cluster');
    % Zusätzlich direkt das Aufräum-Skript starten. Es ist davon auszugehen, 
    % dass der Job vorzeitig abgebrochen wird, da die Rechenzeit unterschätzt
    % wird.
    if Set.general.only_finish_aborted
      continue % In diesem Fall war der Zweck des Aufrufs schon das Aufräum-Skript
    end
    % Neues Skript vorbereiten (in neuem Ordner)
    jobdir2 = [jobdir, '_finish'];
    copyfile(jobdir, jobdir2); % damit wird m- und mat-Datei direkt übernommen
    computation_name2 = [computation_name,'_finish'];
    targetfile2 = fullfile(jobdir2, 'dimsynth_finish.m');
    movefile(fullfile(jobdir2, 'dimsynth_start.m'), targetfile2);
    f = strrep(fileread(targetfile2), '% Set.general.only_finish_aborted', ...
      'Set.general.only_finish_aborted');
    fid  = fopen(targetfile2,'w'); fprintf(fid,'%s',f); fclose(fid);
    % Skript hochladen mit dem vorherigen Job als Abhängigkeit. Wenn der
    % Job vom PBS abgebrochen wird, wird der Aufräum-Job gestartet. Bei
    % Erfolg verfällt der Aufräum-Job.
    pause(2); % Damit Sekunden-Zeitstempel im Ordernamen unterschiedlich ist
    startsettings.afterany = jobIDs(1,kk);
    jobIDs(2,kk) = jobStart(struct( ...
      'name', computation_name2, ...
      'nodes', 1, ...
      'ppn', min(length(I1_kk:I2_kk),Set.general.computing_cluster_cores), ... % gleiche Anzahl wie oben
      'mem', ceil(16+5*ppn), ... % gleiche Zahl wie oben (da beim Finish-Job die Fitness-Funktion auch einmal ausgeführt wird)
      'matFileName', 'dimsynth_finish.m', ...
      'locUploadFolder', jobdir2, ...
      'time',2), ... % % Geht schnell. Veranschlage 2h. Evtl. länger wegen ParPool-Synchronisation.
      startsettings);
    assert(jobIDs(2,kk)~=0, 'Fehler beim Starten des Finish-Jobs auf Cluster');
    cds_log(1, sprintf(['Berechnung von %d Robotern wurde auf Cluster hochgeladen. Ende. ', ...
      'Die Ergebnisse müssen nach Beendigung der Rechnung manuell heruntergeladen ', ...
      'werden.'], length(I1_kk:I2_kk)));
    if kk < length(I1_Struct) % Damit nicht alle exakt zeitgleich starten; exakt gleichzeitiger, ...
      pause(5); % ... paralleler Start des parpools sowieso nicht möglich
    end
  end
  if length(I1_Struct) > 1 && ~Set.general.only_finish_aborted
    % Bei mehreren aufgeteilten Läufen direkt die Zusammenfassung aller Teile
    % als zusätzlichen Job starten
    computation_name3 = sprintf('dimsynth_%s_%s%s', ...
      datestr(now,'yyyymmdd_HHMMSS'), Set.optimization.optname, '_merge');
    jobdir3 = tmpDirFcn(true);
    targetfile3 = fullfile(jobdir3, [computation_name3,'.m']);
    copyfile(clusterheaderfile, targetfile3);
    fid = fopen(targetfile3, 'a');
    fprintf(fid, ['cds_merge_results(''%s'', struct(''mode'', ''move'', ', ...
      '''create_missing_tables'', true, ''create_pareto_fig'', true, ', ...
      '''delete_parts_dirs'', true));'], Set.optimization.optname);
    fclose(fid);
    % Zusammenfassungs-Job auf Cluster hochladen
    pause(5); % bei sofortigem Start sonst Einfrieren von Matlab
    startsettings.afterany = jobIDs(:)'; % Nur starten, wenn vorherige Produktiv- und Aufräum-Jobs erledigt
    jobID_merge = jobStart(struct( ...
      'name', computation_name3, ...
      'nodes', 1, 'ppn', 1, ... % gleiche Anzahl wie oben
      'matFileName', [computation_name3, '.m'], ...
      'locUploadFolder', jobdir3, ...
      'time',1), ... % Geht schnell
      startsettings);
    assert(jobID_merge~=0, 'Fehler beim Starten des Merge-Jobs auf Cluster');
    cds_log(1, sprintf(['Insgesamt %d Optimierungen mit in Summe %d Robotern hochgeladen. ',...
      'Produktiv-Jobs: [%s], Finish-Jobs: [%s], Merge-Job: %d'], ...
      length(I1_Struct), length(Structures), disp_array(jobIDs(1,:), '%d'), ...
      disp_array(jobIDs(2,:), '%d'), jobID_merge));
    jobID_out = jobID_merge; % Wenn dieser Job fertig ist, ist die Synthese fertig
  else
    % Nur ein paralleler Job. Nehme Job-ID des Finish-Jobs für Rückgabe 
    jobID_out = jobIDs(end);
  end

  return;
end
%% Vorbereitung und Durchführung der lokalen Optimierung
if ~isempty(Set.structures.whitelist)
  if length(Set.structures.whitelist) ~= length(unique(Set.structures.whitelist))
    error('Die Positiv-Liste enthält doppelte Einträge');
  end
  Names_in_Struct = {}; % Es können bei Struktursynthese Strukturen doppelt getestet werden
  for i = 1:length(Structures), Names_in_Struct{i} = Structures{i}.Name; end %#ok<AGROW>
  if Set.general.only_finish_aborted
    % Beim Zusammenfassen werden fehlende Teile ignoriert. Ergänze diese
    % hier wieder. TODO: Abgrenzung Robotername vs Modellname beachten
    Set.structures.whitelist = unique([Set.structures.whitelist, Names_in_Struct]);
  end
  missing_in_robotlist = setdiff(unique(Set.structures.whitelist), unique(Names_in_Struct));
  missing_in_whitelist = setdiff(unique(Names_in_Struct), unique(Set.structures.whitelist));
  if ~any(contains(Set.structures.whitelist, '_')) % TODO: Abgrenzung Modellname und Robotername so, dass Warnungen sinnvoll sind.
  if ~isempty(missing_in_whitelist)
    cds_log(-1, sprintf(['Es wurde eine Positiv-Liste mit %d Einträgen ', ...
      'übergeben, dort fehlen %d der jetzt ermittelten %d Strukturen: %s'], ...
      length(unique(Set.structures.whitelist)), length(missing_in_whitelist), ...
      length(Names_in_Struct), disp_array(missing_in_whitelist, '%s')) );
  end
  if ~isempty(missing_in_robotlist)
    cds_log(-1, sprintf(['Es wurde eine Positiv-Liste mit %d Einträgen ', ...
      'übergeben, aber nur %d dieser Strukturen wurden gewählt. %d fehlen: %s'], ...
      length(Set.structures.whitelist), length(Names_in_Struct{i}), ...
      length(unique(missing_in_robotlist)), disp_array(missing_in_robotlist, '%s')) );
  end
  end
end

if ~isempty(Set.optimization.desopt_vars)
  valid_desopt = false;
  % Zielfunktion oder Nebenbedingung basierend auf Dynamik und diese
  % beeinflussende Entwurfsoptimierung
  if any(strcmp(Set.optimization.desopt_vars, 'linkstrength')) && ...
      (~isempty(intersect(Set.optimization.objective, {'mass', 'energy', 'actforce', ...
      'stiffness', 'materialstress'})) || any(Set.optimization.constraint_obj([1 2 3 5 6])))
    valid_desopt = true;
  end
  if (any(strcmp(Set.optimization.desopt_vars, 'joint_stiffness_qref')) || ...
      any(strcmp(Set.optimization.desopt_vars, 'joint_stiffness'))) && ...
      (Set.optimization.joint_stiffness_active_revolute~=0 || ...
       Set.optimization.joint_stiffness_passive_revolute~=0 || ...
       Set.optimization.joint_stiffness_passive_universal~=0) && ...
      (~isempty(intersect(Set.optimization.objective, {'mass', 'energy', 'actforce', ...
      'materialstress'})) || any(Set.optimization.constraint_obj([1 2 3 5 6])))
    valid_desopt = true;
  end
  if ~valid_desopt
    Set.optimization.desopt_vars = {};
    cds_log(1, sprintf(['Entwurfsoptimierung wurde verlangt, aber keine dafür notwendigen ', ...
      'Zielfunktionen oder Nebenbedingungen definiert. Wurde wieder deaktiviert']));
  end
end
% Optimierung der Strukturen durchführen
if ~Set.general.regenerate_summary_only
  % Vorbereitung: Getrennt für serielle und parallele Roboter
  for type = [0 2] % seriell und PKM
    % Stelle vorher eine Liste von Namen zusammen, um doppelte zu finden.
    Names = {};
    I_type = false(length(Structures), 1);
    for k = 1:length(Structures)
      if Structures{k}.Type == type
        Names = [Names, Structures{k}.Name]; %#ok<AGROW>
        I_type(k) = true;
      end
    end
    if isempty(Names), continue; end
    Structures_Type = Structures(I_type); % Strukturen des gesuchten Typs
    % Duplikate löschen (treten z.B. auf, wenn verschiedene Werte für theta
    % in der Struktursynthese möglich sind)
    [Names, I] = unique(Names);
    Structures_I = Structures_Type(I);
    % Entferne doppelte PKM-Namen, die sich nur durch die Gestell-/Platt-
    % formausrichtung unterscheiden. Die Dateien sind gleich.
    if type == 2
      PNames_Legs = cell(size(Names)); % PKM-Namen bezogen auf Beinketten-Kinematik (ohne G-/P-Nummer)
      SNames_Legs = cell(size(Names)); % Namen der Beinketten
      for i = 1:length(Names) % Bestimme Namen der PKM ohne P-/G-Nummer
        [~,Leg_Names_i,~,~,~,~,~,~,PNames_Legs{i}] = parroblib_load_robot(Names{i}, 0);
        SNames_Legs{i} = Leg_Names_i{1}; % Annahme: symmetrische PKM
      end
      [~,I] = unique(PNames_Legs);
      Names = Names(I);
      SNames_Legs = unique(SNames_Legs);
      Structures_I = Structures_Type(I);
    end
    % Vorlagen-Funktionen neu generieren (falls dort Änderungen gemacht
    % wurden). Die automatische Neugenerierung in der parfor-Schleife
    % funktioniert nicht aufgrund von Dateikonflikten, autom. Ordnerlöschung.
    if Set.general.create_template_functions
      cds_log(1, sprintf('Erstelle kompilierbare Funktionsdateien aus Vorlagen für %d Roboter', length(Names)));
      tplmode = false; % Erzeuge alle Dateien neu (auch wenn schon vorhanden)
    else
      tplmode = true; % Erzeuge nur fehlende Dateien neu (sonst später Fehler)
      if Set.general.check_missing_template_functions
        cds_log(1, sprintf('Prüfe Existenz von Funktionsdateien aus Vorlagen für %d Roboter', length(Names)));
      end
    end
    if Set.general.check_missing_template_functions
      III = 1:length(Names); % Zufällige Reihenfolge, damit besser parallelisierbar (Cluster)
      III = III(randperm(length(III)));
      for i = III
        Structure_i = Structures_I{i};
        if type == 0 % Serieller Roboter
          serroblib_create_template_functions(Names(i), tplmode, false);
        else % PKM
          % Zuerst Funktionen für serielle Beinketten neu generieren.
          % Eine PKM-Funktion (Beinketten-IK) ist davon abhängig.
          [~, LEG_Names] = parroblib_load_robot(Names{i}, 0);
          serroblib_writelock('lock', 'template', 0, 5*60, false);
          serroblib_create_template_functions(LEG_Names(1), tplmode, false);
          serroblib_writelock('free', 'template', 0, 5*60, false);
          % Sperrschutz für PKM-Bibliothek (hauptsächlich für Struktursynthese)
          parroblib_writelock('check', 'csv', Structure_i.DoF, 5*60, false);
          % Die Vorlagen-Funktionen können nicht in Parallelinstanzen
          % gleichzeitig erzeugt werden.
          parroblib_writelock('lock', 'template', Structure_i.DoF, 5*60, false);
          try % Abfangen von Fehlern für Sonderfall in Struktursynthese
            parroblib_create_template_functions(Names(i), tplmode, false);
          catch err % Passiert, wenn bei Struktursynthese Roboter beim Aufräumen nicht in Datenbank ist
            cds_log(-1, sprintf('Fehler bei Funktions-Kompilierung für %s: %s', ...
              Names{i}, err.message));
            for k = 1:length(Structures) % Deaktivieren dieser PKM
              if strcmp(Structures{k}.Name, Names{i})
                Structures{k}.deactivated = true;
              end
            end
          end
          parroblib_writelock('free', 'template', Structure_i.DoF, 5*60, false);
        end
      end
    end
    if Set.general.update_template_functions
      if type == 0
        serroblib_writelock('lock', 'template', 0, 5*60, false);
        serroblib_update_template_functions(Names,Set.general.verbosity>2, ...
          ~Set.general.use_mex);
        serroblib_writelock('free', 'template', 0, 5*60, false);
      else
        % Zuerst die Vorlagen-Funktionen für die seriellen Beinketten
        serroblib_writelock('lock', 'template', 0, 5*60, false);
        serroblib_update_template_functions(SNames_Legs,Set.general.verbosity>2, ...
          ~Set.general.use_mex);
        serroblib_writelock('free', 'template', 0, 5*60, false);
        % Danach die Funktionen für die PKM
        parroblib_writelock('lock', 'template', Structures_I{1}.DoF, 5*60, false);
        parroblib_update_template_functions(Names,Set.general.verbosity>2, ...
          ~Set.general.use_mex);
        parroblib_writelock('free', 'template', Structures_I{1}.DoF, 5*60, false);
      end
    end
    if Set.general.use_mex && Set.general.compile_missing_functions
      % Benötigte Funktionen kompilieren (serielle statt parallele Ausführung)
      % (es wird automatisch der codegen-Ordner gelöscht. Kann bei paralleler
      % Rechnung zu Konflikten führen)
      t1 = tic(); % Beginn der Prüfung auf Datei-Existenz
      t_ll = t1; % Zeitpunkt der letzten Log-Ausgabe diesbezüglich
      % Zufällige Reihenfolge, damit besser parallelisierbar (Cluster). Be-
      % trifft unabhängige Parallelinstanzen von Matlab.
      % Voraussetzung: rng('shuffle') vor Aufruf dieses Skripts
      III = 1:length(Names); 
      III = III(randperm(length(III)));
      for i = III
        if type == 0 % Serieller Roboter
          R = serroblib_create_robot_class(Names{i});
        else % PKM
          parroblib_writelock('check', 'csv', logical(Set.task.DoF), 5*60, false);
          R = parroblib_create_robot_class(Names{i}, '',  1,1);
        end
        % Hierdurch werden fehlende mex-Funktionen kompiliert.
        if type == 2 % keine gleichzeitige mex-Kompilierung gleicher Kinematiken erlauben.
          parroblib_writelock('lock', Names{i}, R.I_EE, 60*60, Set.general.verbosity>2);
          serroblib_writelock('lock', R.Leg(1).mdlname, NaN, 60*60, Set.general.verbosity>2); % Auch SerRobLib für Beinkette sperren
        end
        R.fill_fcn_handles(true, true);
        if type == 2 % Sperrschutz für PKM und Beinkette aufheben
          serroblib_writelock('free', R.Leg(1).mdlname, NaN); 
          parroblib_writelock('free', Names{i}, R.I_EE);
        end
        if toc(t_ll) > 20 || i == III(end)
          cds_log(1, sprintf(['%d/%d Roboter vom Typ %d auf Existenz der ', ...
            'Dateien geprüft. Dauer bis hier: %1.1fs'], ...
            find(III==i,1,'first'), length(Names), type, toc(t1)));
          t_ll = tic();
        end
      end
    end
  end
  if ~Set.general.only_finish_aborted && ~Set.optimization.InitPopFromGlobalIndex
    cds_gen_init_pop_index(Set, Structures);
  end

  % Initialisiere ParPool: Bei paralleler Berechnung dürfen keine Dateien
  % geschrieben werden um Konflikte zu vermeiden
  if Set.general.parcomp_struct && ... % Parallele Rechnung ist ausgewählt
      ~Set.general.regenerate_summary_only && ... % für Bildgenerierung ParComp nicht benötigt
      length(Structures) > 1 && ... % für Optimierung eines Roboters keine parallele Rechnung
      Set.general.parcomp_maxworkers > 0 % Parallele Berechnung auch so deaktivierbar
    Set.general.noprogressfigure = true; % Auf ParFor-Worker nicht notwendig
    % Keine (allgemeinen) mat-Dateien speichern
    Set.general.matfile_verbosity = 0;
    parfor_numworkers = cds_start_parpool(Set);
  else
    parfor_numworkers = 0;
  end
  t1 = tic();
  cds_log(1, sprintf('Starte Schleife über %d Roboter. parfor_numworkers=%d', ...
    length(Structures), parfor_numworkers));
  parfor (i = 1:length(Structures), parfor_numworkers)
    Structure_i = Structures{i};
    if isempty(Structure_i.RobName), RobNameStr = '';
    else, RobNameStr = sprintf('; %s', Structure_i.RobName); end
    % Überspringe Roboter, für die oben ein Fehler aufgetreten ist.
    if isfield(Structure_i, 'deactivated') && Structure_i.deactivated
      cds_log(1, sprintf('Überspringe Roboter %d (%s%s)', i, Structure_i.Name, RobNameStr));
      continue; 
    end
    % Auflösung für Debug-Bilder setzen (wird auf ParPool auf Cluster nicht
    % vererbt aus globalen Einstellungen)
    if parfor_numworkers > 0
      set(0, 'defaultfigureposition', [1 1 1920 1080]);
      set(0, 'defaultfigureunits', 'pixels');
    end
    % Maßsynthese für diesen Roboter starten
    if ~Set.general.only_finish_aborted, mode = 'Maßsynthese'; %#ok<PFBNS>
    else,                                mode = 'Abschluss'; end
    cds_log(1, sprintf('Starte %s für Roboter %d (%s%s)', mode, i, ...
      Structure_i.Name, RobNameStr));
    try
      cds_dimsynth_robot(Set, Traj, Structure_i);
    catch err
      dbgfile = fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
        'tmp', ['cds_dimsynth_robot_error_', Set.optimization.optname, '_', Structure_i.Name '.mat']);
      cds_log(-1, sprintf('Fehler in cds_dimsynth_robot: %s.  %s', ...
        err.message, getReport(err, 'extended')));
      try % Eigentlich kann man in parfor den Workspace nicht speichern (mit direktem Save-Befehl)
        save_debug_state(dbgfile, struct('Set', Set, 'Traj', Traj, ...
          'Structure_i', Structure_i, 'err', err, 'i', i))
        cds_log(0, sprintf('Zustand gespeichert: %s', dbgfile));
      catch err2
        cds_log(0, sprintf('Fehler beim Speichern des Zustands: %s (%s)', err2.message, dbgfile));
      end
      continue
    end
    % Log-Funktion muss erneut initialisiert werden (oben evtl zurückgesetzt)
    cds_log(1, sprintf('Beendet: %s für Roboter %d (%s%s)', mode, i, ...
      Structure_i.Name, RobNameStr), 'amend', Set, struct('Number', 0));
  end
  cds_log(1, sprintf(['Optimierung von %d Robotern abgeschlossen. ', ...
    'Dauer: %1.1fs'], length(Structures), toc(t1)), 'amend', Set, struct('Number', 0));
end
if isempty(Structures)
  % Aufgrund der Filterkriterien wurden keine Roboter verglichen.
  % Auswertung ist nicht sinnvoll.
  return
end

%% Ergebnisse darstellen
t1 = tic();
cds_results_table(Set, Traj, Structures);
cds_log(1, sprintf('Erstellung der Tabellen abgeschlossen. Dauer: %1.1fs', toc(t1)));
t2 = tic();
cds_vis_results(Set, Traj, Structures);
cds_log(1, sprintf('Ergebnis-Visualisierung abgeschlossen. Dauer: %1.1fs', toc(t2)));
cds_create_evolution_videos(Set, Traj, Structures);
lfp = cds_log(1, sprintf(['Ergebnis-Nachverarbeitung von %d Robotern ', ...
  'abgeschlossen. Dauer: %1.1fs'], length(Structures), toc(t1)));
if exist(lfp, 'file')
  gzip(lfp); delete(lfp);
end

end

function save_debug_state(dbgfile, savestruct)
% Der Save-Befehl geht nicht direkt oben in der ParFor-Schleife.
save(dbgfile, '-struct', 'savestruct');
end
