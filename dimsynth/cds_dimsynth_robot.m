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
% 
% TODO:
% * Namen der Optimierungsparameter

function RobotOptRes = cds_dimsynth_robot(Set, Traj, Structure)
%% Debug: 
if Set.general.matfile_verbosity > 0
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_robot1.mat'));
end
% Zum Debuggen:
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_robot1.mat'));

%% Initialisierung
% Charakteristische Länge der Aufgabe
Lref = norm(diff(minmax2(Traj.X(:,1:3)')'));
Structure.Lref = Lref;
%% Roboter-Klasse initialisieren
if Structure.Type == 0 % Seriell
  R = serroblib_create_robot_class(Structure.Name);
  NLEG = 1;
elseif Structure.Type == 2 % Parallel
  % Parameter für Basis-Kopplung einstellen
  p_base = 1.5*Lref;
  if Structure.Coupling(1) == 4
    p_base(2) = 0.4*p_base(1);
    p_base(3) = pi/3;
  end
  % Parameter für Plattform-Kopplung einstellen
  p_platform = 0.75*Lref;
  if Structure.Coupling(2) == 3
    p_platform(2) = 0.5*p_platform(1);
  end
  R = parroblib_create_robot_class(Structure.Name, p_base(:), p_platform(:));
  NLEG = R.NLEG;
else
  error('Typ-Nummer nicht definiert');
end
R.fill_fcn_handles(Set.general.use_mex, true);

for i = 1:NLEG
  if Structure.Type == 0
    R_init = R;
  else
    R_init = R.Leg(i);
  end
  R_init.gen_testsettings(false, true); % Setze Kinematik-Parameter auf Zufallswerte
  % Gelenkgrenzen setzen: Schubgelenke (Verfahrlänge nicht mehr als "zwei
  % mal schräg durch Arbeitsraum" (char. Länge))
  R_init.qlim(R_init.MDH.sigma==1,:) = repmat([-0.5*Lref, 1.5*Lref],sum(R_init.MDH.sigma==1),1);
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
  % Dynamikparameter setzen
  R_init.DynPar.mode = 4; % Benutze Minimalparameter-Dynamikfunktionen
  R_init.DesPar.joint_type((1:R.NJ)'==1&R.MDH.sigma==1) = 4; % Linearführung erste Achse
  R_init.DesPar.joint_type((1:R.NJ)'~=1&R.MDH.sigma==1) = 5; % Schubzylinder weitere Achse
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
  plim = NaN(length(R_pkin.pkin),2);
  for i = 1:size(plim,1)
    if R_pkin.pkin_types(i) == 1 || R_pkin.pkin_types(i) == 3 || R_pkin.pkin_types(i) == 5
      % Winkel-Parameter
      plim(i,:) = [-pi, pi];
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
Structure.xT_mean = NaN(3,1);
if Set.optimization.movebase
  % Berechne Mittelpunkt der Aufgabe
  xT_mean = mean(minmax2(Traj.X(:,1:3)')')';
  Structure.xT_mean = xT_mean;
  
  nvars = nvars + sum(Set.structures.DoF(1:3)); % Verschiebung um translatorische FG der Aufgabe
  vartypes = [vartypes; 2*ones(sum(Set.structures.DoF(1:3)),1)];
  if Structure.Type == 0 % Seriell
    % TODO: Stelle den seriellen Roboter vor die Aufgabe
    varlim = [varlim; repmat([-1, 1], sum(Set.structures.DoF(1:3)), 1)];
  else % Parallel
    % Stelle den parallelen Roboter in/über die Aufgabe
    % Bei Parallelen Robotern ist der Arbeitsraum typischerweise in der
    % Mitte des Gestells (bezogen auf x-y-Ebene). Daher müssen die Grenzen
    % nicht so weit definiert werden: 20% der Referenzlänge um Mittelpunkt
    % der Aufgabe. Annahme: x-/y-Komponente werden immer optimiert.
    varlim = [varlim; repmat([-0.2, 0.2],2,1)]; % xy-Komponenten
    % Die z-Komponente der Basis kann mehr variieren (hängender Roboter)
    varlim = [varlim; repmat([-1, 1], sum(Set.structures.DoF(3)), 1)];
  end
  for i = find(Set.structures.DoF(1:3))
    varnames = {varnames{:}, sprintf('base %s', char(119+i))}; %#ok<CCAT>
  end
else
  % Setze Standard-Werte für Basis-Position fest
  if Structure.Type == 0 % Seriell
    % TODO: Stelle den seriellen Roboter vor die Aufgabe
    r_W_0 = [-0.4*Lref;-0.4*Lref;0];
    if Set.structures.DoF(3) == 1
      r_W_0(3) = -0.7*Lref; % Setze Roboter-Basis etwas unter die Aufgabe
    end
  else % Parallel
    r_W_0 = zeros(3,1);
    if Set.structures.DoF(3) == 1
      r_W_0(3) = -0.7*Lref; % Setze Roboter mittig unter die Aufgabe (besser sichtbar)
    end
  end
  R.update_base(r_W_0);
end

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

% Basis-Koppelpunkt Positionsparameter (z.B. Gestelldurchmesser)
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
    % Automatische Einstellung: fünf-fache spezifische Länge als Basis-Durchmesser
    varlim = [varlim; [0.1,5]];
  end
  varnames = {varnames{:}, 'base param'}; %#ok<CCAT>
end

% Plattform-Koppelpunkt Positionsparameter (z.B. Plattformdurchmesser)
if Structure.Type == 2 && Set.optimization.platform_size
  nvars = nvars + 1;
  vartypes = [vartypes; 7];
  if all(~isnan(Set.optimization.platform_size_limits))
    % Nehme absolute Werte (vorgegeben durch Benutzer)
    varlim = [varlim; Set.optimization.platform_size_limits];
  else
    % Automatische Einstellung: Bezogen auf Gestelldurchmesser
    % max. zwei-facher Gestelldurchmesser als Plattformdurchmesser
    varlim = [varlim; [0.1,2]]; 
  end
  varnames = {varnames{:}, 'platform param'}; %#ok<CCAT>
end
if length(vartypes) ~= length(varnames), error('Abgespeicherte Variablennamen stimmen scheinbar nicht'); end
% Gestell-Morphologie-Parameter (z.B. Gelenkpaarabstand).
% Siehe align_base_coupling.m
if Structure.Type == 2 && Set.optimization.base_morphology
  if R.DesPar.base_method == 1 % keine Parameter bei Kreis
  elseif R.DesPar.base_method == 4
    nvars = nvars + 1;
    vartypes = [vartypes; 8];
    varlim = [varlim; [0.2,0.8]]; % Gelenkpaarabstand. Relativ zu Gestell-Radius.
    nvars = nvars + 1;
    vartypes = [vartypes; 8];
    % Die Steigung wird gegen die Senkrechte gezählt. Damit die erste Achse
    % nach unten zeigt, muss der Winkel größe 90° sein
    varlim = [varlim; [pi/4,3*pi/4]]; % Steigung Pyramide; Winkel in rad (Steigung nach unten und oben ergibt Sinn)
    varnames = {varnames{:}, 'base_morph'}; %#ok<CCAT>
  else
    error('base_morphology Nicht implementiert');
  end
end

% Plattform-Morphologie-Parameter (z.B. Gelenkpaarabstand).
% Siehe align_platform_coupling.m
if Structure.Type == 2 && Set.optimization.platform_morphology
  if R.DesPar.platform_method == 1 % keine Parameter bei Kreis
  elseif R.DesPar.platform_method == 3
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
if length(vartypes) ~= length(varnames), error('Abgespeicherte Variablennamen stimmen scheinbar nicht'); end
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

%% Anfangs-Population generieren
% TODO: Existierende Roboter einfügen

NumIndividuals = Set.optimization.NumIndividuals;
InitPop = repmat(varlim(:,1)', NumIndividuals,1) + rand(NumIndividuals, nvars) .* ...
                        repmat(varlim(:,2)'-varlim(:,1)',NumIndividuals,1);

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
fitnessfcn=@(p)cds_fitness(R, Set, Traj, Structure, p(:));
f_test = fitnessfcn(InitPop(1,:)'); %#ok<NASGU> % Testweise ausführen

%% PSO-Aufruf starten
if Set.general.matfile_verbosity > 0
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_robot2.mat'));
end
if false
  % Falls der PSO abbricht: Zwischenergebnisse laden und daraus Endergebnis
  % erzeugen. Dafür ist ein manuelles Eingreifen mit den Befehlen in diesem
  % Block erforderlich. Danach kann die Funktion zu Ende ausgeführt werden.
  load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_robot2.mat')); %#ok<UNRCH>
  filelist_tmpres = dir(fullfile(resdir, 'PSO_Gen*_AllInd_iter.mat'));
  lastres = load(fullfile(resdir, filelist_tmpres(end).name));
  p_val = lastres.optimValues.bestx;
  fval = lastres.optimValues.bestfval;
  exitflag = -6;
else
  % PSO wird ganz normal ausgeführt.
  [p_val,fval,exitflag] = particleswarm(fitnessfcn,nvars,varlim(:,1),varlim(:,2),options);
end
if Set.general.matfile_verbosity > 0
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_robot3.mat'));
end
% Debug:
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_robot3.mat'));
%% Nachverarbeitung der Ergebnisse
% Fitness-Funktion nochmal mit besten Parametern aufrufen. Dadurch werden
% die Klassenvariablen (R.pkin, R.DesPar.seg_par, ...) aktualisiert
for i = 1:Set.general.max_retry_bestfitness_reconstruction
  % Mehrere Versuche vornehmen, da beim Umklappen der Roboterkonfiguration
  % andere Ergebnisse entstehen können
  fval_test = fitnessfcn(p_val');
  if fval_test~=fval
    if fval_test < fval
      t = sprintf('Der neue Wert (%1.1f) ist besser als der alte (%1.1f).', fval_test, fval);
    else
      t = sprintf('Der alte Wert (%1.1f) ist besser als der neue (%1.1f).', fval, fval_test);
    end
    warning('Bei nochmaligem Aufruf der Fitness-Funktion kommt nicht der gleiche Wert heraus (Versuch %d). %s', i, t);
    if fval_test < fval
      fprintf('Nehme den besseren neuen Wert ...\n');
      break;
    end
  else
    if i > 1, fprintf('Zielfunktion konnte nach %d Versuchen rekonstruiert werden\n', i); end
    break;
  end
end
% Grenzen aus diesem letzten Aufruf bestimmen
if Structure.Type == 0 
  Structure.qlim = R.qlim;
else
  Structure.qlim = cat(1, R.Leg.qlim);
end

% Berechne Inverse Kinematik zu erstem Bahnpunkt
Traj_0 = cds_rotate_traj(Traj, R.T_W_0);
% q0 = Structure.qlim(:,1) + rand(R.NJ,1).*(Structure.qlim(:,2)-Structure.qlim(:,1));
if Structure.Type == 0 % Seriell
  % Benutze Referenzpose die bei obigen Zielfunktionsaufruf gespeichert wurde
  [q, Phi] = R.invkin2(Traj_0.XE(1,:)', R.qref);
else % Parallel
  [q, Phi] = R.invkin_ser(Traj_0.XE(1,:)', cat(1,R.Leg.qref));
end
if ~strcmp(Set.optimization.objective, 'valid_act') && any(abs(Phi)>1e-8)
  warning('PSO-Ergebnis für Startpunkt nicht reproduzierbar (ZB-Verletzung)');
end
% Berechne IK der Bahn (für spätere Visualisierung)
if Structure.Type == 0 % Seriell
  s = struct('normalize', false, 'retry_limit', 1);
  [Q, QD, QDD, PHI] = R.invkin2_traj(Traj_0.X, Traj.XD, Traj.XDD, Traj.t, q, s);
else % Parallel
  s = struct('debug', false, 'retry_limit', 1);
  [Q, QD, QDD, PHI] = R.invkin_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD, Traj_0.t, q, s);
end

if ~strcmp(Set.optimization.objective, 'valid_act') && ...
    (any(abs(PHI(:))>1e-8) || any(isnan(Q(:))))
  warning('PSO-Ergebnis für Trajektorie nicht reproduzierbar oder nicht gültig (ZB-Verletzung)');
end
%% Ausgabe der Ergebnisse
RobotOptRes = struct( ...
  'fval', fval, ...
  'R', R, ...
  'p_val', p_val, ...
  'p_limits', varlim, ...
  'options', options, ...
  'q0', q, ...
  'Traj_Q', Q, ...
  'Traj_QD', QD, ...
  'Traj_QDD', QDD, ...
  'Traj_PHI', PHI, ...
  'exitflag', exitflag, ...
  'Structure', Structure, ...
  'fitnessfcn', fitnessfcn);
% Debug: Durch laden dieser Ergebnisse kann nach Abbruch des PSO das
% Ergebnis trotzdem geladen werden
if Set.general.matfile_verbosity > 0
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_robot4.mat'));
end