% Maßsynthese für eine Roboterstruktur

% TODO:
% * Namen der Optimierungsparameter

function RobotOptRes = cds_dimsynth_robot(Set, Traj, Structure)
%% Debug: 
save(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_robot1.mat'));
% error('Halte hier');
% load(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_robot1.mat'));

%% Initialisierung
% Charakteristische Länge der Aufgabe
Lref = sum(diff(minmax2(Traj.X(:,1:3)')'));
Structure.Lref = Lref;
%% Roboter-Klasse initialisieren
if Structure.Type == 0 % Seriell
  R = serroblib_create_robot_class(Structure.Name);
  R.gen_testsettings(true, true); % Setze Parameter auf Zufallswerte
  R.fill_fcn_handles(Set.general.use_mex, false);
  R.qlim(R.MDH.sigma==1,:) = repmat([-2*Lref, 2*Lref],sum(R.MDH.sigma==1),1); % Schubgelenk
  R.qlim(R.MDH.sigma==0,:) = repmat([-pi, pi],    sum(R.MDH.sigma==0),1); % Drehgelenk
elseif Structure.Type == 2 % Parallel
  R = parroblib_create_robot_class(Structure.Name, 2, 1);
  R.Leg(1).gen_testsettings(true, true); % Setze Parameter auf Zufallswerte
  for i = 1:R.NLEG
    R.Leg(i).fill_fcn_handles(Set.general.use_mex, true); % Nutze mex-Funktionen für Beinketten-Funktionen
  end
  R.fill_fcn_handles(Set.general.use_mex, true); % Nutze mex-Funktionen für PKM-Funktionen
  for i = 1:R.NLEG
    R.Leg(i).qlim(R.Leg(i).MDH.sigma==1,:) = repmat([-2*Lref, 2*Lref],sum(R.Leg(i).MDH.sigma==1),1); % Schubgelenk
    R.Leg(i).qlim(R.Leg(i).MDH.sigma==0,:) = repmat([-pi, pi],    sum(R.Leg(i).MDH.sigma==0),1); % Drehgelenk
  end
else
  error('Typ-Nummer nicht definiert');
end

%% Optimierungsparameter festlegen
nvars = 0; vartypes = []; varlim = [];

% Roboterskalierung
% Skalierung für Optimierung ist immer positiv und im Verhältnis zur
% Aufgaben-/Arbeitsraumgröße. Darf nicht Null werden.
nvars = nvars + 1;
vartypes = [vartypes; 0];
varlim = [varlim; [1e-3*Lref, 2*Lref]];

% Strukturparameter der Kinematik
if Structure.Type == 0 || Structure.Type == 2
  if Structure.Type == 0 % Seriell
    R_pkin = R;
  else  % Parallel
    R_pkin = R.Leg(1);
  end
  Ipkinrel = R_pkin.get_relevant_pkin(Set.structures.DoF);
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
  types = R_pkin.get_pkin_parameter_type();
  plim = NaN(length(R_pkin.pkin),2);
  for i = 1:size(plim,1)
    if types(i) == 1 || types(i) == 3 || types(i) == 5
      % Winkel-Parameter
      plim(i,:) = [-pi, pi];
    elseif types(i) == 2 || types(i) == 4 || types(i) == 6
      % Maximale Länge der einzelnen Segmente
      plim(i,:) = [-1, 1]; % in Optimierung bezogen auf Lref
    else
      error('Parametertyp nicht definiert');
    end
  end
  varlim = [varlim; plim(Ipkinrel,:)];
else
  error('Noch nicht definiert');
end

% Basis-Position. Die Komponenten in der Optimierungsvariablen sind nicht
% bezogen auf die Skalierung. Die Position des Roboters ist nur in einigen
% Fällen in Bezug zur Roboterskalierung (z.B. z-Komponente bei hängendem
% Roboter, Entfernung bei seriellem Roboter)
if Set.optimization.movebase
  % Berechne Mittelpunkt der Aufgabe
  xT_mean = mean(minmax2(Traj.X(:,1:3)')');
  xT_mean_norm = xT_mean/Lref;
  
  nvars = nvars + sum(Set.structures.DoF(1:3)); % Verschiebung um translatorische FG der Aufgabe
  vartypes = [vartypes; 2*ones(sum(Set.structures.DoF(1:3)),1)];
  if Structure.Type == 0 % Seriell
    % TODO: Stelle den seriellen Roboter vor die Aufgabe
    varlim = [varlim; repmat([-1, 1], sum(Set.structures.DoF(1:3)), 1)];
  else % Parallel
    % Stelle den parallelen Roboter in/über die Aufgabe
    % Bei Parallelen Robotern ist der Arbeitsraum typischerweise in der
    % Mitte des Gestells (bezogen auf x-y-Ebene). Daher müssen die Grenzen
    % nicht so weit definiert werden
    varlim = [varlim; repmat([-0.2+xT_mean(1), 0.2+xT_mean(2)], sum(Set.structures.DoF(1:2)), 1)];
    % Die z-Komponente der Basis kann mehr variieren (hängender Roboter)
    varlim = [varlim; repmat([-1, 1]+xT_mean_norm(3), sum(Set.structures.DoF(3)), 1)];
  end
end

% EE-Verschiebung
if Set.optimization.ee_translation
  nvars = nvars + sum(Set.structures.DoF(1:3)); % Verschiebung des EE um translatorische FG der Aufgabe
  vartypes = [vartypes; 3*ones(sum(Set.structures.DoF(1:3)),1)];
  varlim = [varlim; repmat([-1, 1], sum(Set.structures.DoF(1:3)), 1)]; % bezogen auf Lref
end

% EE-Rotation
if Set.optimization.ee_rotation
  error('EE-Rotation als Parameter noch nicht definert');
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
end

% Basis-Koppelpunkt Positionsparameter (z.B. Gestelldurchmesser)
if Structure.Type == 2
  % TODO: Die Anzahl der Positionsparameter könnte sich evtl ändern
  % Eventuell ist eine Abgrenzung verschiedener Basis-Anordnungen sinnvoll
  nvars = nvars + 1;
  vartypes = [vartypes; 6];
  % TODO: Untergrenze muss noch sinnvoll gewählt werden (darf nicht Null
  % sein)
  varlim = [varlim; [0.1,5]]; % fünf-fache spezifische Länge als Basis-Durchmesser
end

% Plattform-Koppelpunkt Positionsparameter (z.B. Plattformdurchmesser)
% Bezogen auf Gestelldurchmesser
if Structure.Type == 2
  nvars = nvars + 1;
  vartypes = [vartypes; 7];
  % TODO: Untergrenze muss noch sinnvoll gewählt werden (darf nicht Null
  % sein)
  varlim = [varlim; [0.1,2]]; % max. zwei-facher Gestelldurchmesser als Plattformdurchmesser
end

% Variablen-Typen speichern
Structure.vartypes = vartypes;
%% Anfangs-Population generieren
% TODO: Existierende Roboter einfügen

NumIndividuals = Set.optimization.NumIndividuals;
InitPop = repmat(varlim(:,1)', NumIndividuals,1) + rand(NumIndividuals, nvars) .* ...
                        repmat(varlim(:,2)'-varlim(:,1)',NumIndividuals,1);

%% PSO-Einstellungen festlegen
options = optimoptions('particleswarm');
options.Display='iter';
options.MaxIter = Set.optimization.MaxIter; %70 100 % in GeneralConfig
options.StallIterLimit = 2;
options.SwarmSize = NumIndividuals;
options.ObjectiveLimit = 10; % Damit es schnell vorbei ist
% options.ObjectiveLimit = 0; % Kein Limit
options.InitialSwarmMatrix = InitPop;

%% Fitness-Funktion initialisieren
if Structure.Type == 0 % Seriell
  fitnessfcn=@(p)cds_dimsynth_fitness_ser_plin(R, Set, Traj, Structure, p);
elseif Structure.Type == 2 % Parallel
  fitnessfcn=@(p)cds_dimsynth_fitness_par(R, Set, Traj, Structure, p);
else
  error('Noch nicht definiert');
end
f_test = fitnessfcn(InitPop(1,:)'); % Testweise ausführen

%% PSO-Aufruf starten
[p_val,fval,exitflag, output] = particleswarm(fitnessfcn,nvars,varlim(:,1),varlim(:,2),options);
save(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_robot2.mat'));
% Debug:
% load(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_robot2.mat'));
%% Nachverarbeitung der Ergebnisse
fval_test = fitnessfcn(p_val');
% Berechne Inverse Kinematik zu erstem Bahnpunkt
R = cds_update_robot_parameters(R, Set, Structure, p_val);
Traj_0 = cds_rotate_traj(Traj, R.T_W_0);
if Structure.Type == 0 % Seriell
  [q, Phi] = R.invkin2(Traj_0.XE(1,:)', rand(R.NQJ,1));
else % Parallel
  [q, Phi] = R.invkin_ser(Traj_0.XE(i,:)', rand(R.NJ,1));
end
if any(abs(Phi)>1e-8)
  warning('PSO-Ergebnis für Startpunkt nicht reproduzierbar (ZB-Verletzung)');
end
% Berechne IK der Bahn (für spätere Visualisierung)
if Structure.Type == 0 % Seriell
  [Q, ~, ~, PHI] = R.invkin2_traj(Traj_0.X, Traj.XD, Traj.XDD, Traj.t, q);
else % Parallel
  s = struct('debug', false, 'retry_limit', 1);
  [Q, ~, ~, PHI] = R.invkin_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD, Traj_0.t, q, s);
end
if any(abs(PHI(:))>1e-8) || any(isnan(Q(:)))
  warning('PSO-Ergebnis für Trajektorie nicht reproduzierbar oder nicht gültig (ZB-Verletzung)');
end
%% Ausgabe der Ergebnisse
RobotOptRes = struct( ...
  'fval', fval, ...
  'R', R, ...
  'p_val', p_val, ...
  'p_types', vartypes, ...
  'p_limits', varlim, ...
  'options', options, ...
  'q0', q, ...
  'Traj_Q', Q, ...
  'Traj_PHI', PHI, ...
  'vartypes', vartypes, ...
  'exitflag', exitflag, ...
  'varlim', varlim, ...
  'fitnessfcn', fitnessfcn);
