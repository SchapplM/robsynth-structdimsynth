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

%% Roboter-Klasse initialisieren
if Structure.Type == 0 % Seriell
  R = serroblib_create_robot_class(Structure.Name);
  R.gen_testsettings(true, true);
  R.fill_fcn_handles(true, false); % Nutze mex-Funktionen
  R.qlim(R.MDH.sigma==1,:) = repmat([-Lref, Lref],sum(R.MDH.sigma==1),1); % Schubgelenk
  R.qlim(R.MDH.sigma==0,:) = repmat([-pi, pi],    sum(R.MDH.sigma==0),1); % Drehgelenk
elseif Structure.Type == 1  % Parallel
  R = parroblib_create_robot_class(Structure.Name, 2, 1);
else
  error('Typ-Nummer nicht definiert');
end

%% Optimierungsparameter festlegen
nvars = 0; vartypes = []; varlim = [];

% Referenzlänge
nvars = nvars + 1;
vartypes = [vartypes; 0];
varlim = [varlim; [-2*Lref, 2*Lref]];

% Strukturparameter der Kinematik
if Structure.Type == 0 % Seriell
  Ipkinrel = R.get_relevant_pkin(Set.structures.DoF);
  pkin_init = R.pkin;
  pkin_init(~Ipkinrel) = 0; % nicht relevant Parameter Null setzen
  R.update_mdh(pkin_init);
  nvars = nvars + sum(Ipkinrel);
  vartypes = [vartypes; 1*ones(sum(Ipkinrel),1)];
  % Grenzen für Kinematikparameter anhand der Typen bestimmen
  types = R.get_pkin_parameter_type();
  plim = NaN(length(R.pkin),2);
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
elseif Structure.Type == 1  % Parallel
  error('Noch nicht definiert');
end

% Basis-Position
if Set.optimization.movebase
  nvars = nvars + sum(Set.structures.DoF(1:3)); % Verschiebung um translatorische FG der Aufgabe
  vartypes = [vartypes; 2*ones(sum(Set.structures.DoF(1:3)),1)];
  varlim = [varlim; repmat([-1, 1], sum(Set.structures.DoF(1:3)), 1)]; % bezogen auf Lref
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
Set.optimization.vartypes = vartypes;

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
if Structure.Type == 0 % Seriell
  R = cds_update_robot_parameters(R, Set, p_val);
  Traj_0 = cds_rotate_traj(Traj, R.T_W_0);
  [q, Phi] = R.invkin2(Traj_0.XE(1,:)', rand(R.NQJ,1));
  if any(abs(Phi)>1e-8)
    error('PSO-Ergebnis für Startpunkt nicht reproduzierbar');
  end
  % Berechne IK der Bahn (für spätere Visualisierung)
  [Q, ~, ~, PHI] = R.invkin2_traj(Traj_0.X, Traj.XD, Traj.XDD, Traj.t, q);
  if any(abs(PHI(:))>1e-8)
    error('PSO-Ergebnis für Trajektorie nicht reproduzierbar');
  end
else
  error('Nicht implementiert');
end
%% Ausgabe der Ergebnisse
RobotOptRes = struct( ...
  'fval', fval, ...
  'R', R, ...
  'p_val', p_val, ...
  'q0', q, ...
  'Q', Q, ...
  'vartypes', vartypes, ...
  'exitflag', exitflag, ...
  'varlim', varlim);
