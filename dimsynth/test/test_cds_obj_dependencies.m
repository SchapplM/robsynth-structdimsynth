% Testskript für die Funktion cds_obj_dependencies und cds_obj_dependencies_regmult

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
for robtype = [0 2] % Seriell und PKM
%% Durchsuche alle Ergebnisordner
% Lade einen beliebigen alten Versuch
resdir = fullfile(fileparts(which('structgeomsynth_path_init.m')), 'results');
optdirs = dir(fullfile(resdir, '*'));
found = false;
fprintf(['Gehe %d Ordner durch um bisherige Ergebnisse der Maßsynthese zu ', ...
  'durchsuchen (Typ=%d)\n'], length(optdirs), robtype);
for i = length(optdirs):-1:1 % Unterordner durchgehen. Letzte zuerst.
  dirname_i = fullfile(resdir, optdirs(i).name);
  % Aktuellen Roboter suchen
  resfiles = dir(fullfile(dirname_i, 'Rob*_Endergebnis.mat'));
  for j = 1:length(resfiles)
    resfilename_j_details = strrep(resfiles(j).name, 'Endergebnis', 'Details');
    sflist = dir(fullfile(dirname_i, '*_settings.mat'));
    if length(sflist) ~= 1
      break % Mehr als eine oder keine Einstellungsdatei. Ungültig.
    end
    if ~exist(fullfile(dirname_i, resfilename_j_details), 'file')
      break; % es fehlen die Detail-Ergebnisse
    end
    d1 = load(fullfile(dirname_i, resfiles(j).name));
    if d1.RobotOptRes.Structure.Type ~= robtype
      continue
    end
    if d1.RobotOptRes.fval < 1e3
      found = true;
      break;
    end
  end
  if found, break; end
end
if ~found
  warning('Keine einzige gültige Ergebnisdatei gefunden');
  continue
end
%% Lade gewähltes Ergebnis und bereite vor
fprintf('Lade Ergebnisse aus %s zur Auswertung der Dynamik-Funktionen\n', ...
  optdirs(i).name);
settings = load(fullfile(dirname_i, sflist(1).name));
Set = settings.Set;
Traj_W = settings.Traj;
d2 = load(fullfile(dirname_i, resfilename_j_details));
Structure = d1.RobotOptRes.Structure;
Q = d2.RobotOptDetails.Traj_Q;
QD = d2.RobotOptDetails.Traj_QD;
QDD = d2.RobotOptDetails.Traj_QDD;
R = d2.RobotOptDetails.R;
if R.Type == 0
  serroblib_addtopath({Structure.Name});
else
  parroblib_addtopath({Structure.Name});
end
Traj_0 = cds_transform_traj(R, Traj_W);
if R.Type == 0
  Jinv_ges = [];
else
  Jinv_ges = NaN(size(Q,1), sum(R.I_EE)*R.NJ);
  for i = 1:size(Q,1)
    q_i = Q(i,:)';
    x_i = Traj_0.X(i,:)';
    Phi_q = R.constr4grad_q(q_i);
    Phi_x = R.constr4grad_x(x_i);
    J_x_inv = -Phi_q \ Phi_x;
    Jinv_ges(i,:) = J_x_inv(:);
  end
end

%% Testfälle durchgehen
% Erstelle Matrix mit allen möglichen Kombinationen der Einstellungen
v = [false;true];
M = allcomb(flipud(v),v,v,v,v,v,v);
% Deaktiviere die Marker für Federkraft, wenn keine Gelenkfedern gesetzt
% sind.
M(M(:,7)==false,5) = false; % calc_spring_reg
M(M(:,7)==false,6) = false; % calc_spring_act
M = unique(M, 'rows');
fprintf('Prüfe cds_obj_dependencies für %d verschiedene Fälle\n', size(M,1));
t0 = tic();
for i = 1:size(M,1)
  t1 = tic();
  % Einstellungen in Variable schreiben
  Set.general.debug_calc = M(i,1);
  Structure.calc_cut = M(i,2);
  Structure.calc_dyn_reg = M(i,3);
  Structure.calc_dyn_act = M(i,4);
  Structure.calc_spring_reg = M(i,5);
  Structure.calc_spring_act = M(i,6);
  Set.optimization.joint_stiffness_passive_revolute = M(i,7);
  % Plausibilitätsprüfungen
  if Structure.calc_spring_act && ~Structure.calc_dyn_act
    % Fall nicht sinnvoll. Wenn Antriebskraft für Feder berechnet wird,
    % wird auch die Antriebskraft für die Dynamik berechnet.
    continue
  end
  if Structure.calc_dyn_act && ~Structure.calc_spring_act && ...
      (Set.optimization.joint_stiffness_active_revolute ~= 0 || ...
       Set.optimization.joint_stiffness_passive_revolute ~= 0 || ...
       Set.optimization.joint_stiffness_passive_universal ~= 0)
    % Wenn die Dynamik berechnet wird und bei vorhandener Federsteifigkeit
    % der Effekt nicht berücksichtigt wird, ist das Ergebnis nicht plausibel
    continue
  end
  if ~Structure.calc_spring_act && Structure.calc_spring_reg
    % Regressorform wird berechnet, ohne das Ausdruck selbst gefordert wird
    continue
  end
  if Structure.calc_cut && ~Structure.calc_dyn_act
    % Damit die Schnittkräfte berechnet werden können, muss die Dynamik
    % berechnet werden.
    continue
  end
  if Structure.Type == 0 && ... % Für serielle Roboter keine Federn
      (Structure.calc_spring_act || Structure.calc_spring_reg)
    continue % noch nicht implementiert
  end
  % Zu prüfende Funktionen aufrufen und Ergebnisse validieren
  data_dyn = cds_obj_dependencies(R, Traj_W, Set, Structure, Q, QD, QDD, Jinv_ges);
  if Structure.calc_dyn_reg && isfield(data_dyn, 'TAU')
    error('cds_obj_dependencies: Dynamik-Regressorform soll berechnet werden, es wird aber auch die vollständige Dynamik gespeichert');
  end
  if Structure.calc_spring_reg && isfield(data_dyn, 'TAU_spring')
    error('cds_obj_dependencies: Regressorform der Federkraft soll berechnet werden, es wird aber auch der vollständige Momentenvektor gespeichert');
  end
  if Structure.calc_spring_reg && Structure.calc_cut && ~isfield(data_dyn, 'Wges_spring_reg')
    error('cds_obj_dependencies: Regressorform der Federkraft soll berechnet werden, wird aber nicht.');
  end
  if ~Set.general.debug_calc && Structure.calc_cut && isfield(data_dyn, 'Wges') && isfield(data_dyn, 'Wges_ID_reg')
    error('cds_obj_dependencies: Regressorform der Schnittkraft soll berechnet werden, es wird aber auch die Schnittkraft selbst gespeichert');
  end
  if Structure.calc_cut && (~isfield(data_dyn, 'Wges') && ~isfield(data_dyn, 'Wges_reg') && ~isfield(data_dyn, 'Wges_ID_reg') && ~isfield(data_dyn, 'Wges_spring_reg'))
    error('cds_obj_dependencies: Schnittkräfte sollen berechnet werden, aber zugehörige Ergebnisse fehlen');
  end
  if Structure.calc_dyn_reg || Structure.calc_spring_reg
    % Regressorform nur prüfen, falls auch berechnet wird.
    data_dyn2 = cds_obj_dependencies_regmult(R, data_dyn, Q);
    if Structure.calc_dyn_act && ~isfield(data_dyn2, 'TAU')
      error('cds_obj_dependencies_regmult: Feld TAU fehlt nach Regressor-Berechnung');
    end
    if Structure.calc_cut && ~isfield(data_dyn2, 'Wges')
      error('cds_obj_dependencies_regmult: Feld Wges fehlt nach Regressor-Berechnung');
    end
    if isfield(data_dyn, 'TAU') && isfield(data_dyn2, 'TAU')
      test_TAU = data_dyn.TAU - data_dyn2.TAU;
      if any(abs(test_TAU(:)) > 1e-6), error('TAU stimmt nicht zwischen Regressorform und direkter Berechnung'); end
    end
    if isfield(data_dyn, 'Wges') && isfield(data_dyn2, 'Wges')
      test_Wges = data_dyn.Wges - data_dyn2.Wges;
      if any(abs(test_Wges(:)) > 1e-6), error('Wges stimmt nicht zwischen Regressorform und direkter Berechnung'); end
    end
  end
  fprintf(['Testfall %d/%d berechnet. Dauer %1.1fs dafür. %1.1fs gesamt ', ...
    'bis jetzt.\n'], i, size(M,1), toc(t1), toc(t0));
end
fprintf('Alle möglichen Fälle für cds_obj_dependencies und cds_obj_dependencies_regmult getestet\n');
end