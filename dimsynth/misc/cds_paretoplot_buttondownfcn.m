% Auswertungsbilder nach Klick auf Punkt in Paretofront starten.
% Diese Funktion wird als ButtonDownFcn der Line-Objekte eingesetzt.
% 
% Eingabe:
% hObj
%   Handle zum Objekt, dass der ButtonDownFcn zugeordnet ist (Line-Objekt)
% event
%   Eigenschaft des Klick-Events
% OptName
%   Name der Optimierung (entspricht Ordner in Ergebnissen)
% RobName
%   Name des Roboters (zum Auffinden der Dateien)
% RobNr
%   Nummer des Roboters (zum Auffinden der Dateien)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function cds_paretoplot_buttondownfcn(hObj, event, OptName, RobName, RobNr)
t1=tic();
%% Finde die Koordinaten des geklickten Punkts auf der Pareto-Front
% https://de.mathworks.com/matlabcentral/answers/315760-find-out-which-point-is-clicked-in-axes
% https://de.mathworks.com/matlabcentral/answers/476609-how-to-get-the-point-under-mouse-click-on-the-plot-or-histogram#answer_388094
dataObjs = hObj;
xData = get(dataObjs, 'XData')';
yData = get(dataObjs, 'YData')';
pointsInPlot = [xData, yData];
clickedPoint = event.IntersectionPoint(1:2);
distances = sqrt(sum(bsxfun(@minus, pointsInPlot, clickedPoint).^2,2));
closest = pointsInPlot(distances==min(distances),:);
if size(closest,1) > 1
  warning(['Es wurden %d Punkte gleichzeitig erkannt. Liegen anscheinend ', ...
    'übereinander'],  size(closest,1));
  closest = closest(1,:);
end
% Index des geklickten Punkts entspricht Index in Ergebnis-Variablen
I_point = xData==closest(1) & yData==closest(2);
%% Auslesen des Menüs zur Entscheidung der Plot-Aktion
axhdl = get(hObj, 'Parent');
fighdl = get(axhdl, 'Parent');
uihdl = findobj(fighdl, 'Type', 'UIControl');
Selection = get(uihdl, 'Value');
SelStr = get(uihdl,'String');
fprintf('[%s] Starte Vorbereitung und Plot für %s/%s (Rob. %d) "%s"\n', ...
  datestr(now(),'yyyy-mm-dd HH:MM:SS'), OptName, RobName, RobNr, SelStr{Selection});
%% Lade die Daten
% Benutze den Ordner als Speicherort der Daten, in dem auch das Bild liegt.
resdir_opt = fileparts(get(fighdl, 'FileName'));
if isempty(resdir_opt)
  % Das Bild wurde eventuell gerade erst gezeichnet und daher ist kein
  % Dateiname abgespeichert. Suche den Ordner der Ergebnisse.
  resdir = fullfile(fileparts(which('structgeomsynth_path_init.m')), 'results');
  resdir_opt = fullfile(resdir, OptName);
  if ~exist(resdir_opt, 'file')
    warning('Automatisch ermittelter Ergebnis-Ordner %s existiert nicht. Abbruch.', resdir_opt);
    return
  end
elseif ~exist(resdir_opt, 'file')
  warning('Ergebnis-Ordner %s existiert nicht, obwohl Bild von dort geladen wurde. Abbruch.', resdir_opt);
  return
end
resfile1 = fullfile(resdir_opt, sprintf('Rob%d_%s_Endergebnis.mat', ...
  RobNr, RobName));
resfile2 = fullfile(resdir_opt, sprintf('Rob%d_%s_Details.mat', ...
  RobNr, RobName));
setfile = fullfile(resdir_opt, sprintf('%s_settings.mat', OptName));
if ~exist(resfile1, 'file')
  warning('Ergebnis-Datei %s existiert nicht.', resfile1);
  return
end
% if ~exist(resfile2, 'file')
%   warning('Ergebnis-Datei %s existiert nicht.', resfile2);
%   return
% end
if ~exist(setfile, 'file')
  warning('Einstellungs-Datei %s existiert nicht.', setfile);
  return
end
d1 = load(resfile1, 'RobotOptRes');
RobotOptRes = d1.RobotOptRes;
if exist(resfile2, 'file')
  d2 = load(resfile2, 'RobotOptDetails', 'PSO_Detail_Data');
  RobotOptDetails = d2.RobotOptDetails;
  PSO_Detail_Data = d2.PSO_Detail_Data;
else
  RobotOptDetails = [];
  PSO_Detail_Data = [];
end
d3 = load(setfile, 'Traj', 'Set', 'Structures');
Set = d3.Set;
Traj = d3.Traj;
Structure = d3.Structures{RobNr};
% Ergebnistabelle laden
restabfile = fullfile(resdir_opt, sprintf('%s_results_table.csv', OptName));
ResTab = readtable(restabfile, 'Delimiter', ';');
% Ergebnis-Ordner lokal überschreiben (da neue Bilder gespeichert werden).
[resdir_tmp, optfolder] = fileparts(resdir_opt);
if ~strcmp(optfolder, OptName)
  error(['Der Ordnername der Optimierung heißt lokal anders, als in der ', ...
    'Datei: %s vs %s. Das gibt Probleme beim Speichern der Bilder. Abbruch.'], ...
    optfolder, OptName);
end
Set.optimization.resdir = resdir_tmp;

% Fehlende Felder in den Einstellungen ergänzen, sonst Fehler in Funktionen
Set = cds_settings_update(Set, 1);
%% Bestimme die Nummer des Pareto-Partikels
% Nummer istschon in I_point richtig sein, da xData und yData identisch
% mit den Werten aus fval_pareto (bzw. physval) sind.
% Wenn mehrere Punkte übereinander liegen, wird der erste genommen. Ist
% egal, da sie ja ein identisches Ergebnis haben.
PNr = find(I_point, 1, 'first');
fval = d1.RobotOptRes.fval_pareto(PNr,:)';
if isempty(PNr)
  error('Kein Punkt in Pareto-Daten gefunden');
end
RobData = struct('Name', RobName, 'Number', RobNr, 'ParetoNumber', PNr, ...
  'Type', RobotOptRes.Structure.Type);
%% Berechne fehlende Größen
fitness_recalc_necessary = true;
if any(strcmp(SelStr(Selection), {'Pareto', 'Parameter'}))
  % Neuberechnung der Fitness-Funktion für diese Bilder nicht notwendig.
  fitness_recalc_necessary = false;
end
if fitness_recalc_necessary
  % Für Ausführung der Fitness-Fcn
  if RobData.Type == 0
    serroblib_addtopath({RobName});
  else
    parroblib_addtopath({RobName});
  end
  % Berechne die Fitness-Funktionen
  clear cds_save_particle_details cds_fitness cds_log
  p = RobotOptRes.p_val_pareto(PNr,:)';
  p_desopt = RobotOptRes.desopt_pval_pareto(PNr,:)';
  if any(isnan(p_desopt))
    warning('Ergebnisse für die Entwurfsoptimierung sind NaN.');
    p_desopt = [];
  end
  if isempty(RobotOptDetails) || ~isempty(p_desopt)
    % Falls nur die reduzierten Ergebnis-Daten vorliegen oder die Ergebnisse
    % der Entwurfsoptimierung direkt eingetragen werden sollen. Vermeide die
    % erneute Durchführung der Entwurfsoptimierung, falls diese gemacht wurde.
    [R, Structure] = cds_dimsynth_robot(Set, Traj, Structure, true);
    % Trage die gespeicherte Anfangswerte der Gelenkkonfigurationen in die
    % Roboter-Klasse ein. Dadurch wird wieder die gleiche IK-Konfiguration
    % getroffen.
    if isfield(RobotOptRes, 'q0_pareto')
      q0 = RobotOptRes.q0_pareto(PNr,:)';
    elseif ~isempty(RobotOptDetails)
      % Lade IK-Anfangswerte aus gespeicherten Daten
      [k_gen, k_ind] = cds_load_particle_details(PSO_Detail_Data, fval);
      q0 = PSO_Detail_Data.q0_ik(k_ind,:,k_gen)';
    else
      q0 = NaN(R.NJ,1);
    end
    if any(~isnan(q0))
      if R.Type == 0
        R.qref = q0;
      else
        for iLeg = 1:R.NLEG
          R.Leg(iLeg).qref = q0(R.I1J_LEG(iLeg):R.I2J_LEG(iLeg));
        end
      end
    end
    if isempty(p_desopt)
      [fval2, ~, Q, QD, QDD, TAU] = cds_fitness(R,Set,Traj,Structure,p);
    else
      % Keine erneute Entwurfsoptimierung, also auch keine Regressorform notwendig.
      % Direkte Berechnung der Dynamik, falls für Zielfunktion notwendig.
      Structure.calc_dyn_act = Structure.calc_dyn_act | Structure.calc_dyn_reg;
      Structure.calc_spring_act = Structure.calc_spring_act | Structure.calc_spring_reg;
      Structure.calc_spring_reg = false;
      Structure.calc_dyn_reg = false;
      [fval2, ~, Q, QD, QDD, TAU] = cds_fitness(R,Set,Traj,Structure,p,p_desopt);
    end
  else
    % Alternative Berechnung (erfordert Laden der Detail-Daten).
    % Hier ist die Reproduktion der Zielfunktion besser möglich, da Anfangs-
    % werte für die Gelenkwinkel besser passen.
    [fval2, ~, Q, QD, QDD, TAU] = RobotOptDetails.fitnessfcn(p);
    R = RobotOptDetails.R;
  end
  if any(abs(fval-fval2) > 1e-4)
    warning(['Fitness-Wert von [%s] aus Pareto-Front konnte nicht reproduziert ', ...
      'werden (neu: [%s])'], disp_array(fval', '%1.4f'), disp_array(fval2', '%1.4f'));
  end
  if isempty(Q)
    warning('Keine Gelenkwinkel aus inverser Kinematik berechnet. Abbruch.');
    return
  end
  RobotOptDetails = struct('Traj_Q', Q, 'Traj_QD', QD, 'Traj_QDD', QDD, ...
    'R', R, 'Dyn_Tau', TAU);
else % Roboter-Klasse muss trotzdem neu erstellt werden.
  R = cds_dimsynth_robot(Set, Traj, Structure, true);
  RobotOptDetails = struct('R', R);
end

%% Rufe die Plot-Funktion auf
t2=tic();
if strcmp(SelStr(Selection), 'Visualisierung')
  cds_vis_results_figures('robvisu', Set, Traj, RobData, ResTab, ...
    RobotOptRes, RobotOptDetails);
end
if strcmp(SelStr(Selection), 'Parameter')
  cds_vis_results_figures('optpar', Set, Traj, RobData, ResTab, ...
    RobotOptRes, RobotOptDetails);
end
if strcmp(SelStr(Selection), 'Kinematik')
  cds_vis_results_figures('jointtraj', Set, Traj, RobData, ResTab, ...
    RobotOptRes, RobotOptDetails);
end
if strcmp(SelStr(Selection), 'Animation')
  if isempty(Set.general.animation_styles)
    Set.general.animation_styles = {'3D'};
  end
  cds_vis_results_figures('animation', Set, Traj, RobData, ResTab, ...
    RobotOptRes, RobotOptDetails);
end
if strcmp(SelStr(Selection), 'Dynamik')
  cds_vis_results_figures('dynamics', Set, Traj, RobData, ResTab, ...
    RobotOptRes, RobotOptDetails);
end
if strcmp(SelStr(Selection), 'Dynamikparameter')
  cds_vis_results_figures('dynparvisu', Set, Traj, RobData, ResTab, ...
    RobotOptRes, RobotOptDetails);
end
if strcmp(SelStr(Selection), 'Feder-Ruhelage')
  cds_vis_results_figures('springrestpos', Set, Traj, RobData, ResTab, ...
    RobotOptRes, RobotOptDetails);
end
if strcmp(SelStr(Selection), 'Pareto')
  cds_vis_results_figures('pareto', Set, Traj, RobData, ResTab, ...
    RobotOptRes, RobotOptDetails, PSO_Detail_Data);
end
fprintf('Bilder gezeichnet. Dauer: %1.1fs zur Vorbereitung, %1.1fs zum Zeichnen.\n', ...
  toc(t1)-toc(t2), toc(t2));
