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
% Index des geklickten Punkts entspricht Index in Ergebnis-Variablen
I_point = xData==closest(1) & yData==closest(2);
%% Auslesen des Menüs zur Entscheidung der Plot-Aktion
axhdl = get(hObj, 'Parent');
fighdl = get(axhdl, 'Parent');
uihdl = findobj(fighdl, 'Type', 'UIControl');
Selection = get(uihdl, 'Value');
SelStr = get(uihdl,'String');
fprintf('Starte Vorbereitung und Plot für %s/%s (Rob. %d) "%s"\n', ...
  OptName, RobName, RobNr, SelStr{Selection});
%% Lade die Daten
% Suche die Optimierung im Datenordner. Nehme nicht den abgespeicherten
% Pfad aus der Ergebnis-Datei, da der absolute Pfad auf dem Cluster anders
% ist.
resdir = fullfile(fileparts(which('structgeomsynth_path_init.m')), 'results');
resdir_opt = fullfile(resdir, OptName);
if ~exist(resdir_opt, 'file')
  warning('Ergebnis-Ordner %s existiert nicht.', resdir_opt);
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
  d2 = load(resfile2, 'RobotOptDetails');
  RobotOptDetails = d2.RobotOptDetails;
else
  RobotOptDetails = [];
end
d3 = load(setfile, 'Traj', 'Set', 'Structures');
Set = d3.Set;
Traj = d3.Traj;
Structure = d3.Structures{RobNr};
% Ergebnistabelle laden
restabfile = fullfile(resdir_opt, sprintf('%s_results_table.csv', OptName));
ResTab = readtable(restabfile, 'Delimiter', ';');
% Ergebnis-Ordner lokal überschreiben
Set.Set.optimization.resdir = resdir;

%% Bestimme die Nummer des Pareto-Partikels
% Annahme: Hier nicht bekannt, ob fval- oder physval-Pareto-Diagramm.
% Prüfe auf Gleichheit gegen beide. Einer muss richtig sein.
% Nummer könnte auch schon in I_point richtig sein, voraussgesetzt xData
% und yData sind identisch mit den Werten aus fval_pareto (bzw. physval).
% Das nachträgliche Finden ignoriert die Einheitenkonvertierung (rad-deg)
% im Plot.
% I_fval = repmat(selectedPoint, size(d1.RobotOptRes.fval_pareto,1),1) == ...
%   d1.RobotOptRes.fval_pareto;
% I_physval = repmat(selectedPoint, size(d1.RobotOptRes.physval_pareto,1),1) == ...
%   d1.RobotOptRes.physval_pareto;
% PNr = find(all(I_fval|I_physval,2));
PNr = find(I_point);
fval = d1.RobotOptRes.fval_pareto(PNr,:)';
if isempty(PNr)
  error('Kein Punkt in Pareto-Daten gefunden');
end
RobData = struct('Name', RobName, 'Number', RobNr, 'ParetoNumber', PNr, ...
  'Type', RobotOptRes.Structure.Type);
%% Berechne fehlende Größen
% Für Ausführung der Fitness-Fcn
if RobData.Type == 0
  serroblib_addtopath({RobName});
else
  parroblib_addtopath({RobName});
end
% Berechne die Fitness-Funktionen
clear cds_save_particle_details cds_fitness cds_log
p = RobotOptRes.p_val_pareto(PNr,:)';
if isempty(RobotOptDetails)
  % Falls nur die reduzierten Ergebnis-Daten vorliegen
  [R, Structure] = cds_dimsynth_robot(Set, Traj, Structure, true);
  [fval2, ~, Q, QD, QDD, TAU] = cds_fitness(R,Set,Traj,Structure,p);
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
  return
end
RobotOptDetails = struct('Traj_Q', Q, 'Traj_QD', QD, 'Traj_QDD', QDD, 'R', R, ...
  'Dyn_Tau', TAU);

%% Rufe die Plot-Funktion auf
t2=tic();
if strcmp(SelStr(Selection), 'Visualisierung')
  cds_vis_results_figures('robvisu', Set, Traj, RobData, ResTab, ...
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
fprintf('Bilder gezeichnet. Dauer: %1.1fs zur Vorbereitung, %1.1fs zum Zeichnen.\n', ...
  toc(t1)-toc(t2), toc(t2));