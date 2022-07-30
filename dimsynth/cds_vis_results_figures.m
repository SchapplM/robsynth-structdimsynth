% Visualisierung der Ergebnisse der Maßsynthese für einen Roboter
% 
% Eingabe:
% figname
%   Name des zu erstellenden Bildes. Entspricht Set.general.eval_figures
%   * robvisu - Visualisierung des Roboters (erster Schritt der Trajektorie)
%   * robvisuanim - letztes Bild der Animation (geht nur mit 'animation')
%   * animation - Animation des Roboters
%   * jointtraj - Gelenkwinkel-Trajektorie über die Zeit
%   * pareto - Pareto-Diagramm für alle Individuen dieses Roboters
%   * dynamics - Antriebskräfte (und Plattform-Kräfte bei PKM)
%   * dynparvisu - Visualisierung der Dynamikparameter
%   * optpar - Optimierungsparameter über die Pareto-Front
%   * springrestpos - Ruhelage der Gelenkwinkel-Federn (als Animation)
% Set
%   Einstellungen des Optimierungsalgorithmus. Siehe cds_settings_defaults.
% Traj
%   Trajektorie (bezogen auf Welt-KS)
% RobData
%   Daten zum Roboter. Größtenteils ähnlich zu Variable "Structures" aus
%   andere Funktionen.
% ResTab
%   Inhalt der Ergebnis-Tabelle aus cds_results_table.
% RobotOptRes
%   Zusammengefasste Ergebnisse der Maßsynthese. Siehe cds_dimsynth_robot.
% RobotOptDetails
%   Detaillierte Ergebnisse der Maßsynthese. Siehe cds_dimsynth_robot.
% settings
%   Einstellungen zum Verhalten der Funktion. Felder:
%   .figure_invisible: Erzeuge alle Bilder unsichtbar, ohne Fokus-Klau.
%   .delete_figure: Schließe das Bild direkt nach dem Speichern wieder.
% 
% Siehe auch: cds_vis_results.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function fhdl = cds_vis_results_figures(figname, Set, Traj, RobData, ...
  ResTab, RobotOptRes, RobotOptDetails, PSO_Detail_Data, settings)
%% Initialisierung
assert(isa(figname, 'char'), 'Eingabe figname muss char sein');
fignames_allowed = {'robvisu', 'robvisuanim', 'animation', 'jointtraj', ...
  'pareto', 'dynamics', 'dynparvisu', 'optpar', 'springrestpos'};
assert(any(strcmp(fignames_allowed, figname)), ['Eingabe figname (''%s'') muss ', ...
  'eines dieser der Werte sein: %s'], figname, disp_array(fignames_allowed, '%s'));

settings_defaults = struct(...
  'figure_invisible', false, ...
  'delete_figure', false);
if nargin < 9
  settings = settings_defaults;
end
for f = fields(settings_defaults)'
  if ~isfield(settings, f{1})
    settings.(f{1}) = settings_defaults.(f{1});
  end
end
RNr = RobData.Number;
PNr = RobData.ParetoNumber;
R = RobotOptDetails.R;
Traj_0 = cds_transform_traj(R, Traj);
Name = RobData.Name;
resmaindir = fullfile(Set.optimization.resdir, Set.optimization.optname);
resrobdir = fullfile(resmaindir, sprintf('Rob%d_%s', RNr, Name));
if isempty(RobotOptRes.fval_pareto)
  fval = RobotOptRes.fval;
else
  fval = RobotOptRes.fval_pareto(PNr,:)';
end
if all(fval > 1e3) % NB verletzt. Es gibt nur ein Kriterium
  fval_str = sprintf('%1.1e', fval(1));
elseif length(fval) == 1 % Einkriteriell
  fval_str = sprintf('%1.1f', fval);
else % Mehrkriteriell
  fval_str = ['[',disp_array(fval', '%1.1f'),']'];
end
traj_available = true; % Einige Bilder gehen nur, wenn Zeitverlauf da ist
if Set.task.profile == 0 || all(fval > 1e9)
  % Keine Trajektorie mit Zeitverlauf gefordert oder Fehler dabei
  traj_available = false;
end
% Setze die Aufgaben-FG bei PKM auf vollständig. Sonst wird bei Aufgaben- 
% redundanz die Jacobi-Matrix in den Dynamik-Funktionen falsch berechnet. 
% Damit die EE-Drehung dann in den Plots nicht falsch ist, berechne die
% EE-Drehung mit der direkten Kinematik neu.
X = Traj_0.X; XD = Traj_0.XD; XDD = Traj_0.XDD; XE = Traj_0.XE;
if Set.task.pointing_task && ~isempty(RobotOptDetails.Traj_Q) && traj_available
  [X_fk, XD_fk, XDD_fk] = R.fkineEE2_traj(RobotOptDetails.Traj_Q, ...
    RobotOptDetails.Traj_QD, RobotOptDetails.Traj_QDD);
  X(:,6) = denormalize_angle_traj(X_fk(:,6));
  XD(:,6) = XD_fk(:,6); XDD(:,6) = XDD_fk(:,6);
  XE(Traj_0.IE~=0,:) = X(Traj_0.IE(Traj_0.IE~=0),:);
end
if RobData.Type ~= 0
  R.update_EE_FG(R.I_EE, R.I_EE);
end

%% Animation
if strcmp(figname, 'robvisu')
  % Die Roboter-Visualisierung ist größtenteils identisch zur Animation.
  % Nur dass nicht animiert wird, sondern dass das erste Standbild genommen
  % wird.
  Set.general.animation_styles = {'3D'};
elseif ~strcmp(figname, 'animation')
  Set.general.animation_styles = {};
end
% Einstellung bzgl Dateiformat anpassen. Falls Abbruch bereits bei wenigen
% Eckpunkten, sollte eine GIF-Datei erzeugt werden. MP4 mit nur wenigen
% Einzelbildern sind problematisch beim Abspielen (Grenze: 10 Bilder)
n_qvalid = size(RobotOptDetails.Traj_Q,1);
if any(isnan(isnan(RobotOptDetails.Traj_Q(:))))
  n_qvalid = find(any(isnan(RobotOptDetails.Traj_Q),2),1,'first')-1;
end
if ~traj_available && n_qvalid < 10
  Set.general.save_animation_file_extensions = unique(['gif', ...
    Set.general.save_animation_file_extensions]);
end
% Hole Erklärungstext zum Fitness-Wert aus Tabelle
iRobTab = strcmp(ResTab.Name,Name) & ResTab.LfdNr==RNr;
fval_text = ResTab.Fval_Text{iRobTab};
RobShortName_str = ''; % Wenn kein Wert belegt ist, wird NaN gesetzt
if isa(ResTab.Beschreibung, 'cell')
  RobShortName = ResTab.Beschreibung{iRobTab};
  if ~isempty(RobShortName)
    RobShortName_str = sprintf(' (%s)', RobShortName);
  end
end
for kk = 1:length(Set.general.animation_styles)
  anim_mode = Set.general.animation_styles{kk}; % Strichzeichnung, 3D-Modell, Kollisionskörper
  if settings.figure_invisible, fhdl = figure_invisible();
  else,                         fhdl = figure(); end
  clf; hold all;
  set(fhdl, 'Name', sprintf('Rob%d_P%d_anim_%s', RNr, PNr, anim_mode), 'NumberTitle', 'off', 'color','w');
  title(sprintf('Rob.%d, P.%d %s%s: fval=%s (%s)', RNr, PNr, Name, ...
    RobShortName_str, fval_str, fval_text));
  view(3);
  axis auto
  hold on;grid on;
  xlabel('x in m');ylabel('y in m');zlabel('z in m');
  plot3(Traj.X(:,1), Traj.X(:,2),Traj.X(:,3), 'k-');
  % Falls ein 2T1R-/2T0R-Roboter gezeichnet wird, werden die z-Grenzen
  % automatisch auf -1/+1 gesetzt. Dann ist der Roboter im Video sehr klein
  if Set.task.DoF(3) == 0, zlim(Traj.X(1,3)+[-0.1, +0.1]); end
  % Arbeitsraum-Objekte einzeichnen
  for co_sets = 1:2 % einmal Bauraum-Begrenzung, einmal Störkulissen
    if co_sets == 1, color = 'c'; collobjset = Set.task.installspace;
    else,            color = 'm'; collobjset = Set.task.obstacles; end
    if co_sets == 1 && ~Set.general.animation_installationspace || ...
       co_sets == 2 && ~Set.general.animation_workspaceobstacles
      continue
    end
    for jj = 1:size(collobjset.type,1)
      switch collobjset.type(jj)
        case 1 % Quader
          q_W = collobjset.params(jj,1:3)';
          u1_W = collobjset.params(jj,4:6)';
          u2_W = collobjset.params(jj,7:9)';
          u3_W = cross(u1_W,u2_W); u3_W = u3_W/norm(u3_W)*collobjset.params(jj,10);
          % Umrechnen in Format der plot-Funktion
          cubpar_c = q_W(:)+(u1_W(:)+u2_W(:)+u3_W(:))/2; % Mittelpunkt des Quaders
          cubpar_l = [norm(u1_W); norm(u2_W); norm(u3_W)]; % Dimension des Quaders
          cubpar_a = 180/pi*tr2rpy([u1_W(:)/norm(u1_W), u2_W(:)/norm(u2_W), u3_W(:)/norm(u3_W)],'zyx')'; % Orientierung des Quaders
          drawCuboid([cubpar_c', cubpar_l', cubpar_a'], ...
            'FaceColor', color, 'FaceAlpha', 0.1);
        case 2 % Zylinder
          drawCylinder(collobjset.params(jj,1:7), ...
            'FaceColor', color, 'FaceAlpha', 0.1);
        case 3 % Kapsel
          drawCapsule(collobjset.params(jj,1:7),'FaceColor', color, 'FaceAlpha', 0.1);
        case 4 % Kugel
          drawSphere(collobjset.params(jj,1:4),'FaceColor', color, 'FaceAlpha', 0.1);
        otherwise
          warning('Geometrie mit Nummer %d kann nicht gezeichnet werden', collobjset.type(jj));
      end
    end % for collobjset.type
  end % for co_sets
  s_anim = struct('gif_name', '', 'avi_name', '', 'mp4_name', '');
  for kkk = 1:length(Set.general.save_animation_file_extensions)
    file_ext = Set.general.save_animation_file_extensions{kkk};
    s_anim.(sprintf('%s_name', file_ext)) = fullfile(resrobdir, ...
      sprintf('Rob%d_%s_P%d_Animation_%s.%s', RNr, Name, PNr, anim_mode, file_ext));
  end
  if Set.general.isoncluster
    % Auf Cluster bleiben manchmal halb fertige AVI-Dateien auf Laufwerk
    % bigwork liegen. Ist problematisch. Besser tmp-Laufwerk der Worker
    % benutzen (/scratch/jobid.batch.css.lan/Matlab/....)
    s_anim.tmpdir = tmpDirFcn();
  end
  if ~traj_available
    % Keine Trajektorie mit Zeitverlauf gefordert oder Fehler dabei
    I_anim = 1:size(RobotOptDetails.Traj_Q,1); % Zeichne jeden Zeitschritt
    Traj_X = XE;
    s_anim.FrameRate = 10; % langsamer abspielen
  else
    if Traj_0.t(end) > Set.general.maxduration_animation
      % Reduziere das Video auf die maximale Länge. Die Abtastrate ist 30Hz
      % Nehme das Verhältnis von Ist- und Soll-Zeit als Beschleunigungsfaktor
      % dieser vorgegebenen Abtastrate des Videos
      t_Vid = (0:1/30*(Traj_0.t(end)/Set.general.maxduration_animation):Traj_0.t(end))';
    else
      % Stelle so ein, dass eine Abtastrate von 30Hz erreicht wird. Das Video
      % läuft dann genauso schnell wie die Trajektorie.
      t_Vid = (0:1/30:Traj_0.t(end))'; % Zeitstempel des Videos
    end
    I_anim = knnsearch( Traj_0.t , t_Vid ); % Berechne Indizes in Traj.-Zeitstempeln
    Traj_X = X;
  end
  if RobData.Type == 0 % Seriell
    s_plot = struct( 'straight', 1);
  else % Parallel
    s_plot = struct( 'ks_legs', [], 'straight', 1);
    if kk == 2, s_plot.mode = 4; end
  end
  if strcmp(anim_mode, 'stick')
    s_plot.mode = 1;
  elseif strcmp(anim_mode, 'CAD')
    s_plot.mode = 2;
    % Prüfe, ob auch ein CAD-Modell hinterlegt ist
    if RobData.Type == 0 && isempty(R.CADstruct.link) % Seriell
      warning('Serieller Roboter ohne CAD-Modell. Nur 3D-Plot möglich');
      s_plot.mode = 4;
    elseif RobData.Type == 2
      warning('PKM mit CAD-Modell noch nicht implementiert. Nur 3D-Plot möglich');
      s_plot.mode = 4;
    end
  elseif strcmp(anim_mode, '3D')
    s_plot.mode = 4;
  elseif strcmp(anim_mode, 'collision')
    s_plot.mode = 5;
  else
    error('Modus %s für Animation nicht definiert', anim_mode);
  end
  if strcmp(figname, 'animation')
    R.anim( RobotOptDetails.Traj_Q(I_anim,:), Traj_X(I_anim,:), s_anim, s_plot);
  else
    if RobData.Type == 0 % Seriell
      R.plot( RobotOptDetails.Traj_Q(1,:)', s_plot);
    else % PKM
      R.plot( RobotOptDetails.Traj_Q(1,:)', Traj_X(1,:)', s_plot);
    end
  end
  if any(strcmp(figname, 'robvisu')) || ... % nur speichern, wenn gewünscht
      any(strcmp(Set.general.eval_figures, 'robvisuanim')) && ...
      any(strcmp(figname, 'animation'))
    fname = sprintf('Rob%d_%s_P%d_Skizze_%s', RNr, Name, PNr, anim_mode);
    saveas(fhdl,     fullfile(resrobdir, [fname, '.fig']));
    export_fig(fhdl, fullfile(resrobdir, [fname, '.png']));
    fprintf('Bild %s gespeichert: %s\n', fname, resrobdir);
  end
  if settings.delete_figure, delete(fhdl); end
end
%% Kinematik-Bild
if strcmp(figname, 'jointtraj') && traj_available
  if settings.figure_invisible, fhdl = figure_invisible();
  else,                         fhdl = figure(); end
  clf;hold all;
  set(fhdl, 'Name', sprintf('Rob%d_P%d_KinematikZeit', RNr, PNr), 'NumberTitle', 'off', 'color','w');
  sgtitle(sprintf('Rob.%d, P.%d: fval=%s', RNr, PNr, fval_str));
  if RobData.Type == 0
    x_row = 2; nrows = 2;
  else
    x_row = 3; nrows = 3;
  end
  if RobData.Type == 0
    subplot(2,3,sprc2no(2,3,1,1));
    plot(Traj_0.t, RobotOptDetails.Traj_Q);
    grid on; ylabel('q in rad oder m');
    subplot(2,3,sprc2no(2,3,1,2));
    plot(Traj_0.t, RobotOptDetails.Traj_QD);
    grid on; ylabel('qD in rad/s oder m/s');
    subplot(2,3,sprc2no(2,3,1,3));
    plot(Traj_0.t, RobotOptDetails.Traj_QDD);
    grid on; ylabel('qDD in rad/s² oder m/s²');
  else
    subplot(3,3,sprc2no(3,3,1,1));
    plot(Traj_0.t, RobotOptDetails.Traj_Q(:,R.I_qa));
    grid on; ylabel('q_a in rad oder m');
    subplot(3,3,sprc2no(3,3,1,2));
    plot(Traj_0.t, RobotOptDetails.Traj_QD(:,R.I_qa));
    grid on; ylabel('qD_a in rad/s oder m/s');
    subplot(3,3,sprc2no(3,3,1,3));
    plot(Traj_0.t, RobotOptDetails.Traj_QDD(:,R.I_qa));
    grid on; ylabel('qDD_a in rad/s² oder m/s²');
    
    subplot(3,3,sprc2no(3,3,2,1));
    plot(Traj_0.t, RobotOptDetails.Traj_Q(:,R.I_qd));
    grid on; ylabel('q_p in rad oder m');
    subplot(3,3,sprc2no(3,3,2,2));
    plot(Traj_0.t, RobotOptDetails.Traj_QD(:,R.I_qd));
    grid on; ylabel('qD_p in rad/s oder m/s');
    subplot(3,3,sprc2no(3,3,2,3));
    plot(Traj_0.t, RobotOptDetails.Traj_QDD(:,R.I_qd));
    grid on; ylabel('qDD_p in rad/s² oder m/s²');
  end
  
  subplot(nrows,3,sprc2no(nrows,3,x_row,1));
  plot(Traj_0.t, X);
  grid on; ylabel('x in m oder rad');
  legend({'rx', 'ry', 'rz', 'phix', 'phiy', 'phiz'});
  subplot(nrows,3,sprc2no(nrows,3,x_row,2));
  plot(Traj_0.t, XD);
  grid on; ylabel('xD in m/s oder rad/s');
  subplot(nrows,3,sprc2no(nrows,3,x_row,3));
  plot(Traj_0.t, XDD);
  grid on; ylabel('xDD in m/s² oder rad/s²');
  linkxaxes;
  fname = sprintf('Rob%d_%s_P%d_KinematikZeit', RNr, Name, PNr);
  saveas(fhdl,     fullfile(resrobdir, [fname, '.fig']));
  export_fig(fhdl, fullfile(resrobdir, [fname, '.png']));
  fprintf('Bild %s gespeichert: %s\n', fname, resrobdir);
  if settings.delete_figure, delete(fhdl); end
end

%% Pareto-Fronten für die Zielkriterien
if strcmp(figname, 'pareto')
  if isempty(PSO_Detail_Data)
    error(['Variable PSO_Detail_Data wurde nicht gespeichert. Daher kein ', ...
      'detailliertes Pareto-Diagramm möglich']);
  end
  % Einheiten für die physikalischen Werte der Zielfunktionen vorbereiten
  [obj_units, objscale] = cds_objective_plotdetails(Set);
  if length(Set.optimization.objective) > 1 % Mehrkriterielle Optimierung
    % Gehe alle Kombinationen von zwei Zielkriterien durch (falls mehr als
    % zwei gewählt).
    objcomb = allcomb(1:length(Set.optimization.objective), 1:length(Set.optimization.objective));
    objcomb(objcomb(:,1)==objcomb(:,2),:) = [];
    objcomb(objcomb(:,1)>objcomb(:,2),:) = [];
    for pffig = 1 % Zwei Bilder: Physikalische Werte (1) und normierte Werte (2; deaktiviert)
    if settings.figure_invisible, fhdl = figure_invisible();
    else,                         fhdl = figure(); end
    clf;hold all;
    set(fhdl, 'Name', sprintf('Rob%d_Pareto', RNr), ...
      'NumberTitle', 'off', 'color','w');
    axhdl = NaN(size(objcomb,1),1);
    sprows = floor(sqrt(size(objcomb,1)));
    spcols = ceil(size(objcomb,1)/sprows);
    for kk = 1:size(objcomb,1)
      kk1 = objcomb(kk,1); kk2 = objcomb(kk,2);
      ngen = size(PSO_Detail_Data.fval,3);
      nswarm = size(PSO_Detail_Data.fval,1);
      F1=reshape(PSO_Detail_Data.fval(:,kk1,:), ngen, nswarm);
      F2=reshape(PSO_Detail_Data.fval(:,kk2,:), ngen, nswarm);
      P1=reshape(PSO_Detail_Data.physval(:,kk1,:), ngen, nswarm);
      P2=reshape(PSO_Detail_Data.physval(:,kk2,:), ngen, nswarm);
      leghdl = NaN(10,1);
      axhdl(kk) = subplot(sprows,spcols,kk); hold on;
      % Nicht-dominierende Partikel aus dem Verlauf der Optimierung
      markers = {'rv', 'ro', 'y^', 'gv', 'go', 'c^', 'cv', 'bo', 'b^'};
      % Teile den Verlauf in 9 Teile ein und zeichne die Partikel jeder
      % Phase mit den entsprechenden Markern
      legtext = cell(10,1);
      for jj = 1:9
        mingen_jj = floor(ngen/9*(jj-1))+1;
        maxgen_jj = floor(ngen/9*(jj));
        Igen = mingen_jj:maxgen_jj;
        F1_sel = F1(Igen,:); F1v_sel = F1_sel(:);
        F2_sel = F2(Igen,:); F2v_sel = F2_sel(:);
        P1_sel = P1(Igen,:); P1v_sel = P1_sel(:);
        P2_sel = P2(Igen,:); P2v_sel = P2_sel(:);
        I_iO = F1v_sel < 1e3;
        if ~any(I_iO), continue; end
        if pffig == 1 % Bild mit physikalischen Werten
          leghdl(jj)=plot(objscale(kk1)*P1v_sel(I_iO), objscale(kk2)*P2v_sel(I_iO), markers{jj});
        else
          leghdl(jj)=plot(F1v_sel(I_iO), F2v_sel(I_iO), markers{jj});
        end
        legtext{jj} = sprintf('$%d \\le i_{\\mathrm{gen}} \\le %d$', mingen_jj, maxgen_jj);
      end
      % Pareto-Front einzeichnen
      legtext{10} = 'Pareto-Front';
      if pffig == 1 % Bild mit physikalischen Werten
        leghdl(10)=plot(objscale(kk1)*RobotOptRes.physval_pareto(:,kk1), ...
             objscale(kk2)*RobotOptRes.physval_pareto(:,kk2), 'm*');
        xlabel(sprintf('Zielf. %d (%s) in %s', kk1, Set.optimization.objective{kk1}, obj_units{kk1}));
        ylabel(sprintf('Zielf. %d (%s) in %s', kk2, Set.optimization.objective{kk2}, obj_units{kk2}));
      else % Bild mit normierten Zielfunktionswerten
        leghdl(10)=plot(RobotOptRes.fval_pareto(:,kk1), ...
             RobotOptRes.fval_pareto(:,kk2), 'm*');
        xlabel(sprintf('Zielf. %d (%s) (normiert)', kk1, Set.optimization.objective{kk1}));
        ylabel(sprintf('Zielf. %d (%s)(normiert)', kk2, Set.optimization.objective{kk2}));
      end
      if any(~isnan(leghdl))
        legend(leghdl(~isnan(leghdl)), legtext(~isnan(leghdl)), 'interpreter', 'latex');
      end
      grid on;
    end
    % Prüfe ob ein Wertebereich sehr stark unausgeglichen ist (z.B. wenn die
    % Konditionszahl als Kriterium bis unendlich geht). Dann logarithmisch.
    for ah = axhdl(:)'
      linhdl = get(ah, 'children');
      for kk = 1:3 % x,y,z
        dataminmax = [NaN, NaN];
        for jj = 1:length(linhdl) % xdata, ydata, zdata
          if ~strcmp(get(linhdl(jj), 'type'), 'line'); continue; end
          dataminmax = minmax2([dataminmax, get(linhdl(jj), [char(119+kk), 'data'])]);
        end
        if dataminmax(2) / dataminmax(1) > 1e3 && all(dataminmax>0)
          set(ah, [char(119+kk), 'scale'], 'log');
        end
      end
    end
    if pffig == 1 
      sgtitle(sprintf('Rob. %d: Pareto-Fronten für mehrkrit. Opt. (Physikalische Werte)', RNr));
      name_suffix = 'phys';
    else
      sgtitle(sprintf('Rob. %d: Pareto-Fronten für mehrkrit. Opt. (Normierte Werte)', RNr));
      name_suffix = 'fval';
    end
    saveas(fhdl,     fullfile(resrobdir, sprintf('Rob%d_%s_Pareto2D_%s.fig', RNr, Name, name_suffix)));
    export_fig(fhdl, fullfile(resrobdir, sprintf('Rob%d_%s_Pareto2D_%s.png', RNr, Name, name_suffix)));
    if settings.delete_figure, delete(fhdl); end
    end
  end
  %% (3D)-Pareto-Fronten für die Zielkriterien
  if length(Set.optimization.objective) > 2
    objcomb = allcomb(1:length(Set.optimization.objective), 1:length(Set.optimization.objective), ...
      1:length(Set.optimization.objective));
    objcomb(objcomb(:,1)==objcomb(:,2) | objcomb(:,2)==objcomb(:,3),:) = [];
    objcomb(objcomb(:,1)>objcomb(:,2),:) = [];
    objcomb(objcomb(:,2)>objcomb(:,3),:) = [];
    if settings.figure_invisible, fhdl = figure_invisible();
    else,                         fhdl = figure(); end
    clf;hold all;
    set(fhdl, 'Name', sprintf('Rob%d_Pareto3D', RNr), ...
      'NumberTitle', 'off', 'color','w');
    sprows = floor(sqrt(size(objcomb,1)));
    spcols = ceil(size(objcomb,1)/sprows);
    for kk = 1:size(objcomb,1)
      kk1 = objcomb(kk,1); kk2 = objcomb(kk,2); kk3 = objcomb(kk,3);
      subplot(sprows,spcols,kk);
      plot3(objscale(kk1)*RobotOptRes.physval_pareto(:,kk1), ...
            objscale(kk2)*RobotOptRes.physval_pareto(:,kk2), ...
            objscale(kk3)*RobotOptRes.physval_pareto(:,kk3), 'm*');
      xlabel(sprintf('Zielf. %d (%s) in %s', kk1, Set.optimization.objective{kk1}, obj_units{kk1}));
      ylabel(sprintf('Zielf. %d (%s) in %s', kk2, Set.optimization.objective{kk2}, obj_units{kk2}));
      zlabel(sprintf('Zielf. %d (%s) in %s', kk3, Set.optimization.objective{kk3}, obj_units{kk3}));
      grid on;
    end
    sgtitle(sprintf('Rob. %d: Pareto-Fronten (3D) für mehrkrit. Opt.', RNr));
    saveas(fhdl,     fullfile(resrobdir, sprintf('Rob%d_%s_Pareto3D.fig', RNr, Name)));
    export_fig(fhdl, fullfile(resrobdir, sprintf('Rob%d_%s_Pareto3D.png', RNr, Name)));
    if settings.delete_figure, delete(fhdl); end
  end

end
%% Rechne die Dynamik neu nach
if strcmp(figname, 'dynamics') && traj_available
  Q = RobotOptDetails.Traj_Q;
  QD = RobotOptDetails.Traj_QD;
  QDD = RobotOptDetails.Traj_QDD;
  if RobData.Type == 0
    Dyn_C = NaN(size(Q,1),R.NJ); Dyn_Tau_Static = Dyn_C;
    Dyn_G = Dyn_C; Dyn_A = Dyn_C; Dyn_Tau = Dyn_C; Dyn_S = Dyn_C;
    Dyn_Ext = Dyn_C;
    for i = 1:size(Q,1)
      Dyn_C(i,:) = R.corvec(Q(i,:)', QD(i,:)');
      Dyn_G(i,:) = R.gravload(Q(i,:)');
      Dyn_A(i,:) = R.inertia(Q(i,:)')*QDD(i,:)';
      Dyn_Tau_Static(i,:) = R.invdyn(Q(i,:)', 0*QD(i,:)', 0*QDD(i,:)');
      Dyn_Tau(i,:) = R.invdyn(Q(i,:)', QD(i,:)', QDD(i,:)');
      Dyn_S(i,:) = 0; % keine Federn in Gelenken bei seriellen Robotern
      Dyn_Ext(i,:) = -R.jacobig(Q(i,:)')' * Traj_0.Fext(i,:)';
    end
  else
    Dyn_C = NaN(size(Q,1),sum(R.I_qa)); Dyn_Tau_Static = Dyn_C;
    Dyn_G = Dyn_C; Dyn_A = Dyn_C; Dyn_Tau = Dyn_C; Dyn_S = Dyn_C;
    Dyn_Ext = Dyn_C;
    Dyn_Plf_C = Dyn_C; Dyn_Plf_G = Dyn_C; Dyn_Plf_A = Dyn_C;
    Dyn_Plf_Tau = Dyn_C; Dyn_Plf_S = Dyn_C; Dyn_Plf_Tau_Static = Dyn_C;
    Dyn_Plf_Ext = Dyn_C;
    for i = 1:size(Q,1)
      % Dynamik berechnen. Siehe cds_obj_dependencies und ParRob.
      % Trajektorie in Plattform-Koordinaten umrechnen
      [xP, xPD, xPDD] = R.xE2xP(X(i,:)', XD(i,:)', XDD(i,:)');
      % Jacobi-Matrix aufstellen (auf Plattform bezogen
      G_q = R.constr1grad_q(Q(i,:)', xP, true);
      G_x = R.constr1grad_x(Q(i,:)', xP, true);
      JinvP = - G_q \ G_x; % Siehe: ParRob/jacobi_qa_x
      Jinv_qaD_xD = JinvP(R.I_qa,:); % Auf Antriebsgelenke beziehen
      % Jacobi-Matrix auf Winkelgeschwindigkeiten beziehen. Siehe ParRob/jacobi_qa_x
      if size(Jinv_qaD_xD,2) == 6
        T = [eye(3,3), zeros(3,3); zeros(3,3), euljac(xP(4:6), R.phiconv_W_E)];
        Jinv_qaD_sD = Jinv_qaD_xD / T;
      else
        % Nehme an, dass keine räumliche Drehung vorliegt. TODO: Fall 3T2R
        Jinv_qaD_sD = Jinv_qaD_xD;
      end
      % Einzelne Terme der Dynamik berechnen ...
      Dyn_Plf_C(i,:) = R.coriolisvec2_platform(Q(i,:)', QD(i,:)', xP, xPD, JinvP);
      Dyn_Plf_G(i,:) = R.gravload2_platform(Q(i,:)', xP, JinvP);
      Dyn_Plf_A(i,:) = R.inertia2_platform(Q(i,:)', xP, JinvP)*xPDD(R.I_EE);
      Dyn_Plf_S(i,:) = R.jointtorque_platform(Q(i,:)', xP, R.springtorque(Q(i,:)'), JinvP);
      Dyn_Plf_Tau_Static(i,:) = Dyn_Plf_S(i,:)' + R.invdyn2_platform(Q(i,:)', QD(i,:)', QDD(i,:)', xP, 0*xPD, 0*xPDD);
      R_0_P = eulxyz2r(xP(4:6));
      r_0_P_E = R_0_P * R.T_P_E(1:3,4);
      F_ext_P = adjoint_jacobian(r_0_P_E) * Traj_0.Fext(i,:)';
      Dyn_Plf_Ext(i,:) = F_ext_P(R.I_EE);
      Dyn_Plf_Tau(i,:) = Dyn_Plf_S(i,:)' + R.invdyn2_platform(Q(i,:)', ...
        QD(i,:)', QDD(i,:)', xP, xPD, xPDD) - Dyn_Plf_Ext(i,:)';
      % ...und auf Antriebe umrechnen.
      Dyn_C(i,:) = Jinv_qaD_sD' \ Dyn_Plf_C(i,:)';
      Dyn_G(i,:) = Jinv_qaD_sD' \ Dyn_Plf_G(i,:)';
      Dyn_A(i,:) = Jinv_qaD_sD' \ Dyn_Plf_A(i,:)';
      Dyn_S(i,:) = Jinv_qaD_sD' \ Dyn_Plf_S(i,:)';
      Dyn_Ext(i,:) = Jinv_qaD_sD' \ Dyn_Plf_Ext(i,:)';
      Dyn_Tau(i,:) = Jinv_qaD_sD' \ Dyn_Plf_Tau(i,:)';
      Dyn_Tau_Static(i,:) = Jinv_qaD_sD' \ Dyn_Plf_Tau_Static(i,:)';
    end
  end
  % Ergebnis aus der Optimierung (Fitness-Funktion)
  if ~isfield(RobotOptDetails, 'Dyn_Tau') || isempty(RobotOptDetails.Dyn_Tau)
    Dyn_Tau_fitness = NaN(size(Dyn_Tau));
  else
    Dyn_Tau_fitness = RobotOptDetails.Dyn_Tau;
  end
  % Vergleiche Neuberechnung mit Daten aus der Optimierung
  if ~Set.optimization.static_force_only
    test_TAU = Dyn_Tau - Dyn_Tau_fitness;
  else
    test_TAU = Dyn_Tau_Static - Dyn_Tau_fitness;
  end
  if any(abs(test_TAU(:)) > 1e-6)
    warning('Antriebsmomente konnten nicht reproduziert werden. Fehler: %1.3e', ...
      max(abs(test_TAU(:))));
  end
  % Prüfe, ob Summe stimmt
  test_TAU2 = Dyn_Tau - (Dyn_C + Dyn_G + Dyn_A + Dyn_S - Dyn_Ext);
  if any(abs(test_TAU2(:)) > 1e-6)
    warning('Summe bei Berechnung der inversen Dynamik stimmt nicht. Fehler: %1.3e', ...
      max(abs(test_TAU2(:))));
  end
end

%% Dynamik-Bild (Antriebskräfte)
if strcmp(figname, 'dynamics') && traj_available
  if settings.figure_invisible, fhdl = figure_invisible();
  else,                         fhdl = figure(); end
  clf; hold all;
  set(fhdl, 'Name', sprintf('Rob%d_P%d_Dynamik', RNr, PNr), ...
    'NumberTitle', 'off', 'color','w');
  sgtitle(sprintf('Rob.%d, P.%d: fval=%s', RNr, PNr, fval_str));
  ntau = size(Dyn_Tau, 2);
  if RobData.Type == 0
    plotunits = R.tauunit_sci;
  else
    units = reshape([R.Leg(:).tauunit_sci],R.NJ,1);
    plotunits = units(R.I_qa);
  end
  for i = 1:ntau
    subplot(ceil(sqrt(ntau)), ceil(ntau/ceil(sqrt(ntau))), i);
    hold on; grid on;
    plot(Traj_0.t, Dyn_C(:,i));
    plot(Traj_0.t, Dyn_G(:,i));
    plot(Traj_0.t, Dyn_A(:,i));
    legtxt = {'Cor.', 'Grav.', 'Acc.'};
    if any(Dyn_S(:))
      plot(Traj_0.t, Dyn_S(:,i));
      legtxt = [legtxt, 'Spring']; %#ok<AGROW>
    end
    if any(Dyn_Ext(:))
      plot(Traj_0.t, Dyn_Ext(:,i));
      legtxt = [legtxt, 'Extern']; %#ok<AGROW>
    end
    plot(Traj_0.t, Dyn_Tau(:,i));
    plot(Traj_0.t, Dyn_Tau_Static(:,i));
    plot(Traj_0.t, Dyn_Tau_fitness(:,i), '--');
    legtxt = [legtxt, 'Sum', 'Sum (Static)', 'Sum (fitness)']; %#ok<AGROW>
    ylabel(sprintf('\\tau_%d in %s', i, plotunits{i}));
  end
  legend(legtxt);
  linkxaxes;
  fname = sprintf('Rob%d_%s_P%d_Antriebskraft_Dynamik', RNr, Name, PNr);
  saveas(fhdl,     fullfile(resrobdir, [fname, '.fig']));
  export_fig(fhdl, fullfile(resrobdir, [fname, '.png']));
  fprintf('Bild %s gespeichert: %s\n', fname, resrobdir);
  if settings.delete_figure, delete(fhdl); end
end

%% Dynamik-Bild (Plattform-Kräfte, für PKM)
if strcmp(figname, 'dynamics') && traj_available && RobData.Type ~= 0
  if settings.figure_invisible, fhdl = figure_invisible();
  else,                         fhdl = figure(); end
  clf; hold all;
  set(fhdl, 'Name', sprintf('Rob%d_P%d_PlfDynamik', RNr, PNr), ...
    'NumberTitle', 'off', 'color','w');
  sgtitle(sprintf('Rob.%d, P.%d: fval=%s', RNr, PNr, fval_str));
  ntau = size(Dyn_Plf_Tau, 2);
  tauplf_names = {'fx', 'fy', 'fz', 'mx', 'my', 'mz'};
  Idx_6FG = find(R.I_EE);
  for i = 1:ntau 
    subplot(ceil(sqrt(ntau)), ceil(ntau/ceil(sqrt(ntau))), i);
    hold on; grid on;
    plot(Traj_0.t, Dyn_Plf_C(:,i));
    plot(Traj_0.t, Dyn_Plf_G(:,i));
    plot(Traj_0.t, Dyn_Plf_A(:,i));
    legtxt = {'Cor.', 'Grav.', 'Acc.'};
    if any(Dyn_Plf_S(:))
      plot(Traj_0.t, Dyn_Plf_S(:,i));
      legtxt = [legtxt, 'Spring']; %#ok<AGROW>
    end
    if any(Dyn_Plf_Ext(:))
      plot(Traj_0.t, Dyn_Plf_Ext(:,i));
      legtxt = [legtxt, 'Extern']; %#ok<AGROW>
    end
    plot(Traj_0.t, Dyn_Plf_Tau(:,i));
    plot(Traj_0.t, Dyn_Plf_Tau_Static(:,i));
    legtxt = [legtxt, 'Sum', 'Sum (Static)']; %#ok<AGROW>
    if Idx_6FG(i) < 4, plotunit = 'N'; else, plotunit = 'Nm'; end
    ylabel(sprintf('%s in %s', tauplf_names{Idx_6FG(i)}, plotunit));
  end
  legend(legtxt);
  linkxaxes;
  fname = sprintf('Rob%d_%s_P%d_Plattformkraft_Dynamik', RNr, Name, PNr);
  saveas(fhdl,     fullfile(resrobdir, [fname, '.fig']));
  export_fig(fhdl, fullfile(resrobdir, [fname, '.png']));
  fprintf('Bild %s gespeichert: %s\n', fname, resrobdir);
  if settings.delete_figure, delete(fhdl); end
end

%% Zeichnung der Roboters mit Trägheitsellipsen und Ersatzdarstellung
if strcmp(figname, 'dynparvisu')
  if settings.figure_invisible, fhdl = figure_invisible();
  else,                         fhdl = figure(); end
  clf;hold all;
  set(fhdl, 'Name', sprintf('Rob%d_P%d_DynParVisu', RNr, PNr), 'NumberTitle', 'off', 'color','w');
  sgtitle(sprintf('Rob.%d, P.%d: fval=%s', RNr, PNr, fval_str));
  plotmode = [1 3 4 5];
  plotmodenames = {'Strichmodell', 'Trägheitsellipsen', 'Entwurfsparameter (3D)', 'Kollisionsobjekte'};
  for jj = 1:4
    subplot(2,2,jj);  hold on;grid on;
    view(3); axis auto;
    if RobData.Type == 0 % Seriell
      s_plot = struct( 'straight', 0, 'mode', plotmode(jj));
      R.plot(RobotOptDetails.Traj_Q(1,:)', s_plot);
    else
      s_plot = struct( 'ks_legs', [], 'straight', 0, 'mode', plotmode(jj));
      R.plot(RobotOptDetails.Traj_Q(1,:)', X(1,:)', s_plot);
    end
    title(plotmodenames{jj});
  end
  fname = sprintf('Rob%d_%s_P%d_DynParVisu', RNr, Name, PNr);
  saveas(fhdl,     fullfile(resrobdir, [fname, '.fig']));
  export_fig(fhdl, fullfile(resrobdir, [fname, '.png']));
  fprintf('Bild %s gespeichert: %s\n', fname, resrobdir);
  if settings.delete_figure, delete(fhdl); end
end
%% Optimierungsparameter (über die Pareto-Front)
if strcmp(figname, 'optpar')
  if settings.figure_invisible, fhdl = figure_invisible();
  else,                         fhdl = figure(); end
  clf;hold all;
  set(fhdl, 'Name', sprintf('Rob%d_OptPar', RNr), 'NumberTitle', 'off', 'color','w');
  % Konvertiere die normierten Optimierungsparameter nach physikalischen
  % Parametern
  p_val_pareto_phys = NaN(size(RobotOptRes.p_val_pareto));
  for i = 1:size(RobotOptRes.p_val_pareto,1)
    p_val_pareto_phys(i,:) = cds_update_robot_parameters(R, Set, ...
      RobotOptRes.Structure, RobotOptRes.p_val_pareto(i,:)');
  end
  % Konvertiere die Parametergrenzen in der Optimierung. Nehme schon hier
  % den Betrag, da sonst die kleine Skalierung mit dem negativem Vorzeichen
  % die untere Grenze bildet.
  varlim_phys = NaN(size(RobotOptRes.Structure.varlim'));
  for i = 1:size(varlim_phys,1)
    varlim_phys(i,:)= cds_update_robot_parameters(R, Set, ...
      RobotOptRes.Structure, abs(RobotOptRes.Structure.varlim(:,i)));
  end
  % Nehme den Betrag der Parameter (einzig negativ können DH-Längen sein),
  % das gleicht sich aber über die Kinematik aus
  p_val_pareto_phys = abs(p_val_pareto_phys);
  varlim_phys = abs(varlim_phys);
  % Zeichne Subplots für alle Optimierungsparameter
  varnames = RobotOptRes.Structure.varnames;
  nrows = ceil(sqrt(length(varnames)-1));
  ncols = ceil((length(varnames)-1)/nrows);
  for i = 2:length(varnames)
    subplot(nrows, ncols, i-1);hold on;
    plot(p_val_pareto_phys(:,i));
    plot([1;size(p_val_pareto_phys,1)], varlim_phys(1,i)*[1;1], 'r-');
    plot([1;size(p_val_pareto_phys,1)], varlim_phys(2,i)*[1;1], 'r-');
    ylabel(varnames{i}, 'interpreter', 'none'); grid on;
    % Grenzen des Plots manuell setzen, damit die Parameter-Grenzen nicht
    % die Plot-Grenzen definieren. Die Parameter-Grenzen können wegen der
    % Skalierung sehr groß werden.
    set(gca, 'ylim', minmax2(p_val_pareto_phys(:,i)')+0.05*...
      diff(minmax2(p_val_pareto_phys(:,i)'))*[-1,1])
  end
  linkxaxes
  sgtitle(sprintf('Rob.%d Opt.Par. (%s)', RNr, Name));
  fname = sprintf('Rob%d_%s_OptPar', RNr, Name);
  saveas(fhdl,     fullfile(resrobdir, [fname, '.fig']));
  export_fig(fhdl, fullfile(resrobdir, [fname, '.png']));
  fprintf('Bild %s gespeichert: %s\n', fname, resrobdir);
  if settings.delete_figure, delete(fhdl); end
end

%% Animation der Gelenkfeder (Ruhelage bis Einbaulage)
if strcmp(figname, 'springrestpos')
  if RobData.Type ~= 2 % nur für PKM definiert.
    error('Bild für Roboter-Typ nicht definiert');
  end
  % Erstelle eine Animation für das Einbauen der Feder. Beginnend in
  % Ruhelage, endend in Startpose der Trajektorie
  if settings.figure_invisible, fhdl = figure_invisible();
  else,                         fhdl = figure(); end
  clf;hold all;
  set(fhdl, 'Name', sprintf('Rob%d_P%d_federruhelage', RNr, PNr), ...
    'NumberTitle', 'off', 'color','w');
  title(sprintf('Rob.%d, P.%d %s Gelenkelastizität', RNr, PNr, Name));
  view(3); axis auto; hold on; grid on;
  xlabel('x in m');ylabel('y in m');zlabel('z in m');
  plot3(Traj.X(:,1), Traj.X(:,2),Traj.X(:,3), 'k-');
  s_anim = struct('mp4_name', fullfile(resrobdir, ...
      sprintf('Rob%d_%s_P%d_FederRuhelage_Anim.mp4', RNr, Name, PNr)));
  s_plot = struct( 'ks_legs', [], 'straight', 1, 'mode', 1);
  % Trajektorie erzeugen.
  q_start = RobotOptDetails.Traj_Q(1,:)'; q_end = q_start;
  for i = 1:R.NLEG
    q_start([false(R.I1J_LEG(i)-1,1);R.Leg(i).MDH.sigma==0]) = R.Leg(i).DesPar.joint_stiffness_qref(R.Leg(i).MDH.sigma==0);
  end
  [Q_ges,~,~,T_ges] = traj_trapez2_multipoint([q_start';q_end'], 1, 0.05, 0.01, 0.005, 0);
  % Video auf Länge 10s bringen
  t_Vid = (0:1/30*(T_ges(end)/10):T_ges(end))';
  I_anim = knnsearch( T_ges, t_Vid ); % Indizes für gewünschte Länge
  R.anim( Q_ges(I_anim,:), repmat(X(1,:),length(I_anim),1), s_anim, s_plot);
end

% EE-FG von PKM wieder zurücksetzen auf Aufgaben-FG
if RobData.Type ~= 0
  R.update_EE_FG(R.I_EE, Set.task.DoF);
end
