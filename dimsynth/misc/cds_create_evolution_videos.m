% Erstelle ein Video aller Zwischenschritte bei der Synthese einer Struktur
% 
% Ergebnis: Sehr große mp4-Datei mit komplettem Verlauf für Roboter

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function cds_create_evolution_videos(Set, Traj, Structures)
if Set.general.matfile_verbosity > 0
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_create_evolution_videos1.mat'));
end
if ~Set.general.save_evolution_video
  return
end

resmaindir = fullfile(Set.optimization.resdir, Set.optimization.optname);
length_Structures_parfor = length(Structures);
if Set.general.parcomp_plot && length_Structures_parfor > 0
  % Gehe davon aus, dass der parfor-Pool bereits läuft.
  Pool = gcp();
  parfor_numworkers = Pool.NumWorkers;
  % Warnungen auch in ParPool-Workern unterdrücken: https://github.com/altmany/export_fig/issues/75
  parfevalOnAll(gcp(), @warning, 0, 'off', 'MATLAB:prnRenderer:opengl');
else
  parfor_numworkers = 0;
end
if parfor_numworkers > 0
  close all % Speicher freigeben
end
if Set.general.isoncluster
  use_tmpdir = true;
else
  use_tmpdir = false;
end
parfor (j = 1:length_Structures_parfor, parfor_numworkers)
  % Auflösung für Debug-Bilder setzen (wird auf ParPool auf Cluster nicht
  % vererbt aus globalen Einstellungen)
  pause(2*(j-1)); % Damit nicht alle parfor-Worker gleichzeitig starten (unklar ob notwendig)
  if parfor_numworkers > 0
    close all % Speicher freigeben (evtl. Bilder auf Worker offen)
    set(0, 'defaultfigureposition', [1 1 1920 1080]);
    set(0, 'defaultfigureunits', 'pixels');
  end
%   if Set.general.matfile_verbosity > 1
%     save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_create_evolution_videos2.mat'));
%   end
  %% Initialisierung der Ergebnisse dieser Struktur
  % load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_create_evolution_videos2.mat'));
  Structure = Structures{j};
  Name = Structures{j}.Name;
  resdir_pso = fullfile(resmaindir, 'tmp', sprintf('%d_%s', Structure.Number, Name));
  RobPrefix = sprintf('Rob%d_%s', Structure.Number, Name);
  if use_tmpdir
    videofile_avi = fullfile(tmpDirFcn(), ...
      sprintf('%s_Evolution_Video.avi', RobPrefix));
  else
    videofile_avi = fullfile(resmaindir, RobPrefix, ...
      sprintf('%s_Evolution_Video.avi', RobPrefix));
  end
  fprintf('Erstelle Evolutions-Video für %s\n', Name);
  
  % Ergebnis-Bild für Video initialisieren
  figure(1);clf;

  filedat_detailimg = dir(fullfile(resdir_pso, 'Gen*_Ind*_Eval1_Details.fig'));
  if length(filedat_detailimg) < 10
    warning('Es liegen nur %d Detailbilder im fig-Format im Ordner %s. Video nicht sinnvoll', ...
      length(filedat_detailimg), resdir_pso);
    continue
  end
  if exist(videofile_avi, 'file'), delete(videofile_avi); end
  v = VideoWriter(videofile_avi, 'Uncompressed AVI'); %#ok<TNMLP>
  open(v);
  fprintf('Schreibe Video-Datei %s\n', videofile_avi);
  t1=tic();
  res_1 = []; % Initialisierung gegen parfor-Warnung
  for i = 1:length(filedat_detailimg)
    fprintf('%d/%d (%1.1f%%): %s; %1.0fs nach Start. Verbleibend ca. %1.0fs\n', ...
      i, length(filedat_detailimg), 100*i/length(filedat_detailimg), ...
      filedat_detailimg(i).name, toc(t1), toc(t1)/i*(length(filedat_detailimg)-i))
    uiopen(fullfile(filedat_detailimg(i).folder, filedat_detailimg(i).name),1)
    [tokens_img,~] = regexp(filedat_detailimg(i).name,'Gen(\d+)_Ind(\d+)_Eval(\d+)','tokens','match');
    title(sprintf('Gen %s, Ind %s', tokens_img{1}{1}, tokens_img{1}{2}));
    set(gcf, 'windowstyle', 'normal')
    set(gcf,'color','w');
    f=getframe(gcf);
    % Zuschneiden auf Mod32 für anschließende Kompression
    res_tmp = size(f.cdata);
    res_crop = floor(res_tmp(1:2)/32)*32;
    crop_begin = ceil((res_tmp(1:2)-res_crop(1:2))/2);
    crop_end = floor((res_tmp(1:2)-res_crop(1:2))/2);
    f.cdata = f.cdata((1+crop_begin(1)):(res_tmp(1)-crop_end(1)), ...
                      (1+crop_begin(2)):(res_tmp(2)-crop_end(2)), :);
    % Prüfen, ob Auflösung gleich geblieben ist.
    if i == 1
      res_1 = size(f.cdata);
    else
      if any(size(f.cdata)~=res_1)
%         if Set.general.matfile_verbosity > 1
%           save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_create_evolution_videos3.mat'));
%         end
        warning('Auflösung des aktuellen Bildes (%d x %d) stimmt nicht mit der des ersten Bildes (%d x %d)', ...
          size(f.cdata,1), size(f.cdata,2), res_1(1), res_1(2));
        % Prüfe, ob das Video noch zu retten ist. Wenn nicht. Video bis
        % hier hin erzeugen.
        if size(f.cdata, 1) < res_1(1) || size(f.cdata, 2) < res_1(2)
          warning('Das aktuelle Bild ist kleiner. Kein Zuschneiden möglich. Auffüllen nicht implementiert. Abbruch.');
          break;
        end
        % Zuschneiden
        f.cdata = f.cdata(1:res_1(1), 1:res_1(2), :);
      end
    end
    % Einzelbild so oft wiederholt ins Video hineinschreiben, dass die
    % gewünschte Anzeigedauer entsteht. Annahme: Video hat 30fps
    for k = 1:ceil(Set.general.evolution_video_frametime/(1/30)) %#ok<PFBNS>
      writeVideo(v,f);
    end
    close(gcf);
  end
  close(v);
  
  %% Video komprimieren
  compress_video_file(videofile_avi);
  % Komprimiertes Video an Zielort verschieben
  if use_tmpdir
    [d,f] = fileparts(videofile_avi);
    movefile(fullfile(d, [f, '.mp4']), ...
             fullfile(resmaindir, RobPrefix, [f, '.mp4']));
  end
end
