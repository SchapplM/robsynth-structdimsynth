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

for j = 1:length(Structures)
  if Set.general.matfile_verbosity > 1
    save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_create_evolution_videos2.mat'));
  end
  %% Initialisierung der Ergebnisse dieser Struktur
  % load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_create_evolution_videos2.mat'));
  Structure = Structures{j};
  Name = Structures{j}.Name;
  resdir_pso = fullfile(resmaindir, ...
    'tmp', sprintf('%d_%s', Structure.Number, Structure.Name));
  videofile_avi = fullfile(resmaindir, sprintf('Rob%d_%s_Evolution_Video.avi', ...
    Structure.Number, Structure.Name));
  
  fprintf('Erstelle Evolutions-Video für %s\n', Name);
  
  % Ergebnis-Bild für Video initialisieren
  figure(1);clf;

  filedat_detailimg = dir(fullfile(resdir_pso, 'PSO_Gen*_FitEval*_Details.fig'));
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

  for i = 1:length(filedat_detailimg)
    fprintf('%d/%d (%1.1f%%): %s; %1.0fs nach Start. Verbleibend ca. %1.0fs\n', ...
      i, length(filedat_detailimg), 100*i/length(filedat_detailimg), ...
      filedat_detailimg(i).name, toc(t1), toc(t1)/i*(length(filedat_detailimg)-i))
    uiopen(fullfile(filedat_detailimg(i).folder, filedat_detailimg(i).name),1)
    set(gcf, 'windowstyle', 'normal')
    set(gcf,'units','normalized','outerposition',[0 0 1 1]);
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
        if Set.general.matfile_verbosity > 1
          save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_create_evolution_videos3.mat'));
        end
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
    % Abschluss
    writeVideo(v,f);
    close(gcf);
  end
  close(v);
  
  %% Video komprimieren
  compress_video_file(videofile_avi);
end