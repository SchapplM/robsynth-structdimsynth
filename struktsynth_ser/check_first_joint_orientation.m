% Prüfe die Ausrichtung des ersten Gelenks aller Roboter aus der Datenbank

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-05
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
clc
EE_FG_Mask = [1 1 1 1 1 1];

% Fall 1:
% N_LegDoF = 6;
% EE_FG = [1 1 1 1 1 1];
% Fall 2:
% N_LegDoF = 3;
% EE_FG = [1 1 0 0 0 1];
% Fall 2:
% N_LegDoF = 3;
% EE_FG = [1 1 1 0 0 0];
% Fall 3:
N_LegDoF = 4;
EE_FG = [1 1 1 0 0 1];

roblibpath=fileparts(which('serroblib_path_init.m'));
if isempty(roblibpath)
  error('SerRobLib ist nicht im Pfad. Skript "serroblib_path_init.m" ausführen!');
end
mdllistfile_Ndof = fullfile(roblibpath, sprintf('mdl_%ddof', N_LegDoF), sprintf('S%d_list.mat',N_LegDoF));
l = load(mdllistfile_Ndof, 'Names_Ndof', 'AdditionalInfo');
II = serroblib_filter_robots(N_LegDoF, EE_FG, EE_FG_Mask);

firstaxis = NaN(length(II),2);
ii=0;
for iFK = II(1:end)'
  ii = ii + 1;
  Name = l.Names_Ndof{iFK};
  [~,PS] = serroblib_create_robot_class(Name,[],true);
  
  % Speichere Marker für Dreh-/Schubgelenk
  firstaxis(ii,1) = PS.sigma(1);
  
  % Merke Ausrichtung der Achse
  if PS.alpha(1) == 0 && PS.beta(1) == 0
    firstaxis(ii,2) = 3; % z-Achse
  elseif PS.alpha(1) == pi/2 && PS.beta(1) == pi
    firstaxis(ii,2) = 2; % y-Achse
  elseif PS.alpha(1) == pi/2 && PS.beta(1) == pi/2
    firstaxis(ii,2) = 1; % x-Achse
  else
    error('Fall nicht definiert');
  end
  
  % Auswertung
  if all(EE_FG == [1 1 1 1 1 1]) && firstaxis(ii,2)~=3
    error('Die erste Achse des 3T3R-Roboters %s ist nicht z. Das ist komisch', Name);
  end
  if all(EE_FG == [1 1 0 0 0 1]) && firstaxis(ii,1)==0 && firstaxis(ii,2)~=3
    error('Die erste Achse des 2T1R-Roboters %s ist Drehachse, aber nicht z. Das ist komisch', Name);
  end
  if all(EE_FG == [1 1 0 0 0 1]) && firstaxis(ii,1)==1 && firstaxis(ii,2)~=2
    error('Die erste Achse des 2T1R-Roboters %s ist Schubachse, aber nicht y. Das ist komisch', Name);
  end
end