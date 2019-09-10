% Liste aller Isomorphismen in der Datenbank serieller Roboter
% 
% Neue Definition: Relativer Bezug zwischen Gelenken ist gleich

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-02
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
clc

%% Benutzereingabe
% Benutzereingaben: EE-FG, Robo
EE_FG = [1 1 1 0 0 1]; % Bsp.: 110001="SCARA"-Bewegung
EE_FG_Mask = [1 1 1 1 1 1]; % Die FG müssen genauso auch vom Roboter erfüllt werden (0 darf nicht auch 1 sein)
NDoF = sum(EE_FG);

%% Initialisierung
roblibpath=fileparts(which('serroblib_path_init.m'));
if isempty(roblibpath)
  error('SerRobLib ist nicht im Pfad. Skript "serroblib_path_init.m" ausführen!');
end
mdllistfile_Ndof = fullfile(roblibpath, sprintf('mdl_%ddof', NDoF), sprintf('S%d_list.mat',NDoF));
l = load(mdllistfile_Ndof, 'Names_Ndof', 'BitArrays_Ndof', 'BitArrays_EEdof0');

%% Durchsuche alle Roboterstrukturen nach Isomorphismen
II = serroblib_filter_robots(NDoF, EE_FG, EE_FG_Mask);

for I = II'
  % Roboter aus Datenbank extrahieren
  l.Names_Ndof{I};
  csvline_J = serroblib_bits2csvline(l.BitArrays_Ndof(I,:));
  csvline_EE = serroblib_bits2csvline_EE(l.BitArrays_EEdof0(I,:));
  csvline = {csvline_J{:}, '0', '0', '0', csvline_EE{:}, '0'}; %#ok<CCAT>
  
  % Vorlage für folgenden Code: serroblib_find_robot
  % [found, index, num] = serroblib_find_robot(csvline);

  BA = serroblib_csvline2bits(csvline);
  N = length(BA); % Anzahl Gelenke

  % Filter-Variable für erstes Gelenk: Die ersten Parameter beta, b, alpha, a
  % sollen nicht ausgewertet werden
  BAJ1_Filter = uint16(zeros(1,N));
  for i = 1:N
    BAJ1_Filter(i) = intmax;
  end
  % Setze alle Bits auf 1, bis auf die Bits der Orientierung der ersten Achse
  for i = 2:13 % Bits von beta bis theta
    BAJ1_Filter(1) = bitset(BAJ1_Filter(1), i, 0);
  end % dec2bin(BAJ1_Filter)


  num = 0;
  index_type = [];
  index_N = [];
  found = 0;

  for i = 1:size(l.BitArrays_Ndof, 1) % Alle Roboterstrukturen aus Datenbank durchgehen
    % Zähle, wie viele Roboter dieser Gelenkreihenfolge existieren
    % (das erste Bit kennzeichnet den Gelenktyp)
    num = num + all(bitand(BA, 1) == bitand(l.BitArrays_Ndof(i,:), 1));

    % Prüfe ob alle Bits übereinstimmen (alle MDH-Parameter)
    if(all( bitand(BA,BAJ1_Filter) == bitand(l.BitArrays_Ndof(i,:),BAJ1_Filter) ))
      % Suche nur Vorkommnisse bis zu der Nummer des betrachteten Roboters
      if i > I
        break
      end
      
      % Eintrag ist die gesuchte Nummer (bezogen auf die Roboter derselben
      % Gelenkreihenfolge)
      found = found+1;
      index_type(found) = num; %Index in den Robotern mit derselben Gelenkreihenfolge
      index_N(found) = i; % Index in allen Robotern mit derselben Anzahl Gelenk-FG
    end
  end

  if length(index_N) > 1
    Namen_Doppelt = {};
    Namen_String = [];
    for i = 1:length(index_N)
      Namen_Doppelt{i} = l.Names_Ndof{index_N(i)};
      Namen_String = [Namen_String, Namen_Doppelt{i}];
      if i < length(index_N)
        Namen_String = [Namen_String, ', '];
      end
    end
    % Isomorphismen entfernen
    fprintf('Die %d Strukturen [%s] sind Isomorphismen\n', length(index_N), Namen_String);
    success = serroblib_remove_robot(Namen_Doppelt{2});
    if success
      fprintf('Struktur %s aus Datenbank gelöscht\n', Namen_Doppelt{2});
    end
  end
end
% Roboterliste aus geänderten CSV-Dateien neu generieren
serroblib_gen_bitarrays(N)
return
%% Isomorphismen entfernen
mdlname_del = 'S4PPPR4';

