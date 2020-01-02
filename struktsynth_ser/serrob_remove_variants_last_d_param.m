% Entferne Modellvarianten, die nur daraus bestehen, den letzten
% d-Parameter auf Null zu setzen. In der Maßsynthese für PKM kann dieser
% Parameter auch direkt weggelassen werden. Die große Anzahl an Modell-
% Varianten macht das ganze nur unübersichtlich.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-12
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
clc

%% Initialisierung
roblibpath=fileparts(which('serroblib_path_init.m'));
serroblib_gen_bitarrays(1:7);

%% Durchsuche alle Roboter und stelle die korrekte Orientierung des Endeffektors fest
List_Remove = {};
for N = 3:6
  fprintf('Prüfe Strukturen mit %d FG\n', N);
  % Alle Roboter aus Datenbank laden
  mdllistfile_Ndof = fullfile(roblibpath, sprintf('mdl_%ddof', N), sprintf('S%d_list.mat',N));
  l = load(mdllistfile_Ndof, 'Names_Ndof', 'AdditionalInfo', 'BitArrays_Origin');
  I_noorigin = (l.BitArrays_Origin == 0);
  I_isvar = (l.AdditionalInfo(:,2)==true); % Auswahl aller Roboter, die keine Variante sind
  I_check = I_isvar & I_noorigin;
  i = 0;
  for j = find(I_check)'
    i = i+1;
    RobName_var = l.Names_Ndof{j};
    fprintf('%d/%d: Prüfe Struktur %s (%d/%d gesamt)\n', i, sum(I_check), ...
      RobName_var, j, length(l.Names_Ndof));
    % Nummer des Hauptmodells dieser Variante
    k = l.AdditionalInfo(j,3);
    RobName_gen = l.Names_Ndof{k};
    % Beide Robotermodelle erzeugen
    RS_var = serroblib_create_robot_class(RobName_var);
    RS_gen = serroblib_create_robot_class(RobName_gen);
    % Prüfe, ob der letzte d-Parameter der einzige Unterschied ist
    RS_gen.MDH.d(end) = 0; % Setze dafür den d-Parameter auf Null. Alles andere ist noch NaN.
    RS_gen.update_pkin();
    % Gehe alle pkin-Einträge der Variante durch und vergleiche sie mit dem
    % allgemeinen Modell
    I_diff = true(length(RS_gen.pkin),1);
    for ii = 1:length(RS_var.pkin)
      % Indizes der Kinematik-Parameter im allgemeinen Modell, die
      % identisch mit Parameter ii des Varianten-Modells sind
      I_equal = (RS_var.pkin_types(ii) == RS_gen.pkin_types) & ...
                (RS_var.pkin_jointnumber(ii) == RS_gen.pkin_jointnumber);
      II_equal = find(I_equal);
      if length(II_equal) == 1
        % Der Parameter Nummer ii des Varianten-Modells ist der gefundene
        % gleiche Parameter
        I_diff(II_equal) = false;
      end
    end
    if sum(I_diff) ~= 1
      % Es ist mehr als ein Parameter unterschiedlich zwischen Variante und
      % Hauptmodell. Es kann keine der gesuchten d6-Varianten sein.
      continue
    end
    if RS_gen.pkin(I_diff) ~= 0
      % Es ist nur ein Parameter unterschiedlich, aber es ist anscheinend
      % nicht der am Anfang geänderte Parameter. Komisch
      error('Das sollte eigentlich nicht vorkommen');
    end
    fprintf('Variante %s unterscheidet sich nur hinsichtlich d%d von Hauptmodell %s. Entferne.\n', ...
      RobName_var, N, RobName_gen);
    List_Remove = [List_Remove(:); RobName_var]';
  end
end

%% Entfernen der Roboter. Das muss manuell passieren.
fprintf('Es können %d Varianten mit dem Filterkriterium gelöscht werden:\n', length(List_Remove));
disp(List_Remove);
return
for i = 1:length(List_Remove)' %#ok<UNRCH>
  Name_i = List_Remove{i};
  success = serroblib_remove_robot(Name_i);
  if success
    fprintf('[%d/%d] Variante %s aus Datenbank entfernt\n', ...
      i, length(List_Remove), Name_i);
  else
    error('Fehler beim Löschen der Modellvariante');
  end
end