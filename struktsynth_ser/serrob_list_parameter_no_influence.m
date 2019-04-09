% Liste aller Parameter ohne Einfluss auf die EE-FG in der Datenbank

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-04
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
clc

%% Initialisierung
roblibpath=fileparts(which('serroblib_path_init.m'));

serroblib_gen_bitarrays(1:7);

%% Durchsuche alle Roboter und prüfe die Kinematikparameter
% Zuordnung der Zahlenwerte in der csv-Tabelle zu den physikalischen Werten
% Die in der mat-Datei abgelegte Binär-Kodierung entspricht der Reihenfolge
for N = 3:7
  fprintf('Prüfe Strukturen mit %d FG\n', N);
  % Alle Roboter aus Datenbank laden
  mdllistfile_Ndof = fullfile(roblibpath, sprintf('mdl_%ddof', N), sprintf('S%d_list.mat',N));
  l = load(mdllistfile_Ndof, 'Names_Ndof', 'BitArrays_Ndof', 'BitArrays_EEdof0');
  KinParNonRel = false(length(l.Names_Ndof), 1);
  for j = 1:length(l.Names_Ndof)
    %% Initialisierung
    % Marker, ob EE-Transformation für diese Struktur noch undefiniert ist.
    % Dann sind in der csv-Tabelle "?" als Platzhalter für phix_NE gesetzt
    undef = false;
    RobName = l.Names_Ndof{j};
    fprintf('%d/%d: Prüfe Struktur %s\n', j, length(l.Names_Ndof), RobName);
    % Roboterklasse erstellen
    RS = serroblib_create_robot_class(RobName);
    RS.update_mdh(rand(length(RS.pkin),1));
    % Zufällige Parameter setzen
    % EE-FG des Roboters
    [~,EEFG0] = serroblib_bits2csvline_EE(l.BitArrays_EEdof0(j,:));
    Ipkinrel = RS.get_relevant_pkin(logical(EEFG0([1:3,7:9])));
    parnonrelstr = sprintf('%d/%d Kinematik-Parameter haben keinen Einfluss: [', sum(~Ipkinrel), length(Ipkinrel));
    for ii = find(~Ipkinrel)'
      parnonrelstr = [parnonrelstr, ' ', RS.pkin_names{ii}]; %#ok<AGROW>
    end
    parnonrelstr = [parnonrelstr, ']']; %#ok<AGROW>
    fprintf('\t%s\n', parnonrelstr);
    % RS.get_pkin_parameter_type
    if any(~Ipkinrel)
      % Setze Marker, dass mindestens ein Parameter überflüssig ist
      KinParNonRel(j) = true;
    end
  end
  fprintf('Bei %d/%d Systemen mit %d FG sind Kinematikparameter überflüssig\n', ...
    sum(KinParNonRel), length(KinParNonRel), N);
end
