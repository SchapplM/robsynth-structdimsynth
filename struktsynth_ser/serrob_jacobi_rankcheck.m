% Stelle die EE-FG der Strukturen in der Datenbank fest und bestimme den
% Rang der Jacobi-Matrix. Wenn das System nicht vollen Rang hat, entferne
% es.
% Siehe auch: serrob_mdlbib/scripts/determine_ee_dof.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-09
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
clc

%% Initialisierung
roblibpath=fileparts(which('serroblib_path_init.m'));
serroblib_gen_bitarrays(1:7);

% Einstellungen
usr_overwrite = true; % Überschreibe auch bestehende Einträge. Sinnvoll, wenn die Spalten vorher leer sind
usr_abortonerror = false; % Bei irgendeinem Fehler anhalten
usr_codegen_missing = true; % Bei fehlendem Code neu generieren
usr_plot_robot = false; % Bild des Roboters erstellen
usr_plot_robot_only_on_error = true; % Bild nur bei Rangverlust erstellen (zur Diagnose)
usr_break_at_rankloss = false; % Abbruch bei Erkennung eines Rangverlusts der Gesamt-Jacobi

usr_RobName_List = {};
usr_RobName_List = {'S5PRRRR5',	'S5RPRRR3',	'S5RPRRR11',	'S5RRPRR11',	'S5RRRPR6',	'S5RRRRP5',	'S5RRRRR4'};
% usr_RobName_List = {'S5RRRRP5'};
%% Durchsuche alle Roboter und prüfe die Kinematikparameter
% Zuordnung der Zahlenwerte in der csv-Tabelle zu den physikalischen Werten
% Die in der mat-Datei abgelegte Binär-Kodierung entspricht der Reihenfolge
Bad_Robot_List = {};
for N = 1:6
  fprintf('Prüfe Strukturen mit %d FG\n', N);
  % Alle Roboter aus Datenbank laden
  mdllistfile_Ndof = fullfile(roblibpath, sprintf('mdl_%ddof', N), sprintf('S%d_list.mat',N));
  l = load(mdllistfile_Ndof, 'Names_Ndof', 'AdditionalInfo', 'BitArrays_Origin');
  for j = 1:length(l.Names_Ndof)
    RobName = l.Names_Ndof{j};
    if ~isempty(usr_RobName_List) && ~any(strcmp(usr_RobName_List, RobName))
      continue
    end
    fprintf('%d/%d: Prüfe Struktur %s\n', j, length(l.Names_Ndof), RobName);
    hascode = (l.AdditionalInfo(j,4) == 1);
    isvariant = (l.AdditionalInfo(j,2) == 1);
    % Wenn der Roboter manuell erstellt wurde, wird er nicht zur Löschung
    % vorgeschlagen
    generatedmanually = (l.BitArrays_Origin(j) == uint16(bin2dec('00001')));
    if ~hascode && ~isvariant && usr_codegen_missing
      fprintf('Keine Kinematik-Funktionen für %s verfügbar. Generiere neu.\n', RobName);
      serroblib_generate_mapleinput({RobName});
      serroblib_generate_code({RobName}, true, false, 4); % Nur Kinematik generieren
      serroblib_gen_bitarrays(N); % Damit Informationen aktualisiert werden
      serroblib_create_template_functions({RobName}, false, false);
    end
    try
      RS = serroblib_create_robot_class(RobName);
      RS.gen_testsettings(true, true);
    catch
      warning('Fehler für Modell %s', RobName);
      if usr_abortonerror
        return
      else
        % Füge den Roboter auf die Liste der zu löschenden Roboter hinzu.
        % Das ist der Fall, wenn ein bereits gelöschter Roboter neu
        % eingetragen wurde und kein Code vorliegt.
        Bad_Robot_List = {Bad_Robot_List{:}, RobName}; %#ok<CCAT>
        continue
      end
    end
    %% Prüfe, welche EE-FG die Struktur hat
    % Prüfe, für jedes Gelenk beginnend von hinten, ob es die EE-Position
    % beeinflusst
    
    q = rand(N,1);
    qD = rand(N,1);
    % Berechne Kinematik des letzten Gelenks (PKM-Plattform-Koppelpunkt)
    RS.update_EE(zeros(3,1));
    % Setze die EE-Rotation (falls nicht in Datenbank vorhanden)
    if any(isnan(RS.phi_N_E))
      RS.update_EE([], rand(3,1));
    end
    JgK = RS.jacobig(q);
    vK = JgK*qD;
    JaK = RS.jacobia(q);
    T_K = RS.fkineEE(q);
    xDK = JaK*qD;
    rank_JgK = rank(JgK);
    rank_JaK = rank(JaK);
    
    % Berechne Kinematik eines zusätzlichen Endeffektors (serieller Rob.)
    RS.update_EE(rand(3,1));
    JgE = RS.jacobig(q);
    vE = JgE*qD;
    JaE = RS.jacobia(q);
    T_E = RS.fkineEE(q);
    xDE = JaE*qD;
    rank_JgE = rank(JgE);
    rank_JaE = rank(JaE);
    
    if any(isnan(xDK))
      warning('%s: Euler-Geschwindigkeit kann wegen Singularität nicht berechnet werden', RobName);
      if usr_abortonerror
        return
      else
        continue
      end
    end
    
    rankloss = false;
    if rank_JgK ~= N
      if generatedmanually
        fprintf('Die serielle Kette %s hat Rangverlust, wurde aber manuell in die DB eingetragen\n', RobName);
        continue
      end
      if rank_JgE == N
        warning('Die serielle Kette %s hat als PKM-Beinkette eine geringere Mobilität als sie haben sollte. Als serieller Roboter ist sie möglich.', RobName);
      else
        warning('Die serielle Kette %s hat als PKM-Beinkette eine geringere Mobilität als sie haben sollte und funktioniert nicht als serieller Roboter.', RobName);
      end
      rankloss = true;
      Bad_Robot_List = {Bad_Robot_List{:}, RobName}; %#ok<CCAT>
    end
    
    %% Roboter zeichnen
    if usr_plot_robot || ~usr_plot_robot && usr_plot_robot_only_on_error && rankloss
      s_plot = struct( 'ks', [1:RS.NJ+1, RS.NJ+2], 'straight', 0);
      change_current_figure(1);clf;
      hold on;  grid on;
      xlabel('x in m'); ylabel('y in m'); zlabel('z in m');
      view(3);
      RS.plot( q, s_plot );
      title(sprintf('%s', RobName));
    end
    if usr_break_at_rankloss
      return;
    end
  end
end

fprintf('Entferne folgende %d Roboter:\n', length(Bad_Robot_List));
for i = 1:length(Bad_Robot_List)
  fprintf('%s\t', Bad_Robot_List{i});
  if mod(i,10)==0 || i == length(Bad_Robot_List), fprintf('\n');end
end
fprintf('Das Entfernen muss händisch erfolgen.\n');
% Das sollte händisch gemacht geprüft werden. Skript
return
for i = 1:length(Bad_Robot_List)
  success = serroblib_remove_robot(Bad_Robot_List{i});
  if success
    fprintf('Roboter %s erfolgreich gelöscht\n', Bad_Robot_List{i});
  else
    fprintf('Roboter %s konnte nicht gelöscht werden\n', Bad_Robot_List{i});
  end
end
% Danach muss die Datenbank neu geladen werden
serroblib_gen_bitarrays
