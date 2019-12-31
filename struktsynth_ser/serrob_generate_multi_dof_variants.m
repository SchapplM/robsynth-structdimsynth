% Ändere die Kinematikparameter aller Robotermodelle so, dass das alle
% Varianten von Kugel- und Kardangelenken enstehen
% 
% Notwendige Schritte nach dem Erzeugen dieser Varianten:
% * generate_variant_pkin_conv_fcns.m
% * correct_phi_N_E;
% * determine_ee_dof;
% * determine_jointnumber_influence_ee_position;
% * determine_multi_dof_joints.m
% * write_structsynth_origin.m
% 
% Siehe auch: serrob_change_last_joint_to_spherical.m, determine_multi_dof_joints.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-12
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
clc

%% Initialisierung
roblibpath=fileparts(which('serroblib_path_init.m'));
serroblib_gen_bitarrays(1:7);
%% Benutzereingaben
save_reslist = false; % Nur aktivieren, wenn alle Varianten gesucht werden
typestring_test = ''; %'RRRRR'; % Prüfe nur diese serielle Kette
%% Durchsuche alle Roboter und stelle die korrekte Orientierung des Endeffektors fest
num_success = 0;
num_decline_rank = 0;
num_decline_revolute_end = 0;
num_new = 0;
List_NewMultiDoFChains = {};
for N = 4:6
  fprintf('Prüfe Strukturen mit %d FG\n', N);
  % Alle Roboter aus Datenbank laden
  mdllistfile_Ndof = fullfile(roblibpath, sprintf('mdl_%ddof', N), sprintf('S%d_list.mat',N));
  l = load(mdllistfile_Ndof, 'Names_Ndof', 'AdditionalInfo', 'BitArrays_Origin');
  % Serielle Ketten, die aus Struktursynthese kommen (entweder als Roboter
  % oder als PKM-Beinkette; nicht: Manuell generiert oder aus dieser
  % Mehr-FG-Gelenk-Generierung) 
  % Reihenfolge der Bits: Siehe serroblib_csvline2bits.m, Variable BAO
  Mask_Origin = uint16(bin2dec('01110')); % Probe: dec2bin(Mask_Origin,16)
  I_synth = bitand(l.BitArrays_Origin, repmat(Mask_Origin, length(l.Names_Ndof),1))~=0;
  I_novar = (l.AdditionalInfo(:,2)==false); % Auswahl aller Roboter, die keine Variante sind
  i = 0;
  for j = find(I_synth)'
    i = i+1;
    RobName = l.Names_Ndof{j};
    fprintf('%d/%d: Prüfe Struktur %s (%d/%d gesamt)\n', i, sum(I_synth), ...
      RobName, j, length(l.Names_Ndof));
    %% Kinematikparameter so ändern, dass das mehrwertige Gelenke möglich sind
    typestring = RobName(3:3+N-1); % Roboterdaten aus Namen extrahieren
    if ~isempty(typestring_test) && ~strcmp(typestring, typestring_test)
      continue
    end
    joint_string_old = typestring;
    for jj = 2:N % Schleife zum Ignorieren des jeweiligen Gelenks
      % TODO: Diese Anordnung der Schleifen ist noch nicht sehr effizient
      % Durch diese Schleife wird das letzte Gelenk immer zum Kugelgelenk.
      % Es entstehen aber auch Varianten, bei denen das erste Gelenk kein
      % Kugelgelenk wird, wenn es auch ein Kardan-Gelenk werden kann.
      
      % Roboterklasse erstellen (muss in der Schleife passieren, damit mit
      % NaN belegte freie Parameter wieder zurückgesetzt werden
      RS = serroblib_create_robot_class(RobName);
      joint_string = typestring;
      for ii = jj:N % Schleife zum Zusammenfassen mehrere einwertiger Gelenke
        % Prüfe auf Kardangelenk
        if ii > 1 && ... % Vorkommen erst ab dem zweiten 1FG-Gelenk auffindbar
            all(RS.MDH.sigma(ii-1:ii) == 0) && ... % Zwei Drehgelenke
            RS.MDH.alpha(ii) == pi/2 && ... % Alle beide senkrecht zueinander
            (RS.MDH.a(ii) == 0 || isnan(RS.MDH.a(ii))) && ... % Gelenkachsen schneiden sich in einem Punkt
            (RS.MDH.d(ii) == 0 || isnan(RS.MDH.d(ii)))
          % Dieses Drehgelenk kann zusammen mit dem vorhergehenden ein
          % Kardan-Gelenk werden
          % Setze Abstand auf Null
          if ~any(joint_string(ii-1) == 'US')
            RS.MDH.a(ii) = 0;
            RS.MDH.d(ii) = 0;
            joint_string(ii-1:ii) = '_U'; % Entferne den ersten Eintrag
          end
        end
        % Prüfe auf Kugelgelenk
        if ii > 2 && ... % Vorkommen erst ab dem dritten 1FG-Gelenk auffindbar
            all(RS.MDH.sigma(ii-2:ii) == 0) && ... % Drei Drehgelenke
            all(RS.MDH.alpha(ii-1:ii) == pi/2) && ... % Alle drei senkrecht zueinander
            all(RS.MDH.a(ii-1:ii) == 0 | isnan(RS.MDH.a(ii-1:ii))) && ... % Gelenkachsen schneiden sich in einem Punkt
            all(RS.MDH.d(ii-1:ii) == 0 | isnan(RS.MDH.d(ii-1:ii)))
          % Dieses Drehgelenk wird zusammen mit den beiden vorherigen ein
          % Kugelgelenk.
          % Andere Gelenke wie U/C können überschrieben werden
          if ~any(joint_string(ii-2:ii-1) == 'S')
            RS.MDH.a(ii-1:ii) = 0;
            RS.MDH.d(ii-1:ii) = 0;
            joint_string(ii-2:ii) = '__S'; % Entferne die ersten beiden Einträge
          end
        end
      end
      % Prüfen
      dof = 3*sum(joint_string=='S') + 2*sum(joint_string=='U') + 2*sum(joint_string=='C') + ...
            1*sum(joint_string=='R') + 1*sum(joint_string=='P');
      if dof ~= N
        error('Die Anzahl der Gelenk-FG stimmt nicht mehr nach der Ersetzung');
      end
      % Kopie des Roboters erstellen und Eintragen
      if strcmp(joint_string, typestring)
        % Keine neue Gelenkfolge gefunden. Nichts eintragen
        continue
      end
      if strcmp(joint_string_old, joint_string)
        % In diesem Schleifendurchlauf jj wurde das gleiche Ergebnis wie im
        % vorherigen gefunden
        continue
      elseif jj>2
        debugpoint = 1;
        fprintf('\tWeitere Möglichkeit für diese Kette gefunden. Vorher: %s, Jetzt: %s\n', ...
          joint_string_old, joint_string);
      end
      joint_string_old = joint_string;
      pkin_orig = RS.pkin;
      pkin = RS.update_pkin(); % Aktualisierte DH-Parameter eintragen
      if all(isnan(pkin_orig) == isnan(pkin))
        % Der neue Roboter ist identisch mit dem alten. Es muss nichts
        % eingetragen werden
        continue
      end
      RS_var = copy(RS); % Kopie des Roboters zum Eintragen der Variante
      % Verbliebene Parameter zufällig belegen (nur für Test)
      pkin(isnan(pkin)) = rand(sum(isnan(pkin)),1);
      RS.update_mdh(pkin);
      J = RS.jacobig(rand(RS.NQJ,1));
      EE_dof0 = '000000';
      EE_dof0(abs(J*rand(RS.NQJ,1))>1e-9) = '1';
      
      % Prüfe den Rang der Jacobi-Matrix: Durch Entfernen der a-/d-Parameter
      % kann es sein, dass die serielle Kette nicht mehr Rang 6 hat
      if rank(J) < N
        fprintf('\tVariante hat einen Rangverlust durch die Kinematikvereinfachung. Überspringe.\n');
        num_decline_rank = num_decline_rank + 1;
        continue
      end
      % Parameter für Variante generieren
      MDH_struct_idx = serroblib_mdh_numparam2indexstruct(RS_var.MDH);

      % Neue Variante in Datenbank eintragen
      fprintf('\tVariante mit mehrwertigen Gelenken erzeugt (%s). Füge zu DB hinzu.\n', joint_string);
      serroblib_gen_bitarrays(N);
      fprintf('\t'); % Damit Ausgabe von add_robot eingerückt ist
      [Name_j,new_j] = serroblib_add_robot(MDH_struct_idx, char(EE_dof0));
      num_success = num_success + 1;
      num_new = num_new + new_j;
      List_NewMultiDoFChains = {List_NewMultiDoFChains{:}, Name_j}; %#ok<CCAT>
      % Probe, ob neue Kinematik sinnvoll ist.
      if ~contains(Name_j, 'V')
        error('Neu erzeugter Roboter ist keine Variante. Das ergibt keinen Sinn!');
      end
    end
  end
end
fprintf(['Insgesamt %d Varianten durch den Ansatz erzeugt (davon %d neu). ', ...
  'Bei %d/%d passte die Gelenkstruktur nicht. Bei den verbliebenen fielen %d wegen Rangverlust heraus\n'], ...
  num_success, num_new, num_decline_revolute_end, length(I_novar), num_decline_rank);
fprintf('Folgende PKM-Beinketten wurden generiert:\n');
disp(List_NewMultiDoFChains);

%% Abspeichern der Ergebnisliste in der Roboterdatenbank
if save_reslist
roblibpath=fileparts(which('serroblib_path_init.m'));
robot_list_dir = fullfile(roblibpath, 'synthesis_result_lists');
roblist = fullfile(robot_list_dir, ['multi_dof_joints', '.txt']);
writecell(unique(List_NewMultiDoFChains(:)), roblist);
fprintf('Ergebnisliste in %s gespeichert\n', roblist);
end

%% Nachträgliche Prüfung der Datenbank
% Sind alle Mehr-FG-Gelenk-Varianten, die hier erzeugt wurden auch keine
% Hauptmodelle in der Datenbank?
serroblib_gen_bitarrays(1:7);
for N = 4:6
  fprintf('Prüfe Strukturen mit %d FG\n', N);
  % Alle Roboter aus Datenbank laden
  mdllistfile_Ndof = fullfile(roblibpath, sprintf('mdl_%ddof', N), sprintf('S%d_list.mat',N));
  l = load(mdllistfile_Ndof, 'Names_Ndof', 'AdditionalInfo', 'BitArrays_Origin');
  Mask_Origin = uint16(bin2dec('10000')); % Probe: dec2bin(Mask_Origin,16)
  I_multidofj = bitand(l.BitArrays_Origin, repmat(Mask_Origin, length(l.Names_Ndof),1))~=0;
  I_novar = (l.AdditionalInfo(:,2)==false); % Auswahl aller Roboter, die keine Variante sind
  I_check = I_multidofj & I_novar;
  if any(I_check)
    warning(['Es sind Hauptmodelle in der Datenbank eingetragen, die aus ', ...
      'dieser Struktursynthese stammen sollen. Es sollten aber nur Varianten sein']);
    disp(l.Names_Ndof(I_check));
  end
end
