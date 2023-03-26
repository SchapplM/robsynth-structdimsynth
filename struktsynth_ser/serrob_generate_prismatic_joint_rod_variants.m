% Ändere die Kinematikparameter aller Robotermodelle so, dass das Schub-
% gelenke eine Stabkinematik darstellen ohne Hebel auf darum liegende Gelenke
% 
% Notwendige Schritte nach dem Erzeugen dieser Varianten (wird am Ende gemacht):
% * generate_variant_pkin_conv_fcns.m
% * correct_phi_N_E;
% * determine_ee_dof;
% * determine_jointnumber_influence_ee_position;
% * determine_multi_dof_joints.m
% * write_structsynth_origin.m
% 
% Siehe auch: serrob_generate_multi_dof_variants.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-03
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc

%% Initialisierung
roblibpath=fileparts(which('serroblib_path_init.m'));
serroblib_gen_bitarrays(1:7);
%% Benutzereingaben
save_reslist = true; % Nur aktivieren, wenn alle Varianten gesucht werden
serialchain_test = ''; % S6RRRRRR10% Prüfe nur dieses Hauptmodell
%% Durchsuche alle Roboter und stelle die korrekte Orientierung des Endeffektors fest
num_success = 0;
num_decline_rank = 0;
num_decline_revolute_end = 0;
num_new = 0;
List_NewChains = {};
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
  Mask_prismaticvar = uint16(bin2dec('100000')); % Probe: dec2bin(Mask_Origin,16)
  I_this = bitand(l.BitArrays_Origin, repmat(Mask_prismaticvar, length(l.Names_Ndof),1))~=0;
  fprintf('Basierend auf diesem Skript gibt es %d Ketten mit %d FG\n', sum(I_this), N);
%   disp(l.Names_Ndof{I_this});
  I_novar = (l.AdditionalInfo(:,2)==false); % Auswahl aller Roboter, die keine Variante sind
  I_var = (l.AdditionalInfo(:,2)==true); % Auswahl aller Roboter, die Variante sind
  i = 0;
  I_check = (I_var | I_novar) & ~I_this; % Prüfe alle eingetragenen seriellen Ketten (aber keine, die schon mit diesem Skript erzeugt wurden)
  for j = find(I_check)'
    i = i+1;
    RobName = l.Names_Ndof{j};
    num_prismatic = sum(RobName == 'P');
    if num_prismatic ~= 1
      continue % nicht relevant (mehrere Schubgelenke sowieso nicht betrachtet)
    end
    fprintf('%d/%d: Prüfe Struktur %s (%d/%d gesamt)\n', i, sum(I_check), ...
      RobName, j, length(l.Names_Ndof));
    if ~isempty(serialchain_test) && ~strcmp(RobName, serialchain_test)
      continue % Filterung zu Testzwecken
    end
    % Roboterklasse erstellen (muss in der Schleife passieren, damit mit
    % NaN belegte freie Parameter wieder zurückgesetzt werden
    try
      RS = serroblib_create_robot_class(RobName);
    catch
      fprintf(['Keine Initialisierung der Roboter-Klasse möglich. ', ...
        'Vermutlich noch nicht initialisierte Varianten aus diesem Skript\n']);
      continue
    end
    % Setze die a- und d-Parameter vor und nach Schubgelenken auf Null
    % Siehe auch: cds_dimsynth_robot, cds_gen_robot_list
    I_cyl = find(RS.MDH.sigma == 1 & [0;ones(N-1,1)]);
    RS.gen_testsettings(true, true);
    pkin_orig = RS.pkin;
    pkin_NaN = NaN(length(RS.pkin),1); % Platzhalter-Variable für freie Parameter
    if isempty(I_cyl)
      continue % Kette hat kein passendes Schubgelenk
    end
    for ii_cyl = I_cyl' % Alle Gelenke mit Schubzylinder durchgehen (falls mehrere)
      I_aprecyl = RS.pkin_jointnumber==ii_cyl & RS.pkin_types == 4;
      % Setze den d- und a-Parameter nach einem Schubylinder zu Null. Ist technisch
      % sinnvoller. Dadurch drückt der Zylinder direkt auf das folgende Gelenk
      I_apostcyl = RS.pkin_jointnumber==(ii_cyl+1) & RS.pkin_types == 4;
      I_dpostcyl = RS.pkin_jointnumber==(ii_cyl+1) & RS.pkin_types == 6;
      pkin_NaN(I_aprecyl|I_apostcyl|I_dpostcyl) = 0;
    end
    RS.pkin(pkin_NaN==0) = 0; % Parameter in Klasse Null setzen
    RS.update_mdh(RS.pkin); % Aktualisierte DH-Parameter eintragen
    if all((pkin_orig==0) == (RS.pkin==0))
      % Der neue Roboter ist identisch mit dem alten. Es muss nichts
      % eingetragen werden
      continue
    end
    % Prüfe den Rang der Jacobi-Matrix: Durch Entfernen der a-/d-Parameter
    % kann es sein, dass die serielle Kette nicht mehr Rang 6 hat
    J = RS.jacobig(rand(RS.NQJ,1));
    EE_dof0 = '000000';
    EE_dof0(abs(J*rand(RS.NQJ,1))>1e-9) = '1';
    if rank(J) < N
      fprintf('\tVariante hat einen Rangverlust durch die Kinematikvereinfachung. Überspringe.\n');
      num_decline_rank = num_decline_rank + 1;
      continue
    end
    % Füge die Variante zur Datenbank hinzu
    RS_var = copy(RS); % Kopie des Roboters zum Eintragen der Variante
    RS_var.update_mdh(pkin_NaN); % NaN für freie Parameter, Null für entfernte
    MDH_struct_idx = serroblib_mdh_numparam2indexstruct(RS_var.MDH);
    fprintf(['\tVariante mit hebelfreiem Schubgelenk erzeugt. ', ...
      'Füge zu DB hinzu.\n']);
    serroblib_gen_bitarrays(N);
    fprintf('\t'); % Damit Ausgabe von add_robot eingerückt ist
    [Name_j,new_j] = serroblib_add_robot(MDH_struct_idx, char(EE_dof0));
    num_success = num_success + 1;
    num_new = num_new + new_j;
    List_NewChains = {List_NewChains{:}, Name_j}; %#ok<CCAT>
    % Probe, ob neue Kinematik sinnvoll ist.
    if ~contains(Name_j, 'V')
      error('Neu erzeugter Roboter ist keine Variante. Das ergibt keinen Sinn!');
    end
  end
end
fprintf(['Insgesamt %d Varianten durch den Ansatz erzeugt (davon %d neu). ', ...
  'Bei den verbliebenen fielen %d wegen Rangverlust heraus\n'], ...
  num_success, num_new, num_decline_rank);
fprintf('Folgende PKM-Beinketten wurden generiert:\n');
disp(List_NewChains);

%% Abspeichern der Ergebnisliste in der Roboterdatenbank
if save_reslist
  roblibpath=fileparts(which('serroblib_path_init.m'));
  robot_list_dir = fullfile(roblibpath, 'synthesis_result_lists');
  roblist = fullfile(robot_list_dir, ['prismatic_rod_chains', '.txt']);
  writecell(unique(List_NewChains(:)), roblist);
  fprintf('Ergebnisliste in %s gespeichert\n', roblist);
end

return
%% Aktualisieren der Datenbank
addpath(fullfile(roblibpath, 'scripts'));
generate_variant_pkin_conv_fcns;
correct_phi_N_E;
determine_ee_dof;
determine_jointnumber_influence_ee_position;
determine_multi_dof_joints;
write_structsynth_origin;
