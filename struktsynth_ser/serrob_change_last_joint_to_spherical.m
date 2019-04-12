% Ändere die Kinematikparameter aller Robotermodelle so, dass das letzte
% Gelenk ein Kugelgelenk wird.
% Generiere dadurch Varianten, bei denen als PKM-Beinkette die translatorischen
% Zwangsbedingungen ausreichen. Diese eignen sich gut für die
% Dynamik-Modellierung

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-04
% (C) Institut für Mechatronische Systeme, Universität Hannover

clear
clc

%% Initialisierung
roblibpath=fileparts(which('serroblib_path_init.m'));

serroblib_gen_bitarrays(1:7);

%% Durchsuche alle Roboter und stelle die korrekte Orientierung des Endeffektors fest
for N = 6
  fprintf('Prüfe Strukturen mit %d FG\n', N);
  % Alle Roboter aus Datenbank laden
  mdllistfile_Ndof = fullfile(roblibpath, sprintf('mdl_%ddof', N), sprintf('S%d_list.mat',N));
  l = load(mdllistfile_Ndof, 'Names_Ndof', 'AdditionalInfo');
  I_novar = find(l.AdditionalInfo(:,2)==false); % Auswahl aller Roboter, die keine Variante sind
  i = 0;
  for j = I_novar'
    i = i+1;
    RobName = l.Names_Ndof{j};
    fprintf('%d/%d: Prüfe Struktur %s (%d/%d gesamt)\n', i, length(I_novar), ...
      RobName, j, length(l.Names_Ndof));
    % Roboterklasse erstellen
    RS = serroblib_create_robot_class(RobName);
    
    % Kinematikparameter so ändern, dass das letzte Gelenk ein Kugelgelenk
    % ist
    if any(RS.MDH.sigma(4:6) ~= 0)
      % Die letzten drei müssen Drehgelenke sein
      continue
    end
    % Eigenschaft des Kugelgelenks: Die Achsen aller Drehgelenke schneiden
    % sich, Drehachsen sollten senkrecht sein (müssen aber nicht).
    RS.MDH.d(5:6) = 0;
    RS.MDH.a(5:6) = 0;
    if any(isnan(RS.MDH.alpha(5:6)))
      error('Die Struktur hat alpha5/alpha6 frei wählbar. Widerspricht Kugelgelenk');
    end
    pkin = RS.update_pkin();
    RS_var = copy(RS);
    % Verbliebene Parameter zufällig belegen
    pkin(isnan(pkin)) = rand(sum(isnan(pkin)),1);
    RS.update_mdh(pkin);
    J = RS.jacobig(rand(RS.NQJ,1));
    EE_dof0 = (J*rand(RS.NQJ,1))' ~= 0;
    
    % Parameter für Variante generieren
    MDH_struct_idx = serroblib_mdh_numparam2indexstruct(RS_var.MDH);
    
    % Neue Variante in Datenbank eintragen
    fprintf('\tVariante mit Kugelgelenk am Ende erzeugt. Füge zu DB hinzu.\n');
    
    serroblib_gen_bitarrays(N);
    serroblib_add_robot(MDH_struct_idx, EE_dof0);
  end
end

%% Beispiel für UPS-Kette erzeugen
% Zur Definition siehe ParRob_class_example_6UPS.m
RS = serroblib_create_robot_class('S6RRPRRR14');
RS.update_mdh(zeros(length(RS.pkin),1))
pkin = RS.pkin;
pkin(strcmp(RS.pkin_names,'alpha2')) = pi/2;
pkin(strcmp(RS.pkin_names,'alpha3')) = pi/2;
RS.update_mdh(pkin);
MDH_struct_idx = serroblib_mdh_numparam2indexstruct(RS.MDH);
serroblib_gen_bitarrays(6);
[mdlname, new] = serroblib_add_robot(MDH_struct_idx, [1 1 1 1 1 1]);

% Code für kinematische Kette erstellen
% serroblib_generate_mapleinput({mdlname})
% serroblib_generate_code({mdlname}, true)





