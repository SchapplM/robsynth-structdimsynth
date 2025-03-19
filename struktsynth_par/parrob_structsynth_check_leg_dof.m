% Teste die PKM-Freiheitsgrade aufgrund von Plausibilitätsüberlegungen
% Berücksichtigt Beinketten-FG und Koppelgelenk-Anordnung und einfache
% geometrische Überlegungen ohne Berechnung von Jacobi-Matrizen o.ä.
% 
% Eingabe:
% SName
%   Name der seriellen Beinketten, aus denen der symmetrische Roboter
%   besteht
% Coupling
%   Koppelpunkt-Nummern
% EE_dof0
%   EE-FG (1x6 Vektor mit 0 und 1)
% SerRob_List
%   Liste serieller Roboter aus Seriell-Roboter-Datenbank
%   (siehe serroblib_gen_bitarrays.m; entspricht bspw. Datei S5_list.mat)
% 
% Ausgabe:
% leg_success
%   true, wenn die Beinkette prinzipiell geeignet ist
%   false, wenn sie für die geforderte PKM keinen Sinn ergibt

% Junnan Li, Hiwi bei Moritz Schappler, 2020-03
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-03
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function leg_success = parrob_structsynth_check_leg_dof(SName, Coupling, EE_dof0, EE_dof_legchain, SerRob_List)

leg_success = true;
NLegjoint = str2double(SName(2));
if all(EE_dof0 == [1 1 0 0 0 1])
  return % keine Einschränkungen für 2T1R implementiert
end
if all(EE_dof0 == [1 1 1 1 1 1])
  return % keine Einschränkungen für 3T3R
end

Base_Coupling = Coupling(1);
Platform_Coupling = Coupling(2);

LEG_Dof = EE_dof_legchain; %RS.I_EE;

if Platform_Coupling == 7 && all(EE_dof0==[1 1 1 0 0 0]) % (NLegjoint ~= 4 || 
  % P7 bisher nur für 3T0R-PKM implementiert mit 4 Gelenken pro Beinkette
  % Geht aber auch für fünf Gelenke. Davon abhängig machen ob alle Gelenke
  % bis auf eins parallel sind
  I = strcmp(SerRob_List.Names_Ndof, SName);
  csv_Rob = serroblib_bits2csvline(SerRob_List.BitArrays_Ndof(I,:));
  csv_Rob = csv_Rob(1:1+8*NLegjoint); % Nur Zeilen für Gelenke nehmen (falls mit Nullen rechts aufgefüllt wurde)
  joint_is_revolute = strcmp(csv_Rob(2:8:1+8*NLegjoint), 'R');
  alpha_is_0 = strcmp(csv_Rob(5:8:end), '0');
  alpha_is_90 = strcmp(csv_Rob(5:8:end), 'pi/2');
  if all(alpha_is_0&joint_is_revolute | ~joint_is_revolute)
    % Alle Gelenke sind parallel (wie bspw. PRRR). Für diesen Fall war die
    % Methode (G4P7) ursprünglich gedacht.
    % Die Parallelität muss nur für Drehgelenke gelten
    return
  end
  if sum(alpha_is_90) == 2 && diff(find(alpha_is_90)) == 1 && ...
      joint_is_revolute(find(alpha_is_90,1,'first'))
    % In der Kette gibt es zwei aufeinanderfolgende Drehgelenke, die 90°
    % verdreht sind. Damit müssen alle anderen Gelenke parallel sein, bis
    % auf das erste Gelenk (muss Drehgelenk sein), bei dem alpha=90° ist.
    % Bei 5FG-Beinketten entsteht so der Fall eines unbewegten Drehglenks.
    return
  end
  % Es kann auch die vorherige Bedingung gelten, wenn dazwischen ein
  % Schubgelenk ist.
  I_alphaPis90 = find(alpha_is_90 & ~joint_is_revolute);
  alpha_is_90_corr = alpha_is_90;
  % Verschiebe alpha=90 zum Drehgelenk
  alpha_is_90_corr(I_alphaPis90+1) = true;
  alpha_is_90_corr(I_alphaPis90) = false;
  if sum(alpha_is_90_corr) == 2 && diff(find(alpha_is_90_corr)) == 1 && ...
      joint_is_revolute(find(alpha_is_90_corr,1,'first'))
    % Siehe oben
    return
  end

  if sum(alpha_is_90) == 1 && find(alpha_is_90,1,'first')==2
    % Das erste Gelenk ist anders gedreht als alle anderen.
    return
  end
  if sum(alpha_is_90) == 1 && find(alpha_is_90,1,'first')==NLegjoint
    % Das letzte Gelenk ist anders gedreht als alle anderen.
    return
  end
  % Keiner der oben geprüften Fälle. Es ist also mehr als ein Gelenk nicht
  % Teil der ansonsten parallelen Gelenke.
  leg_success = false;
  return
end

if sum(LEG_Dof(1:6)) == 6
  leg_success = true; % Beinkette haben vollständige FG
elseif (Base_Coupling == 4) || (Base_Coupling ~= 1 && Platform_Coupling == 1)
  % G23P1 und G4P123
  % TODO: Das ist zumindest für P3PRRRR3V1G4P2A1 falsch.
  % leg_success = false;
elseif (Base_Coupling ~= Platform_Coupling) && (LEG_Dof(4) == 0)
  % G2P3 und G3P2 brauchen x-Achse Rotationsfreiheit
  leg_success = false;
else
  leg_success = true;
end

  
