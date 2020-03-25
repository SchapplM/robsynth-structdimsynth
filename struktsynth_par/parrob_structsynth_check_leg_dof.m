% Testen die Rotationsachse der Beinkette und des Platform
% vor Ma�synhese
% TODO: 1. noch mehr Situationen testen
%       2. 3T2R Parrob inplementieren
%       3. Funktioniert nicht für 3T3R!
% 
% Eingabe:
% SName
%   Name der seriellen Beinketten, aus denen der symmetrische Roboter
%   besteht
% Coupling
%   Koppelpunkt-Nummern
% EE_dof0
%   EE-FG (1x6 Vektor mit 0 und 1)
% 
% Ausgabe:
% leg_success
%   true, wenn die Beinkette prinzipiell geeignet ist
%   false, wenn sie für die geforderte PKM keinen Sinn ergibt

% Junnan Li, Hiwi bei Moritz Schappler, 2020-03
% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-03
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function leg_success = parrob_structsynth_check_leg_dof(SName, Coupling, EE_dof0, EE_dof_legchain)

leg_success = true;

if all(EE_dof0 == [1 1 0 0 0 1])
  return % keine Einschränkungen für 2T1R implementiert
end
if all(EE_dof0 == [1 1 1 1 1 1])
  return % keine Einschränkungen für 3T3R
end

Basis_Coupling = Coupling(1);
Koppel_Coupling = Coupling(2);

% RS = serroblib_create_robot_class(SName); % Aufruf braucht viel Zeit
LEG_Dof = EE_dof_legchain; %RS.I_EE;

if sum(LEG_Dof) == 6
  leg_success = true; % Beinkette haben vollständige FG
  return
elseif sum(LEG_Dof(4:6)) == 1
  % Beinkette haben reduzierte FG und nur eine Rotation FG
  if sum(EE_dof0(4:6)) == 1 && Basis_Coupling == 4  
  leg_success = false; % Drehachse der Beinkette sind nicht kegelförmig
  return 
  end
end