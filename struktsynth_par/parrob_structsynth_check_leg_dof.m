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

if Platform_Coupling == 7 && (NLegjoint ~= 4 || ~all(EE_dof0==[1 1 1 0 0 0]))
  % P7 bisher nur für 3T0R-PKM implementiert mit 4 Gelenken pro Beinkette
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

  
