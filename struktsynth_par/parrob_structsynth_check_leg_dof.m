% Testen die Rotationsachse der Beinkette und des Platform
% vor Maßsynhese
% TODO: 1. noch mehr Situationen testen
%       2. 3T2R Parrob inplementieren


% Junnan Li, Hiwi bei Moritz Schappler, 2020-01
% (C) Institut für Mechatronische Systeme, Universität Hannover






function status = parrob_structsynth_check_leg_dof(Name)

status = true;

[~,LEG_Names,~,Coupling,~,~, EE_dof0]=parroblib_load_robot(Name);

Basis_Coupling = Coupling(1);
Koppel_Coupling = Coupling(2);

RS = serroblib_create_robot_class(LEG_Names{1});
LEG_Dof = RS.I_EE;

if sum(LEG_Dof) == 6
  status = true; % Beinkette haben vollständige FG
  return
elseif sum(LEG_Dof(4:6)) == 1
  % Beinkette haben reduzierte FG und nur eine Rotation FG
  if sum(EE_dof0(4:6)) == 1 && Basis_Coupling == 4  
  status = false; % Drehachse der Beinkette sind nicht kegelförmig
  return 
  end
end