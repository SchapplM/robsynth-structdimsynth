% Transformiere die EE-Trajektorie ins Basis-KS des Roboters
% Die Roboter-Funktionen sind alle mit X in Basis-Koordinaten definiert.
% 
% Eingabe:
% R
%   Roboter-Klasse. Enthält Basis-Transformation des Roboters (T_W_0) sowie
%   die vorgegebene Euler-Winkel-Konvention zur Orientierungsdarstellung.
% Traj_W
%   Roboter-Trajektorie (EE) bezogen auf Welt-KS. Felder:
%   XE: N Eckpunkte im Arbeitsraum (Nx6; jew. 1x3 Position und 1x3 Euler-Winkel)
%   X,XD,XDD: Trajektorie aus N Zeitschritten (siehe XE)
%   t: Zeitschritte
%   IE: Indizes der Eckpunkte XE in der Zeit-Trajektorie X
% 
% Ausgabe:
% Traj_0
%   Roboter-Trajektorie (EE) bezogen auf Basis-KS des Roboters.
%   Felder siehe Eingabe
% 
% Siehe auch: RobBase/transform_traj.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function Traj_0 = cds_transform_traj(R, Traj_W)
% Kopie: Dadurch Übernahme der Felder 't', 'IE' und 'nullspace_maxvel_interp'
Traj_0 = Traj_W;
% Eckpunkt-Posen ins Basis-KS transformieren (für cds_constraints)
Traj_W_XE = struct('X', Traj_W.XE);
Traj_0_XE = R.transform_traj(Traj_W_XE);
Traj_0.XE = Traj_0_XE.X;
if ~isfield(Traj_W, 'X')
  return
end
% Vollständige Zeit-Trajektorie transformieren (für cds_constraints_traj)
% Transformation über Roboter-Klasse
Traj_W_X = struct('X', Traj_W.X, 'XD', Traj_W.XD, 'XDD', Traj_W.XDD);
Traj_0_X = R.transform_traj(Traj_W_X);
% Belegung der Ausgabe-Struktur (Felder aus Eingabe behalten)
Traj_0.X = Traj_0_X.X;
Traj_0.XD = Traj_0_X.XD;
Traj_0.XDD = Traj_0_X.XDD;
% Externe Kraft rotieren
if isfield(Traj_0, 'Fext') && any(Traj_W.Fext(:))
  for i = 1:length(Traj_0.t)
    Traj_0.Fext(i,:) = rotate_wrench(Traj_W.Fext(i,:)', R.T_W_0(1:3,1:3)');
  end
end