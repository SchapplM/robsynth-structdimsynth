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
% Initialisierung. Dadurch Übernahme der Felder 't' und 'IE'.
Traj_0 = Traj_W;
% Zwei Eingabestrukturen erstellen (Trajektorie und Eckpunkte)
Traj_W_X = struct('X', Traj_W.X, 'XD', Traj_W.XD, 'XDD', Traj_W.XDD);
Traj_W_XE = struct('X', Traj_W.XE);
% Transformation über Roboter-Klasse
Traj_0_X = R.transform_traj(Traj_W_X);
Traj_0_XE = R.transform_traj(Traj_W_XE);
% Belegung der Ausgabe-Struktur
Traj_0.XE = Traj_0_XE.X;
Traj_0.X = Traj_0_X.X;
Traj_0.XD = Traj_0_X.XD;
Traj_0.XDD = Traj_0_X.XDD;
