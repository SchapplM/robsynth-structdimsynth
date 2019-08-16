% Rotiere die EE-Trajektorie ins Basis-KS des Roboters
% Die Roboter-Funktionen sind alle mit X in Basis-Koordinaten definiert.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Traj_0 = cds_rotate_traj(Traj_W, T_W_0)
Traj_0 = Traj_W;

% Trajektorie verschieben (nur Position. Geschwindigkeit egal)
XE_W = Traj_W.XE;
X_W = Traj_W.X;
% Rechnung: r_0_P = r_W_P - r_W_0; (falls R_W_0 = 1)
X_0 = X_W -   repmat([T_W_0(1:3,4); zeros(3,1)]', size(X_W,1),1);
XE_0 = XE_W - repmat([T_W_0(1:3,4); zeros(3,1)]', size(XE_W,1),1);
% TODO: Bei Rotation der Basis muss die Trajektorie auch gedreht werden
test = T_W_0(1:3,1:3)-eye(3);
if max(abs(test(:))) > 1e-10
  error('Rotation der Basis nicht implementiert');
end

Traj_0.X = X_0;
Traj_0.XE = XE_0;