% Auswahl einer Trajektorie für die Roboteroptimierung

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Traj = cds_gen_traj(DoF, no, trajset)
x0 = [0.5, 0.5, 0, 0, 0, 0]';
k=1; XE = x0';

%% Liste der Trajektorien: 2T1R
if all(DoF == [1 1 0 0 0 1])
  if no == 1
    d1=0.3;
    k=k+1; XE(k,:) = XE(k-1,:) + [ d1,0,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,-d1,0  0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [-d1,0,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,d1,0  0,0,0];
  elseif no == 2
    d1=0.3;
    phimax = pi/2;
    k=k+1; XE(k,:) = XE(k-1,:) + [ d1,0,0, 0,0,phimax];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,-d1,0  0,0,-phimax];
    k=k+1; XE(k,:) = XE(k-1,:) + [-d1,0,0, 0,0,-phimax];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,d1,0  0,0, phimax];
  end
end
%% Liste der Trajektorien: 3T0R
if all(DoF == [1 1 1 0 0 0])
  % Beginn Würfel
  d1=0.3;
  h1=0.3;
  k=k+1; XE(k,:) = XE(k-1,:) + [ d1,0,0, 0,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [0,-d1,0  0,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [-d1,0,0, 0,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [0,0,-h1, 0,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [0,0, h1, 0,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [0,d1,0,  0,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [0,0,-h1, 0,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [ d1,0,0, 0,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [0,-d1,0  0,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [0,0, h1, 0,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [0,0,-h1, 0,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [-d1,0,0, 0,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [0,d1,0,  0,0,0];
  k=k+1; XE(k,:) = XE(k-1,:) + [0,0, h1, 0,0,0];
end
%% Trajektorie generieren
if trajset.profile == 1
  [X_ges,XD_ges,XDD_ges,T_ges] = traj_trapez2_multipoint(XE, ...
    trajset.vmax, trajset.vmax/trajset.amax, trajset.Tv, trajset.Ts, 0); % muss noch bearbeiten
elseif trajset.profile == 0 % Nur Eckpunkte
  X_ges = XE;
  XD_ges = XE*0;
  XDD_ges = XE*0;
  T_ges = 1:size(XE,1);
end

%% Ausgabe
Traj = struct('X', X_ges, 'XD', XD_ges, 'XDD', XDD_ges, 't', T_ges, 'XE', XE);