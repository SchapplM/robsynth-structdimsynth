% Auswahl einer Trajektorie für die Roboteroptimierung
% 
% Eingabe:
% DoF [1x6]
%   Kennung der EE-FG (bezogen auf EE-Geschwindigkeit)
% no
%   Nummer der hinterlegten Beispiel-Trajektorie (für den FG)
% trajset
%   Einstellungen für Trajektorie
%   Siehe cds_settings_defaults.m
% 
% Ausgabe:
% Traj
%   Struktur mit Trajektorie des Endeffektors (Zeit, Position, Geschw.)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function Traj = cds_gen_traj(DoF, no, trajset)
x0 = [0.5, 0.5, 0, 0, 0, 0]';
k=1; XE = x0';

%% Liste der Trajektorien: 2T1R (oder 2T0R)
if all(DoF == [1 1 0 0 0 1]) || all(DoF == [1 1 0 0 0 0])
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
  elseif no == 4 % jeden FG einmal vor, zurück und in die Mitte bewegen
    d1=0.1;
    phimax = 15*pi/180;
    k=k+1; XE(k,:) = XE(k-1,:) + [   d1,0,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [-2*d1,0,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [   d1,0,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0,   d1,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0,-2*d1,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0,   d1,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0,0,0, 0,0,   phimax];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0,0,0, 0,0,-2*phimax];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0,0,0, 0,0,   phimax];
  else
    error('Trajektorie Nr. %d nicht für 2T1R definiert', no);
  end
  if all(DoF == [1 1 0 0 0 0])
    XE(:,6) = 0;
    XE = unique(XE, 'rows');
  end
end
%% Liste der Trajektorien: 3T0R
if all(DoF == [1 1 1 0 0 0])
  d1=0.3;
  h1=0.3;
  if no == 1 % Würfel
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
    % Fahrt in die Mitte (nur Eckpunkte reicht nicht für Struktursynthese.
    % Dann können Umklapp-Lagen bereits die Punkte erfüllen).
    k=k+1; XE(k,:) = XE(k-1,:) + [d1/2,-d1/2, -h1/2, 0,0,0];
  elseif no == 4 % jeden FG einmal vor, zurück und in die Mitte bewegen
    k=k+1; XE(k,:) = XE(k-1,:) + [ 1*d1,0,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [-2*d1,0,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 1*d1,0,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 1*d1,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0,-2*d1,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 1*d1,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 0, 1*h1, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 0,-2*h1, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 0, 1*h1, 0,0,0];
  else
    error('Trajektorie Nr. %d nicht für 3T0R definiert', no);
  end
end

%% Liste der Trajektorien: 3T1R
if all(DoF == [1 1 1 0 0 1])
  d1=0.3;
  h1=0.3;
  if no == 1 % Beginn Würfel mit zusätzlicher EE-Drehung
    k=k+1; XE(k,:) = XE(k-1,:) + [ d1,0,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,-d1,0  0,0, pi/4];
    k=k+1; XE(k,:) = XE(k-1,:) + [-d1,0,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0,-h1, 0,0, pi/4];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0, h1, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,d1,0,  0,0,-pi/4];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0,-h1, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ d1,0,0, 0,0,-pi/4];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,-d1,0  0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0, h1, 0,0,-pi/3];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0,-h1, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [-d1,0,0, 0,0,pi/2];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,d1,0,  0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0, h1, 0,0,-pi/6];
    % Fahrt in die Mitte.
    k=k+1; XE(k,:) = XE(k-1,:) + [d1/2,-d1/2, -h1/2, 0,0,0];
  elseif no == 4 % jeden FG einmal vor, zurück und in die Mitte bewegen
    phimax = 15*pi/180;
    k=k+1; XE(k,:) = XE(k-1,:) + [ 1*d1,0,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [-2*d1,0,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 1*d1,0,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 1*d1,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0,-2*d1,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 1*d1,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 0, 1*h1, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 0,-2*h1, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 0, 1*h1, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 0, 0, 0,0, 1*phimax];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 0, 0, 0,0,-2*phimax];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 0, 0, 0,0, 1*phimax];
  else
    error('Trajektorie Nr. %d nicht für 3T1R definiert', no);
  end  
end

%% Liste der Trajektorien: 3T3R und 3T2R
if all(DoF == [1 1 1 1 1 1]) || all(DoF == [1 1 1 1 1 0])
  d1=0.3;
  h1=0.3;
  if no == 1 || no == 2 || no == 3
    % Beginn Würfel mit zusätzlicher 3D-EE-Drehung
    % obere Ebene
    k=k+1; XE(k,:) = XE(k-1,:) + [ d1,0,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,-d1,0  0,0, pi/4];
    k=k+1; XE(k,:) = XE(k-1,:) + [-d1,0,0, 0,0,-pi/4];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0,-h1, pi/4,0, 0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0, h1, -pi/4,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,d1,0,  0,pi/4,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0,-h1, 0,-pi/4,0];
    % untere Ebene: Fahre anders herum, damit Drehgelenke nicht mehrfache
    % Umdrehungen machen müssen
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0,-d1,0, pi/6,-pi/6,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [d1,0,0  -pi/6,pi/6,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0, h1, -pi/6,pi/6,pi/6];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0,-h1, -pi/6,pi/4,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,d1,0, pi/12,-pi/6,pi/2];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0, h1, pi/6,-pi/4,pi/2];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0,-h1, pi/12,-pi/6,pi/2];
    k=k+1; XE(k,:) = XE(k-1,:) + [-d1,0,0,  -pi/4,0,-pi/6];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0, h1, -pi/12,pi/12,pi/3];
    if no == 2 % Modifikation
      % Nachbearbeiten der Würfel-Trajektorie: Position bleibt so wie sie
      % ist, aber die Orientierung wird mit jeder Würfel-Kante weiter
      % ausgefahren.
      XE(:,4:6) = 0;
      angle_steps = 2 * trajset.maxangle / length(XE);
      for i = 2:size(XE,1)
        if mod(i,2) == 0
          dphi = [angle_steps;0;0];
        else
          dphi = [0;angle_steps;0];
        end
        XE(i,4:6) = XE(i-1,4:6) + dphi';
      end
    elseif no == 3 % Modifikation
      % Der Würfel als Form bleibt bestehen, aber die Orientierung ist nie
      % parallel zur Basis. Der Endeffektor zeigt immer auf den gleichen
      % Punkt unterhalb des Würfels
      % Diese Orientierung soll günstiger für 3T2R-PKM sein, die oft eine
      % Singularität bei Parallelstellung haben.
      task_dim = (max(XE(:,1:3)) - min(XE(:,1:3)));
      task_mid = min(XE(:,1:3)) + task_dim/2;
      refpt = [task_mid(1:2)';task_mid(3)-0.7]; % Punkt unterhalb -> Schwenkwinkel kleiner
      
      for i = 1:size(XE,1)
        % Gerade vom Referenzpunkt zum aktuellen Würfelpunkt
        vec_i = XE(i,1:3)'-refpt;
        % Es wäre intuitiver, wenn die z-Achse nach unten zeigt (auf die
        % Aufgabe). Die Roboter sind aber so definiert, dass standardmäßig
        % die Roboter-z-Achse genau wie die Aufgaben-z-Achse nach oben
        % zeigen
        z_i = vec_i/norm(vec_i);
        % Nehme x-Achse identisch zur Basis. Annahme: Das ist für 3T3R am
        % günstigsten, für 3T2R sowieso egal
        x_i = -cross(z_i, [0;1;0]);
        x_i = x_i / norm(x_i);
        y_i = -cross(x_i, z_i); % Rechtshändiges KS
        y_i = y_i / norm(y_i);
        if abs(det([x_i, y_i, z_i]) - 1) > 1e-10, error('det(R) stimmt nicht'); end
        XE(i,4:6) = r2eulxyz([x_i, y_i, z_i]);
      end
    end
  elseif no == 4
    phimax = 15*pi/180;
    k=k+1; XE(k,:) = XE(k-1,:) + [ 1*d1,0,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [-2*d1,0,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 1*d1,0,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 1*d1,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0,-2*d1,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 1*d1,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 0, 1*h1, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 0,-2*h1, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 0, 1*h1, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 0, 0, 1*phimax, 0, 0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 0, 0,-2*phimax, 0, 0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 0, 0, 1*phimax, 0, 0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 0, 0, 0, 1*phimax, 0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 0, 0, 0,-2*phimax, 0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 0, 0, 0, 1*phimax, 0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 0, 0, 0, 0, 1*phimax];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 0, 0, 0, 0,-2*phimax];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0, 0, 0, 0, 0, 1*phimax];
  else
    error('Trajektorie Nummer %d nicht definiert', no);
  end
  % 3T2R: Entferne die letzte Drehung. Der EE soll nur auf die Kanten des
  % Würfels zeigen
  if all(DoF == [1 1 1 1 1 0])
    XE(:,6) = 0;
  end
end
%% Skaliere die Winkel der Trajektorie herunter
% Relevant, wenn die Dreh- und Schwenkwinkel des Roboters nicht wichtig
% sind (z.B. bei der FG-Prüfung in der Struktursynthese).
maxangle_traj = max(max(abs(XE(:,4:6))));
if maxangle_traj > trajset.maxangle
  % Skaliere so herunter, dass Maximalwinkel gerade erreicht wird.
  % Betrachte nur die einzelnen Euler-Komponenten. Ignoriere Kopplung
  XE(:,4:6) = trajset.maxangle * XE(:,4:6) / maxangle_traj;
end
%% Anpassung für Wandmontage
if trajset.wall_rotate_traj % Drehe die Aufgabe um 90°
  XE_old = XE;
  for i = 1:size(XE,1)
    R_W_0 = rotx(-pi/2);
    % Trajektorie ursprünglich bzgl KS 0
    XE(i,1:3) = R_W_0 * XE_old(i,1:3)';
    XE(i,4:6) = r2eulxyz(R_W_0*eulxyz2r(XE_old(i,4:6)'));
  end
end
%% Trajektorie generieren
if trajset.profile == 1
  % TODO: Hier wäre eine echte Orientierungsinterpolation besser (z.B.
  % SLERP). Aktuell unplausibel, wenn Euler-Winkel über 180° springen.
  % Dann komplette Interpolation von -180° bis +180° hier.
  [X_ges,XD_ges,XDD_ges,T_ges,IE] = traj_trapez2_multipoint(XE, ...
    trajset.vmax, trajset.vmax/trajset.amax, trajset.Tv, trajset.Ts, 0); % muss noch bearbeiten
elseif trajset.profile == 0 % Nur Eckpunkte
  X_ges = XE;
  XD_ges = XE*0;
  XDD_ges = XE*0;
  T_ges = (1:size(XE,1))'; % Muss Spaltenvektor sein
  IE = (1:size(XE,1))';
else
  error('Profil nicht definiert');
end

%% Ausgabe
Traj = struct('X', X_ges, 'XD', XD_ges, 'XDD', XDD_ges, 't', T_ges, 'XE', XE, 'IE', IE);
