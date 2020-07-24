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
  % Fahrt in die Mitte (nur Eckpunkte reicht nicht für Struktursynthese.
  % Dann können Umklapp-Lagen bereits die Punkte erfüllen).
  k=k+1; XE(k,:) = XE(k-1,:) + [d1/2,-d1/2, -h1/2, 0,0,0];
end

%% Liste der Trajektorien: 3T1R
if all(DoF == [1 1 1 0 0 1])
  % Beginn Würfel mit zusätzlicher EE-Drehung
  d1=0.3;
  h1=0.3;
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
end

%% Liste der Trajektorien: 3T3R und 3T2R
if all(DoF == [1 1 1 1 1 1]) || all(DoF == [1 1 1 1 1 0])
  if no == 1 || no == 2
    % Beginn Würfel mit zusätzlicher 3D-EE-Drehung
    d1=0.3;
    h1=0.3;
    k=k+1; XE(k,:) = XE(k-1,:) + [ d1,0,0, 0,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,-d1,0  0,0, pi/4];
    k=k+1; XE(k,:) = XE(k-1,:) + [-d1,0,0, 0,0,-pi/4];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0,-h1, pi/4,0, 0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0, h1, -pi/4,0,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,d1,0,  0,pi/4,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0,-h1, 0,-pi/4,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ d1,0,0, pi/6,-pi/6,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0,0,h1, pi/6,-pi/6,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [ 0,0,-h1, -pi/6,pi/3,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,-d1,0  -pi/6,pi/6,0];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0, h1, -pi/4,pi/4,-pi/3];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0,-h1, pi/2,-pi/6,-pi/3];
    k=k+1; XE(k,:) = XE(k-1,:) + [-d1,0,0, pi/12,-pi/6,pi/2];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,d1,0,  -pi/4,0,-pi/6];
    k=k+1; XE(k,:) = XE(k-1,:) + [0,0, h1, -pi/12,pi/12,pi/3];
    if no == 2
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
    end
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

%% Trajektorie generieren
if trajset.profile == 1
  [X_ges,XD_ges,XDD_ges,T_ges] = traj_trapez2_multipoint(XE, ...
    trajset.vmax, trajset.vmax/trajset.amax, trajset.Tv, trajset.Ts, 0); % muss noch bearbeiten
elseif trajset.profile == 0 % Nur Eckpunkte
  X_ges = XE;
  XD_ges = XE*0;
  XDD_ges = XE*0;
  T_ges = (1:size(XE,1))'; % Muss Spaltenvektor sein
else
  error('Profil nicht definiert');
end

%% Ausgabe
Traj = struct('X', X_ges, 'XD', XD_ges, 'XDD', XDD_ges, 't', T_ges, 'XE', XE);
return

%% Debug: Trajektorie anschauen
% Kippwinkelaus Euler-Winkeln berechnen
phiK_ges = NaN(length(T_ges),1);
for i = 1:length(phiK_ges)
  % Berechne z-Achse
  R = eulxyz2r(X_ges(i,4:6)');
  % Kippwinkel aus 
  phiK_ges(i) = acos(R(:,3)' * [0;0;1]);
end
figure(1);clf;
subplot(2,2,1);view(3);
plot3(1e3*X_ges(:,1), 1e3*X_ges(:,2), 1e3*X_ges(:,3));
xlabel('x in mm');ylabel('y in mm');zlabel('z in mm');
for i = 1:size(XE,1)
  text(1e3*XE(i,1), 1e3*XE(i,2), 1e3*XE(i,3)+3*i, sprintf('%d', i));
end
grid on;
subplot(2,2,2);
plot(T_ges, 1e3*X_ges(:,1:3));
ylabel('pos in mm');
grid on;
subplot(2,2,3);
plot(T_ges, 180/pi*X_ges(:,4:6));
legend({'Eul X', 'Eul Y', 'Eul Z'});
ylabel('Euler-Winkel in Grad');
grid on;
subplot(2,2,4);
plot(T_ges, 180/pi*phiK_ges);
ylabel('Schwenk-Winkel in Grad');
grid on;
