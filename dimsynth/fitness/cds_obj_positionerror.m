% Zielfunktion ("objective function") für Optimierung in der Maßsynthese
% basierend auf Lagegenauigkeit des Roboters.
% Der Positionsfehler wird in einen normierten Zielfunktionswert übersetzt
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Jinvges
%   Zeilenweise (inverse) Jacobi-Matrizen des Roboters (für PKM). Bezogen
%   auf vollständige Gelenkgeschwindigkeiten und EE-Geschw.
% Traj_0
%   Endeffektor-Trajektorie (bezogen auf Basis-KS)
% Q
%   Gelenkpositionen des Roboters (für PKM auch passive Gelenke)
% 
% Ausgabe:
% fval [1x1]
%   Zielfunktionswert, der im PSO-Algorithmus minimiert wird
% fval_debugtext [char]
%   Zeile mit Hinweistext, der bei PSO nach Fitness-Berechnung ausgegeben wird
% debug_info [cell]
%   Zusatz-Informationen, die im Debug-Bild des Roboters angezeigt werden
% f_poserr [1x1]
%   Physikalischer Wert, der dem Zielfunktionswert zugrunde liegt
%   Hier: Maximaler Positionsfehler des Endeffektors (in Trajektorie)
% 
% Quelle:
% * [Rob2LUH] Skript Robotik II (Prof. Ortmaier, Uni Hannover), Kap. 1.6.3

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-10
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval, fval_debugtext, debug_info, f_poserr] = cds_obj_positionerror(R, Set, Jinvges, Traj_0, Q)
debug_info = {};

% Treffe Annahme über die Genauigkeit der angetriebenen Gelenke
% Nehme Beispielwerte aus Datenblättern (Heidenhein). Die genauen Werte
% sind nicht so wichtig, da der Vergleich verschiedener Roboter im
% Vordergrund steht.
% https://www.heidenhain.de/de_DE/produkte/winkelmessgeraete/winkelmessmodule/baureihe-mrp-2000/
% Genauigkeit: 7 Winkelsekunden; Umrechnung in Grad und Radiant
delta_rev = 7 * 1/3600 * pi/180;
% https://www.heidenhain.de/de_DE/produkte/laengenmessgeraete/gekapselte-laengenmessgeraete/fuer-universelle-applikationen/
delta_pris = 10e-6; % 10 Mikrometer

% Benutze angepasste Genauigkeitswerte, je nachdem, wie groß der
% Wertebereich der Gelenke ist. Annahme: Bei einer sehr großen
% Verfahrbewegung kann keine Mikrometergenauigkeit erreicht werden
% TODO: Hier noch Anpassen.
if all(R.MDH.sigma(R.I_qa) == 1) % Alle Antriebe sind Schubantriebe
  % Spannweite der Schubantriebe
  qa_span = diff(minmax2(Q(:,R.I_qa)')');
  % Da symmetrischer Roboter aufgebaut wird, zählt die größte Spannweite
  % der Antriebe
  qa_span_max = max(qa_span);
  if qa_span_max > 0.100 % Verfahrweg mehr als 100mm
    delta_pris = 10e-6; % 10µm
  else
    % Annahme: Feinere Bewegung möglich, da Bereich klein
    delta_pris = 1e-6;
  end
end

if R.Type == 0 % Seriell
  delta_qa = NaN(R.NQJ,1);
  delta_qa(R.MDH.sigma==0) = delta_rev;
  delta_qa(R.MDH.sigma==1) = delta_pris;
else
  delta_qa = NaN(sum(R.I_qa),1);
  delta_qa(R.MDH.sigma(R.I_qa)==0) = delta_rev;
  delta_qa(R.MDH.sigma(R.I_qa)==1) = delta_pris;
end

% Berechne Positionsfehler über Trajektorie
deltapges = NaN(length(Traj_0.t), 1);
if R.Type == 0 % Seriell
  % Berechne Manipulierbarkeit für alle Punkte der Bahn
  for i = 1:length(Traj_0.t)
    J_3T = R.jacobit(Q(i,:)'); % nur translatorisch
    J_transl = J_3T(Set.task.DoF(1:3),:);
    % Berechne Positionsfehler (siehe [Rob2LUH])
    deltapges(i,:) = norm(abs(J_transl)*delta_qa);
  end
else % PKM
  % Berechne Manipulierbarkeit für alle Punkte der Bahn
  for i = 1:length(Traj_0.t)
    Jinv_IK = reshape(Jinvges(i,:), R.NJ, sum(R.I_EE));
    J = inv(Jinv_IK(R.I_qa,:));
    % Wähle translatorischen Teil der Jacobi aus
    J_transl = J(Set.task.DoF(1:3),:);
    deltapges(i,:) = norm(abs(J_transl)*delta_qa); % siehe [Rob2LUH]
  end
end
% Maximaler Fehler (über Trajektorie) ist Kennzahl
f_poserr = max(deltapges);
f_poserr_norm = 2/pi*atan(f_poserr/1.5e-4); % Normierung auf 0 bis 1; 1e-3 ist 0.9.
fval = 1e3*f_poserr_norm; % Normiert auf 0 bis 1e3
fval_debugtext = sprintf('Schlechtester Positionsfehler %1.1fµm.', 1e6*f_poserr);