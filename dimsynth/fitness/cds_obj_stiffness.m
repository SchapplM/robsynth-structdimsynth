% Zielfunktion ("objective function") für Optimierung in der Maßsynthese
% basierend auf der Steifigkeit des Roboters.
% Die Steifigkeit wird in einen normierten Zielfunktionswert übersetzt
% Damit kleine Zielfunktionen besser sind, wird effektiv die Nachgiebigkeit
% benutzt.
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Q
%   Gelenkpositionen des Roboters (für PKM auch passive Gelenke)
%
% Ausgabe:
% fval [1x1]
%   Zielfunktionswert, der im PSO-Algorithmus minimiert wird
%   Wird aus vergleichbarer Nachgiebigkeit gebildet.
% fval_debugtext [char]
%   Zeile mit Hinweistext, der bei PSO nach Fitness-Berechnung ausgegeben wird
% debug_info [cell]
%   Zusatz-Informationen, die im Debug-Bild des Roboters angezeigt werden
% fval_phys [1x1]
%   Physikalischer Wert, der dem Zielfunktionswert zugrunde liegt
%   Hier: Schlechtester Wert für die (translatorische) Nachgiebigkeit in
%   m/N. Es wird ein äquivalenter mittlerer Wert für alle Richtungen
%   benutzt
%
% Quellen:
% [GoerguelueDed2019] A New Stiffness Performance Index: Volumetric
% Isotropy Index
% [Zhao2020] Modellierung und Maßsynthese serieller und paralleler Roboter
% hinsichtlich der strukturellen Steifigkeit (Masterarbeit)
% [Klimchik2011] Enhanced stiffness modeling of serial and parallel
% manipulators for robotic-based processing of high performance materials

% Masterarbeit Yuqi ZHAO, zhaoyuqi.nicolas@gmail.com, 2019-12
% Betreuer: Moritz Schappler, moritz.schappler@imes.uni-hannover.de
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [fval, fval_debugtext, debug_info, fval_phys] = cds_obj_stiffness(R, Set, Q)
debug_info = {};

% Berechne Steifigkeit über Trajektorie
Keigges = NaN(size(Q,1), 3);
% Berechne Steifigkeit für alle Punkte der Bahn
for i = 1:size(Q,1)
  % Kartesische Steifigkeitsmatrix (6x6)
  K_ges = R.stiffness(Q(i,:)');
  % Auswahl der translatorischen Submatrix (3x3), Normierung auf N/mm
  % (damit Zahlenwerte eher im Bereich 1 liegen)
  K_trans_norm = 1e-3*K_ges(1:3,1:3);
  
  Keigges(i,1:3) = sort(eig(K_trans_norm)); % Eigenwert der transl. St. in N/mm
  if any(Keigges(i,1:3)<0)
    error('Die Steifigkeitsmatrix kann keine negativen EW haben!');
  end
  
  % Alternative Berechnung: Volumen des Steifigkeit-Ellipsoids
%   % [GoerguelueDed2019], Gl. 21
%   % Entspricht Volumen des Ellipsoids der Nachgiebigkeits-Matrix
%   V(i,:) = 1/det(K_trans_norm);% 1/prod(eig(K_trans_norm));
end
if any(isnan(Keigges))
  warning('Hier stimmt etwas nicht');
end
[f_sti_min, I_sti_min] = min(Keigges(:,1));
f_com = 1/f_sti_min; % Größte Nachgiebigkeit in mm/N

% Alternative Berechnung für Ellipsoid:
% Schlechtester Wert des Volumens vom Ellipsoid ist Kennzahl
% f_sti = max(V)^(1/3); % Umrechnung vom Volumen auf den Radius mit ^(1/3)

% fprintf('Niedrigste Steifigkeit: %1.3f N/mm bzw. höchste Nachgiebigkeit: %1.3f mm/N\n', 1/f_com, f_com);
% Normierung (bezogen auf Nachgiebigkeit bzw. Steifigkeit): 
% 1e-3 mm/N bzw. 1000 N/mm -> 0.06; (sehr steif; gut)
% 1e-2 mm/N bzw. 100 N/mm  -> 0.50; (moderate Steifigkeit für einen Roboter)
% 0.1 mm/N bzw. 10 N/mm    -> 0.94 (eher niedrige Steifigkeit; schlecht)
% 1 mm/N bzw. 1N/mm        -> 0.99
f_com_norm = 2/pi*atan(f_com/1e-2); 
fval = 1e3*f_com_norm; % Normiert auf 0 bis 1e3
fval_debugtext = sprintf('Nachgiebigkeit %1.3f mm/N; Steifigkeit %1.3f N/mm', f_com, 1/f_com);
fval_phys = 1e-3 * f_com; % Umrechnung in äquivalenten physikalischen Wert (mm/N -> m/N)
debug_info = {sprintf('min. Steifigkeit: %1.3f N/mm', 1/f_com)};

%% Debug-Zeichnung erstellen
if fval < Set.general.plot_details_in_fitness
  change_current_figure(205); clf; hold all;
  if ~strcmp(get(205, 'windowstyle'), 'docked')
    % set(202,'units','normalized','outerposition',[0 0 1 1]);
  end
  hdleig=plot(Keigges);
  hdl=plot([0; size(Q,1)], 1/Set.optimization.constraint_obj(5)*[1;1], 'r--');
  hdlworst=plot(I_sti_min, f_sti_min, 'ko');
  xlabel('Datenpunkte');
  ylabel('Steifigkeit in N/mm (niedriger=schlechter)');
  grid on;
  sgtitle('Analyse der Nachgiebigkeit');
  legend([hdleig;hdl;hdlworst], {'Nmin', 'Nmid', 'Nmax', 'Untergrenze', 'schlechtester'});
end