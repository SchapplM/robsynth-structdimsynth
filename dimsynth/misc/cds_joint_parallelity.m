% Bestimme die Parallelität von PKM-Beinketten anhand der Ergebnisse der
% Kinematik in der Test-Trajektorie. Wird in Struktursynthese benutzt, um
% den Namen des Roboters bestimmen zu können.
% Ist nicht nur anhand der seriellen Beinkette bestimmbar, da im
% zusammengebauten Zustand anders als Beinkette isoliert.
% 
% Eingabe:
% R
%   Matlab-Klasse für Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus
% Structure
%   Eigenschaften der Roboterstruktur
% Q
%   Gelenkwinkel-Trajektorie für Ergebnis der Maßsynthese
% 
% Speichert Datei:
% ...joint_parallelity.txt
%   Nummern für Parallelität der Gelenkachsen der Beinketten.
%   Betrachtet nur eine einzige Kette (Annahme, dass symmetrisch)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-11
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function cds_joint_parallelity(R, Set, Structure, Q)
if Set.general.matfile_verbosity > 2
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', ...
    'cds_get_joint_parallelity.mat'));
end
%% Parallelität der Gelenke anzeigen (anhand der Gelenk-Trajektorie).
NLegJ = size(Q,2) / R.NLEG;
pgroups_legs = NaN(R.NLEG, NLegJ);
for iLeg = 1:R.NLEG
  % Direkte Kinematik für alle Zeitschritte berechnen
  Zges = NaN(size(Q,1), 3*NLegJ); % Alle z-Achsen speichern zum späteren Vergleich
  pgroups_all = zeros(size(Q,1), NLegJ);
  sigma_leg = R.Leg(1).MDH.sigma;
  for k = 1:size(Q,1)
    Tc = R.Leg(1).fkine(Q(k,1:NLegJ)');
    for kk = 1:NLegJ
      Zges(k,(kk-1)*3+1:kk*3) = Tc(1:3,3,1+kk);
    end
    % Werte die Parallelität der Achsen aus
    for jj = 1:NLegJ
%       if sigma_leg(jj) == 1
        % Schubgelenk. Gruppe trotzdem zählen. Sonst würde die Gruppe nach
        % dem Schubgelenk wieder bei 1 anfangen
%         continue
      if jj == 1 % Erstes Gelenk ist per Definition immer Gruppe 1
        pgroups_all(k, jj) = 1;
        continue
      end
      for kk = 1:jj-1
        if sigma_leg(kk) == 1
          % Parallelität zum Schubgelenk wird betrachtet
          % Steht so nicht in [KongGos2007] drin. Wird hier neu eingeführt.
%           continue
        end
        % Prüfe welches die erste z-Achse ist, die identisch mit der
        % aktuellen ist
        z_jj = Zges(k,(jj-1)*3+1:jj*3);
        z_kk = Zges(k,(kk-1)*3+1:kk*3);
        if all(abs(z_jj-z_kk) < 1e-6) || all(abs(z_jj+z_kk) < 1e-6) % parallel oder antiparallel ist gleichwertig
          pgroups_all(k, jj) = pgroups_all(k, kk); % Gelenk jj ist parallel zu Gelenk kk
          break
        else % Debug
          % deltaphi = acos(dot(z_jj,z_kk));
          % fprintf('Beingelenk %d vs %d: %1.1f deg Verdreht\n', jj, kk, 180/pi*deltaphi);
        end
      end
      if pgroups_all(k, jj) == 0
        pgroups_all(k, jj) = max(pgroups_all(k, 1:jj-1)) + 1; % Neue Gruppe
      end
    end
    if k > 1 && any(pgroups_all(k-1,:) ~= pgroups_all(k,:))
      cds_log(2, sprintf('[joint_parallelity] Parallelität ändert sich in Zeitschritt %d (kein Fehler)', k));
    end
  end
  if any(any(diff(pgroups_all)))
    warning('Die Parallelität ändert sich im Zeitverlauf. Sollte nicht sein.');
    % Kann passieren, wenn Drehgelenke parallel zu einem Schubgelenk werden
    % Nehme den allgemeineren Fall mit mehr unterschiedlichen Gruppen
    I_maxdiv = max(pgroups_all, [], 2);
    pgroups = pgroups_all(find(I_maxdiv,1,'first'),:);
  else
    % Keine Änderung der Parallelität. Nehme die Gruppen vom ersten Fall.
    pgroups = pgroups_all(1,:);
  end
  pgroups_legs(iLeg,:) = pgroups;
end
%% Prüfen der Ergebnisse
legs_error = false;
for iLeg = 1:R.NLEG
  if ~all(pgroups_legs(iLeg,:) == pgroups)
    cds_log(-1, sprintf(['[joint_parallelity] Parallelität der Gelenke ', ...
      'in den verschiedenen Beinketten nicht gleich (Kette %d vs %d)'], iLeg, R.NLEG));
    legs_error = true;
  end
end
if legs_error % Die Datei markiert, dass es einen Fehler gab.
  writematrix(pgroups_legs, fullfile(Set.optimization.resdir, Set.optimization.optname, ...
    sprintf('Rob%d_%s_joint_parallelity_error_legs.txt', Structure.Number, Structure.Name)));
end
%% Abspeichern der Parallelität
writematrix(pgroups, fullfile(Set.optimization.resdir, Set.optimization.optname, ...
  sprintf('Rob%d_%s_joint_parallelity.txt', Structure.Number, Structure.Name)));
cds_log(1, sprintf('[joint_parallelity] Parallelität der Beinketten exportiert: [%s]', disp_array(pgroups, '%d')));
