% Gütefunktion für Maßsynthese von Robotern (allgemein)
% 
% Eingabe:
% R
%   Matlab-Klasse für Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus
% Traj_W
%   Trajektorie (bezogen auf Welt-KS)
% Structure
%   Eigenschaften der Roboterstruktur
% p
%   Vektor der Optimierungsvariablen für PSO
% desopt_pval (optional)
%   Vektor der Optimierungsvariablen der Entwurfsoptimierung. Falls
%   gegeben, wird keine Entwurfsoptimierung durchgeführt. Notwendig bei
%   nachträglicher Auswertung.
% 
% Ausgabe:
% fval
%   Fitness-Wert für den Parametervektor p. Enthält Strafterme für
%   Verletzung von Nebenbedingungen oder Wert der Zielfunktion (je nachdem)
%   Werte:
%   0...1e3: gewählte Zielfunktion
%   1e3...1e4: Überschreitung dynamischer NB (Antriebskraft)
%   1e4...1e5: Nebenbedingung für Zielfunktion in Entwurfsopt. überschritten
%   1e5...1e6: Überschreitung Belastungsgrenze der Segmente (aus cds_dimsynth_desopt_fitness)
%   1e6...5e6: Überschreitung kinematischer NB (Positionsfehler)
%   5e6...1e7: Überschreitung kinematischer NB (Kondition)
%   1e7...1e9: Siehe cds_constraints_traj. Werte von dort mit Faktor 1e4 multipliziert
%   1e9...1e13: Siehe cds_constraints. Werte von dort mit Faktor 1e4 multipliziert
%   1e13...1e14: Siehe cds_constraints_parameters (Parameter unplausibel)
% physval
%   Physikalische Werte, die den Werten aus fval entsprechen
% Q_out, QD_out, QDD_out
%   Gelenkwinkel-Trajektorie für Ergebnis
% TAU_out, JP_out, Jinv_out, X6Traj_out
%   Dem Fitness-Wert zugrunde liegende Antriebsmomente, Punktkoordinaten
%   der Gelenke und (inverse) Jacobi-Matrizen, EE-Drehungen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval, physval, Q_out, QD_out, QDD_out, TAU_out, JP_out, Jinv_out, X6Traj_out] = ...
  cds_fitness(R, Set, Traj_W, Structure, p, desopt_pval)
% Prüfe Zurücksetzen der persistenten Variable
persistent abort_fitnesscalc
if nargin == 0
  abort_fitnesscalc = [];
  return;
end
repopath = fileparts(which('structgeomsynth_path_init.m'));
rng(0); % Für Wiederholbarkeit der Versuche: Zufallszahlen-Initialisierung

% Debug:
if Set.general.matfile_verbosity > 2
  save(fullfile(repopath, 'tmp', 'cds_fitness_1.mat'));
end
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_fitness_1.mat'),  '-regexp', '^(?!abort_fitnesscalc$).');

t1=tic();
Q_out = []; QD_out = []; QDD_out = []; TAU_out = []; JP_out = []; Jinv_out = [];
X6Traj_out = [];
debug_info = {};
% Alle möglichen physikalischen Werte von Nebenbedingungen (für spätere Auswertung)
constraint_obj_val = NaN(length(Set.optimization.constraint_obj),1);
fval = NaN(length(Set.optimization.objective),1);
physval = fval;
desopt_pval_given = false;
if nargin == 6 && ~isempty(desopt_pval) && ~all(isnan(desopt_pval))% Die Eingabevariable ist gesetzt
  % Prüfe, ob Entwurfsvariable der Schubgelenk-Offsets gegeben ist
  if ~any(isnan(desopt_pval(Structure.desopt_ptypes==1)))
    % Keine Optimierung von Entwurfsparametern durchführen. Trage die Schub-
    % gelenk-Offsets aus dem gegebenen Ergebnis direkt in die Klasse ein.
    if Structure.desopt_prismaticoffset
      p_prismaticoffset = desopt_pval(Structure.desopt_ptypes==1);
      if Structure.Type == 0
        R.DesPar.joint_offset(R.MDH.sigma==1) = p_prismaticoffset;
      else
        for i = 1:R.NLEG
          R.Leg(i).DesPar.joint_offset(R.Leg(i).MDH.sigma==1) = p_prismaticoffset;
        end
      end
    end
    Structure.desopt_prismaticoffset = false; % keine Optimierung
  else
    % Übergebener Wert war NaN. Optimierung doch durchführen
    Structure.desopt_prismaticoffset = true;
  end
  % Prüfe, ob weitere Entwurfsvariablen gegeben sind.
  if all(~isnan(desopt_pval(Structure.desopt_ptypes~=1)))
    desopt_pval_given = true; % bezieht sich nicht auf den obigen Fall
    % Werte für die Gelenkfeder-Ruhelagen weiter unten einstellen.
    % Optimierungsvariablen deaktivieren. Hierdurch keine erneute Optimierung.
    Set.optimization.desopt_vars = {};
    % Keine Berechnungen für Entwurfsoptimierung mehr notwendig
    Structure.calc_dyn_reg = false;
    Structure.calc_spring_reg = false;
  else
    % Übergebener Wert war NaN. Optimierung doch durchführen. Keine
    % Anpassung notwendig (desopt_vars ist noch richtig eingestellt).
  end
else
  desopt_pval = NaN(length(Structure.desopt_ptypes),1);
end
%% Abbruch prüfen
t_end_plan = Set.optimization.start_time + ... % Rechne in Tagen
  Set.optimization.max_time/(24*3600); % geplante Endzeit
% Prüfe, ob Berechnung schon abgebrochen werden kann, weil ein anderes
% Partikel erfolgreich berechnet wurde. Dauert sonst eine ganze Generation.
if isempty(abort_fitnesscalc)
  abort_fitnesscalc = false;
elseif abort_fitnesscalc
  if t_end_plan < now() && ~abort_fitnesscalc % Meldung nur einmal zeigen und nur vor wirklichem Abbruch.
    cds_log(2,sprintf('[fitness] Zeit-Grenze (%1.1fh ab %s) überschritten. Abbruch.', ...
      Set.optimization.max_time/3600, datestr(Set.optimization.start_time, 'YYYY-mm-dd HH:MM:SS')));
  end
  fval(:) = Inf;
  cds_save_particle_details(Set, R, toc(t1), fval, p, physval, constraint_obj_val, desopt_pval);
  return;
end
% Prüfe Abbruch aufgrund überschrittener Rechenzeit. Nach obiger Prüfung,
% damit einmaliger Aufruf von cds_fitness ohne Löschung der Variable max_time weiterhin möglich ist.
if t_end_plan < now()
  abort_fitnesscalc = true;
end
% Generations- und Individuumsnummer der Optimierung bestimmen
[~, i_gen, i_ind] = cds_save_particle_details([],[],0,0,NaN,NaN,NaN,NaN,'output');
%% Parameter prüfen
if p(1) == 0
  error('Roboterskalierung kann nicht Null werden');
end

%% Parameter aktualisieren
% Keine Verwendung der Ausgabe: Parameter werden direkt in ursprüngliche
% Funktion geschrieben; R.pkin ist vor/nach dem Aufruf unterschiedlich
p_phys = cds_update_robot_parameters(R, Set, Structure, p);

%% Plausibilitätsprüfung der Parameter
try
  [fval_constrparam, constrvioltext] = cds_constraints_parameters(R, Set, Structure, p, p_phys);
catch err
  fval(:) = 1e14; % Größtmöglicher Wert für cds_constraints_parameters
  cds_log(-1, sprintf(['[fitness] Fehler in cds_constraints_parameters: %s. ' ...
    'Setze fval=%1.1e.\n%s'], err.message, fval(1), getReport(err, 'extended')));
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
    'tmp', ['cds_fitness_call_cds_constraints_parameters_error_', R.mdlname, '.mat']));
  abort_fitnesscalc = true;
  cds_save_particle_details(Set, R, toc(t1), fval, p, physval, constraint_obj_val, desopt_pval);
  return
end
if fval_constrparam > 0
  fval(:) = fval_constrparam;
  cds_log(2,sprintf(['[fitness] G=%d;I=%d. Fitness-Evaluation in %1.2fs. ', ...
    'fval=%1.3e. %s'], i_gen, i_ind, toc(t1), fval_constrparam, constrvioltext));
  cds_save_particle_details(Set, R, toc(t1), fval, p, physval, constraint_obj_val, desopt_pval);
  return
end
%% Trajektorien-Struktur bezüglich der Eckpunkte anpassen
% Komplette Trajektorie wird erst weiter unten gebraucht. Rechenzeit sparen
if ~isfield(Traj_W, 'nullspace_maxvel_interp')
  Traj_W.nullspace_maxvel_interp = zeros(2,0); % Abwärtskompatibilität
end
Traj_0_E = cds_transform_traj(R, struct('XE', Traj_W.XE));

%% Nebenbedingungen prüfen (für Eckpunkte)
t0 = tic();
qlim_test = R.update_qlim(Structure.qlim);
if all(isnan(qlim_test(:)))
  cds_log(-1, '[fitness] qlim ist NaN. Darf nicht sein.')
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
    'tmp', 'cds_fitness_qlimnan_error_debug.mat'));
end
try
  [fval_constr,QE_iIKC, Q0, constrvioltext, Stats_constraints] = cds_constraints(R, Traj_0_E, Set, Structure);
catch err
  fval(:) = 1e13; % Größtmöglicher Wert für cds_constraints
  dbgfile=fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
    'tmp', ['cds_fitness_call_cds_constraints_error_', R.mdlname, '.mat']);
  cds_log(-1, sprintf(['[fitness] Fehler in cds_constraints: %s. Zustand ' ...
    'gespeichert: %s. Setze fval=%1.1e.\n%s'], err.message, dbgfile, fval(1), getReport(err, 'extended')));
  save(dbgfile);
  abort_fitnesscalc = true;
  cds_save_particle_details(Set, R, toc(t1), fval, p, physval, constraint_obj_val, desopt_pval);
  return
end
cds_log(2,sprintf(['[fitness] G=%d;I=%d. Nebenbedingungen für Einzelpunkte ', ...
  'in %1.2fs geprüft. %d IK-Konfigurationen gefunden. fval_constr=%1.3e. %s'],...
  i_gen, i_ind, toc(t0), size(Q0,1)-sum(any(isnan(Q0),2)), fval_constr, constrvioltext));
% Füge weitere Anfangswerte für die Trajektorien-IK hinzu. Diese werden
% zusätzlich vorgegeben (bspw. aus vorherigem Ergebnis, das reproduziert
% werden muss). Wird genutzt, falls aus numerischen Gründen die Einzelpunkt-
% IK nicht mehr reproduzierbar ist (z.B. durch Code-Änderung)
if all(~isnan(Structure.q0_traj)) && Set.task.profile ~= 0 % nur, falls es auch eine Trajektorie gibt
  % Prüfe, ob diese vorgegebene Werte auch von alleine gefunden wurden.
  Q0_err = Q0-repmat(Structure.q0_traj',size(Q0,1),1);
  Q0_err(:,R.MDH.sigma==0) = angleDiff(Q0(:,R.MDH.sigma==0), repmat(Structure.q0_traj(R.MDH.sigma==0)',size(Q0,1),1));
  I_match = all(abs(Q0_err)<1e-6,2);
  if ~any(I_match)
    if any(isnan(Q0_err(:)))
      cds_log(-1,['[fitness] Vorher gab es keine IK-Lösung. q0_traj löst ', ...
        'die IK aber. Dürfte eigentlich nicht sein. Lösung nicht reproduzierbar.']);
    else
      cds_log(-1,sprintf(['[fitness] Vorgegebene Werte aus q0_traj wurden nicht ', ...
        'in den %d IK-Konfigurationen gefunden. Max. Diff. %1.1e'], size(Q0,1), min(max(abs(Q0_err),[],2))));
    end
    % Prüfe, ob der vorgegebene Wert die IK löst. Wenn nicht, treten
    % nachfolgend Fehler in der IK auf (z.B. Dynamische Programmierung)
    % Ist relevant für den Fall der Aufgabenredundanz (geregelt in Klasse)
    if R.Type == 0
      Phi_test = R.constr2(Structure.q0_traj, R.x2tr(Traj_0_E.XE(1,:)'), true);
      Phi_test = Phi_test(R.I_constr_red);
    else
      Phi_test = R.constr3(Structure.q0_traj, Traj_0_E.XE(1,:)');
    end
    if any(abs(Phi_test) > 1e-8)
      cds_log(-1,sprintf(['[fitness] Vorgegebene Werte aus q0_traj lösen ', ...
        'nicht die Kinematik (max err %1.1e). Nicht verwenden.'], max(abs(Phi_test))));
    else
      % Füge vorgegebene Startkonfiguration hinzu
      if fval_constr < 1e6 && ~any(isnan(Q0(:))) % IK-Lösung gefunden
        Q0 = [Structure.q0_traj'; Q0]; % Prüfe vorgegebenen Wert zuerst.
        % Erzeuge Platzhalter-Werte für spätere Rechnungen
        QE_iIKC(:,:,size(QE_iIKC,3)+1) = repmat(Structure.q0_traj', size(QE_iIKC,1), 1);
        QE_iIKC = QE_iIKC(:,:,[end,1:end-1]); % Stelle konsistente Reihenfolge zu Q0 wieder her
      else % Es wurde keine IK-Lösung gefunden (sichtbar durch NaN). Nehme nur die q0_traj
        Q0 = Structure.q0_traj';
        QE_iIKC = [Structure.q0_traj'; NaN(size(QE_iIKC,1)-1, size(QE_iIKC,2))];
      end
      % Damit wird die Traj.-IK immer geprüft, auch wenn die Einzelpunkt-IK
      % nicht erfolgreich gewesen sein sollte
      fval_constr = 1e3;
    end
  else
    % Der Wert wurde ungefähr erreicht. Ersetze durch den genau exakten
    % Wert, damit es nicht zu numerischen Abweichungen kommen kann.
    II_match = find(I_match, 1, 'first');
    Q0(II_match,:) = Structure.q0_traj';
    QE_iIKC(1,:,II_match) = Structure.q0_traj';
    % Setze die Reihenfolge so, dass der gesuchte Wert zuerst kommt. Dann
    % direkter Abbruch möglich über obj_limit.
    Q0 = [Q0(II_match,:); Q0(~I_match,:)];
    QE_iIKC = QE_iIKC(:,:,[II_match; find(~I_match)]);
    % Konsistente Reihenfolge
    if fval_constr > 1e3
      cds_log(-1,sprintf(['[fitness] Vorgegebene Werte aus q0_traj erzeugen ', ...
        'unzulässige Lösung in Positions-IK. Benutze trotzdem.']));
      fval_constr = 1e3;
    end
  end
end

% Entwurfsparameter speichern (falls hiernach direkt Abbruch)
if Structure.desopt_prismaticoffset % siehe cds_desopt_prismaticoffset.m
  if Structure.Type == 0
    desopt_pval(Structure.desopt_ptypes==1) = R.DesPar.joint_offset(R.MDH.sigma==1);
  else
    desopt_pval(Structure.desopt_ptypes==1) = R.Leg(1).DesPar.joint_offset(R.Leg(1).MDH.sigma==1);
  end
end
% Bei Struktursynthese soll die prinzipielle Nicht-Lösbarkeit zum Abbruch
% führen. Wird durch Abfrage auf Inkonsistenz der NB in cds_constraints ausgelöst.
if any(strcmp(Set.optimization.objective, 'valid_act')) && fval_constr==1e7
  if ~abort_fitnesscalc % Folgende Meldung nur einmal anzeigen.
    cds_log(2,sprintf('[fitness] Die PKM ist nicht modellierbar.'));
  end
  abort_fitnesscalc = true;
  % Der Abbruch erfolgt innerhalb der nächsten Prüfung
end


%% Abbruchbedingung aus Eckpunkt-Nebenbedingungen prüfen
if all(fval_constr > 9e5) && all(fval_constr < 1e6) || ... % Grenzen für IK-Jacobi-Singularität
    all(fval_constr > 8e5) && all(fval_constr < 9e5) || ... % Grenzen für Jacobi-Singularität
    fval_constr == 9.9e6 % Bisher ausschließlich Singularität und IK-Misserfolg
  % Prüfe, wie oft schon dieses Ergebnis vorlag
  PSO_Detail_Data = cds_save_particle_details([], [], 0, 0, NaN, NaN, NaN, NaN, 'output');
  if ~isempty(PSO_Detail_Data)
    % Bei mehrkriterieller Optimierung bei NB-Verletzung gleiche Einträge 
    fval_hist = PSO_Detail_Data.fval(:,1,:);
    I_sing = fval_hist > 1e4*8e5 & fval_hist < 1e4*1e6 | fval_hist == 1e4*9.9e6;
    I_nonsing = fval_hist < 1e4*8e5;
    if fval_constr == 9.9e6 && sum(I_sing(:)) > 20 && sum(I_nonsing(:)) == 0 ||... % mehr Versuche zulassen
       fval_constr ~= 9.9e6 && sum(I_sing(:)) > 4 && sum(I_nonsing(:)) == 0
      if ~abort_fitnesscalc % Meldung nur einmal zeigen
        cds_log(2,sprintf(['[fitness] Es gab %d singuläre Ergebnisse und kein ', ...
          'nicht-singuläres. Roboter nicht geeignet. Abbruch der Optimierung.'], sum(I_sing(:))));
      end
      abort_fitnesscalc = true;
    end
  end
end


% NB-Verletzung in Eckpunkt-IK wird in Ausgabe mit Werten von 1e5 aufwärts
% angegeben. Umwandlung in Werte von 1e9 aufwärts.
% Ursache: Nachträgliches Einfügen von weiteren Nebenbedingungen.
if fval_constr > 1000 % Nebenbedingungen verletzt.
  fval(:) = fval_constr*1e4; % Erhöhung, damit später kommende Funktionswerte aus Entwurfsoptimierung kleiner sein können
  % Speichere die Anfangs-Winkelstellung in der Roboterklasse für später.
  % Dient zum Vergleich und zur Reproduktion der Ergebnisse
  R.update_qref(Q0(1,:)');
  % Belege die Ausgabe mit den berechneten Gelenkwinkeln. Dann kann immer
  % noch das Bild gezeichnet werden (zur Fehlersuche)
  Q_out = QE_iIKC(all(~isnan(QE_iIKC),2), :);
  
  % Speichere Gelenk-Grenzen. Damit sehen eine hieraus erstellte 3D-Bilder
  % besser aus, weil die Schubgelenk-Führungsschienen ungefähr stimmen.
  update_joint_limits(R, Set, QE_iIKC(:,:,1), true, 0);
  cds_log(2,sprintf(['[fitness] G=%d;I=%d. Fitness-Evaluation in %1.2fs. ', ...
    'fval=%1.3e. %s'],i_gen, i_ind, toc(t1), fval(1), constrvioltext));
  cds_fitness_debug_plot_robot(R, Q0(1,:)', Traj_0_E, Traj_W, Set, Structure, p, mean(fval), debug_info);
  cds_save_particle_details(Set, R, toc(t1), fval, p, physval, constraint_obj_val, desopt_pval);
  save_generation(Set, Structure, i_gen);
  return
end
% Gelenkgrenzen merken (werden später überschrieben)
qlim = R.update_qlim(); % Nur Ausgabe, nichts in Klasse eintragen

%% Trajektorie auf Roboter-Basis umrechnen (wegen Basis-Verschiebung)
% Erst hier berechnen, da zeitaufwändig bei größeren Trajektorien
Traj_0 = cds_transform_traj(R, Traj_W);


%% Bestimme die Reihenfolge, in der die Konfigurationen geprüft werden
if Set.optimization.traj_ik_abort_on_success && size(Q0,1) > 1 && ...
    size(Q0,1) == size(Stats_constraints.minmaxcondJ,2) % sonst Reihenfolge mit q0_traj bereits gesetzt.
  % Reihenfolge anhand eines Leistungsmerkmals
  I_IKC = [];
  if any(R.MDH.sigma == 1)
    % Benutze Spannweite der Schubgelenkkoordinaten als erstes Kriterium
    % Bei PKM wird die Symmetrie bei Schubgelenken berücksichtigt.
    qrange_from_q0 = NaN(size(Q0,1),R.NJ);
    for i = 1:size(Q0,1)
      qrange_from_q0(i,:) = diff(update_joint_limits(R, Set, Q0(i,:), 1, 0, 1)');
    end
    qrange_from_q0(isinf(qrange_from_q0)) = NaN;
    if diff(minmax2(sum(qrange_from_q0(:,R.MDH.sigma == 1),2,'omitnan')')) > 1e-3 % Schubgelenk-Koordinaten unterscheiden sich
      [qrange_sort, I_IKC] = sort(sum(qrange_from_q0(:,R.MDH.sigma == 1),2,'omitnan')'); %#ok<TRSRT> 
      cds_log(3,sprintf(['[fitness] G=%d;I=%d. %d Konfig. anhand Schubgelenk ', ...
        'sortiert (Verfahrweg: %1.1f...%1.1f): [%s]'], i_gen, i_ind, size(Q0,1), ...
        min(qrange_sort)/R.NLEG, max(qrange_sort)/R.NLEG, disp_array(I_IKC, '%d')));      
    end
  end
  if isempty(I_IKC) && all(~isnan(Stats_constraints.minmaxcondJ(2,:))) && ...
      diff(minmax2(Stats_constraints.minmaxcondJ(2,:))) > 1e-3 % Konditionszahlen unterscheiden sich
    [maxcondJ_sort, I_IKC] = sort(Stats_constraints.minmaxcondJ(2,:));
    cds_log(3,sprintf(['[fitness] G=%d;I=%d. %d Konfig. anhand max. condJ ', ...
      'sortiert (%1.1f...%1.1f): [%s]'], i_gen, i_ind, size(Q0,1), ...
      min(maxcondJ_sort), max(maxcondJ_sort), disp_array(I_IKC, '%d')));
  end
  if isempty(I_IKC) && all(~isnan(Stats_constraints.bestcolldist)) && ...
      diff(minmax2(Stats_constraints.bestcolldist)) > 1e-3 % Abstände unterscheiden sich
    [bestcolldist_sort, I_IKC] = sort(Stats_constraints.bestcolldist);
    cds_log(3,sprintf(['[fitness] G=%d;I=%d. %d Konfig. anhand kleinstem ', ...
      'Kollisionsabstand sortiert (%1.1fmm ... %1.1fmm): [%s]'], i_gen, i_ind, ...
      size(Q0,1), 1e3*min(bestcolldist_sort), 1e3*max(bestcolldist_sort), disp_array(I_IKC, '%d')));
  end
  if isempty(I_IKC) && all(~isnan(Stats_constraints.minmaxcondJik(2,:))) && ...
      diff(minmax2(Stats_constraints.minmaxcondJik(2,:))) > 1e-3 % unterscheiden sich
    [maxcondJik_sort, I_IKC] = sort(Stats_constraints.minmaxcondJik(2,:));
    cds_log(3,sprintf(['[fitness] G=%d;I=%d. %d Konfig. anhand max. condJik ', ...
      'sortiert (%1.1f...%1.1f): [%s]'], i_gen, i_ind, size(Q0,1), ...
      min(maxcondJik_sort), max(maxcondJik_sort), disp_array(I_IKC, '%d')));
  end
  if isempty(I_IKC)
    cds_log(3,sprintf('[fitness] G=%d;I=%d. %d Konfig. Kein Sortierungskriterium vorab bestimmt.', ...
      i_gen, i_ind, size(Q0,1)));
    I_IKC = 1:size(Q0,1);
  end
else
  I_IKC = 1:size(Q0,1); % der Reihe nach durchgehen
end
%% Alle IK-Konfigurationen durchgehen
% Berechne jeweils alle Zielfunktionen. Bestimme erst danach, welcher
% Parametersatz der beste ist

fval_IKC = NaN(size(Q0,1), length(Set.optimization.objective));
physval_IKC = fval_IKC; 
constrvioltext_IKC = cell(size(Q0,1), 1);
constraint_obj_val_IKC = NaN(length(Set.optimization.constraint_obj),size(Q0,1));
fval_debugtext_IKC = constrvioltext_IKC;
Q_IKC = NaN(size(Traj_0.X,1), R.NJ, size(Q0,1));
QD_IKC = Q_IKC; QDD_IKC = Q_IKC;
if R.Type == 0
  JP_IKC = NaN(size(Traj_0.X,1), 3*(1+R.NJ+1), size(Q0,1));
else
  JP_IKC = NaN(size(Traj_0.X,1), 3*(1+R.NJ+R.NLEG+1+1), size(Q0,1));
end
Jinv_IKC = NaN(size(Traj_0.X,1), sum(R.I_EE)*R.NJ, size(Q0,1));
X6Traj_IKC = NaN(size(Traj_0.X,1), 3, size(Q0,1));
desopt_pval_IKC = repmat(desopt_pval(:)', size(Q0,1), 1); % NaN-initialisiert oder aus Eingabe-Argument.
if R.Type == 0, n_actjoint = R.NJ;
else,           n_actjoint = sum(R.I_qa); end
TAU_IKC = NaN(size(Traj_0.X,1), n_actjoint, size(Q0,1));

for iIKC = I_IKC
  %% Gelenkwinkel-Grenzen aktualisieren
  if ~all(abs(QE_iIKC(1,:,iIKC) - Q0(iIKC,:))<1e-8) % NaN gibt auch Fehler.
    save(fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
      'tmp', 'cds_fitness_q0_qE_error.mat'));
    error('Q0 und QE_iIKC passen nicht zusammen'); % Prüfe wegen Umsortierung oben
  end
  % Als Spannweite vorgegebene Gelenkgrenzen neu zentrieren. Benutze dafür
  % alle Eckpunkte aus der Einzelpunkt-IK
  if ~Set.optimization.fix_joint_limits && ~any(any(isnan(QE_iIKC(:,:,iIKC))))
    % Zunächst die Spannweite der Grenzen wieder zurücksetzen (wurde even-
    % tuell überschrieben für die vorherigen IK-Anfangswerte in iIKC-Schleife)
    R.update_qlim(qlim);
    % Erst danach die Gelenkgrenzen neu zentrieren für das aktuelle iIKC
    qlim_neu = update_joint_limits(R, Set, QE_iIKC(:,:,iIKC), false, 0);
  else
    qlim_neu = qlim; % Variable einheitlich definieren
  end
  if any(Q0(iIKC,:)' < qlim_neu(:,1) | Q0(iIKC,:)' > qlim_neu(:,2))
    cds_log(-1, '[cds_fitness] Anfangswert für Gelenkwinkel außerhalb der Grenzen. Für Traj. ungünstig.');
    try
      save(fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
        'tmp', 'cds_fitness_qlim_viol_q0.mat'));
    catch err % Dateisystem nicht immer stabil auf Cluster
      cds_log(-1, sprintf('[cds_fitness] %s', err.message));
    end
  end
  %% Trajektorie berechnen
  Structure.config_index = iIKC; % Index dieser Konfiguration (für Debuggen)
  Structure.config_number = size(Q0,1); % Anzahl der Konfigurationen insgesamt
  if Set.task.profile ~= 0 % Nur Berechnen, falls es eine Trajektorie gibt
    t0 = tic();
    try
      [fval_trajconstr,Q,QD,QDD,Jinv_ges,JP,constrvioltext_IKC{iIKC}, Traj_0_corr] = ...
        cds_constraints_traj(R, Traj_0, Q0(iIKC,:)', Set, Structure, Stats_constraints, p);
    catch err
      fval(:) = 1e9; % Größtmöglicher Wert für cds_constraints_traj
      dbgfile = fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
        'tmp', ['cds_fitness_call_cds_constraints_traj_error_', R.mdlname, '.mat']);
      cds_log(-1, sprintf(['[fitness] Fehler in cds_constraints_traj: %s. ' ...
        'Zustand gespeichert: %s. Setze fval=%1.1e\n%s'], err.message, dbgfile, fval(1), getReport(err, 'extended')));
      save(dbgfile);
      abort_fitnesscalc = true;
      return
    end
    cds_log(2,sprintf(['[fitness] G=%d;I=%d (Konfig %d/%d). Nebenbedingungen ', ...
      'für Trajektorie in %1.2fs geprüft. fval_constr=%1.3e. %s'], i_gen, ...
      i_ind, iIKC, size(Q0,1), toc(t0), fval_trajconstr, constrvioltext_IKC{iIKC}));
    % NB-Verletzung in Traj.-IK wird in Ausgabe mit Werten von 1e3 aufwärts
    % angegeben. Umwandlung in Werte von 1e7 aufwärts.
    % Ursache: Nachträgliches Einfügen von weiteren Nebenbedingungen.
    fval_IKC(iIKC,:) = 1e4*fval_trajconstr;
    % Speichere Offset als Ergebnis der Entwurfsoptimierung in cds_constraints_traj.
    if Structure.desopt_prismaticoffset
      if Structure.Type == 0, p_prismaticoffset = R.DesPar.joint_offset(R.MDH.sigma==1);
      else, p_prismaticoffset = R.Leg(1).DesPar.joint_offset(R.Leg(1).MDH.sigma==1);
      end
      desopt_pval_IKC(iIKC,Structure.desopt_ptypes==1) = p_prismaticoffset;
    end
    % Übernehme die korrigierte Trajektorie (für den Fall von 3T2R). Sonst
    % sind die folgenden Berechnungen nicht konsistent
    Traj_0 = Traj_0_corr;
  else
    Q = QE_iIKC(:,:,iIKC);
    QD = 0*Q; QDD = QD;
    fval_trajconstr = 0; % Alle Nebenbedingungen bereits in cds_constraints oben geprüft
    % Es liegt keine Trajektorie vor. Es reicht also, das Ergebnis der IK von
    % der Eckpunkt-Berechnung zu benutzen um die Jacobi-Matrix zu berechnen
    if R.Type == 0 % Seriell
      Jinv_ges = NaN; % Platzhalter
    else % Parallel
      Jinv_ges = NaN(size(Q,1), sum(R.I_EE)*size(Q,2));
      for i = 1:size(Q,1)
        if any(isnan(Q(i,:))); break; end
        % Berechne die korrigierten Werte für die Euler-Winkel nach
        if Structure.task_red || all(R.I_EE_Task == [1 1 1 1 1 0])
          x_i = R.fkineEE2_traj(Q(i,:))';
          x_i(1:5) = Traj_0.X(i,1:5);
          Traj_0.X(i,:) = x_i;
        end
        [~,J_x_inv] = R.jacobi_qa_x(Q(i,:)', Traj_0.X(i,:)');
        if any(isnan(J_x_inv(:))) || any(isinf(J_x_inv(:)))
          % Durch numerische Fehler können Inf- oder NaN-Einträge in der
          % Jacobi-Matrix entstehen (Singularität)
          if Set.general.matfile_verbosity > 2
            save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_fitness_J_infnan.mat'));
          end
          J_x_inv = zeros(size(J_x_inv));
        end
        Jinv_ges(i,:) = J_x_inv(:);
      end
    end
    % Bestimme auch die Gelenkpositionen nochmal mit der direkten Kinematik
    JP = NaN(size(JP_IKC(:,:,iIKC))); % wurde oben nicht belegt
    for i = 1:size(Q,1)
      if any(isnan(Q(i,:))); break; end
      [~, JointPos_all_i_fromdirkin] = R.fkine_coll2(Q(i,:)');
      JP(i,:) = JointPos_all_i_fromdirkin(:);
    end
  end
  Q_IKC(:,:,iIKC) = Q;
  QD_IKC(:,:,iIKC) = QD;
  QDD_IKC(:,:,iIKC) = QDD;
  JP_IKC(:,:,iIKC) = JP;
  if R.Type == 2
    Jinv_IKC(:,:,iIKC) = Jinv_ges;
  end
  X6Traj_IKC(:,:,iIKC) = [Traj_0.X(:,6),Traj_0.XD(:,6),Traj_0.XDD(:,6)];
  % Kein Normalisieren der Winkel (wenn dann erst hier durchführen, da
  % einige Prüfungen oben davon beeinflusst werden).
  % Falls Gelenksteifigkeiten vorgesehen sind springt das Federmoment.
  % Normalisiere nur den ersten Wert, falls dieser bereits jenseits pi ist.
  qoff_norm = Q(1,R.MDH.sigma==0)-wrapToPi(Q(1,R.MDH.sigma==0));
  if any(qoff_norm)
    Q(:,R.MDH.sigma==0) = Q(:,R.MDH.sigma==0) - repmat(qoff_norm,size(Q,1),1);
  end
  if R.Type == 0
    R.qref(R.MDH.sigma==0) = wrapToPi(R.qref(R.MDH.sigma==0));
  else
    for k = 1:R.NLEG
      R.Leg(k).qref(R.Leg(k).MDH.sigma==0) = wrapToPi(R.Leg(k).qref(R.Leg(k).MDH.sigma==0));
    end
  end

  if fval_trajconstr > 1000 % Nebenbedingungen verletzt.
    continue; % Nächste Anfangs-Konfiguration
  end
  % Prüfe Validität der Jacobi (nur für PKM)
  if any(isinf(Jinv_ges(:))) || Structure.Type~=0 && any(isnan(Jinv_ges(:)))
    save(fullfile(repopath, 'tmp', 'cds_fitness_J_infnan.mat'));
    error('Jacobi hat Inf oder NaN. Darf hier eigentlich nicht sein!');
  end
  % Prüfe Validität der Geschwindigkeiten
  if isempty(QD) || isempty(QDD)
    save(fullfile(repopath, 'tmp', 'cds_fitness_QD_empty.mat'));
    error('Geschwindigkeit ist nicht belegt. Darf hier eigentlich nicht sein!');
  end
  if Set.general.matfile_verbosity > 2
    save(fullfile(repopath, 'tmp', 'cds_fitness_2.mat'));
  end
  % Debug:
  % load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_fitness_2.mat'));
  %% Konditionszahl als Nebenbedingung prüfen
  if Set.optimization.constraint_obj(4) > 0 % NB für Kondition gesetzt
    [fval_cond,fval_debugtext_cond, debug_info_cond, Jcond] = cds_obj_condition(R, Set, Structure, Jinv_ges, Traj_0, Q, QD);
    constraint_obj_val_IKC(4,iIKC) = Jcond;
    if Jcond > Set.optimization.constraint_obj(4)
      fval_IKC(iIKC,:) = 1e6*(5+5*fval_cond/1e3); % normiert auf 5e6 bis 1e7
      % debug_info = {sprintf('Kondition %1.1e > %1.1e', Jcond, Set.optimization.constraint_obj(4)); debug_info_cond{1}};
      constrvioltext_IKC{iIKC} = sprintf(['Konditionszahl ist zu schlecht: ', ...
        '%1.1e > %1.1e'], Jcond, Set.optimization.constraint_obj(4));
      continue
    end
  end
  %% Positionsfehler als Nebenbedingung prüfen
  % Rein kinematische Nebenbedingung. Wird vor Dynamikberechnung bestimmt
  if Set.optimization.constraint_obj(7) > 0 % NB für Positionsfehler gesetzt
    [fval_pe, fval_debugtext_pe, debug_info_pe, physval_pe] = cds_obj_positionerror(R, Set, Jinv_ges, Q);
    constraint_obj_val_IKC(7,iIKC) = physval_pe;
    if physval_pe > Set.optimization.constraint_obj(7)
      fval_IKC(iIKC,:) = 1e6*(1+4*fval_pe/1e3); % normiert auf 1e6 bis 5e6
      constrvioltext_IKC{iIKC} = sprintf(['Positionsfehler ist zu groß: ', ...
        '%1.1eµm > %1.1eµm'], 1e6*physval_pe, 1e6*Set.optimization.constraint_obj(7));
      continue
    end
  end
  %% Dynamik-Parameter
  % Gelenkgrenzen für Schubgelenke in Roboterklasse neu eintragen.
  % Für Drehgelenke keine Aktualisierung notwendig (wird nicht benutzt).
  % Nutzen: Berechnung der Masse von Führungsschienen und Hubzylindern,
  % Anpassung der Kollisionskörper (für nachgelagerte Prüfung und Plots)
  % Bereits hier, damit Ergebnis-Visualisierung konsistent ist.
  update_joint_limits(R, Set, Q, true, 0);
  massparam_set = false; % Marker, ob Masseparameter gesetzt wurden
  if ~isempty(intersect(Set.optimization.objective, {'energy', 'power', 'mass', ...
      'actforce', 'stiffness', 'materialstress'})) || ... % Für Zielfunktion benötigt
      Set.optimization.constraint_obj(3) ~= 0 % Für Nebenbedingung benötigt
    % Dynamik-Parameter aktualisieren. Keine Nutzung der Ausgabe der Funktion
    % (Parameter werden direkt in Klasse geschrieben; R.DesPar.seg_par ist
    % vor/nach dem Aufruf unterschiedlich)
    if ~any(Structure.desopt_ptypes==2)
      % Keine Entwurfsoptimierung mit Segmentstärke. Daher hier die Massen
      % einmal mit Standard-Werten belegen.
      cds_dimsynth_design(R, Q, Set, Structure);
    elseif desopt_pval_given
      % Entwurfsoptimierung mit Segmentstärke eigentlich vorgesehen, aber
      % Ergebnis bereits vorgegeben (nachträgliche Auswertung)
      p_linkstrength = desopt_pval(Structure.desopt_ptypes==2);
      cds_dimsynth_design(R, Q, Set, Structure, p_linkstrength);
      if Set.optimization.constraint_collisions_desopt % muss konsistent mit cds_dimsynth_desopt_fitness sein
        Set.optimization.collision_bodies_size = p_linkstrength(2) + ...
          Set.optimization.collision_bodies_safety_distance * 2;
        Structure.collbodies_robot = cds_update_collbodies(R, Set, Structure, Q);
      end
    end
    if ~isempty(Set.optimization.desopt_vars) % Entwurfsoptimierung aktiv.
      % Berechne Dynamik-Funktionen als Regressorform für die Entwurfsopt.
      data_dyn = cds_obj_dependencies(R, Traj_0, Set, Structure, Q, QD, QDD, Jinv_ges);
      t0 = tic();
      [fval_desopt, desopt_pval, vartypes_desopt] = cds_dimsynth_desopt( ...
        R, Traj_0, Q, QD, QDD, JP, Jinv_ges, data_dyn, Set, Structure);
      cds_log(2,sprintf(['[fitness] G=%d;I=%d (Konfig %d/%d). Entwurfs', ...
        'optimierung in %1.2fs durchgeführt. fval_desopt=%1.3e. pval=[%s]'], ...
        i_gen, i_ind, iIKC, size(Q0,1), toc(t0), fval_desopt, disp_array(desopt_pval(:)', '%1.2g')));
      if fval_desopt > 1e5
        warning('Ein Funktionswert > 1e5 ist nicht für Entwurfsoptimierung vorgesehen');
      end
      if any(strcmp(Set.optimization.desopt_vars, 'linkstrength'))
        p_linkstrength = desopt_pval(vartypes_desopt==2);
        % Speichere die Parameter der Segmentstärke (jedes Segment gleich)
        desopt_pval_IKC(iIKC,Structure.desopt_ptypes==2) = p_linkstrength;
        if Set.optimization.constraint_collisions_desopt % muss konsistent mit cds_dimsynth_desopt_fitness sein
          Set.optimization.collision_bodies_size = p_linkstrength(2) + ...
            Set.optimization.collision_bodies_safety_distance * 2;
          Structure.collbodies_robot = cds_update_collbodies(R, Set, Structure, Q);
        end
      end
      if any(strcmp(Set.optimization.desopt_vars, 'joint_stiffness_qref'))
        % Speichere die Parameter der Gelenkfeder-Ruhelage (jede Beinkette
        % gleich). Siehe cds_dimsynth_desopt_fitness.
        desopt_pval_IKC(iIKC,Structure.desopt_ptypes==3) = desopt_pval(vartypes_desopt==3);
      end
      if any(strcmp(Set.optimization.desopt_vars, 'joint_stiffness'))
        % Speichere die Parameter der Gelenkfeder-Steifigkeit (jede Beinkette
        % gleich). Siehe cds_dimsynth_desopt_fitness.
        desopt_pval_IKC(iIKC,Structure.desopt_ptypes==4) = desopt_pval(vartypes_desopt==4);
      end
      if fval_desopt > 1000 % Nebenbedingungen in Entwurfsoptimierung verletzt.
        % Neue Werte (geändert gegenüber cds_dimsynth_desopt_fitness.)
        % 1e4...1e5: Nebenbedingung von Zielfunktion überschritten
        % 1e5...1e6: Überschreitung Belastungsgrenze der Segmente
        fval_IKC(iIKC,:) = 10*fval_desopt; % Wert ist im Bereich 1e3...1e5. Bringe in Bereich 1e4 bis 1e6
        cds_fitness_debug_plot_robot(R, zeros(R.NJ,1), Traj_0, Traj_W, Set, Structure, p, fval_IKC(iIKC,1), debug_info);
        constrvioltext_IKC{iIKC} = 'Verletzung der Nebenbedingungen in Entwurfsoptimierung';
        continue
      end
    end
    massparam_set = true;
  end
  if Set.general.matfile_verbosity > 2
    save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_fitness_3.mat'));
  end
  % Debug:
  % load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_fitness_3.mat'));

  %% Berechnungen für Zielfunktionen
  % Nullstellung der Gelenk-Steifigkeit einsetzen (Sonderfall für
  % Starrkörpergelenke und Gelenk-Federn)
  if R.Type ~= 0 && (Set.optimization.joint_stiffness_active_revolute ~= 0 || ...
      Set.optimization.joint_stiffness_passive_revolute ~= 0 || ...
      Set.optimization.joint_stiffness_passive_universal ~= 0) && ...
      ~any(strcmp(Set.optimization.desopt_vars, 'joint_stiffness_qref'))
    if desopt_pval_given && any(Structure.desopt_ptypes==3)
      % Ruhelage der Federn ist vorgegeben
      p_js_off = desopt_pval(Structure.desopt_ptypes==3);
      for i = 1:R.NLEG
        I_actrevolute_opt = R.Leg(1).MDH.mu ~= 1 & R.Leg(1).DesPar.joint_type==0 & ...
          Set.optimization.joint_stiffness_active_revolute ~= 0;
        I_passrevolute_opt = R.Leg(1).MDH.mu == 1 & R.Leg(1).DesPar.joint_type==0 & ...
          Set.optimization.joint_stiffness_passive_revolute ~= 0;
        I_passuniversal_opt = R.Leg(1).MDH.mu == 1 & R.Leg(1).DesPar.joint_type==2 & ...
          Set.optimization.joint_stiffness_passive_universal ~= 0;
        I_joints = I_actrevolute_opt|I_passrevolute_opt|I_passuniversal_opt;
        R.Leg(i).DesPar.joint_stiffness_qref(I_joints) = p_js_off;
      end
    else
      % Ruhelage der Feder ist Mittelstellung der Gelenk-Trajektorie (erzeugt
      % minimale Federmomente). Alle Beine sind symmetrisch. Nur hier ein-
      % setzen, falls nicht in cds_dimsynth_desopt bereits getan. Sehr an-
      % fällig, falls Konfigurationen in einer Beinkette umklappen. Sollte 
      % aber nicht passieren.
      qminmax_legs = reshape(minmax2(Q'),R.Leg(1).NJ,2*R.NLEG);
      for i = 1:R.NLEG
        R.Leg(i).DesPar.joint_stiffness_qref = mean(minmax2(qminmax_legs),2);
      end
    end
    if desopt_pval_given && any(Structure.desopt_ptypes==4)
      % Feder-Steifigkeit ist auch vorgegeben
      p_js = desopt_pval(Structure.desopt_ptypes==4);
      for i = 1:R.NLEG
        I_actrevolute_opt = R.Leg(i).MDH.mu ~= 1 & R.Leg(i).DesPar.joint_type==0 & ...
          isnan(Set.optimization.joint_stiffness_active_revolute);
        I_passrevolute_opt = R.Leg(i).MDH.mu == 1 & R.Leg(i).DesPar.joint_type==0 & ...
          isnan(Set.optimization.joint_stiffness_passive_revolute);
        I_passuniversal_opt = R.Leg(i).MDH.mu == 1 & R.Leg(i).DesPar.joint_type==2 & ...
          isnan(Set.optimization.joint_stiffness_passive_universal);
        I_joints = I_actrevolute_opt|I_passrevolute_opt|I_passuniversal_opt;
        R.Leg(i).DesPar.joint_stiffness(I_joints) = p_js;
      end
    end
  end
  if ~Structure.calc_dyn_reg && ~Structure.calc_spring_reg
    % Keine Regressorform. Nehme direkten Aufruf für Dynamikberechnung.
    data_dyn2 = cds_obj_dependencies(R, Traj_0, Set, Structure, Q, QD, QDD, Jinv_ges);
  else
    % Dynamik nochmal mit Regressorform mit neuen Dynamikparameter berechnen
    data_dyn2 = cds_obj_dependencies_regmult(R, data_dyn, Q);
  end
  if ~isempty(intersect(Set.optimization.objective, {'energy', 'power', 'actforce'})) || ...  % Für Zielf. benötigt
      Set.optimization.constraint_obj(3) ~= 0 % Für NB benötigt
    TAU = data_dyn2.TAU;
    TAU_IKC(:,:,iIKC) = TAU; % Zum Debuggen
  else
    TAU = [];
  end

  %% Nebenbedingungen der Entwurfsvariablen berechnen: Festigkeit der Segmente
  if Set.optimization.constraint_obj(6) > 0 && isempty(Set.optimization.desopt_vars)
    % Wenn desopt_vars gesetzt ist, wurde die Nebenbedingung bereits oben
    % geprüft und hier ist keine Berechnung notwendig.
    % Für den anderen Fall wird hier der gleiche Wertebereich genutzt (1e5..1e6)
    [fval_matstress, fval_debugtext_matstress, debug_info_materialstress, ...
      physval_materialstress] = cds_obj_materialstress(R, Set, data_dyn2, Jinv_ges, Q, Traj_0);
    if physval_materialstress > Set.optimization.constraint_obj(6)
      constrvioltext_IKC{iIKC} = fval_debugtext_matstress;
      fval_IKC(iIKC,:) = 100*fval_matstress; % Bringe von Bereich 1e2 bis 1e3 in Bereich 1e5 ... 1e6
      continue
    end
  end
  %% Nebenbedingungen der Entwurfsvariablen berechnen: Steifigkeit
  if Set.optimization.constraint_obj(5)
    [fval_st, ~, ~, fval_phys_st] = cds_obj_stiffness(R, Set, Q);
    constraint_obj_val_IKC(5,iIKC) = fval_phys_st;
    if fval_phys_st > Set.optimization.constraint_obj(5)
      % Nutze den gleichen Wertebereich wie Entwurfsoptimierung oben.
      fval_IKC(iIKC,:) = fval_st*100; % Bringe in Bereich 1e4 ... 1e5
      constrvioltext_IKC{iIKC} = sprintf('Die Nachgiebigkeit ist zu groß: %1.1e > %1.1e', ...
        fval_phys_st, Set.optimization.constraint_obj(5));
      cds_log(2,sprintf('[fitness] Fitness-Evaluation in %1.2fs. fval=%1.3e. %s', toc(t1), fval_IKC(iIKC,1), constrvioltext_stiffness));
      continue
    end
  end

  %% Antriebskraft als Nebenbedingung prüfen
  if Set.optimization.constraint_obj(3) > 0 % NB für Antriebskraft gesetzt
    [fval_actforce,fval_debugtext_actforce, debug_info_actforce, tau_a_max] = cds_obj_actforce(TAU);
    constraint_obj_val_IKC(3,iIKC) = tau_a_max;
    if tau_a_max > Set.optimization.constraint_obj(3)
      fval_IKC(iIKC,:) = 1e3*(1+9*fval_actforce/1e3); % normiert auf 1e3 bis 1e4
      debug_info = {sprintf('Antriebskraft %1.1e > %1.1e', tau_a_max, Set.optimization.constraint_obj(3))};
      cds_fitness_debug_plot_robot(R, Q(1,:)', Traj_0, Traj_W, Set, Structure, p, fval_IKC(iIKC,1), debug_info);
      constrvioltext_IKC{iIKC} = sprintf('Antriebskraft zu hoch: %1.1e > %1.1e', tau_a_max, Set.optimization.constraint_obj(3));
      continue
    end
  end
  %% Zielfunktion berechnen
  fval_debugtext = '';
  if any(strcmp(Set.optimization.objective, 'valid_act'))
    [fval_va,fval_debugtext_va, debug_info] = cds_obj_valid_act(R, Set, Jinv_ges);
    if fval_va < 1e3 % Wenn einmal der Freiheitsgrad festgestellt wurde, reicht das
      % Prüfe, wie oft schon dieses Ergebnis vorlag
      PSO_Detail_Data = cds_save_particle_details([], [], 0, 0, NaN, NaN, NaN, NaN, 'output');
      if ~isempty(PSO_Detail_Data)
        fval_hist = PSO_Detail_Data.fval(:,1,:);
        % Prüfe, wie oft bereits ein Rangdefizit erkannt wurde
        I_va_rd = fval_hist > 10 & fval_hist < 1e3;
      else
        I_va_rd = 0;
      end
      if sum(I_va_rd(:)) > 7 % mehrmals unabhängig voneinander feststellen (mit anderen Parametern). 
        % Annahme: Dann liegt es nicht an der Singularität einer
        % bestimmten Pose
        if ~abort_fitnesscalc % Meldung nur einmal zeigen
          cds_log(2,sprintf(['[fitness] Der PKM-Laufgrad mit Rangdefizit wurde ', ...
            ' mehrmals festgestellt. Hiernach Abbruch der Optimierung.']));
        end
        abort_fitnesscalc = true;
      elseif fval_va == 10 % Voller Laufgrad muss nur einmal erkannt werden
        if ~abort_fitnesscalc
          cds_log(2,sprintf(['[fitness] Der PKM-Laufgrad mit vollem Rang wurde ', ...
            ' festgestellt. Hiernach Abbruch der Optimierung.']));
        end
        abort_fitnesscalc = true;
      end
    end
    fval_IKC(iIKC,strcmp(Set.optimization.objective, 'valid_act')) = fval_va;
    physval_IKC(iIKC,strcmp(Set.optimization.objective, 'valid_act')) = NaN; % nicht definiert.
    fval_debugtext = [fval_debugtext, ' ', fval_debugtext_va]; %#ok<AGROW>
  end
  if any(strcmp(Set.optimization.objective, 'condition'))
    if Set.optimization.constraint_obj(4) == 0
      [fval_cond, fval_debugtext_cond, debug_info, constraint_obj_val_IKC(4,iIKC)] = cds_obj_condition(R, Set, Structure, Jinv_ges, Traj_0, Q, QD);
    else % Bereits oben berechnet. Keine Neuberechnung notwendig.
      debug_info = debug_info_cond;
    end
    fval_IKC(iIKC,strcmp(Set.optimization.objective, 'condition')) = fval_cond;
    physval_IKC(iIKC,strcmp(Set.optimization.objective, 'condition')) = constraint_obj_val_IKC(4,iIKC);
    fval_debugtext = [fval_debugtext, ' ', fval_debugtext_cond]; %#ok<AGROW>
  end
  if any(strcmp(Set.optimization.objective, 'chainlength'))
    [fval_cl, fval_debugtext_cl, ~, physval_cl] = cds_obj_chainlength(R);
    fval_IKC(iIKC,strcmp(Set.optimization.objective, 'chainlength')) = fval_cl;
    physval_IKC(iIKC,strcmp(Set.optimization.objective, 'chainlength')) = physval_cl;
    fval_debugtext = [fval_debugtext, ' ', fval_debugtext_cl]; %#ok<AGROW>
  end
  if any(strcmp(Set.optimization.objective, 'actvelo'))
    [fval_av, fval_debugtext_av, ~, physval_av] = cds_obj_actvelo(R, QD);
    fval_IKC(iIKC,strcmp(Set.optimization.objective, 'actvelo')) = fval_av;
    physval_IKC(iIKC,strcmp(Set.optimization.objective, 'actvelo')) = physval_av;
    fval_debugtext = [fval_debugtext, ' ', fval_debugtext_av]; %#ok<AGROW>
  end
  if any(strcmp(Set.optimization.objective, 'installspace'))
    [fval_instspc, fval_debugtext_instspc, ~, physval_instspc] = cds_obj_installspace(R, Set, Structure, Traj_0, Q, JP);
    fval_IKC(iIKC,strcmp(Set.optimization.objective, 'installspace')) = fval_instspc;
    physval_IKC(iIKC,strcmp(Set.optimization.objective, 'installspace')) = physval_instspc;
    fval_debugtext = [fval_debugtext, ' ', fval_debugtext_instspc]; %#ok<AGROW>
  end
  if any(strcmp(Set.optimization.objective, 'footprint'))
    [fval_footprint, fval_debugtext_footprint, ~, physval_footprint] = cds_obj_footprint(R, Set, Structure, Traj_0, Q, JP);
    fval_IKC(iIKC,strcmp(Set.optimization.objective, 'footprint')) = fval_footprint;
    physval_IKC(iIKC,strcmp(Set.optimization.objective, 'footprint')) = physval_footprint;
    fval_debugtext = [fval_debugtext, ' ', fval_debugtext_footprint]; %#ok<AGROW>
  end
  if any(strcmp(Set.optimization.objective, 'colldist'))
    [fval_colldist, fval_debugtext_colldist, ~, physval_colldist] = cds_obj_colldist(R, Set, Structure, Traj_0, Q, JP);
    fval_IKC(iIKC,strcmp(Set.optimization.objective, 'colldist')) = fval_colldist;
    physval_IKC(iIKC,strcmp(Set.optimization.objective, 'colldist')) = physval_colldist;
    fval_debugtext = [fval_debugtext, ' ', fval_debugtext_colldist]; %#ok<AGROW>
  end
  if any(strcmp(Set.optimization.objective, 'energy'))
    [fval_en,fval_debugtext_en, debug_info, physval_en] = cds_obj_energy(R, Set, Structure, Traj_0, TAU, QD);
    fval_IKC(iIKC,strcmp(Set.optimization.objective, 'energy')) = fval_en;
    physval_IKC(iIKC,strcmp(Set.optimization.objective, 'energy')) = physval_en;
    fval_debugtext = [fval_debugtext, ' ', fval_debugtext_en]; %#ok<AGROW>
  end
  if any(strcmp(Set.optimization.objective, 'power'))
    [fval_pwr,fval_debugtext_pwr, ~, physval_pwr] = cds_obj_power(R, Set, TAU, QD);
    fval_IKC(iIKC,strcmp(Set.optimization.objective, 'power')) = fval_pwr;
    physval_IKC(iIKC,strcmp(Set.optimization.objective, 'power')) = physval_pwr;
    fval_debugtext = [fval_debugtext, ' ', fval_debugtext_pwr]; %#ok<AGROW>
  end
  if massparam_set
    % Berechne in jedem Fall die Gesamtmasse, sobald das möglich ist (geht sehr 
    % schnell und hilft im Entscheidungsfall am Ende dieser Funktion)
    [fval_m,fval_debugtext_m, debug_info, constraint_obj_val_IKC(1,iIKC)] = cds_obj_mass(R);
  end
  if any(strcmp(Set.optimization.objective, 'mass'))
    fval_IKC(iIKC,strcmp(Set.optimization.objective, 'mass')) = fval_m;
    physval_IKC(iIKC,strcmp(Set.optimization.objective, 'mass')) = constraint_obj_val_IKC(1,iIKC);
    fval_debugtext = [fval_debugtext, ' ', fval_debugtext_m]; %#ok<AGROW>
  end
  if any(strcmp(Set.optimization.objective, 'actforce'))
    if Set.optimization.constraint_obj(3) == 0
      [fval_actforce,fval_debugtext_actforce, debug_info, tau_a_max] = cds_obj_actforce(TAU);
      constraint_obj_val_IKC(3,iIKC) = tau_a_max;
    else % Bereits oben berechnet.
      debug_info = debug_info_actforce;
    end
    fval_IKC(iIKC,strcmp(Set.optimization.objective, 'actforce')) = fval_actforce;
    physval_IKC(iIKC,strcmp(Set.optimization.objective, 'actforce')) = tau_a_max;
    fval_debugtext = [fval_debugtext, ' ', fval_debugtext_actforce]; %#ok<AGROW>
  end
  if any(strcmp(Set.optimization.objective, 'materialstress'))
    if Set.optimization.constraint_obj(6) == 0 || ... % Nicht als NB berechnet, also hier nochmal zu berechnen.
        ~isempty(Set.optimization.desopt_vars) % Es gab eine Entwurfsoptimierung. Die Zielfunktion, die dort berechnet wurde, ist hier aber nicht verfügbar.
      [fval_matstress,fval_debugtext_matstress, debug_info_materialstress, ...
        physval_materialstress] = cds_obj_materialstress(R, Set, data_dyn2, Jinv_ges, Q, Traj_0);
      constraint_obj_val_IKC(6,iIKC) = physval_materialstress;
    else % Bereits oben berechnet.
      debug_info = debug_info_materialstress;
    end
    fval_IKC(iIKC,strcmp(Set.optimization.objective, 'materialstress')) = fval_matstress;
    physval_IKC(iIKC,strcmp(Set.optimization.objective, 'materialstress')) = physval_materialstress;
    fval_debugtext = [fval_debugtext, ' ', fval_debugtext_matstress]; %#ok<AGROW>
  end
  if any(strcmp(Set.optimization.objective, 'jointrange'))
    [fval_jr,fval_debugtext_jr, debug_info, physval_jr] = cds_obj_jointrange(R, Set, Structure, Q);
    fval_IKC(iIKC,strcmp(Set.optimization.objective, 'jointrange')) = fval_jr;
    physval_IKC(iIKC,strcmp(Set.optimization.objective, 'jointrange')) = physval_jr;
    fval_debugtext = [fval_debugtext, ' ', fval_debugtext_jr]; %#ok<AGROW>
  end
  if any(strcmp(Set.optimization.objective, 'jointlimit'))
    [fval_jl,fval_debugtext_jl, debug_info, physval_jl] = cds_obj_jointlimit(R, Set, Structure, Q);
    fval_IKC(iIKC,strcmp(Set.optimization.objective, 'jointlimit')) = fval_jl;
    physval_IKC(iIKC,strcmp(Set.optimization.objective, 'jointlimit')) = physval_jl;
    fval_debugtext = [fval_debugtext, ' ', fval_debugtext_jl]; %#ok<AGROW>
  end
  if any(strcmp(Set.optimization.objective, 'stiffness'))
    [fval_st,fval_debugtext_st, debug_info, physval_st] = cds_obj_stiffness(R, Set, Q);
    fval_IKC(iIKC,strcmp(Set.optimization.objective, 'stiffness')) = fval_st;
    physval_IKC(iIKC,strcmp(Set.optimization.objective, 'stiffness')) = physval_st;
    fval_debugtext = [fval_debugtext, ' ', fval_debugtext_st]; %#ok<AGROW>
  end
  if any(strcmp(Set.optimization.objective, 'manipulability'))
    [fval_ma, fval_debugtext_ma, debug_info, physval_ma] = cds_obj_manipulability(R, Set, Jinv_ges, Traj_0, Q);
    fval_IKC(iIKC,strcmp(Set.optimization.objective, 'manipulability')) = fval_ma;
    physval_IKC(iIKC,strcmp(Set.optimization.objective, 'manipulability')) = physval_ma;
    fval_debugtext = [fval_debugtext, ' ', fval_debugtext_ma]; %#ok<AGROW>
  end
  if any(strcmp(Set.optimization.objective, 'minjacsingval'))
    [fval_msv, fval_debugtext_msv, debug_info, physval_msv] = cds_obj_minjacsingval(R, Set, Jinv_ges, Traj_0, Q);
    fval_IKC(iIKC,strcmp(Set.optimization.objective, 'minjacsingval')) = fval_msv;
    physval_IKC(iIKC,strcmp(Set.optimization.objective, 'minjacsingval')) = physval_msv;
    fval_debugtext = [fval_debugtext, ' ', fval_debugtext_msv]; %#ok<AGROW>
  end
  if any(strcmp(Set.optimization.objective, 'positionerror'))
    if Set.optimization.constraint_obj(7) == 0
      [fval_pe, fval_debugtext_pe, debug_info, physval_pe] = cds_obj_positionerror(R, Set, Jinv_ges, Q);
    else % Bereits oben berechnet. Keine Neuberechnung notwendig.
      debug_info = debug_info_pe;
    end
    fval_IKC(iIKC,strcmp(Set.optimization.objective, 'positionerror')) = fval_pe;
    physval_IKC(iIKC,strcmp(Set.optimization.objective, 'positionerror')) = physval_pe;
    fval_debugtext = [fval_debugtext, ' ', fval_debugtext_pe]; %#ok<AGROW>
  end
  fval_debugtext_IKC{iIKC} = fval_debugtext;
  if any(fval_IKC(iIKC,:)>1e3)
    save(fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
      'tmp', 'cds_fitness_obj_notdef_error_debug2.mat'));
    error('Zielfunktion "%s" nicht definiert', Set.optimization.objective{fval_IKC(iIKC,:)>1e3});
  end
  if all(fval_IKC(iIKC,:)' <= Set.optimization.obj_limit) || ...
     all(physval_IKC(iIKC,:)' <= Set.optimization.obj_limit_physval)
    if ~abort_fitnesscalc % Folgende Meldung nur einmal anzeigen.
      cds_log(2,sprintf(['[fitness] Geforderte Grenze der Zielfunktion erreicht. ', ...
        'Abbruch der Optimierung.']));
    end
    abort_fitnesscalc = true;
    % Wenn eine IK-Konfiguration erfolgreich berechnet wird, sofort
    % abbrechen, wenn dies das Ziel der Optimierung ist.
    if all(Set.optimization.obj_limit == 1e3) && ... % keine konkrete Vorgabe, hauptsache i.O.
        all(Set.optimization.obj_limit_physval==0) % keine Vorgabe
      fval_IKC(isnan(fval_IKC)) = inf; % Sonst unten Fehler bei Bestimmung der besten Konfiguration wegen NaN
      break;
    end
  end
  if Set.optimization.traj_ik_abort_on_success && all(fval_IKC(iIKC,:) < 1e3)
    cds_log(2,sprintf('[fitness] i.O. Konfiguration gefunden. Ignoriere andere.'));
    fval_IKC(isnan(fval_IKC)) = inf;
    break;
  end
end % Schleife über IK-Konfigurationen
if Set.general.matfile_verbosity > 2
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_fitness_4.mat'));
end
%% Bestes Ergebnis heraussuchen
I_IKC_iO = find(all(fval_IKC <= 1e3, 2)); % i.O.-Partikel (für Auswertungen)
if length(Set.optimization.objective) == 1
  fval_min = min(fval_IKC);
  iIKCopt = find(fval_IKC==fval_min); % es kann mehrere gleich gute Lösungen geben
elseif size(fval_IKC,1) == 1
  iIKCopt = 1; % Wenn es nur eine Lösung gibt, ist keine Pareto-Prüfung notwendig
else
  % Prüfe, welche Ergebnisse Pareto-optimal sind
  % Nehme alle Kriterien als Differenz zum jeweils besten. Dadurch ist noch
  % ein Runden möglich. Exakte Gleichheit tritt aufgrund der numerischen IK
  % nicht auf. Identische Zielfunktionswerte sind bei kinematischen Größen
  % möglich. Dadurch ist weiter unten eine weitere Auswahl anhand anderer
  % Kriterien möglich.
  fval_IKC_check = fval_IKC; % Kleinster Spaltenwert ist jeweils Null.
  for mm = 1:size(fval_IKC,2) % Kriterien durchgehen
    fval_IKC_check(:,mm) = fval_IKC_check(:,mm) - min(fval_IKC(:,mm));
  end
  fval_IKC_check(abs(fval_IKC_check)<1e-10) = 0; % Runde kleine Differenzen auf Gleichheit
  % Wähle Kriterien zur Prüfung der Optimalität, müssen nicht alle sein
  if ~isempty(Set.optimization.criteria_config_selection)
    I_check = false(1,length(Set.optimization.objective));
    for kk = 1:length(Set.optimization.objective)
      I_check(kk) = any(strcmp(Set.optimization.objective{kk}, Set.optimization.criteria_config_selection));
    end
  else % Alle Optimierungskriterien berücksichtigen zur Auswahl
    I_check = true(1,length(Set.optimization.objective));
  end
  % Prüfung auf Pareto-Optimalität der Lösungen
  dom_vector = pareto_dominance(fval_IKC_check(:,I_check));
  % Aus Menge der möglichen Pareto-optimalen Lösungen auswählen.
  iIKCopt = find(~dom_vector);
end
% Definition "optimaler" Lösungen: Die beste/besten Lösungen.
% "Beste" Lösung: Nach bestimmten Kriterien aus mehreren Optimalen gewählt
n_fval_opt = length(iIKCopt);
if length(iIKCopt) == 1
  iIKCbest = iIKCopt(1);
else
  % Es gibt mehrere gleichwertige "optimale" Lösungen (einkriteriell) oder
  % Pareto-Optimale Lösungen. Wähle anhand eines weiteren Kriteriums aus.
  % Die gewählte Lösung ist dann die "beste" Lösung.
  I_best_opt = 0; % Index zur Auswahl in iIKCopt
  % Nehme die Lösung mit dem geringsten Offset von Führungsschienen. Zu
  % lange Offsets wirken unnatürlich (falls Zielkriterium darauf abzielt)
  if Structure.desopt_prismaticoffset && ...
      any(strcmp(Set.optimization.objective, 'chainlength'))
    p_prismaticoffset_IKCopt = desopt_pval_IKC(iIKCopt,Structure.desopt_ptypes==1);
    if ~all(abs(p_prismaticoffset_IKCopt-p_prismaticoffset_IKCopt(1))<1e-6)
      % Die optimalen Lösungen unterscheiden sich im Wert für das Kriterium
      % (sonst wird das Kriterium nicht zur Auswahl verwendet).
      [~,I_best_opt] = min(abs(p_prismaticoffset_IKCopt));
    end
  end
  % Nehme die Lösung mit der besten Konditionszahl. Toleranz gegen
  % Rundungsfehler; sonst beliebige Wahl quasi identischer Lösungen.
  if I_best_opt==0
    Jcond_IKC = constraint_obj_val_IKC(4,:)';
    Jcond_IKC_check = Jcond_IKC - Jcond_IKC(iIKCopt(1));
    Jcond_IKC_check(abs(Jcond_IKC_check)<1e-10) = 0;
    if all(~isnan(Jcond_IKC(iIKCopt))) && any(Jcond_IKC_check(iIKCopt))
      [~,I_best_opt] = min(Jcond_IKC_check(iIKCopt));
    end
  end
  % Nehme den Roboter mit der geringsten (bewegten) Masse. Kann unterschied-
  % lich sein, wenn Schubgelenke verschieden lang sind. Entspricht obiger
  % Prüfung für desopt_prismaticoffset, aber nur aktiv bei Dynamik-Kennzahl
  mges_IKC = constraint_obj_val_IKC(1,:)';
  mges_IKC_check = mges_IKC - mges_IKC(iIKCopt(1));
  mges_IKC_check(abs(mges_IKC_check)<1e-10) = 0;
  if all(~isnan(mges_IKC_check(iIKCopt))) && any(mges_IKC_check(iIKCopt))
    [~,I_best_opt] = min(mges_IKC_check(iIKCopt));
  end
  % Wenn es Schubgelenke gibt, nehme die Möglichkeit mit der geringsten
  % Auslenkung. Das ist am ehesten plausibel. Meistens betrifft es das
  % gestellfeste Gelenk.
  if I_best_opt==0 && any(R.MDH.sigma==1)
    qP_value_opt = NaN(length(iIKCopt), 1);
    for j = 1:length(iIKCopt)
      qP_value_opt(j) = max(max(abs(Q_IKC(:,R.MDH.sigma==1,iIKCopt(j)))));
    end
    [~,I_best_opt] = min(qP_value_opt); % Wähle die "beste" der "optimalen" Lösungen
  end
  if I_best_opt==0 % keine weiteren Kriterien definiert.
    I_best_opt = 1;% Nehme das erste (beliebig);
  end
  iIKCbest = iIKCopt(I_best_opt);
  if false % Debug: Prüfe, warum die eine Lösung besser als die andere ist
    % Passe Schubgelenk-Offset für den Plot an
    if Structure.desopt_prismaticoffset %#ok<UNRCH>
      if Structure.Type == 0
          R.DesPar.joint_offset(R.MDH.sigma==1) = desopt_pval_IKC(iIKCbest,Structure.desopt_ptypes==1);
        else
          for i = 1:R.NLEG
            R.Leg(i).DesPar.joint_offset(R.Leg(i).MDH.sigma==1) = desopt_pval_IKC(iIKCbest,Structure.desopt_ptypes==1);
          end
      end
    end
    % Passe Schubgelenk-Grenzen für den Plot an
    update_joint_limits(R, Set, Q_IKC(:,:, iIKCbest), true, 0);
    % Zeige den Roboter für die gewählte Konfiguration
    cds_fitness_debug_plot_robot(R, Q_IKC(1,:,iIKCbest)', Traj_0, ...
      Traj_W, Set, Structure, p, mean(fval_IKC(iIKCbest,:)), debug_info);
    % Weitere Debug-Plots
    figure(333);clf;
    for i = 1:size(TAU_IKC,2)
      for k = 1:length(I_IKC_iO)
        [~,I_max] = max(abs(TAU_IKC(:,i,I_IKC_iO(k))));
        subplot(size(TAU_IKC,2),1,i); hold on;
        plot(Traj_0.t, TAU_IKC(:,i,I_IKC_iO(I_IKC_iO(k))));
        plot(Traj_0.t(I_max), TAU_IKC(I_max,i,k), 'ko');
      end
      ylabel(sprintf('tau %d', i)); grid on;
    end
    linkxaxes
    figure(334);clf;
    for i = 1:R.NJ
      for k = 1:length(I_IKC_iO)
        subplot(ceil(sqrt(R.NJ)), ceil(R.NJ/ceil(sqrt(R.NJ))), i); hold on;
        plot(Traj_0.t, Q_IKC(:,i,I_IKC_iO(k)));
      end
      ylabel(sprintf('q %d', i)); grid on;
    end
    linkxaxes
  end
end
fval = fval_IKC(iIKCbest,:)';
physval = physval_IKC(iIKCbest,:)';
constraint_obj_val = constraint_obj_val_IKC(:,iIKCbest);
desopt_pval = desopt_pval_IKC(iIKCbest,:)';
n_fval_iO = length(I_IKC_iO);
Q_out = Q_IKC(:,:,iIKCbest);
QD_out = QD_IKC(:,:,iIKCbest);
QDD_out = QDD_IKC(:,:,iIKCbest);
TAU_out = TAU_IKC(:,:,iIKCbest);
JP_out = JP_IKC(:,:,iIKCbest);
if R.Type == 2
  Jinv_out = Jinv_IKC(:,:,iIKCbest);
end
X6Traj_out = X6Traj_IKC(:,:,iIKCbest);
%% Abbruchbedingung aufgrund von Singularität prüfen
if ~isinf(Set.optimization.condition_limit_sing) && R.Type == 2 && ...
    all(fval > 1e4*5e4) && all(fval < 1e4*6e4) % Grenzen für PKM-Singularität
  % Prüfe, wie oft schon dieses Ergebnis vorlag
  PSO_Detail_Data = cds_save_particle_details([], [], 0, 0, NaN, NaN, NaN, NaN, 'output');
  if ~isempty(PSO_Detail_Data)
    % Bei mehrkriterieller Optimierung bei NB-Verletzung gleiche Einträge 
    fval_hist = PSO_Detail_Data.fval(:,1,:);
    I_sing = fval_hist > 1e4*5e4 & fval_hist < 1e4*6e4;
    I_nonsing = fval_hist < 1e4*5e4;
    if sum(I_sing(:)) > 4 && sum(I_nonsing(:)) == 0
      if ~abort_fitnesscalc % Meldung nur einmal zeigen
        cds_log(2,sprintf(['[fitness] Es gab %d singuläre Ergebnisse und kein ', ...
          'nicht-singuläres. PKM nicht geeignet. Abbruch der Optimierung.'], sum(I_sing(:))));
      end
      abort_fitnesscalc = true;
    end
  end
end
%% Abbruchbedingung aufgrund von parasitärer Bewegung prüfen
% Eine Struktursynthese wird durchgeführt und die PKM ist zwar beweglich,
% aber in ungewünschte Richtungen. Annahme: PKM ist strukturell nicht
% änderbar durch Kinematik-Parameter
if any(strcmp(Set.optimization.objective, 'valid_act')) && R.Type == 2 && ...
    all(fval > 1e4*9e3) && all(fval < 1e4*1e4) % Grenzen für parasitäre Bewegung
  % Prüfe, wie oft schon dieses Ergebnis vorlag
  PSO_Detail_Data = cds_save_particle_details([], [], 0, 0, NaN, NaN, NaN, NaN, 'output');
  if ~isempty(PSO_Detail_Data)
    % Bei mehrkriterieller Optimierung bei NB-Verletzung gleiche Einträge 
    fval_hist = PSO_Detail_Data.fval(:,1,:);
    I_para = fval_hist > 1e4*9e3 & fval_hist < 1e4*1e4;
    I_nonpara = fval_hist < 1e4*9e3; % erfolgreicherere Berechnung (besser)
    if sum(I_para(:)) > 4 && sum(I_nonpara(:)) == 0
      if ~abort_fitnesscalc % Meldung nur einmal zeigen
        cds_log(2,sprintf(['[fitness] Es gab %d Ergebnisse mit parasitärer ', ...
          'Bewegung und keins ohne. PKM nicht geeignet. Abbruch der Optimierung.'], sum(I_para(:))));
      end
      abort_fitnesscalc = true;
    end
  end
end
%% Abbruchbedingung aufgrund inaktiver Gelenke prüfen
if Set.structures.no_inactive_joints && all(fval==1e4*1e4)
  % Prüfe, wie oft schon dieses Ergebnis vorlag
  PSO_Detail_Data = cds_save_particle_details([], [], 0, 0, NaN, NaN, NaN, NaN, 'output');
  if ~isempty(PSO_Detail_Data)
    % Bei mehrkriterieller Optimierung bei NB-Verletzung gleiche Einträge 
    fval_hist = PSO_Detail_Data.fval(:,1,:);
    I_inact = fval_hist == 1e4*1e4;
    I_noninact = fval_hist < 1e4*1e4; % erfolgreicherere Berechnung (besser)
    if sum(I_inact(:)) > 4 && sum(I_noninact(:)) == 0
      if ~abort_fitnesscalc % Meldung nur einmal zeigen
        cds_log(2,sprintf(['[fitness] Es gab %d Ergebnisse mit inaktiven Gelenken ', ...
          'und keins ohne. Roboter nicht geeignet. Abbruch der Optimierung.'], sum(I_inact(:))));
      end
      abort_fitnesscalc = true;
    end
  end
end
%% Ende
% Anfangs-Gelenkwinkel in Roboter-Klasse speichern (zur Reproduktion der
% Ergebnisse)
R.update_qref(Q0(iIKCbest,:)');
% Erneutes Eintragen der Gelenkgrenzen des gewählten besten Ergebnisses.
% Grenzen numerisch etwas erweitern. Dadurch kein Fehlschlag, falls
% rundungsbedingte Abweichungen auftreten. Ansonsten dadurch Verletzung
% der Grenzen möglich.
if ~Set.optimization.fix_joint_limits
  update_joint_limits(R, Set, Q_IKC(:,:, iIKCbest), true, 1e-6);
end
% Trage Parameter der Entwurfsoptimierung neu in Klasse ein. Falls der
% letzte Aufruf von constraints_traj nicht der beste war, ist sonst der
% falsche Wert gespeichert. Ist hilfreich, wenn die Fitness-Funktion
% aufgerufen wird, um die Parameter des Roboters zu aktualisieren. Für Maß-
% synthese selbst nicht unbedingt notwendig.
if Structure.desopt_prismaticoffset
  p_prismaticoffset = desopt_pval(Structure.desopt_ptypes==1);
  if Structure.Type == 0
    R.DesPar.joint_offset(R.MDH.sigma==1) = p_prismaticoffset;
  else
    for k = 1:R.NLEG
      R.Leg(k).DesPar.joint_offset(R.Leg(k).MDH.sigma==1) = p_prismaticoffset;
    end
  end
end
cds_fitness_debug_plot_robot(R, Q_IKC(1,:, iIKCbest)', Traj_0, Traj_W, Set, Structure, p, mean(fval), debug_info);
cds_save_particle_details(Set, R, toc(t1), fval, p, physval, constraint_obj_val, desopt_pval);
if all(fval<=1e3)
  if length(fval)>1, fvalstr=['[',disp_array(fval', '%1.3e'),']'];
  else,              fvalstr=sprintf('%1.3e', fval); end
  cds_log(2,sprintf(['[fitness] G=%d;I=%d. Fitness-Evaluation in %1.2fs. fval=%s. Erfolg', ...
    'reich. %s Auswahl aus %d IK-Konfigurationen (davon %d i.O., %d optimal)'], ...
    i_gen, i_ind, toc(t1), fvalstr, fval_debugtext_IKC{iIKCbest}(2:end), ...
    size(fval_IKC,1), n_fval_iO, n_fval_opt));
else
  cds_log(2,sprintf('[fitness] G=%d;I=%d. Fitness-Evaluation in %1.2fs. fval=%1.3e. %s', ...
    i_gen, i_ind, toc(t1), fval(1), constrvioltext_IKC{iIKCbest}));
end
rng('shuffle'); % damit Zufallszahlen in anderen Funktionen zufällig bleiben
save_generation(Set, Structure, i_gen);
end % function

function qlim_neu = update_joint_limits(R, Set, Q, limit_pris_to_Q, tol, dryrun)
% Funktion zum Aktualisieren der Gelenkgrenzen in der Matlab-Klasse.
% Schreibt die Min-/Max-Werte der Schubgelenk-Koordinaten von Q als Grenze.
% Drehgelenkgrenzen werden mit dem ursprünglichen Bereich neu zentriert
% Eingabe:
% R: Matlab-Klasse (Zeiger, wird auch geändert)
% Set: Einstellungen
% Q: Gelenk-Koordinaten
% limit_pris_to_Q: Falls true: Nur Schubgelenke aktualisieren
% tol: Toleranz, um die die Grenzen zusätzlich aufgeweitet werden
% dryrun: Bei true wird die Klassenvariable nicht geschrieben
if nargin < 5
  tol = 0;
end
if nargin < 6
  dryrun = 0;
end
if Set.optimization.fix_joint_limits
  % Bei Roboter-Modellen mit vorgegebenen Gelenkgrenzen wird nichts gemacht
  return
end
I_valid = all(~isnan(Q),2);
if ~any(I_valid), return; end % keine Daten übergeben. Aktualisiere nichts.
qlim = R.update_qlim(); % Nur Auslesen, nicht in Klasse schreiben
% Verschiebe die Winkelgrenzen
qlim_range = qlim(:,2) - qlim(:,1);
qlim_neu = qlim; % Für Dreh- und Schubgelenke separat. Berücksichtige 2pi-Periodizität
% Naiver Mittelwert: Für Schubgelenke korrekt
qlim_neu(R.MDH.sigma==1,:) = repmat(mean(Q(:,R.MDH.sigma==1),1)',1,2)+...
  [-qlim_range(R.MDH.sigma==1), qlim_range(R.MDH.sigma==1)]/2;
% Mittelwert der Winkel, zunächst normalisiert um pi
qErot_mean = meanangle(Q(:,R.MDH.sigma==0),1)';
% Zentrieren um ersten Schritt. Sonst kann q0 außerhalb liegen, obwohl
% es eigentlich der korrekte Bereich ist.
qErot_meannorm = normalizeAngle(qErot_mean, Q(1,R.MDH.sigma==0)');
% Einträge für Drehgelenke überschreiben
qlim_neu(R.MDH.sigma==0,:) = repmat(qErot_meannorm,1,2)+...
  [-qlim_range(R.MDH.sigma==0), qlim_range(R.MDH.sigma==0)]/2;
% Verschiebe die Grenzen nochmals, falls die ersten Winkel nicht drin sind
Q_delta_ll = qlim_neu(:,1) - Q(1,:)';
if any(Q_delta_ll > 0) % untere Grenze wird verletzt
  qlim_neu(Q_delta_ll>0) = qlim_neu(Q_delta_ll>0) - Q_delta_ll(Q_delta_ll>0) - ...
    qlim_range(Q_delta_ll>0) * 0.02; % Sicherheitsabstand
end
Q_delta_ul = qlim_neu(:,2) - Q(1,:)';
if any(Q_delta_ul < 0) % untere Grenze wird verletzt
  qlim_neu(Q_delta_ul<0) = qlim_neu(Q_delta_ul<0) - Q_delta_ul(Q_delta_ul<0) - ...
    qlim_range(Q_delta_ul<0) * 0.02; % Sicherheitsabstand
end
% Schubgelenke auf die minimal nötige Grenze reduzieren (für Animationen)
if limit_pris_to_Q
  if R.Type == 0 % Seriell
    % Schubgelenk-Grenzen so setzen, dass Bewegung gerade so möglich ist
    I = R.MDH.sigma==1;
    qlim_neu(I,:) = minmax2(Q(I_valid,I)') + tol*repmat([-1, 1], sum(I), 1);
  else % PKM
    % Grenzen bei symmetrischer Wahl
    if Set.optimization.joint_limits_symmetric_prismatic
      qminmax_legs = reshape(minmax2(Q(I_valid,:)'),R.Leg(1).NJ,2*R.NLEG);
      qminmax_leg = minmax2(qminmax_legs);
    end
    for i = 1:R.NLEG
      I = R.Leg(i).MDH.sigma==1;
      if ~Set.optimization.joint_limits_symmetric_prismatic
        Q_i = Q(:,R.I1J_LEG(i):R.I2J_LEG(i));
        qminmax_legI = minmax2(Q_i(:,I)');
      else
        qminmax_legI = qminmax_leg(I,:);
      end
      qlim_i = R.Leg(i).qlim;
      qlim_i(I,:) = qminmax_legI + tol*repmat([-1, 1], sum(I), 1);
      qlim_neu(R.I1J_LEG(i):R.I2J_LEG(i),:) = qlim_i;
    end
  end
end
if all(isnan(qlim_neu(:)))
  cds_log(-1, '[fitness/update_joint_limits] qlim_neu ist NaN. Darf nicht sein.')
  save(fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
    'tmp', 'cds_fitness_qlimnan_error_debug2.mat'));
end
if ~dryrun
  R.update_qlim(qlim_neu);
end
end % function

function save_generation(Set, Structure, i_gen)
% Speichere den aktuellen Zwischenstand der Optimierung.
% Zusätzlich zur generationsweisen Speicherung durch den Opt.-Algorithmus.
% Prüfe, ob die Optimierung bald abgebrochen wird und ob die Generation vor-
% zeitig gespeichert werden sollte. Hilfreich, falls eine Generation lange
% dauert und sonst zu viele Daten verloren gehen würden bei Cluster-Timeout.
persistent t_lastsave; % Zeitpunkt des letzten Speicherns hierüber
persistent t_lastcheck; % Zeitpunkt der letzten Prüfung
if isempty(t_lastsave), t_lastsave = 0; end % Initialisierung
if isempty(t_lastcheck), t_lastcheck = 0; end % Initialisierung
if ~Set.general.isoncluster, return; end % nur auf Cluster machen
if now() < t_lastcheck + 2/(24*60) 
  % Letzte Prüfung ist erst zwei Minuten her.
  return
end
if now() < t_lastsave + 4/(24*60) 
  % Letztes Speichern ist erst vier Minuten her.
  return
end
t_end_plan = Set.general.computing_cluster_start_time + ... % Rechne in Tagen
  Set.general.computing_cluster_max_time/(24*3600); % geplante Endzeit
if now() > t_end_plan + 10/(24*60) 
  % Annahme: Mehr als 10min nach Ende entspricht Offline-Auswertung
  return
end
% Einmal nach der Hälfte der Zeit speichern, damit zu lange Rechenzeiten
% eines Partikels am Ende nicht das Speichern blockiert
force_save = false;
if t_lastsave == 0 && (t_end_plan-now()) < ... % Restlaufzeit in d
    0.5*Set.general.computing_cluster_max_time/(24*3600) % Erlaubte Dauer in s
  force_save = true;
  cds_log(4, sprintf(['[fitness/save_generation] Noch %1.1fh verbleiben ', ...
    '(von %1.1fh). Speichere.'], (t_end_plan-now())*24, Set.general.computing_cluster_max_time/3600));
end
if ~force_save % Prüfe weitere Speicher-Bedingungen gegen Ende der Optimierung
  % Prüfe, wie lange der Fitness-Aufruf maximal dauerte.
  PSO_Detail_Data = cds_save_particle_details([], [], 0, 0, NaN, NaN, NaN, NaN, 'output');
  T_fitness_max = max(PSO_Detail_Data.comptime(:));
  % Wenn 100% mehr als diese Zeit nur noch verbleibt, speichere. Mindestens
  % aber 15min vor Ende einmal speichern, falls Fitness-Aufruf schnell geht
  t_lastcheck = now();
  if now() < t_end_plan - max(2*T_fitness_max, 15*60)/(24*3600)
    return % Das Planmäßige Ende ist noch zu lange entfernt. Nicht speichern
  end
end
if strcmp(Set.optimization.algorithm, 'mopso')
  cds_save_all_results_mopso([], Set, Structure);
elseif strcmp(Set.optimization.algorithm, 'gamultiobj')
  cds_save_all_results_gamultiobj(struct(),struct('Generation', i_gen, ...
    'Score', []), 'iter', Set, Structure);
elseif strcmp(Set.optimization.algorithm, 'pso')
  cds_save_all_results_pso(struct('iteration', i_gen),'iter', Set, Structure);
end
t_lastsave = now();
end
