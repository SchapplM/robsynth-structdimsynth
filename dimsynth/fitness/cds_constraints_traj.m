% Nebenbedingungen für Roboter-Maßsynthese berechnen (Teil 2: Geschwindig- 
% keitsebene, inkl Trajektorienberechnung). Entspricht Strafterm aus
% Nebenbedingungsverletzung.
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% Traj_0
%   Endeffektor-Trajektorie (bezogen auf Basis-KS)
% q
%   Anfangs-Gelenkwinkel für die Trajektorien-IK (gradientenbasiert)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Structure
%   Eigenschaften der Roboterstruktur (aus cds_gen_robot_list.m)
% Stats_constraints
%   Zusätzliche Eigenschaften der Anfangswerte aus cds_constraints.
% 
% Ausgabe:
% fval
%   Zielfunktionswert, der im PSO-Algorithmus minimiert wird. Entspricht
%   Strafterm in der Fitnessfunktion bei Verletzung der Nebenbedingungen
%   Werte:
%   1e3: Keine Verletzung der Nebenbedingungen. Alles i.O.
%   1e3...2e3: Arbeitsraum-Hindernis-Kollision in Trajektorie
%   2e3...3e3: Bauraumverletzung in Trajektorie
%   3e3...4e3: Selbstkollision in Trajektorie
%   4e3...5e3: Konfiguration springt
%   5e3...6e3: Beschleunigungsgrenzen
%   6e3...7e3: Geschwindigkeitsgrenzen
%   7e3...8e3: Gelenkwinkelgrenzen in Trajektorie (alle Beinkette zusammen)
%   8e3...9e3: Gelenkwinkelgrenzen in Trajektorie (jede Beinkette)
%   9e3...1e4: Parasitäre Bewegung (Roboter strukturell unpassend)
%   1e4...4e4: Inkonsistente Pos./Geschw./Beschl. in Traj.-IK. für Beink. 1 (Sonderfall 3T2R)
%   4e4...5e4: Singularität in Beinkette (obige Betrachtung daher sinnlos)
%   5e4...6e4: Singularität der PKM (bezogen auf Aktuierung)
%   6e4...1e5: IK in Trajektorie nicht lösbar (später mit vorherigem zusammengefasst)
%   1e5...1e9: Nicht belegt (siehe cds_constraints.m)
% Q,QD,QDD
%   Gelenkpositionen und -geschwindigkeiten des Roboters (für PKM auch
%   passive Gelenke)
% Jinvges
%   Zeilenweise (inverse) Jacobi-Matrizen des Roboters (für PKM). Wird hier
%   ausgegeben, da sie bei Berechnung der IK anfällt. Bezogen auf
%   Geschwindigkeit aller Gelenke und EE-Geschwindigkeit
% JP
%   Zeilenweise Gelenkpositionen aller Gelenke des Roboters
% constrvioltext [char]
%   Text mit Zusatzinformationen, die beim Aufruf der Fitness-Funktion
%   ausgegeben werden

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval,Q,QD,QDD,Jinv_ges,JP,constrvioltext] = cds_constraints_traj( ...
  R, Traj_0, q, Set, Structure, Stats_constraints)
% Debug
% save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_traj_0.mat'));
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_traj_0.mat')); nargin=6

% Initialisierung
fval = NaN;
constrvioltext = 'Undefiniert';
X2phizTraj_alt = [];
Q_alt = [];
QD_alt = [];
QDD_alt = [];
Jinv_ges_alt = [];
JP_alt = [];
wn_all = [];
mincolldist_all = NaN(3,1);
mininstspcdist_all = NaN(3,1);
% Speicherung der Trajektorie mit aktualisierter EE-Drehung bei Aufg.-Red.
X2 = NaN(size(Traj_0.X)); XD2 = NaN(size(Traj_0.X)); XDD2 = NaN(size(Traj_0.X));
constrvioltext_alt = '';
[currgen,currind,~,resdir] = cds_get_new_figure_filenumber(Set, Structure, '');
% Schleife über mehrere mögliche Nebenbedingungen der inversen Kinematik
fval_ar = NaN(1,2);
task_red = R.Type == 0 && sum(R.I_EE_Task) < R.NJ || ... % Seriell: Redundant wenn mehr Gelenke als Aufgaben-FG
           R.Type == 2 && sum(R.I_EE_Task) < sum(R.I_EE); % Parallel: Redundant wenn mehr Plattform-FG als Aufgaben-FG
if task_red
  ar_loop = 1:3; % Aufgabenredundanz liegt vor. Zusätzliche Schleife. Dritte Schleife ist nur zur Prüfung.
else
  ar_loop = 1; % Keine Aufgabenredundanz. Nichts zu berechnen.
end
for i_ar = ar_loop
% Speichere Ergebnis der vorherigen Iteration
if i_ar > 1
  fval_ar(i_ar-1) = fval;
end

%% Debug vorherige Iteration: Karte der Leistungsmerkmale für Aufgabenredundanz zeichnen
if i_ar > 1 && task_red && Set.general.debug_taskred_perfmap
  nt_red = size(Traj_0.X,1); % Zum Debuggen: Reduktion der Stützstellen
  if i_ar == 2 % Nur einmal die Rasterung generieren
    t1 = tic();
    cds_log(2, sprintf(['[constraints_traj] Konfig %d/%d: Beginne Aufgabenredundanz-', ...
      'Diagnosebild für Trajektorie mit %d Zeit-Stützstellen'], Structure.config_index, Structure.config_number, nt_red));
    [H_all, ~, s_ref, s_tref, phiz_range] = R.perfmap_taskred_ik( ...
      Traj_0.X(1:nt_red,:), Traj_0.IE, struct('settings_ik', s, ...
      'q0', q, 'I_EE_red', Set.task.DoF, 'map_phistart', X2(1,end), ...
      ... % nur grobe Diskretisierung für die Karte (geht schneller)
      'mapres_thresh_eepos', 10e-3, 'mapres_thresh_eerot', 5*pi/180, ...
      'mapres_thresh_pathcoordres', 0.2, 'mapres_redcoord_dist_deg', 5, ...
      'maplim_phi', [-1, +1]*210*pi/180)); % 210° statt 180° als Grenze
    cds_log(2, sprintf(['[constraints_traj] Konfig %d/%d: Daten für Diagnose-Bild der Aufgabenredundanz ', ...
      'erstellt. Auflösung: %dx%d. Dauer: %1.0fs'], Structure.config_index, Structure.config_number, length(s_ref), ...
      length(phiz_range), toc(t1)));
    % Speichere die Redundanzkarte (da die Berechnung recht lange dauert)
    suffix = 'TaskRedPerfMap_Data';
    name_prefix_ardbg = sprintf('Gen%02d_Ind%02d_Konfig%d', currgen, currind, Structure.config_index);
    save(fullfile(resdir,sprintf('%s_%s.mat', name_prefix_ardbg, suffix)), ...
      'Structure', 'H_all', 's_ref', 's_tref', 'phiz_range', 'i_ar', 'q', ...
      'nt_red');
    % Redundanzkarte für jedes Zielkriterium zeichnen (zur Einschätzung)
    wn_test = zeros(R.idx_ik_length.wnpos,1);
    wn_phys = zeros(4,1);
    for ll = 1:length(wn_test)+4 % letzter Durchlauf nur Konditionszahl zeichnen
      wn_test(:) = 0; wn_phys(:) = 0;
      if ll <= length(wn_test)
        wn_test(ll) = 1;
      else
        wn_phys(ll-length(wn_test)) = 1;
      end
      for ls = [false, true] % Skalierung des Bildes: Linear und Logarithmisch
        % Wähle Kriterien, die als Nebenbedigung gegen unendlich gehen.
        I_nbkrit = [R.idx_ikpos_wn.qlim_hyp, R.idx_ikpos_wn.ikjac_cond, ...
                    R.idx_ikpos_wn.jac_cond, R.idx_ikpos_wn.coll_hyp, ...
                    R.idx_ikpos_wn.instspc_hyp, R.idx_ikpos_wn.xlim_hyp];
        if ls && ~any(wn_test(I_nbkrit))
          continue % kein hyperbolisches Kriterium. Log-Skalierung nicht sinnvoll.
        end
        critnames = fields(R.idx_ikpos_wn)';
        critnames = [critnames(:)', {'coll_phys', 'instspc_phys', 'cond_ik_phys', 'cond_phys'}]; %#ok<NASGU>
        cds_debug_taskred_perfmap(Set, Structure, H_all, s_ref, s_tref(1:nt_red), ...
          phiz_range, X2(1:nt_red,6), Stats.h(1:nt_red,1), struct('wn', [wn_test;wn_phys], ...
          'i_ar', i_ar-1, 'i_fig', ll, 'name_prefix_ardbg', name_prefix_ardbg, 'fval', fval, ...
          'constrvioltext', constrvioltext, 'deactivate_time_figure', true, ...
          'critnames', {critnames}, 'ignore_h0', true, 'logscale', ls));
      end
    end
  end
  % Rechne die IK-Kriterien von Traj.- zu Pos.-IK um.
  % Reihenfolge: Siehe IK-Funktionen oder ik_optimcrit_index.m
  i=0; I_wn_traj = zeros(R.idx_ik_length.wnpos,1);
  for f = fields(R.idx_ikpos_wn)'
    i=i+1; I_wn_traj(i) = R.idx_iktraj_wnP.(f{1});
  end
  save(fullfile(resdir,sprintf('%s_TaskRed_Traj%d.mat', name_prefix_ardbg, i_ar-1)), ...
    'X2', 'Q', 'i_ar', 'q', 'Stats', 'fval', 's');
  cds_debug_taskred_perfmap(Set, Structure, H_all, s_ref, s_tref(1:nt_red), ...
    phiz_range, X2(1:nt_red,6), Stats.h(1:nt_red,1), struct('wn', [s.wn(I_wn_traj); zeros(4,1)], ...
    'i_ar', i_ar-1, 'name_prefix_ardbg', name_prefix_ardbg, 'fval', fval, ...
    'critnames', {critnames}, 'constrvioltext', constrvioltext));
  % Falls beim Debuggen die Aufgaben-Indizes zurückgesetzt wurden
  R.update_EE_FG(R.I_EE, Set.task.DoF);
end

%% Inverse Kinematik der Trajektorie berechnen
% Einstellungen für IK in Trajektorien
s = struct( ...
  ... % kein Winkel-Normalisierung, da dadurch Sprung in Trajektorie und keine 
  ... % Prüfung gegen vollständige Umdrehungen möglich
  'normalize', false, ... 
  ... % Einhaltung der Gelenkwinkelgrenzen nicht über NR-Bewegung erzwingen
  'enforce_qlim', false, ...
  'Phit_tol', 1e-10, 'Phir_tol', 1e-10); % feine Toleranz
s.wn = zeros(R.idx_ik_length.wntraj,1);
if R.Type == 2 % PKM
  s.debug = Set.general.debug_calc;
end
% Benutze eine Singularitätsvermeidung, die nur in der Nähe von
% Singularitäten aktiv ist. Wenn die Kondition wesentlich schlechter wird,
% wird der Roboter am Ende sowieso verworfen. Also nicht so schlimm, falls
% diese Parameter zu Instabilität führen.
% Die Nullraumbewegung fängt sehr langsam bei Überschreiten der Schwelle
% an. Daher Schwellwert niedriger setzen als die Grenze zur Singularität.
% Daher auch kleinere Schwellwerte als in cds_constraints.
cond_thresh_jac = 100;
cond_thresh_ikjac = 250;
if Set.optimization.constraint_obj(4) ~= 0 % Grenze für Jacobi-Matrix für Abbruch
  % IK-Jacobi (Aufgaben-Koordinaten) wichtig für Gelenkgeschwindigkeit in
  % redundanter Traj.-IK.
  cond_thresh_ikjac = Set.optimization.constraint_obj(4)/2;
  % Jacobi bzgl. Antriebe/Gesamtkoordinaten maßgeblich für Systemeigenschaft
  cond_thresh_jac = Set.optimization.constraint_obj(4)/4;
end
s.cond_thresh_ikjac = cond_thresh_ikjac;
s.cond_thresh_jac = cond_thresh_jac; % Für Seriell und PKM
% IK-Jacobi (Aufgaben-FG)
s.wn(R.idx_iktraj_wnP.ikjac_cond) = 1; % P-Anteil Konditionszahl (IK-Jacobi)
s.wn(R.idx_iktraj_wnD.ikjac_cond) = 0.1; % D-Anteil Konditionszahl (IK-Jacobi)
% Jacobi (analytischbei PKM, geometrisch bei seriell).
s.wn(R.idx_iktraj_wnP.jac_cond) = 1; % P-Anteil Konditionszahl (Jacobi)
s.wn(R.idx_iktraj_wnD.jac_cond) = 0.1; % D-Anteil Konditionszahl (Jacobi)

% Stelle Schwellwerte zur Aktivierung der Kollisions- und Bauraumeinhaltung
% fest. Benutze die Werte, die in der Eckpunkt-IK gefunden wurden
if nargin >= 6
  % Die Kriterien werden aktiviert, wenn der beste Wert um 30%
  % verschlechtert wird. Dann ist das Kriterium im besten Fall nicht aktiv
  % und führt zu Schwingungen der IK.
  s.installspace_thresh = 0.7*min(Stats_constraints.bestinstspcdist(:));
  % Ausgegebene Kollisionsabstände sind negativ, wenn es keine Koll. gibt.
  s.collision_thresh = -0.7*min(Stats_constraints.bestcolldist(:));
  if s.collision_thresh <= 0
    s.collision_thresh = NaN;
  end
end
  
% Zusätzliche Optimierung für Aufgabenredundanz.
% TODO: Die Reglereinstellungen sind noch nicht systematisch ermittelt.
if i_ar == 1 % erster Durchlauf ohne zusätzliche Optimierung (nimmt minimale Geschwindigkeit)
  % Dadurch auch keine Nullraumbewegung für Gelenkgrenzen o.ä.
end
% elseif false && i_ar == 2 && fval > 5e4 % deaktiviert
%   % Vorher nicht lösbar. Optimiere noch weiter. Die normale Lösung kann bei
%   % Annäherung an Arbeitsraumgrenzen fehlschlagen.
%   % Gehe in Fall ganz unten (Konditionszahl-Optimierung)
if i_ar == 2 && fval > 7e3 && fval < 9e3
  % Positionsgrenzen wurden verletzt. Besonders in Nebenbedingungen
  % berücksichtigen
  s.wn(R.idx_iktraj_wnP.qlim_par) = 0.99; % P-Anteil quadratische Grenzen
  s.wn(R.idx_iktraj_wnP.qlim_hyp) = 0.01; % P-Anteil hyperbolische Grenzen
  s.wn(R.idx_iktraj_wnD.qlim_par) = 0.1; % D-Anteil quadratische Grenzen (Dämpfung)
  s.wn(R.idx_iktraj_wnD.qlim_hyp) = 0.001; % D-Anteil hyperbolische Grenzen (Dämpfung)
  s.enforce_qlim = true; % Bei Verletzung maximal entgegenwirken
end
if i_ar == 2
  % Dämpfung der Geschwindigkeit, gegen Schwingungen
  s.wn(R.idx_iktraj_wnP.qDlim_par) = 0.7;
  % Auch Dämpfung bezüglich der redundanten Koordinate
  s.wn(R.idx_iktraj_wnP.xDlim_par) = 0.7;
end
if i_ar == 2 && fval > 6e3 && fval < 7e3
  % Geschwindigkeit wurde verletzt. Wird in NB eigentlich schon automatisch
  % berücksichtigt. Eine weitere Reduktion ist nicht möglich.
end
if i_ar == 2 && any(strcmp(Set.optimization.objective, 'colldist')) && any(fval_ar <= 1e3)
  % Wenn Kollisionsabstände ein Zielkriterium sind, optimiere diese hier permanent
  s.wn(R.idx_iktraj_wnP.coll_par) = 0.1; % P-Anteil Kollisionsvermeidung (quadratisch)
  s.wn(R.idx_iktraj_wnD.coll_par) = 0.01; % D-Anteil Kollisionsvermeidung (quadratisch)
end

if i_ar == 2 && (any(fval_ar > 3e3 & fval_ar < 4e3) || ... % Ausgabewert für Kollision
    ... % Wenn Kollisionen grundsätzlich geprüft werden sollen, immer als NB setzen,
    ... % wenn vorher auch die Bauraumprüfung fehlgeschlagen ist. Beide im Zielkonflikt
    Set.optimization.constraint_collisions && any(fval_ar > 2e3 & fval_ar < 3e3) )
  % Das Kriterium ist sowieso immer aktiv (s.u.)
  s.collbodies_thresh = 5; % 400% größere Kollisionskörper für Aktivierung (statt 50%)
end

if i_ar == 2 && ~isempty(Set.task.installspace.type) && ...
    (any(fval_ar > 2e3 & fval_ar < 3e3) || ... % Ausgabewert für Bauraumverletzung
    ... % auch vorsorglich aktivieren, wenn Bauraum geprüft wird. Sehr wahrscheinlich,
    ... % dass nach der Kollision direkt der Bauraum fehlschlägt.
    any(fval_ar > 3e3 & fval_ar < 4e3)) % Ausgabewert für Kollision
  % Bauraumverletzung trat auf. Bauraum als Nebenbedingung sowieso immer
  % aktiv.
  % Aktivierung der Bauraum-Nebenbedingung bereits mit größerem Abstand.
  % TODO: Bezug auf charakteristische Länge des Bauraums
  s.installspace_thresh = 0.2; % 200mm Abstand von Bauraumgrenze von innen
end
if i_ar == 2 && ~isempty(Set.task.installspace.type) && s.wn(R.idx_iktraj_wnP.instspc_hyp)==0
  % Bauraumprüfung ist allgemein aktiv, wird aber in der
  % Nullraumoptimierung nicht bedacht. Zusätzliche Aktivierung mit sehr
  % kleinem Schwellwert zur Aktivierung (nur für Notfälle)
  s.wn(R.idx_iktraj_wnP.instspc_hyp) = 1e-4; % P-Anteil Bauraumeinhaltung
  s.wn(R.idx_iktraj_wnD.instspc_hyp) = 1e-5; % D-Anteil Bauraumeinhaltung
  s.installspace_thresh = 0.050; % 50mm Abstand von Bauraumgrenze von innen
end
if i_ar == 2 && Set.optimization.constraint_collisions && s.wn(R.idx_iktraj_wnP.coll_hyp)==0
  % Kollisionsprüfung ist allgemein aktiv, wird aber in der
  % Nullraumoptimierung nicht bedacht. Zusätzliche Aktivierung mit sehr
  % kleinem Schwellwert zur Aktivierung (nur für Notfälle, als Nebenbedingung)
  s.wn(R.idx_iktraj_wnP.coll_hyp) = 1; % P-Anteil Kollisionsvermeidung (hyperbolisch)
  s.wn(R.idx_iktraj_wnD.coll_hyp) = 0.1; % D-Anteil Kollisionsvermeidung (hyperbolisch)
  % Aktivierungsbereich für Kollisionsvermeidung verkleinern (nur für
  % Ausnahmefälle stark an Grenze)
  s.collbodies_thresh = 1.25; % 25% größere Kollisionskörper für Aktivierung (statt 50%)
end
if any(strcmp(Set.optimization.objective, 'condition')) && any(fval_ar <= 1e3)
  % Die Jacobi-Matrix soll optimiert werden. Setze den Schwellwert zur
  % Aktivierung dieser Kennzahl auf "immer".
  % Das Kriterium ist anhand der Gewichtungsfaktoren sowieso aktiv. (s.o.)
  s.cond_thresh_jac = 1;
end
if i_ar == 3
  Q_change = Q - Q_alt;
  if all(abs(Q_change(:)) < 1e-6)
    cds_log(-1, sprintf(['[constraints_traj] Konfig %d/%d Ergebnis der IK unverändert, ', ...
      'trotz erneuter Durchführung mit anderer Gewichtung. Vorher: [%s], ', ...
      'nachher: [%s]'], Structure.config_index, Structure.config_number, disp_array(wn_alt', '%1.1f'), disp_array(s.wn', '%1.1f')));
  end
  % Bestimme zusätzliche Kennzahlen zur Bewertung des Erfolgs der
  % mehrfachen Wiederholung der Bewegung aufgrund der Redundanz
  debug_str = sprintf('max(phiDDz): %1.1f -> %1.1f', ...
    max(abs(X2phizTraj_alt(:,3))), max(abs(XDD2(:,6))));
  debug_str = [debug_str, sprintf('; maxcondJ: %1.1f -> %1.1f', ...
    max(Stats_alt.condJ(:,1)), max(Stats.condJ(:,1)))]; %#ok<AGROW>
  debug_str = [debug_str, sprintf('; maxcondPhiq: %1.1f -> %1.1f', ...
    max(Stats_alt.condJ(:,2)), max(Stats.condJ(:,2)))]; %#ok<AGROW>
  if any(~isnan(mincolldist_all))
    debug_str = [debug_str, sprintf('; mincolldist [mm]: %1.1f -> %1.1f', ...
      1e3*mincolldist_all(1), 1e3*mincolldist_all(2))]; %#ok<AGROW>
  end
  if any(~isnan(mininstspcdist_all))
    debug_str = [debug_str, sprintf('; instspcdist [mm]: %1.1f -> %1.1f', ...
      1e3*mininstspcdist_all(1), 1e3*mininstspcdist_all(2))]; %#ok<AGROW>
  end
  if fval_ar(1) < fval_ar(2)
    cds_log(3, sprintf(['[constraints_traj] Konfig %d/%d: Ergebnis der ', ...
      'Traj.-IK hat sich nach Nullraumbewegung verschlechtert: %1.3e -> ', ...
      '%1.3e ("%s" -> "%s"), delta: %1.3e. %s'], Structure.config_index, ...
      Structure.config_number, fval_ar(1), fval_ar(2), constrvioltext_alt, ...
      constrvioltext, fval_ar(2)-fval_ar(1), debug_str));
    % Anmerkung: Das muss nicht unbedingt ein Fehler sein. Das Verletzen
    % der Geschwindigkeitsgrenzen kann eine Konsequenz sein. Die Hinzunahme
    % eines vorher nicht betrachteten Kriteriums kann eine Verschlechterung
    % erst erzeugen (z.B. Scheitern bei Ausweichbewegung)
    Q = Q_alt;
    QD = QD_alt;
    QDD = QDD_alt;
    Jinv_ges = Jinv_ges_alt;
    JP = JP_alt;
    fval = fval_ar(1);
    constrvioltext = [constrvioltext_alt, ' Erneute IK-Berechnung ohne Verbesserung'];
  elseif fval_ar(1) == fval_ar(2) && fval_ar(1) ~= 1e3
    % Ergebnisse identisch, obwohl es n.i.O. ist. Deutet auf Logik-Fehler
    % oder unverstandene Einstellungen
    cds_log(3, sprintf(['[constraints_traj] Konfig %d/%d: Ergebnis der ', ...
      'Traj.-IK nach Nullraumbewegung gleich: fval=%1.3e ("%s"). s.wn=[%s]'], ...
      Structure.config_index, Structure.config_number, fval_ar(1), constrvioltext, ...
      disp_array(wn_alt', '%1.1g')));
    constrvioltext = [constrvioltext, sprintf(' Identisches Ergebnis mehrfach berechnet')]; %#ok<AGROW>
  elseif fval_ar(1) == fval_ar(2) && fval_ar(1) == 1e3
    % Bestmöglicher Fall. Vorher und nachher in Ordnung. 
    % Nehme zusätzliche Kennzahlen um zu prüfen, ob sich das Ergebnis auch
    % verbessert hat. Wobei Verbesserung hier eigentlich kein Kriterium
    % ist, da primär Nebenbedingungen ("constraints") geprüft werden.
    cds_log(3, sprintf(['[constraints_traj] Konfig %d/%d: Ergebnis der ', ...
      'Traj.-IK vor und nach Nullraumbewegung i.O.: %s; \n\twn: [%s] -> [%s]'], ...
      Structure.config_index, Structure.config_number, debug_str, ...
      disp_array(wn_all(1,:), '%1.1g'), disp_array(wn_all(2,:), '%1.1g')));
  else
    % Zweiter Durchlauf der Optimierung brachte Verbesserung. Jetzt ist es genug.
    cds_log(3, sprintf(['[constraints_traj] Konfig %d/%d: Ergebnis der ', ...
      'Traj.-IK hat sich nach Nullraumbewegung verbessert: %1.3e -> ', ...
      '%1.3e ("%s" -> "%s"), delta: %1.3e. %s'], Structure.config_index, ...
      Structure.config_number, fval_ar(1), fval_ar(2), constrvioltext_alt, ...
      constrvioltext, fval_ar(2)-fval_ar(1), debug_str));
    constrvioltext = [constrvioltext, sprintf([' Verbesserung durch ', ...
      'erneute IK-Berechnung (%1.3e->%1.3e, delta: %1.3e). Vorher: %s'], ...
      fval_ar(1), fval_ar(2), fval_ar(2)-fval_ar(1), constrvioltext_alt)]; %#ok<AGROW>
  end
  % Debug: Vergleiche vorher-nachher.
  if Set.general.debug_taskred_fig
    RP = ['R', 'P'];
    change_current_figure(3009);clf;
    set(3009,'Name','AR_TrajDbg_q', 'NumberTitle', 'off');
    if ~strcmp(get(3009, 'windowstyle'), 'docked')
      set(3009,'units','normalized','outerposition',[0 0 1 1]);
    end
    for i = 1:R.NJ
      if R.Type ~= 0
        legnum = find(i>=R.I1J_LEG, 1, 'last');
        legjointnum = i-(R.I1J_LEG(legnum)-1);
      end
      subplot(ceil(sqrt(R.NJ)), ceil(R.NJ/ceil(sqrt(R.NJ))), i);
      hold on; grid on;
      plot(Traj_0.t, Q_alt(:,i), '-');
      plot(Traj_0.t, Q(:,i), '-');
      plot(Traj_0.t([1,end]), repmat(Structure.qlim(i,:),2,1), 'r-');
      ylim(minmax2([Q(:,i);Q(:,i);Q_alt(:,i);Q_alt(:,i)]'));
      if R.Type == 0
        title(sprintf('q %d (%s)', i, RP(R.MDH.sigma(i)+1)));
      else
        title(sprintf('q %d (%s), L%d,J%d', i, RP(R.MDH.sigma(i)+1), legnum, legjointnum));
      end
    end
    linkxaxes
    sgtitle('Gelenkpositionen (vor/nach AR)');
    legend({'ohne AR opt.' 'mit Opt.'});
    change_current_figure(3010);clf;
    set(3010,'Name','AR_TrajDbg_qD', 'NumberTitle', 'off');
    if ~strcmp(get(3010, 'windowstyle'), 'docked')
      set(3010,'units','normalized','outerposition',[0 0 1 1]);
    end
    for i = 1:R.NJ
      if R.Type ~= 0
        legnum = find(i>=R.I1J_LEG, 1, 'last');
        legjointnum = i-(R.I1J_LEG(legnum)-1);
      end
      subplot(ceil(sqrt(R.NJ)), ceil(R.NJ/ceil(sqrt(R.NJ))), i);
      hold on; grid on;
      plot(Traj_0.t, QD_alt(:,i), '-');
      plot(Traj_0.t, QD(:,i), '-');
      plot(Traj_0.t([1,end]), repmat(Structure.qDlim(i,:),2,1), 'r-');
      if ~all(isnan(QD(:)))
        ylim(minmax2([QD(:,i);QD(:,i);QD_alt(:,i);QD_alt(:,i)]'));
      end
      if R.Type == 0
        title(sprintf('qD %d (%s)', i, RP(R.MDH.sigma(i)+1)));
      else
        title(sprintf('qD %d (%s), L%d,J%d', i, RP(R.MDH.sigma(i)+1), legnum, legjointnum));
      end
    end
    linkxaxes
    sgtitle('Gelenkgeschwindigkeiten (vor/nach AR)');
    legend({'ohne AR opt.' 'mit Opt.'});
    change_current_figure(3011);clf;
    set(3011,'Name','AR_TrajDbg_qDD', 'NumberTitle', 'off');
    if ~strcmp(get(3011, 'windowstyle'), 'docked')
      set(3011,'units','normalized','outerposition',[0 0 1 1]);
    end
    for i = 1:R.NJ
      if R.Type ~= 0
        legnum = find(i>=R.I1J_LEG, 1, 'last');
        legjointnum = i-(R.I1J_LEG(legnum)-1);
      end
      subplot(ceil(sqrt(R.NJ)), ceil(R.NJ/ceil(sqrt(R.NJ))), i);
      hold on; grid on;
      plot(Traj_0.t, QDD_alt(:,i), '-');
      plot(Traj_0.t, QDD(:,i), '-');
      plot(Traj_0.t([1,end]), repmat(Structure.qDDlim(i,:),2,1), 'r-');
      if ~all(isnan(QDD(:)))
        ylim(minmax2([QDD(:,i);QDD(:,i);QDD_alt(:,i);QDD_alt(:,i)]'));
      end
      if R.Type == 0
        title(sprintf('qDD %d (%s)', i, RP(R.MDH.sigma(i)+1)));
      else
        title(sprintf('qDD %d (%s), L%d,J%d', i, RP(R.MDH.sigma(i)+1), legnum, legjointnum));
      end
    end
    linkxaxes
    sgtitle('Gelenkbeschleunigungen (vor/nach AR)');
    legend({'ohne AR opt.' 'mit Opt.'});
    change_current_figure(3012);clf;
    set(3012,'Name','AR_TrajDbg_h', 'NumberTitle', 'off');
    if ~strcmp(get(3012, 'windowstyle'), 'docked')
      set(3012,'units','normalized','outerposition',[0 0 1 1]);
    end
    for i = 1:size(Stats.h,2)-1
      subplot(4,4,i); hold on;
      plot(Traj_0.t, Stats_alt.h(:,1+i), '-');
      plot(Traj_0.t, Stats.h(:,1+i), '-');
      ylabel(sprintf('h %d', i)); grid on;
    end
    subplot(4,4,15); hold on;
    plot(Traj_0.t, Stats_alt.condJ(:,1), '-');
    plot(Traj_0.t, Stats.condJ(:,1), '-');
    ylabel('Jacobi-Konditionszahl'); grid on;
    if R.Type == 2
      subplot(4,4,16); hold on;
      plot(Traj_0.t, Stats_alt.condJ(:,2), '-');
      plot(Traj_0.t, Stats.condJ(:,2), '-');
      ylabel('IK-Jacobi-Konditionszahl'); grid on;
    end
    linkxaxes
    sgtitle('Zielkriterien (vor/nach AR)');
    legend({'ohne AR opt.' 'mit Opt.'});
    change_current_figure(3013);clf;
    set(3013,'Name','AR_TrajDbg_x6', 'NumberTitle', 'off');
    if ~strcmp(get(3013, 'windowstyle'), 'docked')
      set(3013,'units','normalized','outerposition',[0 0 1 1]);
    end
    for i = 1:3
      subplot(1,3,i); hold on; grid on;
      plot(Traj_0.t, X2phizTraj_alt(:,i), '-');
      switch i
        case 1, plot(Traj_0.t, X2(:,6), '-');   Dstr = '';
        case 2, plot(Traj_0.t, XD2(:,6), '-');  Dstr = 'D';
        case 3, plot(Traj_0.t, XDD2(:,6), '-'); Dstr = 'DD';
      end
      ylabel(sprintf('phiz %s', Dstr));
    end
    linkxaxes
    sgtitle('Redundante Koordinate (vor/nach AR)');
    legend({'ohne AR opt.' 'mit Opt.'})
    % Debug-Bilder speichern
    name_prefix_ardbg = sprintf('Gen%02d_Ind%02d_Konfig%d', currgen, ...
      currind, Structure.config_index);
    for fignr = 3009:3013
      for fileext=Set.general.save_robot_details_plot_fitness_file_extensions
        if strcmp(fileext{1}, 'fig')
          saveas(fignr, fullfile(resdir, sprintf('%s_TaskRed_%s.fig', ...
            name_prefix_ardbg, get(fignr, 'name'))));
        else
          export_fig(fignr, fullfile(resdir, sprintf('%s_TaskRed_%s.%s', ...
            name_prefix_ardbg, get(fignr, 'name'), fileext{1})));
        end
      end
    end
  end
  return
end
if i_ar == 2
  Q_alt = Q;
  QD_alt = QD;
  QDD_alt = QDD;
  X2phizTraj_alt = [X2(:,6),XD2(:,6),XDD2(:,6)];
  wn_alt = s.wn;
  Stats_alt = Stats; %#ok<NASGU>
  Jinv_ges_alt = Jinv_ges;
  JP_alt = JP;
end
% Entfernen des dritten Euler-Winkels aus der Trajektorie (wird sonst
% als Referenz benutzt und dann Kopplung zwischen Iterationen der Traj.-IK)
% Wird nach IK-Berechnung wieder eingetragen
if task_red % Nur bei Redundanz relevant (Nebenbedingungen)
  Traj_0.X(:,6) = 0; % wird ignoriert (xlim ist nicht aktiv als Kriterium)
  Traj_0.XD(:,6) = 0; % Wird für Dämpfung benötigt
  Traj_0.XDD(:,6) = 0; % wird ignoriert
end
wn_all(i_ar,:) = s.wn(:)'; %#ok<AGROW>
if R.Type == 0 % Seriell
  qlim = R.qlim;
  [Q, QD, QDD, PHI, JP, Stats] = R.invkin2_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD, Traj_0.t, q, s);
  Jinv_ges = NaN; % Platzhalter für gleichartige Funktionsaufrufe. Speicherung nicht sinnvoll für seriell.
else % PKM
  qlim = cat(1,R.Leg(:).qlim);
  [Q, QD, QDD, PHI, Jinv_ges, ~, JP, Stats] = R.invkin2_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD, Traj_0.t, q, s);
end

constrvioltext_alt = constrvioltext;
% Anfangswerte nochmal neu speichern, damit der Anfangswert exakt der
% Wert ist, der für die Neuberechnung gebraucht wird. Ansonsten ist die
% Reproduzierbarkeit durch die rng-Initialisierung der mex-Funktionen
% gefährdet.
if R.Type == 0 % Seriell
  R.qref = Q(1,:)';
else
  for i = 1:R.NLEG, R.Leg(i).qref = Q(1,R.I1J_LEG(i):R.I2J_LEG(i))'; end
end

% Prüfe Erfolg der Trajektorien-IK
% Erkenne eine valide Trajektorie bereits bei Fehler kleiner als 1e-6 an.
% Das ist deutlich großzügiger als die eigentliche IK-Toleranz
I_ZBviol = any(abs(PHI) > 1e-6,2) | any(isnan(Q),2) | ...
  ... % Falls beim letzten Schritt die Position noch i.O. ist, aber die Ge-
  ... % schwindigkeit schon NaN ist: Auch hier als Abbruch zählen.
  any(isnan(QD),2) | any(isnan(QDD),2);
%% Endeffektor-Bewegung neu für 3T2R-Roboter berechnen
% Der letzte Euler-Winkel ist nicht definiert und kann beliebige Werte einnehmen.
% Muss schon berechnet werden, bevor der Abbruch der Trajektorie geprüft
% wird (Variable X2 wird für Redundanzkarte benötigt).
% Gilt für 3T2R-PKM und für 3T3R-PKM in 3T2R-Aufgaben
if task_red || all(R.I_EE_Task == [1 1 1 1 1 0]) || Set.general.debug_calc
  LastTrajIdx = find(~I_ZBviol, 1, 'last' ); % berechne nur für i.O.-Punkte dir Kin.
  [X2(1:LastTrajIdx,:), XD2(1:LastTrajIdx,:), XDD2(1:LastTrajIdx,:)] = ...
    R.fkineEE2_traj(Q(1:LastTrajIdx,:), QD(1:LastTrajIdx,:), QDD(1:LastTrajIdx,:));
  % Erlaube auch EE-Drehungen größer als 180°
  X2(:,6) = denormalize_angle_traj(X2(:,6), XD2(:,6), Traj_0.t);
  % Debug: EE-Trajektorie zeichnen
  if false
    figure(4002);clf; %#ok<UNRCH>
    for rr = 1:2
      if rr == 1, l = 'trans'; else, l = 'rot'; end
      subplot(3,2,sprc2no(3,2,1,rr));
      plot(Traj_0.t, X2(:,(1:3)+(rr-1)*3), '-');
      grid on; ylabel(sprintf('x %s', l));
      subplot(3,2,sprc2no(3,2,2,rr));
      plot(Traj_0.t, XD2(:,(1:3)+(rr-1)*3), '-');
      grid on; ylabel(sprintf('xD %s', l));
      subplot(3,2,sprc2no(3,2,3,rr));
      plot(Traj_0.t, XDD2(:,(1:3)+(rr-1)*3), '-');
      grid on; ylabel(sprintf('xDD %s', l));
    end
    legend({'x', 'y', 'z'});
    linkxaxes
  end
end

%% Prüfe Erfolg der Trajektorien-IK
if any(I_ZBviol)
  % Bestimme die erste Verletzung der ZB (je später, desto besser)
  IdxFirst = find(I_ZBviol, 1 );
  % Umrechnung in Prozent der Traj.
  Failratio = 1-IdxFirst/length(Traj_0.t); % Wert zwischen 0 und 1
  fval = 1e4*(6+4*Failratio); % Wert zwischen 6e4 und 1e5.
  % Keine Konvergenz der IK. Weitere Rechnungen machen keinen Sinn.
  constrvioltext = sprintf('Keine IK-Konvergenz in Traj. Bis %1.0f%% (%d/%d) gekommen.', ...
    (1-Failratio)*100, IdxFirst, length(Traj_0.t));
  continue
  % Debug: Trajektorie zeichnen
  qDlim = Structure.qDlim;
  qDDlim = Structure.qDDlim;
  RP = ['R', 'P'];
  Q_norm = (Q - repmat(qlim(:,1)', size(Q,1), 1)) ./ ...
            repmat(qlim(:,2)'-qlim(:,1)', size(Q,1), 1);
  QD_norm = (QD - repmat(qDlim(:,1)', size(QD,1), 1)) ./ ...
            repmat(qDlim(:,2)'-qDlim(:,1)', size(QD,1), 1);
  QDD_norm = (QDD - repmat(qDDlim(:,1)', size(QDD,1), 1)) ./ ...
            repmat(qDDlim(:,2)'-qDDlim(:,1)', size(QDD,1), 1);
  change_current_figure(4001);clf;
  subplot(2,2,1);
  plot(Traj_0.t, Q, '-');
  grid on; ylabel('q');
  subplot(2,2,2);
  plot(Traj_0.t, Q_norm, '-');
  grid on; ylabel('q (norm)');
  subplot(2,2,3);
  plot(Traj_0.t, QD_norm, '-');
  grid on; ylabel('qD (norm)');
  subplot(2,2,4);
  plot(Traj_0.t, QDD_norm, '-');
  grid on; ylabel('qDD (norm)');
  linkxaxes
end

%% Singularität der PKM prüfen (bezogen auf Aktuierung)
% Wird bereits hier gemacht, damit die Anzahl der Berechnungen reduziert
% werden. Annahme: Wenn eine PKM strukturell immer singulär ist, gibt es
% nie eine Lösung.
% Nur für PKM prüfen. Annahme: Serielle Singularität immer vermeidbar.
if ~isinf(Set.optimization.condition_limit_sing_act) && R.Type == 2
  IdxFirst = 0; c = 0;
  % Berechne Konditionszahl für alle Punkte der Bahn
  for i = 1:length(Traj_0.t)
    % Vollständige (inverse) PKM-Jacobi-Matrix (bezogen auf alle Gelenke)
    Jinv_IK = reshape(Jinv_ges(i,:), R.NJ, sum(R.I_EE));
    % Konditionszahl der auf Antriebe bezogengen (inversen) Jacobi-Matrix
    c = cond(Jinv_IK(R.I_qa,:));
    if c > Set.optimization.condition_limit_sing
      IdxFirst = i;
      break;
    end
  end
  if IdxFirst ~= 0
    Failratio = 1-IdxFirst/length(Traj_0.t); % Wert zwischen 0 und 1
    constrvioltext = sprintf(['PKM ist singulär (Konditionszahl %1.1e ', ...
      'bei %1.0f%% der Traj. bzw. Schritt %d/%d).'], c, 100*(1-Failratio), IdxFirst, length(Traj_0.t));
    fval = 1e4*(5+1*Failratio); % Wert zwischen 5e4 und 6e4.
    continue
  end
end

%% Singularität der Beinketten prüfen (für PKM)
% Im Gegensatz zu cds_obj_condition wird hier die gesamte Beinkette
% betrachtet. Entspricht Singularität der direkten Kinematik der Beinkette.
IdxFirst = 0;
if R.Type == 2 % nur PKM; TODO: Auch für seriell prüfen?
  for jj = 1:length(Traj_0.t)
    Jinv_jj = reshape(Jinv_ges(jj,:), R.NJ, sum(R.I_EE));
    for kk = 1:R.NLEG
      % Jacobi-Matrix für alle Gelenke der Beinkette (bezug zu XD des EE)
      Jinv_kk = Jinv_jj(R.I1J_LEG(kk):R.I2J_LEG(kk),:);
      if any(isinf(Jinv_kk(:))) || any(isnan(Jinv_kk(:)))
        kappa_jjkk = NaN; % keine Bestimmung der Kondition möglich
      else
        kappa_jjkk = cond(Jinv_kk);
      end
      if Set.general.debug_calc && ~isinf(kappa_jjkk)
        % Probe, ob Jinv richtig ist. Nehme die aktualisierte
        % Plattform-Geschwindigkeit, falls 3T2R benutzt wird.
        xD_jj = Traj_0.XD(jj,:)'; xD_jj(~R.I_EE_Task)=XD2(jj,~R.I_EE_Task);
        qD_kk2 = Jinv_kk*xD_jj(R.I_EE);
        qD_kk1 = QD(jj,R.I1J_LEG(kk):R.I2J_LEG(kk))';
        diff_qD_abs = qD_kk1-qD_kk2;
        diff_qD_rel = diff_qD_abs./qD_kk2;
        if any(abs(diff_qD_abs) > 1e-8 & abs(diff_qD_rel) > 1e-3)
          save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', ...
            'cds_constraints_traj_sing_error_debug.mat'));
          error(['Neu berechnete Geschwindigkeit aus Beinketten-Jacobi-Matrix ', ...
            'stimmt nicht. Fehler: abs %1.3e, rel %1.3e'], ...
            max(abs(diff_qD_abs)), max(abs(diff_qD_rel)));
        end
      end
      if kappa_jjkk > 1e4
        IdxFirst = jj;
        break;
      end
    end
    if IdxFirst ~= 0
      break;
    end
  end
end
if IdxFirst ~= 0
  Failratio = 1-IdxFirst/length(Traj_0.t); % Wert zwischen 0 und 1
  fval = 1e4*(4+1*Failratio); % Wert zwischen 4e4 und 5e4.
  % Singularität in Beinkette. Weitere Rechnungen ergeben keinen Sinn
  % (Geschwindigkeit der Gelenke kann beliebig springen)
  constrvioltext = sprintf('Singularität in Beinkette %d (cond=%1.1e). Bis %1.0f%% (%d/%d) gekommen.', ...
    kk, kappa_jjkk, (1-Failratio)*100, IdxFirst, length(Traj_0.t));
  continue
end

%% IK mit zweiter Implementierung prüfen (nur Debug, für PKM)
% Dieser Test wird nach der Prüfung der Jacobi-Matrix der Beinkette
% durchgeführt. Annahme: Ist diese schlecht konditioniert, können bei den
% beiden IK-Implementierungen verschiedene Ergebnisse herauskommen.
if R.Type == 2 && Set.general.debug_calc % PKM; Rechne nochmal mit Klassenmethode nach
  [Q_debug, QD_debug, QDD_debug, PHI_debug, ~, ~, JP_debug] = R.invkin_traj( ...
    Traj_0.X, Traj_0.XD, Traj_0.XDD, Traj_0.t, q, s);
  ik_res_ik2 = (all(max(abs(PHI(:,R.I_constr_t_red)))<s.Phit_tol) && ...
      all(max(abs(PHI(:,R.I_constr_r_red)))<s.Phir_tol));% IK-Status Funktionsdatei
  ik_res_iks = (all(max(abs(PHI_debug(:,R.I_constr_t_red)))<s.Phit_tol) && ... 
      all(max(abs(PHI_debug(:,R.I_constr_r_red)))<s.Phir_tol)); % IK-Status Klassenmethode
  if ik_res_ik2 ~= ik_res_iks % Vergleiche IK-Status (Erfolg / kein Erfolg)
    if Set.general.matfile_verbosity > 0
      save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_trajik_error_debug.mat'));
    end
    % Hier keine Warnung wie oben. Traj.-IK darf nicht von Zufall abhängen.
    % TODO: Wirklich Fehlermeldung einsetzen. Erstmal so lassen, da
    % nicht kritisch.
    cds_log(-1, sprintf(['[constraints_traj] Konfig %d/%d: Traj.-IK-Berechnung ', ...
      'mit Funktionsdatei hat anderen Status (%d) als Klassenmethode (%d).'], ...
      Structure.config_index, Structure.config_number, ik_res_ik2, ik_res_iks));
  end
  % Prüfe, ob die ausgegebenen Gelenk-Positionen auch stimmen
  for i = 1:size(Q,1)
    JointPos_all_i_frominvkin = reshape(JP(i,:)',3,1+R.NJ+R.NLEG);
    Tc_Lges = R.fkine_legs(Q(i,:)');
    JointPos_all_i_fromdirkin = [zeros(3,1), squeeze(Tc_Lges(1:3,4,1:end))];
    % Vergleiche die Positionen. In fkine_legs wird zusätzlich ein
    % virtuelles EE-KS ausgegeben, nicht aber in invkin_ser.
    for kk = 1:R.NLEG
      test_JP = JointPos_all_i_frominvkin(:,kk+(-1+R.I1J_LEG(kk):R.I2J_LEG(kk))) - ...
      JointPos_all_i_fromdirkin(:,kk*2+(-2+R.I1J_LEG(kk):-1+R.I2J_LEG(kk)));
      if any(abs(test_JP(:)) > 1e-8)
        save(fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
          'tmp', 'cds_constraints_trajjointpos_error_debug.mat'));
        error(['Ausgegebene Gelenkpositionen stimmen nicht gegen direkte ', ...
          'Kinematik. Zeitpunkt %d, Beinkette %d. Max Fehler %1.1e'], i, kk, max(abs(test_JP(:))));
      end
    end
  end
  % Prüfe ob die Gelenk-Positionen aus Klasse und Vorlage stimmen
  % (nur prüfen, wenn die IK erfolgreich war. Sonst große Fehler bei
  % Zeitschritt des Abbruchs der Berechnung)
  if ik_res_ik2 && ik_res_iks
    test_Q = Q-Q_debug;
    if any(abs(test_Q(:))>1e-3)
      save(fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
        'tmp', 'cds_constraints_trajq_error_debug.mat'));
      % Bei schlecht konditionierten PKM oder bei Verzweigungspunkten der
      % IK-Lösung können verschiedene Lösungen entstehen (durch numerische 
      % Abweichungen in der Implementierung). Dann springen Winkel um
      % unterschiedliche Vielfache von pi. Darf hier nicht passieren, da
      % Jacobi oben geprüft wurde.
      error(['Ausgabevariable Q aus invkin_traj vs invkin2_traj stimmt nicht. ', ...
        'Max Fehler %1.4e.'], max(abs(test_Q(:))));
    end
    test_QD = QD-QD_debug;
    if any(abs(test_QD(:))>1e-3) && ~any(abs(test_Q(:))>1e-3)
      % Nur Geschwindigkeit testen, wenn Position erfolgreich war
      save(fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
        'tmp', 'cds_constraints_trajqD_error_debug.mat'))
      error(['Ausgabevariable QD aus invkin_traj vs invkin2_traj stimmt nicht. ', ...
        'Max Fehler %1.4e.'], max(abs(test_QD(:))));
    end
    test_QDD_abs = QDD-QDD_debug; % nahe Singularität große Zahlenwerte für QDD ...
    test_QDD_rel = test_QDD_abs./QDD; % ... dadurch Numerik-Probleme möglich.
    I_err = abs(test_QDD_abs)>1e-3 & abs(test_QDD_rel)>1e-3; % 0,1% Abweichung erlaubt
    if any(I_err(:)) && ~any(abs(test_Q(:))>1e-3)
      % Nur Beschleunigung testen, wenn Position erfolgreich war
      save(fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
        'tmp', 'cds_constraints_trajqDD_error_debug.mat'));
      error(['Ausgabevariable QDD aus invkin_traj vs invkin2_traj stimmt nicht. ', ...
        'Max Fehler abs %1.4e., rel %1.4f%%. Zuerst Zeitschritt %d.'], ...
        max(abs(test_QDD_abs(:))), 100*max(abs(test_QDD_rel(:))), ...
        find(any(I_err,2),1,'first'));
    end
    test_JPtraj = JP-JP_debug;
    if any(abs(test_JPtraj(:))>1e-6) && ~any(abs(test_Q(:))>1e-3)
      % Nur testen, wenn Gelenkkoordinaten übereinstimmen
      save(fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
        'tmp', 'cds_constraints_trajjointpos2_error_debug.mat'));
      error(['Ausgabevariable JP aus invkin_traj vs invkin2_traj stimmt nicht. ', ...
        'Max Fehler %1.4e.'], max(abs(test_JPtraj(:))));
    end
  end
end

%% Prüfe neue Endeffektor-Bewegung für 3T2R-Roboter
% Die Neuberechnung erfolgt bereits weiter oben (3T2R-PKM/3T2R-Aufgabe)
if task_red || all(R.I_EE_Task == [1 1 1 1 1 0]) || Set.general.debug_calc
  % Teste nur die ersten fünf Einträge (sind vorgegeben). Der sechste
  % Wert wird an dieser Stelle erst berechnet und kann nicht verglichen werden.
  % Hier wird nur eine Hin- und Rückrechnung (InvKin/DirKin) gemacht. 
  test_X = Traj_0.X(:,1:5) - X2(:,1:5);
  test_X([false(size(test_X,1),3),abs(abs(test_X(:,4:5))-2*pi)<1e-3]) = 0; % 2pi-Fehler entfernen
  if any(abs(test_X(:))>1e-6)
    % Bestimme die mittlere Abweichung zwischen Position des Endeffektors
    % aus inverser und direkter Kinematik
    % Dieser Fall darf eigentlich gar nicht auftreten, wenn invkin und
    % fkin korrekt implementiert sind.
    fval_x = mean(test_X(:));
    fval_x_norm = 2/pi*atan(fval_x*70); % Normierung auf 0 bis 1. 0.1 -> 0.9
    fval = 1e4*(3+fval_x_norm); % Werte zwischen 3e4 und 4e4
    constrvioltext=sprintf(['Fehler der EE-Lage der ersten Beinkette ', ...
      'zwischen invkin und fkine. Max Fehler %1.2e'], max(abs(test_X(:))));
    continue
  end
  test_XD = Traj_0.XD(:,1:5) - XD2(:,1:5);
  if any(abs(test_XD(:))>1e-5)
    % Bestimme die mittlere Abweichung zwischen Geschwindigkeit des Endeffektors
    % aus inverser und direkter differentieller Kinematik. Darf
    % eigentlich nicht passieren (s.o.).
    fval_xD = mean(test_XD(:));
    fval_xD_norm = 2/pi*atan(fval_xD*70); % Normierung auf 0 bis 1. 0.1 -> 0.9
    fval = 1e4*(2+fval_xD_norm); % Werte zwischen 2e4 und 3e4
    constrvioltext=sprintf(['Fehler der EE-Geschwindigkeit der ersten Beinkette ', ...
      'zwischen invkin und fkine. Max Fehler %1.2e'], max(abs(test_XD(:))));
    continue
  end
  test_XDD = Traj_0.XDD(:,1:5) - XDD2(:,1:5);
  if any(abs(test_XDD(:))>1e-4)
    % Bestimme die mittlere Abweichung zwischen Beschleunigung des Endeffektors
    % aus inverser und direkter differentieller Kinematik. Darf
    % eigentlich nicht passieren (s.o.).
    fval_xDD = mean(test_XDD(:));
    fval_xDD_norm = 2/pi*atan(fval_xDD*70); % Normierung auf 0 bis 1. 0.1 -> 0.9
    fval = 1e4*(1+fval_xDD_norm); % Werte zwischen 1e4 und 2e4
    constrvioltext=sprintf(['Fehler der EE-Beschleunigung der ersten Beinkette ', ...
      'zwischen invkin und fkine. Max Fehler %1.2e'], max(abs(test_XDD(:))));
    continue
  end
  % Eintragen des dritten Euler-Winkels, damit spätere Vergleiche funktionieren.
  if task_red || all(R.I_EE_Task == [1 1 1 1 1 0]) % 3T2R-Aufgabe oder 3T2R-PKM
    Traj_0.X(:,6) = X2(:,6);
    Traj_0.XD(:,6) = XD2(:,6);
    Traj_0.XDD(:,6) = XDD2(:,6);
  end
end
%% Prüfe, ob eine Verletzung der Geschwindigkeits-Zwangsbedingungen vorliegt
% Bei 3T2R-PKM kann eine Positions-ZB ungleich Null für die z-Rotation
% korrekt sein. Diese muss aber konstant bleiben und darf sich nicht
% ändern. Durch die Prüfung der ZB-Zeitableitung wird geprüft, ob QD und XD
% konsistent sind.
if any(strcmp(Set.optimization.objective, 'valid_act')) && R.Type ~= 0 % nur sinnvoll bei PKM-Struktursynthese
  % Geschwindigkeits-Zwangsbedingungen der Koppelpunkte.
  PHI4D_ges = R.constr4D2_traj(Q, QD, Traj_0.X, Traj_0.XD);
  % Zum Debuggen: Weitere Zwangsbedingungen
%   PHI1D_ges=NaN(size(PHI4D_ges)); PHI2D_ges=PHI1D_ges;
%   for jj = 1:length(Traj_0.t)
%     [~,PHI1D_ges(jj,:)] = R.constr1D(Q(jj,:)', QD(jj,:)', Traj_0.X(jj,:)',Traj_0.XD(jj,:)');
%     [~,PHI2D_ges(jj,:)] = R.constr2D(Q(jj,:)', QD(jj,:)', Traj_0.X(jj,:)',Traj_0.XD(jj,:)');
%   end
  if any(abs(PHI4D_ges(:))>1e-6)
    % Bilde Kennzahl aus Schwere der parasitären Bewegung
    fval_paras = mean(abs(PHI4D_ges(:)));
    fval_paras_norm = 2/pi*atan(fval_paras*700); % Normierung auf 0 bis 1. 0.01 -> 0.9
    fval = 1e3*(9+1*fval_paras_norm); % Normierung auf 9e3...1e4
    constrvioltext = sprintf(['Es gibt eine parasitäre Bewegung in %d/%d ', ...
      'Zeitschritten. Im Mittel %1.4f (rad/s bzw. m/s). Zuerst bei Zeitschritt %d.'], ...
      sum(any(abs(PHI4D_ges)>1e-3,2)), length(Traj_0.t), fval_paras, ...
      find(any(abs(PHI4D_ges)>1e-6,2),1,'first'));
    continue
  end
end
if R.Type ~= 0 && Set.general.debug_calc
  % Debuggen der Geschwindigkeits-Konsistenz: Vergleiche EE-Trajektorie von
  % verschiedenen Beinketten aus berechnet.
  for j = 2:R.NLEG
    [X3,XD3,XDD3] = R.fkineEE2_traj(Q, QD, QDD, uint8(j));
    test_X = Traj_0.X(:,1:6) - X3(:,1:6);
    test_X([false(size(test_X,1),3),abs(abs(test_X(:,4:6))-2*pi)<1e-3]) = 0; % 2pi-Fehler entfernen
    test_XD_abs = Traj_0.XD(:,1:6) - XD3(:,1:6);
    test_XD_rel = test_XD_abs ./ Traj_0.XD(:,1:6);
    test_XDD_abs = Traj_0.XDD(:,1:6) - XDD3(:,1:6);
    test_XDD_rel = test_XDD_abs ./ Traj_0.XDD(:,1:6);
    if any(abs(test_X(:))>1e-6)
      save(fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
        'tmp', 'cds_constraints_xtraj_legs_inconsistency.mat'));
      error(['Die Endeffektor-Trajektorie X aus Beinkette %d stimmt nicht ', ...
        'gegen Beinkette 1. Zuerst in Zeitschritt %d/%d. Fehler max. %1.4e.'], j, ...
        find(any(abs(test_X)>1e-6,2),1,'first'), length(Traj_0.t), max(abs(test_X(:))));
    end
    if any(abs(test_XD_abs(:))>1e-5 & abs(test_XD_rel(:))>1e-3)
        save(fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
        'tmp', 'cds_constraints_xDtraj_legs_inconsistency.mat'));
      error(['Die Endeffektor-Trajektorie XD aus Beinkette %d stimmt nicht ', ...
        'gegen Beinkette 1. Zuerst in Zeitschritt %d/%d. Fehler max. %1.4e.'], j, ...
        find(any(abs(test_XD)>1e-6,2),1,'first'), length(Traj_0.t), max(abs(test_XD(:))));
    end
    if any(abs(test_XDD_abs(:))>1e-4 & abs(test_XDD_rel(:))>1e-2)
      save(fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
        'tmp', 'cds_constraints_xDDtraj_legs_inconsistency.mat'));
      error(['Die Endeffektor-Trajektorie XDD aus Beinkette %d stimmt nicht ', ...
        'gegen Beinkette 1. Zuerst in Zeitschritt %d/%d. Fehler max. %1.4e.'], j, ...
        find(any(abs(test_XDD)>1e-6,2),1,'first'), length(Traj_0.t), max(abs(test_XDD(:))));
    end
  end
end
%% Prüfe, ob die Gelenkwinkelgrenzen verletzt werden
% Andere Prüfung als in cds_constraints.m. Gehe davon aus, dass die
% Trajektorie stetig und sprungfrei ist. Ist eine Winkelspannweite von mehr
% als 360° erlaubt, ist die Prüfung auf Winkelspannweite mit angle_range
% immer erfolgreich, auch wenn sich Gelenke mehrfach umdrehen.
q_range_T = diff(minmax2(Q')');
q_range_max = qlim(:,2)-qlim(:,1);
qlimviol_T = q_range_max' - q_range_T;
I_qlimviol_T = (qlimviol_T < 0);
if any(I_qlimviol_T)
  if Set.general.matfile_verbosity > 2
    save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_qviolT.mat'));
  end
  % Bestimme die größte relative Verletzung der Winkelgrenzen
  [fval_qlimv_T, I_worst] = min(qlimviol_T(I_qlimviol_T)./(q_range_max(I_qlimviol_T))');
  II_qlimviol_T = find(I_qlimviol_T); IIw = II_qlimviol_T(I_worst);
  fval_qlimv_T_norm = 2/pi*atan((-fval_qlimv_T)/0.3); % Normierung auf 0 bis 1; 2 ist 0.9
  fval = 1e3*(8+1*fval_qlimv_T_norm); % Wert zwischen 8e3 und 9e3
  % Überschreitung der Gelenkgrenzen (bzw. -bereiche). Weitere Rechnungen machen keinen Sinn.
  constrvioltext = sprintf(['Gelenkgrenzverletzung in Traj. Schlechteste ', ...
    'Spannweite: %1.2f/%1.2f (Gelenk %d)'], q_range_T(IIw), q_range_max(IIw), IIw);
  if 1e4*fval < Set.general.plot_details_in_fitness
    RP = ['R', 'P'];
    change_current_figure(1000); clf;
    for i = 1:R.NJ
      if R.Type ~= 0
        legnum = find(i>=R.I1J_LEG, 1, 'last');
        legjointnum = i-(R.I1J_LEG(legnum)-1);
      end
      subplot(ceil(sqrt(R.NJ)), ceil(R.NJ/ceil(sqrt(R.NJ))), i);
      hold on; grid on;
      plot(Traj_0.t, Q(:,i), '-');
      plot(Traj_0.t([1,end]), [1;1]*min(Q(:,i)), 'r-');
      plot(Traj_0.t([1,end]), [1;1]*(min(Q(:,i))+q_range_max(i)), 'r-');
      if R.Type == 0
        title(sprintf('q %d (%s)', i, RP(R.MDH.sigma(i)+1)));
      else
        title(sprintf('q %d (%s), L%d,J%d', i, RP(R.MDH.sigma(i)+1), legnum, legjointnum));
      end
    end
    sgtitle('Überschreitung der Gelenkwinkelspannweite');
  end
  continue
end

%% Prüfe die Gelenkwinkelgrenzen für eine symmetrische PKM-Konfiguration
% Bei Annahme einer symmetrischen PKM müssen die Gelenkkoordinaten aller
% Beinketten auch gemeinsam die Bedingung der Winkelspannweite erfüllen. 
% Dadurch wird eine Optimierung der Gelenkfeder-Ruhelagen ermöglicht. Sonst 
% kann man nicht für alle Beinketten die gleichen Parameter wählen.
% Trifft auch zu, wenn die Feder-Ruhelagen nicht optimiert, sondern nur
% mittig gewählt werden
if R.Type == 2 && Set.optimization.joint_stiffness_passive_revolute
  % Siehe cds_dimsynth_desopt.m (konsistent dazu).
  % Fasse die Gelenke jeder Beinkette zusammen (symmetrische Anordnung)
  qminmax_legs = reshape(minmax2(Q'),R.Leg(1).NJ,2*R.NLEG);
  qminmax_leg = minmax2(qminmax_legs);
  q_range_T_all_legs = repmat(diff(qminmax_leg'), 1, R.NLEG);
  qlimviol_T = q_range_max' - q_range_T_all_legs;
  I_qlimviol_T = (qlimviol_T < 0);
  if any(I_qlimviol_T)
    if Set.general.matfile_verbosity > 2
      save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', ...
        'cds_constraints_qviolT_all_legs.mat'));
    end
    % Bestimme die größte relative Verletzung der Winkelgrenzen
    [fval_qlimv_T, I_worst] = min(qlimviol_T(I_qlimviol_T)./(q_range_max(I_qlimviol_T))');
    II_qlimviol_T = find(I_qlimviol_T); IIw = II_qlimviol_T(I_worst);
    fval_qlimv_T_norm = 2/pi*atan((-fval_qlimv_T)/0.3); % Normierung auf 0 bis 1; 2 ist 0.9
    fval = 1e3*(7+1*fval_qlimv_T_norm); % Wert zwischen 7e3 und 8e3
    % Überschreitung der Gelenkgrenzen (bzw. -bereiche). Dadurch werden die
    % Bedingungen für Gelenkfedern später nicht mehr erfüllt.
    legnum = find(IIw>=R.I1J_LEG, 1, 'last');
    legjointnum = IIw-(R.I1J_LEG(legnum)-1);
    constrvioltext = sprintf(['Gelenkgrenzverletzung in Traj bei Be', ...
      'trachtung aller Beinketten. Schlechteste Spannweite: %1.2f/%1.2f ', ...
      '(Gelenk %d; Beinkette %d, Beingelenk %d)'], q_range_T_all_legs(IIw), ...
      q_range_max(IIw), IIw, legnum, legjointnum);
    continue
  end
end

%% Prüfe, ob die Geschwindigkeitsgrenzen verletzt werden
% Diese Prüfung erfolgt zusätzlich zu einer Antriebsauslegung.
% Gedanke: Wenn die Gelenkgeschwindigkeit zu schnell ist, ist sowieso kein
% Antrieb auslegbar und die Parameter können schneller verworfen werden.
% Außerdem liegt wahrscheinlich eine Singularität vor.
if any(~isinf(Structure.qDlim(:)))
  qD_max = max(abs(QD))';
  qD_lim = Structure.qDlim(:,2); % Annahme symmetrischer Geschw.-Grenzen
  [f_qD_exc,ifmax] = max(qD_max./qD_lim);
  if f_qD_exc>1
    f_qD_exc_norm = 2/pi*atan((f_qD_exc-1)); % Normierung auf 0 bis 1; 1->0.5; 10->0.94
    fval = 1e3*(6+1*f_qD_exc_norm); % Wert zwischen 6e3 und 7e3
    % Weitere Berechnungen voraussichtlich wenig sinnvoll, da vermutlich eine
    % Singularität vorliegt
    constrvioltext = sprintf('Geschwindigkeit eines Gelenks zu hoch: max Verletzung %1.1f%% (Gelenk %d)', ...
      (f_qD_exc-1)*100, ifmax);
    if 1e4*fval < Set.general.plot_details_in_fitness
      RP = ['R', 'P'];
      change_current_figure(1004);clf;
      for i = 1:R.NJ
        if R.Type ~= 0
          legnum = find(i>=R.I1J_LEG, 1, 'last');
          legjointnum = i-(R.I1J_LEG(legnum)-1);
        end
        subplot(ceil(sqrt(R.NJ)), ceil(R.NJ/ceil(sqrt(R.NJ))), i);
        hold on; grid on;
        plot(Traj_0.t, QD(:,i), '-');
        plot(Traj_0.t([1,end]), repmat(Structure.qDlim(i,:),2,1), 'r-');
        ylim(minmax2([QD(:,i);QD(:,i)]'));
        if R.Type == 0
          title(sprintf('qD %d (%s)', i, RP(R.MDH.sigma(i)+1)));
        else
          title(sprintf('qD %d (%s), L%d,J%d', i, RP(R.MDH.sigma(i)+1), legnum, legjointnum));
        end
      end
      linkxaxes
      sgtitle('Gelenkgeschwindigkeiten');
    end
    continue
  end
end

%% Prüfe, ob die Beschleunigungsgrenzen verletzt werden
% Gleiche Überlegung wie bei Prüfung der Geschwindigkeitsgrenzen
if any(~isinf(Structure.qDDlim(:)))
  qDD_max = max(abs(QDD))';
  qDD_lim = Structure.qDDlim(:,2); % Annahme symmetrischer Geschw.-Grenzen
  [f_qDD_exc,ifmax] = max(qDD_max./qDD_lim);
  if f_qDD_exc>1
    f_qDD_exc_norm = 2/pi*atan((f_qDD_exc-1)); % Normierung auf 0 bis 1; 1->0.5; 10->0.94
    fval = 1e3*(5+1*f_qDD_exc_norm); % Wert zwischen 5e3 und 6e3
    % Weitere Berechnungen voraussichtlich wenig sinnvoll, da vermutlich eine
    % Singularität vorliegt
    constrvioltext = sprintf(['Beschleunigung eines Gelenks zu hoch: ', ...
      'max Verletzung %1.1f%% (Gelenk %d)'], (f_qDD_exc-1)*100, ifmax);
    if 1e4*fval < Set.general.plot_details_in_fitness
      RP = ['R', 'P'];
      change_current_figure(1005);clf;
      for i = 1:R.NJ
        if R.Type ~= 0
          legnum = find(i>=R.I1J_LEG, 1, 'last');
          legjointnum = i-(R.I1J_LEG(legnum)-1);
        end
        subplot(ceil(sqrt(R.NJ)), ceil(R.NJ/ceil(sqrt(R.NJ))), i);
        hold on; grid on;
        plot(Traj_0.t, QDD(:,i), '-');
        plot(Traj_0.t([1,end]), repmat(Structure.qDDlim(i,:),2,1), 'r-');
        ylim(minmax2([QDD(:,i);QDD(:,i)]'));
        if R.Type == 0
          title(sprintf('qDD %d (%s)', i, RP(R.MDH.sigma(i)+1)));
        else
          title(sprintf('qDD %d (%s), L%d,J%d', i, RP(R.MDH.sigma(i)+1), legnum, legjointnum));
        end
      end
      linkxaxes
      sgtitle('Gelenkbeschleunigungen');
    end
    continue
  end
end

%% Prüfe, ob die Konfiguration umklappt während der Trajektorie
% Geschwindigkeit neu mit Differenzenquotient berechnen
QD_num = zeros(size(Q));
QD_num(2:end,R.MDH.sigma==1) = diff(Q(:,R.MDH.sigma==1))./...
  repmat(diff(Traj_0.t), 1, sum(R.MDH.sigma==1)); % Differenzenquotient
QD_num(2:end,R.MDH.sigma==0) = (mod(diff(Q(:,R.MDH.sigma==0))+pi, 2*pi)-pi)./...
  repmat(diff(Traj_0.t), 1, sum(R.MDH.sigma==0)); % Siehe angdiff.m
% Position neu mit Trapezregel berechnen (Integration)
Q_num = repmat(Q(1,:),size(Q,1),1)+cumtrapz(Traj_0.t, QD);
% Bestimme Korrelation zwischen den Verläufen (1 ist identisch)
corrQD = diag(corr(QD_num, QD));
corrQ = diag(corr(Q_num, Q));
% Falls eine Größe konstant ist, und die andere numerisch leicht schwankt,
% wird eine große Abweichung per Korrelation erkannt. Setze als i.O.
corrQ(all(abs(Q_num-Q)<1e-6)) = 1;
corrQD(all(abs(QD_num-QD)<1e-3)) = 1;
corrQ(all(abs(QD)<1e-10)) = 1; % qD=0 und q schwankt numerisch (als Nullraumbewegung) wegen IK-Positionskorrektur
if task_red && (any(corrQD < 0.95) || any(corrQ < 0.98))
  % TODO: Inkonsistenz ist Fehler in Traj.-IK. Dort korrigieren.
  cds_log(-1, sprintf(['[constraints_traj] Konfig %d/%d: Nullraumbewegung führt zu nicht ', ...
    'konsistenten Gelenkverläufen. Korrelation Geschw. min. %1.2f, ', ...
    'Position %1.2f'], Structure.config_index, Structure.config_number, min(corrQD), min(corrQ)'));
end
if ~task_red && (any(corrQD < 0.95) || any(corrQ < 0.98))
  % Wenn eine Gelenkgröße konstant ist (ohne Rundungsfehler), wird die
  % Korrelation NaN (Teilen durch 0). Werte NaN als Korrelation 1.
  % Damit werden zwei unterschiedliche, konstante Werte auch als Korr.=1
  % gewertet. Reicht für die Erkennung eines Sprungs/Umklappens
  corrQD(isnan(corrQD)) = 1;
  corrQ(isnan(corrQ)) = 1;
  % Bilde normierten Strafterm aus Korrelationskoeffizienten (zwischen -1
  % und 1).
  fval_jump_norm = 0.5*(mean(1-corrQ) + mean(1-corrQD));
  fval = 1e3*(4+1*fval_jump_norm); % Wert zwischen 4e3 und 5e3
  constrvioltext = sprintf('Konfiguration scheint zu springen. Korrelation Geschw. min. %1.2f, Position %1.2f', ...
    min(corrQD), min(corrQ));
  if 1e4*fval < Set.general.plot_details_in_fitness
    % Geschwindigkeit neu mit Trapezregel berechnen (Integration)
    QD_num2 = repmat(QD(1,:),size(QD,1),1)+cumtrapz(Traj_0.t, QDD);
    RP = ['R', 'P'];
    change_current_figure(1001);clf;
    for i = 1:R.NJ
      if R.Type == 2
      legnum = find(i>=R.I1J_LEG, 1, 'last');
      legjointnum = i-(R.I1J_LEG(legnum)-1);
      end
      subplot(ceil(sqrt(R.NJ)), ceil(R.NJ/ceil(sqrt(R.NJ))), i);
      hold on; grid on;
      hdl1=plot(Traj_0.t, QD(:,i), '-');
      hdl2=plot(Traj_0.t, QD_num(:,i), '--');
      hdl3=plot(Traj_0.t, QD_num2(:,i), ':');
      plot(Traj_0.t([1,end]), repmat(Structure.qDlim(i,:),2,1), 'r-');
      ylim(minmax2([QD_num(:,i);QD_num(:,i)]'));
      if R.Type == 0
        ylabel(sprintf('qD %d (%s)', i, RP(R.MDH.sigma(i)+1)));
      else
        ylabel(sprintf('qD %d (%s), L%d,J%d', i, RP(R.MDH.sigma(i)+1), legnum, legjointnum));
      end
      title(sprintf('corr(qD/qDD)=%1.2f', corrQD(i)));
      if i == length(q), legend([hdl1;hdl2;hdl3], {'qD','diff(q)', 'int(qDD)'}); end
    end
    linkxaxes
    sgtitle('Vergleich Gelenkgeschw.');
    change_current_figure(1002);clf;
    for i = 1:R.NJ
      if R.Type == 2
      legnum = find(i>=R.I1J_LEG, 1, 'last');
      legjointnum = i-(R.I1J_LEG(legnum)-1);
      end
      subplot(ceil(sqrt(R.NJ)), ceil(R.NJ/ceil(sqrt(R.NJ))), i);
      hold on; grid on;
      hdl1=plot(Traj_0.t, Q(:,i), '-');
      hdl2=plot(Traj_0.t, Q_num(:,i), '--');
      if R.Type == 0
        ylabel(sprintf('q %d (%s)', i, RP(R.MDH.sigma(i)+1)));
      else
        ylabel(sprintf('q %d (%s), L%d,J%d', i, RP(R.MDH.sigma(i)+1), legnum, legjointnum));
      end
      title(sprintf('corr(q/qD)=%1.2f', corrQ(i)));
      if i == length(q), legend([hdl1;hdl2], {'q','int(qD)'}); end
    end
    linkxaxes
    sgtitle('Verlauf Gelenkkoordinaten');
    change_current_figure(1003);clf;
    QDD_num = zeros(size(Q)); % Differenzenquotient
    QDD_num(1:end-1,:) = diff(QD(:,:))./ repmat(diff(Traj_0.t), 1, R.NJ);
    corrQDD = diag(corr(QDD_num, QDD));
    for i = 1:R.NJ
      if R.Type == 2
      legnum = find(i>=R.I1J_LEG, 1, 'last');
      legjointnum = i-(R.I1J_LEG(legnum)-1);
      end
      subplot(ceil(sqrt(R.NJ)), ceil(R.NJ/ceil(sqrt(R.NJ))), i);
      hold on; grid on;
      hdl1=plot(Traj_0.t, QDD(:,i), '-');
      hdl2=plot(Traj_0.t, QDD_num(:,i), '--');
      if R.Type == 0
        ylabel(sprintf('qDD %d (%s)', i, RP(R.MDH.sigma(i)+1)));
      else
        ylabel(sprintf('qDD %d (%s), L%d,J%d', i, RP(R.MDH.sigma(i)+1), legnum, legjointnum));
      end
      title(sprintf('corrQDD(qD/qDD)=%1.2f', corrQDD(i)));
      if i == length(q), legend([hdl1;hdl2], {'qDD','diff(qD)'}); end
    end
    linkxaxes
    sgtitle('Verlauf Gelenkbeschleunigungen');
  end
  continue
end
%% Aktualisiere Roboter für Kollisionsprüfung (geänderte Grenzen aus Traj.-IK)
if Set.optimization.constraint_collisions || ...
    ~isempty(Set.task.installspace.type) || ~isempty(Set.task.obstacles.type)
  [Structure.collbodies_robot, Structure.installspace_collbodies] = ...
    cds_update_collbodies(R, Set, Structure, Q);
end
%% Anpassung des Offsets für Schubgelenke
% Siehe cds_constraints.m
if Structure.desopt_prismaticoffset
  [fval_coll_tmp, fval_instspc_tmp] = cds_desopt_prismaticoffset(R, ...
    Traj_0.X, Set, Structure, JP, Q);
  if R.Type == 0, new_offset=R.DesPar.joint_offset(R.MDH.sigma==1);
  else, new_offset=R.Leg(1).DesPar.joint_offset(R.Leg(1).MDH.sigma==1); end
  cds_log(4, sprintf(['[constraints_traj] Konfig %d/%d: Schubgelenk-Offset wurde ', ...
    'optimiert. Ergebnis: %1.1fmm'], Structure.config_index, Structure.config_number, 1e3*new_offset));
  % Kollisionskörper müssen nochmal aktualisiert werden (wegen Offset)
  [Structure.collbodies_robot, Structure.installspace_collbodies] = ...
    cds_update_collbodies(R, Set, Structure, Q);
else
  fval_coll_tmp = NaN;
  fval_instspc_tmp = NaN;
end
%% Selbstkollisionserkennung für Trajektorie
if Set.optimization.constraint_collisions && ...
      (isnan(fval_coll_tmp) || fval_coll_tmp > 0)
  [fval_coll_traj, coll_traj, colldepth_abs] = cds_constr_collisions_self(R, Traj_0.X, ...
    Set, Structure, JP, Q, [3e3; 4e3]);
  mincolldist_all(i_ar) = min(colldepth_abs(:));
  if fval_coll_traj > 0
    fval = fval_coll_traj; % Normierung auf 3e3 bis 4e3 -> bereits in Funktion
    constrvioltext = sprintf('Kollision in %d/%d Traj.-Punkten.', ...
      sum(any(coll_traj,2)), size(coll_traj,1));
    continue
  end
end

%% Bauraumprüfung für Trajektorie
if ~isempty(Set.task.installspace.type) && ...
      (isnan(fval_instspc_tmp) || fval_instspc_tmp > 0)
  [fval_instspc_traj, f_constrinstspc_traj] = cds_constr_installspace( ...
    R, Traj_0.X, Set, Structure, JP, Q, [2e3;3e3]);
  mininstspcdist_all(i_ar) = f_constrinstspc_traj;
  if fval_instspc_traj > 0
    fval = fval_instspc_traj; % Normierung auf 2e3 bis 3e3 -> bereits in Funktion
    constrvioltext = sprintf(['Verletzung des zulässigen Bauraums in Traj.', ...
      'Schlimmstenfalls %1.1f mm draußen.'], 1e3*f_constrinstspc_traj);
    continue
  end
end
%% Arbeitsraum-Hindernis-Kollisionsprüfung für Trajektorie
if ~isempty(Set.task.obstacles.type)
  [fval_obstcoll_traj, coll_obst_traj, f_constr_obstcoll_traj] = cds_constr_collisions_ws( ...
    R, Traj_0.X, Set, Structure, JP, Q, [1e3;2e3]);
  if fval_obstcoll_traj > 0
    fval = fval_obstcoll_traj; % Normierung auf 1e3 bis 2e3 -> bereits in Funktion
    constrvioltext = sprintf(['Arbeitsraum-Kollision in %d/%d Traj.-Punkten. ', ...
      'Schlimmstenfalls %1.1f mm in Kollision.'], sum(any(coll_obst_traj,2)), ...
      size(coll_obst_traj,1), f_constr_obstcoll_traj);
    continue
  end
end
%% Fertig. Bis hier wurden alle Nebenbedingungen geprüft.
fval = 1e3;
constrvioltext = 'i.O.';
end
