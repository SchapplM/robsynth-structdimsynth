% Nebenbedingungen für Roboter-Maßsynthese berechnen (Teil 1: Positions-
% ebene). Entspricht Strafterm aus Nebenbedingungsverletzung
% 
% Eingabe:
% R
%   Matlab-Klasse für zu optimierenden Roboter (SerRob/ParRob)
% Traj_0
%   Endeffektor-Trajektorie (bezogen auf Basis-KS)
% Set
%   Einstellungen des Optimierungsalgorithmus (aus cds_settings_defaults.m)
% Structure
%   Eigenschaften der Roboterstruktur (aus cds_gen_robot_list.m)
% 
% Ausgabe:
% fval
%   Zielfunktionswert, der im PSO-Algorithmus minimiert wird. Entspricht
%   Strafterm in der Fitnessfunktion bei Verletzung der Nebenbedingungen
%   Werte:
%   1e3: Keine Verletzung der Nebenbedingungen. Alles i.O.
%   1e3...1e5: Nicht belegt (siehe cds_fitness)
%   1e5...2e5: Arbeitsraum-Hindernis-Kollision in Einzelpunkten
%   2e5...3e5: Bauraumverletzung in Einzelpunkten
%   3e5...4e5: Selbstkollision in Einzelpunkten
%   4e5...5e5: Gestell ist wegen Schubgelenken zu groß (nach IK erkannt)
%   5e5...9e5: Gelenkwinkelgrenzen in Einzelpunkten
%   9e5...1e6  Singularität in Eckpunkten (trotz lösbarer IK)
%   1e6...1e7: IK in Einzelpunkten nicht lösbar
%   1e7...1e8: Geometrie nicht plausibel lösbar (2: Reichweite PKM-Koppelpunkte)
%   1e8...1e9: Geometrie nicht plausibel lösbar (1: Schließen PKM-Ketten)
% QE_all (Anz. Eckpunkte x Anz. Gelenke x Anz. Konfigurationen)
%   Gelenkpositionen des Roboters (für PKM auch passive Gelenke)
%   für alle Eckpunkte im Arbeitsraum und für alle gefundenen
%   IK-Konfigurationen
% Q0
%   Erste Gelenkkonfiguration der Trajektorie (als Startwert für zukünftige
%   Berechnung der IK)
% constrvioltext [char]
%   Text mit Zusatzinformationen, die beim Aufruf der Fitness-Funktion
%   ausgegeben werden

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [fval,QE_all,Q0,constrvioltext] = cds_constraints(R, Traj_0, Set, Structure)
Q0 = NaN(1,R.NJ);
QE_all = Q0;
%% Geometrie auf Plausibilität prüfen (1)
if R.Type == 0 % Seriell
  % Prüfe, ob alle Eckpunkte der Trajektorie im Arbeitsraum des Roboters liegen
  dist_max = R.reach();
  dist_exc_tot = NaN(size(Traj_0.XE,1),1);
  for i = 1:size(Traj_0.XE,1)
    dist_i = norm(Traj_0.XE(i,1:3));
    dist_exc_tot(i) = dist_max-dist_i;
  end
  if any(dist_exc_tot < 0)
    % Mindestens ein Punkt überschreitet die maximale Reichweite des Roboters
    f_distviol = -min(dist_exc_tot)/dist_max; % maximale Abstandsverletzung (relativ zu Maximalreichweite)
    % Werte sind typischerweise zwischen 0 und 100. Je kleiner der Roboter,
    % desto größer der Wert; das regt dann zur Vergrößerung des Roboters an.
    f_distviol_norm = 2/pi*atan((f_distviol)); % 1->0.5; 10->0.94
    %  normiere auf 1e8 bis 1e9
    fval = 1e8*(1+9*f_distviol_norm);
    constrvioltext = sprintf('Roboter zu kurz. Es fehlen %1.2f m bei einer Reichweite von max. %1.2f m (%1.0f%%).', -min(dist_exc_tot), dist_max,100*f_distviol);
    return
  end
else % PKM
  % Prüfe, ob die Ketten sich überhaupt schließen können (sind die Beine lang
  % genug um sich zu den Koppelpunkten zu verbinden.
  % Dieses Maß ist nur eine Eigenschaft der Kinematik, unabhängig von der
  % gewünschten Trajektorie
  d_base = 2*R.DesPar.base_par(1);
  d_platf = 2*R.DesPar.platform_par(1);
  l_max_leg = R.Leg(1).reach(); % Annahme: Symmetrischer Roboter; alle Beine max. gleich lang

  l_legtooshort = abs(d_base - d_platf)/2 - l_max_leg; % Fehlende Länge jedes Beins (in Meter)
  if l_legtooshort > 0
    f_legtooshort = l_legtooshort/l_max_leg; % fehlende Beinlänge (relativ zu Maximalreichweite)
    % Verhältnis im Bereich 1 bis 100
    f_distviol_norm = 2/pi*atan((f_legtooshort)); % 1->0.5; 10->0.94
    %  normiere auf 1e8 bis 1e9
    fval = 1e8*(1+9*f_distviol_norm);
    constrvioltext = sprintf('Beinkette mit max. Länge %1.2fm zu kurz für Plattform. Es fehlen max. %1.2fm.', l_max_leg, l_legtooshort);
    return
  end
end

%% Geometrie auf Plausibilität prüfen (2)
if R.Type == 0 % Seriell
  % Entfällt. Nur eine Prüfung für serielle Roboter
else % PKM
  % Berechne die Position der Koppelpunkte für die vorgesehenen Eckpunkte der
  % Trajektorie. Dieses Maß wird durch die Trajektorie bestimmt.
  % TODO: Funktioniert noch nicht bei Aufgabenredundanz.
  dist_exc_tot = NaN(size(Traj_0.XE,1),R.NLEG);
  T_P_P_E = R.T_P_E;
  for i = 1:R.NLEG
    % Transformationen für die Beinkette i
    T_0_0i = R.Leg(i).T_W_0;
    r_P_P_Bi = R.r_P_B_all(:,i);
    for j = 1:size(Traj_0.XE,1)
      % EE-Transformation für Bahnpunkt j
      T_0_Ej = R.x2t(Traj_0.XE(j,:)');
      % Position des Plattform-Koppelpunktes der Beinkette i für den
      % Bahnpunkt j (und die dafür vorgesehene Orientierung)
      rh_0i_0i_Bij = invtr(T_0_0i)*T_0_Ej*invtr(T_P_P_E)*[r_P_P_Bi;1];
      dist_j = norm(rh_0i_0i_Bij);
      dist_exc_tot(j,i) = l_max_leg-dist_j;
    end
  end
  if any(dist_exc_tot(:) < 0)
    % Mindestens ein Punkt überschreitet die maximale Reichweite einer Beinkette
    f_distviol = -min(dist_exc_tot(:))/l_max_leg; % maximale Abstandsverletzung (relativ zu Maximalreichweite)
    % Werte sind typischerweise zwischen 0 und 100. Je kleiner die Beinkette,
    % desto größer der Wert; das regt dann zur Vergrößerung des Roboters an.
    f_distviol_norm = 2/pi*atan((f_distviol)); % 1->0.5; 10->0.94
    %  normiere auf 1e7 bis 1e8
    fval = 1e7*(1+9*f_distviol_norm);
    constrvioltext = sprintf('Beinkette mit max. Länge %1.2fm zu kurz für Bahnpunkte. Es fehlen max. %1.2fm.', ...
      l_max_leg, -min(dist_exc_tot(:)));
    return
  end
end

%% Inverse Kinematik für Eckpunkte der Trajektorie berechnen
% Einstellungen für IK. Immer feine Toleranz. Gröbere Toleranz für
% zweiten bis letzten Punkt bringen nur sehr geringe Rechenzeitersparnis.
% Dafür ist das Ergebnis teilweise nicht mehr reproduzierbar (bei Lösung
% am Rand des zulässigen Bereichs)
s = struct('Phit_tol', 1e-9, 'Phir_tol', 1e-9, ...
  'normalize', false, ... % Keine Winkel-Normalisierung (für einige Nebenbedingungen schädlich)
  'rng_seed', 0); % damit die Ergebnisse exakt reproduzierbar werden.
if Set.task.profile ~= 0 % Normale Trajektorie mit stetigem Zeitverlauf
  % Nur Berechnung der Eckpunkte zur Prüfung.
  s.retry_limit = 20;
else % Nur Eckpunkte
  % Eckpunkte haben keinen direkten Bezug zueinander und bilden die
  % Trajektorie. Da keine Traj. berechnet wird, kann hier mehr Aufwand
  % betrieben werden (besonders bei seriellen Robotern auch notwendig).
  s.retry_limit = 50;
  s.n_max = 5000;
end
if R.Type == 0 % Seriell
  qlim = R.qlim;
  qref = R.qref;
  Phi_E = NaN(size(Traj_0.XE,1), sum(Set.task.DoF));
  condJik = NaN(size(Traj_0.XE,1), 1);
  QE = NaN(size(Traj_0.XE,1), R.NQJ);
  % Variable zum Speichern der Gelenkpositionen (für Kollisionserkennung)
  JPE = NaN(size(Traj_0.XE,1), R.NL*3);
else % PKM
  qlim = cat(1,R.Leg(:).qlim);
  qref = cat(1,R.Leg(:).qref);
  nPhi = R.I2constr_red(end);
  Phi_E = NaN(size(Traj_0.XE,1), nPhi);
  condJik = NaN(size(Traj_0.XE,1), R.NLEG);
  QE = NaN(size(Traj_0.XE,1), R.NJ);
  % Abbruch der IK-Berechnung, wenn eine Beinkette nicht erfolgreich war.
  % Dadurch wesentlich schnellerer Durchlauf der PKM-IK
  s_par = struct('abort_firstlegerror', true);
  JPE = NaN(size(Traj_0.XE,1), (R.NL-1+R.NLEG)*3);
end
n_jic = 30;
fval_jic = NaN(1,n_jic);
calctimes_jic = NaN(2,n_jic);
constrvioltext_jic = cell(n_jic,1);
% IK-Statistik (für Aufgabenredundanz). Absolute Verbesserung von
% Zielkriterien gegenüber der nicht-redundanten Kinematik
arikstats_jic = NaN(size(Traj_0.XE,1),n_jic);
Q_jic = NaN(size(Traj_0.XE,1), R.NJ, n_jic);
fval_jic_old = fval_jic;
constrvioltext_jic_old = constrvioltext_jic;
Q_jic_old = Q_jic;
JP_jic = NaN(n_jic, size(JPE,2)); % zum späteren Prüfen der IK-Konfigurationen und deren Auswirkungen
% Wenn Grenzen auf unendlich gesetzt sind, wähle -pi bis pi für Startwert
qlim_norm = qlim;
qlim_norm(isinf(qlim_norm)) = sign(qlim_norm(isinf(qlim_norm)))*pi;
% Nehme für Drehgelenke nicht die Grenzen aus qlim für Anfangswert, da
% diese für die Position eine Spannweite angeben und keine absoluten
% Winkel. Wähle komplett zufällige Winkel als Referenz für den Start
qlim_norm(R.MDH.sigma==0,:) = repmat([-pi, +pi], sum(R.MDH.sigma==0),1);
qlim_range = qlim(:,2)-qlim(:,1);
qlim_range_norm = qlim_range;
qlim_range_norm(isinf(qlim_range)) = 2*pi;
task_red = R.Type == 0 && sum(R.I_EE_Task) < R.NJ || ... % Seriell: Redundant wenn mehr Gelenke als Aufgaben-FG
           R.Type == 2 && sum(R.I_EE_Task) < sum(R.I_EE); % Parallel: Redundant wenn mehr Plattform-FG als Aufgaben-FG
if task_red
  ar_loop = 1:2; % Aufgabenredundanz liegt vor. Zusätzliche Schleife
else
  ar_loop = 1; % Keine Aufgabenredundanz. Nichts zu berechnen.
end
% Bestimme zufällige Anfangswerte für Gelenkkonfigurationen.
% Benutze Gleichverteilung und kein Latin Hypercube (dauert zu lange).
Q0_lhs = repmat(qlim_norm(:,1), 1, n_jic) + ...
  rand(R.NJ, n_jic) .* repmat(qlim_norm(:,2)-qlim_norm(:,1), 1, n_jic);
for jic = 1:n_jic % Schleife über IK-Konfigurationen (30 Versuche)
  Phi_E(:) = NaN; QE(:) = NaN; % erneut initialisieren wegen jic-Schleife.
  if jic == 1 && all(~isnan(qref)) && any(qref~=0) % nehme Referenz-Pose (kann erfolgreiche gespeicherte Pose bei erneutem Aufruf enthalten)
    q0 = qref; % Wenn hier nur Nullen stehen, werden diese ignoriert.
  else
    % Zufällige Anfangswerte geben vielleicht neue Konfiguration.
    q0 = Q0_lhs(:,jic);
  end
  % Anpassung der IK-Anfangswerte für diesen Durchlauf der IK-Konfigurationen.
  % Versuche damit eine andere Konfiguration zu erzwingen
  if fval_jic(1) > 1e6
    % IK hat beim ersten Mal schon nicht funktioniert (dort werden aber
    % zufällige Neuversuche gemacht). Andere Anfangswerte sind zwecklos.
    break;
  end

  if jic > n_jic/3 && jic < 2*n_jic/3 % zwischen ein Drittel und zwei Drittel der Versuche
    % Setze die Anfangswerte (für Schubgelenke) ganz weit nach "links"
    q0(R.MDH.sigma==1) = q0(R.MDH.sigma==1) - 0.5*rand(1)*...
      (qlim_norm(R.MDH.sigma==1,2)-qlim_norm(R.MDH.sigma==1,1));
  elseif jic >= 2*n_jic/3 % Letztes Drittel der Versuche
    % Anfangswerte weit nach rechts
    q0(R.MDH.sigma==1) = q0(R.MDH.sigma==1) + 0.5*rand(1)*...
      (qlim_norm(R.MDH.sigma==1,2)-qlim_norm(R.MDH.sigma==1,1));
  end
  % Normalisiere den Anfangswert (außerhalb [-pi,pi) nicht sinnvoll).
  % (Betrifft nur Fall, falls Winkelgrenzen groß gewählt sind)
  q0(R.MDH.sigma==0) = normalize_angle(q0(R.MDH.sigma==0));
  % Übertrage in Variable für Menge mehrerer möglicher Anfangswerte.
  % hiermit theoretisch auch mehrere Anfangswerte auf einmal vorgebbar.
  % Wird benutzt, damit die Ergebnisse vorheriger Punkte ausprobiert
  % werden können
  Q0 = q0;

  % Berechne die Inverse Kinematik. Im Fall von Aufgabenredundanz
  % IK-Aufruf und Nebenbedingung zweimal testen: Einmal mit normaler IK
  % ohne Optimierung (schnell), einmal mit darauf aufbauender Nullraum-
  % Optimierung (etwas langsamer). Dadurch bessere Einhaltung von Nebenbed.
  for i_ar = ar_loop % 1=ohne Nebenopt.; 2=mit
  t1 = tic();
  if Set.general.debug_calc && i_ar == 2
    fval_jic_old(jic) = fval_jic(jic);
    Q_jic_old(:,:,jic) = Q_jic(:,:,jic);
    constrvioltext_jic_old{jic} = constrvioltext_jic{jic};
  end
  if i_ar == 2 && ... % Optimierung von Nebenbedingungen nur, ...
      fval_jic(jic) > 1e6 % ... falls normale IK erfolgreich war.
    break; % sonst ist die zweite Iteration nicht notwendig.
  end
  constrvioltext_jic{jic} = 'i.O.'; % hier zurücksetzen. Berechne Nebenbedingungen ab hier neu.
  % IK für alle Eckpunkte
  for i = 1:size(Traj_0.XE,1)
    if Set.task.profile ~= 0 % Trajektorie wird in cds_constraints_traj berechnet
      if i == 1 % erster berechneter Wert und Startpunkt der Trajektorie
      elseif i == 2 % zweiter berechneter Wert
        % Annahme: Weniger Neuversuch der IK. Wenn die Gelenkwinkel zufällig neu
        % gewählt werden, springt die Konfiguration voraussichtlich. Dann ist
        % die Durchführung der Trajektorie unrealistisch. Kann nicht zu
        % Null gewählt werden, da die Einzelpunkt-IK nicht immer gut kon-
        % vergiert. Bei 0 werden teilweise funktionierende Roboter wieder verworfen.
        s.retry_limit = 15;
      end
    end
    if i_ar == 1 % IK ohne Optimierung von Nebenbedingungen
      Stats = struct('coll', false); %#ok<NASGU> % Platzhalter-Variable
      if R.Type == 0 % Seriell
        [q, Phi, Tc_stack, Stats] = R.invkin2(R.x2tr(Traj_0.XE(i,:)'), Q0, s);
        nPhi_t = sum(R.I_EE_Task(1:3));
        ik_res_ik2 = all(abs(Phi(1:nPhi_t))<s.Phit_tol) && ...
                     all(abs(Phi(nPhi_t+1:end))<s.Phir_tol);
        condJik(i) = Stats.condJ(1+Stats.iter);
      else % PKM
        Q0_mod = Q0;
        Q0_mod(R.I1J_LEG(2):end,end) = NaN; % Für Beinkette 2 Ergebnis von BK 1 nehmen (dabei letzten Anfangswert für BK 2 und folgende verwerfen)
        [q, Phi, Tc_stack, Stats] = R.invkin2(Traj_0.XE(i,:)', Q0_mod, s, s_par); % kompilierter Aufruf
        % Rechne kinematische Zwangsbedingungen nach. Ist für Struktur-
        % synthese sinnvoll, falls die Modellierung unsicher ist.
        if any(strcmp(Set.optimization.objective, 'valid_act')) || Set.general.debug_calc
          [~,Phi_test] = R.constr1(q, Traj_0.XE(i,:)');
          if all(abs(Phi) < 1e-6) && any(abs(Phi_test) > 1e-3)
            if Set.general.matfile_verbosity > 0
              save(fullfile(fileparts(which('structgeomsynth_path_init.m')), ...
                'tmp', 'cds_constraints_constr1error.mat'));
            end
            cds_log(-1, sprintf(['[constraints] Kinematische Zwangsbedingungen ', ...
              'in vollständiger Form werden nicht erfüllt, in reduzierter Form aber schon']));
            Phi_E(:) = inf; % Dadurch schlechtestmöglicher Ergebniswert
            QE(i,:) = q;
            break; % keine weiteren Punkte prüfen
          end
        end
        ik_res_ik2 = (all(abs(Phi(R.I_constr_t_red))<s.Phit_tol) && ...
            all(abs(Phi(R.I_constr_r_red))<s.Phir_tol));% IK-Status Funktionsdatei
        for ll=1:R.NLEG, condJik(i,ll) = Stats.condJ(1+Stats.iter(ll),ll); end
        Phi_Leg1 = Phi(1:sum(R.I_EE_Task)); % Zwangsbed. für erste Beinkette (aufgabenredundant)
        if ~ik_res_ik2 && task_red && all(abs(Phi_Leg1)<s.Phit_tol) % Vereinfachung: Toleranz transl/rot identisch.
          % Die Einzelbeinketten-IK für die erste Beinkette war erfolgreich, 
          % aber nicht für die weiteren Beinketten. Mögliche Ursache
          % ist die freie Drehung der Plattform durch die erste Beinkette.
          % Erneuter Versuch mit der IK für alle Beinketten gemeinsam.
          % Bei genanntem Abbruch sind nicht funktionierende Beinketten
          % direkt NaN. Dafür andere Werte einsetzen.
          q(isnan(q)) = Q0(isnan(q),1);
          Q0_v2 = [q,Q0];
          [q, Phi, Tc_stack, Stats] = R.invkin4(Traj_0.XE(i,:)', Q0_v2, s);
          condJik(i,:) = Stats.condJ(1+Stats.iter);
%           if all(abs(Phi)<s.Phit_tol)
%             cds_log(3, sprintf(['[constraints] jic=%d, i=%d. IK-Berechnung mit invkin2 ', ...
%               'fehlgeschlagen für eine Beinkette, mit invkin4 erfolgreich.'], jic, i));
%           end
        end
      end
      if R.Type == 2 && Set.general.debug_calc
        [~, Phi_debug, ~] = R.invkin_ser(Traj_0.XE(i,:)', Q0_mod, s, s_par); % Klassenmethode
        ik_res_iks = (all(abs(Phi_debug(R.I_constr_t_red))<s.Phit_tol) && ... 
            all(abs(Phi_debug(R.I_constr_r_red))<s.Phir_tol)); % IK-Status Klassenmethode
        if ik_res_ik2 ~= ik_res_iks % Vergleiche IK-Status (Erfolg / kein Erfolg)
          if Set.general.matfile_verbosity > 0
            save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_ik_error_debug.mat'));
          end
          % Mögliche Ursache: Mehr Glück mit Zufallszahlen für Anfangswert.
          % Das darf nicht allzu oft passieren. Die Zufallszahlen sollten
          % eigentlich gleich gebildet werden.
          cds_log(-1, sprintf(['[constraints] IK-Berechnung mit Funktionsdatei hat anderen ', ...
            'Status (%d) als Klassenmethode (%d).'], ik_res_ik2, ik_res_iks));
        elseif ik_res_iks % beide IK erfolgreich
          % Dieser Test wird vorerst nicht weiter verfolgt (Ergebnisse nicht
          % identisch wegen unterschiedlicher Zufallszahlen)
%         ik_test_q = q - q_debug;
%         ik_test_Tcstack = Tc_stack - Tc_stack_debug;
%         if Set.general.matfile_verbosity > 0
%           save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_ikfkin_error_debug.mat'));
%         end
%         if max(abs(ik_test_Tcstack(:))) > 1e-2
%           warning('Tc_stack zwischen invkin2 und invkin_ser passen nicht. Max. Fehler: %1.1e', max(abs(ik_test_Tcstack(:))));
%         end
%         if max(abs(ik_test_q(:))) > 1e-2
%           warning('q zwischen invkin2 und invkin_ser passen nicht. Max. Fehler: %1.1e', max(abs(ik_test_q(:))));
%         end
        end
      end
    end
    % zweiter Durchlauf. Benutze Optimierung, da vorher Verletzung von 
    % Nebenbedingungen. Annahme: Optimierung in IK gut für Nebenbedingungen.
    % Auch für ersten Bahnpunkt machen (damit bestmöglicher Startwert für
    % Trajektorien-IK in cds_constraints_traj).
    if i_ar == 2% || ... % Bei AR IK erneut mit Optimierung durchführen
%        ... % Erster Bahnpunkt nur, wenn normale IK erfolgreich (und AR vorhanden ist). TODO: Warum nochmal genau?
%         i_ar == 1 && i == 1 && all(~isnan(q)) && ik_res_ik2 && task_red
      % Aufgabenredundanz. Benutze anderen IK-Ansatz (Reziproke
      % Euler-Winkel; gemeinsame Modellierung aller Beinketten)
      % Berechne trotzdem zuerst die einfache IK für jede Beinkette
      % einzeln. Mit dessen Ergebnis dann nur noch Nullraumbewegung zum
      % Optimum der Nebenbedingungen
      if i_ar == 2 % Startwert bei vorheriger Lösung. Dadurch reine Nullraumbewegung
        q0_arik = QE(i,:)';
      else
        q0_arik = q; % Ergebnis der normalen IK direkt von oben
      end
      % Einstellungs-Struktur für Optimierungs-IK vorbereiten
      s4 = s;
      s4.Phit_tol = 1e-8; s4.Phir_tol = 1e-8; % feine Toleranz (für Nullraumbewegung)
      s4.scale_lim = 0; % Überschreitung von Grenzen zulassen
      s4.retry_limit = 0;
      if Set.optimization.constraint_collisions
        s4.avoid_collision_finish = true; % Am Ende versuchen, Kollisionen aufzulösen
        s4.n_max = 1500; %erlaube mehr Versuch zur finalen Kollisionsvermeidung
      end
      % Nebenbedingung: Optimiere die Konditionszahl (ist fast immer gut)
      if R.Type == 0 % Seriell
        s4.wn = [0;0;1;0;0];
      else % PKM
        s4.wn = [0;0;0;1;0;0]; % PKM-Jacobi, nicht IK-Jacobi
      end
      % Setze die Einstellungen und Nebenbedingungen so, dass sich das
      % Ergebnis bestmöglich verändert.
      if i_ar == 2 && fval_jic(jic) > 5e5 && fval_jic(jic) < 9e5
        % Der vorherige Ausschlussgrund war eine zu große Winkelspannweite.
        % Als IK-Nebenbedingung sollte die Winkelspannweite verkleinert
        % werden
        % Prüfe vorher, ob dieser Punkt verantwortlich für die
        % Überschreitung war. Benutze die Grenzen basierend auf Spannweite.
        qlim_alt = repmat(mean(QE(:,:),1)',1,2)+[-qlim_range,qlim_range]/2;
        if i~=1 && all(QE(i,:)' > qlim_alt(:,1)) && all(QE(i,:)' < qlim_alt(:,2))
          % Dieser Punkt war nicht kritisch. Neuberechnung bringt nichts.
          % Ersten Punkt immer optimieren
          continue
        end
        if R.Type == 0 % Seriell
          s4.wn = zeros(5,1);
        else % PKM
          s4.wn = zeros(6,1);
        end
        s4.wn(1) = 1; % quadratische Funktion für Gelenkgrenzen (Startwinkel bereits außerhalb der Grenzen)
        % Setze die Gelenkwinkel-Grenzen neu. Annahme: Die absoluten Werte
        % der Winkel sind nicht wichtig. Es kommt auf die Spannweite an.
        % Lasse Schubgelenke so, wie sie sind. Annahme: Werden schon so
        % kurz wie möglich gewählt (wegen Einfluss auf Masse/Dynamik)
        qlim_neu = qlim;
        qlim_neu(R.MDH.sigma==0,:) = repmat(mean(QE(1:i,R.MDH.sigma==0),1)',1,2)+...
          [-qlim_range(R.MDH.sigma==0), qlim_range(R.MDH.sigma==0)]/2;
        % qlim_neu(R.MDH.sigma==1,:) = qlim(R.MDH.sigma==1,:); % Schubgelenke zurücksetzen
        if R.Type == 0 % Seriell
          R.qlim = qlim_neu;
        else % PKM
          for kk = 1:R.NLEG, R.Leg(kk).qlim = qlim_neu(R.I1J_LEG(kk):R.I2J_LEG(kk),:); end
        end
        qlim = qlim_neu;
        % Zwinge den Startwert in die neuen Grenzen (auf 5% innerhalb).
        % Hat oft zur Folge, dass die IK gar nicht mehr konvergiert. Daher
        % doch nicht machen. Entscheidung
        % q0_arik(q0_arik>qlim(:,2)) = qlim(q0_arik>qlim(:,2))-0.05*qlim_range(q0_arik>qlim(:,2));
        % q0_arik(q0_arik<qlim(:,1)) = qlim(q0_arik<qlim(:,1))+0.05*qlim_range(q0_arik<qlim(:,1));
        % % Dann keine Überschreitung der neuen Grenzen mehr zulassen
        % s4.scale_lim = 0.7; % Scheint die Erfolgsquote stark zu verschlechtern.
      elseif i_ar == 2 && fval_jic(jic) > 3e5 && fval_jic(jic) < 4e5
        % Der vorherige Ausschlussgrund war eine Kollision.
        % Kollisionsvermeidung als Optimierung
        % Prüfe vorher, ob dieser Punkt ausschlaggebend für die Kollision
        % war. Wenn nicht, kann er übersprungen werden und die bestehende
        % Lösung wird genutzt.
        if i~=1 && ~any(coll_self(i,:)) % Die Kollision bezog sich nicht auf diesen Eckpunkt
          continue
        end
        if R.Type == 0 % Seriell
          s4.wn = zeros(5,1);
          s4.wn(4) = 1;
        else % PKM
          s4.wn = zeros(6,1);
          s4.wn(5) = 1;
        end
        if ~isempty(Set.task.installspace.type)
          % Vorsorglich auch Optimierung nach Bauraumgrenzen aktivieren.
          % Bei Kollisionsvermeidung größtmöglicher Abstand von Segmenten
          % zueinander führt tendenziell zu Bauraumüberschreitung.
          if R.Type == 0 % Seriell
            s4.wn(5) = 1;
          else % PKM
            s4.wn(6) = 1;
          end
          s4.installspace_thresh = 0.05; % Nur bei geringem Abstand aktiv
        end
      elseif i_ar == 2 && fval_jic(jic) > 2e5 && fval_jic(jic) < 3e5
        % Der vorherige Ausschlussgrund war eine Bauraumverletzung.
        if R.Type == 0 % Seriell
          s4.wn = zeros(5,1);
          s4.wn(5) = 1;
        else % PKM
          s4.wn = zeros(6,1);
          s4.wn(6) = 1;
        end
        s4.installspace_thresh = 0.1500; % Etwas höherer Abstand zur Aktivierung der Funktion
      elseif i_ar == 2 && fval_jic(jic) == 1e3 % Vorher erfolgreich
        if any(strcmp(Set.optimization.objective, 'colldist'))
          s4.wn(5) = 1;
          s4.collbodies_thresh = 9; % 10 mal größere Kollisionskörper im Warnbereich für permanente Aktivierung
        end
      end
      if i == 1
        % nur für ersten Traj.-Punkt die Kondition verbessern
        % TODO: Mehrere Zielfunktionen neigen zum oszillieren.
        if R.Type == 0 % Seriell
          s4.wn(3) = 0.5;
        else % PKM
          s4.wn(4) = 0.5; % PKM-Jacobi, nicht IK-Jacobi
        end
      end
      % IK für Nullraumbewegung durchführen
      if R.Type == 0
        [q, Phi, Tc_stack, Stats] = R.invkin2(R.x2tr(Traj_0.XE(i,:)'), q0_arik, s4);
      else
        [q, Phi, Tc_stack, Stats] = R.invkin4(Traj_0.XE(i,:)', q0_arik, s4);
      end
      % Benutze nicht die sehr strenge Grenze von Phit_tol von oben, da aus
      % numerischen Gründen diese teilweise nicht erreicht werden kann
      ik_res_ikar = all(abs(Phi) < 1e-6);
      if Set.general.debug_calc && all(abs(q-q0_arik) < 1e-9)
        cds_log(-1, sprintf(['[constraints] Konfig %d, Eckpunkt %d, Iter. %d: IK-Berechnung mit Aufgabenredundanz ', ...
          'hat gleiches Ergebnis wie ohne (max delta q = %1.1e).'], jic, i, i_ar, max(abs(q-q0_arik))));
        % TODO: Eventuell liegt hier noch ein Implementierungsfehler vor.
        % Alternativ kann ein Fehler im Matlab Coder vorliegen.
%         R.fill_fcn_handles(false);
%         % matlabfcn2mex({'S6RRRRRR10V2_invkin_eulangresidual'},false,false,true);
%         [q_nomex, Phi_nomex, Tc_stack_nomex, Stats_nomex] = R.invkin2(R.x2tr(Traj_0.XE(i,:)'), q0_arik, s4);
%         if Stats.iter ~= Stats_nomex.iter
%           warning('Implementierung mit/ohne mex ist anders');
%         end
%         R.fill_fcn_handles(true);
%         [q, Phi, Tc_stack, Stats] = R.invkin2(R.x2tr(Traj_0.XE(i,:)'), q0_arik, s4);
      end
      if ~ik_res_ikar % Keine Lösung gefunden. Sollte eigentlich nicht passieren.
        if s4.scale_lim == 0 && ... % Bei scale_lim kann der Algorithmus feststecken
            ~Stats.coll && ... % Wenn Kollisionsvermeidung aktiv wurde, kann das zum Scheitern führen
            any(Stats.condJ([1,1+Stats.iter]) < 1e3) % Bei singulären Beinketten ist das Scheitern erwartbar
          cds_log(3, sprintf(['[constraints] Konfig %d, Eckpunkt %d: IK-Berechnung ', ...
            'mit Aufgabenredundanz fehlerhaft, obwohl es ohne AR funktioniert ', ...
            'hat. wn=[%s]. max(Phi)=%1.1e. Iter %d/%d'], jic, i, disp_array(s4.wn','%1.1g'), ...
            max(abs(Phi)), Stats.iter, size(Stats.Q,1)-1));
        else
          % Falls neue Grenzen gesetzt wurden, ist die IK eventuell nicht
          % innerhalb der Grenzen lösbar. In diesem Fall hier kein Fehler
        end
        continue % Verwerfe das neue Ergebnis (nehme dadurch das vorherige)
      end
      % Ergebnis der Nullraumoptimierung auswerten und vergleichen.
      % Benutzung der Summe aus Ausgabe nicht möglich (wn verändert sich).
      h_opt_pre  = sum(s4.wn' .* Stats.h(1,2:(1+length(s4.wn))) );
      h_opt_post = sum(s4.wn' .* Stats.h(1+Stats.iter,2:(1+length(s4.wn))) );
      % Die Nebenbedingungen müssen sich verbessern, wenn schon bei einer
      % gültigen Startpose gestartet wurde. Wurde nur mit einer groben
      % Näherung (1e-3) gestartet, kann man die Nebenbedingungen nicht ver-
      % gleichen.
      if ik_res_ikar % Speichern der Debug-Information
        arikstats_jic(i, jic) = h_opt_post - h_opt_pre;
      else
        arikstats_jic(i, jic) = inf; % steht für "Fehlerhafte IK". Annahme, dass es nachher nicht auf "inf" geht.
      end
      if ik_res_ikar && h_opt_post > h_opt_pre + 1e-3 && ...% Toleranz gegen Numerik-Fehler
          Stats.condJ(1) < 1e3 % Wenn die PKM am vorher singulär ist, dann ist die Verschlechterung der Nebenopt. kein Ausschlussgrund für die neue Lösung
        if abs(Stats.h(1+Stats.iter,1)-h_opt_post) < 1e-3 && ... % es lag nicht am geänderten `wn` in der Funktion
            h_opt_post < 1e8 % Es ist kein numerisch großer und ungenauer Wert
          cds_log(3, sprintf(['[constraints] Konfig %d, Eckpunkt %d: IK-Berechnung ', ...
            'mit Aufgabenredundanz hat Nebenoptimierung verschlechtert: ', ...
            '%1.4e -> %1.4e. wn=[%s]'], jic, i, h_opt_pre, h_opt_post, disp_array(s4.wn','%1.1g')));
        end
        continue % Verwerfe das neue Ergebnis (nehme dadurch das vorherige)
        % Zum Debuggen
        if R.Type == 0, I_constr_red = [1 2 3 5 6]; %#ok<UNRCH>
        else,           I_constr_red = R.I_constr_red; end
        figure(2345);clf;
        subplot(3,3,1);
        plot(Stats.condJ(1:Stats.iter));
        xlabel('Iterationen'); grid on;
        ylabel('cond(J)');
        subplot(3,3,2);
        plot([diff(Stats.Q(1:Stats.iter,:));NaN(1,R.NJ)]);
        xlabel('Iterationen'); grid on;
        ylabel('diff q');
        subplot(3,3,3);
        plot([Stats.Q(1:Stats.iter,:);NaN(1,R.NJ)]);
        xlabel('Iterationen'); grid on;
        ylabel('q');
        Stats_Q_norm = (Stats.Q(1:Stats.iter,:)-repmat(qlim(:,1)',Stats.iter,1))./ ...
                        repmat(qlim_range',Stats.iter,1);
        subplot(3,3,4);
        plot([Stats_Q_norm(1:Stats.iter,:);NaN(1,R.NJ)]);
        xlabel('Iterationen'); grid on;
        ylabel('q norm');
        subplot(3,3,5);
        plot([diff(Stats.PHI(1:Stats.iter,I_constr_red));NaN(1,length(I_constr_red))]);
        xlabel('Iterationen'); grid on;
        ylabel('diff Phi');
        subplot(3,3,6);
        plot(Stats.PHI(1:Stats.iter,I_constr_red));
        xlabel('Iterationen'); grid on;
        ylabel('Phi');
        subplot(3,3,7);
        plot(diff(Stats.h(1:Stats.iter,[true,s4.wn'~=0])));
        xlabel('Iterationen'); grid on;
        ylabel('diff h');
        subplot(3,3,8);
        plot(Stats.h(1:Stats.iter,[true,s4.wn'~=0]));
        legend(['w.sum',cellfun(@(s)sprintf('h%d',s),num2cell(find(s4.wn'~=0)),'UniformOutput',false)]);
   
        xlabel('Iterationen'); grid on;
        ylabel('h');
        if any(Stats.maxcolldepth>0) % es sollte eine Kollision gegeben haben
          % Die Ausgabe maxcolldepth wird nur geschrieben, wenn Kollisionen
          % geprüft werden sollten
          subplot(3,3,9);
          plot(Stats.maxcolldepth(1:Stats.iter,:));
          xlabel('Iterationen'); grid on;
          ylabel('Kollisionstiefe (>0 Koll.)');
        elseif R.Type == 0 && s4.wn(5) || R.Type ~= 0 && s4.wn(6)
          subplot(3,3,9);
          plot(Stats.instspc_mindst(1:Stats.iter,:));
          xlabel('Iterationen'); grid on;
          ylabel('Abstand zum Bauraum (>0 draußen)');
        end
        linkxaxes
      end
      % Prüfe, ob IK-Ergebnis eine Kollision hat. Wenn ja, müssen keine
      % weiteren Punkte geprüft werden (schnellere Rechnung). Auswertung der
      % Kollisionskennzahl hierfür nicht möglich (auch Warnbereich > 0)
      if Stats.coll
        % Damit werden für alle noch folgenden Punkte die Ergebnisse des
        % nicht-optimierten Aufrufs genommen (kann Bewertung in Maßsynthese
        % leicht verzerren, ist aber tolerierbar).
        break; 
      end
      % Gleiche Betrachtung bei Bauraumprüfung. Ausgabe wird nur belegt, wenn
      % Kennzahl auch geprüft wird
      if Stats.instspc_mindst(1+Stats.iter,:) > 0
        break;
      end
      % Neue Werte aus der IK wurden nicht verworfen. Schreiben Konditionszahl
      condJik(i,:) = Stats.condJ(1+Stats.iter,:);
    end
    % Normalisiere den Winkel. Bei manchen Robotern springt das IK-Ergebnis
    % sehr stark. Dadurch wird die Gelenkspannweite sonst immer verletzt.
    q(R.MDH.sigma==0) = normalize_angle(q(R.MDH.sigma==0));
    Phi_E(i,:) = Phi;
    QE(i,:) = q;
    if any(abs(Phi(:)) > 1e-2) || any(isnan(Phi))
      constrvioltext_jic{jic} = sprintf('Keine IK-Konvergenz');
      break; % Breche Berechnung ab (zur Beschleunigung der Berechnung)
    end
    % Probiere für den nächsten Punkt die Gelenkwinkel aller bisher
    % berechneter Punkte aus. Dadurch wird eher die gleiche Konfiguration
    % gefunden.
    Q0 = QE(i:-1:1,:)';
    JPE(i,:) = Tc_stack(:,4); % Vierte Spalte ist Koordinatenursprung der Körper-KS
    if Set.general.debug_calc && R.Type == 2
      % Prüfe, ob die ausgegebenen Gelenk-Positionen auch stimmen
      JointPos_all_i_frominvkin = reshape(JPE(i,:)',3,1+R.NLEG+R.NJ);
      Tc_Lges = R.fkine_legs(QE(i,:)');
      JointPos_all_i_fromdirkin = [zeros(3,1), squeeze(Tc_Lges(1:3,4,1:end))];
      % Vergleiche die Positionen. In fkine_legs wird zusätzlich ein
      % virtuelles EE-KS ausgegeben, nicht aber in invkin_ser.
      for kk = 1:R.NLEG
        test_JPE = JointPos_all_i_frominvkin(:,kk+(-1+R.I1J_LEG(kk):R.I2J_LEG(kk))) - ...
        JointPos_all_i_fromdirkin(:,kk*2+(-2+R.I1J_LEG(kk):-1+R.I2J_LEG(kk)));
        if any(abs(test_JPE(:)) > 1e-8)
          if Set.general.matfile_verbosity > 0
            save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_trajjointpos_error_debug.mat'));
          end
          error(['Ausgegebene Gelenkpositionen stimmen nicht gegen direkte ', ...
            'Kinematik. Zeitpunkt %d, Beinkette %d. Max Fehler %1.1e'], i, kk, max(abs(test_JPE(:))));
        end
      end
    end
    % Prüfe, ob für die Berechnung der neuen Konfiguration das gleiche
    % rauskommt. Dann könnte man aufhören. Prüfe das für den ersten Eck-
    % punkt. Entspricht der ersten Berechnung.
    if jic > 1 && i == 1
      % Normiere die möglichen Gelenkwinkel für den ersten Punkt um
      % festzustellen, welche eindeutig sind (Dimensionsunabhängig)
      test_vs_all_prev = repmat(q,1,jic-1)-reshape(squeeze(Q_jic(1,:,1:jic-1)),R.NJ,jic-1);
      % 2pi-Fehler entfernen
      test_vs_all_prev(abs(abs(test_vs_all_prev)-2*pi)<1e-1) = 0; % ungenaue IK ausgleichen
      test_vs_all_prev_norm = test_vs_all_prev ./ repmat(qlim_range_norm,1,size(test_vs_all_prev,2));
      % Prüfe ob eine Spalte gleich ist wie die aktuellen Gelenkwinkel.
      % Kriterium: 3% bezogen auf erlaubte Spannweite
      if any(all(abs(test_vs_all_prev_norm)<3e-2,1))
        fval_jic(jic) = Inf; % damit jic-Iteration nicht weiter gemacht wird
        constrvioltext_jic{jic} = sprintf('Erste Gelenkwinkel identisch zu vorherigem Versuch');
        break;  % Das IK-Ergebnisse für den ersten Eckpunkt gibt es schon. Nicht weiter rechnen.
      end
    end
  end % Schleife über Trajektorienpunkte
  if isinf(fval_jic(jic)) % Aufhören bei Duplikat.
    calctimes_jic(i_ar,jic) = toc(t1);
    continue;
  end
  QE(isnan(QE)) = 0;
  Q_jic(:,:,jic) = QE;
  JP_jic(jic,:) = JPE(1,:);
  Phi_E(isnan(Phi_E)) = 1e6;
  if any(abs(Phi_E(:)) > 1e-2) || ... % Die Toleranz beim IK-Verfahren ist etwas größer
      any(abs(Phi_E(1,:))>1e-8) % Startpunkt für Traj. Hat feine Toleranz, sonst missverständliche Ergebnisse. Konsistent mit Toleranz oben.
    % Nehme die mittlere IK-Abweichung aller Eckpunkte (Translation/Rotation
    % gemischt). Typische Werte von 1e-2 bis 10.
    % Bei vorzeitigem Abbruch zählt die Anzahl der erfolgreichen Eckpunkte
    f_PhiE = mean(abs(Phi_E(:)));
    f_phiE_norm = 2/pi*atan(f_PhiE/0.9e6*35); % Normierung auf 0 bis 1. 0.9e6 -> 0.98
    fval_jic(jic) = 1e6*(1+9*f_phiE_norm); % Normierung auf 1e6 bis 1e7
    % Keine Konvergenz der IK. Weitere Rechnungen machen keinen Sinn.
    constrvioltext_jic{jic} = sprintf(['Keine IK-Konvergenz in Eckwerten. ', ...
      'Untersuchte Eckpunkte: %d/%d. Durchschnittliche ZB-Verl. %1.2e'], ...
      i, size(Traj_0.XE,1), f_PhiE);
    calctimes_jic(i_ar,jic) = toc(t1);
    continue;
  end
  %% Prüfe die Konditionszahl der vollständigen Jacobi-Matrix der inversen Kinematik
  % Wenn die PKM schon bezüglich der Beinketten singulär ist, ist die
  % weitere Rechnung für die Trajektorie sowieso sinnlos
  n_condexc = sum(any(condJik > Set.optimization.condition_limit_sing, 2));
  if n_condexc > 0
    fval = 1e5*(9+n_condexc/size(condJik,1)); % Normierung auf 9e5 bis 1e6
    fval_jic(jic) = fval;
    constrvioltext_jic{jic} = sprintf(['Konditionszahl in IK für %d/%d ', ...
      'Eckpunkte zu groß. max(cond(J))=%1.1e.'], n_condexc, size(condJik,1), max(condJik(:)));
    calctimes_jic(i_ar,jic) = toc(t1);
    continue;
  end

  %% Bestimme die Spannweite der Gelenkkoordinaten (getrennt Dreh/Schub)
  q_range_E = NaN(1, R.NJ);
  q_range_E(R.MDH.sigma==1) = diff(minmax2(QE(:,R.MDH.sigma==1)')');
  q_range_E(R.MDH.sigma==0) = angle_range( QE(:,R.MDH.sigma==0));
  % Bestimme ob die maximale Spannweite der Koordinaten überschritten wurde
  qlimviol_E = qlim_range' - q_range_E;
  I_qlimviol_E = (qlimviol_E < 0);
  if any(I_qlimviol_E) && Set.optimization.check_jointrange_points
    if Set.general.matfile_verbosity > 2
      save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_qviolE.mat'));
    end
    % Bestimme die größte relative Verletzung der Winkelgrenzen
    [fval_qlimv_E, I_worst] = min(qlimviol_E(I_qlimviol_E)./(qlim(I_qlimviol_E,2)-qlim(I_qlimviol_E,1))');
    II_qlimviol_E = find(I_qlimviol_E); IIw = II_qlimviol_E(I_worst);
    fval_qlimv_E_norm = 2/pi*atan((-fval_qlimv_E)/0.3); % Normierung auf 0 bis 1; 2 ist 0.9
    fval = 1e5*(5+4*fval_qlimv_E_norm); % Normierung auf 5e5 bis 9e5
    fval_jic(jic) = fval;
    % Überschreitung der Gelenkgrenzen (bzw. -bereiche). Weitere Rechnungen machen keinen Sinn.
    if R.Type ~= 0
      legnum = find(IIw>=R.I1J_LEG, 1, 'last');
      legjointnum = IIw-(R.I1J_LEG(legnum)-1);
      jointstr = sprintf('; Bein %d, Beingelenk %d', legnum, legjointnum);
    else
      jointstr = '';
    end
    constrvioltext_jic{jic} = sprintf(['Gelenkgrenzverletzung in AR-Eckwerten. ', ...
      'Schlechteste Spannweite: %1.2f/%1.2f (Gelenk %d%s)'], q_range_E(IIw), ...
      qlim(IIw,2)-qlim(IIw,1), IIw, jointstr);
    if 1e4*fval < Set.general.plot_details_in_fitness
      change_current_figure(1000); clf; hold on;
      % Gut-Einträge: Dummy-NaN-Eintrag mit plotten, damit Handle für Legende nicht leer bleibt.
      hdl_iO= plot([find(~I_qlimviol_E),NaN], [QE(:,~I_qlimviol_E)-min(QE(:,~I_qlimviol_E)),NaN(size(QE,1),1)], 'co');
      hdl_niO=plot(find( I_qlimviol_E), QE(:, I_qlimviol_E)-min(QE(:, I_qlimviol_E)), 'bx'); % Kein NaN-Dummy notwendig.
      hdl1=plot(qlim_range', 'r--');
      hdl2=plot([1;size(QE,2)], [0;0], 'm--');
      xlabel('Koordinate Nummer'); ylabel('Koordinate Wert');
      grid on;
      legend([hdl_iO(1);hdl_niO(1);hdl1;hdl2], {'iO-Gelenke', 'niO-Gelenke', 'qmax''', 'qmin''=0'});
      sgtitle(sprintf('Auswertung Grenzverletzung AR-Eckwerte. fval=%1.2e', fval));
    end
    calctimes_jic(i_ar,jic) = toc(t1);
    continue;
%   elseif i_ar == 2 && fval_jic(jic) > 1e3
%     fprintf('Vorher Fehler. Jetzt erfolgreich!\n');
  end
  if Set.general.matfile_verbosity > 2
    save(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_2.mat'));
  end
  % Debug:
  % load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_constraints_2.mat'));

  %% Aktualisiere Roboter für Kollisionsprüfung (geänderte Winkelgrenzen aus IK)
  if Set.optimization.constraint_collisions || ... % Kollisionsprüfung
      ~isempty(Set.task.installspace.type) || ... % Bauraum-Grenzen definiert
      ~isempty(Set.task.obstacles.type) || ... % Arbeitsraum-Objekte definiert
      ~isnan(Set.optimization.base_size_limits(2)) && any(Structure.I_firstprismatic) % Gestell-Schubachsen
    [Structure.collbodies_robot, Structure.installspace_collbodies] = ...
      cds_update_collbodies(R, Set, Structure, QE);
  end
  %% Prüfe Gestell-Durchmesser durch geänderte gestellfeste Führungsschienen
  if ~isnan(Set.optimization.base_size_limits(2)) && ...% Obere Grenze für Gestell-Radius gesetzt
      any(Structure.I_firstprismatic) % Es gibt ein gestellfestes Schubgelenk
    % Finde den effektiven Gestell-Radius heraus (weitesten vom
    % Mittelpunkt entfernte Enden der Führungsschienen)
    I_guidance = Structure.collbodies_robot.type==13;
    pts = [Structure.collbodies_robot.params(I_guidance,1:3); ...
           Structure.collbodies_robot.params(I_guidance,4:6)];
    % Max. Abstand aller Punkte der Führungsschiene von PKM-Basis (Maschinenmitte)
    r_base_eff = max((pts(:,1).^2 + pts(:,2).^2).^0.5);
    % Um so viel ist der Radius zu groß (bezogen auf Grenze). Die Grenze
    % für das Gestell wird in diesem Fall noch vergrößert, da die Führungs-
    % schienen nach oben abstehen.
    fval_rbase = r_base_eff/(Set.optimization.base_size_limits(2) * ...
      Set.optimization.base_tolerance_prismatic_guidance)-1;
    if fval_rbase > 0
      constrvioltext_jic{jic} = sprintf(['Gestell-Radius ist durch Schub', ...
        'achsen-Führungsschienen um %1.0f%% zu groß (%1.1fmm>%1.1fmm).'], ...
        100*fval_rbase, 1e3*r_base_eff, 1e3*1/((fval_rbase+1)/r_base_eff));
      fval_rbase_norm = 2/pi*atan(fval_rbase*3); % Normierung auf 0 bis 1; 100% zu groß ist 0.8
      fval = 1e5*(4+1*fval_rbase_norm); % Normierung auf 4e5 bis 5e5
      fval_jic(jic) = fval;
      calctimes_jic(i_ar,jic) = toc(t1);
      continue;
    end
  end
  %% Anpassung des Offsets für Schubgelenke
  % Hierdurch wird der Ort der Führungsschienen auf der Gelenkachse
  % verschoben. Das beeinflusst sowohl die Bauraumprüfung, als auch die
  % Prüfung auf Selbstkollision.
  if Structure.desopt_prismaticoffset
    % Bei Optimierung des Offsets wird auch bereits die Kollisionsprüfung
    % durchgeführt und die Ergebnisse weiter unten genutzt.
    [fval_coll_tmp, fval_instspc_tmp] = cds_desopt_prismaticoffset(R, ...
      Traj_0.XE, Set, Structure, JPE, QE);
    if R.Type == 0, new_offset=R.DesPar.joint_offset(R.MDH.sigma==1);
    else, new_offset=R.Leg(1).DesPar.joint_offset(R.Leg(1).MDH.sigma==1); end
    cds_log(4, sprintf(['[constraints] Schubgelenk-Offset wurde optimiert. ', ...
      'Ergebnis: %1.1fmm (Start-Konfig. %d)'], 1e3*new_offset, jic));
    % Kollisionskörper müssen nochmal aktualisiert werden (wegen Offset)
    [Structure.collbodies_robot, Structure.installspace_collbodies] = ...
      cds_update_collbodies(R, Set, Structure, QE);
  else
    fval_coll_tmp = NaN; % Keine Berechnung durchgeführt ...
    fval_instspc_tmp = NaN; % ... Erstmalige Berechnung unten erforderlich.
  end
  %% Selbst-Kollisionsprüfung für Einzelpunkte
  if Set.optimization.constraint_collisions && ...
      (isnan(fval_coll_tmp) || fval_coll_tmp > 0) % nutze bereits vorliegende Daten
    [fval_coll, coll_self] = cds_constr_collisions_self(R, Traj_0.XE, Set, Structure, JPE, QE, [3e5;4e5]);
    if fval_coll > 0
      fval_jic(jic) = fval_coll; % Normierung auf 3e5 bis 4e5 bereits in Funktion
      constrvioltext_jic{jic} = sprintf('Selbstkollision in %d/%d AR-Eckwerten.', ...
        sum(any(coll_self,2)), size(coll_self,1));
      if fval_jic_old(jic) > 3e5 && fval_jic_old(jic) < 4e5 && fval_coll > fval_jic_old(jic)+1e-4
        cds_log(3, sprintf(['[constraints] Die Schwere der Kollisionen hat ', ...
          'sich trotz Optimierung vergrößert (Konfig %d)'], jic)); % Gewertet wird die Eindringtiefe, nicht die Anzahl
      end
      calctimes_jic(i_ar,jic) = toc(t1);
      continue;
    elseif i_ar == 2 && fval_jic_old(jic) > 3e5 && fval_jic_old(jic) < 4e5
%       cds_log(3, sprintf('[constraints] Nach Optimierung keine Kollision mehr (Konfig %d).', jic));
    end
  end
  %% Bauraumprüfung für Einzelpunkte
  if ~isempty(Set.task.installspace.type) && ...
      (isnan(fval_instspc_tmp) || fval_instspc_tmp > 0) % nutze bereits vorliegende Daten
    [fval_instspc, f_constrinstspc] = cds_constr_installspace(R, Traj_0.XE, Set, Structure, JPE, QE, [2e5;3e5]);
    if fval_instspc > 0
      fval_jic(jic) = fval_instspc; % Normierung auf 2e5 bis 3e5 -> bereits in Funktion
      constrvioltext_jic{jic} = sprintf(['Verletzung des zulässigen Bauraums in AR-', ...
        'Eckpunkten. Schlimmstenfalls %1.1f mm draußen.'], 1e3*f_constrinstspc);
      calctimes_jic(i_ar,jic) = toc(t1);
      continue;
    end
  end
  %% Arbeitsraum-Hindernis-Kollisionsprüfung für Einzelpunkte
  if ~isempty(Set.task.obstacles.type)
    [fval_obstcoll, coll_obst, f_constr_obstcoll] = cds_constr_collisions_ws(R, Traj_0.XE, Set, Structure, JPE, QE, [1e5;2e5]);
    if fval_obstcoll > 0
      fval_jic(jic) = fval_obstcoll; % Normierung auf 1e5 bis 2e5 -> bereits in Funktion
      constrvioltext_jic{jic} = sprintf(['Arbeitsraum-Kollision in %d/%d AR-Eckwerten. ', ...
        'Schlimmstenfalls %1.1f mm in Kollision.'], sum(any(coll_obst,2)), ...
        size(coll_obst,1), 1e3*f_constr_obstcoll);
      calctimes_jic(i_ar,jic) = toc(t1);
      continue;
    end
  end
  fval_jic(jic) = 1e3; % Bis hier hin gekommen. Also erfolgreich.
  calctimes_jic(i_ar,jic) = toc(t1);
  end % Schleife über IK ohne/mit zusätzlicher Optimierung
end % Schleife über IK-Konfigurationen

% Debuggen der IK-Optimierung (zum Nachvollziehen, ob die Nutzung von
% Redundanz an dieser Stelle überhaupt etwas bringt)
if task_red
  cds_log(3, sprintf(['[constraints] Eckpunkte mit %d Konfigurationen berechnet. Dauer ', ...
    'AR=0 %1.1fs (mean %1.2fs; n=%d): AR=1 %1.1fs (mean %1.2fs; n=%d)'], ...
    size(calctimes_jic,2), sum(calctimes_jic(1,:),'omitnan'), ...
    mean(calctimes_jic(1,:),'omitnan'), sum(~isnan(calctimes_jic(1,:))), ...
    sum(calctimes_jic(2,:),'omitnan'), mean(calctimes_jic(2,:),'omitnan'), ...
    sum(~isnan(calctimes_jic(2,:)))));
  if any(~isnan(arikstats_jic(:)))
    cds_log(3, sprintf(['[constraints] Insgesamt für %d Punkte (von max. %dx%d=%d) die IK nochmal ', ...
      'neu gerechnet. Dabei %d mal mit Verbesserung, %d mal mit Verschlechterung ', ...
      'der Zusatzoptimierung. %d Mal Kein Erfolg der AR-IK. '], sum(~isnan(arikstats_jic(:))), ...
      size(arikstats_jic,1), size(arikstats_jic,2), length(arikstats_jic(:)), ...
      sum(arikstats_jic(:)<0), sum(arikstats_jic(:)>0), sum(isinf(arikstats_jic(:)))));
  end
end
if Set.general.debug_calc && task_red
  % Prüfe, für wie viele Konfigurationen eine Verbesserung eintritt.
  I_besser = fval_jic < fval_jic_old;
  if any(I_besser)
    cds_log(3, sprintf(['[constraints] Durch Aufgabenredundanz Verbesserung ', ...
      'in Einzelpunkt-IK (für %d Start-Konfigurationen). Jetzt %d i.O. Konfig. (anstatt %d)'], ...
      sum(I_besser), sum(fval_jic==1e3), sum(fval_jic_old==1e3)));
    for i = find(I_besser)
      cds_log(3, sprintf('[constraints] Konfig. %d: %1.2e -> %1.2e (%s -> %s)', ...
        i, fval_jic_old(i), fval_jic(i), constrvioltext_jic_old{i}, constrvioltext_jic{i}));
      % Q_jic - Q_jic_old;
    end
  end
  % Gegenprobe: Ist eine Verschlechterung eingetreten?
  I_schlechter = fval_jic > fval_jic_old & ~isinf(fval_jic);
  if any(I_schlechter)
    cds_log(3, sprintf(['[constraints] Durch Aufgabenredundanz Verschlechterung ', ...
      'in Einzelpunkt-IK (für %d Start-Konfigurationen). Mache Rückgängig.'], ...
      sum(I_schlechter)));
    % Benutze wieder die ursprünglichen Ergebnisse ohne Veränderung durch
    % Aufgabenredundanz. Verschlechterung kann passieren, da die Kriterien
    % nach der Gesamtheit aller Punkte gewählt werden.
    for i = find(I_schlechter)
      cds_log(3, sprintf('[constraints] Konfig. %d: %1.2e -> %1.2e (%s -> %s)', ...
        i, fval_jic_old(i), fval_jic(i), constrvioltext_jic_old{i}, constrvioltext_jic{i}));
    end
    fval_jic(I_schlechter) = fval_jic_old(I_schlechter);
    Q_jic(:,:,I_schlechter) = Q_jic_old(:,:,I_schlechter);
    constrvioltext_jic(I_schlechter) = constrvioltext_jic_old(I_schlechter);
  end
  if ~any(Q_jic(:) - Q_jic_old(:))
    cds_log(3, sprintf(['[constraints] Keine Veränderung durch Aufgaben', ...
      'redundanz (%d Konfigurationen)'], size(Q_jic_old,3)));
  end
end
%% IK-Konfigurationen für Eckpunkte auswerten. Nehme besten.
[fval, jic_best] = min(fval_jic);
constrvioltext = constrvioltext_jic{jic_best};

I_iO = find(fval_jic == 1e3);
if ~any(I_iO) % keine gültige Lösung für Eckpunkte
  Q0 = Q_jic(1,:,jic_best); % Gebe nur eine einzige Konfiguration aus
  QE_all = Q_jic(:,:,jic_best);
  % Wenn die IK nicht für alle Punkte erfolgreich war, wurde abgebrochen.
  % Dann stehen nur Nullen im Ergebnis. Schlecht zum Debuggen.
  if all(Q0==0)
    Q0 = QE_all(end,:); % Der letzte Wert wurde oben zuerst berechnet. Ergebnis liegt vor.
    % Setze für die Eckpunkte überall den (falschen) IK-Ergebniswert ein.
    % Macht das Debuggen einfacher (Bild plausibler)
    I_zeros = all(QE_all==0,2);
    QE_all(I_zeros,:) = repmat(Q0,sum(I_zeros),1);
  end
else % Gebe alle gültigen Lösungen aus
  Q0 = reshape(squeeze(Q_jic(1,:,I_iO)),R.NJ,length(I_iO))';
  [Q0_unique] = unique(round(Q0,2), 'rows');
  if size(Q0_unique,1) ~= size(Q0,1)
    cds_log(-1, sprintf('[constraints] Doppelte Konfigurationen als Ergebnis. Darf nicht sein'));
  end

  % Prüfe, welche der IK-Konfigurationen andere Gelenk-Positionen zur Folge haben. Nur diese werden genommen.
  [~,I,~] = unique(round(JP_jic(I_iO,:),5), 'rows', 'first');
  % Reduziere die Ergebnisse. Damit werden IK-Konfigurationen verworfen,
  % die nur rechnerisch eine andere Koordinate haben, aber kein Umklappen
  % darstellen
  Q0 = Q0(I,:);
  I_iO = I_iO(I);  
  % Ausgabe der IK-Werte für alle Eckpunkte. Im weiteren Verlauf der
  % Optimierung benötigt, falls keine Trajektorie berechnet wird.
  QE_all = Q_jic(:,:,I_iO);
  % Debug: Zeige die verschiedenen Lösungen an
  if 1e4*fval < Set.general.plot_details_in_fitness
    change_current_figure(2000);clf;
    for k = 1:length(I_iO)
      subplot(floor(ceil(length(I_iO))), ceil(sqrt(length(I_iO))), k);
      view(3); axis auto; hold on; grid on;
      plotmode = 1; % Strichzeichnung
      if R.Type == 0 % Seriell
        s_plot = struct( 'ks', [], 'straight', 1, 'mode', plotmode);
        R.plot( Q0(k,:)', s_plot);
      else % PKM
        s_plot = struct( 'ks_legs', [], 'ks_platform', [], ...
          'straight', 1, 'mode', plotmode);
        R.plot( Q0(k,:)', Traj_0.X(1,:)', s_plot);
      end
      title(sprintf('Konfiguration %d', k));
    end
  end
end
