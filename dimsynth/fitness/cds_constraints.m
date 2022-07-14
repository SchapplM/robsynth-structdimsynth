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
%   4e5...4.25e5: Schubzylinder geht ... (symmetrisch für PKM)
%   4.25e5...4.5e5: Schubzylinder geht zu weit nach hinten weg (nach IK erkannt)
%   4.5e5...5e5: Gestell ist wegen Schubgelenken zu groß (nach IK erkannt)
%   5e5...6e5: Gelenkwinkelgrenzen (Absolut) in Einzelpunkten
%   6e5...7e5: Gelenkwinkelgrenzen (Spannweite) in Einzelpunkten
%   7e5...8e5  Jacobi-Grenzverletzung in Eckpunkten (trotz lösbarer IK)
%   8e5...9e5  Jacobi-Singularität in Eckpunkten (trotz lösbarer IK)
%   9e5...1e6  IK-Singularität in Eckpunkten (trotz lösbarer IK)
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
% Stats_constraints (struct)
%   Zusätzliche Informationen zu den einzelnen Werten aus Q0. Felder:
%   .bestcolldist: Bestmöglicher Kollisionsabstand
%   .bestinstspcdist: Bestmöglicher Abstand zum Bauraum

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [fval,QE_all,Q0,constrvioltext,Stats_constraints] = cds_constraints(R, Traj_0, Set, Structure)
Q0 = NaN(1,R.NJ);
QE_all = Q0;
Stats_constraints = struct('bestcolldist', [], 'bestinstspcdist', []);
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
  s.retry_limit = 20+Set.optimization.pos_ik_tryhard_num;
else % Nur Eckpunkte
  % Eckpunkte haben keinen direkten Bezug zueinander und bilden die
  % Trajektorie. Da keine Traj. berechnet wird, kann hier mehr Aufwand
  % betrieben werden (besonders bei seriellen Robotern auch notwendig).
  s.retry_limit = 50+Set.optimization.pos_ik_tryhard_num;
  s.n_max = 5000;
end
condJ = NaN(size(Traj_0.XE,1), 1); % Gesamt-Jacobi (Antriebe-EE)
if R.Type == 0 % Seriell
  qlim = R.qlim;
  qref = R.qref;
  Phi_E = NaN(size(Traj_0.XE,1), sum(Set.task.DoF));
  condJik = NaN(size(Traj_0.XE,1), 1); % IK-Jacobi
  QE = NaN(size(Traj_0.XE,1), R.NQJ);
  % Variable zum Speichern der Gelenkpositionen (für Kollisionserkennung)
  JPE = NaN(size(Traj_0.XE,1), (R.NL+1)*3);
else % PKM
  qlim = cat(1,R.Leg(:).qlim);
  qref = cat(1,R.Leg(:).qref);
  nPhi = R.I2constr_red(end);
  Phi_E = NaN(size(Traj_0.XE,1), nPhi);
  condJik = NaN(size(Traj_0.XE,1), R.NLEG); % Spalten: IK-Jacobi (jede Beinkette einzeln)
  QE = NaN(size(Traj_0.XE,1), R.NJ);
  % Abbruch der IK-Berechnung, wenn eine Beinkette nicht erfolgreich war.
  % Dadurch wesentlich schnellerer Durchlauf der PKM-IK
  s_par = struct('abort_firstlegerror', true);
  JPE = NaN(size(Traj_0.XE,1), (R.NL+R.NLEG+1)*3);
end
if R.Type == 2 % zusätzliche IK-Konfigurationen für PKM
  n_ikcomb = 2^R.NLEG; % Annahme zwei Umklapp-Lagen für IK jeder Beinkette
else
  n_ikcomb = 0; % Für serielle Roboter nicht relevant.
end
n_jic = 30+Set.optimization.pos_ik_tryhard_num+n_ikcomb;
fval_jic = NaN(1,n_jic);
calctimes_jic = NaN(2,n_jic);
constrvioltext_jic = cell(n_jic,1);
bestcolldist_jic = NaN(1,n_jic);
bestinstspcdist_jic = NaN(1,n_jic);
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
if Structure.task_red
  ar_loop = 1:2; % Aufgabenredundanz liegt vor. Zusätzliche Schleife
else
  ar_loop = 1; % Keine Aufgabenredundanz. Nichts zu berechnen.
end
% Schwellwert für Jacobi-Matrix in Nullraumbewegung (falls nur als
% Nebenbedingung und nicht als Optimierungsziel)
cond_thresh_jac = 250;
cond_thresh_ikjac = 500;
if Set.optimization.constraint_obj(4) ~= 0 % Grenze für Jacobi-Matrix für Abbruch
  % Zu invertierende IK-Jacobi (bezogen auf Euler-Winkel-Residuum)
  cond_thresh_ikjac = min(cond_thresh_ikjac,Set.optimization.constraint_obj(4));
  % Jacobi-Matrix des Roboters (bezogen auf Antriebe und Arbeitsraum-FG)
  cond_thresh_jac = Set.optimization.constraint_obj(4);
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
  if jic <= (n_jic-n_ikcomb)/3
    % Benutze Zufallswerte von oben (nicht überschreiben wie in anderen
    % Fällen)
  elseif jic > (n_jic-n_ikcomb)/3 && jic < 2*(n_jic-n_ikcomb)/3 % zwischen ein Drittel und zwei Drittel der regulären Versuche
    % Setze die Anfangswerte (für Schubgelenke) ganz weit nach "links"
    q0(R.MDH.sigma==1) = q0(R.MDH.sigma==1) - 0.5*rand(1)*...
      (qlim_norm(R.MDH.sigma==1,2)-qlim_norm(R.MDH.sigma==1,1));
  elseif jic >= 2*(n_jic-n_ikcomb)/3 && jic <= n_jic-n_ikcomb % Letztes Drittel der regulären Versuche
    % Anfangswerte weit nach rechts
    q0(R.MDH.sigma==1) = q0(R.MDH.sigma==1) + 0.5*rand(1)*...
      (qlim_norm(R.MDH.sigma==1,2)-qlim_norm(R.MDH.sigma==1,1));
  elseif jic > n_jic-n_ikcomb % Versuche mit Kombinationen der bisherigen gefundenen Konfigurationen (für PKM)
    if jic == n_jic-n_ikcomb+1 && R.Type == 2
      % Bestimme alle Kombinationen der Gelenkkoordinaten der Beinketten
      Q_configperm1 = NaN(0,size(Q_jic,2));
      % alle Kombinationen der einzelnen Beinketten durchgehen
      nj = zeros(1,R.NLEG); % Anzahl der Kombinationen für jede Beinkette
      indvec = cell(1,R.NLEG); % Vektor mit Indizes zum Bilden der PKM-Kombinationen
      dist_pt1 = NaN(size(Q_jic,3), R.NLEG);
      Idesc = dist_pt1;
      for j = 1:R.NLEG
        Ij = R.I1J_LEG(j):R.I2J_LEG(j); % Index für Beinketten-Gelenke
        nj(j) = 0;
        for i = 1:size(Q_jic,3) % Über alle Neuversuche der IK (vorher berechnet)
          if any(isnan(Q_jic(1,Ij,i))), continue; end
          test_exist = repmat(Q_jic(1,Ij,i), size(Q_configperm1,1), 1) - Q_configperm1(:,Ij);
          if isempty(test_exist) || ~any(all( abs(test_exist) < 1e-6, 2 ))
            % Neue Konfiguration für diese Beinkette gefunden. Hinzufügen.
            nj(j) = nj(j) + 1;
            Q_configperm1(nj(j),Ij) = Q_jic(1,Ij,i);
            % Bestimme den Abstand zur ersten gefundenen Beinkette als
            % Betragssumme aller Gelenkwinkeldifferenzen (ohne
            % Schubgelenke). Annahme: Entspricht am ehesten den
            % Umklapplagen)
            Ijrev = Ij(R.Leg(j).MDH.sigma==0);
            dist_qi_q1 = angleDiff(Q_configperm1(1, Ijrev),Q_configperm1(nj(j), Ijrev));
            dist_pt1(i,j) = sum(abs(dist_qi_q1));
          end
        end % i
        % Indizes gemäß Abstand zum ersten sortieren. Hilft bei Aufgaben-
        % redundanz, wenn unendlich viele Konfigurationen möglich sind.
        [~,Idesc(1:nj(j),j)] = sort(dist_pt1(1:nj(j),j), 'desc');
        % Nehme zwei Konfigurationen für die Beinkette. Konsistent mit der
        % Initialisierung von n_ikcomb. Mehr als zwei noch nicht
        % implementiert.
        indvec{j} = [1, Idesc(1,j)];
      end % for j
      Q_configperm_idx = allcomb(indvec{:});
      if size(Idesc,1)>2 && ~isnan(Idesc(3,1))
        % Nehme absichtlich einen anderen Wert, wenn Konfigurationen
        % doppelt sind. Sonst werden diese erneut berechnet ohne Mehrwert.
        Q_configperm_idx(all(diff(Q_configperm_idx')==0),1) = Idesc(2);
      end
      Q_configperm3 = NaN(size(Q_configperm_idx,1), size(Q_configperm1,2));
      for ii = 1:size(Q_configperm3,1)
        for j = 1:R.NLEG
          Q_configperm3(ii,R.I1J_LEG(j):R.I2J_LEG(j)) = ...
            Q_configperm1(Q_configperm_idx(ii,j), R.I1J_LEG(j):R.I2J_LEG(j));
        end
      end
      % Entferne doppelte Einträge durch NaN-Setzen (dadurch überspringen)
      Q_configperm3(all(diff(Q_configperm_idx')==0),:) = NaN;
    end
    if jic-(n_jic-n_ikcomb) > size(Q_configperm3,1), break; end % keine weiteren Kombinationen verfügbar
    q0 = Q_configperm3(jic-(n_jic-n_ikcomb),:)';
    if any(isnan(q0))
      constrvioltext_jic{jic} = 'Gelenkwinkel aus Kombination der Beinketten-IK-Konfigurationen doppelt';
      fval_jic(jic) = inf;
      continue; % NaN als Marker für doppelte q0 gegenüber vorheriger Iteration überspringen
    end
  else
    error('Fall nicht vorgesehen. Logik-Fehler bei Index-Bereichen')
  end % Fälle für Bestimmung von q0
  % Normalisiere den Anfangswert (außerhalb [-pi,pi) nicht sinnvoll).
  % (Betrifft nur Fall, falls Winkelgrenzen groß gewählt sind)
  q0(R.MDH.sigma==0) = normalize_angle(q0(R.MDH.sigma==0));
  % Übertrage in Variable für Menge mehrerer möglicher Anfangswerte.
  % hiermit theoretisch auch mehrere Anfangswerte auf einmal vorgebbar.
  % Wird benutzt, damit die Ergebnisse vorheriger Punkte ausprobiert
  % werden können
  Q0_ik = q0;

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
        [q, Phi, Tc_stack, Stats] = R.invkin2(R.x2tr(Traj_0.XE(i,:)'), Q0_ik, s);
        nPhi_t = sum(R.I_EE_Task(1:3));
        ik_res_ik2 = all(abs(Phi(1:nPhi_t))<s.Phit_tol) && ...
                     all(abs(Phi(nPhi_t+1:end))<s.Phir_tol);
        condJik(i,1) = Stats.condJ(1+Stats.iter,1);
        condJ(i,1) = Stats.condJ(1+Stats.iter,2);
      else % PKM
        Q0_mod = Q0_ik;
        if Set.optimization.pos_ik_tryhard_num > 0
          % Annahme: Wenn Modus aktiv ist, sollen alte Ergebnisse
          % reproduziert werden. Dann zuerst die vollständige Gelenk-
          % Konfiguration vorgeben und nicht ab wie unten mit NaN ersetzen.
          Q0_mod = [Q0_ik, Q0_ik(:,1)];
          s_par.prefer_q0_over_first_legchain = true; % q0 bevorzugen
        end
        if jic <= n_jic-n_ikcomb
           % Für Beinkette 2 Ergebnis von BK 1 nehmen (dabei letzten
           % Anfangswert für BK 2 und folgende verwerfen). Nur, wenn es
           % nicht gerade darum gehen soll, alle Kombinationen auszu-
           % probieren (siehe Definition der jic-Bereiche für q0 oben)
          Q0_mod(R.I1J_LEG(2):end,end) = NaN;
        end
        [q, Phi, Tc_stack, Stats] = R.invkin2(Traj_0.XE(i,:)', Q0_mod, s, s_par); % kompilierter Aufruf
        % Rechne kinematische Zwangsbedingungen nach. Ist für Struktur-
        % synthese sinnvoll, falls die Modellierung unsicher ist.
        if any(strcmp(Set.optimization.objective, 'valid_act')) || Set.general.debug_calc
          x2 = R.fkineEE2_traj(q')'; % Bei 3T2R muss die EE-Drehung aktualisiert werden
          x2(1:5) = Traj_0.XE(i,1:5);
          [~,Phi_test] = R.constr1(q, x2);
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
        if ~ik_res_ik2 && all(abs(Phi_Leg1)<s.Phit_tol) && ... % Vereinfachung: Toleranz transl/rot identisch.
            (Structure.task_red || all(R.I_EE==[1 1 1 1 1 0]))
          % Die Einzelbeinketten-IK für die erste Beinkette war erfolgreich, 
          % aber nicht für die weiteren Beinketten. Mögliche Ursache
          % ist die freie Drehung der Plattform durch die erste Beinkette.
          % Erneuter Versuch mit der IK für alle Beinketten gemeinsam.
          % Ist auch bei strukturell bedingten 3T2R relevant wegen
          % IK-Umklappen.
          % Bei genanntem Abbruch sind nicht funktionierende Beinketten
          % direkt NaN. Dafür andere Werte einsetzen.
          q(isnan(q)) = Q0_ik(isnan(q),1);
          Q0_v2 = [q,Q0_ik];
          [q, Phi, Tc_stack, Stats] = R.invkin4(Traj_0.XE(i,:)', Q0_v2, s);
          condJik(i,:) = Stats.condJ(1+Stats.iter,1);
          condJ(i,1) = Stats.condJ(1+Stats.iter,2);
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
    if i_ar == 2 % Bei AR IK erneut mit Optimierung durchführen
      % Aufgabenredundanz. Benutze anderen IK-Ansatz (Reziproke
      % Euler-Winkel; gemeinsame Modellierung aller Beinketten)
      % Berechne trotzdem zuerst die einfache IK für jede Beinkette
      % einzeln. Mit dessen Ergebnis dann nur noch Nullraumbewegung zum
      % Optimum der Nebenbedingungen
      % Startwert bei vorheriger Lösung. Dadurch reine Nullraumbewegung
      q0_arik = QE(i,:)';
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
      % Die Jacobi (bzgl. Antriebe), und die IK-Jacobi nehmen.
      % Die IK-Jacobi kann schlecht sein, bei guter Antriebs-Jacobi.
      s4.wn = zeros(R.idx_ik_length.wnpos,1);
      s4.wn(R.idx_ikpos_wn.ikjac_cond) = 1;
      s4.wn(R.idx_ikpos_wn.jac_cond) = 1;
      % Versuche die Gelenkwinkelgrenzen einzuhalten, wenn explizit gefordert
      if Set.optimization.fix_joint_limits
        s4.wn(R.idx_ikpos_wn.qlim_hyp) = 1;
        s4.optimcrit_limits_hyp_deact = 0.95; % Nur am Rand der Grenzen aktiv werden
      end
      % Setze die Einstellungen und Nebenbedingungen so, dass sich das
      % Ergebnis bestmöglich verändert.
      if fval_jic(jic) == 1e3 && i>1
        % Wenn vorher alle Punkte in Ordnung waren, muss nur noch der erste
        % Punkt optimiert werden. Für die hierauf folgende Trajektorie
        % haben die anderen Punkte sowieso keine Bedeutung.
        break;
      elseif fval_jic(jic) > 5e5 && fval_jic(jic) < 7e5
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
        s4.wn(:) = 0;
        s4.wn(R.idx_ikpos_wn.qlim_par) = 1; % quadratische Funktion für Gelenkgrenzen (Startwinkel bereits außerhalb der Grenzen)
        % Setze die Gelenkwinkel-Grenzen neu. Annahme: Die absoluten Werte
        % der Winkel sind nicht wichtig. Es kommt auf die Spannweite an.
        % Lasse Schubgelenke so, wie sie sind. Annahme: Werden schon so
        % kurz wie möglich gewählt (wegen Einfluss auf Masse/Dynamik)
        if ~Set.optimization.fix_joint_limits
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
        end
        % Zwinge den Startwert in die neuen Grenzen (auf 5% innerhalb).
        % Hat oft zur Folge, dass die IK gar nicht mehr konvergiert. Daher
        % doch nicht machen. Entscheidung
        % q0_arik(q0_arik>qlim(:,2)) = qlim(q0_arik>qlim(:,2))-0.05*qlim_range(q0_arik>qlim(:,2));
        % q0_arik(q0_arik<qlim(:,1)) = qlim(q0_arik<qlim(:,1))+0.05*qlim_range(q0_arik<qlim(:,1));
        % % Dann keine Überschreitung der neuen Grenzen mehr zulassen
        % s4.scale_lim = 0.7; % Scheint die Erfolgsquote stark zu verschlechtern.
      elseif fval_jic(jic) > 3e5 && fval_jic(jic) < 4e5
        % Der vorherige Ausschlussgrund war eine Kollision.
        % Kollisionsvermeidung als Optimierung
        % Prüfe vorher, ob dieser Punkt ausschlaggebend für die Kollision
        % war. Wenn nicht, kann er übersprungen werden und die bestehende
        % Lösung wird genutzt.
        if i~=1 && ~any(coll_self(i,:)) % Die Kollision bezog sich nicht auf diesen Eckpunkt
          continue
        end
        s4.wn(:) = 0;
        s4.wn(R.idx_ikpos_wn.coll_hyp) = 1;
        if ~isempty(Set.task.installspace.type)
          % Vorsorglich auch Optimierung nach Bauraumgrenzen aktivieren.
          % Bei Kollisionsvermeidung größtmöglicher Abstand von Segmenten
          % zueinander führt tendenziell zu Bauraumüberschreitung.
          s4.wn(R.idx_ikpos_wn.instspc_hyp) = 1;
          s4.installspace_thresh = 0.05; % Nur bei geringem Abstand aktiv
        end
      elseif fval_jic(jic) > 2e5 && fval_jic(jic) < 3e5
        % Der vorherige Ausschlussgrund war eine Bauraumverletzung.
        s4.wn(:) = 0;
        s4.wn(R.idx_ikpos_wn.instspc_hyp) = 1;
        s4.installspace_thresh = 0.1500; % Etwas höherer Abstand zur Aktivierung der Funktion
      elseif fval_jic(jic) == 1e3 % Vorher erfolgreich
        % Vermeide Singularitäten. Abseits davon keine Betrachtung.
        s4.cond_thresh_ikjac = cond_thresh_ikjac; % IK-Jacobi
        s4.wn(R.idx_ikpos_wn.ikjac_cond) = 1;
        % Geometrische Jacobi (SerRob) bzw. PKM-Jacobi (ParRob)
        s4.wn(R.idx_ikpos_wn.jac_cond) = 1;
        s4.cond_thresh_jac = cond_thresh_jac;
        if any(strcmp(Set.optimization.objective, 'colldist'))
          % Benutze quadratische Abstandsfunktion der Kollisionen (ohne
          % Begrenzung). Dadurch maximaler Abstand gesucht
          s4.wn(R.idx_ikpos_wn.coll_par) = 1; % Kollision
        end
        if any(strcmp(Set.optimization.objective, 'footprint')) || ...
           any(strcmp(Set.optimization.objective, 'installspace'))
          % Benutze quadratische Abstandsfunktion für Bauraum (ohne
          % Begrenzung). Dadurch maximaler Abstand zum Grenze gesucht
          % Das hilft möglicherweise bei der Einhaltung einer kleinen
          % Grundfläche
          s4.wn(R.idx_ikpos_wn.instspc_par) = 1; % Bauraumeinhaltung
        end
        if any(strcmp(Set.optimization.objective, 'jointrange'))
          % Versuche die Gelenkwinkel so gut wie möglich in die Mitte der
          % Grenzen zu ziehen. Es zählt die Spannweite und nicht die
          % konkreten Grenzen
          s4.wn(R.idx_ikpos_wn.qlim_par) = 1; % quadratische Grenzen
        end
        if any(strcmp(Set.optimization.objective, 'jointlimit'))
          % Versuche die Gelenkwinkel vorrangig von den Grenzen weg zu
          % bewegen. Hier ist das hyperbolische Kriterium stärker
          s4.wn(R.idx_ikpos_wn.qlim_hyp) = 1; % hyperbolische Grenzen
          s4.optimcrit_limits_hyp_deact = 0.4; % fast immer aktiv (bis zu 30% zu den Grenzen hin)
        end
      end
      if i == 1
        % Für den ersten Punkt sollten die Optimierungskriterien konsistent
        % zu cds_constraints_traj sein. Sonst komme es zu Beginn der Traj-
        % ektorie zu starken Nullraumbewegungen
        % Nur für ersten Traj.-Punkt die Kondition verbessern (falls
        % schlechter als Schwellwert)
        % TODO: Mehrere Zielfunktionen neigen zum oszillieren. Tritt hier
        % auf, wenn oben weitere Bedingungen gesetzt wurden.
        s4.wn(R.idx_ikpos_wn.jac_cond) = 1;
        if Set.optimization.constraint_collisions
          % Aktiviere hyperbolisches Kollisions-Kriterium
          s4.wn(R.idx_ikpos_wn.coll_hyp) = 1;
          % 25% größere Kollisionskörper für Aktivierung. Wird dann zum
          % Mindestabstand
          s4.collbodies_thresh = 1.25;
        end
      end
      % IK für Nullraumbewegung durchführen
      if R.Type == 0
        [q, Phi, Tc_stack, Stats] = R.invkin2(R.x2tr(Traj_0.XE(i,:)'), q0_arik, s4);
      else
        [q, Phi, Tc_stack, Stats] = R.invkin4(Traj_0.XE(i,:)', q0_arik, s4);
      end
%       if Stats.iter == 2 % TODO: Unklar, ob das hier schlecht ist
%         cds_log(4, sprintf(['[constraints] Konfig %d/%d, Eckpunkt %d, Iter. %d: ', ...
%           'Nur eine Iteration bei AR-IK. Parametrierung schlecht oder Start in Optimum.'], jic, n_jic, i, i_ar));
%       end
      % Trage die Kollisionsabstände für den Startwert ein (entspricht
      % Ergebnis der normalen IK von oben. Dort werden die Kennzahlen nicht
      % berechnet). Annahme: Die IK in der ersten Iteration ist i.O.
      bestcolldist_jic(jic) = min([bestcolldist_jic(jic);Stats.maxcolldepth(1,1)]);
      bestinstspcdist_jic(jic) = min([bestinstspcdist_jic(jic);Stats.instspc_mindst(1,1)]);
      % Benutze nicht die sehr strenge Grenze von Phit_tol von oben, da aus
      % numerischen Gründen diese teilweise nicht erreicht werden kann
      ik_res_ikar = all(abs(Phi) < 1e-6);
      if Set.general.debug_calc && all(abs(q-q0_arik) < 1e-9)
        cds_log(-1, sprintf(['[constraints] Konfig %d/%d, Eckpunkt %d, Iter. %d: IK-Berechnung mit Aufgabenredundanz ', ...
          'hat gleiches Ergebnis wie ohne (max delta q = %1.1e).'], jic, n_jic, i, i_ar, max(abs(q-q0_arik))));
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
            ~any(Stats.condJ([1,1+Stats.iter],1) < 1e3) % Bei singulären Beinketten ist das Scheitern erwartbar
          cds_log(3, sprintf(['[constraints] Konfig %d/%d, Eckpunkt %d: IK-Berechnung ', ...
            'mit Aufgabenredundanz fehlerhaft, obwohl es ohne AR funktioniert ', ...
            'hat. wn=[%s]. max(Phi)=%1.1e. Iter %d/%d'], jic, n_jic, i, disp_array(s4.wn','%1.1g'), ...
            max(abs(Phi)), Stats.iter, size(Stats.Q,1)-1));
        else
          % Falls neue Grenzen gesetzt wurden, ist die IK eventuell nicht
          % innerhalb der Grenzen lösbar. In diesem Fall hier kein Fehler
        end
        continue % Verwerfe das neue Ergebnis (nehme dadurch das vorherige)
      end
      % Ergebnis der Nullraumoptimierung auswerten und vergleichen.
      % Benutzung der Summe aus Ausgabe nicht möglich (wn verändert sich).
      h_opt_pre  = sum(s4.wn' .* Stats.h(1,2:(1+R.idx_ik_length.hnpos)) );
      h_opt_post = sum(s4.wn' .* Stats.h(1+Stats.iter,2:(1+R.idx_ik_length.hnpos)) );
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
          Stats.condJ(1,1) < 1e3 % Wenn die PKM am Anfang singulär ist, dann ist die Verschlechterung der Nebenopt. kein Ausschlussgrund für die neue Lösung
        if abs(Stats.h(1+Stats.iter,1)-h_opt_post) < 1e-3 && ... % es lag nicht am geänderten `wn` in der Funktion
             (h_opt_post < 1e8 || ... % Es ist kein numerisch großer und ungenauer Wert
             h_opt_post > 1e8 && h_opt_pre < 1e8) % Wenn sich der Wert auf unendlich verschlechtert
          debug_str = sprintf('condPhiq: %1.1f -> %1.1f', ...
            Stats.condJ(1,1), Stats.condJ(1+Stats.iter,1));
          debug_str = [debug_str, sprintf('; condJ: %1.1f -> %1.1f', ...
            Stats.condJ(1,2), Stats.condJ(1+Stats.iter,2))]; %#ok<AGROW>
          if any(~isnan(Stats.maxcolldepth(:)))
            debug_str = [debug_str, sprintf('; maxcolldepth [mm]: %1.1f -> %1.1f', ...
              1e3*Stats.maxcolldepth(1,1), 1e3*Stats.maxcolldepth(1+Stats.iter,1))]; %#ok<AGROW>
          end
          cds_log(3, sprintf(['[constraints] Konfig %d/%d, Eckpunkt %d: IK-Berechnung ', ...
            'mit Aufgabenredundanz hat Nebenoptimierung verschlechtert: ', ...
            '%1.4e -> %1.4e. wn=[%s]. max(condJik)=%1.2f. coll=%d. %s'], ...
            jic, n_jic, i, h_opt_pre, h_opt_post, disp_array(s4.wn','%1.1g'), ...
            max(Stats.condJ(1:1+Stats.iter,1)), Stats.coll, debug_str));
        end
        if Set.general.debug_taskred_fig % Zum Debuggen
        if R.Type == 0, I_constr_red = [1 2 3 5 6];
        else,           I_constr_red = R.I_constr_red; end
        FigARDbg = change_current_figure(2345);clf;
        Iter = 1:1+Stats.iter;
        set(FigARDbg,'Name','AR_PTPDbg', 'NumberTitle', 'off');
        subplot(3,3,1);
        plot(Stats.condJ(Iter,:));
        xlabel('Iterationen'); grid on;
        ylabel('cond(J)');
        legend({'IK-Jacobi', 'Jacobi'});
        subplot(3,3,2);
        plot([diff(Stats.Q(Iter,:),1,1);NaN(1,R.NJ)]);
        xlabel('Iterationen'); grid on;
        ylabel('diff q');
        subplot(3,3,3); hold on;
%         plot([Stats.Q(1:Stats.iter,:);NaN(1,R.NJ)]);
%         xlabel('Iterationen'); grid on;
%         ylabel('q');
        Stats_X = R.fkineEE2_traj(Stats.Q(Iter,:));
        Stats_X(:,6) = denormalize_angle_traj(Stats_X(:,6));
        I_Phi_iO = all(abs(Stats.PHI(Iter,I_constr_red))<1e-6,2);
        I_Phi_med = all(abs(Stats.PHI(Iter,I_constr_red))<1e-3,2)&~I_Phi_iO;
        I_Phi_niO = ~I_Phi_iO & ~I_Phi_med;
        legdhl = NaN(3,1);
        if any(I_Phi_iO)
          legdhl(1)=plot(Iter(I_Phi_iO), 180/pi*Stats_X(I_Phi_iO,6), 'gv');
        end
        if any(I_Phi_med)
          legdhl(2)=plot(Iter(I_Phi_med), 180/pi*Stats_X(I_Phi_med,6), 'mo');
        end
        if any(I_Phi_niO)
          legdhl(3)=plot(Iter(I_Phi_niO), 180/pi*Stats_X(I_Phi_niO,6), 'rx');
        end
        plot(Iter, 180/pi*Stats_X(:,6), 'k-');
        legstr = {'Phi<1e-6', 'Phi<1e-3', 'Phi>1e-3'};
        legend(legdhl(~isnan(legdhl)), legstr(~isnan(legdhl)));
        xlabel('Iterationen'); grid on;
        ylabel('phi_z in deg');
        Stats_Q_norm = (Stats.Q(Iter,:)-repmat(qlim(:,1)',1+Stats.iter,1))./ ...
                        repmat(qlim_range',1+Stats.iter,1);
        subplot(3,3,4);
        plot([Stats_Q_norm(Iter,:);NaN(1,R.NJ)]);
        xlabel('Iterationen'); grid on;
        ylabel('q norm');
        subplot(3,3,5);
        plot([diff(Stats.PHI(Iter,I_constr_red));NaN(1,length(I_constr_red))]);
        xlabel('Iterationen'); grid on;
        ylabel('diff Phi');
        subplot(3,3,6);
        plot(Stats.PHI(Iter,I_constr_red));
        xlabel('Iterationen'); grid on;
        ylabel('Phi');
        subplot(3,3,7);
        plot(diff(Stats.h(Iter,[true,s4.wn'~=0])));
        xlabel('Iterationen'); grid on;
        ylabel('diff h');
        subplot(3,3,8);
        plot(Stats.h(Iter,[true,s4.wn'~=0]));
        critnames = fields(R.idx_ikpos_wn)';
        legend(['w.sum',critnames(s4.wn'~=0)], 'interpreter', 'none');
        xlabel('Iterationen'); grid on;
        ylabel('h');
        if any(Stats.maxcolldepth(:)>0) || ... % es sollte eine Kollision gegeben haben
            s4.wn(R.idx_ikpos_wn.coll_par) % Kollisionsabstand war quadr. Zielfunktion
          % Die Ausgabe maxcolldepth wird nur geschrieben, wenn Kollisionen
          % geprüft werden sollten
          subplot(3,3,9);
          plot(Stats.maxcolldepth(Iter,:));
          xlabel('Iterationen'); grid on;
          ylabel('Kollisionstiefe (>0 Koll.)');
          legend({'alle', 'beeinflussbar'});
        elseif s4.wn(R.idx_ikpos_wn.instspc_hyp) || s4.wn(R.idx_ikpos_wn.instspc_par)
          subplot(3,3,9);
          plot(Stats.instspc_mindst(Iter,:));
          xlabel('Iterationen'); grid on;
          ylabel('Außerhalb Bauraum (>0 raus)');
          legend({'alle', 'beeinflussbar'});
        end
        linkxaxes
        set(FigARDbg,'color','w');
        [currgen,currind,~,resdir] = cds_get_new_figure_filenumber(Set, Structure, '');
        name_prefix_ardbg = sprintf('Gen%02d_Ind%02d_Konfig%d_Pt%d', currgen, ...
          currind, jic, i);
        for fileext=Set.general.save_robot_details_plot_fitness_file_extensions
          if strcmp(fileext{1}, 'fig')
            saveas(FigARDbg, fullfile(resdir, sprintf('%s_TaskRed_%s.fig', ...
              name_prefix_ardbg, get(FigARDbg, 'name'))));
          else
            export_fig(FigARDbg, fullfile(resdir, sprintf('%s_TaskRed_%s.%s', ...
              name_prefix_ardbg, get(FigARDbg, 'name'), fileext{1})));
          end
        end
        end
        continue % Verwerfe das neue Ergebnis (nehme dadurch das vorherige)
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
      % Neue Werte aus der IK wurden nicht verworfen. Schreibe Konditionszahl
      % (erster Eintrag ist IK-Jacobi für Gesamt-PKM)
      condJik(i,:) = Stats.condJ(1+Stats.iter,1);
      condJ(i,1) = Stats.condJ(1+Stats.iter,2);
      % Merke besten Kollisions- und Bauraumabstand. Damit Bestimmung von
      % Schwellwerten für die Trajektorien-Nebenbedingungen.
      if R.Type == 0, I_constr_red = [1 2 3 5 6];
      else,           I_constr_red = R.I_constr_red; end
      I_iO = all(abs(Stats.PHI(:,I_constr_red)) < 1e-6, 2);
      bestcolldist_jic(jic) = min([bestcolldist_jic(jic);Stats.maxcolldepth(I_iO)]);
      bestinstspcdist_jic(jic) = min([bestinstspcdist_jic(jic);Stats.instspc_mindst(I_iO)]);
    end
    % Normalisiere den Winkel. Bei manchen Robotern springt das IK-Ergebnis
    % sehr stark. Dadurch wird die Gelenkspannweite sonst immer verletzt.
    if i == 1 % Normalisierung von -pi bis pi (Mitte 0)
      q(R.MDH.sigma==0) = normalizeAngle(q(R.MDH.sigma==0), 0);
    else
      % Normalisierung nicht um den Wert Null herum, sondern um die Winkel
      % der ersten Konfiguration. Das führt zu einer minimalen Spannweite
      % bezüglich aller Gelenkkonfigurationen der Eckpunkte und wird auch
      % als Mittelwert für die später festgelegten Gelenkgrenzen benutzt.
      q(R.MDH.sigma==0) = normalizeAngle(q(R.MDH.sigma==0), ...
        QE(1, R.MDH.sigma==0)'); % Bezugswinkel erste Punkt
    end
    Phi_E(i,:) = Phi;
    QE(i,:) = q;
    if any(abs(Phi(:)) > 1e-2) || any(isnan(Phi))
      constrvioltext_jic{jic} = sprintf('Keine IK-Konvergenz');
      break; % Breche Berechnung ab (zur Beschleunigung der Berechnung)
    end
    % Probiere für den nächsten Punkt die Gelenkwinkel aller bisher
    % berechneter Punkte aus. Dadurch wird eher die gleiche Konfiguration
    % gefunden.
    Q0_ik = QE(i:-1:1,:)';
    JPE(i,:) = Tc_stack(:,4); % Vierte Spalte ist Koordinatenursprung der Körper-KS
    if Set.general.debug_calc && R.Type == 2
      % Prüfe, ob die ausgegebenen Gelenk-Positionen auch stimmen
      JointPos_all_i_frominvkin = reshape(JPE(i,:)',3,1+R.NLEG+R.NJ+1+1);
      Tc_Lges = R.fkine_legs(QE(i,:)');
      JointPos_all_i_fromdirkin = [zeros(3,1), squeeze(Tc_Lges(1:3,4,1:end)), NaN(3,1)];
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
      'Eckpunkte zu groß. max(cond(J_IK))=%1.1e.'], n_condexc, size(condJik,1), max(condJik(:)));
    calctimes_jic(i_ar,jic) = toc(t1);
    continue;
  end
  %% Prüfe die Konditionszahl der Jacobi-Matrix (bezogen auf Antriebe)
  % Wenn die Kondition im Anfangswert oder einem Zwischenpunkt schlecht ist,
  % braucht anschließend keine Trajektorie mehr gerechnet werden
  if Set.optimization.constraint_obj(4) == 0
    n_condexc = 0; % Nebenbedingung für Jacobi ist nicht aktiv
  else
    n_condexc = sum(condJ(:) > Set.optimization.constraint_obj(4));
  end
  n_condexc2 = sum(condJ(:) > Set.optimization.condition_limit_sing_act);
  if n_condexc > 0 || n_condexc2 > 0
    if n_condexc2 > 0
      % Konditionszahl ist so hoch, dass es eine komplette Singularität ist
      fval = 1e5*(8+n_condexc2/size(condJ,1)); % Normierung auf 8e5 bis 9e5
      constrvioltext_jic{jic} = sprintf(['Jacobi-Konditionszahl für %d/%d ', ...
        'Eckpunkte singulär. max(cond(J))=%1.1e.'], n_condexc2, size(condJ,1), max(condJ(:)));
    else
      % Konditionszahl ist hoch, aber nur "schlechter-Wert-hoch"
      fval = 1e5*(7+n_condexc/size(condJ,1)); % Normierung auf 7e5 bis 8e5
      constrvioltext_jic{jic} = sprintf(['Jacobi-Konditionszahl für %d/%d ', ...
        'Eckpunkte zu groß. max(cond(J))=%1.1e.'], n_condexc, size(condJ,1), max(condJ(:)));
    end
    fval_jic(jic) = fval;

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
    fval = 1e5*(6+1*fval_qlimv_E_norm); % Normierung auf 6e5 bis 7e5
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
    if Set.general.plot_details_in_fitness < 0 && 1e4*fval >= abs(Set.general.plot_details_in_fitness) || ... % Gütefunktion ist schlechter als Schwellwert: Zeichne
       Set.general.plot_details_in_fitness > 0 && 1e4*fval <= abs(Set.general.plot_details_in_fitness)
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

  %% Prüfe die Verletzung von absoluten Grenzen der Winkel (falls vorhanden)
  % Falls nicht gesetzt, haben die absoluten Grenzen in der Maßsynthese
  % keine Bedeutung, sondern es kommt nur auf die Spannweite an. Falls sie
  % gesetzt sind, ist die Spannweite noch wichtiger (daher vorher).
  if Set.optimization.fix_joint_limits
    % Normalisiere Gelenkwinkel auf 0...1
    QE_norm = (QE-repmat(qlim(:,1)',size(QE,1),1))./...
                repmat(qlim_range', size(QE,1),1);
    % Normalisiere auf -0.5...+0.5. Dadurch Erkennung der Verletzung einfacher
    Q_limviolA = abs(QE_norm-0.5); % 0 entspricht jetzt der Mitte.
    if any(Q_limviolA(:) > 0.5)
      [lvmax,Imax] = max(Q_limviolA,[],1);
      [delta_lv_maxrel,Imax2] = max(lvmax-0.5);
      fval_qlimva_E_norm = 2/pi*atan((delta_lv_maxrel)/0.3); % Normierung auf 0 bis 1; 2 ist 0.9
      fval = 1e5*(5+1*fval_qlimva_E_norm); % Normierung auf 5e5 bis 6e5
      fval_jic(jic) = fval;
      constrvioltext_jic{jic} = sprintf(['Gelenkgrenzverletzung in AR-Eckwerten. ', ...
        'Größte relative Überschreitung: %1.1f%% (Gelenk %d, Eckpunkt %d/%d)'], ...
        100*delta_lv_maxrel, Imax2, Imax(Imax2), size(QE,1));
      continue;
    end
  end
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
      fval = 1e5*(4.5+0.5*fval_rbase_norm); % Normierung auf 4.5e5 bis 5e5
      fval_jic(jic) = fval;
      calctimes_jic(i_ar,jic) = toc(t1);
      continue;
    end
  end
  %% Prüfe die Länge von Schubzylindern
  % Ist mit der Gelenkwinkel-Spannweite verbunden. Für die Schubzylinder-
  % Spannweite muss ein entsprechender Platz in einem Außenzylinder gegeben
  % sein. Der Außenzylinder sollte nicht durch das vorherige Gelenk gehen.
  if ~Set.optimization.prismatic_cylinder_allow_overlength && any(Structure.I_straightcylinder)
    % Berechne die Länge, die der Zylinder nach hinten geht
    qmin_cyl = min(abs(QE(:,Structure.I_straightcylinder)), [], 1);
    qmax_cyl = max(abs(QE(:,Structure.I_straightcylinder)), [], 1);
    length_cyl = qmax_cyl - qmin_cyl;
    [fval_cyllen, Iworst] = max(length_cyl./qmin_cyl);
    if fval_cyllen > 1
      constrvioltext_jic{jic} = sprintf(['Länge eines Schubzylinders steht ', ...
        'nach hinten über. Min. Abstand %1.1fmm, Innenzylinder Länge %1.1fmm ', ...
        '(Gelenk %d)'], 1e3*qmin_cyl(Iworst), 1e3*length_cyl(Iworst), Iworst);
      fval_cyllen_norm = 2/pi*atan((fval_cyllen-1)*3); % Normierung auf 0 bis 1; 100% zu lang ist 0.8
      fval = 1e5*(4.25+0.25*fval_cyllen_norm); % Normierung auf 4.25e5 bis 4.5e5
      fval_jic(jic) = fval;
      calctimes_jic(i_ar,jic) = toc(t1);
      continue;
      % Debug: Zeichnen des Roboters in der Konfiguration
      if R.Type == 0 %#ok<UNRCH>
        R.Leg.qlim(:,:) = minmax2(QE');
      else
        for kkk = 1:R.NLEG
          R.Leg(kkk).qlim(:,:) = minmax2(QE(:,R.I1J_LEG(kkk):R.I2J_LEG(kkk))');
        end
      end
      cds_fitness_debug_plot_robot(R, QE(1,:)', Traj_0, Traj_0, Set, Structure, [], mean(fval), {});
    end
    % Gleiche Rechnung, nur für symmetrische Anordnung der PKM-Beinketten.
    % Annahme: Symmetrischer Aufbau, also zählen die Bewegungen aller Beine
    if R.Type == 2 && Set.optimization.joint_limits_symmetric_prismatic
      % Min-/Max-Werte für jede Beinkette einzeln ermitteln (Betrag für
      % Zylinder-Länge; sonst Umgehung durch negative Koordinate)
      qminmax_cyl_legs = reshape(minmax2(abs(QE(:,Structure.I_straightcylinder)')),...
        sum(Structure.I_straightcylinder(1:R.Leg(1).NJ)),2*R.NLEG);
      % Gemeinsame Min-/Max-Werte für alle Beinketten gemeinsam.
      qminmax_cyl = minmax2(qminmax_cyl_legs);
      length_cyl = qminmax_cyl(:,2) - qminmax_cyl(:,1);
      [fval_cyllen, Iworst] = max(length_cyl./qminmax_cyl(:,1));
      if fval_cyllen > 1
        constrvioltext_jic{jic} = sprintf(['Länge eines Schubzylinders steht ', ...
          'nach hinten über. Min. Abstand %1.1fmm, Innenzylinder Länge %1.1fmm ', ...
          '(Gelenk %d). Aufgrund symmetrischer Auslegung der Beinketten.'], ...
          1e3*qmin_cyl(Iworst), 1e3*length_cyl(Iworst), Iworst);
        fval_cyllen_norm = 2/pi*atan((fval_cyllen-1)*3); % Normierung auf 0 bis 1; 100% zu lang ist 0.8
        fval = 1e5*(4+0.25*fval_cyllen_norm); % Normierung auf 4e5 bis 4.25e5
        fval_jic(jic) = fval;
        calctimes_jic(i_ar,jic) = toc(t1);
        continue
      end
    end
  end
  %% Anpassung des Offsets für Schubgelenke
  % Hierdurch wird der Ort der Führungsschienen auf der Gelenkachse
  % verschoben. Das beeinflusst sowohl die Bauraumprüfung, als auch die
  % Prüfung auf Selbstkollision.
  if Structure.desopt_prismaticoffset
    % Bei Optimierung des Offsets wird auch bereits die Kollisionsprüfung
    % durchgeführt. Die Ergebnisse können weiter unten nur für die Bauraum-
    % prüfung genutzt werden.
    [~, fval_instspc_tmp] = cds_desopt_prismaticoffset(R, ...
      Traj_0.XE, Set, Structure, JPE, QE);
    if R.Type == 0, new_offset=R.DesPar.joint_offset(R.MDH.sigma==1);
    else, new_offset=R.Leg(1).DesPar.joint_offset(R.Leg(1).MDH.sigma==1); end
    cds_log(4, sprintf(['[constraints] Konfig %d/%d: Schubgelenk-Offset ', ...
      'wurde optimiert. Ergebnis: %1.1fmm'], jic, n_jic, 1e3*new_offset));
    % Kollisionskörper müssen nochmal aktualisiert werden (wegen Offset)
    [Structure.collbodies_robot, Structure.installspace_collbodies] = ...
      cds_update_collbodies(R, Set, Structure, QE);
  else
    fval_instspc_tmp = NaN; % ... Keine Berechnung durchgeführt. Erstmalige Berechnung unten erforderlich.
  end
  %% Selbst-Kollisionsprüfung für Einzelpunkte
  if Set.optimization.constraint_collisions
    % Keine Nutzung bereits vorliegender Daten (obige Berechnung mit
    % weniger Kollisionskörpern)
    [fval_coll, coll_self] = cds_constr_collisions_self(R, Traj_0.XE, Set, Structure, JPE, QE, [3e5;4e5]);
    if fval_coll > 0
      fval_jic(jic) = fval_coll; % Normierung auf 3e5 bis 4e5 bereits in Funktion
      constrvioltext_jic{jic} = sprintf('Selbstkollision in %d/%d AR-Eckwerten.', ...
        sum(any(coll_self,2)), size(coll_self,1));
      if fval_jic_old(jic) > 3e5 && fval_jic_old(jic) < 4e5 && fval_coll > fval_jic_old(jic)+1e-4
        cds_log(3, sprintf(['[constraints] Konfig %d/%d: Die Schwere der Kollisionen hat ', ...
          'sich trotz Optimierung vergrößert'], jic, n_jic)); % Gewertet wird die Eindringtiefe, nicht die Anzahl
      end
      calctimes_jic(i_ar,jic) = toc(t1);
      continue;
    elseif i_ar == 2 && fval_jic_old(jic) > 3e5 && fval_jic_old(jic) < 4e5
%       cds_log(3, sprintf('[constraints] Nach Optimierung keine Kollision mehr (Konfig %d/%d).', jic, n_jic));
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
  if fval_jic(jic) == 1e3 && ...% Nur die erste Konfiguration belassen. Geht schneller.
      all(abs(Q_jic(1,:,jic)' - Structure.q0_traj) < 1e-6) % Nur, falls Anfangswert schon gegeben
    % Bei der nachträglichen Auswertung soll es schneller gehen und die
    % alternativen Konfigurationen werden voraussichtlich sowieso nicht
    % besser sein.
    break;
  end
end % Schleife über IK-Konfigurationen

% Debuggen der IK-Optimierung (zum Nachvollziehen, ob die Nutzung von
% Redundanz an dieser Stelle überhaupt etwas bringt)
if Structure.task_red
  cds_log(3, sprintf(['[constraints] Eckpunkte mit %d Konfigurationen berechnet. Dauer ', ...
    'AR=0 %1.1fs (mean %1.2fs; n=%d): AR=1 %1.1fs (mean %1.2fs; n=%d)'], ...
    size(calctimes_jic,2), sum(calctimes_jic(1,:),'omitnan'), ...
    mean(calctimes_jic(1,:),'omitnan'), sum(~isnan(calctimes_jic(1,:))), ...
    sum(calctimes_jic(2,:),'omitnan'), mean(calctimes_jic(2,:),'omitnan'), ...
    sum(~isnan(calctimes_jic(2,:)))));
  if any(~isnan(arikstats_jic(:)))
    cds_log(3, sprintf(['[constraints] Insgesamt für %d Versuche (von max. %dx%d=%d) die IK nochmal ', ...
      'neu gerechnet. Dabei %d mal mit Verbesserung, %d mal mit Verschlechterung ', ...
      'der Zusatzoptimierung. %d Mal Kein Erfolg der AR-IK.'], sum(~isnan(arikstats_jic(:))), ...
      size(arikstats_jic,1), size(arikstats_jic,2), length(arikstats_jic(:)), ...
      sum(arikstats_jic(:)<0), sum((arikstats_jic(:)>0)&(arikstats_jic(:)~=inf)), ...
      sum(arikstats_jic(:)==inf) ) );
  end
end
if Set.general.debug_calc && Structure.task_red
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
  bestcolldist_jic = bestcolldist_jic(jic_best);
  bestinstspcdist_jic = bestinstspcdist_jic(jic_best);
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
  Q0_unique = unique(round(Q0,2), 'rows');
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
  bestcolldist_jic = bestcolldist_jic(I);
  bestinstspcdist_jic = bestinstspcdist_jic(I);
  
  % Ausgabe der IK-Werte für alle Eckpunkte. Im weiteren Verlauf der
  % Optimierung benötigt, falls keine Trajektorie berechnet wird.
  QE_all = Q_jic(:,:,I_iO);
  % Debug: Zeige die verschiedenen Lösungen an
  if Set.general.plot_details_in_fitness < 0 && 1e4*fval >= abs(Set.general.plot_details_in_fitness) || ... % Gütefunktion ist schlechter als Schwellwert: Zeichne
     Set.general.plot_details_in_fitness > 0 && 1e4*fval <= abs(Set.general.plot_details_in_fitness)
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
Stats_constraints = struct('bestcolldist', bestcolldist_jic, 'bestinstspcdist', bestinstspcdist_jic);
