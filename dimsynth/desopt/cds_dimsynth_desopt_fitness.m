% Gütefunktion für Maßsynthese von Robotern (allgemein)
% 
% Eingabe:
% R
%   Matlab-Klasse für Roboter (SerRob/ParRob)
% Traj_0
%   Roboter-Trajektorie (EE) bezogen auf Basis-KS des Roboters
% Set
%   Einstellungen des Optimierungsalgorithmus
% Q, QD, QDD
%   Gelenkwinkel-Trajektorie
% Jinvges
%   Zeilenweise (inverse) Jacobi-Matrizen des Roboters (für PKM). Bezogen
%   auf vollständige Gelenkgeschwindigkeiten und Plattform-Geschw. (in
%   x-Koordinaten, nicht: Winkelgeschwindigkeit)
% Structure
%   Eigenschaften der Roboterstruktur
% p_desopt
%   Vektor der Optimierungsvariablen für PSO. Siehe cds_dimsynth_design.
%
% Ausgabe:
% fval
%   Fitness-Wert für den Parametervektor p. Enthält Strafterme für
%   Verletzung von Nebenbedingungen oder Wert der Zielfunktion (je nachdem)
%   Werte:
%   0...1e3: gewählte Zielfunktion
%   1e3...1e4: Nebenbedingung von Zielfunktion überschritten
%   1e4...1e5: Überschreitung Belastungsgrenze der Segmente
%   1e8...1e9: Unplausible Eingabe (Radius vs Wandstärke)
% 
% Siehe auch: cds_fitness.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-01
% (C) Institut für Mechatronische Systeme, Universität Hannover

function fval = cds_dimsynth_desopt_fitness(R, Set, Traj_0, Q, QD, QDD, Jinv_ges, data_dyn_reg, Structure, p_desopt)
t1 = tic();
% Debug:
if Set.general.matfile_verbosity > 3
  repopath = fileparts(which('structgeomsynth_path_init.m'));
  save(fullfile(repopath, 'tmp', 'cds_dimsynth_desopt_fitness.mat'));
end
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_desopt_fitness.mat'));

fval = 0;

%% Plausibilität der Eingabe prüfen
if p_desopt(1) > p_desopt(2)/2 % Wandstärke darf nicht größer als Radius sein
  f_wall_vs_rad = (p_desopt(1) - p_desopt(2)/2)/p_desopt(2); % Grad der Überschreitung
  f_wall_vs_rad_norm = 2/pi*atan(f_wall_vs_rad); % 50% -> 0.3
  fval = 1e8*(1+9*f_wall_vs_rad_norm); % Normierung auf 1e8...1e9
  constrvioltext = sprintf('Radius (%1.1fmm) ist kleiner als Wandstärke (%1.1fmm)', ...
    1e3*p_desopt(2)/2, 1e3*p_desopt(1));
end

%% Dynamikparameter aktualisieren
% Trage die Dynamikparameter
if fval == 0
  cds_dimsynth_design(R, Q, Set, Structure, p_desopt);
end
%% Dynamik neu berechnen
if fval == 0 && Structure.calc_reg
  % Abhängigkeiten neu berechnen (Dynamik)
  data_dyn = cds_obj_dependencies_regmult(R, Set, data_dyn_reg);
  if Set.general.debug_calc
    % Zu Testzwecken die Dynamik neu ohne Regressorform berechnen und mit
    % Regressor-Berechnung vergleichen
    data_dyn2 = cds_obj_dependencies(R, Traj_0, Set, Structure, Q, QD, QDD, Jinv_ges);
    test_TAU = data_dyn2.TAU - data_dyn.TAU;
    if any(abs(test_TAU(:))>1e-8)
      error('Antriebskräfte aus Regressorform stimmt nicht');
    end
    test_W = data_dyn2.Wges - data_dyn.Wges;
    if any(abs(test_W(:))>1e-8)
      error('Schnittkräfte aus Regressorform stimmt nicht');
    end
  end
end

%% Nebenbedingungen der Entwurfsvariablen berechnen: Festigkeit der Segmente
if fval == 0 && Set.optimization.desopt_link_yieldstrength
  % Konstanten, Definitionen
  % Dehngrenze von Aluminium-Legierung. Quellen:
  % https://de.wikipedia.org/wiki/Streckgrenze
  % https://de.wikipedia.org/wiki/Aluminium-Kupfer-Legierung
  R_e=250e6;
  
  if R.Type == 0 % Seriell
    NLEG = 1; NL = R.NL;
  else % PKM
    NLEG = R.NLEG; NL = R.Leg(1).NL;
  end
  f_yieldstrength = NaN(7,NLEG);
  sigma_ges_alle = NaN(NLEG, NL, length(Traj_0.t));
  I_sigma_exc = 0;
  for i = 1:NLEG
    % Schnittkräfte dieser Beinkette
    if Structure.Type == 0
      Rob = R;
      Wges = data_dyn.Wges;
    else
      Rob = R.Leg(i);
      Wges_i = data_dyn.Wges(i,:,:);
      Wges = reshape(Wges_i, 6*Rob.NL, size(Q,1))'; % in gleiches Format wie SerRob bringen
    end
    % Effektivwert von Kraft und Moment im Zeitverlauf bestimmen
    % Alle Segmente durchgehen
    for j = 1:Rob.NL
      % Parameter des Segmentes laden
      e_j = Rob.DesPar.seg_par(i,1); % Wandstärke
      R_j = 0.5*Rob.DesPar.seg_par(i,2); % Radius des Hohlzylinders
      % Querschnittsfläche des Balkenmodells (aus Hohlzylinder)
      A0 = pi*( R_j^2 - (R_j-e_j)^2 ); % Differenz Außen- und Innenkreis
      % Widerstandsmoment Hohlzylinder. Ettemeyer, Schäfer, Wallrapp, TM2, II.41
      W = pi/4*( (R_j^4 - (R_j-e_j)^4)/R_j );
      % Bestimme die maximale Kraft und Moment am basisnäheren Gelenk des
      % Segments. Das ist die größte Schnittkraft, die an diesem Segment
      % wirkt (Hypothese).
      F_eff_ges=sqrt(sum(Wges(:,3*(j-1)+(1:3)).^2,2)); % Beträge der Schnittkraft über die Zeit
      M_eff_ges=sqrt(sum(Wges(:,Rob.NL*3+3*(j-1)+(1:3)).^2,2)); % Schnittmomente
      % Bestimme die Materialspannung bei Annahme, dass Kraft und Moment
      % immer einen einachsigen Belastungsfall des Segments als Biegebalken
      % darstellen (Hypothese, konservative Annahme)
      sigma_ges = F_eff_ges/A0 + M_eff_ges/W;
      % Maximale Spannung ist maßgeblich für die Festigkeitsuntersuchung
      % Quelle: Gross, Hauger, TM2 (Elastostatik), S. 163
      [sigma_max, I_max_ij] = max(sigma_ges);
      % TODO: Momente in "Balkenrichtung" umrechnen und komplexere Formel
      %       nehmen (Aufteilung in Biegespannung und Torsionsspannung),
      %       Vergleichsspannung z.B. nach von Mises
      % Prüfe, ob Vergleichsspannung größer als Streckgrenze/Dehngrenze ist
      f_yieldstrength(j,i) = sigma_max/R_e;
      
      % Speichere zusätzliche Informationen zum Erstellen der Bilder
      sigma_ges_alle(i,j,:) = sigma_ges;
      if f_yieldstrength(j,i) >= max(f_yieldstrength(:)), I_sigma_exc = I_max_ij; end
    end
  end
  % Prüfe, ob für ein Segment die Materialspannung überschritten wurde
  if any(f_yieldstrength(:)>1)
    f_maxstrengthviol = max(f_yieldstrength(:));
    % Normiere auf Wert zwischen 0 und 1
    f_maxstrengthviol_norm = 2/pi*atan(f_maxstrengthviol-1); % 1->0; 10->0.93
    fval = 1e4*(1+9*f_maxstrengthviol_norm); % Normiere in Bereich 1e4...1e5
    constrvioltext = sprintf('Materialbelastungsgrenze überschritten (%d/%d mal; max Faktor %1.1f)', ...
      sum(f_yieldstrength(:)>1), length(f_yieldstrength(:)), f_maxstrengthviol);
  end
  
  %% Debug-Plot für Schnittkräfte
  if Set.general.plot_details_in_desopt < 0 && fval >= abs(Set.general.plot_details_in_desopt) || ... % Gütefunktion ist schlechter als Schwellwert: Zeichne
     Set.general.plot_details_in_desopt > 0 && fval <= abs(Set.general.plot_details_in_desopt) % Gütefunktion ist besser als Schwellwert: Zeichne
    if R.Type == 0 % Seriell
    else % PKM
      % Zeichne Materialspannungen (maßgeblich für Auslegung)
      change_current_figure(1999);clf;
      set(1999, 'Name', 'Materialspannung', 'NumberTitle', 'off');
      sphdl = NaN(NLEG, NL);
      for i = 1:NLEG
        for j = 1:NL
          sphdl(i,j) = subplot(NLEG, NL, sprc2no(NLEG, NL, i, j)); hold on; grid on
          plot(Traj_0.t, squeeze(sigma_ges_alle(i,j,:)));
          plot(Traj_0.t([1 end]), R_e*[1;1], 'r-');
          if i == 1, title(sprintf('Bein %d', j)); end
          if j == 1, ylabel(sprintf('Segment %d', i)); end
        end
      end
      sgtitle('Materialspannung (N/mm²)');
      linkxaxes
      remove_inner_labels(sphdl,1); 
      
      % Zeichne Schnittkräfte
      for fm = [0 1] % Schnittkräfte (0) und -momente (1)
        % Bild initialisieren
        change_current_figure(2000+fm);clf;
        if fm == 0, set(2000+fm, 'Name', 'Schnittkräfte', 'NumberTitle', 'off');
        else, set(2000+fm, 'Name', 'Schnittmoment', 'NumberTitle', 'off'); end
        sphdl=NaN(R.Leg(1).NL, R.NLEG);
        for k = 1:R.NLEG % Beine in den Spalten des Bildes
          % Vektor der Schnittkräfte für aktuelles Bein extrahieren
          W_k = squeeze(data_dyn.Wges(k,:,:))';
          for j = 1:R.Leg(1).NL % Beingelenke in den Zeilen des Bildes
            % Kräfte und Momente in den Gesamtdaten extrahieren
            if fm == 0, fmnorm_ges_jk = sqrt(sum(W_k(:,3*(j-1)+(1:3)).^2,2)); % s.o.
            else, fmnorm_ges_jk = sqrt(sum(W_k(:,Rob.NL*3+3*(j-1)+(1:3)).^2,2)); end
            % Plotten
            sphdl(j,k)=subplot(size(sphdl,1),size(sphdl,2),sprc2no(size(sphdl,1),size(sphdl,2),j,k)); hold on;
            plot(Traj_0.t, fmnorm_ges_jk);
            % Formatierung
            grid on;
            if j == 1, title(sprintf('Beinkette %d', k)); end
            if k == 1, ylabel(sprintf('Seg. %d', j-1)); end
          end
        end
        if fm == 0,     sgtitle('Schnittkräfte (Betrag)');
        elseif fm == 1, sgtitle('Schnittmomente (Betrag)'); end
        linkxaxes
        remove_inner_labels(sphdl,1); 
      end
      
      % Zeichne Verlauf der Konditionszahl, zur Einordnung, ob
      % Überschreitung aufgrund von Singularität auftritt
      % Berechne Konditionszahl für alle Punkte der Bahn
      Jdet_all = NaN(length(Traj_0.t), NLEG+1); % Determinanten
      Jcond_all = Jdet_all; % Konditionszahlen
      for i = 1:length(Traj_0.t)
        Jinv_IK = reshape(Jinv_ges(i,:), R.NJ, sum(R.I_EE));
        for k = 1:NLEG
          Jinv_IK_k = Jinv_IK(R.I1J_LEG(k):R.I2J_LEG(k),:);
          Jdet_all(i,k) = log10(abs(det(Jinv_IK_k)));
          Jcond_all(i,k) = cond(Jinv_IK_k);
        end
        Jdet_all(i,end) = log10(abs(det(Jinv_IK(R.I_qa,:))));
        Jcond_all(i,end) = cond(Jinv_IK(R.I_qa,:));
      end
      change_current_figure(1998);clf;
      set(1998, 'Name', 'Jacobi', 'NumberTitle', 'off');
      subplot(2,2,1);
      plot(Traj_0.t, Jdet_all(:,1:NLEG)); grid on;
      ylabel('Log-Determinante Beinketten');
      subplot(2,2,2);
      plot(Traj_0.t, Jdet_all(:,end)); grid on;
      ylabel('Log-Determinante Antriebe');
      subplot(2,2,3);
      plot(Traj_0.t, Jcond_all(:,1:NLEG)); grid on;
      ylabel('Kondition Beinketten');
      subplot(2,2,4);
      plot(Traj_0.t, Jcond_all(:,end)); grid on;
      ylabel('Kondition Antriebe');
      linkxaxes
      sgtitle('Eigenschaften der Jacobi-Matrix');
    
      % Zeichne den Roboter in einer kritischen Konfiguration
      change_current_figure(1997);clf;
      set(1997, 'Name', 'Robot_YieldStrength', 'NumberTitle', 'off');
      view(3);
      axis auto
      hold on;grid on;
      xlabel('x in m');ylabel('y in m');zlabel('z in m');
      s_plot = struct( 'ks_legs', [1,2], 'straight', 0, 'mode', 4);
      R.plot( Q(I_sigma_exc,:)', Traj_0.X(I_sigma_exc,:)', s_plot);
      sgtitle('CAD-Modell bei max. Überschreitung der Belastungsgrenze');
    end
  end
end

%% Nebenbedingungen der Zielfunktionswerte berechnen
if fval == 0 && any(Set.optimization.constraint_obj)
  if Set.optimization.constraint_obj(1) % NB für Masse gesetzt
    [fval_mass, fval_debugtext_mass, ~, fphys] = cds_obj_mass(R);
    viol_rel = (fphys - Set.optimization.constraint_obj(1))/Set.optimization.constraint_obj(1);
    if viol_rel > 0
      f_massvio_norm = 2/pi*atan((viol_rel)); % 1->0.5; 10->0.94
      fval = 1e3*(1+9*f_massvio_norm); % 1e3 ... 1e4
      constrvioltext = sprintf('Masse ist zu groß (%1.1f > %1.1f)', fphys, Set.optimization.constraint_obj(1));
    end
  end
  if any(Set.optimization.constraint_obj([2 3]))
    error('Grenzen für andere Zielfunktionen als die Masse noch nicht implementiert');
  end
end
if fval > 1000 % Nebenbedingungen verletzt.
  if Set.general.verbosity > 3
    fprintf('[desopt] DesOpt-Fitness-Evaluation in %1.1fs. fval=%1.3e. %s\n', toc(t1), fval, constrvioltext);
  end
  return
end

%% Fitness-Wert berechnen
% TODO: Die gleichen Zielfunktionen wie für die Maßsynthese können auch in
% der Entwurfsoptimierung berechnet werden
if strcmp(Set.optimization.objective, 'mass')
  if Set.optimization.constraint_obj(1) % Vermeide doppelten Aufruf der Funktion
    fval = fval_mass; % Nehme Wert von der NB-Berechnung oben
    fval_debugtext = fval_debugtext_mass;
  else
    [fval,fval_debugtext] = cds_obj_mass(R);
  end
elseif strcmp(Set.optimization.objective, 'energy')
  if Set.optimization.constraint_obj(2) % Vermeide doppelten Aufruf der Funktion
    fval = fval_energy; % Nehme Wert von der NB-Berechnung oben
    fval_debugtext = fval_debugtext_energy;
  else
    [fval,fval_debugtext] = cds_obj_energy(R, Set, Structure, Traj_0, data_dyn.TAU, QD);
  end
else
  error('Andere Zielfunktion als Masse noch nicht implementiert');
end
if Set.general.verbosity >3
  fprintf('[desopt] DesOpt-Fitness-Evaluation in %1.1fs. Parameter: [%s]. fval=%1.3e. Erfolgreich. %s.\n', ...
    toc(t1), disp_array(p_desopt', '%1.3f'), fval, fval_debugtext);
end
