% Nebenbedingung für die Maßsynthese von Robotern: Materialspannung
% Gibt einen Fehlerwert bei Überschreitung der zulässigen
% Materialspannungen durch die Schnittkräfte und -Momente in den Segmenten
% 
% Eingabe:
% R
%   Matlab-Klasse für Roboter (SerRob/ParRob)
% Set
%   Einstellungen des Optimierungsalgorithmus
% data_dyn
%   Struktur mit Dynamik-Eigenschaften des Roboters als Zeitreihe über die
%   Trajektorie. Felder:
%   Wges
%     Schnittkräfte in allen Segmenten des Roboters. Format siehe
%     cds_obj_dependencies_regmult bzw. cds_obj_dependencies
% Jinvges
%   Zeilenweise (inverse) Jacobi-Matrizen des Roboters (für PKM). Bezogen
%   auf vollständige Gelenkgeschwindigkeiten und Plattform-Geschw. (in
%   x-Koordinaten, nicht: Winkelgeschwindigkeit)
% Q
%   Gelenkwinkel-Trajektorie
% Traj_0
%   Roboter-Trajektorie (EE) bezogen auf Basis-KS des Roboters
% 
% Ausgabe:
% fval
%   Grad der Verletzung der Nebenbedingung für Materialspannung als
%   normierte Nebenbedingung
%   0: Alles i.O.
%   1e4..1e5: Nebenbedingung nicht erfüllt
% constrvioltext
%   Fehlertext
% f_maxstrengthviol
%   Grad der Ausnutzung der Materialgrenzen:
%   1=Grenzwert gerade so erfüllt; 10=Grenzwert zehnfach überschritten.
%   (Ohne Berücksichtigung des Sicherheitsfaktors)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-01
% (C) Institut für Mechatronische Systeme, Universität Hannover

function [fval, constrvioltext, f_maxstrengthviol] = cds_constr_yieldstrength(R, Set, data_dyn, Jinv_ges, Q, Traj_0)
fval = 0;
constrvioltext = '';

%% Konstanten, Definitionen
% Sicherheitsfaktor für Materialspannung
safety_factor = Set.optimization.safety_link_yieldstrength;
% Dehngrenze von Aluminium-Legierung. Quellen:
% https://de.wikipedia.org/wiki/Streckgrenze
% https://de.wikipedia.org/wiki/Aluminium-Kupfer-Legierung
if any(R.Type == [0 1])
  R_e=R.DesPar.material(4);
else
  R_e=R.Leg(1).DesPar.material(4);
end

%% Berechnung der Materialspannung
if any(R.Type == [0 1]) % Seriell
  NLEG = 1; NL = R.NL;
  % 2D-Matrix mit Zeit als Zeilen
  nt = size(data_dyn.Wges,1);
else % PKM
  NLEG = R.NLEG; NL = R.Leg(1).NL;
  % 3D-Matrix mit Zeit als dritte Dimension
  nt = size(data_dyn.Wges,3);
end
f_yieldstrength = NaN(7,NLEG);
sigma_ges_alle = NaN(NLEG, NL, nt);
I_sigma_exc = 0;
for i = 1:NLEG
  % Schnittkräfte dieser Beinkette
  if any(R.Type == [0 1])
    Rob = R;
    Wges = data_dyn.Wges;
  else
    Rob = R.Leg(i);
    Wges_i = data_dyn.Wges(i,:,:);
    Wges = reshape(Wges_i, 6*Rob.NL, nt)'; % in gleiches Format wie SerRob bringen
  end
  % Effektivwert von Kraft und Moment im Zeitverlauf bestimmen
  % Alle Segmente durchgehen, auch die Basis (Index 1). Nehme an, dass
  % Basis immer ausreichend stark dimensioniert werden kann. Bei Schub-
  % gelenken kann aber nur an der Basis die Länge des Segments
  % berücksichtigt werden.
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
[f_maxstrengthviol, I_maxstrengthviol] = max(f_yieldstrength(:));
if any(f_maxstrengthviol > 1/safety_factor) % Prüfe gegen Sicherheitsfaktor. Bspw. bei SF=2 nur max. f_yieldstrength=50% erlaubt.
  % Normiere auf Wert zwischen 0 und 1
  f_maxstrengthviol_norm = 2/pi*atan(f_maxstrengthviol-1/safety_factor); % 1/safety_factor->0; 10/safety_factor->0.93
  fval = 1e4*(1+9*f_maxstrengthviol_norm); % Normiere in Bereich 1e4...1e5
  constrvioltext = sprintf(['Materialbelastungsgrenze überschritten ' ...
    '(%d/%d Prüfungen über 100%%; %d über Sicherheitsfaktor %1.0f%%; max Faktor %1.1f).'], ...
    sum(f_yieldstrength(:)>1), length(f_yieldstrength(:)), sum(f_yieldstrength(:)>1/safety_factor), 100*safety_factor, f_maxstrengthviol);
end

%% Debug-Plot für Schnittkräfte
if Set.general.plot_details_in_desopt < 0 && fval >= abs(Set.general.plot_details_in_desopt) || ... % Gütefunktion ist schlechter als Schwellwert: Zeichne
   Set.general.plot_details_in_desopt > 0 && fval <= abs(Set.general.plot_details_in_desopt) % Gütefunktion ist besser als Schwellwert: Zeichne
  % Reduziere wirksame Materialspannungsgrenze mit Sicherheitsfaktor
  R_e_eff = R_e / safety_factor; % nur zum Plotten
  % Zeichne Materialspannungen (maßgeblich für Auslegung)
  change_current_figure(1999);clf;
  set(1999, 'Name', 'Materialspannung', 'NumberTitle', 'off');
  if any(R.Type == [0 1]) % Seriell
    sphdl = NaN(3, 3);
    i = 1;
    for j = 1:NL
      sphdl(j) = subplot(3, 3, j); hold on; grid on
      plot(Traj_0.t, squeeze(sigma_ges_alle(i,j,:)));
      plot(Traj_0.t([1 end]), R_e_eff*[1;1], 'r-');
      if j == 1, ylabel(sprintf('Segment %d', i)); end
    end
  else % PKM
    sphdl = NaN(NL, NLEG);
    for k = 1:NLEG % Beine in den Spalten des Bildes
      for j = 1:NL
        sphdl(j,k)=subplot(size(sphdl,1),size(sphdl,2),sprc2no(size(sphdl,1),size(sphdl,2),j,k)); 
        hold on; grid on;
        plot(Traj_0.t, squeeze(sigma_ges_alle(k,j,:)));
        plot(Traj_0.t([1 end]), R_e_eff*[1;1], 'r-');
        plot(Traj_0.t([1 end]), R_e*[1;1], 'g-');
        if j == 1, title(sprintf('Beinkette %d', k)); end
        if k == 1, ylabel(sprintf('Seg. %d', j-1)); end
      end
    end
    remove_inner_labels(sphdl,1); 
  end
  sgtitle('Materialspannung (N/mm²)');
  linkxaxes
  
  % Zeichne Schnittkräfte
  for fm = [0 1] % Schnittkräfte (0) und -momente (1)
    % Bild initialisieren
    change_current_figure(2000+fm);clf;
    if fm == 0, set(2000+fm, 'Name', 'Schnittkräfte', 'NumberTitle', 'off');
    else, set(2000+fm, 'Name', 'Schnittmoment', 'NumberTitle', 'off'); end
    if any(R.Type == [0 1]) % Seriell
      sphdl=NaN(3, 3);
      for j = 1:R.NL
        if fm == 0, fmnorm_ges_jk = sqrt(sum(data_dyn.Wges(:,3*(j-1)+(1:3)).^2,2)); % s.o.
        else, fmnorm_ges_jk = sqrt(sum(data_dyn.Wges(:,R.NL*3+3*(j-1)+(1:3)).^2,2)); end
        fmnorm_ges_jk(abs(fmnorm_ges_jk)<1e-12) = 0; % numerisches Rauschen ignorieren
        sphdl(j)=subplot(3, 3, j); hold on;
        plot(Traj_0.t, fmnorm_ges_jk);
        % Formatierung
        grid on;
        ylabel(sprintf('Seg. %d', j-1));
      end
    else % PKM
      sphdl=NaN(R.Leg(1).NL, R.NLEG);
      for k = 1:R.NLEG % Beine in den Spalten des Bildes
        % Vektor der Schnittkräfte für aktuelles Bein extrahieren
        W_k = squeeze(data_dyn.Wges(k,:,:))';
        for j = 1:R.Leg(1).NL % Beingelenke in den Zeilen des Bildes
          % Kräfte und Momente in den Gesamtdaten extrahieren
          if fm == 0, fmnorm_ges_jk = sqrt(sum(W_k(:,3*(j-1)+(1:3)).^2,2)); % s.o.
          else, fmnorm_ges_jk = sqrt(sum(W_k(:,R.Leg(1).NL*3+3*(j-1)+(1:3)).^2,2)); end
          fmnorm_ges_jk(abs(fmnorm_ges_jk)<1e-12) = 0; % numerisches Rauschen ignorieren
          % Plotten
          sphdl(j,k)=subplot(size(sphdl,1),size(sphdl,2),sprc2no(size(sphdl,1),size(sphdl,2),j,k)); hold on;
          plot(Traj_0.t, fmnorm_ges_jk);
          % Formatierung
          grid on;
          if j == 1, title(sprintf('Beinkette %d', k)); end
          if k == 1, ylabel(sprintf('Seg. %d', j-1)); end
        end
      end
      remove_inner_labels(sphdl,1); 
    end
    linkxaxes
    if fm == 0,     sgtitle('Schnittkräfte (Betrag)');
    elseif fm == 1, sgtitle('Schnittmomente (Betrag)'); end
  end
  
  if R.Type == 2 % PKM
    % Zeichne Verlauf der Konditionszahl, zur Einordnung, ob
    % Überschreitung aufgrund von Singularität auftritt
    % Berechne Konditionszahl für alle Punkte der Bahn. Zusätzlich
    % Manipulierbarkeit. Berechnung für jede Beinkette einzeln.
    Jdet_all = NaN(length(Traj_0.t), NLEG+1); % Determinanten
    Jcond_all = Jdet_all; % Konditionszahlen
    for i = 1:length(Traj_0.t)
      Jinv_IK = reshape(Jinv_ges(i,:), R.NJ, sum(R.I_EE));
      for k = 1:NLEG
        Jinv_IK_k = Jinv_IK(R.I1J_LEG(k):R.I2J_LEG(k),:);
        Jdet_all(i,k) = log10(abs(sqrt(det(Jinv_IK_k'*Jinv_IK_k))));
        Jcond_all(i,k) = cond(Jinv_IK_k);
      end
      Jdet_all(i,end) = log10(sqrt(abs(det(Jinv_IK(R.I_qa,:)'*Jinv_IK(R.I_qa,:)))));
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
  end
  
  % Zeichne den Roboter in einer kritischen Konfiguration. Färbe das am
  % stärksten überlastete Segment ein.
  change_current_figure(1997);clf;
  set(1997, 'Name', 'Spannungsgrenze_Roboter', 'NumberTitle', 'off');
  view(3); axis auto; hold on; grid on;
  xlabel('x in m');ylabel('y in m');zlabel('z in m');
  if any(R.Type == [0 1]) % Seriell
    s_plot = struct( 'ks', [1, R.NL+1], 'straight', 1, 'mode', 4);
    R.plot( Q(I_sigma_exc,:)', s_plot);
  else % PKM
    s_plot = struct( 'ks_legs', R.I1L_LEG, 'straight', 1, 'mode', 4);
    R.plot( Q(I_sigma_exc,:)', Traj_0.X(I_sigma_exc,:)', s_plot);
  end
  % Finde den Ort der Überlastung heraus
  [iLink,iLeg] = ind2sub(size(f_yieldstrength),I_maxstrengthviol);
  for c = get(gca, 'children')'
    if ~contains(get(c, 'DisplayName'), 'Leg')
      continue % Ist kein 3D-Körper, der in plot-Funktion mit Namen versehen wurde.
    end
    if strcmp(get(c, 'DisplayName'), sprintf('Leg%d_Link%d', iLeg, iLink-1))
      set(c, 'edgeColor', [1 0 0]);
    end
  end
  sgtitle(sprintf(['CAD-Modell bei max. Überschreitung der Belastungsgrenze ', ...
    '(Bein %d, Seg. %d)'], iLeg, iLink));
end
