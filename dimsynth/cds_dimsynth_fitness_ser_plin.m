% Gütefunktion für serielle Roboter unter Ausnutzung der Regressorform

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function fval = cds_dimsynth_fitness_ser_plin(R, Set, Traj_W, Structure, p)
% Debug: 
% save(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_fitness_ser_plin.mat'));
% error('Halte hier');
% load(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_fitness_ser_plin.mat'));
t1=tic();
debug_info = {};
%% Parameter prüfen
if p(1) == 0
  error('Roboterskalierung kann nicht Null werden');
end

%% Parameter aktualisieren
% Keine Verwendung der Ausgabe: Parameter werden direkt in ursprüngliche
% Funktion geschrieben; R.pkin ist vor/nach dem Aufruf unterschiedlich
cds_update_robot_parameters(R, Set, Structure, p);

%% Trajektorie anpassen
Traj_0 = cds_rotate_traj(Traj_W, R.T_W_0);

%% Geometrie auf Plausibilität prüfen
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
  %  normiere auf 1e6 bis 1e7
  fval = 1e6+9e6*f_distviol_norm;
  fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3e. Roboter zu kurz.\n', toc(t1), fval);
  debug_plot_robot(R, zeros(R.NJ,1), Traj_W, Set, Structure, p, fval, debug_info);
  return
end
%% Inverse Kinematik für Eckpunkte der Trajektorie berechnen
q0 = rand(R.NQJ,1);
Phi_E = NaN(sum(Set.structures.DoF), size(Traj_0.XE,1));
for i = size(Traj_0.XE,1):-1:1
  s = struct('Phit_tol', 1e-3, 'Phir_tol', 1e-3, 'retry_limit', 5);
  [q, Phi] = R.invkin2(Traj_0.XE(i,:)', q0, s);
  q0 = q; % Annahme: Startwert für nächsten Eckwert nahe aktuellem Eckwert
  Phi_E(:,i) = Phi;
  if any(isnan(q))
    error('Ergebnis NaN');
  end
end
if any(abs(Phi_E(:)) > 1e-3)
  % Nehme die Summe der IK-Abweichung aller Eckpunkte (Translation/Rotation
  % gemischt)
  f_PhiE = max(sum(abs(Phi_E(:)))) / size(Traj_0.XE,1);
  f_phiE_norm = 2/pi*atan((f_PhiE)/10); % Normierung auf 0 bis 1
  fval = 1e5+9e5*f_phiE_norm; % Normierung auf 1e5 bis 1e6
  % Keine Konvergenz der IK. Weitere Rechnungen machen keinen Sinn.
  fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3e. Keine IK-Konvergenz\n', toc(t1), fval);
  debug_plot_robot(R, zeros(R.NJ,1), Traj_W, Set, Structure, p, fval, debug_info);
  return
end
%% Inverse Kinematik der Trajektorie berechnen
[Q, QD, QDD, PHI] = R.invkin2_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD, Traj_0.t, q);
I_ZBviol = any(abs(PHI) > 1e-3,2) | any(isnan(Q),2);
if any(I_ZBviol)
  % Bestimme die erste Verletzung der ZB (je später, desto besser)
  IdxFirst = find(I_ZBviol, 1 );
  % Umrechnung in Prozent der Traj.
  Failratio = 1-IdxFirst/length(Traj_0.t); % Wert zwischen 0 und 1
  fval = 1e4+9e4*Failratio; % Wert zwischen 1e4 und 1e5 -> IK-Abbruch bei Traj.
  % Keine Konvergenz der IK. Weitere Rechnungen machen keinen Sinn.
  fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3e. Keine IK-Konvergenz in Traj.\n', toc(t1), fval);
  debug_plot_robot(R, Q(1,:)', Traj_W, Set, Structure, p, fval, debug_info);
  return
end
%% Dynamik-Parameter
if strcmp(Set.optimization.objective, 'energy') || strcmp(Set.optimization.objective, 'mass')
  % Dynamik-Parameter aktualisieren. Keine Nutzung der Ausgabe der Funktion
  % (Parameter werden direkt in Klasse geschrieben; R.DesPar.seg_par ist
  % vor/nach dem Aufruf unterschiedlich)
  cds_dimsynth_desopt(R, Q, Set, Structure);
end
%% Zielfunktion berechnen
if strcmp(Set.optimization.objective, 'condition')
  Cges = NaN(length(Traj_0.t), 1);
  % Berechne Konditionszahl für alle Punkte der Bahn
  for i = 1:length(Traj_0.t)
    J_3T3R = R.jacobig(Q(i,:)');
    J_task = J_3T3R(Set.structures.DoF,:);
    Cges(i,:) = cond(J_task);
  end
  % Schlechtester Wert der Konditionszahl
  % Nehme Logarithmus, da Konditionszahl oft sehr groß ist.
  f_cond1 = max(Cges);
  f_cond = log(f_cond1); 
  f_cond_norm = 2/pi*atan((f_cond)/20); % Normierung auf 0 bis 1; 150 ist 0.9
  fval = 1e3*f_cond_norm; % Normiert auf 0 bis 1e3
  fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3f. Konditionszahl %1.3e\n', toc(t1), fval, f_cond1);
elseif strcmp(Set.optimization.objective, 'energy')
  % Antriebskräfte berechnen
  TAU = R.invdyn2_traj(Q, QD, QDD);
  % Aktuelle mechanische Leistung in allen Gelenken
  P_ges = NaN(size(Q,1), R.NJ);
  for j = 1:R.NJ
    P_ges(:,j) = TAU(:,j) .* QD(:,j);
  end
  % Energie berechnen
  if Set.optimization.ElectricCoupling
    % Mit Zwischenkreis: Summe aller Leistungen
    P_Kreis = sum(P_ges,2);
    % Negative Leistungen im Kreis abschneiden (keine Rückspeisung ins
    % Netz)
    P_Netz = P_Kreis;
    P_Netz(P_Netz<0) = 0;
  else
    % Ohne Zwischenkreis: Negative Leistungen der Antriebe abschneiden
    P_ges_cut = P_ges;
    P_ges_cut(P_ges_cut<0) = 0;
    P_Kreis = sum(P_ges_cut,2);
    % Kein weiteres Abschneiden notwendig. Kreisleistung kann nicht negativ
    % sein. Keine Rückspeisung ins Netz möglich.
    P_Netz = P_Kreis;
  end
  E_Netz_res = sum(trapz(Traj_0.t, P_Netz)); % Integral der Leistung am Ende
  f_en_norm = 2/pi*atan((E_Netz_res)/100); % Normierung auf 0 bis 1; 620 ist 0.9
  fval = 1e3*f_en_norm; % Normiert auf 0 bis 1e3
  
  if fval < Set.general.plot_details_in_fitness
    E_Netz = cumtrapz(Traj_0.t, P_Netz);
    figure(202);clf;
    if Set.optimization.ElectricCoupling, sgtitle('Energieverteilung (mit Zwischenkreis)');
    else,                                 sgtitle('Energieverteilung (ohne Zwischenkreis'); end
    subplot(2,2,1);
    plot(Traj_0.t, P_ges);
    ylabel('Leistung Achsen'); grid on;
    subplot(2,2,2); hold on;
    plot(Traj_0.t, P_Kreis);
    plot(Traj_0.t, P_Netz, '--');
    plot(Traj_0.t, E_Netz);
    plot(Traj_0.t(end), E_Netz_res, 'o');
    ylabel('Leistung/Energie Gesamt'); legend({'P(Zwischenkreis)', 'P(Netz)', 'E(Netz)'}); grid on;
    subplot(2,2,3); hold on
    plot(Traj_0.t, QD);
    ylabel('Gelenk-Geschw.'); grid on;
    subplot(2,2,4); hold on
    plot(Traj_0.t, TAU);
    ylabel('Gelenk-Moment.'); grid on;
    linkxaxes
  end
elseif strcmp(Set.optimization.objective, 'mass')
  % Gesamtmasse berechnen
  m_sum = sum(R.DynPar.mges(2:end));
  f_mass_norm = 2/pi*atan((m_sum)/100); % Normierung auf 0 bis 1; 620 ist 0.9. TODO: Skalierung ändern
  fval = 1e3*f_mass_norm; % Normiert auf 0 bis 1e3
  debug_info = {debug_info{:}; sprintf('masses: total %1.2fkg, Segments [%s] kg', ...
    m_sum, disp_array(R.DynPar.mges(2:end)', '%1.2f'))};
else
  error('Zielfunktion nicht definiert');
end
debug_plot_robot(R, Q(1,:)', Traj_W, Set, Structure, p, fval, debug_info);
end


function debug_plot_robot(R, q, Traj_W, Set, Structure, p, fval, debug_info)
% Zeichne den Roboter für den aktuellen Parametersatz.
if Set.general.plot_robot_in_fitness < 0 && fval > abs(Set.general.plot_robot_in_fitness) || ... % Gütefunktion ist schlechter als Schwellwert: Zeichne
   Set.general.plot_robot_in_fitness > 0 && fval < abs(Set.general.plot_robot_in_fitness) % Gütefunktion ist besser als Schwellwert: Zeichne
  % Zeichnen fortsetzen
else 
  return
end
tt = '';
for i = 1:length(debug_info), tt = [tt, newline(), debug_info{i}]; end %#ok<AGROW>

figure(200);clf;hold all;
view(3);
axis auto
hold on;grid on;
xlabel('x in m');ylabel('y in m');zlabel('z in m');
plot3(Traj_W.X(:,1), Traj_W.X(:,2),Traj_W.X(:,3), 'k-');
set(200,'units','normalized','outerposition',[0 0 1 1])
s_plot = struct( 'ks_legs', [], 'straight', 0);
R.plot( q, s_plot);
title(sprintf('fval=%1.2e; p=[%s]; %s', fval,disp_array(p','%1.3f'), tt));
xlim([-1,1]*Structure.Lref*1.5+mean(minmax2(Traj_W.XE(:,1)')'));
ylim([-1,1]*Structure.Lref*1.5+mean(minmax2(Traj_W.XE(:,2)')'));
zlim([-1,1]*Structure.Lref*1+mean(minmax2(Traj_W.XE(:,3)')'));
end