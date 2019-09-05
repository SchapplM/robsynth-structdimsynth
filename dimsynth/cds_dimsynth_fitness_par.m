% Gütefunktion für Parallele Roboter

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2019-08
% (C) Institut für Mechatronische Systeme, Universität Hannover

function fval = cds_dimsynth_fitness_par(R, Set, Traj_W, Structure, p)
% Debug: 
% save(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_fitness_par1.mat'));
% error('Halte hier');
% load(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_fitness_par1.mat'));

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

%% Geometrie auf Plausibilität prüfen (1)
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
  fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3e. Beinkette zu kurz für Plattform. Es fehlen max. %1.2fm.\n', toc(t1), fval, l_legtooshort);
  debug_plot_robot(R, zeros(R.NJ,1), Traj_0, Traj_W, Set, Structure, p, fval, debug_info);
  return
end

%% Geometrie auf Plausibilität prüfen (2)
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
  fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3e. Beinkette zu kurz für Bahnpunkte. Es fehlen max. %1.2fm.\n', toc(t1), fval, -min(dist_exc_tot(:)));
  debug_plot_robot(R, zeros(R.NJ,1), Traj_0, Traj_W, Set, Structure, p, fval, debug_info);
  return
end
% basediam = 0;
% if any(Structure.vartypes == 6)
%   basediam = 2*p(Structure.vartypes == 6)*p(1);
% else
%   % TODO: Das ist noch eine zu große Annäherung an den Kreis-Wert. Besser
%   % Parameter abspeichern und auslesen
%   error('Noch nicht definiert');
%   basediam = norm(diff(minmax2(R.r_0_A_all)')');
% end
% maxleglength = sum(abs(p_lengthpar));
% lengthdeficit = basediam-maxleglength/2;
% TODO: Für jede Beinkette prüfen, ob alle Traj.-Punkte im theoretisch
% möglichen maximalen Arbeitsraum sind.

%% Inverse Kinematik für Eckpunkte der Trajektorie berechnen
qlim_PKM = cat(1,R.Leg(:).qlim);
q0 = qlim_PKM(:,1) + rand(R.NJ,1).*(qlim_PKM(:,2)-qlim_PKM(:,1));

nPhi = R.I2constr_red(end);
Phi_E = NaN(nPhi, size(Traj_0.XE,1));
QE = NaN(size(Traj_0.XE,1), R.NJ);
for i = size(Traj_0.XE,1):-1:1
  s = struct('Phit_tol', 1e-4, 'Phir_tol', 1e-3, 'retry_limit', 5, ...
    'normalize', false);
  [q, Phi] = R.invkin_ser(Traj_0.XE(i,:)', q0, s);
  Phi_E(:,i) = Phi;
  if ~any(isnan(q))
    q0 = q; % Annahme: Startwert für nächsten Eckwert nahe aktuellem Eckwert
  end
  QE(i,:) = q;
end

QE(isnan(QE)) = 0;
Phi_E(isnan(Phi_E)) = 1e6;
if any(abs(Phi_E(:)) > 1e-2) % Die Toleranz beim IK-Verfahren ist etwas größer
  % Nehme die mittlere IK-Abweichung aller Eckpunkte (Translation/Rotation
  % gemischt). Typische Werte von 1e-2 bis 10
  f_PhiE = max(sum(abs(Phi_E(:)))) / size(Traj_0.XE,1);
  f_phiE_norm = 2/pi*atan((f_PhiE)/10); % Normierung auf 0 bis 1
  fval = 1e6*(1+9*f_phiE_norm); % Normierung auf 1e6 bis 1e7
  % Keine Konvergenz der IK. Weitere Rechnungen machen keinen Sinn.
  fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3e. Keine IK-Konvergenz in Eckwerten\n', toc(t1), fval);
  debug_plot_robot(R, QE(1,:)', Traj_0, Traj_W, Set, Structure, p, fval, debug_info);
  return
end

% Bestimme die Spannweite der Gelenkkoordinaten (getrennt Dreh/Schub)
q_range_E = NaN(1, R.NJ);
q_range_E(R.MDH.sigma==1) = diff(minmax2(QE(:,R.MDH.sigma==1)')');
q_range_E(R.MDH.sigma==0) = angle_range(QE(:,R.MDH.sigma==0));
% Bestimme ob die maximale Spannweite der Koordinaten überschritten wurde
qlimviol_E = (qlim_PKM(:,2)-qlim_PKM(:,1))' - q_range_E;
I_qlimviol_E = (qlimviol_E < 0);
if any(I_qlimviol_E)
  save(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_fitness_par_qviolE.mat'));
  % Bestimme die größte relative Verletzung der Winkelgrenzen
  fval_qlimv_E = -min(qlimviol_E(I_qlimviol_E)./(qlim_PKM(I_qlimviol_E,2)-qlim_PKM(I_qlimviol_E,1))');
  fval_qlimv_E_norm = 2/pi*atan((fval_qlimv_E)/0.3); % Normierung auf 0 bis 1; 2 ist 0.9
  fval = 1e5*(1+9*fval_qlimv_E_norm); % Normierung auf 1e5 bis 1e6
  % Überschreitung der Gelenkgrenzen (bzw. -bereiche). Weitere Rechnungen machen keinen Sinn.
  fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3e. Gelenkgrenzverletzung in AR-Eckwerten.\n', toc(t1), fval);
  debug_plot_robot(R, QE(1,:)', Traj_0, Traj_W, Set, Structure, p, fval, debug_info);
  return
end

save(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_fitness_par2.mat'));
% Debug:
% load(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_fitness_par2.mat'));
%% Inverse Kinematik der Trajektorie berechnen
% s = struct('debug', true, 'retry_limit', 1);
s = struct('normalize', false, 'retry_limit', 1);
[Q, QD, ~, PHI] = R.invkin_traj(Traj_0.X, Traj_0.XD, Traj_0.XDD, Traj_0.t, q, s);
I_ZBviol = any(abs(PHI) > 1e-3,2) | any(isnan(Q),2);
if any(I_ZBviol)
  % Bestimme die erste Verletzung der ZB (je später, desto besser)
  IdxFirst = find(I_ZBviol, 1 );
  % Umrechnung in Prozent der Traj.
  Failratio = 1-IdxFirst/length(Traj_0.t); % Wert zwischen 0 und 1
  fval = 1e4*(1+9*Failratio); % Wert zwischen 1e4 und 1e5
  % Keine Konvergenz der IK. Weitere Rechnungen machen keinen Sinn.
  fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3e. Keine IK-Konvergenz in Traj.\n', toc(t1), fval);
  debug_plot_robot(R, Q(1,:)', Traj_0, Traj_W, Set, Structure, p, fval, debug_info);
  return
end
% Prüfe, ob die Gelenkwinkelgrenzen verletzt werden
q_range_T = NaN(1, R.NJ);
q_range_T(R.MDH.sigma==1) = diff(minmax2(Q(:,R.MDH.sigma==1)')');
q_range_T(R.MDH.sigma==0) = angle_range(Q(:,R.MDH.sigma==0));
qlimviol_T = (qlim_PKM(:,2)-qlim_PKM(:,1))' - q_range_T;
I_qlimviol_T = (qlimviol_T < 0);
if any(I_qlimviol_E)
  save(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_fitness_par_qviolT.mat'));
  figure(1000);
  subplot(2,1,1);
  plot(Traj_0.t, Q-repmat(min(Q), length(Traj_0.t), 1));
  % Bestimme die größte relative Verletzung der Winkelgrenzen
  fval_qlimv_T = -min(qlimviol_T(I_qlimviol_T)./(qlim_PKM(I_qlimviol_T,2)-qlim_PKM(I_qlimviol_T,1))');
  fval_qlimv_T_norm = 2/pi*atan((fval_qlimv_T)/0.3); % Normierung auf 0 bis 1; 2 ist 0.9
  fval = 1e3*(1+9*fval_qlimv_T_norm); % Wert zwischen 1e3 und 1e4
  % Überschreitung der Gelenkgrenzen (bzw. -bereiche). Weitere Rechnungen machen keinen Sinn.
  fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3e. Gelenkgrenzverletzung in Traj.\n', toc(t1), fval);
  debug_plot_robot(R, Q(1,:)', Traj_0, Traj_W, Set, Structure, p, fval, debug_info);
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
  n_qa = sum(R.I_qa);
  % Berechne Konditionszahl für alle Punkte der Bahn
  for i = 1:length(Traj_0.t)
    Jinv_xred = R.jacobi_qa_x(Q(i,:)', Traj_0.X(i,:)');
    Jinv_3T3R = zeros(6, n_qa);
    Jinv_3T3R(R.I_EE,:) = Jinv_xred;
    Jinv_task = Jinv_3T3R(Set.structures.DoF,:);
    Cges(i,:) = cond(Jinv_task);
  end
  % Schlechtester Wert der Konditionszahl
  % Nehme Logarithmus, da Konditionszahl oft sehr groß ist.
  f_cond1 = max(Cges);
  f_cond = log(f_cond1); 
  f_cond_norm = 2/pi*atan((f_cond)/20); % Normierung auf 0 bis 1; 150 ist 0.9
  fval = 1e3*f_cond_norm; % Normiert auf 0 bis 1e3
  fprintf('Fitness-Evaluation in %1.1fs. fval=%1.3f. Konditionszahl %1.3e\n', toc(t1),fval,  f_cond1);

  if fval < Set.general.plot_details_in_fitness
    % Debug-Werte berechnen
    det_ges = NaN(length(Traj_0.t), 3);
    for i = 1:length(Traj_0.t)
      G_q  = R.constr1grad_q(Q(i,:)', Traj_0.X(i,:)');
      G_x = R.constr1grad_x(Q(i,:)', Traj_0.X(i,:)');
      G_d = G_q(:,R.I_qd);
      G_dx = [G_d, G_x];
      Jinv_num_voll = -G_q \ G_x;
      Jinv = Jinv_num_voll(R.I_qa,:);
      % Debug: Vergleich Jacobi
      if any(any(abs(Jinv - R.jacobi_qa_x(Q(i,:)', Traj_0.X(i,:)')) > 1e-6))
        save(fullfile(fileparts(which('struktsynth_bsp_path_init.m')), 'tmp', 'cds_dimsynth_fitness_par3.mat'));
        warning('Jacobi numerisch vs. symbolisch stimmt nicht');
      end
      det_ges(i,:) = [det(G_dx), det(G_q), det(Jinv)];
    end
    
    if Set.general.plot_robot_in_fitness < 0 && fval > abs(Set.general.plot_robot_in_fitness) || ... % Gütefunktion ist schlechter als Schwellwert: Zeichne
       Set.general.plot_robot_in_fitness > 0 && fval < abs(Set.general.plot_robot_in_fitness) % Gütefunktion ist besser als Schwellwert: Zeichne
      change_current_figure(201); clf;
      if ~strcmp(get(201, 'windowstyle'), 'docked')
        set(201,'units','normalized','outerposition',[0 0 1 1]);
      end
      sgtitle('Auswertung Jacobi-Matrizen')
      subplot(2,3,1);
      plot(Traj_0.t, Cges); hold on;
      ylabel('Konditionszahl'); grid on;
      subplot(2,3,2);
      plot(Traj_0.t, Traj_0.XD); hold on;
      ylabel('EE-Geschw.'); grid on;
      legend({'r_x', 'r_y', 'r_z', '\phi_1', '\phi_2', '\phi_3'});
      subplot(2,3,3);
      plot(Traj_0.t, QD(:,R.I_qa)); hold on;
      ylabel('Gelenk-Geschw. (aktiv)'); grid on;
      subplot(2,3,4);
      plot(Traj_0.t, QD(:,~R.I_qa)); hold on;
      ylabel('Gelenk-Geschw. (passiv)'); grid on;
      subplot(2,3,5);
      plot(Traj_0.t, det_ges); hold on;
      ylabel('Determinanten'); grid on;
      legend({'A (dh/dx; DirKin)', 'B (dh/dq; InvKin)', 'Jinv'});
      subplot(2,3,6);
      plot(Traj_0.t, log(abs(det_ges))); hold on;
      ylabel('Log |Determinanten|'); grid on;
      legend({'A (dh/dx; DirKin)', 'B (dh/dq; InvKin)', 'Jinv'});
      linkxaxes
      [currgen,currimg,resdir] = get_new_figure_filenumber(Set, Structure,'ParRobJacobian');
      for fileext=Set.general.save_robot_details_plot_fitness_file_extensions
        if strcmp(fileext{1}, 'fig')
          saveas(201, fullfile(resdir, sprintf('PSO_Gen%02d_FitEval%03d_ParRobJacobian.fig', currgen, currimg)));
        else
          export_fig(201, fullfile(resdir, sprintf('PSO_Gen%02d_FitEval%03d_ParRobJacobian.%s', currgen, currimg, fileext{1})));
        end
      end
    end
  end
elseif strcmp(Set.optimization.objective, 'energy')
  % Trajektorie in Plattform-KS umrechnen
  XP = Traj_0.X;
  XPD = Traj_0.XD;
  XPDD = Traj_0.XDD;
  for i = 1:length(Traj_0.t)
    [XP(i,:),XPD(i,:),XPDD(i,:)] = xE2xP(R, Traj_0.X(i,:)', Traj_0.XD(i,:)', Traj_0.XDD(i,:)');
  end
  % Antriebskräfte berechnen
  Fx_red_traj = invdyn_platform_traj(R, Q, XP, XPD, XPDD);
  TAU = NaN(length(Traj_0.t), sum(R.I_qa));
  for i = 1:length(Traj_0.t)
    Jinv_xred_i = R.jacobi_qa_x(Q(i,:)', Traj_0.X(i,:)');
    tauA_i = (Jinv_xred_i') \ Fx_red_traj(i,:)';
    TAU(i,:) = tauA_i;
  end
  % Mechanische Leistung berechnen
  % Aktuelle mechanische Leistung in allen Gelenken
  P_ges = NaN(size(Q,1), sum(R.I_qa));
  II_qa = find(R.I_qa);
  for j = 1:sum(sum(R.I_qa))
    P_ges(:,j) = TAU(:,j) .* QD(:,II_qa(j));
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
    if ~strcmp(get(202, 'windowstyle'), 'docked')
      set(202,'units','normalized','outerposition',[0 0 1 1]);
    end
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
    hdl1=plot(Traj_0.t, QD(:,R.I_qa));
    hdl2=plot(Traj_0.t, QD(:,~R.I_qa), '--');
    ylabel('Gelenk-Geschw.'); grid on; legend([hdl1(1), hdl2(1)], {'Antriebe', 'Passive'});
    subplot(2,2,4); hold on
    plot(Traj_0.t, TAU);
    ylabel('Gelenk-Moment.'); grid on;
    linkxaxes
    [currgen,currimg,resdir] = get_new_figure_filenumber(Set, Structure,'ParRobEnergy');
    for fileext=Set.general.save_robot_details_plot_fitness_file_extensions
      if strcmp(fileext{1}, 'fig')
        saveas(202, fullfile(resdir, sprintf('PSO_Gen%02d_FitEval%03d_ParRobEnergy.fig', currgen, currimg)));
      else
        export_fig(202, fullfile(resdir, sprintf('PSO_Gen%02d_FitEval%03d_ParRobEnergy.%s', currgen, currimg, fileext{1})));
      end
    end
  end
elseif strcmp(Set.optimization.objective, 'mass')
  % Gesamtmasse berechnen
  m_sum = sum(R.DynPar.mges(1:end-1))*R.NLEG + R.DynPar.mges(end);
  f_mass_norm = 2/pi*atan((m_sum)/100); % Normierung auf 0 bis 1; 620 ist 0.9. TODO: Skalierung ändern
  fval = 1e3*f_mass_norm; % Normiert auf 0 bis 1e3
  debug_info = {debug_info{:}; sprintf('masses: total %1.2fkg, 1Leg %1.2fkg, Pf %1.2fkg', ...
    m_sum, sum(sum(R.DynPar.mges(1:end-1))), R.DynPar.mges(end))};
else
  error('Zielfunktion nicht definiert');
end

debug_plot_robot(R, Q(1,:)', Traj_0, Traj_W, Set, Structure, p, fval, debug_info);
end


function debug_plot_robot(R, q, Traj_0, Traj_W, Set, Structure, p, fval, debug_info)
% Zeichne den Roboter für den aktuellen Parametersatz.
if Set.general.plot_robot_in_fitness < 0 && fval > abs(Set.general.plot_robot_in_fitness) || ... % Gütefunktion ist schlechter als Schwellwert: Zeichne
   Set.general.plot_robot_in_fitness > 0 && fval < abs(Set.general.plot_robot_in_fitness) % Gütefunktion ist besser als Schwellwert: Zeichne
  % Zeichnen fortsetzen
else 
  return
end
tt = '';
for i = 1:length(debug_info), tt = [tt, newline(), debug_info{i}]; end %#ok<AGROW>

change_current_figure(200); clf; hold all;
if ~strcmp(get(200, 'windowstyle'), 'docked')
  set(200,'units','normalized','outerposition',[0 0 1 1]);
end
view(3);
axis auto
hold on;grid on;
xlabel('x in m');ylabel('y in m');zlabel('z in m');
plot3(Traj_W.X(:,1), Traj_W.X(:,2),Traj_W.X(:,3), 'k-');
s_plot = struct( 'ks_legs', [], 'straight', 0, 'mode', 4);
R.plot( q, Traj_0.X(1,:)', s_plot);
title(sprintf('fval=%1.2e; p=[%s]; %s', fval,disp_array(p','%1.3f'), tt));
xlim([-1,1]*Structure.Lref*3+mean(minmax2(Traj_W.XE(:,1)')'));
ylim([-1,1]*Structure.Lref*3+mean(minmax2(Traj_W.XE(:,2)')'));
zlim([-1,1]*Structure.Lref*1+mean(minmax2(Traj_W.XE(:,3)')'));
if ~isempty(Set.general.save_robot_details_plot_fitness_file_extensions)
  [currgen,currimg,resdir] = get_new_figure_filenumber(Set, Structure,'Details');
  for fileext=Set.general.save_robot_details_plot_fitness_file_extensions
    if strcmp(fileext{1}, 'fig')
      saveas(200, fullfile(resdir, sprintf('PSO_Gen%02d_FitEval%03d_Details.fig', currgen, currimg)));
    else
      export_fig(200, fullfile(resdir, sprintf('PSO_Gen%02d_FitEval%03d_Details.%s', currgen, currimg, fileext{1})));
    end
  end
end
end

function [currgen,currimg,resdir] = get_new_figure_filenumber(Set, Structure, suffix)
resdir = fullfile(Set.optimization.resdir, Set.optimization.optname, ...
  'tmp', sprintf('%d_%s', Structure.Number, Structure.Name));
matfiles = dir(fullfile(resdir, 'PSO_Gen*.mat'));
if isempty(matfiles)
  % Es liegen noch keine .mat-Dateien vor. Also wird aktuell die erste
  % Generation berechnet
  currgen = 0;
else
  [tokens_mat,~] = regexp(matfiles(end).name,'PSO_Gen(\d+)','tokens','match');
  currgen = str2double(tokens_mat{1}{1})+1; % Es fängt mit Null an
end
imgfiles = dir(fullfile(resdir, sprintf('PSO_Gen%02d_FitEval*_%s*',currgen,suffix)));
if isempty(imgfiles)
  % Es liegen noch keine Bild-Dateien vor.
  currimg = 0;
else
  [tokens_img,~] = regexp(imgfiles(end).name,sprintf('PSO_Gen%02d_FitEval(\\d+)_%s',currgen,suffix),'tokens','match');
  currimg = str2double(tokens_img{1}{1})+1;
end
end
