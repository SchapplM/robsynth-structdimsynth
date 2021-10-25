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
% data_dyn_reg
%   Struktur mit Dynamik-Eigenschaften des Roboters als Regressormatrizen.
%   Felder: Siehe cds_obj_dependencies_regmult
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
% physval_desopt
%   Physikalische Entsprechung des für das Abbruchkriterium maßgeblichen
%   Kennwertes. Beispielsweise relative Überlastung der Materialspannung
%   oder der Antriebe. Entspricht dem Kriterium des Wertebereichs aus fval.
% abort_fitnesscalc_retval
%   Schalter für Abbruch der Berechnung, wenn alle gesetzten Grenzen
%   erreicht werden.
% 
% Siehe auch: cds_fitness.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [fval, physval_desopt, abort_fitnesscalc_retval] = cds_dimsynth_desopt_fitness(R, Set, Traj_0, Q, QD, QDD, Jinv_ges, data_dyn_reg, Structure, p_desopt)
t1 = tic();
% Debug:
if Set.general.matfile_verbosity > 3
  repopath = fileparts(which('structgeomsynth_path_init.m'));
  save(fullfile(repopath, 'tmp', 'cds_dimsynth_desopt_fitness.mat'));
end
% load(fullfile(fileparts(which('structgeomsynth_path_init.m')), 'tmp', 'cds_dimsynth_desopt_fitness.mat'));

fval = 0;
physval_desopt = 0;
fval_debugtext = '';
% Abbruch prüfen
persistent abort_fitnesscalc
abort_fitnesscalc_retval = false;
if isempty(abort_fitnesscalc)
  abort_fitnesscalc = false;
elseif abort_fitnesscalc
  fval = Inf;
  cds_desopt_save_particle_details(toc(t1), fval, p_desopt);
  return;
end
vartypes = Structure.desopt_ptypes(Structure.desopt_ptypes~=1);
p_ls = p_desopt(vartypes==2);
p_jsoff = p_desopt(vartypes==3);
p_js = p_desopt(vartypes==4);

%% Plausibilität der Eingabe prüfen
if any(vartypes==2) && p_ls(1) > p_ls(2)/2 % Wandstärke darf nicht größer als Radius sein
  f_wall_vs_rad = (p_ls(1) - p_ls(2)/2)/p_ls(2); % Grad der Überschreitung
  f_wall_vs_rad_norm = 2/pi*atan(f_wall_vs_rad); % 50% -> 0.3
  fval = 1e8*(1+9*f_wall_vs_rad_norm); % Normierung auf 1e8...1e9
  constrvioltext = sprintf('Radius (%1.1fmm) ist kleiner als Wandstärke (%1.1fmm)', ...
    1e3*p_ls(2)/2, 1e3*p_ls(1));
end

%% Dynamikparameter aktualisieren
if any(vartypes==2) && fval == 0
  cds_dimsynth_design(R, Q, Set, Structure, p_ls);
end
%% Gelenksteifigkeiten aktualisieren
if any(vartypes==3)
  for i = 1:R.NLEG
    I_actrevolute_opt = R.Leg(i).MDH.mu ~= 1 & R.Leg(i).DesPar.joint_type==0 & ...
      Set.optimization.joint_stiffness_active_revolute ~= 0;
    I_passrevolute_opt = R.Leg(i).MDH.mu == 1 & R.Leg(i).DesPar.joint_type==0 & ...
      Set.optimization.joint_stiffness_passive_revolute ~= 0;
    I_passuniversal_opt = R.Leg(i).MDH.mu == 1 & R.Leg(i).DesPar.joint_type==2 & ...
      Set.optimization.joint_stiffness_passive_universal ~= 0;
    I_update = I_actrevolute_opt | I_passrevolute_opt | I_passuniversal_opt;
    R.Leg(i).DesPar.joint_stiffness_qref(I_update) = p_jsoff;
  end
end
if any(vartypes==4)
  for i = 1:R.NLEG
    I_actrevolute_opt = R.Leg(i).MDH.mu ~= 1 & R.Leg(i).DesPar.joint_type==0 & ...
      isnan(Set.optimization.joint_stiffness_active_revolute);
    I_passrevolute_opt = R.Leg(i).MDH.mu == 1 & R.Leg(i).DesPar.joint_type==0 & ...
      isnan(Set.optimization.joint_stiffness_passive_revolute);
    I_passuniversal_opt = R.Leg(i).MDH.mu == 1 & R.Leg(i).DesPar.joint_type==2 & ...
      isnan(Set.optimization.joint_stiffness_passive_universal);
    I_update = I_actrevolute_opt | I_passrevolute_opt | I_passuniversal_opt;
    R.Leg(i).DesPar.joint_stiffness(I_update) = p_js;
  end
end
%% Dynamik neu berechnen
if fval == 0 && (Structure.calc_dyn_reg || Structure.calc_spring_reg)
  % Abhängigkeiten neu berechnen (Dynamik)
  data_dyn = cds_obj_dependencies_regmult(R, data_dyn_reg, Q);
  if Set.general.debug_calc
    % Zu Testzwecken die Dynamik neu ohne Regressorform berechnen und mit
    % Regressor-Berechnung vergleichen
    Structure_tmp = Structure;
    Structure_tmp.calc_dyn_act = true;
    Structure_tmp.calc_dyn_reg = false;
    if Set.optimization.joint_stiffness_active_revolute ~= 0 || ...
       Set.optimization.joint_stiffness_passive_revolute ~= 0 || ...
       Set.optimization.joint_stiffness_passive_universal ~= 0
      Structure_tmp.calc_spring_act = true;
      Structure_tmp.calc_spring_reg = false;
    end
    data_dyn2 = cds_obj_dependencies(R, Traj_0, Set, Structure_tmp, Q, QD, QDD, Jinv_ges);
    test_TAU_abs = data_dyn2.TAU - data_dyn.TAU;
    test_TAU_rel = test_TAU_abs ./ data_dyn2.TAU;
    I_err = abs(test_TAU_abs)>1e-3 & abs(test_TAU_rel)>1e-2;
    if any(I_err(:))
      save(fullfile(Set.optimization.resdir, Set.optimization.optname, 'tmp', ...
        sprintf('%d_%s', Structure.Number, Structure.Name), 'desopt_TAU_reprowarning.mat'));
      error('Antriebskräfte aus Regressorform stimmt nicht. Fehler: abs %1.2e, rel %1.2e', ...
        max(abs(test_TAU_abs(I_err))), max(abs(test_TAU_rel(I_err))));
    end
    if Structure.calc_cut
      test_W_abs = data_dyn2.Wges - data_dyn.Wges;
      test_W_rel = test_W_abs ./ data_dyn2.Wges;
      I_err = abs(test_W_abs)>1e-3 & abs(test_W_rel)>1e-2;
      if any(I_err(:))
        save(fullfile(Set.optimization.resdir, Set.optimization.optname, 'tmp', ...
          sprintf('%d_%s', Structure.Number, Structure.Name), 'desopt_W_reprowarning.mat'));
        error('Schnittkräfte aus Regressorform stimmt nicht. Fehler: abs %1.2e, rel %1.e2', ...
          max(abs(test_W_abs(I_err))), max(abs(test_W_rel(I_err))));
      end
    end
    if any(Set.optimization.joint_stiffness_active_revolute~=0) || ...
        any(Set.optimization.joint_stiffness_passive_revolute~=0) || ...
        any(Set.optimization.joint_stiffness_passive_universal~=0)
      if isfield(data_dyn2, 'TAU_spring') && isfield(data_dyn, 'TAU_spring')
        test_TAU_spring_abs = data_dyn2.TAU_spring - data_dyn.TAU_spring;
        test_TAU_spring_rel = test_TAU_spring_abs ./ data_dyn2.TAU_spring;
        I_err = abs(test_TAU_spring_abs)>1e-3 & abs(test_TAU_spring_rel)>1e-2;
        if any(I_err(:))
          save(fullfile(Set.optimization.resdir, Set.optimization.optname, 'tmp', ...
            sprintf('%d_%s', Structure.Number, Structure.Name), 'desopt_TAUspring_reprowarning.mat'));
          error('Antriebskräfte für Gelenkfeder aus Regressorform stimmt nicht. Fehler: abs %1.2e, rel %1.2e', ...
            max(abs(test_TAU_spring_abs(I_err))), max(abs(test_TAU_spring_rel(I_err))));
        end
      end
      if isfield(data_dyn2, 'Wges_spring') && isfield(data_dyn, 'Wges_spring')
        test_W_spring_abs = data_dyn2.Wges_spring - data_dyn.Wges_spring;
        test_W_spring_rel = test_W_spring_abs ./ data_dyn2.Wges_spring;
        I_err = abs(test_W_spring_abs)>1e-3 & abs(test_W_spring_rel)>1e-2;
        if any(I_err(:))
          save(fullfile(Set.optimization.resdir, Set.optimization.optname, 'tmp', ...
            sprintf('%d_%s', Structure.Number, Structure.Name), 'desopt_Wspring_reprowarning.mat'));
          error('Schnittkräfte für Gelenkfeder aus Regressorform stimmt nicht. Fehler: abs %1.2e, rel %1.2e', ...
            max(abs(test_W_spring_abs(I_err))), max(abs(test_W_spring_rel(I_err))));
        end
      end
    end
  end
end

%% Nebenbedingungen der Zielfunktionswerte berechnen
% Festigkeit der Segmente (mit höherem Strafterm)
if fval == 0 && Set.optimization.constraint_obj(6)
  [fval_ms, constrvioltext_ms, physval_materialstress] = cds_constr_yieldstrength(R, Set, data_dyn, Jinv_ges, Q, Traj_0);
  fval = fval_ms;
  constrvioltext = constrvioltext_ms;
  physval_desopt = physval_materialstress;
end
if fval == 0 && Set.optimization.constraint_obj(1) % NB für Masse gesetzt
  [fval_mass, fval_debugtext_mass, ~, fphys_m] = cds_obj_mass(R);
  physval_desopt = fphys_m / Set.optimization.constraint_obj(1);
  viol_rel_m = physval_desopt - 1;
  if viol_rel_m > 0 % Relative Überschreitung der Grenze für die Masse
    f_massvio_norm = 2/pi*atan((viol_rel_m)); % 1->0.5; 10->0.94
    fval = 1e3*(1+1*f_massvio_norm); % 1e3 ... 2e3
    constrvioltext = sprintf('Masse ist zu groß (%1.1f > %1.1f)', ...
      fphys_m, Set.optimization.constraint_obj(1));
  end
end
if fval == 0  && Set.optimization.constraint_obj(3) % NB für Antriebskraft gesetzt
  [fval_actforce, fval_debugtext_actforce, ~, fphys_actforce] = cds_obj_actforce(data_dyn.TAU);
  physval_desopt = fphys_actforce / Set.optimization.constraint_obj(3);
  viol_rel_actforce = physval_desopt - 1;
  if viol_rel_actforce > 0 % Relative Überschreitung der Grenze für die Antriebskraft
    f_actforcevio_norm = 2/pi*atan((viol_rel_actforce)); % 1->0.5; 10->0.94
    fval = 1e3*(1+1*f_actforcevio_norm); % 2e3 ... 3e3
    constrvioltext = sprintf('Antriebskraft ist zu groß (%1.1f > %1.1f)', ...
      fphys_actforce, Set.optimization.constraint_obj(3));
  end
end
if fval == 0  && Set.optimization.constraint_obj(2) % NB für Energie gesetzt
  error('Grenzen für Zielfunktionen Energie noch nicht implementiert');
end
if fval == 0  && Set.optimization.constraint_obj(5) % NB für Steifigkeit gesetzt
  [fval_st, fval_debugtext_st, ~, fphys_st] = cds_obj_stiffness(R, Set, Q);
  physval_desopt = fphys_st / Set.optimization.constraint_obj(5);
  viol_rel_st = physval_desopt - 1;
  if viol_rel_st > 0 % Relative Überschreitung der Nachgiebigkeit
    f_stvio_norm = 2/pi*atan((viol_rel_st)); % 1->0.5; 10->0.94
    fval = 1e3*(2+1*f_stvio_norm); % 3e3 ... 4e3
    constrvioltext = sprintf('Nachgiebigkeit ist zu groß (%1.1f > %1.1f)', ...
      fphys_st, Set.optimization.constraint_obj(5));
  end
end
if fval > 1000 % Nebenbedingungen verletzt.
  [~, i_gen, i_ind] = cds_desopt_save_particle_details(toc(t1), fval, p_desopt);
  cds_log(4,sprintf(['[desopt/fitness] G=%d;I=%d. DesOpt-Fitness-Evaluation ', ...
    'in %1.2fs. Parameter: [%s]. fval=%1.3e. %s'], i_gen, i_ind, toc(t1), ...
    disp_array(p_desopt', '%1.3f'), fval, constrvioltext));
end

%% Fitness-Wert berechnen
% Eintrag in Fitness-Wert für die äußere Optimierungsschleife in der
% Maßsynthese. Nehme in dieser Optimierung nur ein Zielkriterium, auch wenn
% die Maßsynthese mehrkriteriell ist. Fange mit den einfachen Kriterien an.
fval_main = NaN(length(Set.optimization.objective),1);
physval_main = NaN(length(Set.optimization.objective),1);
abort_logtext = '';
if fval > 1000
  % Nichts machen. Materialspannung wurde schon verletzt. Gehe nur bis
  % unten durch, um eventuell Debug-Plots zu zeichnen
elseif any(strcmp(Set.optimization.objective, 'mass'))
  if Set.optimization.constraint_obj(1) % Vermeide doppelten Aufruf der Funktion
    fval = fval_mass; % Nehme Wert von der NB-Berechnung oben
    fval_debugtext = fval_debugtext_mass;
  else
    [fval, fval_debugtext, ~, fphys_m] = cds_obj_mass(R);
  end
  fval_main(strcmp(Set.optimization.objective, 'mass')) = fval;
  physval_main(strcmp(Set.optimization.objective, 'mass')) = fphys_m;
elseif any(strcmp(Set.optimization.objective, 'energy'))
  if Set.optimization.constraint_obj(2) % Vermeide doppelten Aufruf der Funktion
    fval = fval_energy; % Nehme Wert von der NB-Berechnung oben
    fval_debugtext = fval_debugtext_energy;
  else
    [fval,fval_debugtext,~,physval_en] = cds_obj_energy(R, Set, Structure, Traj_0, data_dyn.TAU, QD);
  end
  fval_main(strcmp(Set.optimization.objective, 'energy')) = fval;
  physval_main(strcmp(Set.optimization.objective, 'energy')) = physval_en;
elseif any(strcmp(Set.optimization.objective, 'actforce'))
  if Set.optimization.constraint_obj(3) % Vermeide doppelten Aufruf der Funktion
    fval = fval_actforce; % Nehme Wert von der NB-Berechnung oben
    fval_debugtext = fval_debugtext_actforce;
  else
    [fval, fval_debugtext, ~, fphys_actforce] = cds_obj_actforce(data_dyn.TAU);
  end
  fval_main(strcmp(Set.optimization.objective, 'actforce')) = fval;
  physval_main(strcmp(Set.optimization.objective, 'actforce')) = fphys_actforce;
elseif any(strcmp(Set.optimization.objective, 'stiffness'))
  if Set.optimization.constraint_obj(5) % Vermeide doppelten Aufruf der Funktion
    fval = fval_st; % Nehme Wert von der NB-Berechnung oben
    fval_debugtext = fval_debugtext_st;
  else
    [fval,fval_debugtext,~,fphys_st] = cds_obj_stiffness(R, Set, Q);
  end
  fval_main(strcmp(Set.optimization.objective, 'stiffness')) = fval;
  physval_main(strcmp(Set.optimization.objective, 'stiffness')) = fphys_st;
elseif any(strcmp(Set.optimization.objective, 'materialstress'))
  if Set.optimization.constraint_obj(6) % Vermeide doppelten Aufruf der Funktion
    fval = fval_ms; % Nehme Wert von der NB-Berechnung oben
    fval_debugtext = constrvioltext_ms;
  else
    [fval,fval_debugtext,~,physval_materialstress] = cds_obj_materialstress(R, Set, data_dyn, Jinv_ges, Q, Traj_0);
  end
  fval_main(strcmp(Set.optimization.objective, 'materialstress')) = fval;
  physval_main(strcmp(Set.optimization.objective, 'materialstress')) = physval_materialstress;
else
  % Es wurde eine Dimensionierung gefunden, die alle Nebenbedingungen ein-
  % hält. Keine Zielfunktion definiert, die jetzt noch profitieren würde.
  abort_fitnesscalc = true;
  abort_fitnesscalc_retval = true;
  abort_logtext = ' Nebenbedingung erfüllt. Keine Zielfunktion für Entwurfsoptimierung.';
end
% Prüfe, ob in Entwurfsoptimierung berechnete Zielfunktionen ihre Grenze
% erreicht haben. Kinematik-bezogene Zielfunktionen werden hier nicht
% aktualisiert und bleiben NaN, werden also dabei nicht betrachtet.
if fval <= 1000 && (all(fval_main(~isnan(fval_main)) <= Set.optimization.obj_limit(~isnan(fval_main)) ) || ...
   all(physval_main(~isnan(fval_main)) <= Set.optimization.obj_limit_physval(~isnan(fval_main))))
  % Die Fitness-Funktion ist besser als die Grenze. Optimierung kann
  % hiernach beendet werden.
  abort_fitnesscalc = true;
  abort_fitnesscalc_retval = true;
  abort_logtext = ' Abbruchgrenze für Zielfunktion erreicht.';
end
if fval <= 1000
  [~, i_gen, i_ind] = cds_desopt_save_particle_details(toc(t1), fval, p_desopt);
  cds_log(4,sprintf(['[desopt/fitness] G=%d;I=%d. DesOpt-Fitness-Evaluation ', ...
    'in %1.2fs. Parameter: [%s]. fval=%1.3e. Erfolgreich. %s %s'], i_gen, ...
    i_ind, toc(t1), disp_array(p_desopt', '%1.3f'), fval, fval_debugtext, abort_logtext));
end
if Set.general.plot_details_in_desopt < 0 && fval >= abs(Set.general.plot_details_in_desopt) || ... % Gütefunktion ist schlechter als Schwellwert: Zeichne
   Set.general.plot_details_in_desopt > 0 && fval <= abs(Set.general.plot_details_in_desopt) % Gütefunktion ist besser als Schwellwert: Zeichne
  % Plotte die Antriebskraft, wenn die Feder-Ruhelagen optimiert werden
  if any(vartypes==3) || any(vartypes==4)
    change_current_figure(986);clf;
    set(986, 'Name', 'DesOpt_ActForce', 'NumberTitle', 'off');
    ntau = size(data_dyn.TAU_spring, 2);
    units = reshape([R.Leg(:).tauunit_sci],R.NJ,1);
    plotunits = units(R.I_qa);
    for i = 1:ntau
      subplot(ceil(sqrt(ntau)), ceil(ntau/ceil(sqrt(ntau))), i);
      hold on; grid on;
      plot(Traj_0.t, data_dyn.TAU_ID(:,i));
      plot(Traj_0.t, data_dyn.TAU_spring(:,i));
      plot(Traj_0.t, data_dyn.TAU(:,i));
      ylabel(sprintf('\\tau_%d in %s', i, plotunits{i}));
    end
    legend({'ID', 'spring', 'total'});
    sgtitle(sprintf('Antriebskraft in Entwurfsoptimierung. fval=%1.2e', fval));
    drawnow();
    % Berechne die Kraft in Endeffektor-Koordinaten
    Fx_TAU = NaN(length(Traj_0.t), 6);
    Fx_TAU_spring = Fx_TAU; Fx_TAU_ID = Fx_TAU;
    for i = 1:length(Traj_0.t)
      Jinv_all = reshape(Jinv_ges(i,:), R.NJ, sum(R.I_EE));
      Jinv_qa = Jinv_all(R.I_qa,:);
      Fx_TAU(i,:) = (Jinv_qa')*data_dyn.TAU(i,:)';
      Fx_TAU_ID(i,:) = (Jinv_qa')*data_dyn.TAU_ID(i,:)';
      Fx_TAU_spring(i,:) = (Jinv_qa')*data_dyn.TAU_spring(i,:)';
    end
    change_current_figure(988);clf;
    set(988, 'Name', 'DesOpt_PlfForce', 'NumberTitle', 'off');
    tauplf_names = {'fx', 'fy', 'fz', 'mx', 'my', 'mz'};
    for i = 1:6
      subplot(2,3,i);hold on; grid on;
      plot(Traj_0.t, Fx_TAU_ID(:,i));
      plot(Traj_0.t, Fx_TAU_spring(:,i));
      plot(Traj_0.t, Fx_TAU(:,i));
      if i < 4, plotunit = 'N'; else, plotunit = 'Nm'; end
      ylabel(sprintf('%s in %s', tauplf_names{i}, plotunit));
    end
    legend({'ID', 'spring', 'total'});
    sgtitle(sprintf('Plattformkraft in Entwurfsoptimierung. fval=%1.2e', fval));
    drawnow();
  end
  if any(vartypes==3)
    change_current_figure(987);clf;
    set(987, 'Name', 'JointSpring_Angles', 'NumberTitle', 'off');
    for i = 1:R.NJ
      legnum = find(i>=R.I1J_LEG, 1, 'last');
      legjointnum = i-(R.I1J_LEG(legnum)-1);
      switch R.Leg(legnum).DesPar.joint_type(legjointnum)
        case 0, type = 'R';
        case 1, type = 'P';
        case 2, type = 'U';
        case 3, type = 'S';
      end
      subplot(ceil(sqrt(R.NJ)), ceil(R.NJ/ceil(sqrt(R.NJ))), i);
      hold on; grid on;
      hdl1=plot(Traj_0.t, Q(:,i));
      if R.Leg(legnum).DesPar.joint_stiffness(legjointnum) ~= 0
        hdl2=plot(Traj_0.t([1,end]), R.Leg(legnum).DesPar.joint_stiffness_qref(legjointnum)*[1;1]);
      end
      title(sprintf('q %d (%s), L%d,J%d, k=%1.2f', i, type, legnum, legjointnum, ...
        R.Leg(legnum).DesPar.joint_stiffness(legjointnum)));
      if i == R.NJ, legend([hdl1;hdl2], {'q','qref'}); end
      if legjointnum == 1, ylabel(sprintf('Beinkette %d',legnum)); end
    end
    sgtitle(sprintf('Gelenkwinkel und Feder-Ruhelage. fval=%1.2e', fval));
    linkxaxes
  end
end
