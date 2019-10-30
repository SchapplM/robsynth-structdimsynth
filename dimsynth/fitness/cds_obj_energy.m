function [fval, fval_debugtext, debug_info] = cds_obj_energy(R, Set, Structure, Traj_0, TAU, QD)
debug_info = {};

if R.Type == 0
  error('TODO');
else
  % Mechanische Leistung berechnen
  % Aktuelle mechanische Leistung in allen Gelenken
  P_ges = NaN(size(QD,1), sum(R.I_qa));
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
  fval_debugtext = sprintf('Energieverbrauch %1.1f J', E_Netz_res);
  % TODO: Falls NaN auftritt, Zeitpunkt innerhalb Gesamt-Traj kodieren
  % im Bereich 100-1000. Energie-Wert erst bei 0 bis 100
  if isnan(fval), fval = 1000-eps(1000); end
  if fval < Set.general.plot_details_in_fitness
    E_Netz = cumtrapz(Traj_0.t, P_Netz);
    change_current_figure(202); clf;
    if ~strcmp(get(202, 'windowstyle'), 'docked')
      % set(202,'units','normalized','outerposition',[0 0 1 1]);
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
    drawnow();
    for fileext=Set.general.save_robot_details_plot_fitness_file_extensions
      [currgen,currimg,resdir] = cds_get_new_figure_filenumber(Set, Structure,'ParRobEnergy');
      if strcmp(fileext{1}, 'fig')
        saveas(202, fullfile(resdir, sprintf('PSO_Gen%02d_FitEval%03d_ParRobEnergy.fig', currgen, currimg)));
      else
        export_fig(202, fullfile(resdir, sprintf('PSO_Gen%02d_FitEval%03d_ParRobEnergy.%s', currgen, currimg, fileext{1})));
      end
    end
  end
end