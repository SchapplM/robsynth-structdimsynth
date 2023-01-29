% Teste die Begrenzung des effektiven Gestell- oder Plattform-Radius bei
% PKM mit paarweise angeordneten Gelenken
% 
% Ablauf: Synthese eines Hexapods mit paarweise angeordneten Gelenken.
% Danach Prüfung auf erwartete Eigenschaften.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

DoF = [1 1 1 1 1 1];
Set = cds_settings_defaults(struct('DoF', DoF));
Traj = cds_gen_traj(DoF, 1, Set.task);
Set.structures.whitelist = {'P6RRPRRR14V3G6P4A1'}; % Hexapod mit Gestell/Plattform als Gelenkpaare
Set.optimization.objective = {'condition'};
Set.optimization.obj_limit = 1e3; % bei Erfolg aufhören
Set.optimization.platform_morphology = true;
Set.general.save_animation_file_extensions = {};
Set.general.eval_figures = {'robvisu'};

% Teste mit fixen Grenzen
for i_case = 2%:4
  % Standard-Einstellungen setzen (werden in Schleife überschrieben)
  Set.optimization.base_size = true;
  Set.optimization.platform_size = true;
  Set.optimization.base_size_limits = NaN(1,2);
  Set.optimization.platform_size_limits = NaN(1,2);
  %% Parametrierung für Testfälle
  if i_case == 1 % Teste mit fixem Wert für den Gestell-Radius
    Set.optimization.optname = 'test_basesizelimit_const';
    Set.optimization.base_size_limits = [0.500, 0.500];
    Set.optimization.base_size = false;
  elseif i_case == 2 % Teste mit Grenzen für den Plattform-Radius
    Set.optimization.optname = 'test_platformsizelimit';
    Set.optimization.base_size_limits = [0.550, 0.750];
  elseif i_case == 3 % Teste mit fixem Wert für den Plattform-Radius
    Set.optimization.optname = 'test_platformsizelimit_const';
    Set.optimization.platform_size_limits = [0.100, 0.100];
    Set.optimization.platform_size = false;
  else % Teste mit Grenzen für den Plattform-Radius
    Set.optimization.optname = 'test_platformsizelimit';
    Set.optimization.platform_size_limits = [0.110, 0.250];
  end
  %% Optimierung durchführen
  cds_start(Set,Traj);

  %% Teste Ergebnis
  resmaindir = fullfile(Set.optimization.resdir, Set.optimization.optname);
  resdat1 = fullfile(resmaindir, sprintf('Rob%d_%s_Endergebnis.mat', 1, Set.structures.whitelist{1}));
  tmp1 = load(resdat1, 'RobotOptRes');
  resdat2 = fullfile(resmaindir, sprintf('Rob%d_%s_Details.mat', 1, Set.structures.whitelist{1}));
  tmp2 = load(resdat2, 'RobotOptDetails', 'PSO_Detail_Data');
  % Bestimme effektiven Radius
  R_eff_base = norm(tmp2.RobotOptDetails.R.r_0_A_all(:,1));
  R_eff_plf = norm(tmp2.RobotOptDetails.R.r_P_B_all(:,1));
  % Zähle, wie oft die Verletzung auftrat
  I_plfviol =  squeeze(tmp2.PSO_Detail_Data.fval) > 1e13 & ...
               squeeze(tmp2.PSO_Detail_Data.fval) < 5e13;
  I_baseviol = squeeze(tmp2.PSO_Detail_Data.fval) > 5e13 & ...
               squeeze(tmp2.PSO_Detail_Data.fval) < 1e14;
  I_iO = ~isnan(squeeze(tmp2.PSO_Detail_Data.fval)) & ...
         ~isinf(squeeze(tmp2.PSO_Detail_Data.fval));
  fprintf(['%d Fitness-Auswertungen, davon %d mit Fehler für Parameter ', ...
    'außerhalb des erlaubten Bereichs für Plattform, %d für Gestell\n'], ...
    sum(I_iO(:)), sum(I_plfviol(:)), sum(I_baseviol(:)));
  if i_case == 1 % Gestell fix
    assert(all(abs(Set.optimization.base_size_limits-R_eff_base)<1e-10), ...
      'Vorgegebener effektiver Gestell-Radius wird nicht erhalten')
    assert(tmp2.RobotOptDetails.R.DesPar.base_par(2)>1e-3, ...
      'Gestell-Gelenk-Paarabstand ist zu klein. Soll nicht sein.');
    assert(sum(I_plfviol(:))==0, ['Bei fest vorgegebener Gestell-', ...
      'Größe darf die Überschreitung nicht getroffen werden'])
  elseif i_case == 2 % Gestell in Grenzen
    assert(R_eff_base>=Set.optimization.base_size_limits(1) && ...
           R_eff_base<=Set.optimization.base_size_limits(2), ...
        'Effektiver Gestell-Radius außerhalb der Grenzen');
  elseif i_case == 3 % Plattform fix
    assert(all(abs(Set.optimization.platform_size_limits-R_eff_plf)<1e-10), ...
      'Vorgegebener effektiver Plattform-Radius wird nicht erhalten')
    assert(tmp2.RobotOptDetails.R.DesPar.platform_par(2)>1e-3, ...
      'Plattform-Gelenk-Paarabstand ist zu klein. Soll nicht sein.');
    assert(sum(I_plfviol(:))==0, ['Bei fest vorgegebener Plattform-', ...
      'Größe darf die Überschreitung nicht getroffen werden'])
  elseif i_case == 4 % Plattform in Grenzen
    assert(R_eff_plf>=Set.optimization.platform_size_limits(1) && ...
           R_eff_plf<=Set.optimization.platform_size_limits(2), ...
        'Effektiver Plattform-Radius außerhalb der Grenzen');
  end
end
