% Teste die Begrenzung des effektiven Plattform-Radius bei PKM mit
% paarweise angeordneten Gelenken
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
Set.structures.whitelist = {'P6RRPRRR14V3G6P4A1'}; % P6RRRRRR10V6G6P4A1
Set.optimization.objective = {'condition'};
Set.optimization.obj_limit = 1e3; % bei Erfolg aufhören
Set.optimization.platform_morphology = true;
Set.general.save_animation_file_extensions = {};
Set.general.eval_figures = {'robvisu'};

% Teste mit fixen Grenzen
for i_case = 1:2
  %% Parametrierung für Testfälle
  if i_case == 1 % Teste mit fixen Grenzen
    Set.optimization.optname = 'test_platformsizelimit_const';
    Set.optimization.platform_size_limits = [0.100, 0.100];
    Set.optimization.platform_size = false;
  else
    Set.optimization.optname = 'test_platformsizelimit';
    Set.optimization.platform_size_limits = [0.110, 0.250];
    Set.optimization.platform_size = true;
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
  R_eff = norm(tmp2.RobotOptDetails.R.r_P_B_all(:,1));
  % Zähle, wie oft die Verletzung auftrat
  I_plfviol = squeeze(tmp2.PSO_Detail_Data.fval) > 1e13;
  I_iO = ~isnan(squeeze(tmp2.PSO_Detail_Data.fval)) & ...
         ~isinf(squeeze(tmp2.PSO_Detail_Data.fval));
  fprintf(['%d Fitness-Auswertungen, davon %d mit Fehler für Parameter ', ...
    'außerhalb des erlaubten Bereichs\n'], sum(I_iO(:)), sum(I_plfviol(:)));

  if i_case == 1
    assert(all(abs(Set.optimization.platform_size_limits-R_eff)<1e-10), ...
      'Vorgegebener effektiver Plattform-Radius wird nicht erhalten')
    assert(tmp2.RobotOptDetails.R.DesPar.platform_par(2)>1e-3, ...
      'Plattform-Gelenk-Paarabstand ist zu klein. Soll nicht sein.');
    assert(sum(I_plfviol(:))==0, ['Bei fest vorgegebener Plattform-', ...
      'Größe darf die Überschreitung nicht getroffen werden'])
  elseif i_case == 2
    assert(R_eff>=Set.optimization.platform_size_limits(1) && ...
           R_eff<=Set.optimization.platform_size_limits(2), ...
        'Effektiver Plattform-Radius außerhalb der Grenzen');
  end
end
