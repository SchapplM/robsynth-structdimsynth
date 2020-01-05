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
% JinvD_ges
%   Zeitableitung von Jinvges
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
%   1e3...1e4: Überschreitung Belastungsgrenze der Segmente
% 
% Siehe auch: cds_fitness.m

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-01
% (C) Institut für Mechatronische Systeme, Universität Hannover

function fval = cds_dimsynth_desopt_fitness(R, Set, Traj_0, Q, QD, QDD, Jinv_ges, Structure, p_desopt)
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
  p_desopt(1) = p_desopt(2)/2;
  fval = 1e8;
  constrvioltext = 'Radius ist kleiner als Wandstärke';
end

%% Dynamikparameter aktualisieren
% Trage die Dynamikparameter
cds_dimsynth_design(R, Q, Set, Structure, p_desopt);

%% Nebenbedingungen der Entwurfsvariablen berechnen
% TODO: Festigkeit der Segmente, Grenzen der Antriebe
if Set.optimization.desopt_link_yieldstrength
  % Konstanten, Definitionen
  % Dehngrenze von Aluminium-Legierung. Quellen:
  % https://de.wikipedia.org/wiki/Streckgrenze
  % https://de.wikipedia.org/wiki/Aluminium-Kupfer-Legierung
  R_e=250e6;

  % Abhängigkeiten neu berechnen (Dynamik)
  output = cds_obj_dependencies(R, Traj_0, Set, Q, QD, QDD, Jinv_ges);
  if R.Type == 0 % Seriell
    NLEG = 1;
  else % PKM
    NLEG = R.NLEG;
  end
  f_yieldstrength = NaN(7,NLEG);
  for i = 1:NLEG
    % Schnittkräfte dieser Beinkette
    if Structure.Type == 0
      Rob = R;
      Wges = output.Wges;
    else
      Rob = R.Leg(i);
      Wges_i = output.Wges(i,:,:);
      Wges = reshape(Wges_i, size(Q,1), 6*Rob.NL); % in gleiches Format wie SerRob bringen
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
      sigma_max = max(sigma_ges);
      % TODO: Momente in "Balkenrichtung" umrechnen und komplexere Formel
      %       nehmen (Aufteilung in Biegespannung und Torsionsspannung),
      %       Vergleichsspannung z.B. nach von Mises
      % Prüfe, ob Vergleichsspannung größer als Streckgrenze/Dehngrenze ist
      f_yieldstrength(j,i) = sigma_max/R_e;
    end
  end
  % Prüfe, ob für ein Segment die Materialspannung überschritten wurde
  if any(f_yieldstrength(:)>1)
    f_maxstrengthviol = max(f_yieldstrength(:));
    % Normiere auf Wert zwischen 0 und 1
    f_maxstrengthviol_norm = 2/pi*atan(f_maxstrengthviol-1); % 1->0; 10->0.93
    fval = 1e3*(1+9*f_maxstrengthviol_norm); % Normiere in Bereich 1e3...1e4
    constrvioltext = sprintf('Materialbelastungsgrenze überschritten (%d/%d mal; max Faktor %1.1f)', ...
      sum(f_yieldstrength(:)>1), length(f_yieldstrength(:)), f_maxstrengthviol);
  end
end

%% Nebenbedingungen der Zielfunktionswerte berechnen
if fval == 0 && any(Set.optimization.constraint_obj)
  if Set.optimization.constraint_obj(1) % NB für Masse gesetzt
    [fval_mass, fval_debugtext_mass, ~, fphys] = cds_obj_mass(R);
    viol_rel = (fphys - Set.optimization.constraint_obj(1))/Set.optimization.constraint_obj(1);
    if viol_rel > 0
      f_massvio_norm = 2/pi*atan((viol_rel)); % 1->0.5; 10->0.94
      fval = 1e4*(1+9*f_massvio_norm);
      constrvioltext = sprintf('Masse ist zu groß (%d > %d)', fval, fphys, Set.optimization.constraint_obj(1));
    elseif Set.general.verbosity > 3
      fprintf('fval = %1.3e. Masse i.O. (%d < %d)\n', fval, fphys, Set.optimization.constraint_obj(1));
    end
  elseif any(Set.optimization.constraint_obj(1))
    error('Grenzen für andere Zielfunktionen als die Masse noch nicht implementiert');
  end
end
if fval > 1000 % Nebenbedingungen verletzt.
  if Set.general.verbosity > 3
    fprintf('DesOpt-Fitness-Evaluation in %1.1fs. fval=%1.3e. %s\n', toc(t1), fval, constrvioltext);
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
else
  error('Andere Zielfunktion als Masse noch nicht implementiert');
end
if Set.general.verbosity >3
  fprintf('DesOpt-Fitness-Evaluation in %1.1fs. Parameter: [%s]. fval=%1.3e. Erfolgreich. %s.\n', ...
    toc(t1), disp_array(p_desopt', '%1.3f'), fval, fval_debugtext);
end