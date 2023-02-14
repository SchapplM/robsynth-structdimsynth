% Konvertiere die Detail-Ergebnisse der Optimierung zu einer Pareto-Front
% 
% Eingabe:
% PSO_Detail_Data
%   Struktur mit detaillierten Daten über alle Zwischenschritte der PSO
%   (siehe cds_save_particle_details.m)
% 
% Ausgabe
% RobotOptRes
%   Daten im Format der Ergebnis-Struktur der Roboter-Optimierung
%   (siehe cds_dimsynth_robot.m)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-02
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function RobotOptRes = cds_convert_pareto_details2front(PSO_Detail_Data)
% Bestimme die maximale Generationsnummer (um Daten zu reduzieren)
for gg = 1:size(PSO_Detail_Data.fval,3)
  if any( isnan(PSO_Detail_Data.fval(:,1,gg)) )
    break;
  end
end
maxgen = min(gg + 1, size(PSO_Detail_Data.fval,3));
% Variablen initialisieren
numparticles = size(PSO_Detail_Data.fval,1)*maxgen;
fval_all = NaN(numparticles, size(PSO_Detail_Data.fval,2));
physval_all = fval_all;
pval_all = NaN(numparticles, size(PSO_Detail_Data.pval,2));
q0_all = NaN(numparticles, size(PSO_Detail_Data.q0_ik,2));
% Alle Kriterien durchgehen und Daten in neues Format bringen
for kk = 1:size(PSO_Detail_Data.fval,2)
  fval_all(:,kk) = reshape(squeeze(PSO_Detail_Data.fval(:,kk,1:maxgen)), ...
    numparticles, 1);
  physval_all(:,kk) = reshape(squeeze(PSO_Detail_Data.physval(:,kk,1:maxgen)), ...
    numparticles, 1);
end
for kk = 1:size(PSO_Detail_Data.pval,2)
  pval_all(:,kk) = reshape(squeeze(PSO_Detail_Data.pval(:,kk,1:maxgen)), ...
    numparticles, 1);
end
for kk = 1:size(PSO_Detail_Data.q0_ik,2)
  q0_all(:,kk) = reshape(squeeze(PSO_Detail_Data.q0_ik(:,kk,1:maxgen)), ...
    numparticles, 1);
end
% Reduziere die Daten bis zum ersten NaN. Ab dort wurde aufgehört zu rechnen
I_firstnan = find(isnan(fval_all(:,1)), 1, 'first');
if ~isempty(I_firstnan)
  fval_all = fval_all(1:I_firstnan-1,:);
  physval_all = physval_all(1:I_firstnan-1,:);
  pval_all = pval_all(1:I_firstnan-1,:);
end
% Als Struktur ausgeben
RobotOptRes = struct('fval_all', fval_all, 'physval_all', physval_all, ...
  'pval_all', pval_all, 'q0_all', q0_all);