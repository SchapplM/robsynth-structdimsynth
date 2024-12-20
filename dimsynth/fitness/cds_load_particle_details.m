% Lade die während der Optimierung gespeicherten Detail-Informationen
% 
% Eingabe:
% PSO_Detail_Data
%   Struktur mit Detailsauswertung für die einzelnen PSO-Partikel
% fval
%   Fitness-Wert, der in den Daten gefunden werden soll
% 
% Ausgabe:
% k_gen
%   Index der Generation des Optimierungsalgorithmus 
%   (0=Initialpopulation, -1=Fehler)
% k_ind
%   Index des Individuums in der Generation k_gen
% 
% Siehe auch:
% cds_save_particle_details.m


% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2020-09
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [k_gen, k_ind] = cds_load_particle_details(PSO_Detail_Data, fval)
assert(size(fval,2) == 1 && size(fval,1) == size(PSO_Detail_Data.fval,2), ...
  'cds_load_particle_details: fval muss %dx1 sein.', size(PSO_Detail_Data.fval,2))
size_data = [size(PSO_Detail_Data.fval,3), size(PSO_Detail_Data.fval,1)];
% Suche den exakten Treffer für alle Zielkriterien. Benutze eine Maske zur
% Prüfung aller Kriterien der Pareto-Optimierung.
fval_oc_mask = true(size_data)';
for oc = 1:length(fval) % Gehe alle Optimierungskriterien durch
  % Variable zum Finden: Dim. 1: Generationen, Dim. 2: Individuen
  fval_oc = squeeze(PSO_Detail_Data.fval(:,oc,:))';
  % Lasse nur dort eine 1, wo dieses Zielkriterium exakt den als
  % Endergebnis der Optimierung gefundenen Wert hat.
  if ~isnan(fval(oc))
    fval_oc_mask = fval_oc_mask & (fval(oc) == fval_oc');
  else % NaN zum Suchen der aktuellen Iterationsnummer
    fval_oc_mask = fval_oc_mask & isnan(fval_oc');
  end
end
k = find(fval_oc_mask, 1, 'first'); % Umrechnen der Indizes
if isempty(k)
  % Abbruch, deutet auf einen Syntax-Fehler hin. Wert muss da sein.
  k_gen = -1;
  k_ind = -1;
  warning('Gesuchter Wert [%s] nicht in gespeicherten Daten gefunden.', ...
    disp_array(fval(:)', '%1.3e'));
  return
end
[k_ind,k_gen] = ind2sub(fliplr(size_data),k); % Umrechnung in 2D-Indizes: Generation und Individuum

% Prüfe, ob die Indizes stimmen
if ~any(isnan(fval))
  test_fval = PSO_Detail_Data.fval(k_ind,:,k_gen)' - fval;
else
  test_fval = ~isnan(PSO_Detail_Data.fval(k_ind,:,k_gen)');
end
if any(test_fval~=0)
  error('Geladene Daten stimmen nicht zu dem übergebenen Fitness-Wert');
end
