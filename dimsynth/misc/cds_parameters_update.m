% Aktualisiere geladene Parameter eines früheren Versuchs um
% Code-Änderungen zu berücksichtigen, die Standardwerte ändern.
% Beide Ausgabe-Variablen stellen unterschiedliche Ansätze dar, um ein
% altes Ergebnis neu auszuwerten.
% 
% Eingabe:
% Structure_alt
%   Variable "Structure" aus cds_dimsynth_robot geladen aus Ergebnissen
% Structure_neu
%   Neue Struktur-Variable, direkt von cds_dimsynth_robot erstellt (in der
%   aktuellen Version des Programms)
% p_alt
%   Parametervektor der Optimierung, geladen aus Ergebnis-Datei
% 
% Ausgabe:
% p_neu
%   Aktualisierter Parametervektor, so wie von cds_update_robot_parameters
%   mit Structure_neu zusammen erwartet, aber mit allen möglichen Werten
%   aus p_alt. Der Rest sind Standardwerte
% Structure_neu_mod
%   Modifizierte Version von Structure_neu, mit der der Parametervektor
%   p_alt direkt in cds_update_robot_parameters genutzt werden kann
% state
%   0: Keine Mehrdeutigkeiten, 1: Auflösung eventuell nicht erfolgreich
% 
% Siehe auch: cds_gen_init_pop

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-04
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function [p_neu, Structure_neu_mod, state] = cds_parameters_update(Structure_alt, Structure_neu, p_alt)

state = 0;

Structure_neu_mod = Structure_neu;

% Umbenennung eines Parameters nachvollziehen
Structure_alt.varnames(strcmp(Structure_alt.varnames,'platform_morph')) = ...
  {'platform_morph_pairdist'}; 

% Parameter vergleichen: Schnittmenge der Parameter lässt sich übertragen.
% Fehlende Parameter werden eventuell falsch eingestellt aufgrund der Annahmen
[~, I_neu, I_alt] = intersect(Structure_neu.varnames, Structure_alt.varnames, ...
  'stable');
II_neu = false(length(Structure_neu.varnames),1);
II_neu(I_neu) = true;
II_alt = false(length(Structure_alt.varnames),1);
II_alt(I_alt) = true;
if length(I_neu) ~= length(p_alt)
  % Unterschied der Parameter:
  if any(~II_alt)
    fprintf('Alte Parameter, die nicht im neuen Vektor vorkommen: %s\n', ...
      disp_array(Structure_alt.varnames(~II_alt), '%s'));
  end
  if any(~II_neu)
    fprintf('Neue Parameter, für die keine Werte eingegeben wurden: %s\n', ...
      disp_array(Structure_neu.varnames(~II_neu), '%s'));
  end
  state = 1;
end

% Entferne Optimierungsvariablen, die neu hinzugekommen sind (zwischen der
% Version, mit der der Versuch erstellt wurde und der aktuellen Version)
Structure_neu_mod.varnames = Structure_neu.varnames(I_neu);
Structure_neu_mod.vartypes = Structure_neu.vartypes(I_neu);
Structure_neu_mod.varlim = Structure_neu.varlim(I_neu,:);
% Entferne Einträge zu relevanten Kinematikparametern (z.B. da letzter
% d-Parameter seit Anfang 2022 mit optimiert wird
Structure_neu_mod.Ipkinrel = Structure_alt.Ipkinrel;

% Setze nicht belegte Werte im Parametervektor auf Null
p_neu = zeros(length(Structure_neu.varnames), 1);
p_neu(I_neu) = p_alt(I_alt);

return
%% Debug: Parameter in Tabellenform ausgeben und testen
ParamTab_alt = cell2table(cell(length(p_alt),2), 'VariableNames', {'Name', 'Value'});
ParamTab_alt.Name(:) = Structure_alt.varnames;
ParamTab_alt.Value = p_alt;

ParamTab_neu = cell2table(cell(length(p_neu),2), 'VariableNames', {'Name', 'Value'});
ParamTab_neu.Name(:) = Structure_neu.varnames;
ParamTab_neu.Value = p_neu;

for i_alt = 1:size(ParamTab_alt, 1)
  i_neu = strcmp(ParamTab_neu.Name, ParamTab_alt.Name(i_alt));
  if ~isempty(i_neu)
    assert(abs(ParamTab_alt.Value(i_alt)-ParamTab_neu.Value(i_neu))<1e-10, ...
      'Übertrag der Parameter nicht erfolgreich');    
  end
end