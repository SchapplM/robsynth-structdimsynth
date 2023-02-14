% Entferne Einstellungen aus der Maßsynthese in den bestehenden Ergebnissen
% Kann benutzt werden, wenn Einstellungen falsch implementiert wurden und
% dadurch Prüfungen alter Ergebnisse in cds_gen_init_pop falsch sind.
% 
% Eingabe:
% OptName_Pattern
%   Suchasudruck der Optimierung (RegExp, z.B. 'cryopkm_20230202_paramconstr_(.)*')
% Set_del
%   Einstellungs-Struktur mit Feldern. Alle existierenden Felder werden
%   gelöscht. Orientiert sich an Struktur aus cds_settings_defaults.
% ResDir
%   Ordner, in dem die Maßsynthese-Ergebnisse liegen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-02
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function cds_remove_settings_in_results(Set_del, OptName_Pattern, ResDir)

% Alle Optimierungsordner durchgehen
optdirs = dir(fullfile(ResDir, '*'));
fprintf(['Gehe durch alle %d Unterordner von %s und prüfe auf Aktualisierung ', ...
  'der Einstellungen.\n'], length(optdirs), ResDir);
for i = 1:length(optdirs)
  if ~optdirs(i).isdir
    continue
  end
  if optdirs(i).name(1) == '.'
    continue
  end
  [tokens, ~] = regexp(optdirs(i).name, OptName_Pattern, 'tokens', 'match');
  if isempty(tokens)
    continue
  else
    fprintf('Suchausdruck in Optimierung %s gefunden.\n', optdirs(i).name);
  end

  sf = fullfile(ResDir,optdirs(i).name,[optdirs(i).name,'_settings.mat']);
  if ~exist(sf, 'file')
    fprintf('%s: Einstellungsdatei existiert nicht\n', optdirs(i).name);
    continue % Altes Format oder ungültiges Verzeichnis
  end
  ds = load(sf);
  Set = ds.Set;
  for f1 = fields(Set_del)'
    Set_f1 = Set.(f1{1});
    for f2 = fields(Set_del.(f1{1}))'
      Set_f1 = rmfield(Set_f1, f2{1});
      fprintf('Entferne Feld %s.%s aus den Einstellungen\n', f1{1}, f2{1});
    end
    Set.(f1{1}) = Set_f1;
  end
  save(sf, 'Set');
  fprintf('Datei %s neu gespeichert\n', sf);
end