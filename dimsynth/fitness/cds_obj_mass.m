function [fval, fval_debugtext, debug_info] = cds_obj_mass(R)
debug_info = {};

% Gesamtmasse berechnen
if R.Type == 0
  error('TODO');
else
  m_sum = sum(R.DynPar.mges(1:end-1))*R.NLEG + R.DynPar.mges(end);
  debug_info = {debug_info{:}; sprintf('masses: total %1.2fkg, 1Leg %1.2fkg, Pf %1.2fkg', ...
    m_sum, sum(sum(R.DynPar.mges(1:end-1))), R.DynPar.mges(end))};
end
fval_debugtext = sprintf('Gesamtmasse %1.1f kg', m_sum);
f_mass_norm = 2/pi*atan((m_sum)/100); % Normierung auf 0 bis 1; 620 ist 0.9. TODO: Skalierung ändern
fval = 1e3*f_mass_norm; % Normiert auf 0 bis 1e3