function [fval, fval_debugtext, debug_info] = cds_obj_valid_act(R, Set, Jinvges)
debug_info = {};

if R.Type == 0
  error('Diese Funktion ergibt nur für parallele Roboter Sinn.');
end

% Zielfunktion ist der Rang. Bei vollem Rang "funktioniert" der Roboter
% und der entsprechende Schwellwert wird unterschritten
% Berechne Rang der Jacobi nur für ersten Bahnpunkt. Annahme: Keine
% Singularität, da Trajektorie und IK lösbar ist.
n_qa = sum(R.I_qa);
for i = 1
  Jinv_IK = reshape(Jinvges(i,:), sum(R.I_EE), sum(R.I_qa));
  Jinv_xred = Jinv_IK;
  % Jinv_xred = R.jacobi_qa_x(Q(i,:)', Traj_0.X(i,:)');
  Jinv_3T3R = zeros(6, n_qa);
  Jinv_3T3R(R.I_EE,:) = Jinv_xred;
  Jinv_task = Jinv_3T3R(Set.structures.DoF,:);
  % Rangprüfung. Sehr schlecht konditionierte Matrizen sollen auch als
  % Rangverlust gekennzeichnet werden.
  % Mit Toleranz 1e-4 werden Matrizen mit Kond. 1e7 teilweise noch als
  % voller Rang gewertet. Schlechtere Matrizen werden als Rangverlust
  % gewertet.
  tol = 5e10*max(size(Jinv_task)) * eps(norm(Jinv_task)); % ca. 1e-4 bei "normaler" Matrix mit Werten ungefähr 0 bis 1
  rankJ = rank(Jinv_task, tol);
end
RD = sum(R.I_EE) - rankJ; % Rangdefizit
if RD == 0
  fval = 10; % Reicht zum aufhören
else
  fval = 100*RD; % Kodiere Rangdefizit in Zielfunktion
end
fval_debugtext = sprintf('Rangdefizit %d (Konditionszahl %e, Determinante %e)', ...
  RD, cond(Jinv_task), det(Jinv_task));