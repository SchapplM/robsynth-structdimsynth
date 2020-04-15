function InitPop = cds_gen_init_pop(NumIndividuals,nvars,varlim,varnames)

InitPop = rand(NumIndividuals,nvars);

for i = 1:nvars
  scale_check = contains(varnames{i},{'scale'}); %cellfun('isempty',strfind(varnames{i},{'scale'}));
  if scale_check == 1 % nicht pkin parameter
    InitPop(1:NumIndividuals,i) = 1; 
    continue
  end  
  
  pkin_check = contains(varnames{i},'pkin'); 
  if pkin_check == 0 % nicht pkin parameter
    continue
  end
  pkin_theta = contains(varnames(i),{'theta'});
  if pkin_theta == 1 % theta parameter
    InitPop(1:ceil(NumIndividuals/2),i) = pi/2; 
    InitPop(ceil(NumIndividuals/2)+1:end,i) = 0;
%     InitPop(ceil(NumIndividuals/2)+1:end,i) = repmat(varlim(i,1)', ceil(NumIndividuals/2)-1,1)...
%                                               + rand(ceil(NumIndividuals/2)-1,1).*...
%                                               repmat(varlim(i,2)'-varlim(i,1)',ceil(NumIndividuals/2)-1,1);
    continue
  end
  pkin_alpha = contains(varnames(i),{'alpha'});
  if pkin_alpha == 1 % alpha parameter
    InitPop(1:ceil(NumIndividuals/4),i) = 0; % alpha Init ist 0
    InitPop(ceil(NumIndividuals*3/4)+1:end,i) = 0;
    InitPop(ceil(NumIndividuals/4)+1:ceil(NumIndividuals*3/4),i) = pi/2; 
    continue
  end
  pkin_d = contains(varnames(i),{'d'});
  if pkin_d == 1 % alpha parameter
    InitPop(1:ceil(NumIndividuals/2),i) = 0.1*rand(ceil(NumIndividuals/2),1); % alpha Init ist 0
    if mod(NumIndividuals,2)
      InitPop(ceil(NumIndividuals/2)+1:end,i) = repmat(varlim(i,1)', ceil(NumIndividuals/2)-1,1)...
        + rand(ceil(NumIndividuals/2)-1,1).*...
        repmat(varlim(i,2)'-varlim(i,1)',ceil(NumIndividuals/2)-1,1);
    else
      InitPop(ceil(NumIndividuals/2)+1:end,i) = repmat(varlim(i,1)', ceil(NumIndividuals/2),1)...
        + rand(ceil(NumIndividuals/2),1).*...
        repmat(varlim(i,2)'-varlim(i,1)',ceil(NumIndividuals/2),1);
    end
    continue
  end  
  pkin_a = contains(varnames(i),{'a'});
  if pkin_a == 1 % a parameter
    init_a = 0.3+rand(NumIndividuals,1)*0.2;
    InitPop(:,i) = init_a(1:NumIndividuals)';
%     InitPop(:,i) = repmat(varlim(i,1)', NumIndividuals,1) + rand(NumIndividuals, 1) .* ...
%                         repmat(varlim(i,2)'-varlim(i,1)',NumIndividuals,1); 
    continue
  end 
end