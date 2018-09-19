
% % reduce the dimention for both NaN case
% m = 1;
% for i = 1:length(zt)
%     if check(i) ~= 0
%         index(m) = i;
%         m = m+1;
%     end
% end
% if m > 1
%     check = check(index);
%     Kt = Kt(index,index);
%     reducedD = Kt*(check');
%     mu = muBar1;
%     for i = 1:length(index)
%         mu(index(i)) = mu(index(i))+reducedD(i);
%     end
% else
%     mu = muBar1;
% end