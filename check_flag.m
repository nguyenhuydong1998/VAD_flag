A = zeros(9,1369);
for i = 0:8
[~,~] = system(['coder sample-2.inp flag_VAD' num2str(i) '.cod' ]);
A(i+1,:) = readBin(['flag_VAD' num2str(i) '.cod'],'short');
disp(['coder tst.inp flag' num2str(i) '.cod' ]);
end

disp_flag('sample-2.inp','flag_VAD0.cod');

