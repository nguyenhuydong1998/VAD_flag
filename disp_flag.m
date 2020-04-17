function disp_flag(filespeech,fileflag)
sp = readBin(filespeech,'short');
fl = readBin(fileflag,'short');
sp = sp(1:length(fl)*320);
sp = sp./(max(abs(sp)));
A = fl + 2;
A = upsample(A,320) ;

for i = 1:length(A)
    if A(i) ~= 0 
        buf = A(i);
    end
    if A(i) == 0
        A(i) = buf;
    end
end
A = (A - 2).*-1 - 0.5;
hold on;
plot(sp,'b');
plot(A,'r');
ylim([-2 2]);
hold off;
