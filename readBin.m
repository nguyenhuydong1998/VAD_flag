function A = readBin(fileName,dataType)

fid = fopen(fileName,'r');
A = fread(fid,dataType);
fclose('all');
