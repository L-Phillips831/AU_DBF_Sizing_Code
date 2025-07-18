function Perf = mkPropData(Path)

TableFlag = 0;
n = 0;
i = 1;

Data = zeros(30,15,50);

fid = fopen(Path);
tline = fgetl(fid);

while ischar(tline)
    tline = fgetl(fid);
    if tline == -1
        
    elseif contains(tline,"RPM =")
        i = i+1;
        n = n+1;
        TableFlag = 1;
    elseif TableFlag == 1

        Data(:,:,i-1) = readmatrix(Path,"Range",string(n+5)+":"+string(n+35));
        TableFlag = 0;
        n = n+1;
    else
        n = n+1;
    end

end


Perf = {1,length(Data(1,1,:))};

for j=1:length(Data(1,1,:))
    Perf{j} = array2table(Data(:,:,j));
    Perf{j}.Properties.VariableNames = {'V (mph)','J','Pe','Ct','Cp','PWR (Hp)','Torque (in-lbf)',...
        'Thrust (lbf)','PWR (W)','Torque (N-m)','Thrust (N)','THR/PWR (g/W)','Mach','Reyn','FOM'};
    Perf{j}.Properties.UserData.RPM = j*1000;
end

end