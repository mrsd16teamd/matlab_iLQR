function generateOutput(dt, Uc)
% input is rear wheel velocity [m/s], steering [radians]
% dt is in seconds -> convert to ms
disp('Generating control input txt file...');

fileID = fopen('controls.txt','w');
fprintf(fileID, 'dt %f\n', dt*1000);
for i = 1:length(Uc)
    vx = Uc(1,i);
    steer = Uc(2,i);
    if (vx>5.0)
       disp('Never mind, these controls suck. Try again.'); 
    end
    fprintf(fileID, 'u %f %f\n', vx, steer);
end

fprintf(fileID, 'end');
fclose(fileID);
disp('Done.');

end