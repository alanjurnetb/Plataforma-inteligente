%clear 
clc
close all
s=instrfind;    
fclose(s);
auto=new_car('COM6');

%%
samples=2000;

% el calor maximo equivale a 255 que es igual a 9W por resistencia
    
carData=zeros(samples,3);
radarData=zeros(samples,5);
%%
wait_connection(auto);
display('connection succes')
stop_car(auto)
set_goal(auto,[3 -0.5]);
start_car(auto);

%%
 for i=1:samples
    [carData(i,:),radarData(i,1:5)]=get_car_data(auto);    
   
    display(carData(i,:));
    display(radarData(i,:));
    
    %if (mod(i,100)==0)

        figure(1)
        scatter(carData(i,1),carData(i,2));
        hold on;
        plot([carData(i,1),carData(i,1)+1000*cos(carData(i,3)*pi/180)],[carData(i,2),carData(i,2)+1000*sin(carData(i,3)*pi/180)]);
        hold on;
        scatter([carData(i,1)+1000*cos(carData(i,3)*pi/180)],[carData(i,2)+1000*sin(carData(i,3)*pi/180)]);
        hold on;
        for j=1:5
            plot([carData(i,1),carData(i,1)+radarData(i,j)*cos((carData(i,3)+(-90-(j-5)*45))*pi/180)],[carData(i,2),carData(i,2)+radarData(i,j)*sin((carData(i,3)+(-90-(j-5)*45))*pi/180)]);
            hold on;
        end
        
        axis([-5000 5000 -5000 5000])
         pause(0.1);

%         hold off
    %end
 end

hold off
%%
 figure(2)
% for i=1:1000
   
    scatter(carData(1:1000,1),carData(1:1000,2));
            axis([-0 4000 -4000 4000])
            axis equal

%     hold on;
%     plot([carData(i,1),carData(i,1)+1000*cos(carData(i,3)*pi/180)],[carData(i,2),carData(i,2)+1000*sin(carData(i,3)*pi/180)]);
%     hold on;
%     scatter([carData(i,1)+1000*cos(carData(i,3)*pi/180)],[carData(i,2)+1000*sin(carData(i,3)*pi/180)]);
%     hold on;
%     for j=1:5
%         plot([carData(i,1),carData(i,1)+radarData(i,j)*cos((carData(i,3)+(-90-(j-5)*45))*pi/180)],[carData(i,2),carData(i,2)+radarData(i,j)*sin((carData(i,3)+(-90-(j-5)*45))*pi/180)]);
%         hold on;
%     end
% end

%%
 
fclose(s)
delete(s)
clear s

%% 
s=instrfind;
fclose(s);