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
set_goal(auto,[0 -3]);
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
%         plot([carData(i,1),carData(i,1)+1000*cos(carData(i,3)*pi/180)],[carData(i,2),carData(i,2)+1000*sin(carData(i,3)*pi/180)]);
%         hold on;
        scatter([carData(i,1)+1000*cos(carData(i,3)*pi/180)],[carData(i,2)+1000*sin(carData(i,3)*pi/180)]);
        hold on;
        for j=1:5
            plot([carData(i,1),carData(i,1)+radarData(i,j)*cos((carData(i,3)+(-90-(j-5)*45))*pi/180)],[carData(i,2),carData(i,2)+radarData(i,j)*sin((carData(i,3)+(-90-(j-5)*45))*pi/180)]);
            hold on;
        end
        
        axis([-5000 5000 -5000 5000])
         pause(0.1);

        hold off
    %end
 end

hold off
%%
load('cardata.mat')
 figure(2)
% for i=1:1000

    carData(:,2)=carData(:,2)+150
    scatter(carData(1:54,1),carData(1:54,2),'b');
            hold on;
                plot(carData(1:54,1),carData(1:54,2),'b');

    cd=carData(1:54,:);
    cd(12:end,1)=cd(12:end,1)*0.2+400;
%     scatter(cd(:,1),cd(:,2),'red');
    axis([-2000 1500 -4500 1500])
            axis equal
     
%       plot([carData(1:1000,1),carData(1:1000,1)+1000*cos(carData(1:1000,3)*pi/180)],[carData(1:1000,2),carData(1:1000,2)+1000*sin(carData(1:1000,3)*pi/180)]);
     hold on;
     scatter([0],[-3000],'red','+')
     scatter([50],[-2800],'green','+')
    wall_x=10*[0,110,110,75,75,110,110,-150,-150,10,10,50,50,10,10,0];
    wall_y=10*[80,80,-165,-165,-195,-195,-800,-800,-180,-180,-120,-120,-95,-95,-50,-50]+100;
    plot(wall_x,wall_y,'black')
%     scatter([carData(1:1000,1)+1000*cos(carData(1:1000,3)*pi/180)],[carData(1:1000,2)+1000*sin(carData(1:1000,3)*pi/180)]);
    xlabel('x in mm');
    ylabel('y in mm');
    title('Deadreckoning');
    hold on;
    %%
for i=1:1000
     for j=1:5
        plot([carData(i,1),carData(i,1)+radarData(i,j)*cos((carData(i,3)+(-90-(j-5)*45))*pi/180)],[carData(i,2),carData(i,2)+radarData(i,j)*sin((carData(i,3)+(-90-(j-5)*45))*pi/180)]);
        hold on;
    end
end
hold off

%%
 
fclose(s)
delete(s)
clear s

%% 
s=instrfind;
fclose(s);
