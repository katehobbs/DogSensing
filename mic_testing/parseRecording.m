function [data, acceldata] = parseRecording(starttime) 
fid = fopen("incomingData.txt",'r');
tline = 0;
t = 0;
samplenumber=1;

readUint8=false;
stopRecord = false;

data = ones(60*8000*25,1)*-1;
acceldata = ones(60*8000*25/512*6,1)*-1;
counter = 1;
counter1 = 1;
tic
while (tline ~= -1)
    tline = fgets(fid);
    
    if (readUint8)
        if contains(string(tline),'512 uint8 values')
            %display(tline);
            tline = fgets(fid);
            temp = sscanf(tline,'%x');
            if (length(tline) == 56)
                acceldata = [acceldata; temp(1:6)];
                data(counter1:counter1+length(temp(7:end))-1) =  temp(7:end);
                counter1 = counter1+length(temp(7:end));
                tline = fgets(fid);
            end
            
            
            while (length(tline) == 56)
                %data = [data; sscanf(tline,'%x')];
                [temp,m] = sscanf(tline,'%x');
                
                data(counter1:counter1+m-1) = temp;
                counter1 = counter1+m;
                tline = fgets(fid);
                if contains(string(tline),'4 uchar values')
                    tline = -1;
                end
            end
            if contains(string(tline),'4 uchar values')
                tline = -1;
            end
        end
        
        if contains(string(tline),'4 uchar values')
            tline = -1;
        end
        %display(tline);
    end
    
    if contains(string(tline),'Recording on') % break point before next test segment
        %display(tline);
        if contains(string(tline), char(starttime))  %    14:43:45.459 - 23:06:52.612
            readUint8=true;
            display(tline);
        end
        %             t = t + 1;
        %             D = textscan(fid,'%f,%f,%f,%f');
        %
        %             if (samplenumber == 1)
        %                 force1 = D{1};
        %                 displacement1 = D{2};
        %                 time1 = D{3};
        %             end
        %             if (samplenumber == 2)
        %                 force2 = D{1};
        %                 displacement2 = D{2};
        %                 time2 = D{3};
        %             end
        %             if (samplenumber == 3)
        %                 force3 = D{1};
        %                 displacement3 = D{2};
        %                 time3 = D{3};
        %             end
        %             samplenumber = samplenumber + 1;
    end
    
end
fclose(fid);
data(find(data<0,1):end) = [];
acceldata(find(acceldata<0,1):end) = [];
toc