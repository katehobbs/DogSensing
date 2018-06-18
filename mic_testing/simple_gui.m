function varargout = simple_gui(varargin)
% SIMPLE_GUI M-file for simple_gui.fig
%      SIMPLE_GUI, by itself, creates a new SIMPLE_GUI or raises the existing
%      singleton*.
%
%      H = SIMPLE_GUI returns the handle to a new SIMPLE_GUI or the handle to
%      the existing singleton*.
%
%      SIMPLE_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SIMPLE_GUI.M with the given input arguments.
%
%      SIMPLE_GUI('Property','Value',...) creates a new SIMPLE_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before simple_gui_OpeningFunction gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to simple_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help simple_gui

% Copyright 2001-2003 The MathWorks, Inc.

% Last Modified by GUIDE v2.5 22-Jan-2018 00:26:14

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @simple_gui_OpeningFcn, ...
    'gui_OutputFcn',  @simple_gui_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin & isstr(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before simple_gui is made visible.
function simple_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to simple_gui (see VARARGIN)
%% Create the data to plot
clear allData counter
global allData counter;         % unsure why it exists
global numDataSetsInPacket;
global xlimit;
global xcounter;
global accelCounter;
global countToClearBuffer;
global secondsBetweenFlushes;
global dataForPlot;
global starttime;

dataForPlot = [];
allData = [];
counter = 0;
xlimit = 512*16*2;
numDataSetsInPacket = 512; %Change this value if needed = # sets of data in a packet
xcounter = 0;
accelCounter = 0;
countToClearBuffer = 0;
secondsBetweenFlushes = 10;

% set current time to editable text
t = datetime('now');
t.Format = 'dd-MM-uuuu HH-mm-ss';
set(handles.timeEditText,'string',string(t));

% Initialize things under handles struct
handles.xlimit = xlimit;              % # data points in x axis
handles.numDataSetsInPacket = 512;  % # sets of data in a packet
handles.xcounter = 0;
handles.countToClearBuffer = 0;
handles.secondsBetweenFlushes = 10;
guidata(hObject, handles);
% create interface with ESP32
global interfaceObject;
echoudp('off');
echoudp('on',3333);
interfaceObject = udp('192.168.4.1',3333);
flushinput(interfaceObject);
interfaceObject.Timeout = 1;
% Choose default command line output for simple_gui
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);

% --- Outputs from this function are returned to the command line.
function varargout = simple_gui_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in pushbutton1.
function start_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global t1;              % global so that previous/future changes apply
global interfaceObject; % global to all buttons
global heartRatePlot;
global accelX;
global accelY;
global accelZ;
global starttime;
% Add more data points to plot

heartRatePlot = animatedline('Color','b', 'MaximumNumPoints', handles.xlimit,  'Parent', handles.audio_axes,'Clipping','on');
accelX = animatedline('Color', 'c', 'MaximumNumPoints', 160,  'Parent', handles.accel_axes,'Clipping','on');
accelY = animatedline('Color', 'r', 'MaximumNumPoints', 160,  'Parent', handles.accel_axes,'Clipping','on');
accelZ = animatedline('Color', 'm', 'MaximumNumPoints', 160,  'Parent', handles.accel_axes,'Clipping','on');

%% Setup interface object to read chunks of data
% Set the number of bytes to read at a time
bytesToRead = 512;
% Define a callback function to be executed when desired number of bytes
% are available in the input buffer
interfaceObject.BytesAvailableFcn = {@localReadAndPlot, bytesToRead, handles};
interfaceObject.BytesAvailableFcnMode = 'byte';
interfaceObject.BytesAvailableFcnCount = bytesToRead;
interfaceObject.InputBufferSize = 512;
%% Take in data
fopen(interfaceObject);
interfaceObject.RecordDetail = 'verbose';
interfaceObject.RecordName = 'incomingData.txt';
interfaceObject.RecordMode = 'append';
record(interfaceObject,'on');

fwrite(interfaceObject,'setRTCTime');
t = datetime('now');
t.Format = 'HH:mm:ss';
starttime = string(t);
t.Format = 'dd-MM-uuuu HH-mm-ss';
% 21:16:22

set(handles.timeEditText,'String',string(t));
set(handles.startTimeText,'backgroundcolor',[0.33 0.85 0.1]);
set(handles.startTimeText,'String',string(t));
fwrite(interfaceObject,get(handles.timeEditText, 'string'));

fwrite(interfaceObject, 'start');
t1=clock;       % First clock value


function localReadAndPlot(interfaceObject,~, bytesToRead, handles)
global xcounter;
global accelCounter;
global xlimit;
global numDataSetsInPacket;
global t1;
global countToClearBuffer;
global secondsBetweenFlushes;
global dataForPlot;
global heartRatePlot;
global accelX;
global accelY;
global accelZ;

if (strcmp(interfaceObject.Status,'open'))
    data = fread(interfaceObject, bytesToRead, 'uint8'); % reads from ESP32

% accelAxis = [1, 2, 3, 4, 5, 6];
% for i=1:1:30
%     accelAxis(i, :) = data(i,:)
% end

if (length(data) == bytesToRead)
    %     if xcounter >= xlimit   % x axis reached max. # data points
    %         xcounter = 0;       % clear counter;
    %         clearpoints(heartRatePlot); % clears from animated line.
    %     end
    %     if accelCounter >= 1000
    %         accelCounter = 0;
    %         clearpoints(accelX);
    %         clearpoints(accelY);
    %         clearpoints(accelZ);
    %     end
    %xData = xcounter+1:(xcounter+(numDataSetsInPacket-6)); % array starts from 1
    
    accelData = data(1:6);
    if (accelData(1)>99)
        x = 100 - accelData(1) - (accelData(2)-100)/100;
    else
        x = accelData(1) - 1 + (accelData(2)-100)/100;
    end
    
    if (accelData(3)>99)
        y = 100 - accelData(3) - (accelData(4)-100)/100;
    else
        y = accelData(3) - 1 + (accelData(4)-100)/100;
    end
    
    if (accelData(5)>99)
        z = 100 - accelData(5) - (accelData(6)-100)/100;
    else
        z = accelData(5) -1 + (accelData(6)-100)/100;
    end
    dataForPlot = [dataForPlot; data(7:end)];
    
    if length(dataForPlot) > 4000
        % audio plot
        xData = xcounter+1:(xcounter+8*(numDataSetsInPacket-6)); % array starts from 1
        axes(handles.audio_axes);
        addpoints(heartRatePlot, xData/8000, dataForPlot);
        xcounter = xcounter + 8*(numDataSetsInPacket-6);
        drawnow;
        dataForPlot = [];
    end
    
    %     % audio plot
    %         axes(handles.audio_axes);
    %         addpoints(heartRatePlot, xData, data(7:end));
    %         xcounter = xcounter + (numDataSetsInPacket-6);
    %         drawnow;
    
    % accel plot
    axes(handles.accel_axes);
    addpoints(accelX, accelCounter, x);
    addpoints(accelY, accelCounter, y);
    addpoints(accelZ, accelCounter, z);
    accelCounter = accelCounter + 506/8000;
    drawnow;
end
t2 = clock;
if (etime(t2,t1) > secondsBetweenFlushes)
    flushinput(interfaceObject);
    disp('Flushed and Reset Clock');
    t1 = clock;
end
countToClearBuffer = countToClearBuffer + 1;
end


%% --- Clean up the interface object
function stop_pushbutton_Callback(hObject, eventdata, handles)
global interfaceObject;
global starttime;
global h1;
global h2;

fwrite(interfaceObject,'stop');
fclose(interfaceObject);
set(handles.startTimeText,'backgroundcolor',[0.85 0.33 0.1]);

[audio, accel] = parseRecording(starttime);
set(handles.audio_axes,'Clipping','off')
axes(handles.audio_axes);
h1 = plot(audio);
axes(handles.accel_axes);
set(handles.accel_axes,'Clipping','off')
h2 = plot(accel);
audiowrite(char(strcat(starttime,'.wav')),2*detrend(audio/max(audio)),7900);
save(char(strcat(starttime,'.mat')),'accel');

set(handles.pushbutton9,'Enable','on');
set(handles.newRecord,'Enable','on');
set(handles.start_pushbutton,'Enable','off');

% --- Executes during object creation, after setting all properties.
% not sure if needed
% function figure1_CreateFcn(hObject, eventdata, handles)
% % hObject    handle to figure1 (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    empty - handles not created until after all CreateFcns called


% --- Executes on button press in newRecord.
function newRecord_Callback(hObject, eventdata, handles)
% hObject    handle to newRecord (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global interfaceObject;
global accelCounter;
global xcounter;
global heartRatePlot;
global accelX;
global accelY;
global accelZ;
global h1;
global h2;

fopen(interfaceObject);
fwrite(interfaceObject,'new');
fclose(interfaceObject);
set(handles.newRecord,'Enable','off');
set(handles.start_pushbutton,'Enable','on');

accelCounter = 0;
clearpoints(accelX);
clearpoints(accelY);
clearpoints(accelZ);

xcounter = 0;       % clear counter;
clearpoints(heartRatePlot); % clears from animated line.

delete(h1);  %delete(gco)
delete(h2);
set(handles.audio_axes,'Clipping','on')
set(handles.accel_axes,'Clipping','on')

% --- Executes on button press in pushbutton9.
function pushbutton9_Callback(hObject, eventdata, handles)  % end all operations
% hObject    handle to pushbutton9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global interfaceObject;
fopen(interfaceObject);
fwrite(interfaceObject,'stop');
flushinput(interfaceObject);
fclose(interfaceObject);
pause(0.20);
delete(interfaceObject);
clear interfaceObject;
echoudp('off');
set(handles.newRecord,'Enable','off');
set(handles.pushbutton9,'Enable','off');
set(handles.stop_pushbutton,'Enable','off');
set(handles.start_pushbutton,'Enable','off');
close(simple_gui);

% --- Executes on button press in pushbutton10.
function pushbutton10_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global interfaceObject;
fwrite(interfaceObject,'WiFiOn');

% --- Executes on button press in pushbutton11.
function pushbutton11_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global interfaceObject;
global accelCounter;
global xcounter;
global accelX;
global accelY;
global accelZ;
global heartRatePlot;

fwrite(interfaceObject,'WiFiOff');

pause(1);
clearpoints(accelX);
clearpoints(accelY);
clearpoints(accelZ);


clearpoints(heartRatePlot); % clears from animated line.


% --- Executes on button press in pushbutton12.
function pushbutton12_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% global interfaceObject;
% fopen(interfaceObject);
% fwrite(interfaceObject,'setRTCTime');
t = datetime('now');
t.Format = 'dd-MM-uuuu HH-mm-ss';
set(handles.timeEditText,'String',string(t));
% fwrite(interfaceObject,get(handles.timeEditText, 'String'));
% fclose(interfaceObject);


function timeEditText_Callback(hObject, eventdata, handles)
% hObject    handle to timeEditText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of timeEditText as text
%        str2double(get(hObject,'String')) returns contents of timeEditText as a double


% --- Executes during object creation, after setting all properties.
function timeEditText_CreateFcn(hObject, eventdata, handles)
% hObject    handle to timeEditText (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
