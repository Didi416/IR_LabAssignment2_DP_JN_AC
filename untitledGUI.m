function varargout = untitledGUI(varargin)

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @untitledGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @untitledGUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before untitledGUI is made visible.
function untitledGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to untitledGUI (see VARARGIN)

% Choose default command line output for untitledGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% This sets up the initial plot - only do when we are invisible
% so window can get raised using untitledGUI.
if strcmp(get(hObject,'Visible'),'off')
    plot(rand(5));
end

% UIWAIT makes untitledGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = untitledGUI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
cla
%{
%Linear ur10 -not working :3
L1 = Link('theta', pi, 'd', 0, 'a', 0, 'alpha', pi/2, 'offset', 1, 'qlim', [-0.8, -0.01]);
L2 = Link('d', 0, 'a', 0.1697, 'alpha', -pi/2, 'offset', 0, 'qlim', [-360*pi/180, 360*pi/180]);
L3 = Link('d', 0, 'a', 0.176, 'alpha', -pi, 'offset', 0, 'qlim', [-90*pi/180, 90*pi/180]);
L4 = Link('d', 0, 'a', 0.12781, 'alpha', pi, 'offset', 0, 'qlim', [-170*pi/180, 170*pi/180]);
L5 = Link('d', 0, 'a', 0.1157, 'alpha', -pi/2, 'offset', 0, 'qlim', [-360*pi/180, 360*pi/180]);
L6 = Link('d', 0, 'a', 0.1157, 'alpha', -pi/2, 'offset', 0, 'qlim', [-360*pi/180, 360*pi/180]);
L7 = Link('d', 0, 'a', 0, 'alpha', 0, 'offset', 0, 'qlim', [-360*pi/180, 360*pi/180]);
%}

axes(handles.axes1);
%{
%ur 5
L1 = Link('d',0.0892,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
L2 = Link('d',0.1357,'a',0.425,'alpha',-pi,'offset',-pi/2,'qlim',[deg2rad(-90),deg2rad(90)]);
L3 = Link('d',0.1197,'a',0.39243,'alpha',pi,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
L4 = Link('d',0.093,'a',0,'alpha',-pi/2,'offset',-pi/2,'qlim',[deg2rad(-360),deg2rad(360)]);
L5 = Link('d',0.093,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
L6 = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
%}

% Robo links
L1 = Link('d', 0.09, 'a', 0, 'alpha', pi/2, 'offset', 0, 'qlim', [deg2rad(-360), deg2rad(360)]);
L2 = Link('d', 0, 'a', -0.43, 'alpha', 0, 'offset', 0, 'qlim', [deg2rad(-90), deg2rad(90)]);
L3 = Link('d', 0, 'a', -0.39, 'alpha', 0, 'offset', 0, 'qlim', [deg2rad(-180), deg2rad(180)]);
L4 = Link('d', 0.11, 'a', 0, 'alpha', pi/2, 'offset', 0, 'qlim', [deg2rad(-360), deg2rad(360)]);
L5 = Link('d', 0.09, 'a', 0, 'alpha', -pi/2, 'offset', 0, 'qlim', [deg2rad(-180), deg2rad(180)]);
L6 = Link('d', 0.08, 'a', 0, 'alpha', 0, 'offset', 0, 'qlim', [deg2rad(-360), deg2rad(360)]);

model = SerialLink([L1 L2 L3 L4 L5 L6],'name','Robo');

for linkIndex = 0:model.n
    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['RoboLink',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
    model.faces{linkIndex+1} = faceData;
    model.points{linkIndex+1} = vertexData;
end
% Display robot
workspace = [-2 2 -2 2 -0.3 2];
model.plot3d(zeros(1,model.n),'noarrow','workspace',workspace);
if isempty(findobj(get(gca,'Children'),'Type','Light'))
    camlight
end
model.delay = 0;
% Try to correctly colour the arm (if colours are in ply file data)
for linkIndex = 0:model.n
    handles = findobj('Tag', model.name);
    h = get(handles,'UserData');
    try
        h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
            , plyData{linkIndex+1}.vertex.green ...
            , plyData{linkIndex+1}.vertex.blue]/255;
        h.link(linkIndex+1).Children.FaceColor = 'interp';
    catch ME_1
        disp(ME_1);
        continue;
    end
end
data = guidata(hObject);
data.model = model;
guidata(hObject,data);


% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% axes(handles.axes1);
% cla;
% 
% popup_sel_index = get(handles.popupmenu1, 'Value');
% switch popup_sel_index
%     case 1
%         plot(rand(5));
%     case 2
%         plot(sin(1:0.01:25.99));
%     case 3
%         bar(1:.5:10);
%     case 4
%         plot(membrane);
%     case 5
%         surf(peaks);
% end


% --------------------------------------------------------------------
function FileMenu_Callback(hObject, eventdata, handles)
% hObject    handle to FileMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function OpenMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to OpenMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
file = uigetfile('*.fig');
if ~isequal(file, 0)
    open(file);
end

% --------------------------------------------------------------------
function PrintMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to PrintMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
printdlg(handles.figure1)

% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to CloseMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
                     ['Close ' get(handles.figure1,'Name') '...'],...
                     'Yes','No','Yes');
if strcmp(selection,'No')
    return;
end

delete(handles.figure1)


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
     set(hObject,'BackgroundColor','white');
end

set(hObject, 'String', {'plot(rand(7))', 'plot(sin(1:0.01:25))', 'bar(1:.5:10)', 'plot(membrane)', 'surf(peaks)'});

%% X buttons
% --- Executes on button press in plusX_pushbutton.
function plusX_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to plusX_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(1,4) = tr(1,4) + 0.01;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);

% --- Executes on button press in minusX_pushbutton.
function minusX_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to minusX_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(1,4) = tr(1,4) - 0.01;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);


%% Y buttons
% --- Executes on button press in plusX_pushbutton.
function plusY_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to plusX_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(2,4) = tr(2,4) + 0.01;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);

% --- Executes on button press in minusX_pushbutton.
function minusY_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to minusX_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(2,4) = tr(2,4) - 0.01;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);


%% Z  buttons
% --- Executes on button press in plusX_pushbutton.
function plusZ_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to plusX_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(3,4) = tr(3,4) + 0.01;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);

% --- Executes on button press in minusX_pushbutton.
function minusZ_pushbutton_Callback(hObject, eventdata, handles)
% hObject    handle to minusX_pushbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(3,4) = tr(3,4) - 0.01;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);