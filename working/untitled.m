function varargout = untitled(varargin)
% UNTITLED MATLAB code for untitled.fig
%      UNTITLED, by itself, creates a new UNTITLED or raises the existing
%      singleton*.
%
%      H = UNTITLED returns the handle to a new UNTITLED or the handle to
%      the existing singleton*.
%
%      UNTITLED('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in UNTITLED.M with the given input arguments.
%
%      UNTITLED('Property','Value',...) creates a new UNTITLED or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before untitled_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to untitled_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help untitled

% Last Modified by GUIDE v2.5 08-May-2022 00:23:54

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @untitled_OpeningFcn, ...
                   'gui_OutputFcn',  @untitled_OutputFcn, ...
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


% --- Executes just before untitled is made visible.
function untitled_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to untitled (see VARARGIN)

% Choose default command line output for untitled
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% This sets up the initial plot - only do when we are invisible
% so window can get raised using untitledGUI.
if strcmp(get(hObject,'Visible'),'off')
    plot(rand(5));
end
% UIWAIT makes untitled wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = untitled_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in map.
function map_Callback(hObject, eventdata, handles)
% hObject    handle to map (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cla 
map = Environments(2);

data = guidata(hObject);
data.map = map;
guidata(hObject,data);


% --- Executes on button press in stuff.
function stuff_Callback(hObject, eventdata, handles)
% hObject    handle to stuff (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
hold on;
ws = [-0.5 0.5 -0.5 0.5 0 0.8];
q = [0,pi/2,3*pi/4,constrain_joint4(pi/2,3*pi/4),0];
r1 = Dobot(ws,1,2);
r1.model.base = transl(0.2,0,0.490)*trotz(pi);
r2 = Dobot(ws,2,2);
r2.model.base = transl(-0.2,0,0.490)*trotz(-pi/2);
r1.model.animate(q);
r2.model.animate(q);
drawnow();
axis equal

handles.q1 = 0;
handles.q2 = pi/2;
handles.q3 = 3*pi/4;
handles.q4 = constrain_joint4(handles.q2,handles.q3);
handles.q5 = 0;
qpos = r1.model.getpos;
pos = r1.model.fkine(qpos);
handles.px = pos(1,4);
handles.py = pos(2,4);
handles.pz = pos(3,4);


set(handles.text3, 'String',handles.q1);
set(handles.text4, 'String',handles.q2);
set(handles.text5, 'String',handles.q3);
set(handles.text6, 'String',handles.q5);

set(handles.text7, 'String',handles.px);
set(handles.text8, 'String',handles.py);
set(handles.text9, 'String',handles.pz);

handles.r1 = r1;
handles.r2 = r2;
guidata(hObject,handles);


% --- Executes on button press in simstarter.
function simstarter_Callback(hObject, eventdata, handles)
% hObject    handle to simstarter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in estop.
function estop_Callback(hObject, eventdata, handles)
% hObject    handle to estop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of estop
state = get(hObject,'Value');
handles.state = state;
if state == 1
    handles.conti = 0;
    set(handles.stoptext, 'Visible','on');
end
if state == 0
    set(handles.stoptext, 'BackgroundColor','green');
end

guidata(hObject, handles);


% --- Executes on button press in connect_b.
function connect_b_Callback(hObject, eventdata, handles)
% hObject    handle to connect_b (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in q1minus.
function q1minus_Callback(hObject, eventdata, handles)
% hObject    handle to q1minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
txt = get(handles.text3, 'String');
txt = str2double(txt);

txt = txt - 0.1;
set(handles.text3, 'String',txt);

handles.q1 = txt;

updatebotq(hObject,handles);

guidata(hObject,handles);

% --- Executes on button press in q2minus.
function q2minus_Callback(hObject, eventdata, handles)
% hObject    handle to q2minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
txt = get(handles.text4, 'String');
txt = str2double(txt);

txt = txt - 0.1;
set(handles.text4, 'String',txt);

handles.q2 = txt;
handles.q4 = constrain_joint4(handles.q2,handles.q3);

updatebotq(hObject,handles);

guidata(hObject,handles);



% --- Executes on button press in q3minus.
function q3minus_Callback(hObject, eventdata, handles)
% hObject    handle to q3minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
txt = get(handles.text5, 'String');
txt = str2double(txt);

txt = txt - 0.1;
set(handles.text5, 'String',txt);

handles.q3 = txt;
handles.q4 = constrain_joint4(handles.q2,handles.q3);

updatebotq(hObject,handles);

guidata(hObject,handles);


% --- Executes on button press in q4minus.
function q4minus_Callback(hObject, eventdata, handles)
% hObject    handle to q4minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
txt = get(handles.text6, 'String');
txt = str2double(txt);

txt = txt - 0.1;
set(handles.text6, 'String',txt);

handles.q5 = txt;

updatebotq(hObject,handles);

guidata(hObject,handles);


% --- Executes on button press in xminus.
function xminus_Callback(hObject, eventdata, handles)
% hObject    handle to xminus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
txt = get(handles.text7, 'String');
txt = str2double(txt);

txt = txt - 0.001;
set(handles.text7, 'String',txt);

handles.px = txt;

updatebotp(hObject,handles);

guidata(hObject,handles);


% --- Executes on button press in yminus.
function yminus_Callback(hObject, eventdata, handles)
% hObject    handle to yminus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
txt = get(handles.text8, 'String');
txt = str2double(txt);

txt = txt - 0.001;
set(handles.text8, 'String',txt);

handles.py = txt;

updatebotp(hObject,handles);

guidata(hObject,handles);

% --- Executes on button press in zminus.
function zminus_Callback(hObject, eventdata, handles)
% hObject    handle to zminus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
txt = get(handles.text9, 'String');
txt = str2double(txt);

txt = txt - 0.001;
set(handles.text9, 'String',txt);

handles.pz = txt;

updatebotp(hObject,handles);

guidata(hObject,handles);

% --- Executes on button press in q1plus.
function q1plus_Callback(hObject, eventdata, handles)
% hObject    handle to q1plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
txt = get(handles.text3, 'String');
txt = str2double(txt);

txt = txt + 0.1;
set(handles.text3, 'String',txt);

handles.q1 = txt;

updatebotq(hObject,handles);

guidata(hObject,handles);


% --- Executes on button press in q2plus.
function q2plus_Callback(hObject, eventdata, handles)
% hObject    handle to q2plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
txt = get(handles.text4, 'String');
txt = str2double(txt);

txt = txt + 0.1;
set(handles.text4, 'String',txt);

handles.q2 = txt;
handles.q4 = constrain_joint4(handles.q2,handles.q3);

updatebotq(hObject,handles);

guidata(hObject,handles);


% --- Executes on button press in q3plus.
function q3plus_Callback(hObject, eventdata, handles)
% hObject    handle to q3plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
txt = get(handles.text5, 'String');
txt = str2double(txt);

txt = txt + 0.1;
set(handles.text5, 'String',txt);

handles.q3 = txt;
handles.q4 = constrain_joint4(handles.q2,handles.q3);

updatebotq(hObject,handles);

guidata(hObject,handles);


% --- Executes on button press in q4plus.
function q4plus_Callback(hObject, eventdata, handles)
% hObject    handle to q4plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
txt = get(handles.text6, 'String');
txt = str2double(txt);

txt = txt + 0.1;
set(handles.text6, 'String',txt);

handles.q5 = txt;

updatebotq(hObject,handles);

guidata(hObject,handles);


% --- Executes on button press in xplus.
function xplus_Callback(hObject, eventdata, handles)
% hObject    handle to xplus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
txt = get(handles.text7, 'String');
txt = str2double(txt);

txt = txt + 0.001;
set(handles.text7, 'String',txt);

handles.px = txt;

updatebotp(hObject,handles);

guidata(hObject,handles);


% --- Executes on button press in yplus.
function yplus_Callback(hObject, eventdata, handles)
% hObject    handle to yplus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
txt = get(handles.text8, 'String');
txt = str2double(txt);

txt = txt + 0.001;
set(handles.text8, 'String',txt);

handles.py = txt;

updatebotp(hObject,handles);

guidata(hObject,handles);


% --- Executes on button press in zplus.
function zplus_Callback(hObject, eventdata, handles)
% hObject    handle to zplus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
txt = get(handles.text9, 'String');
txt = str2double(txt);

txt = txt + 0.001;
set(handles.text9, 'String',txt);

handles.pz = txt;

updatebotp(hObject,handles);

guidata(hObject,handles);


% --- Executes on button press in cont.
function cont_Callback(hObject, eventdata, handles)
% hObject    handle to cont (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
state = handles.state;
contstate = handles.conti;

if contstate == 0
    if state == 0
        handles.conti = 1;
        set(handles.stoptext, 'Visible','off');
    end
end
guidata(hObject,handles);


% --- Executes on button press in applybtn.
function applybtn_Callback(hObject, eventdata, handles)
% hObject    handle to applybtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
txt1 = get(handles.text9, 'String');
txt1 = str2double(txt1);
txt2 = get(handles.text9, 'String');
txt2 = str2double(txt2);
txt3 = get(handles.text9, 'String');
txt3 = str2double(txt3);
txt4 = get(handles.text9, 'String');
txt4 = str2double(txt4);

txt5 = get(handles.text9, 'String');
txt5 = str2double(txt5);
txt6 = get(handles.text9, 'String');
txt6 = str2double(txt6);
txt7 = get(handles.text9, 'String');
txt7 = str2double(txt7);

handles.q1 = txt1;
handles.q2 = txt2;
handles.q3 = txt3;
handles.q4 = constrain_joint4(handles.q2,handles.q3);
handles.q5 = txt4;


handles.px = txt5;
handles.py = txt6;
handles.pz = txt7;
guidata(hObject,handles);

function updatebotq(hObject,handles)

% updates the plot and the xyz values based on the q values when called
q = [handles.q1,handles.q2,handles.q3,handles.q4,handles.q5];
handles.r1.model.animate(q);

pos = handles.r1.model.getpos;
handles.px = pos(1,1);
handles.py = pos(1,2);
handles.pz = pos(1,3);

set(handles.text7, 'String',handles.px);
set(handles.text8, 'String',handles.py);
set(handles.text9, 'String',handles.pz);

guidata(hObject,handles);

function updatebotp(hObject,handles)
% updates the plot and the q values based on the xyz values when called
qnew = handles.r1.model.ikcon(transl(handles.px,handles.py,handles.pz));
handles.q1 = qnew(1,1);
handles.q2 = qnew(1,2);
handles.q3 = qnew(1,3);
handles.q4 = qnew(1,4);
handles.q5 = qnew(1,5);

q = [handles.q1,handles.q2,handles.q3,handles.q4,handles.q5];
handles.r1.model.animate(q);

set(handles.text3, 'String',handles.q1);
set(handles.text4, 'String',handles.q2);
set(handles.text5, 'String',handles.q3);
set(handles.text6, 'String',handles.q5);

guidata(hObject,handles);
