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

% Last Modified by GUIDE v2.5 10-May-2022 19:07:38

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
map = Environments(0.1);

data = guidata(hObject);
data.map = map;
guidata(hObject,data);


% --- Executes on button press in stuff.
function stuff_Callback(hObject, eventdata, handles)
% hObject    handle to stuff (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.h = 0.51;
handles.pt = 0.03;

ws = [-0.5 0.5 -0.5 0.5 0 0.8];
[q2_, q3_] = deal(deg2rad(35), deg2rad(105));
q0 = [-pi/2 q2_ q3_ constrain_joint4(q2_, q3_) 0];

r1 = Dobot(ws, 1, 2);
r1.model.base = transl(-0.1, -0.15, 0.49) * trotz(pi);
r1.model.animate(q0);

r2 = Dobot(ws, 2, 2);
r2.model.base = transl(0.2, -0.45, 0.49) * trotz(pi);
r2.model.animate(q0);
hold on

drawnow();
axis equal

pcb1 = PCB(1, transl(-0.33, 0, handles.h) * trotz(pi/2));
pcb2 = PCB(2, transl(-0.31, 0.3, handles.h));
pcb3 = PCB(3, transl(-0.33, 0.6, handles.h));
pcbs = [pcb1, pcb2, pcb3];

view([0 0 1]);

handles.r1 = r1;
handles.r2 = r2;
handles.pcbs = pcbs;
handles.q0 = q0;

set(handles.teachpannel, 'Visible', 'on');
update_strings(hObject,handles);
guidata(hObject,handles);


function animate_traj(q, dest, model, obj, path, weight, plot, move_ply)
    if ~exist('pt', 'var'), pt = 0.02; end

    current_pose = model.fkine(model.getpos);
    [qmatrix, desired] = rmrc(current_pose, dest, q, model, false, path, weight);
    
    if plot == true, plot3(desired(1, :), desired(2, :), desired(3, :), 'y.', 'LineWidth', 1); end % plot
    for i=1:length(qmatrix)
       model.animate(qmatrix(i, :));
       ee = model.fkine(qmatrix(i, :));

       if plot == true, plot3(ee(1, 4), ee(2, 4), ee(3, 4), 'b*'); end
       if move_ply == true, obj.MoveMesh(ee); end

       pause(pt);
    end
    if move_ply == true, obj.MoveMesh(dest); end


function animate_conveyor(obj, start, finish, steps)
    if ~exist('pt', 'var'), pt = 0.02; end
    start_pos = start(1:3, 4)';
    finish_pos = finish(1:3, 4)';

    x = zeros(3, steps);
    theta = zeros(3, 3, steps);

    s = lspb(0,1,steps);
    for i = 1:steps
        x(1,i) = (1-s(i)) * start_pos(1) + s(i) * finish_pos(1);       % Points in x
        x(2,i) = (1-s(i)) * start_pos(2) + s(i) * finish_pos(2);       % Points in y
        x(3,i) = (1-s(i)) * start_pos(3) + s(i) * finish_pos(3);       % Points in z
    end

    for i=1:steps
        obj.tran(x(:, i)');
        pause(pt);
    end


function trace_path(obj, model, plot)
    if ~exist('pt', 'var'), pt = 0.02; end
    % trace devel from centre pose of shape for translations agnostic of the
    % initial coords

    if obj.type == 1
        plt = [];
        cp = model.fkine(model.getpos);
        [qmatrix, desired] = rmrc(cp, cp + transl(-0.02, 0.07, 0), model.getpos, model, false, 1, false);
        plt = loop_qmatrix(qmatrix, model, plt, false, false);
%         if plot == true, plot3(desired(1, :), desired(2, :), desired(3, :), 'y.', 'LineWidth', 1); end % plot

        cp = model.fkine(model.getpos);
        [qmatrix, desired] = rmrc(cp, cp + transl(0.04, 0, 0), model.getpos, model, false, 1, false);
        plt = loop_qmatrix(qmatrix, model, plt, true, 'c*');

        cp = model.fkine(model.getpos);
        [qmatrix, desired] = rmrc(cp, cp + transl(-0.04, -0.04, 0), model.getpos, model, false, 1, false);
        plt = loop_qmatrix(qmatrix, model, plt, true, 'c*');

        cp = model.fkine(model.getpos);
        [qmatrix, desired] = rmrc(cp, cp + transl(0, -0.04, 0), model.getpos, model, false, 1, false);
        plt = loop_qmatrix(qmatrix, model, plt, true, 'c*');

        cp = model.fkine(model.getpos);
        [qmatrix, desired] = rmrc(cp, cp + transl(0.04, 0, 0), model.getpos, model, false, 1, false);
        plt = loop_qmatrix(qmatrix, model, plt, true, 'c*');

        cp = model.fkine(model.getpos);
        [qmatrix, desired] = rmrc(cp, cp + transl(0, -0.04, 0), model.getpos, model, false, 1, false);
        plt = loop_qmatrix(qmatrix, model, plt, true, 'c*');

        cp = model.fkine(model.getpos);
        [qmatrix, desired] = rmrc(cp, model.base + transl(-0.25, -0.15, 0), model.getpos, model, false, 1, false);
        plt = loop_qmatrix(qmatrix, model, plt, false, false);

        disp(strcat(['    Completed trace path for PCB ', num2str(obj.type)]));
        pause(3);

        for i=1:length(plt), delete(plt{i}); end

    end
 

function plots = loop_qmatrix(qmatrix, model, plots, plot, linespec)
    if ~exist('linespec', 'var'), linespec = 'b*'; end
    if ~exist('pt', 'var'), pt = 0.02; end

    for i=1:length(qmatrix)
           model.animate(qmatrix(i, :));
           ee = model.fkine(qmatrix(i, :));
           if plot == true
               plt = plot3(ee(1, 4), ee(2, 4), ee(3, 4), linespec, 'MarkerSize', 0.1); 
               plots{length(plots)+1} = plt;
           end
           pause(pt);
    end


% --- Executes on button press in simstarter.
function simstarter_Callback(hObject, eventdata, handles)
% hObject    handle to simstarter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

ws_origin = transl(-0.03, -0.42, handles.h) * trotz(pi/2);
stow_away = [ws_origin(1:3, 1:3), ws_origin(1:3, 4) - [0.1; -0.1; 0]; zeros(1, 3), 1;];
conv_out = transl(-0.3, -0.4, handles.h);

for i=1:length(handles.pcbs)
    disp(' ')
    disp(strcat(['PCB: ', num2str(i)]));

    disp('a. Move R1 from q0 to pcb1')
    animate_traj(handles.q0, handles.pcbs(i).pose, handles.r1.model, false, 1, 0, false, false);
    
    disp('b. Move R1 to ws origin')
    animate_traj(handles.r1.model.getpos, ws_origin, handles.r1.model, handles.pcbs(i), 5, -0.1, false, true)
    
    disp('c. Move R1 to stow away pose for laser operation')
    animate_traj(handles.r1.model.getpos, stow_away, handles.r1.model, false, 1, false, false, false)
    
    disp('d. Move R2 to ws origin')
    animate_traj(handles.q0, ws_origin, handles.r2.model, false, 1, false, false, false)
    
    % Insert trace paths here
    trace_path(handles.pcbs(i), handles.r2.model, true)
    
    disp('f. Move R2 to q0')
    animate_traj(handles.q0, handles.r2.model.fkine(handles.q0), handles.r2.model, false, 1, false, false, false)
    handles.r2.model.animate(handles.q0);
    
    disp('g. Move R1 from stow away to pcb1')
    animate_traj(handles.r1.model.getpos, handles.pcbs(i).pose, handles.r1.model, handles.pcbs(i), 1, false, false, false)
    
    disp('h. Move R1 from ws origin to conveyor out with pcb1')
    animate_traj(handles.r1.model.getpos, conv_out, handles.r1.model, handles.pcbs(i), 1, false, false, true)
    
    disp('i. Move R1 from conveyor out to q0')
    animate_traj(handles.r1.model.getpos, handles.r1.model.fkine(handles.q0), handles.r1.model, false, 5, -0.2, false, false)

    % 8a. Animate Conveyor
    animate_conveyor(handles.pcbs(i), handles.pcbs(i).pose, handles.pcbs(i).pose - transl(0.7, 0, 0), 30);
    if i < 2 animate_conveyor(handles.pcbs(2), handles.pcbs(2).pose, handles.pcbs(2).pose - transl(0, 0.3, 0), 30); end
    if i < 3 animate_conveyor(handles.pcbs(3), handles.pcbs(3).pose, handles.pcbs(3).pose - transl(0, 0.3, 0), 30); end

end


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
    set(handles.stoptext, 'BackgroundColor','red');
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
path = get(hObject,'String');
s = serialport(path,9600);
handles.s = s;

guidata(hObject,handles);

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
q = handles.r1.model.getpos;
q(1,1) = q(1,1)-0.1;
handles.r1.model.animate(q);

update_strings(hObject,handles)

guidata(hObject,handles);

% --- Executes on button press in q2minus.
function q2minus_Callback(hObject, eventdata, handles)
% hObject    handle to q2minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.r1.model.getpos;
q(1,2) = q(1,2)-0.1;
q(1,4) = constrain_joint4(q(1,2),q(1,3));
handles.r1.model.animate(q);

update_strings(hObject,handles)

guidata(hObject,handles);



% --- Executes on button press in q3minus.
function q3minus_Callback(hObject, eventdata, handles)
% hObject    handle to q3minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.r1.model.getpos;
q(1,3) = q(1,3)-0.1;
q(1,4) = constrain_joint4(q(1,2),q(1,3));
handles.r1.model.animate(q);

update_strings(hObject,handles)

guidata(hObject,handles);


% --- Executes on button press in q4minus.
function q4minus_Callback(hObject, eventdata, handles)
% hObject    handle to q4minus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.r1.model.getpos;
q(1,5) = q(1,5)-0.1;
handles.r1.model.animate(q);

update_strings(hObject,handles)

guidata(hObject,handles);


% --- Executes on button press in xminus.
function xminus_Callback(hObject, eventdata, handles)
% hObject    handle to xminus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.r1.model.getpos;
tr = handles.r1.model.fkine(q);

tr(1,4) = tr(1,4)-0.01;

update_strings(hObject,handles);

newq = handles.r1.model.ikcon(tr,q);
handles.r1.model.animate(newq);

% updatebotp(hObject,handles);

guidata(hObject,handles);


% --- Executes on button press in yminus.
function yminus_Callback(hObject, eventdata, handles)
% hObject    handle to yminus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.r1.model.getpos;
tr = handles.r1.model.fkine(q);

tr(2,4) = tr(2,4)-0.01;

update_strings(hObject,handles)

newq = handles.r1.model.ikcon(tr,q);
handles.r1.model.animate(newq);

% updatebotp(hObject,handles);

guidata(hObject,handles);

% --- Executes on button press in zminus.
function zminus_Callback(hObject, eventdata, handles)
% hObject    handle to zminus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)ws_origin = transl(-0.03, -0.42, h) * trotz(pi/2);

stow_away = [ws_origin(1:3, 1:3), ws_origin(1:3, 4) - [0.1; -0.1; 0]; zeros(1, 3), 1;];
conv_out = transl(-0.3, -0.4, h);

q = handles.r1.model.getpos;
tr = handles.r1.model.fkine(q);

tr(3,4) = tr(3,4)-0.01;

update_strings(hObject,handles)

newq = handles.r1.model.ikcon(tr,q);
handles.r1.model.animate(newq);

% updatebotp(hObject,handles);

guidata(hObject,handles);

% --- Executes on button press in q1plus.
function q1plus_Callback(hObject, eventdata, handles)
% hObject    handle to q1plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.r1.model.getpos;
q(1,1) = q(1,1)+0.1;
handles.r1.model.animate(q);

update_strings(hObject,handles)

guidata(hObject,handles);


% --- Executes on button press in q2plus.
function q2plus_Callback(hObject, eventdata, handles)
% hObject    handle to q2plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.r1.model.getpos;
q(1,2) = q(1,2)+0.1;
q(1,4) = constrain_joint4(q(1,2),q(1,3));
handles.r1.model.animate(q);

update_strings(hObject,handles)

guidata(hObject,handles);


% --- Executes on button press in q3plus.
function q3plus_Callback(hObject, eventdata, handles)
% hObject    handle to q3plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.r1.model.getpos;
q(1,3) = q(1,3)+0.1;
q(1,4) = constrain_joint4(q(1,2),q(1,3));
handles.r1.model.animate(q);

update_strings(hObject,handles)

guidata(hObject,handles);


% --- Executes on button press in q4plus.
function q4plus_Callback(hObject, eventdata, handles)
% hObject    handle to q4plus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.r1.model.getpos;
q(1,5) = q(1,5)+0.1;
handles.r1.model.animate(q);

update_strings(hObject,handles)

guidata(hObject,handles);


% --- Executes on button press in xplus.
function xplus_Callback(hObject, eventdata, handles)
% hObject    handle to xplus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


q = handles.r1.model.getpos;
tr = handles.r1.model.fkine(q);

tr(1,4) = tr(1,4)+0.01;

update_strings(hObject,handles)

newq = handles.r1.model.ikcon(tr,q);
handles.r1.model.animate(newq);

% updatebotp(hObject,handles);

guidata(hObject,handles);


% --- Executes on button press in yplus.
function yplus_Callback(hObject, eventdata, handles)
% hObject    handle to yplus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.r1.model.getpos;
tr = handles.r1.model.fkine(q);

tr(2,4) = tr(2,4)+0.01;

update_strings(hObject,handles)

newq = handles.r1.model.ikcon(tr,q);
handles.r1.model.animate(newq);

% updatebotp(hObject,handles);

guidata(hObject,handles);


% --- Executes on button press in zplus.
function zplus_Callback(hObject, eventdata, handles)
% hObject    handle to zplus (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.r1.model.getpos;
tr = handles.r1.model.fkine(q);

tr(3,4) = tr(3,4)+0.01;

update_strings(hObject,handles)

newq = handles.r1.model.ikcon(tr,q);
handles.r1.model.animate(newq);

% updatebotp(hObject,handles);

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


function checkhardwarestop(hObject,handles)
% polls serial port to see the state of the estop
flush(s);
data = read(s,1,"char");
% arduino code send 1 if estop is on and 0 if estop is turned off
% update the variables
if data == "1"
    handles.cont = 0;
    handles.state = 1;
    set(handles.stoptext, 'Visible','on');
    set(handles.stoptext, 'BackgroundColor','red');
end
if data == "0"
    handles.state = 0;
    set(handles.stoptext, 'BackgroundColor','green');
end

guidata(hObject,handles);

function update_strings(hObject,handles)
% function updates the all the strings on the jogging UI
q = handles.r1.model.getpos;

set(handles.text3, 'String',q(1,1));
set(handles.text4, 'String',q(1,2));
set(handles.text5, 'String',q(1,3));
set(handles.text6, 'String',q(1,5));

tr = handles.r1.model.fkine(q);

set(handles.text7, 'String',tr(1,4));
set(handles.text8, 'String',tr(2,4));
set(handles.text9, 'String',tr(3,4));

guidata(hObject,handles);



function path_Callback(hObject, eventdata, handles)
% hObject    handle to path (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of path as text
%        str2double(get(hObject,'String')) returns contents of path as a double


% --- Executes during object creation, after setting all properties.
function path_CreateFcn(hObject, eventdata, handles)
% hObject    handle to path (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit conr2.model.base + transl(-0.3, -0.05, 0)trols usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1
value = get(hObject,'Value');
disp(value)
if value == 1
    set(handles.vspannel, 'Visible', 'on');
    
    handles.centr = transl(0,0,0)* troty(pi/2);
    handles.P = getP(handles.centr,0.025);
    drawpoints(hObject,handles);
else
    set(handles.vspannel, 'Visible', 'off');
end
data = guidata(hObject)
guidata(hObject,handles);



% --- Executes on button press in vsminusx.
function vsminusx_Callback(hObject, eventdata, handles)
% hObject    handle to vsminusx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.centr = transl(-0.01,0,0) * handles.centr;
handles.P = getP(handles.centr,0.025);
deletepoints(hObject,handles);
drawpoints(hObject,handles);

guidata(hObject,handles);

% --- Executes on button press in vsminusy.
function vsminusy_Callback(hObject, eventdata, handles)
% hObject    handle to vsminusy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.centr = transl(0,-0.01,0) * handles.centr;
handles.P = getP(handles.centr,0.025);
deletepoints(hObject,handles);
drawpoints(hObject,handles);

guidata(hObject,handles);

% --- Executes on button press in vsminusz.
function vsminusz_Callback(hObject, eventdata, handles)
% hObject    handle to vsminusz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.centr = transl(0,0,-0.01) * handles.centr;
handles.P = getP(handles.centr,0.025);
deletepoints(hObject,handles);
drawpoints(hObject,handles);

guidata(hObject,handles);

% --- Executes on button press in vsplusx.
function vsplusx_Callback(hObject, eventdata, handles)
% hObject    handle to vsplusx (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.centr = transl(0.01,0,0) * handles.centr;
handles.P = getP(handles.centr,0.025);
deletepoints(hObject,handles);
drawpoints(hObject,handles);

guidata(hObject,handles);

% --- Executes on button press in vsplusy.
function vsplusy_Callback(hObject, eventdata, handles)
% hObject    handle to vsplusy (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.centr = transl(0,0.01,0) * handles.centr;
handles.P = getP(handles.centr,0.025);
deletepoints(hObject,handles);
drawpoints(hObject,handles);

guidata(hObject,handles);

% --- Executes on button press in vsplusz.
function vsplusz_Callback(hObject, eventdata, handles)
% hObject    handle to vsplusz (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.centr = transl(0,0,0.01) * handles.centr;
handles.P = getP(handles.centr,0.025);
deletepoints(hObject,handles);
drawpoints(hObject,handles);

guidata(hObject,handles);

% --- Executes on button press in vsminusroll.
function vsminusroll_Callback(hObject, eventdata, handles)
% hObject    handle to vsminusroll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
currentCent = handles.centr;
rot = eye(4);
rot(1:3,1:3) = handles.centr(1:3,1:3);
currentCent = currentCent * inv(currentCent);
currentCent = currentCent*rot*trotx(-0.1);
currentCent(1:3,4) = handles.centr(1:3,4);


handles.centr = currentCent;
handles.P = getP(handles.centr,0.025);

deletepoints(hObject,handles);
drawpoints(hObject,handles);

guidata(hObject,handles);

% --- Executes on button press in vsminuspitch.
function vsminuspitch_Callback(hObject, eventdata, handles)
% hObject    handle to vsminuspitch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in vsminusyaw.
function vsminusyaw_Callback(hObject, eventdata, handles)
% hObject    handle to vsminusyaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in vsplusroll.
function vsplusroll_Callback(hObject, eventdata, handles)
% hObject    handle to vsplusroll (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
currentCent = handles.centr;
rot = eye(4);
rot(1:3,1:3) = handles.centr(1:3,1:3);
currentCent = currentCent * inv(currentCent);
currentCent = currentCent*rot*trotx(0.1);
currentCent(1:3,4) = handles.centr(1:3,4);


handles.centr = currentCent;
handles.P = getP(handles.centr,0.025);

deletepoints(hObject,handles);
drawpoints(hObject,handles);

guidata(hObject,handles);

% --- Executes on button press in vspluspitch.
function vspluspitch_Callback(hObject, eventdata, handles)
% hObject    handle to vspluspitch (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in vsplusyaw.
function vsplusyaw_Callback(hObject, eventdata, handles)
% hObject    handle to vsplusyaw (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

function updatevsstrings(hObject, eventdata,handles)
% function updates the text labels for in the vspannel
tr = handles.centr;
set(handles.vsx, 'String',tr(1,4));
set(handles.vsy, 'String',tr(2,4));
set(handles.vsz, 'String',tr(3,4));

guidata(hObject,handles);

function drawpoints(hObject,handles)
% plots the tracking spheres
P = handles.P;
pl1 = plot_sphere(P(:,1), 0.025, 'b');
pl2 = plot_sphere(P(:,2), 0.025, 'b');
pl3 = plot_sphere(P(:,3), 0.025, 'b');
pl4 = plot_sphere(P(:,4), 0.025, 'b');

handles.surfs = pl1;
data = guidata(hObject)
guidata(hObject,handles);

function deletepoints(hObject,handles)
% deletes the plotted ones
    % make a class for the points
%     drawnow();
guidata(hObject,handles);
