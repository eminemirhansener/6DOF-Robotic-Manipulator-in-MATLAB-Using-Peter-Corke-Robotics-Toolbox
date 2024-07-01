function varargout = sixDOFmanipulator(varargin)
% SIXDOFMANIPULATOR MATLAB code for sixDOFmanipulator.fig
%      SIXDOFMANIPULATOR, by itself, creates a new SIXDOFMANIPULATOR or raises the existing
%      singleton*.
%
%      H = SIXDOFMANIPULATOR returns the handle to a new SIXDOFMANIPULATOR or the handle to
%      the existing singleton*.
%
%      SIXDOFMANIPULATOR('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SIXDOFMANIPULATOR.M with the given input arguments.
%
%      SIXDOFMANIPULATOR('Property','Value',...) creates a new SIXDOFMANIPULATOR or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before sixDOFmanipulator_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to sixDOFmanipulator_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help sixDOFmanipulator

% Last Modified by GUIDE v2.5 16-Mar-2024 16:40:08

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @sixDOFmanipulator_OpeningFcn, ...
                   'gui_OutputFcn',  @sixDOFmanipulator_OutputFcn, ...
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


% --- Executes just before sixDOFmanipulator is made visible.
function sixDOFmanipulator_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to sixDOFmanipulator (see VARARGIN)

% Choose default command line output for sixDOFmanipulator
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes sixDOFmanipulator wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = sixDOFmanipulator_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function theta1_txt_Callback(hObject, eventdata, handles)
% hObject    handle to theta1_txt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta1_txt as text
%        str2double(get(hObject,'String')) returns contents of theta1_txt as a double


% --- Executes during object creation, after setting all properties.
function theta1_txt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta1_txt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta2_txt_Callback(hObject, eventdata, handles)
% hObject    handle to theta2_txt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta2_txt as text
%        str2double(get(hObject,'String')) returns contents of theta2_txt as a double


% --- Executes during object creation, after setting all properties.
function theta2_txt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta2_txt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta3_txt_Callback(hObject, eventdata, handles)
% hObject    handle to theta3_txt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta3_txt as text
%        str2double(get(hObject,'String')) returns contents of theta3_txt as a double


% --- Executes during object creation, after setting all properties.
function theta3_txt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta3_txt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta4_txt_Callback(hObject, eventdata, handles)
% hObject    handle to theta4_txt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta4_txt as text
%        str2double(get(hObject,'String')) returns contents of theta4_txt as a double


% --- Executes during object creation, after setting all properties.
function theta4_txt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta4_txt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta5_txt_Callback(hObject, eventdata, handles)
% hObject    handle to theta5_txt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta5_txt as text
%        str2double(get(hObject,'String')) returns contents of theta5_txt as a double


% --- Executes during object creation, after setting all properties.
function theta5_txt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta5_txt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta6_txt_Callback(hObject, eventdata, handles)
% hObject    handle to theta6_txt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta6_txt as text
%        str2double(get(hObject,'String')) returns contents of theta6_txt as a double


% --- Executes during object creation, after setting all properties.
function theta6_txt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta6_txt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Px_txt_Callback(hObject, eventdata, handles)
% hObject    handle to Px_txt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Px_txt as text
%        str2double(get(hObject,'String')) returns contents of Px_txt as a double


% --- Executes during object creation, after setting all properties.
function Px_txt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Px_txt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Py_txt_Callback(hObject, eventdata, handles)
% hObject    handle to Py_txt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Py_txt as text
%        str2double(get(hObject,'String')) returns contents of Py_txt as a double


% --- Executes during object creation, after setting all properties.
function Py_txt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Py_txt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Pz_txt_Callback(hObject, eventdata, handles)
% hObject    handle to Pz_txt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pz_txt as text
%        str2double(get(hObject,'String')) returns contents of Pz_txt as a double


% --- Executes during object creation, after setting all properties.
function Pz_txt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Pz_txt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Beta_txt_Callback(hObject, eventdata, handles)
% hObject    handle to Beta_txt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Beta_txt as text
%        str2double(get(hObject,'String')) returns contents of Beta_txt as a double


% --- Executes during object creation, after setting all properties.
function Beta_txt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Beta_txt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Gama_txt_Callback(hObject, eventdata, handles)
% hObject    handle to Gama_txt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Gama_txt as text
%        str2double(get(hObject,'String')) returns contents of Gama_txt as a double


% --- Executes during object creation, after setting all properties.
function Gama_txt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Gama_txt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Phi_txt_Callback(hObject, eventdata, handles)
% hObject    handle to Phi_txt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Phi_txt as text
%        str2double(get(hObject,'String')) returns contents of Phi_txt as a double


% --- Executes during object creation, after setting all properties.
function Phi_txt_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Phi_txt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in ForwardButton.
function ForwardButton_Callback(hObject, eventdata, handles)
% hObject    handle to ForwardButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%%
theta_1 = str2double(handles.theta1_txt.String);
theta_2 = str2double(handles.theta2_txt.String);
theta_3 = str2double(handles.theta3_txt.String);
theta_4 = str2double(handles.theta4_txt.String);
theta_5 = str2double(handles.theta5_txt.String);
theta_6 = str2double(handles.theta6_txt.String);

%% DH Parameters
d = [580, 0, 0, 410, 0, 70];
theta = [theta_1, theta_2 - 90, theta_3, theta_4, theta_5, theta_6];
a = [0, 410, 0, 0, 0, 0];
alpha = [-90, 0, -90, 90, -90, 0] * (pi/180);
%% Parameters
n = 6; % # of links

% initial angle definitions
Q1_initial = 0;
Q2_initial = 0;
Q3_initial = 0;
Q4_initial = 0;
Q5_initial = 0;
Q6_initial = 0;
initial_angles = [Q1_initial, Q2_initial - 90, Q3_initial, Q4_initial, Q5_initial, Q6_initial];

path_res = 10;
%% Forward Kinematics applied and positions are written into textboxes
Tr = fkine(d, theta, a, alpha, n);

set(handles.Px_txt,'String',num2str(round(Tr(1,4), 3)));
set(handles.Py_txt,'String',num2str(round(Tr(2,4), 3)));
set(handles.Pz_txt,'String',num2str(round(Tr(3,4), 3)));
%% Links are defined
%      theta, d, a , alpha
L(1) = Link([0 d(1) a(1) alpha(1)]);
L(2) = Link([-90 d(2) a(2) alpha(2)]);
L(3) = Link([0 d(3) a(3) alpha(3)]);
L(4) = Link([0 d(4) a(4) alpha(4)]);
L(5) = Link([0 d(5) a(5) alpha(5)]);
L(6) = Link([0 d(6) a(6) alpha(6)]);

%% Path is driven
T_1_path = linspace(initial_angles(1), theta(1), path_res);
T_2_path = linspace(initial_angles(2), theta(2), path_res);
T_3_path = linspace(initial_angles(3), theta(3), path_res);
T_4_path = linspace(initial_angles(4), theta(4), path_res);
T_5_path = linspace(initial_angles(5), theta(5), path_res);
T_6_path = linspace(initial_angles(6), theta(6), path_res);

T_1_path_rad = T_1_path * (pi/180);
T_2_path_rad = T_2_path * (pi/180);
T_3_path_rad = T_3_path * (pi/180);
T_4_path_rad = T_4_path * (pi/180);
T_5_path_rad = T_5_path * (pi/180);
T_6_path_rad = T_6_path * (pi/180);
%% Robot is plotted
Robot = SerialLink(L); % Creating Robot Line Model
Robot.name = '6 DOF Robot';
%% Path is traced
P_path_1 = zeros(1, path_res);
P_path_2 = zeros(1, path_res);
P_path_3 = zeros(1, path_res);
for i = 1:path_res
    T = fkine(d, [T_1_path(i), T_2_path(i), T_3_path(i), T_4_path(i), T_5_path(i), T_6_path(i)], a, alpha, n);
    P_path_1(i) = T(1,4);
    P_path_2(i) = T(2,4);
    P_path_3(i) = T(3,4);
end

plot3(P_path_1(:), P_path_2(:), P_path_3(:), 'Color', [1,0,0], 'LineWidth', 2);
%%
for i = 1:path_res
    Robot.plot([T_1_path_rad(i), T_2_path_rad(i), T_3_path_rad(i), T_4_path_rad(i), T_5_path_rad(i), T_6_path_rad(i)], 'scale', 0.5, 'perspective', 'jointdiam', 2, 'jaxes', 'shadow');
end


% --- Executes on button press in InverseButton.
function InverseButton_Callback(hObject, eventdata, handles)
% hObject    handle to InverseButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%% Parameters
Px = str2double(handles.Px_txt.String);
Py = str2double(handles.Py_txt.String);
Pz = str2double(handles.Pz_txt.String);
Beta = str2double(handles.Beta_txt.String);
Gama = str2double(handles.Gama_txt.String);
Phi = str2double(handles.Phi_txt.String);

P = [Px, Py, Pz];
d = [580, 0, 0, 410, 0, 70];
a = [0, 410, 0, 0, 0, 0];
alpha = [-90, 0, -90, 90, -90, 0] * (pi/180);

% initial angle definitions
Q1_initial = 0;
Q2_initial = 0;
Q3_initial = 0;
Q4_initial = 0;
Q5_initial = 0;
Q6_initial = 0;
initial_angles = [Q1_initial, Q2_initial - 90, Q3_initial, Q4_initial, Q5_initial, Q6_initial];

path_res = 10;
n = 6; % # of links
%% Links are defined
%      theta, d, a , alpha
L(1) = Link([0 d(1) a(1) alpha(1)]);
L(2) = Link([-90 d(2) a(2) alpha(2)]);
L(3) = Link([0 d(3) a(3) alpha(3)]);
L(4) = Link([0 d(4) a(4) alpha(4)]);
L(5) = Link([0 d(5) a(5) alpha(5)]);
L(6) = Link([0 d(6) a(6) alpha(6)]);
%% Inverse Kinematics applied
motor_angles = ikine(P, d, a, alpha, Beta, Gama, Phi);
angles = [motor_angles(1), motor_angles(2) - 90, motor_angles(3), motor_angles(4), motor_angles(5), motor_angles(6)];
motor_angles = round(motor_angles,3);
%motor_angles_rad = motor_angles * (pi/180);
%% Angles are written to textboxes
set(handles.theta1_txt,'String',num2str(motor_angles(1)));
set(handles.theta2_txt,'String',num2str(motor_angles(2)));
set(handles.theta3_txt,'String',num2str(motor_angles(3)));
set(handles.theta4_txt,'String',num2str(motor_angles(4)));
set(handles.theta5_txt,'String',num2str(motor_angles(5)));
set(handles.theta6_txt,'String',num2str(motor_angles(6)));
%% Path is driven
T_1_path = linspace(initial_angles(1), angles(1), path_res);
T_2_path = linspace(initial_angles(2), angles(2), path_res);
T_3_path = linspace(initial_angles(3), angles(3), path_res);
T_4_path = linspace(initial_angles(4), angles(4), path_res);
T_5_path = linspace(initial_angles(5), angles(5), path_res);
T_6_path = linspace(initial_angles(6), angles(6), path_res);

T_1_path_rad = T_1_path * (pi/180);
T_2_path_rad = T_2_path * (pi/180);
T_3_path_rad = T_3_path * (pi/180);
T_4_path_rad = T_4_path * (pi/180);
T_5_path_rad = T_5_path * (pi/180);
T_6_path_rad = T_6_path * (pi/180);
%% Robot is plotted
Robot = SerialLink(L); % Creating Robot Line Model
Robot.name = '6 DOF Robot';
%% Path is traced
P_path_1 = zeros(1, path_res);
P_path_2 = zeros(1, path_res);
P_path_3 = zeros(1, path_res);
for i = 1:path_res
    T = fkine(d, [T_1_path(i), T_2_path(i), T_3_path(i), T_4_path(i), T_5_path(i), T_6_path(i)], a, alpha, n);
    P_path_1(i) = T(1,4);
    P_path_2(i) = T(2,4);
    P_path_3(i) = T(3,4);
end

plot3(P_path_1(:), P_path_2(:), P_path_3(:), 'Color', [1,0,0], 'LineWidth', 2);

%%
for i = 1:path_res
    Robot.plot([T_1_path_rad(i), T_2_path_rad(i), T_3_path_rad(i), T_4_path_rad(i), T_5_path_rad(i), T_6_path_rad(i)], 'scale', 0.5, 'perspective', 'jointdiam', 2, 'jaxes', 'shadow');
end
