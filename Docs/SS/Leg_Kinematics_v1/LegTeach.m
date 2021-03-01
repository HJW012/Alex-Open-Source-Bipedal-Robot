function varargout = LegTeach(varargin)
% LEGTEACH MATLAB code for LegTeach.fig
%      LEGTEACH, by itself, creates a new LEGTEACH or raises the existing
%      singleton*.
%
%      H = LEGTEACH returns the handle to a new LEGTEACH or the handle to
%      the existing singleton*.
%
%      LEGTEACH('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LEGTEACH.M with the given input arguments.
%
%      LEGTEACH('Property','Value',...) creates a new LEGTEACH or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before LegTeach_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to LegTeach_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help LegTeach

% Last Modified by GUIDE v2.5 06-Sep-2020 16:48:37

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @LegTeach_OpeningFcn, ...
                   'gui_OutputFcn',  @LegTeach_OutputFcn, ...
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


% --- Executes just before LegTeach is made visible.
function LegTeach_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to LegTeach (see VARARGIN)

% Choose default command line output for LegTeach
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes LegTeach wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = LegTeach_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
