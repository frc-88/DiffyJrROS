function varargout = pitchgui(varargin)
%**************************************************************%
%This functions is intended for use the pitchgui.m file.  %
%It contains the code necessary to run the GUI's animation     %
%and controls.                                                 %
%                                                              %
%Copyright (C) 1997 by the Regents of the University of        %
%Michigan.                                                     %
%Modified by Asst. Prof. Rick Hill (U Detroit-Mercy) and his   %
%student Li Guan.                                              %
%**************************************************************%

gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @pitchgui_OpeningFcn, ...
                   'gui_OutputFcn',  @pitchgui_OutputFcn, ...
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


% --- Executes just before pitchgui is made visible.
function pitchgui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to pitchgui (see VARARGIN)

% Choose default command line output for pitchgui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes pitchgui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = pitchgui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in rub.
function rub_Callback(hObject, eventdata, handles)
% hObject    handle to rub (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
      	yo=[0 0 0];

      	A=[-0.313       56.7    0;
           -0.0139      -0.426  0;
            0           56.7    0];
                
      	B=[0.232;
           0.0203;
           0];
              
      	C=[0 0 1];

      	D=[0];
        
%Get the weighing factor from the editable text field%
		wx = get(handles.wfactor,'String');
	    wx = str2num(wx);
      	Q=[0 0 0;
           0 0 0;
           0 0 wx];
       
%Use the LQR command to find the K-matrix%             
      	[K]= lqr (A,B,Q,1);
        
%Check for reference input%
        Nbarval = get(handles.reference,'Value');
        if Nbarval == 0
           Nbar = 1;
        elseif Nbarval == 1
           s = size(A,1);
           Z = [zeros([1,s]) 1];
           N = inv([A,B;C,D])*Z';
           Nx = N(1:s);
           Nu = N(1+s);
           Nbar=Nu + K*Nx;
        end     

%Get the value of the step input from the slider%        
        stepval=get(handles.stepslider,'value');
        
        t=0:0.1:10;
      	de=stepval*ones(size(t));
        
%Run the simulation%             
      	[Y,X]=lsim (A-B*K,B*Nbar,C,D,de,t,yo);
      	theta=Y;
        
%Place a zero at the first point%    	
      	change1=theta;
      	lt=length(theta);
      	for n1=1:lt-1
      	  change1(1)=0;
      	  change1(n1+1)=theta(n1);
      	end
      	theta=change1;
      	  
      	deltae=-K*X'+Nbar*de;
      	deltae=deltae';
      	ld=length(deltae);
      	change2=deltae;
      	for n2=1:ld-1
      	  change2(1)=0;
      	  change2(n2+1)=deltae(n2);
      	end 
      	deltae=change2; 
        
%Check if the response is to be plotted separately%       	
        plotval=get(handles.plotbox,'Value');   
        axes(handles.axes1)
        if plotval == 1
           plot(t,theta)
        else
           plot(t(1),theta(1))
           hold on
        end

%Set the axis limits for the step response plot%
        if stepval > 0
           axis([0 10 0 stepval*2])
        elseif stepval < 0
           axis([0 10 stepval*2 0])
        else
           axis([0 10 -0.4 0.4])
        end

        title ('Step Response')
      	ylabel ('Pitch angle (rad)')
      	xlabel ('Time (sec)')
        
        hold on

%Animation Program begins%
	l=0.7;
	l2=0.15;
	l3=0.3;
	l4=0.6;
	l5=0.3;
	l6=0.05;
	h=0.03;
	h2=0.2;
	c=2*sqrt(h^2+(l/2)^2);
	c2=2*sqrt(h^2+(l3/2)^2);
	c3=2*sqrt(h^2+(l4/2)^2);
	c4=2*sqrt(h^2+(l5/2)^2);
	c5=sqrt(h2^2+(l6)^2);
	phi=atan(2*h/l);
  	phi2=atan(2*h/l3);
  	phi3=atan(2*h/l4);
  	phi4=atan(2*h/l5);
  	phi5=atan(2*h2/l6);
  	
%Lines%
  	xtopnose=c/2*cos(phi+theta);
	ytopnose=c/2*sin(phi+theta);
	xtoptail=-(c/2)*cos(phi-theta);
	ytoptail=(c/2)*sin(phi-theta);

	xbotnose=-c/2*cos(pi-phi+theta);
	ybotnose=-c/2*sin(pi-phi+theta);
	xbottail=(c2/2)*cos(pi-phi2-theta);
	ybottail=-(c2/2)*sin(pi-phi2-theta);

	xtcaptip=(l/2+l2)*cos(theta);
	ytcaptip=(l/2+l2)*sin(theta);
	xtcapend=c/2*cos(phi+theta);
	ytcapend=c/2*sin(phi+theta);

	xbcaptip=(l/2+l2)*cos(theta);
	ybcaptip=(l/2+l2)*sin(theta);
	xbcapend=-c/2*cos(pi-phi+theta);
	ybcapend=-c/2*sin(pi-phi+theta);

	xendtop=-(c/2)*cos(phi-theta);
	yendtop=(c/2)*sin(phi-theta);
	xendbot=-(l/2)*cos(theta);
	yendbot=-(l/2)*sin(theta);
	
	xdiatop=-(l/2)*cos(theta);
	ydiatop=-(l/2)*sin(theta);
	xdiabot=(c2/2)*cos(pi-phi2-theta);
	ydiabot=-(c2/2)*sin(pi-phi2-theta);

	xwlbot=-(c3/2)*cos(phi3-theta);
	ywlbot=(c3/2)*sin(phi3-theta);
	xwltop=-(c3/2)*cos(phi3-theta)-h2*sin(theta);
	ywltop=(c3/2)*sin(phi3-theta)+h2*cos(theta);

	xwrbot=-(c4/2)*cos(phi4-theta);
	ywrbot=(c4/2)*sin(phi4-theta);
	xwrtop=-(c4/2)*cos(phi4-theta)-c5*sin((0.6+pi)/2-phi5+theta);
	ywrtop=(c4/2)*sin(phi4-theta)+c5*cos((0.6+pi)/2-phi5+theta);

	xwtopr=-(c4/2)*cos(phi4-theta)-c5*sin((0.6+pi)/2-phi5+theta);
	ywtopr=(c4/2)*sin(phi4-theta)+c5*cos((0.6+pi)/2-phi5+theta);
	xwtopl=-(c3/2)*cos(phi3-theta)-h2*sin(theta);
	ywtopl=(c3/2)*sin(phi3-theta)+h2*cos(theta);
	
  	elevxl = -(l/3)*cos(theta)-0.12*cos(deltae);
	elevxr = -(l/3)*cos(theta);
	elevyl = -(l/3)*sin(theta)+0.12*sin(deltae);
	elevyr = -(l/3)*sin(theta);
  	
%Center of gravity represented by a dot
  	radius=0.005;
	arcstep = 18; % angle increment (in degrees)
	j = 0:arcstep:(360-arcstep); % create ball sides using arclengths
	arcx = radius * cos((j+arcstep) * pi/180); % x coordinates of ball sides
	arcy = radius * sin((j+arcstep) * pi/180); % y coordinates of ball sides
%Plotting the first frame of the animation%
	axes(handles.axes2)
	cla
	K = plot([xtopnose(1) xtoptail(1)], [ytopnose(1) ytoptail(1)], 'k',...
	 	'LineWidth',[2]);
	hold on
	L = plot([xbotnose(1) xbottail(1)], [ybotnose(1) ybottail(1)],...
		'k','LineWidth',[2]);
    hold on

	M = plot([xtcaptip(1) xtcapend(1)], [ytcaptip(1) ytcapend(1)],'k',...
		'LineWidth',[2]);
    hold on
	
	N = plot([xbcaptip(1) xbcapend(1)],[ybcaptip(1) ybcapend(1)],'k',...
		'LineWidth',[2]);
    hold on

	O = plot([xendtop(1) xendbot(1)],[yendtop(1) yendbot(1)],'k',...
		'LineWidth',[2]);
    hold on
		
	O2 = plot([elevxl(1) elevxr(1)],[elevyl(1) elevyr(1)],'r',...
		'LineWidth',[3]);
    hold on
	
	P = plot([xdiatop(1) xdiabot(1)],[ydiatop(1) ydiabot(1)],'k',...
		'LineWidth',[2]);
    hold on
	
	Q = plot([xwrtop(1) xwrbot(1)],[ywrtop(1) ywrbot(1)],'k',...
		'LineWidth',[2]);
    hold on
	
	R = plot([xwltop(1) xwlbot(1)],[ywltop(1) ywlbot(1)],'k',...
		'LineWidth',[2]);
    hold on
		
	S = plot([xwtopr(1) xwtopl(1)],[ywtopr(1) ywtopl(1)],'k',...
		'LineWidth',[2]);
    hold on
	
	H = patch(arcx, arcy,'b');
    hold on
        
        axis([-0.55 0.55 -0.55 0.55])

%Check if the frames should advance manually or automaically%
        man=get(handles.manualbox,'Value');
        if man == 1
           'Press any key to advance the animation'
            pause
        end   
	
	
	ltime=length(t);      
	title ('Animation')
    
%Run the animation%
	for i = 2:ltime-1,
	
	   if man == 1
	      pause
	   end   
      
      	   set(K,'XData',[xtopnose(i) xtoptail(i)]);
      	   set(K,'YData',[ytopnose(i) ytoptail(i)]);
      	   set(L,'Xdata',[xbotnose(i) xbottail(i)]);
      	   set(L,'Ydata',[ybotnose(i) ybottail(i)]);
      	   set(M,'XData',[xtcaptip(i) xtcapend(i)]);
      	   set(M,'YData',[ytcaptip(i) ytcapend(i)]);
      	   set(N,'XData',[xbcaptip(i) xbcapend(i)]);
      	   set(N,'YData',[ybcaptip(i) ybcapend(i)]);
           set(O,'XData',[xendtop(i) xendbot(i)]);
           set(O,'YData',[yendtop(i) yendbot(i)]);
           set(P,'XData',[xdiatop(i) xdiabot(i)]);
           set(P,'YData',[ydiatop(i) ydiabot(i)]);
           set(Q,'XData',[xwrtop(i) xwrbot(i)]);
           set(Q,'YData',[ywrtop(i) ywrbot(i)]);
      	   set(R,'XData',[xwltop(i) xwlbot(i)]);
      	   set(R,'YData',[ywltop(i) ywlbot(i)]);
      	   set(S,'XData',[xwtopr(i) xwtopl(i)]);
      	   set(S,'YData',[ywtopr(i) ywtopl(i)]);
      	   
      	   set(O2,'XData',[elevxl(i), elevxr(i)]);
      	   set(O2,'YData',[elevyl(i), elevyr(i)]);
      	   drawnow;
           
      	  axes(handles.axes1)
      	   if plotval == 0
      	      plot([t(i),t(i+1)],[theta(i),theta(i+1)])
              hold on
      	   end 
      	  
      	end      
guidata(hObject, handles);

% --- Executes on button press in reset.
function reset_Callback(hObject, eventdata, handles)
% hObject    handle to reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%The callback for the Reset button: Initialze plots% 	
%Constant values
	l=0.7;
	l2=0.15;
	l3=0.3;
	l4=0.6;
	l5=0.3;
	l6=0.05;
	h=0.03;
	h2=0.2;
	c=2*sqrt(h^2+(l/2)^2);
	c2=2*sqrt(h^2+(l3/2)^2);
	c3=2*sqrt(h^2+(l4/2)^2);
	c4=2*sqrt(h^2+(l5/2)^2);
	c5=sqrt(h2^2+(l6)^2);
	phi=atan(2*h/l);
  	phi2=atan(2*h/l3);
  	phi3=atan(2*h/l4);
  	phi4=atan(2*h/l5);
  	phi5=atan(2*h2/l6);
  	theta=0;
  	deltae=0;
%Lines
  	xtopnose=c/2*cos(phi+theta);
	ytopnose=c/2*sin(phi+theta);
	xtoptail=-(c/2)*cos(phi-theta);
	ytoptail=(c/2)*sin(phi-theta);

	xbotnose=-c/2*cos(pi-phi+theta);
	ybotnose=-c/2*sin(pi-phi+theta);
	xbottail=(c2/2)*cos(pi-phi2-theta);
	ybottail=-(c2/2)*sin(pi-phi2-theta);

	xtcaptip=(l/2+l2)*cos(theta);
	ytcaptip=(l/2+l2)*sin(theta);
	xtcapend=c/2*cos(phi+theta);
	ytcapend=c/2*sin(phi+theta);

	xbcaptip=(l/2+l2)*cos(theta);
	ybcaptip=(l/2+l2)*sin(theta);
	xbcapend=-c/2*cos(pi-phi+theta);
	ybcapend=-c/2*sin(pi-phi+theta);

	xendtop=-(c/2)*cos(phi-theta);
	yendtop=(c/2)*sin(phi-theta);
	xendbot=-(l/2)*cos(theta);
	yendbot=-(l/2)*sin(theta);
	
	xdiatop=-(l/2)*cos(theta);
	ydiatop=-(l/2)*sin(theta);
	xdiabot=(c2/2)*cos(pi-phi2-theta);
	ydiabot=-(c2/2)*sin(pi-phi2-theta);

	xwlbot=-(c3/2)*cos(phi3-theta);
	ywlbot=(c3/2)*sin(phi3-theta);
	xwltop=-(c3/2)*cos(phi3-theta)-h2*sin(theta);
	ywltop=(c3/2)*sin(phi3-theta)+h2*cos(theta);

	xwrbot=-(c4/2)*cos(phi4-theta);
	ywrbot=(c4/2)*sin(phi4-theta);
	xwrtop=-(c4/2)*cos(phi4-theta)-c5*sin((0.6+pi)/2-phi5+theta);
	ywrtop=(c4/2)*sin(phi4-theta)+c5*cos((0.6+pi)/2-phi5+theta);

	xwtopr=-(c4/2)*cos(phi4-theta)-c5*sin((0.6+pi)/2-phi5+theta);
	ywtopr=(c4/2)*sin(phi4-theta)+c5*cos((0.6+pi)/2-phi5+theta);
	xwtopl=-(c3/2)*cos(phi3-theta)-h2*sin(theta);
	ywtopl=(c3/2)*sin(phi3-theta)+h2*cos(theta);
	
	elevxl = -(l/3)*cos(theta)-0.12*cos(deltae);
	elevxr = -(l/3)*cos(theta);
	elevyl = -(l/3)*sin(theta)-0.12*sin(deltae);
	elevyr = -(l/3)*sin(theta);
  	
%Center of gravity represented by a dot
  	radius=0.005;
	arcstep = 18; % angle increment (in degrees)
	j = 0:arcstep:(360-arcstep); % create ball sides using arclengths
	arcx = radius * cos((j+arcstep) * pi/180); % x coordinates of ball sides
	arcy = radius * sin((j+arcstep) * pi/180); % y coordinates of ball sides
    
%Plotting the initial position of the plane%
	axes(handles.axes2)
	cla
	K = plot([xtopnose xtoptail], [ytopnose ytoptail], 'k',...
	 	'LineWidth',[2]);
	hold on
	
	L = plot([xbotnose xbottail], [ybotnose ybottail],...
		'k','LineWidth',[2]);
    hold on

	M = plot([xtcaptip xtcapend], [ytcaptip ytcapend],'k',...
		'LineWidth',[2]);
    hold on
	
	N = plot([xbcaptip xbcapend],[ybcaptip ybcapend],'k',...
		'LineWidth',[2]);
    hold on

	O = plot([xendtop xendbot],[yendtop yendbot],'k',...
		'LineWidth',[2]);
    hold on
		
	O2 = plot([elevxl elevxr],[elevyl elevyr],'r',...
		'LineWidth',[3]);	
    hold on
	
	P = plot([xdiatop xdiabot],[ydiatop ydiabot],'k',...
		'LineWidth',[2]);	
    hold on
	
	Q = plot([xwrtop xwrbot],[ywrtop ywrbot],'k',...
		'LineWidth',[2]);
    hold on
	
	R = plot([xwltop xwlbot],[ywltop ywlbot],'k',...
		'LineWidth',[2]);
    hold on
		
	S = plot([xwtopr xwtopl],[ywtopr ywtopl],'k',...
		'LineWidth',[2]);
    hold on
	
	H = patch(arcx, arcy,'b');
    hold on
	
	hold off
	axis([-0.55 0.55 -0.55 0.55])
	title ('Animation')
    
%Clear the step response plot%	
	axes(handles.axes1)
	cla
	axis ([0 10 0 0.25])
	ylabel ('Pitch angle (rad)')
	xlabel ('Time (sec)')
	title ('Step Response')
guidata(hObject, handles);

% --- Executes on slider movement.
function stepslider_Callback(hObject, eventdata, handles)
% hObject    handle to stepslider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

stepval=get(handles.stepslider,'value');
set(handles.text2,'string',sprintf('%6.4f',stepval));
        
guidata(hObject, handles);

% --- Executes during object creation, after setting all properties.
function stepslider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to stepslider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in manualbox.
function manualbox_Callback(hObject, eventdata, handles)
% hObject    handle to manualbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of manualbox


% --- Executes on button press in reference.
function reference_Callback(hObject, eventdata, handles)
% hObject    handle to reference (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of reference


% --- Executes on button press in plotbox.
function plotbox_Callback(hObject, eventdata, handles)
% hObject    handle to plotbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of plotbox



function wfactor_Callback(hObject, eventdata, handles)
% hObject    handle to wfactor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of wfactor as text
%        str2double(get(hObject,'String')) returns contents of wfactor as a double


% --- Executes during object creation, after setting all properties.
function wfactor_CreateFcn(hObject, eventdata, handles)
% hObject    handle to wfactor (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function text2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called



% --- Executes on button press in Exit.
function Exit_Callback(hObject, eventdata, handles)
% hObject    handle to Exit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close(pitchgui);


% --- Executes during object creation, after setting all properties.
function Exit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Exit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
