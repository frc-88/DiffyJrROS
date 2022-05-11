function varargout = invgui(varargin)
%%**************************************************************%
%This functions is intended for use the invgui.m file.  It      %
%contains the code necessary to run the GUI's simulations and   %
%controls.                                                      %
%                                                               %
%Copyright (C) 1997 by the Regents of the University of         %
%Michigan.                                                      %
%Modified by Asst. Prof. Rick Hill (U Detroit-Mercy) and his    %
%student Li Guan.                                               %
%**************************************************************%
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @invgui_OpeningFcn, ...
                   'gui_OutputFcn',  @invgui_OutputFcn, ...
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
 
 
% --- Executes just before invgui is made visible.
function invgui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to invgui (see VARARGIN)
 
% Choose default command line output for invgui
handles.output = hObject;
 
% Update handles structure
guidata(hObject, handles);
 
% UIWAIT makes invgui wait for user response (see UIRESUME)
% uiwait(handles.figure1);
 
 
% --- Outputs from this function are returned to the command line.
function varargout = invgui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 
% Get default command line output from handles structure
varargout{1} = handles.output;
 
 
% --- Executes on button press in Run.
function Run_Callback(hObject, eventdata, handles)
% hObject    handle to Run (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global K
global cartpos
global pendangl
global T
global Nbar
global stepval
% Nbar data stored in the Run.UserData
     A = [0   1.0000        0        0;
          0  -0.1818   2.6727        0;
          0        0        0   1.0000;
          0  -0.4545  31.1818        0];
     B = [0; 1.8182; 0; 4.5455];
     C = [1 0 0 0];
     D = [0];
%Get the weighing factors from the editable text fields%
    x=str2num(get(handles.xtext,'string'));    
    y=str2num(get(handles.ytext,'String'));
       
      Q=[x 0 0 0;
         0 0 0 0;
         0 0 y 0;
         0 0 0 0];
      R = 1;
 
%Find the K matrix with the lqr command%      
      K = lqr(A,B,Q,R);
      Ac = [(A-B*K)];
      Bc = [B];
      Cc = [C];
      Dc = [D];
%Check if the refernece input is selected%      
      Nbarval = get(handles.reference,'Value');
      if Nbarval == 0
        Nbar = 1;
        set(handles.Run,'UserData',Nbar);
        stepaxis=stepval/1000; 
      elseif Nbarval == 1
        s = size(A,1);
        Z = [zeros([1,s]) 1];
        N = inv([A,B;C,D])*Z';
        Nx = N(1:s);
        Nu = N(1+s);
        Nbar = Nu + K*Nx;
        set(handles.Run,'UserData',Nbar);
      end        
       
%Get the value of the step input from the step slider%
      stepval=get(handles.stepslider,'Value');
      
%Check whether linear or non-linear system is to be run%
      sysval = get(handles.syscheckbox,'Value');
     
      if sysval == 0                
        T=0:0.1:6;      
        U=stepval*ones(size(T));
        [Y,X]=lsim(Ac,Nbar*Bc,Cc,Dc,U,T);
        cartpos=X(:,1);
        pendangl=X(:,3);      
      else 
        x0=[0 0 0 0];
%Check version of Matlab%  
        v=version;
     
    if eval(v(1))>=5
           tspan=[0 6];
           %options=odeset('Refine',1,'RelTol',1e-2,'AbsTol',1e-5);
           'Please wait while simulation is running'
           [T,X]=ode45('invODE',tspan,x0);
        else
           'Please wait while simulation is running'
           [T,X]=ode45('invODE',0,6,x0); 
        end
     
        cartpos=X(:,1);
        pendangl=X(:,3);
       
      end    
%Pendulum and cart data%    
      cart_length=0.3;
      cl2=cart_length/2;
       
      ltime=length(cartpos);
       
      cartl=cartpos-cl2;
      cartr=cartpos+cl2;
       
      pendang=-pendangl;
      pendl=0.6;
     
      pendx=pendl*sin(pendang)+cartpos;
      pendy=pendl*cos(pendang)+0.03;
       
%Check if the step response and animation are to be plotted separately%            
      plotval=get(handles.plotbox,'Value');   
      if plotval == 1
         axes(handles.axes1)
         plot(T,cartpos,'r')
         plot(T,pendangl,'b')
      elseif plotval == 0
         axes(handles.axes1)
         plot(T(1),cartpos(1), 'r')
         hold on
         plot(T(1),pendangl(1), 'b')
         hold on
      end
       
%Set the axis for the step response plot%   
      axes(handles.axes1)
      if stepval > 0
        axis([0 6 -stepval/2 stepval*2])
      elseif stepval < 0
        axis([0 6 stepval*2 -stepval/2])
      else
        axis([0 6 -0.5 0.5])
      end
       
      title(sprintf('Step Response to %0.4f cm input',stepval))
      xlabel('Time (sec)')
       
      hold on
 %Plot the first frame of the animation%          
      axes(handles.axes2)
      cla
      L = plot([cartpos(1) pendx(1)], [0.03 pendy(1)], 'b',...
      'LineWidth',7);
  drawnow
      hold on
      J = plot([cartl(1) cartr(1)], [0 0], 'r',...
      'LineWidth',20);  
  drawnow
       
      axis([-.7 0.7 -0.1 0.7])
      title('Inverted Pendulum Animation')
      xlabel('X Position (m)')
      ylabel('Y Position (m)')
       
%Check if the animation is to be advanced manually%      
      manual=get(handles.manualbox,'Value');   
 
%Run the animation%      
    for i = 2:ltime-1
         if manual == 1
              pause
         end
           
         set(J,'XData', [cartl(i) cartr(i)]);
         set(L,'XData', [cartpos(i) pendx(i)]);
         set(L,'YData', [0.03 pendy(i)]); 
         drawnow;
          
         if plotval == 0    
             axes(handles.axes1)
             plot([T(i),T(i+1)],[cartpos(i),cartpos(i+1)])
             
             hold on
             plot([T(i),T(i+1)],[pendangl(i),pendangl(i+1)])
             hold on
             
             
         end
     end
%Add legend to step plot%
    axes(handles.axes1)
    hold
    legend('Pendulum Angle (rad.)','Cart Position (cm.)')
 
guidata(hObject, handles);    
     
     
function xtext_Callback(hObject, eventdata, handles)
% hObject    handle to xtext (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of xtext as text
%        str2double(get(hObject,'String')) returns contents of xtext as a double
 
 
% --- Executes during object creation, after setting all properties.
function xtext_CreateFcn(hObject, eventdata, handles)
% hObject    handle to xtext (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
 
 
 
function ytext_Callback(hObject, eventdata, handles)
% hObject    handle to ytext (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'String') returns contents of ytext as text
%        str2double(get(hObject,'String')) returns contents of ytext as a double
 
 
% --- Executes during object creation, after setting all properties.
function ytext_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ytext (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
 
 
% --- Executes on button press in reference.
function reference_Callback(hObject, eventdata, handles)
% hObject    handle to reference (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hint: get(hObject,'Value') returns toggle state of reference
 
 
% --- Executes during object creation, after setting all properties.
function reference_CreateFcn(hObject, eventdata, handles)
% hObject    handle to reference (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
 
 
% --- Executes on slider movement.
function stepslider_Callback(hObject, eventdata, handles)
% hObject    handle to stepslider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
%Get the value of the step input from the step slider%
      stepval=get(hObject,'Value');
      set(handles.text5,'string',sprintf('%6.4f',stepval));
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
 
 
% --- Executes during object creation, after setting all properties.
function text5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
 
 
% --- Executes on button press in syscheckbox.
function syscheckbox_Callback(hObject, eventdata, handles)
% hObject    handle to syscheckbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 
% Hint: get(hObject,'Value') returns toggle state of syscheckbox
 
 
% --- Executes during object creation, after setting all properties.
function syscheckbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to syscheckbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
 
 
% --- Executes on button press in plotbox.
function plotbox_Callback(hObject, eventdata, handles)
% hObject    handle to plotbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 
% Hint: get(hObject,'Value') returns toggle state of plotbox
 
 
% --- Executes during object creation, after setting all properties.
function plotbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to plotbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
 
 
% --- Executes on button press in manualbox.
function manualbox_Callback(hObject, eventdata, handles)
% hObject    handle to manualbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 
% Hint: get(hObject,'Value') returns toggle state of manualbox
 
 
% --- Executes during object creation, after setting all properties.
function manualbox_CreateFcn(hObject, eventdata, handles)
% hObject    handle to manualbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
 
 
% --- Executes on button press in clear.
function clear_Callback(hObject, eventdata, handles)
% hObject    handle to clear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
 
%Callback for the RESET button%
    axes(handles.axes1)
    cla
    axis([0 6 -0.5 0.5])
    title('Step Response')
    xlabel('Time (sec)')
     
    axes(handles.axes2)
    cartpos=0;
    cart_length=0.3;
    cl2=cart_length/2;
       
    cartl=cartpos-cl2;
    cartr=cartpos+cl2;
       
    pendang=0;
    pendl=0.6;
     
    pendx=pendl*sin(pendang)+cartpos;
    pendy=pendl*cos(pendang)+0.03;
    cla
    K = plot([cartpos(1) pendx(1)], [0.03 pendy(1)], 'b',...
    'LineWidth',[7]);
    drawnow
    hold on
    J = plot([cartl(1) cartr(1)], [0 0], 'r',...
    'LineWidth',[20]);  
    drawnow
   guidata(hObject, handles);  
 
 
% --- Executes on button press in repeat.
function repeat_Callback(hObject, eventdata, handles)
% hObject    handle to repeat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global cartpos
global pendangl
global T
      
    cart_length=0.3;
    cl2=cart_length/2;
       
    ltime=length(cartpos);
       
    cartl=cartpos-cl2;
    cartr=cartpos+cl2;
       
    pendang=-pendangl;
    pendl=0.6;
     
    pendx=pendl*sin(pendang)+cartpos;
    pendy=pendl*cos(pendang)+0.03;
       
     plotval=get(handles.plotbox,'Value');
    if plotval == 1
       axes(handles.axes1);
       plot(T,cartpos,'r')
       plot(T,pendangl,'b')
       legend('Cart','Pendulum')
    elseif plotval == 0
       axes(handles.axes1);
       plot(T(1),cartpos(1), 'r')
       hold on
       plot(T(1),pendangl(1), 'b')
       hold on
    end
       
    stepval=get(handles.stepslider,'Value');
    Nbarval=get(handles.reference,'Value');
    if Nbarval == 1
      stepaxis=stepval;
    else
      stepaxis=stepval/1000;   
    end
       
    if stepval > 0
      axis([0 6 -stepval/2 stepval*2])
    elseif stepval < 0
      axis([0 6 stepval*2 -stepval/2])
    else
      axis([0 6 -0.5 0.5])
    end
       
    title(sprintf('Step Response to %0.4f cm input',stepval))
    xlabel('Time (sec)')
       
       
    hold on
           
    axes(handles.axes2)
    cla
    L = plot([cartpos(1) pendx(1)], [0.03 pendy(1)], 'b',...
    'LineWidth',[7]);
    drawnow
    hold on
    J = plot([cartl(1) cartr(1)], [0 0], 'r',...
    'LineWidth',[20]);  
    drawnow
     
    axis([-.7 0.7 -0.1 0.7])
    title('Inverted Pendulum Animation')
    xlabel('X Position (m)')
    ylabel('Y Position (m)')
      
    manual=get(handles.manualbox,'Value');
       
    for i = 2:ltime-1,
       if manual == 1
            pause
       end
          
       set(J,'XData', [cartl(i) cartr(i)]);
       set(L,'XData', [cartpos(i) pendx(i)]);
       set(L,'YData', [0.03 pendy(i)]); 
       drawnow;
          
       if plotval == 0  
           axes(handles.axes1)
           plot([T(i),T(i+1)],[cartpos(i),cartpos(i+1)], 'r')
           hold on
           plot([T(i),T(i+1)],[pendangl(i),pendangl(i+1)], 'b')
           hold on
       end
   end
guidata(hObject, handles);
 
 
% --- Executes during object creation, after setting all properties.
function text1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to text1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
 
 
% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
 
% Hint: place code in OpeningFcn to populate axes1
 
 
% --- Executes during object creation, after setting all properties.
function axes2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
 
% Hint: place code in OpeningFcn to populate axes2
 
 
% --- Executes on button press in exit.
function exit_Callback(hObject, eventdata, handles)
% hObject    handle to exit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close(invgui)