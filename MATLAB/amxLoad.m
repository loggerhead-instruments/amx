% amxLoad
% loads AMX files into matrices
% modified 7/26/2016

% Surface pressure estimate in mbar for depth calculation
% depth calculation assumes 1 bar = 10 m
surfacepress=1010;

%ans=input('Append? 0=No  1=Yes ');
ans=0;

%% Uncomment to load one file at a time using dialog box
[FileName,PathName,FilterIndex] = uigetfile({'*.amx','AMX files (*.amx)'},'Select an AMX file');
if isequal(FileName,0)|isequal(PathName,0)
   return
end
    
FileName
cd(PathName);

%% Load file
[DF_HEAD, SID_SPEC, SID_REC]=oAMX(FileName);

if(ans==0)
    amxdata1=[];
    amxdata2=[];
    amxdata3=[];
    amxdata4=[];
    amxdata5=[];
end

INER=[];
INER.accel=[];INER.mag=[];INER.gyro=[];
ADC=[];
PTMP=[];
INER_ts=[];
PTMP_ts=[];

for x=1:length(SID_REC)
    cur_sid=(SID_REC(x).nSID) + 1;
    if(cur_sid==1)
        amxdata1=vertcat(amxdata1,SID_REC(x).data);
    end
    if(cur_sid==2)
        amxdata2=vertcat(amxdata2,SID_REC(x).data);
    end
    if(cur_sid==3)
        amxdata3=vertcat(amxdata3,SID_REC(x).data);
    end
    if(cur_sid==4)
        amxdata4=vertcat(amxdata4,SID_REC(x).data);
    end
        if(cur_sid==5)
        amxdata5=vertcat(amxdata5,SID_REC(x).data);
    end
end