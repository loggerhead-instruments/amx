function [DF_HEAD, SID_SPEC, SID_REC]=oAMX(filename)
% Program to load AMX files from Loggerhead AMX
STR_MAX = 8;

fid=fopen(filename);
if(fid<1)
    disp('Unable to Open File');
    FileName
    return
end 

% Read in DF_Head
DF_HEAD.Version = fread(fid,1,'uint32');
DF_HEAD.UserID = fread(fid,1,'uint32');
DF_HEAD.Voltage = fread(fid,1,'float32');
DF_HEAD.sec = fread(fid,1,'uint8');
DF_HEAD.min = fread(fid,1,'uint8');
DF_HEAD.hour = fread(fid,1,'uint8');
DF_HEAD.day = fread(fid,1,'uint8');
DF_HEAD.month = fread(fid,1,'uint8');
fread(fid,3,'uint8'); % NU
DF_HEAD.year = fread(fid,1,'int16'); 
DF_HEAD.tzOffset = fread(fid,1,'int16');

% Read in SID_SPECS until get all zeroes
notdone=1;
SID_SPEC=[];
nSIDSPEC=1;
while(notdone)
    SID_SPEC(nSIDSPEC).SID = fread(fid,STR_MAX,'uint8=>char');
    SID_SPEC(nSIDSPEC).sidType = fread(fid, 1, 'uint16');
    fread(fid, 1, 'uint16'); %NU
    SID_SPEC(nSIDSPEC).nSamples = fread(fid,1,'uint32');
    
    SID_SPEC(nSIDSPEC).sensor.chipName = fread(fid, STR_MAX, 'uint8=>char');
    SID_SPEC(nSIDSPEC).sensor.nChan = fread(fid, 1, 'uint16');
    fread(fid, 1, 'uint16');  %NU
    SID_SPEC(nSIDSPEC).sensor.name = [];
    SID_SPEC(nSIDSPEC).sensor.units = [];
    SID_SPEC(nSIDSPEC).sensor.cal = [];
    
    for i = 1:12
       SID_SPEC(nSIDSPEC).sensor.name = [SID_SPEC(nSIDSPEC).sensor.name; fread(fid,STR_MAX,'uint8=>char')'];
    end
    for i = 1:12
       SID_SPEC(nSIDSPEC).sensor.units = [SID_SPEC(nSIDSPEC).sensor.units; fread(fid,STR_MAX,'uint8=>char')'];
    end
    for i = 1:12
       SID_SPEC(nSIDSPEC).sensor.cal = [SID_SPEC(nSIDSPEC).sensor.cal; fread(fid,1,'float32')'];
    end
    
    SID_SPEC(nSIDSPEC).DForm = fread(fid,1,'uint32');
    SID_SPEC(nSIDSPEC).srate = fread(fid,1,'float32'); 
    
    if(SID_SPEC(nSIDSPEC).nSamples==0)
        notdone=0;
    end
    nSIDSPEC=nSIDSPEC+1; 
end
nSIDSPEC=nSIDSPEC-1;
SID_SPEC(nSIDSPEC)=[];  % delete last one with all zeroes
nSIDSPEC=nSIDSPEC-1;

% Read in next SID_REC header and data
eofstat=0;
reccounter=0;
SID_REC=[]; 

while(eofstat==0)
    reccounter=reccounter+1;
    SID_REC(reccounter).nSID=fread(fid,1,'uint32');
    fread(fid,1,'uint32'); %NU
    fread(fid,1,'uint32'); %NU 
    fread(fid,1,'uint32'); %NU 

    cur_sid=(SID_REC(reccounter).nSID) + 1;
    if(cur_sid >0 & cur_sid<8)         
        if(SID_SPEC(cur_sid).DForm==2)
            if (SID_SPEC(cur_sid).SID(1)=='3')
                SID_REC(reccounter).data=fread(fid,SID_SPEC(cur_sid).nSamples,'int16', 'ieee-be');
            else
                SID_REC(reccounter).data=fread(fid,SID_SPEC(cur_sid).nSamples,'int16');
            end
        end
        if(SID_SPEC(cur_sid).DForm==5)
            SID_REC(reccounter).data=fread(fid,SID_SPEC(cur_sid).nSamples,'float32');  % 32-bit samples read in 8 bits at a time
        end        
    else
        SID_REC(reccounter)=[]; %last one was bad so delete this entry
        reccounter=reccounter-1;
    end
     
    ftell(fid);
    eofstat = feof(fid);
end

fclose(fid);
