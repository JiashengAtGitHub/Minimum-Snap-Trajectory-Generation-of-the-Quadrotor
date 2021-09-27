%% User Panel
% Notice
% length(times)-1=length(DOF_a_segments)=length(DOF_b_segments)=length(DOF_c_segments)=length(DOF_d_segments)
times=[0 2 4 6]; % time at each waypoint
waypoints=[[0 0 0]',[1 1 1]',[1 1 0]',[0 0 0]'];
psi_IC=0; % specify initial value of desired psi
DOF_a_piecewise=[8 8 8];  % specify the degree of freedom of the 1st,2nd,...,mth segment of vector a , respectively.
DOF_b_piecewise=[8 8 8];  % specify the degree of freedom of the 1st,2nd,...,mth segment of vector b , respectively.
DOF_c_piecewise=[8 8 8];  % specify the degree of freedom of the 1st,2nd,...,mth segment of vector c , respectively.
DOF_d_piecewise=[3 3 3];  % specify the degree of freedom of the 1st,2nd,...,mth segment of vector d , respectively. 
%% initialize variables
syms t
Px=zeros(sum(DOF_a_piecewise));
Ax=zeros(6*length(times)-2,sum(DOF_a_piecewise)); % initialize Ax. Note the ultimate height will be 6*length(times)-6
bx=zeros(6*length(times)-6,1);
Py=zeros(sum(DOF_b_piecewise));
Ay=zeros(6*length(times)-2,sum(DOF_b_piecewise)); % initialize Ay. Note the ultimate height will be 6*length(times)-6
by=zeros(6*length(times)-6,1);
Pz=zeros(sum(DOF_c_piecewise));
Az=zeros(6*length(times)-2,sum(DOF_c_piecewise)); % initialize Az. Note the ultimate height will be 6*length(times)-6
bz=zeros(6*length(times)-6,1);
Ppsi=zeros(sum(DOF_d_piecewise));
Apsi=zeros(3*length(times),sum(DOF_d_piecewise)); % initialize Apsi. Note the ultimate height will be 3*length(times)-3
bpsi=zeros(3*length(times)-3,1);
%% compute Qx,Ax,bx,Qy,Ay,by,Qz,Az,bz,Qpsi,Apsi,bpsi
for i=1:(length(times)-1)
%--------------------------------------------------------------------------
tx=1;
for j=1:(DOF_a_piecewise(i)-1)   % I might be able to use ployval(),polyder() or sth to simplify the code, but now I'm too busy
    tx=[tx t^j];
end % tx=[1, t, t^2, ... ,t^n],a=[a0 a1 ... an],x=tx*a', where n=length_of_a(i)-1
tx_dot=diff(tx,t);
tx_dot2=diff(tx_dot,t);
tx_dot3=diff(tx_dot2,t);
tx_dot4=diff(tx_dot3,t);
Px_of_t_piece=tx_dot4'*tx_dot4;
Px(sum(DOF_a_piecewise(1:i))-DOF_a_piecewise(i)+1:sum(DOF_a_piecewise(1:i)),sum(DOF_a_piecewise(1:i))-DOF_a_piecewise(i)+1:sum(DOF_a_piecewise(1:i)))=double(int(Px_of_t_piece,t,times(i),times(i+1)));
Ax(2*i-1:2*i,sum(DOF_a_piecewise(1:i))-DOF_a_piecewise(i)+1:sum(DOF_a_piecewise(1:i)))=[subs(tx,times(i));
                                                                                        subs(tx,times(i+1))];
Ax(2*length(times)+4*i-5:2*length(times)+4*i+2,sum(DOF_a_piecewise(1:i))-DOF_a_piecewise(i)+1:sum(DOF_a_piecewise(1:i)))=[subs(tx_dot,times(i));
                                                                                                                          subs(tx_dot2,times(i));
                                                                                                                          subs(tx_dot3,times(i));
                                                                                                                          subs(tx_dot4,times(i));
                                                                                                                         -subs(tx_dot,times(i+1));
                                                                                                                         -subs(tx_dot2,times(i+1));
                                                                                                                         -subs(tx_dot3,times(i+1));
                                                                                                                         -subs(tx_dot4,times(i+1))];
bx(2*i-1:2*i)=[waypoints(1,i);waypoints(1,i+1)];
%--------------------------------------------------------------------------
ty=1;
for j=1:(DOF_b_piecewise(i)-1)  
    ty=[ty t^j];
end % ty=[1, t, t^2, ... ,t^n],b=[b0 b1 ... bn],y=ty*b',where n=length_of_b(i)-1
ty_dot=diff(ty,t);
ty_dot2=diff(ty_dot,t);
ty_dot3=diff(ty_dot2,t);
ty_dot4=diff(ty_dot3,t);
Py_of_t_piece=ty_dot4'*ty_dot4;
Py(sum(DOF_b_piecewise(1:i))-DOF_b_piecewise(i)+1:sum(DOF_b_piecewise(1:i)),sum(DOF_b_piecewise(1:i))-DOF_b_piecewise(i)+1:sum(DOF_b_piecewise(1:i)))=double(int(Py_of_t_piece,t,times(i),times(i+1)));
Ay(2*i-1:2*i,sum(DOF_b_piecewise(1:i))-DOF_b_piecewise(i)+1:sum(DOF_b_piecewise(1:i)))=[subs(ty,times(i));
                                                                                        subs(ty,times(i+1))];
Ay(2*length(times)+4*i-5:2*length(times)+4*i+2,sum(DOF_b_piecewise(1:i))-DOF_b_piecewise(i)+1:sum(DOF_b_piecewise(1:i)))=[subs(ty_dot,times(i));
                                                                                                                          subs(ty_dot2,times(i));
                                                                                                                          subs(ty_dot3,times(i));
                                                                                                                          subs(ty_dot4,times(i));
                                                                                                                         -subs(ty_dot,times(i+1));
                                                                                                                         -subs(ty_dot2,times(i+1));
                                                                                                                         -subs(ty_dot3,times(i+1));
                                                                                                                         -subs(ty_dot4,times(i+1))];
by(2*i-1:2*i)=[waypoints(2,i);waypoints(2,i+1)];
%--------------------------------------------------------------------------
tz=1;
for j=1:(DOF_c_piecewise(i)-1)  
    tz=[tz t^j];
end % tz=[1, t, t^2, ... ,t^n],c=[c0 c1 ... cn],z=tz*b',where n=length_of_c(i)-1
tz_dot=diff(tz,t);
tz_dot2=diff(tz_dot,t);
tz_dot3=diff(tz_dot2,t);
tz_dot4=diff(tz_dot3,t);
Pz_of_t_piece=tz_dot4'*tz_dot4;
Pz(sum(DOF_c_piecewise(1:i))-DOF_c_piecewise(i)+1:sum(DOF_c_piecewise(1:i)),sum(DOF_c_piecewise(1:i))-DOF_c_piecewise(i)+1:sum(DOF_c_piecewise(1:i)))=double(int(Pz_of_t_piece,t,times(i),times(i+1)));
Az(2*i-1:2*i,sum(DOF_c_piecewise(1:i))-DOF_c_piecewise(i)+1:sum(DOF_c_piecewise(1:i)))=[subs(tz,times(i));
                                                                                        subs(tz,times(i+1))];
Az(2*length(times)+4*i-5:2*length(times)+4*i+2,sum(DOF_c_piecewise(1:i))-DOF_c_piecewise(i)+1:sum(DOF_c_piecewise(1:i)))=[subs(tz_dot,times(i));
                                                                                                                          subs(tz_dot2,times(i));
                                                                                                                          subs(tz_dot3,times(i));
                                                                                                                          subs(tz_dot4,times(i));
                                                                                                                         -subs(tz_dot,times(i+1));
                                                                                                                         -subs(tz_dot2,times(i+1));
                                                                                                                         -subs(tz_dot3,times(i+1));
                                                                                                                         -subs(tz_dot4,times(i+1))];
bz(2*i-1:2*i)=[waypoints(3,i);waypoints(3,i+1)];
%--------------------------------------------------------------------------
tpsi=1;
for j=1:(DOF_d_piecewise(i)-1)
    tpsi=[tpsi t^i];
end % tpsi=[1, t, t^2, ... ,t^n],d=[d0 d1 ... dn],psi=tpsi*d',where n=length_of_d(i)-1
tpsi_dot=diff(tpsi,t);
tpsi_dot2=diff(tpsi_dot,t);
Ppsi_of_t_piece=tpsi_dot2'*tpsi_dot2;
Ppsi(sum(DOF_d_piecewise(1:i))-DOF_d_piecewise(i)+1:sum(DOF_d_piecewise(1:i)),sum(DOF_d_piecewise(1:i))-DOF_d_piecewise(i)+1:sum(DOF_d_piecewise(1:i)))=double(int(Ppsi_of_t_piece,t,times(i),times(i+1)));
Apsi(3*i-2:3*i+3,sum(DOF_d_piecewise(1:i))-DOF_d_piecewise(i)+1:sum(DOF_d_piecewise(1:i)))=[subs(tpsi,times(i));
                                                                                            subs(tpsi_dot,times(i));
                                                                                            subs(tpsi_dot2,times(i));
                                                                                           -subs(tpsi,times(i+1));
                                                                                           -subs(tpsi_dot,times(i+1));
                                                                                           -subs(tpsi_dot2,times(i+1))];
%--------------------------------------------------------------------------                                                                                       
end
Ax=Ax(1:end-4,:);
Ay=Ay(1:end-4,:);
Az=Az(1:end-4,:);
Apsi=Apsi(1:end-3,:);
bpsi(1)=psi_IC;
%% 4 optimization problems
cvx_begin
variable a(sum(DOF_a_piecewise))
minimize(a'*Px*a)
Ax*a==bx;
cvx_end
opt_status1=cvx_status;
opt_value1=a'*Px*a;
%--------------------------------------------------------------------------
cvx_begin
variable b(sum(DOF_b_piecewise))
minimize(b'*Py*b)
Ay*b==by;
cvx_end
opt_status2=cvx_status;
opt_value2=b'*Py*b;
%--------------------------------------------------------------------------
cvx_begin
variable c(sum(DOF_c_piecewise))
minimize(c'*Pz*c)
Az*c==bz;
cvx_end
opt_status3=cvx_status;
opt_value3=c'*Pz*c;
%--------------------------------------------------------------------------
cvx_begin
variable d(sum(DOF_d_piecewise))
minimize(d'*Ppsi*d)
Apsi*d==bpsi;
cvx_end
opt_status4=cvx_status;
opt_value4=d'*Ppsi*d;
%% make piecewise expression of a,b,c,d, which are matrices
a_piecewise=zeros(max(DOF_a_piecewise),length(DOF_a_piecewise));
b_piecewise=zeros(max(DOF_b_piecewise),length(DOF_b_piecewise));
c_piecewise=zeros(max(DOF_c_piecewise),length(DOF_c_piecewise));
d_piecewise=zeros(max(DOF_d_piecewise),length(DOF_d_piecewise));
for i=1:(length(times)-1)
a_piecewise(1:DOF_a_piecewise(i),i)=a(sum(DOF_a_piecewise(1:i))-DOF_a_piecewise(i)+1:sum(DOF_a_piecewise(1:i)));
b_piecewise(1:DOF_b_piecewise(i),i)=b(sum(DOF_b_piecewise(1:i))-DOF_b_piecewise(i)+1:sum(DOF_b_piecewise(1:i)));
c_piecewise(1:DOF_c_piecewise(i),i)=c(sum(DOF_c_piecewise(1:i))-DOF_c_piecewise(i)+1:sum(DOF_c_piecewise(1:i)));
d_piecewise(1:DOF_d_piecewise(i),i)=d(sum(DOF_d_piecewise(1:i))-DOF_d_piecewise(i)+1:sum(DOF_d_piecewise(1:i)));
end

