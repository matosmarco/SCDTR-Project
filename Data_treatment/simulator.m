% Course page simulator
%initialization
tau_u = 0.02;
tau_d = 0.03;
R = 10000;
C = 0.00001;
g = 0.8;
R0 = 200000/(10^(-g));
Vcc = 3.3;
T = 0.001;
step = 1000;
x = 1;
r(1) = R0*x^(-g);
v(1) = Vcc*R/(R+r(1));

%Cycle with increments of 10 Lux
for( loop = 0:9 )
    x = x+10;
    for (i = loop*step+2:(loop+1)*step+1)
        r(i) = r(i-1)+T*(-r(i-1)/tau_u+R0*x^(-g)/tau_u);
        v(i) = v(i-1)+T*( (Vcc-v(i-1))/C/r(i-1) - v(i-1)/R/C );
    end
end;
%cycle with decrements of 10 lux
for( loop = 10:19 )
    x = x-10;
    for (i = loop*step+2:(loop+1)*step+1)
        r(i) = r(i-1)+T*(-r(i-1)/tau_d+R0*x^(-g)/tau_d);
        v(i) = v(i-1)+T*( (Vcc-v(i-1))/C/r(i-1) - v(i-1)/R/C );
    end
end;


%plot
t=(1:(loop+1)*step+1)*T;
figure(1);
plot(t ,v);
title('ADC Voltage');
xlabel('t [sec]');
ylabel('v [volt]');
grid;