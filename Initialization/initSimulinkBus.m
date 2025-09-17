function initSimulinkBus(params)

%% Navigator Bus
navParams = params.navParams;

N = length(navParams.x);

elems(1) = Simulink.BusElement;
elems(1).Name = 'x';
elems(1).Dimensions = [N 1];


elems(2) = Simulink.BusElement;
elems(2).Name = 'x_min';
elems(2).Dimensions = [N 1];

elems(3) = Simulink.BusElement;
elems(3).Name = 'Q';
elems(3).Dimensions = [N N];

elems(4) = Simulink.BusElement;
elems(4).Name = 'P';
elems(4).Dimensions = [N N];

elems(5) = Simulink.BusElement;
elems(5).Name = 'P_min';
elems(5).Dimensions = [N N];

elems(6) = Simulink.BusElement;
elems(6).Name = 'W_m';
elems(6).Dimensions = [1 (N*2+1)];

elems(7) = Simulink.BusElement;
elems(7).Name = 'W_c';
elems(7).Dimensions = [1 (N*2+1)];

navBus = Simulink.Bus;
navBus.Elements = elems;

assignin('base', 'navBus', navBus);

clear elems;
end