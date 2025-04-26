C:

cd C:\Program Files\FlightGear

SET FG_ROOT=C:\Program Files\FlightGear\data

START .\\bin\fgfs.exe --fg-aircraft=.\\customAircraft\ --fdm=null --native-fdm=socket,in,30,localhost,5502,udp --native-ctrls=socket,out,30,localhost,5505,udp --enable-terrasync --prop:/sim/rendering/shaders/quality-level=0 --aircraft=HPMR --fog-fastest --disable-clouds --start-date-lat=2004:06:01:09:00:00 --disable-sound --in-air --airport=KORH --runway=29 --altitude=7224 --heading=113 --offset-distance=4.72 --offset-azimuth=0  
