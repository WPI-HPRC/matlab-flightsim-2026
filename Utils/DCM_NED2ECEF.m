function R_ET = DCM_NED2ECEF(lat, lon)

    R_ET = [
        -sind(lat)*cosd(lon), -sind(lon), -cosd(lat)*cosd(lon);
        -sind(lat)*sind(lon), cosd(lon), -cosd(lat)*sind(lon);
        cosd(lat), 0, -sind(lat);
    ];
    

end