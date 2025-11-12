function pos_ecef = nedPos2ECEF(ned_pos, R_ET, init_ecef)

    pos_ecef = R_ET * ned_pos + init_ecef;

end