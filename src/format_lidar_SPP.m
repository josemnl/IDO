function z_t = format_lidar_SPP(z_t, range)
    ranges = z_t.Ranges;
    ranges(isnan(ranges) | ranges==0) = range;
    z_t = lidarScan(ranges, z_t.Angles);
end