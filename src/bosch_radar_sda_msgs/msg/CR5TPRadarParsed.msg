int32 crc                           # crc16 validate 
int32 counter                       # crc16 validate counter
float64 timestamp                   # not implemented
float64 long_vel_var                # not implemented
float64 long_pos_var                # not implemented
float64 long_ext_front_var          # not implemented
float64 long_ext_back_var           # not implemented
float64 long_acc_var                # not implemented
float64 lat_vel_var                 # not implemented 
float64 lat_pos_var                 # not implemented
float64 lat_ext_right_var           # not implemented
float64 lat_ext_left_var            # not implemented
float64 lat_acc_var                 # not implemented
float64 heading_var                 # not implemented
float64 life_time                   # the life time of object
float64 height                      # object heigh
float64 heading                     # object heading angle
float64 coast_index                 # cycles of how long the object was not seen but measured in current cycle is mapped
int32 obj_id                        # object id
int32 obj_dyn_class                 # to indicate if the object is movable or stationary
float64 obj_class_conf              # not implented
int32 obj_class                     # internal calculated
float64 long_acc                    # longitude acceleration to the radar; (+) = left; unit = m/s2 
float64 lat_ext_right               # not implemented
int32 track_status                  # 0:measured 1:predicted 2:tracked 3:invalid 4:lost 5:spare 6:error 7: not available
float64 long_vel                    # longitude velocity to the radar; (+) = left; unit = m/s
float64 lat_vel                     # lateral velocity to the radar; (+) = left; unit = m/s
float64 lat_pos                     # lateral position
float64 long_pos                    # longitude position
float64 long_ext_front              # not implemented
float64 long_ext_back               # not implemented
float64 lat_acc                     # lateral acceleration to the radar; (+) = left; unit = m/s2 
float64 lat_ext_left                # not implemented
float64 exist_conf                  # not implemented
