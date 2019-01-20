

delta_time = 0.01

### Postion Filter Logic
def update_pitch(gyro):
    pitch += gyro.y * delta_time

def clean_imu(raw_imu, bearing, pitch):
    north_acc = raw_imu.x * cos(pitch) * sin(90 - bearing)
    east_acc = raw_imu.x * cos(pitch) * cos(90 - bearing)
    z_acc = raw_imu.x * sin(pitch)
    return imu(north_acc, east_acc, z_acc)

def decompose_ground_speed(ground_speed, bearing):
    vel_x = ground_speed * cos(90 - bearing)
    vel_y = ground_speed * sin(90 - bearing)
    return velocity(vel_x, vel_y, 0)

def finalize_velocity(imu, old_velocity, ground_speed):
    vel_North = imu_weight * (old_velocity.x + imu.x * delta_time) + ground_speed_weight * ground_speed.x
    vel_East = imu_weight * (old_velocity.y + imu.y * delta_time) + ground_speed_weight * ground_speed.y
    vel_z = imu_weight * (old_velocity.z + imu.z * delta_time)
    return velocity(vel_North, vel_East, vel_z)

def finalize_position(old_position, velocity, gps):
    lat_minutes = old_position.lat.minutes + velocity.north * delta_time * meters_to_latitude_minutes
    lat_degrees += velocity_weight * (lat_minutes // 60) + gps_weight * gps.lat.degrees
    lat_minutes = velocity_weight * (lat_minutes % 60) + gps_weight * gps.lat.minutes
    lon_minutes = old_position.lat.minutes + velocity.north * delta_time * meters_to_longitude_minutes
    lon_degrees += velocity_weight * (lon_minutes // 60) + gps_weight * gps.lon.degrees
    lon_minutes = velocity_weight * (lon_minutes % 60) + gps_weight * gps.lon.minutes
    return position(lat_degrees, lat_minutes, lon_degrees, lon_minutes)

### Bearing Filter Logic

def stationary_bearing(old_bearing, magnetometer):
    return (old_bearing + magnetometer) / 2

def turning_bearing(old_bearing, gyro, magnetometer):
    return gyro_weight * (old_bearing + gyro.y * delta_time) + magnetometer_weight * magnetometer

def moving_bearing(track_angle, magnetometer):
    return track_angle_weight * track_angle + magnetometer_weight * magnetometer

def finalize_bearing(state, magnetometer, gyro, track_angle, old_bearing):
    switch = {
        'stationary': stationary_bearing,
        'turning': turning_bearing,
        'moving': moving_bearing
    }

    args = {
        'stationary': [old_bearing, magnetometer],
        'turning': [old_bearing, gyro, magnetometer],
        'moving': [track_angle, magnetometer]
    }
    return switch.get(state, lambda *a: None)(*args.get(state, None))


