

delta_time = 0.01

### Postion Filter Logic
def update_pitch(gyro):
    '''
    Updates the pitch angle used to correct the forward acceleration of the rover
    :param gyro: object holding the latest readings from the gyroscope
    :return:
    '''
    pitch += gyro.y * delta_time

def clean_imu(raw_imu, bearing, pitch):
    """
    Converts the imu data to absolute coordinite system used by the gps
    and corrects for the pitch angle, also updates the z_acceleration
    :param raw_imu: object holding the data from the imu
    :param bearing: degrees from due north
    :param pitch: degrees from flat plane
    :return: imu object containing the acceleration in North, East, And Z Directions
    """
    north_acc = raw_imu.x * cos(pitch) * sin(90 - bearing)
    east_acc = raw_imu.x * cos(pitch) * cos(90 - bearing)
    z_acc = raw_imu.x * sin(pitch)
    return imu(north_acc, east_acc, z_acc)

def decompose_ground_speed(ground_speed, bearing):
    """
    Breaks the groud speed velocity into components aligned with gps coordinate system
    :param ground_speed: velocity (m/s) data from the gps
    :param bearing: degrees from due north
    :return: velocity object containing the velocity in the North and East Directions
    """
    vel_East = ground_speed * cos(90 - bearing)
    vel_North = ground_speed * sin(90 - bearing)
    return velocity(vel_East, vel_North, 0)

def finalize_velocity(imu, old_velocity, ground_speed):
    """
    Combines the ground speed and velocity from the imu in a weighted average
    :param imu: cleaned imu data returned from clean_imu
    :param old_velocity: the previous final velocity value
    :param ground_speed: object containing the ground speed componenets returned from
    decompose_ground_speed
    :return: velocity object containing the velocity in the North, East, and Z directions
    """
    vel_North = imu_weight * (old_velocity.x + imu.x * delta_time) + ground_speed_weight * ground_speed.x
    vel_East = imu_weight * (old_velocity.y + imu.y * delta_time) + ground_speed_weight * ground_speed.y
    vel_z = imu_weight * (old_velocity.z + imu.z * delta_time)
    return velocity(vel_North, vel_East, vel_z)

def finalize_position(old_position, velocity, gps):
    """
    Combines the gps data with the velocity added to the old position in a weighted average
    :param old_position: object containing the components for the last final value for the position
    :param velocity: object containing the components of the velocity on gps coordinate system returned from
    finalize velocity
    :param gps: object containing the compenets of the gps position from the gps lcm message
    :return: object containing componenets of final position of the rover
    """
    lat_minutes = old_position.lat.minutes + velocity.north * delta_time * meters_to_latitude_minutes
    lat_degrees += velocity_weight * (lat_minutes // 60) + gps_weight * gps.lat.degrees
    lat_minutes = velocity_weight * (lat_minutes % 60) + gps_weight * gps.lat.minutes
    lon_minutes = old_position.lat.minutes + velocity.north * delta_time * meters_to_longitude_minutes
    lon_degrees += velocity_weight * (lon_minutes // 60) + gps_weight * gps.lon.degrees
    lon_minutes = velocity_weight * (lon_minutes % 60) + gps_weight * gps.lon.minutes
    return position(lat_degrees, lat_minutes, lon_degrees, lon_minutes)

### Bearing Filter Logic

def stationary_bearing(old_bearing, magnetometer):
    """
    Updates the bearing using an average of the old bearing and the value from the magnetometer
    :param old_bearing: previous final bearing value
    :param magnetometer: value from the magnetometer lcm message (if we have 2 magnetometers could be averaged)
    :return: the final bearing
    """
    return (old_bearing + magnetometer) / 2

def turning_bearing(old_bearing, gyro, magnetometer):
    """
    Updates the bearing using a weighted average of gyro derived bearing, the old bearing and magnetometer
    :param old_bearing: previous final bearing value
    :param gyro: object containing the components of the gyro lcm message
    :param magnetometer: degrees from due north
    :return: the final bearing
    """
    return gyro_weight * (old_bearing + gyro.y * delta_time) + magnetometer_weight * magnetometer

def moving_bearing(track_angle, magnetometer):
    """
    Updates the bearing using a weighted average of the track angle and magnetometer
    :param track_angle: degrees from due north from gps lcm
    :param magnetometer: degrees from due north from magnetometer lcm
    :return: the final bearing
    """
    return track_angle_weight * track_angle + magnetometer_weight * magnetometer

def finalize_bearing(state, magnetometer, gyro, track_angle, old_bearing):
    """
    Updates the bearing depending on the state of the rover
    :param state: whether the rover is turning, stationary, or moving
    :param magnetometer: degrees from due north
    :param gyro: object containing the data from the gyro lcm
    :param track_angle: degrees from due north from the gps lcm
    :param old_bearing: the previous final bearing
    :return:
    """
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


