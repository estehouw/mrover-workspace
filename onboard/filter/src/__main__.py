import time
import enum
from rover_common import aiolcm
import asyncio
from rover_msgs import Bearing
from rover_msgs import Odometry
from rover_msgs import NavStatus
from rover_common.aiohelper import run_coroutines
from .rawmessages import mag_bearing
from .rawmessages import raw_gps
from .rawmessages import nav_status
from .rawmessages import clean_odom

# from rover_msgs import Wheelenc
# from rover_msgs import IMU

UNDEFINED = 0
INFREQUENCY = 0.2  # inverse of frequency of slowest sensor (probably GPS)
delta_time = 0.01

"""
Filter Goals:

    -Input:
        accel from imu: x, y, z
        gyro from imu: x y z velocities        
        mag bearing
        GPS track angle
        rover state
        GPS ground speed
        GPS position (lat deg, lon deg, lat min, lon min)
        
                                                        
                
    -Output:
        pitch
        velocity
        gps:
            lat_min
            lat_deg
            lon_min
            lon_deg
        bearing


    Can use g vector when sitting still to determine which way is down
    Use to correct pitch^

"""



class NavState(enum.Enum):
    Off = 0
    Done = 1
    Turn = 10
    Drive = 11
    SearchFaceNorth = 20
    SearchFace120 = 21
    SearchFace240 = 22
    SearchFace360 = 23
    SearchTurn = 24
    SearchDrive = 25
    TurnToBall = 28
    DriveToBall = 29
    TurnAroundObs = 30
    DriveAroundObs = 31
    SearchTurnAroundObs = 32
    SearchDriveAroundObs = 33
    Unknown = 255

class raw_accel:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class absolute_accel:
    def __init__(self, north, east, z):
        self.north = north
        self.east = east
        self.z = z

class gyro:
    def __init__(self, pitch, roll, yaw):
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw

class absolute_vel:
    def __init__(self, north, east, z):
        self.north = north
        self.east = east
        self.z = z

class gps:
    def __init__(self, lat_deg, lat_min, lon_deg, lon_min):
        self.lat_deg = lat_deg
        self.lat_min = lat_min
        self.lon_deg = lon_deg
        self.lon_min = lon_min


class FilterClass:
    def __init__(self):
        self.gps = raw_gps()
        self.imu_bearing = mag_bearing() #this should probably say mag bearing not imu?
        #self.mag_bearing = mag_bearing()
        self.navstate = nav_status()
        self.odomf = clean_odom()
        self.vel = absolute_vel(0, 0, 0)
        self.pitch = 0
        #TODO: accel object = from raw imu 
        #TODO: gyro object = from raw imu

    def gps_callback(self, channel, msg):
        print('-------------- @time : ' + str(time.clock()))
        print('gps callback called')
        m = Odometry.decode(msg)
        self.gps.updateGPS(m)
        #add filter later, dont cop
        self.odomf.copy_gps(self.gps)
        print('gps track angle: ' + str(self.gps.track_theta))
        return None

    #everything in here that says imu should probably say mag?
    def mag_bearing_callback(self, channel, msg): 
        print('-------------- @time : ' + str(time.clock()))
        print('imu callback called')
        m = Bearing.decode(msg)
        self.imu_bearing.update_mag_bearing(m)
        print('mag bearing: ' + str(self.imu_bearing.mbearing))
        return None

    def navstat_callback(self, channel, msg):
        print('-------------- @time : ' + str(time.clock()))
        print('navstat callback called')
        m = NavStatus.decode(msg)
        self.navstat.update_nav_status(m)
        if self.turning():
            print('turning')
        else:
            print("not turning")
        print('nav status: ' + str(self.navstat.navState))
        return None
    
    ##############
    #TODO: make IMU callbacks for accel and gyro velocities    
    ##############

    def stationary(self):
        """Determine if rover is stationary."""
        if self.navstat == NavState.Off or \
           self.navstat == NavState.Done:
            return True
        return False

    def turning(self):
        """Determine if rover is turning."""
        if self.navstat.navState == NavState.Turn or \
           self.navstat.navState == NavState.SearchTurn or \
           self.navstat.navState == NavState.SearchFaceNorth or \
           self.navstat.navState == NavState.SearchFace120 or \
           self.navstat.navState == NavState.SearchFace240 or \
           self.navstat.navState == NavState.SearchFace360 or \
           self.navstat.navState == NavState.TurnToBall or \
           self.navstat.navState == NavState.TurnAroundObs or \
           self.navstat.navState == NavState.SearchTurnAroundObs:
            return True

        return False

    def driving(self):
        """Determine if rover is driving."""
        if self.navstat.navState == NavState.Drive or \
           self.navstat.navState == NavState.SearchDrive or \
           self.navstat.navState == NavState.DriveToBall or \
           self.navstat.navState == NavState.DriveAroundObs or \
           self.navstat.navState == NavState.SearchDriveAroundObs:
            return True

        return False

    """def filter_bearing():
        return None"""

    # this function is run as a co-routine for publishing fused odometry
    async def publishOdom(self, lcm_):
        while True:
            print('async af')
            # self.filter_bearing()
            # self.filter_location()            
            msg = self.odomf.create_lcm()
            lcm_.publish('/odometryf', msg.encode())
            await asyncio.sleep(1)

        return None

    ### Postion Filter Logic
    def update_pitch(self):
        '''
        Updates the pitch angle used to correct the forward acceleration of the rover
        TODO Correct using g vector
        :return:
        '''
        gyro = self.gyro
        self.pitch += gyro.y * delta_time

    def generate_absolute_acceleration(self):
        """
        Converts the imu data to absolute coordinite system used by the gps
        and corrects for the pitch angle, also updates the z_acceleration
        :return: absolute_accel object containing the acceleration in North, East, And Z Directions
        """
        bearing = odomf.bearing
        pitch = self.pitch
        raw_imu = self.imu
        north_acc = raw_imu.x * cos(pitch) * sin(90 - bearing)
        east_acc = raw_imu.x * cos(pitch) * cos(90 - bearing)
        z_acc = raw_imu.x * sin(pitch)
        return absolute_accel(north_acc, east_acc, z_acc)

    def decompose_ground_speed(self):
        """
        Breaks the ground speed velocity into components aligned with gps coordinate system
        :return: vel object containing the velocity in the North, East and Z Directions
        """
        ground_speed = self.raw_gps.ground_speed
        bearing = odomf.bearing
        vel_East = ground_speed * cos(90 - bearing)
        vel_North = ground_speed * sin(90 - bearing)
        return absolute_vel(vel_East, vel_North, 0)

    def finalize_velocity(self):
        """
        Combines the ground speed and velocity from the imu in a weighted average
        :return: absolute_vel object containing the velocity in the North, East, and Z directions
        """
        ground_speed = self.decompose_ground_speed()
        accel = self.generate_absolute_acceleration()
        old_velocity = self.vel
        vel_North = accel_weight * (old_velocity.x + accel.x * delta_time) + ground_speed_weight * ground_speed.x
        vel_East = accel_weight * (old_velocity.y + accel.y * delta_time) + ground_speed_weight * ground_speed.y
        vel_z = accel_weight * (old_velocity.z + accel.z * delta_time)
        self.vel = absolute_vel(vel_North, vel_East, vel_z)
        return self.vel

    def filter_location(self):
        """
        Combines the gps data with the velocity added to the old position in a weighted average
        :return: object containing componenets of final position of the rover
        """
        old_position = self.odomf
        velocity = self.finalize_velocity()
        gps = self.gps

        lat_minutes = old_position.lat.minutes + velocity.north * delta_time * meters_to_latitude_minutes
        lat_degrees += velocity_weight * (lat_minutes // 60) + gps_weight * gps.lat.degrees
        lat_minutes = velocity_weight * (lat_minutes % 60) + gps_weight * gps.lat.minutes
        lon_minutes = old_position.lat.minutes + velocity.north * delta_time * meters_to_longitude_minutes
        lon_degrees += velocity_weight * (lon_minutes // 60) + gps_weight * gps.lon.degrees
        lon_minutes = velocity_weight * (lon_minutes % 60) + gps_weight * gps.lon.minutes
        return gps(lat_degrees, lat_minutes, lon_degrees, lon_minutes)

    ### Bearing Filter Logic

    def stationary_bearing(self, old_bearing, magnetometer):
        """
        Updates the bearing using an average of the old bearing and the value from the magnetometer
        :param old_bearing: previous final bearing value
        :param magnetometer: value from the magnetometer lcm message (if we have 2 magnetometers could be averaged)
        :return: the final bearing
        """
        return (old_bearing + magnetometer) / 2

    def turning_bearing(self, old_bearing, gyro, magnetometer):
        """
        Updates the bearing using a weighted average of gyro derived bearing, the old bearing and magnetometer
        :param old_bearing: previous final bearing value
        :param gyro: object containing the components of the gyro lcm message
        :param magnetometer: degrees from due north
        :return: the final bearing
        """
        return gyro_weight * (old_bearing + gyro.y * delta_time) + magnetometer_weight * magnetometer

    def moving_bearing(self, track_angle, magnetometer):
        """
        Updates the bearing using a weighted average of the track angle and magnetometer
        :param track_angle: degrees from due north from gps lcm
        :param magnetometer: degrees from due north from magnetometer lcm
        :return: the final bearing
        """
        return track_angle_weight * track_angle + magnetometer_weight * magnetometer

    def finalize_bearing(self, state, magnetometer, gyro, track_angle, old_bearing):
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





def main():
    lcm_ = aiolcm.AsyncLCM()
    filter_ = FilterClass()

    lcm_.subscribe("/odometry", filter_.gps_callback)
    lcm_.subscribe("/bearing", filter_.mag_bearing_callback)
    lcm_.subscribe("/nav_status", filter_.navstat_callback)

    run_coroutines(lcm_.loop(), filter_.publishOdom(lcm_))


if __name__ == "__main__":
    main()
