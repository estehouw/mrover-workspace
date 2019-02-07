import time
import enum
from math import sin, cos, radians
from rover_common import aiolcm
import asyncio
from rover_msgs import IMU, GPS, NavStatus
from rover_common.aiohelper import run_coroutines
from .rawmessages import raw_imu, raw_gps, nav_status, clean_odom


UNDEFINED = None
INFREQUENCY = 0.2  # inverse of frequency of slowest sensor (probably GPS)
delta_time = 0.01


class FilterConfig():
    def __init__(self):
        self.imu_accel_weight = .5
        self.imu_bearing_weight = .5
        self.imu_delta_time = .01
        self.gyro_weight = .5
        self.gps_loc_weight = .5
        self.gps_tracking_angle_weight = .5
        self.gps_ground_speed_weight = .5
        self.calculated_velocity_weight = .5
        self.update_rate = .1
        self.meters_to_latitude_minutes = 0.0005389625
        self.EARTH_RADIUS = 6371000
        self.EARTH_CIRCUM = 40075000
        self.meters_to_longitude_minutes = None

    def meters_to_long_min(self, lat_deg, lat_min):
        return 60 / (self.EARTH_CIRCUM * cos(radians(lat_deg + lat_min)) / 360)


filterconfig = FilterConfig()


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
        self._gps = raw_gps()
        self._imu = raw_imu()
        self._navstat = nav_status()
        self._odomf = clean_odom()
        self._vel = absolute_vel(0, 0, 0)
        self._pitch = 0

    def gps_callback(self, channel, msg):
        print('-------------- @time : ' + str(time.clock()))
        print('gps callback called')
        m = GPS.decode(msg)
        self._gps.updateGPS(m)
        self._odomf.copy_gps(self._gps)
        print('gps track angle: ' + str(self._gps._track_theta))
        return None

    def imu_bearing_callback(self, channel, msg):
        print('-------------- @time : ' + str(time.clock()))
        print('imu callback called')
        m = IMU.decode(msg)
        self._imu.update_imu_bearing(m)
        print('mag bearing: ' + str(self._imu._bearing))
        return None

    def navstat_callback(self, channel, msg):
        print('-------------- @time : ' + str(time.clock()))
        print('navstat callback called')
        m = NavStatus.decode(msg)
        self._navstat.update_nav_status(m)
        if self.turning():
            print('turning')
        else:
            print("not turning")
        print('nav status: ' + str(self._navstat._navState))
        return None

    def stationary(self):
        """Determine if rover is stationary."""
        if self._navstat._navState == NavState.Off or \
           self._navstat._navState == NavState.Done:
            return True
        return False

    def turning(self):
        """Determine if rover is turning."""
        if self._navstat._navState == NavState.Turn or \
           self._navstat._navState == NavState.SearchTurn or \
           self._navstat._navState == NavState.SearchFaceNorth or \
           self._navstat._navState == NavState.SearchFace120 or \
           self._navstat._navState == NavState.SearchFace240 or \
           self._navstat._navState == NavState.SearchFace360 or \
           self._navstat._navState == NavState.TurnToBall or \
           self._navstat._navState == NavState.TurnAroundObs or \
           self._navstat._navState == NavState.SearchTurnAroundObs:
            return True

        return False

    def driving(self):
        """Determine if rover is driving."""
        if self._navstat._navState == NavState.Drive or \
           self._navstat._navState == NavState.SearchDrive or \
           self._navstat._navState == NavState.DriveToBall or \
           self._navstat._navState == NavState.DriveAroundObs or \
           self._navstat._navState == NavState.SearchDriveAroundObs:
            return True

        return False

    def update_pitch(self):
        '''
        Updates the pitch angle used to correct
        the forward acceleration of the rover
        TODO Correct using g vector
        :return:
        '''
        gyro = self._gyro
        self._pitch += gyro.y * filterconfig.delta_time

    def generate_absolute_acceleration(self):
        """
        Converts the imu data to absolute coordinite system used by the gps
        and corrects for the pitch angle, also updates the z_acceleration
        :return: absolute_accel object containing
        the acceleration in North, East, And Z Directions
        """
        bearing = self._odomf.bearing
        pitch = self._pitch
        raw_imu = self._imu
        north_acc = raw_imu.x * cos(pitch) * sin(90 - bearing)
        east_acc = raw_imu.x * cos(pitch) * cos(90 - bearing)
        z_acc = raw_imu.x * sin(pitch)
        return absolute_accel(north_acc, east_acc, z_acc)

    def decompose_ground_speed(self):
        """
        Breaks the ground speed velocity into
        components aligned with gps coordinate system
        :return: vel object containing the velocity
         in the North, East and Z Directions
        """
        ground_speed = self._gps.ground_speed
        bearing = self._odomf.bearing
        vel_East = ground_speed * cos(90 - bearing)
        vel_North = ground_speed * sin(90 - bearing)
        return absolute_vel(vel_East, vel_North, 0)

    def finalize_velocity(self):
        """
        Combines the ground speed and velocity from
        the imu in a weighted average
        :return: absolute_vel object containing the
        velocity in the North, East, and Z directions
        """
        ground_speed = self.decompose_ground_speed()
        accel = self.generate_absolute_acceleration()
        old_velocity = self._vel
        vel_North = filterconfig.imu_accel_weight \
            * (old_velocity.x + accel.x * filterconfig.imu_delta_time) \
            + filterconfig.gps_ground_speed_weight * ground_speed.x
        vel_East = filterconfig.imu_accel_weight \
            * (old_velocity.y + accel.y * filterconfig.imu_delta_time) \
            + filterconfig.gps_ground_speed_weight * ground_speed.y
        vel_z = filterconfig.imu_accel_weight \
            * (old_velocity.z + accel.z * filterconfig.imu_delta_time)
        self._vel = absolute_vel(vel_North, vel_East, vel_z)
        return self.vel

    def filter_location(self):
        """
        Combines the gps data with the velocity added
        to the old position in a weighted average
        :return: object containing componenets of final position of the rover
        """
        old_position = self._odomf
        velocity = self.finalize_velocity()
        gps = self._gps

        if not filterconfig.meters_to_longitude_minutes:
            filterconfig.meters_to_longitude_minutes = \
                filterconfig.meters_to_long_min(gps.lat_deg, gps.lat_min)

        meters_to_longitude_minutes = filterconfig.meters_to_longitude_minutes
        meters_to_latitude_minutes = filterconfig.meters_to_latitude_minutes
        lat_degrees = gps.lat_deg
        lon_degrees = gps.lon_deg

        lat_minutes = old_position.lat.minutes + velocity.north \
            * filterconfig.imu_delta_time * meters_to_latitude_minutes
        lat_degrees += filterconfig.calculated_velocity_weight \
            * (lat_minutes // 60) + filterconfig.gps_loc_weight \
            * gps.lat.degrees
        lat_minutes = filterconfig.calculated_velocity_weight \
            * (lat_minutes % 60) + filterconfig.gps_loc_weight \
            * gps.lat.minutes
        lon_minutes = old_position.lat.minutes + velocity.north \
            * filterconfig.imu_delta_time * meters_to_longitude_minutes
        lon_degrees += filterconfig.calculated_velocity_weight \
            * (lon_minutes // 60) + filterconfig.gps_loc_weight \
            * gps.lon.degrees
        lon_minutes = filterconfig.calculated_velocity_weight \
            * (lon_minutes % 60) + filterconfig.gps_loc_weight \
            * gps.lon.minutes
        return gps(lat_degrees, lat_minutes, lon_degrees, lon_minutes)

    async def publishOdom(self, lcm_):
        while True:
            print('async af')
            # self.filter_bearing()
            # self.filter_location()
            msg = self._odomf.create_lcm()
            lcm_.publish('/odometryf', msg.encode())
            await asyncio.sleep(filterconfig.update_rate)

        return None


def main():
    lcm_ = aiolcm.AsyncLCM()
    filter_ = FilterClass()

    lcm_.subscribe("/gps", filter_.gps_callback)
    lcm_.subscribe("/imu", filter_.imu_bearing_callback)
    lcm_.subscribe("/nav_status", filter_.navstat_callback)

    run_coroutines(lcm_.loop(), filter_.publishOdom(lcm_))


if __name__ == "__main__":
    main()
