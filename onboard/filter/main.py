import os
# from serial import Serial
# from configparser import ConfigParser
from rover_common import aiolcm
from rover_msgs import Bearing
import asyncio
import os
from rover_common.aiohelper import exec_later, run_coroutines
from time import time
#from rover_msgs import GPS
#from rover_msgs import IMU
#from rover_msgs import Joystick
#from rover_msgs import wheelenc
wack = -999

class NavState:
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

class FilterClass:
    def __init__(self ):#, time_):
        # prev
        global wack
        self.mag_bearing = wack
        self.gx , self.gy, self.gz = wack, wack, wack
        self.accx , self.accy, self.accz = wack , wack , wack
        self.lat_deg = wack
        self.lat_min = wack
        self.long_deg = wack
        self.long_min = wack
        self.track_theta = wack
        self.ground_speed = wack
        self.navstatus = wack
        self.rear_l = wack
        self.rear_r = wack
        self.front_l = wack
        self.front_r = wack
        self.deltatime = wack
        self.time_of_IMU = time()
        self.update = True
        self.is_turning = False # Is the rover turning.
        self.bearing = wack # Bearing of rover.
        self.initial_gyro = wack # Initial gyro value.


    def gps_callback(self, channel, msg):
        m = GPS.decode(msg)
        self.lat_deg = m.latitude_deg
        self.lat_min = m.latitude_min
        self.long_deg = m.longitude_deg
        self.long_min = m.longitude_min
        self.track_theta = m.bearing_deg
        self.ground_speed = m.groundspeed
        self.updateFlag = True
        # exec_later()
        return None

    def imu_callback(self, channel, msg):
        m = IMU.decode(msg)
        self.accx , self.accy, self.accz = m.accx , m.accy, m.accz
        self.gx , self.gy, self.gz = m.gyrox , m.gyroy, m.gyroz
        self.deltatime = time() - self.time_of_IMU
        self.time_of_IMU = time()
        self.updateFlag = True
        return None


    def wheelenc_callback(self, channel, msg):
        m = wheelenc.decode(msg)
        self.rear_l = m.rearleft  # Names not established: todo
        self.rear_r = m.rearright
        self.front_l = m.frontleft
        self.front_r = m.frontright
        self.updateFlag = True

        return None

    def navstat_callback(self, channel, msg):
        m = NavStatus.decode(msg)
        self.navstatus = m.nav_state
        self.updateFlag = True
        return None


async def publish_Odometry():
    while True:
        od = Odometry()
        od.bearing_deg = wack
        od.lat_deg = wack
        od.lat_min = wack
        od.long_deg = wack
        od.long_min = wack
        # ec.joint_a = int(
        #     await rover.talons[Talons.arm_joint_a.value].get_enc_pos() or 0)
        # ec.joint_b = int(
        #     await rover.talons[Talons.arm_joint_b.value].get_enc_pos() or 0)
        # ec.joint_c = int(
        #     await rover.talons[Talons.arm_joint_c.value].get_enc_pos() or 0)
        # do this^    

        lcm_.publish('/Odometry', od.encode())

        await asyncio.sleep(0.1)

    def stationary(self):
        """Determine if rover is stationary."""
        if self.navstatus == NavState.Off or \
           self.navstatus == NavState.Done:
            return True
        return False

    def turning(self):
        """Determine if rover is turning."""
        if self.navstatus == NavState.Turn or \
           self.navstatus == NavState.SearchTurn or \
           self.navstatus == NavState.SearchFaceNorth or \
           self.navstatus == NavState.SearchFace120 or \
           self.navstatus == NavState.SearchFace240 or \
           self.navstatus == NavState.SearchFace360 or \
           self.navstatus == NavState.TurnToBall or \
           self.navstatus == NavState.TurnAroundObs or \
           self.navstatus == NavState.SearchTurnAroundObs:
            return True
        return False

    def driving(self):
        """Determine if rover is driving."""
        if self.navstatus == NavState.Drive or \
           self.navstatus == NavState.SearchDrive or \
           self.navstatus == NavState.DriveToBall or \
           self.navstatus == NavState.DriveAroundObs or \
           self.navstatus == NavState.SearchDriveAroundObs:
            return True
        return False

    def filter_bearing(self):
        if self.turning() == True: 
            if not self.is_turning:
                self.is_turning = False
                return self.bearing
            # Save initial gyro value on first loop of turning.
            # Subtract gyro value from saved initial value to get difference.
            # Fuse mag value with gyro to average it.
        elif self.driving() == True:
            self.is_turning = False
            pass
        elif self.stationary() == True:
            self.is_turning = False
            pass
        else:
            print('shitz fukd, check your weights homie')

    def filter_location(self):
        pass

    # actual kalman filtering
    # class filter:: filter()
    def filter(self):

        clean_lat_min, clean_lat_deg, clean_long_min, clean_long_deg = 0,0,0,0
        clean_bearing , clean_ground_speed = 0 , 0
        clean_bearing = self.filter_bearing()


        return clean_lat_min, clean_lat_deg, clean_long_min, clean_long_deg , clean_bearing , clean_ground_speed



    # IMU lcm
    #   double mag_bearing
    #   double gyrox
    #   double gyroy
    #   double gyroz
    #   double accx, accy,accz


    # GPS lcm
    #     int32_t latitude_deg;
    #     double latitude_min;
    #     int32_t longitude_deg;
    #     double longitude_min;
    #     double bearing_deg;
    #     double groundspeed;
    #

    # wheelenc LCM
    #   double revs ????
    #   x 4



def main():

    lcm = aiolcm.AsyncLCM()

    filter_ = filterClass()
    # global rover
    # rover = Rover()
    gps_subscription = lcm_.subscribe("/GPS", filter_.gps_callback )
    IMU = lcm_.subscribe("/IMU", filter_.imu_callback)
    wheel_enc = lcm_.subscribe("/wheel_enc", filter_.wheelenc_callback)
    navstate = lcm_.subscribe("/nav_status", filter_.navstat_callback)
    odom = lcm_.subscribe("/odometry", filter_.odom_callback)

    # run_coroutines(publish_Odometry(), lcm_.loop(), rover.run_all())
    while True:

        if filter_.updateFlag == True: # if there was an input LCM update since last weighting
            latm_ , latd_ , longm_ , longd_ , bearing_ , speed_ = filter_.filter()
            odom_msg = Odometry()
            odom_msg.latitude_deg = latd_
            odom_msg.latitude_min = latm_
            odom_msg.longitude_deg = longd_
            odom_msg.longitude_min = longm_
            odom_msg.bearing_deg = bearing_
            odom_msg.speed = speed_
            lcm.publish('/odometry', odom_msg.encode())
            filter_.updateFlag = False # no need to consider re-weighting


if __name__ == "__main__":
    main()
