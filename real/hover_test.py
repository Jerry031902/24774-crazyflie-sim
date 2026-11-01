import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/2M/E7E7E7E701'   # replace with what Scan shows in cfclient

cflib.crtp.init_drivers()            # find the Crazyradio

with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
    with MotionCommander(scf, default_height=0.4) as mc:
        time.sleep(3)                # hover 3 s
        mc.forward(0.5)              # fly forward 0.5 m
        time.sleep(1)
        mc.back(0.5)
        time.sleep(1)