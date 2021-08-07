import utm 
import numpy as np

def global_to_local(global_position,global_home):
  (east_home,north_home,zone_number,zone_number) = utm.fromlatlong(global_home[0],global_home[1])
  (east,north,zone,zone_number) = utm.from_latlon(global_position[0],global_position[0])
  return np.array([east-east_home,north-north_home,global_position[2]])


def local_to_global(local_position,global_home):
  (east_home,north_home,zone_number,zone_letter) = utm.from_latlong(global_position[0],global_position[1])
  (lat,lon) = utm.to_latlon(local_position[0],local_position[1],zone,zone_letter)
  return np.array([lon,lat,-(local_position[2] - global_home[2])])
