###################################
# IMPORTS

# Add parent directory to sys.path so that we can import from ae483clients
import sys, os
sys.path.append(os.path.abspath('..'))
from ae483clients import CrazyflieClient, QualisysClient

# Do all other imports
import time
import json


###################################
# PARAMETERS

# -- PROBABLY THE SAME FOR EVERY FLIGHT IN LABS 1-10 --

# Specify the uri of the drone to which you want to connect (if your radio
# channel is X, the uri should be 'radio://0/X/2M/E7E7E7E7E7')
uri = 'radio://0/67/2M/E7E7E7E7E7'

# Specify the name of the rigid body that corresponds to your active marker
# deck in the motion capture system. If your marker deck number is X, this name
# should be 'marker_deck_X'.
marker_deck_name = 'marker_deck_60'

# Specify the marker IDs that correspond to your active marker deck in the
# motion capture system. If your marker deck number is X, these IDs should be
# [X + 1, X + 2, X + 3, X + 4]. They are listed in clockwise order (viewed
# top-down), starting from the front.
marker_deck_ids = [61, 62, 63, 64]

# -- MAY CHANGE FROM FLIGHT TO FLIGHT --

# Specify whether or not to use the motion capture system
use_mocap = True

# Specify whether or not to use a custom controller
use_controller = False

# Specify whether or not to use a custom observer
use_observer = False

# Specify the variables you want to log at 100 Hz from the drone
variables = [
    # State estimates (custom observer)
    'ae483log.p_x',
    'ae483log.p_y',
    'ae483log.p_z',
    'ae483log.psi',
    'ae483log.theta',
    'ae483log.phi',
    'ae483log.v_x',
    'ae483log.v_y',
    'ae483log.v_z',
    # State estimates (default observer)
    'stateEstimate.x',
    'stateEstimate.y',
    'stateEstimate.z',
    'stateEstimate.yaw',
    'stateEstimate.pitch',
    'stateEstimate.roll',
    'stateEstimate.vx',
    'stateEstimate.vy',
    'stateEstimate.vz',
    # Measurements
    'ae483log.w_x',
    'ae483log.w_y',
    'ae483log.w_z',
    'ae483log.n_x',
    'ae483log.n_y',
    'ae483log.r',
    'ae483log.a_z',
    # Desired position (custom controller)
    'ae483log.p_x_des',
    'ae483log.p_y_des',
    'ae483log.p_z_des',
    # Desired position (default controller)
    'ctrltarget.x',
    'ctrltarget.y',
    'ctrltarget.z',
    # Motor power commands
    'ae483log.m_1',
    'ae483log.m_2',
    'ae483log.m_3',
    'ae483log.m_4',
    # Mocap data
    'ae483log.p_x_mocap',
    'ae483log.p_y_mocap',
    'ae483log.p_z_mocap',
    'ae483log.psi_mocap',
    'ae483log.theta_mocap',
    'ae483log.phi_mocap',
]


###################################
# FLIGHT CODE

# Create and start the client that will connect to the drone
drone_client = CrazyflieClient(
    uri,
    use_controller=use_controller,
    use_observer=use_observer,
    marker_deck_ids=marker_deck_ids if use_mocap else None,
    variables=variables,
)

# Wait until the client is fully connected to the drone
while not drone_client.is_fully_connected:
    time.sleep(0.1)

# Create and start the client that will connect to the motion capture system
if use_mocap:
    mocap_client = QualisysClient([{'name': marker_deck_name, 'callback': drone_client.send_pose}])


# using mocap(x_0), mocap(y_0), mocap(z_0)




# drone_client.move(0.0, 0.0, 0.20, 0.0, 6.0)


# # Hover

# # Graceful takeoff
# drone_client.move(0.0, 0.0, 0.20, 0.0, 1.0)
# drone_client.move(0.0, 0.0, 0.35, 0.0, 1.0)
# drone_client.move(0.0, 0.0, 0.50, 0.0, 1.0)

# # Hover for ten seconds
# drone_client.move(0.0, 0.0, 0.50, 0.0, 10.0)

# # Graceful landing
# drone_client.move(0.0, 0.0, 0.50, 0.0, 1.0)
# drone_client.move(0.0, 0.0, 0.35, 0.0, 1.0)
# drone_client.move(0.0, 0.0, 0.20, 0.0, 1.0)

# Square flight

# Graceful takeoff
# drone_client.move(0.0, 0.0, 0.20, 0.0, 1.0)
# drone_client.move(0.0, 0.0, 0.35, 0.0, 1.0)
# drone_client.move(0.0, 0.0, 0.50, 0.0, 1.0)

# # Move in a square
# drone_client.move(0.5, 0.0, 0.50, 0.0, 2.5)
# drone_client.move(0.5, 0.5, 0.50, 0.0, 2.5)
# drone_client.move(0.0, 0.5, 0.50, 0.0, 2.5)
# drone_client.move(0.0, 0.0, 0.50, 0.0, 2.5)

# # Graceful landing
# drone_client.move(0.0, 0.0, 0.50, 0.0, 1.0)
# drone_client.move(0.0, 0.0, 0.35, 0.0, 1.0)
# drone_client.move(0.0, 0.0, 0.20, 0.0, 1.0)

drone_client.stop(1)
# Graceful takeoff
drone_client.move(-2.487643003463745, 1.1996726989746096, 0.20 + 0.08037926256656627, 0.0, 1.0)
drone_client.move(-2.487643003463745, 1.1996726989746096, 0.35 + 0.08037926256656627, 0.0, 1.0)
drone_client.move(-2.487643003463745, 1.1996726989746096, 0.50 + 0.08037926256656627, 0.0, 1.0)

# Move in a square
drone_client.move(-2.487643003463745 + 0.5, 1.1996726989746096, 0.50 + 0.08037926256656627, 0.0, 2.5)
drone_client.move(-2.487643003463745 + 0.5, 0.5 + 1.1996726989746096, 0.50 + 0.08037926256656627, 0.0, 2.5)
drone_client.move(-2.487643003463745, 0.5 + 1.1996726989746096, 0.50 + 0.08037926256656627, 0.0, 2.5)
drone_client.move(-2.487643003463745, 1.1996726989746096, 0.50 + 0.08037926256656627, 0.0, 2.5)

# Graceful landing
drone_client.move(-2.487643003463745, 1.1996726989746096, 0.50 + 0.08037926256656627, 0.0, 1.0)
drone_client.move(-2.487643003463745, 1.1996726989746096, 0.35 + 0.08037926256656627, 0.0, 1.0)
drone_client.move(-2.487643003463745, 1.1996726989746096, 0.20 + 0.08037926256656627, 0.0, 1.0)

drone_client.stop(1)

# start at z = 0.2
dy = 0.02 # meters
dz = 0.02 # meters
dphi = np.rad(3) # degrees
pitch = 0
yaw = 0
roll = 0
steps = 15
v = 0.2
position = [0, 0, 0, 0, 0, 0] # x, y, z, psi, theta, phi 
# update with current position in reference bc custom observer, if you use mocap the world frame is shifted
desired_position = [0, dy, dz, 0, 0, dphi]
drone_client.stop(1)
drone_client.move(-2.487643003463745, 1.1996726989746096, 0.20 + 0.08037926256656627, 0.0, 1.0) 
# update with current position in reference bc custom observer, with hovering at 0.2 meters



for i in steps:
    drone_client.movesmooth_and_rotate(self, position, desired_position, yaw, pitch, roll, v)
    position[1] += dy
    position[2] += dz
    position[5] += dphi
    desired_position[1] += dy
    desired_position[2] += dz
    desired_position[5] += dphi
    roll += dphi
    

for i in steps:
    drone_client.movesmooth_and_rotate(self, position, desired_position, yaw, pitch, roll, v)
    position[1] += -dy
    position[2] += -dz
    position[5] += -dphi
    desired_position[1] += -dy
    desired_position[2] += -dz
    desired_position[5] += -dphi
    roll += -dphi

for i in steps:
    drone_client.movesmooth_and_rotate(self, position, desired_position, yaw, pitch, roll, v)
    position[1] += -dy
    position[2] += -dz
    position[5] += -dphi
    desired_position[1] += -dy
    desired_position[2] += -dz
    desired_position[5] += -dphi
    roll += -dphi


for i in steps:
    drone_client.movesmooth_and_rotate(self, position, desired_position, yaw, pitch, roll, v)
    position[1] += dy
    position[2] += dz
    position[5] += dphi
    desired_position[1] += dy
    desired_position[2] += dz
    desired_position[5] += dphi
    roll += dphi
    
# Disconnect from the drone
drone_client.disconnect()

# Disconnect from the motion capture system
if use_mocap:
    mocap_client.close()

# Assemble flight data from both clients
data = {}
data['drone'] = drone_client.data
data['mocap'] = mocap_client.data.get(marker_deck_name, {}) if use_mocap else {}
data['bodies'] = mocap_client.data if use_mocap else {}

# Write flight data to a file
with open('hardware_data.json', 'w') as outfile:
    json.dump(data, outfile, sort_keys=False)
    