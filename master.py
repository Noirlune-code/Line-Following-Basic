from pymavlink import mavutil
import time
def get_altitude_h(nection):
    msg = nection.recv_match(type="LOCAL_POSITION_NED",blocking=True )

    altitude = round(msg.z, 2)
    return altitude

the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

the_connection.wait_heartbeat()
print("Heartbeat from (system %u and component %u)"% (the_connection.target_system, the_connection.target_component))
the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                       the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111111000), 0, 0, -2, 0, 0, 0, 0, 0, 0, 0, 0))
mode = the_connection.mode_mapping()['GUIDED']

the_connection.mav.set_mode_send(the_connection.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,the_connection.mode_mapping()['GUIDED'])
time.sleep(1)

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0 , 1, 0, 0 ,0, 0, 0 ,0 )
time.sleep(2)
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0, 0, 0, 0, 0 ,0, 0, 2)
msg = the_connection.recv_match(type="COMMAND_ACK" ,blocking= True) 
print(msg)


# the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
#                        the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111111000), 0, 0, -2, 0, 0, 0, 0, 0, 0, 0, 0))
# # while True:
#     msg = the_connection.recv_match(type='SYS_STATUS', blocking=True)
#     print(msg)
#     if msg:
#         # Check if calibration is complete
#         if msg.onboard_control_sensors_health == 0:  # If health is OK, calibration may be done
#             print("All systems are calibrated and healthy.")
#         else:
#             print("Calibration in progress or issues detected.")

# msg = the_connection.recv_match(type='SYS_STATUS', blocking=True)
# print(msg)

msg = the_connection.recv_match(type="LOCAL_POSITION_NED",blocking=True )

print(msg)