# desktop implementation of listener for VESC/ENNOID

import struct
import time
import can

vehicle_data = {'battery_voltage':0,
                'battery_current':0,
                'battery_voltage_BMS':0,
                'battery_current_BMS':0,
                'high_cell_voltage':0,
                'low_cell_voltage':0,
                'high_battery_temp':0,
                'high_BMS_temp':0,
                'motor_rpm':0,
                'total_current':0,
                'motor_temperature':0,
                'motor_current':0,
                'controller_temperature':0}

vehicle_data_last_read = {'battery_voltage':0,
                          'battery_current':0,
                          'battery_voltage_BMS':0,
                          'battery_current_BMS':0,
                          'high_cell_voltage':0,
                          'low_cell_voltage':0,
                          'high_battery_temp':0,
                          'high_BMS_temp':0,
                          'motor_rpm':0,
                          'total_current':0,
                          'motor_temperature':0,
                          'motor_current':0,
                          'controller_temperature':0}

print("ENNOID/VESC CAN reader")

def print_to_console():

    print()
    print("R", vehicle_data['motor_rpm']/1E0, end=' | ')
    print("H", vehicle_data['high_cell_voltage']/1E5, end=' | ')
    print("L", vehicle_data['low_cell_voltage']/1E5, end=' | ')
    print('Vb', vehicle_data['battery_voltage_BMS']/1E5, end=' | ')
    print('Vc', vehicle_data['battery_voltage']/1E1, end=' | ')
    print('I', vehicle_data['battery_current_BMS']/1E5, end=' | ')
    print('Im', vehicle_data['total_current']/1E1, end=' | ')
    print('Tc', vehicle_data['controller_temperature']/1E1, end=' | ')
    print('Tm', vehicle_data['motor_temperature']/1E1, end=' | ')
    print(time.monotonic())

def update_parameters():

    # currently updates variables every time a packet is encountered
    # will need to modify to get deltas
    # or maybe filter and then derived_fsm samples?
    # this could be a dict of dicts specifying variable code, byte start, byte length, and variable string

    def process_vesc_09XX(vehicle_data):
        vehicle_data['motor_rpm'] = struct.unpack('>L', message.data[0:4])[0]
        vehicle_data['total_current'] = struct.unpack('>H', message.data[4:6])[0]

    def process_vesc_10XX(vehicle_data):
        vehicle_data['controller_temperature'] = struct.unpack('>H', message.data[0:2])[0]
        vehicle_data['motor_temperature'] = struct.unpack('>H', message.data[2:4])[0]

    def process_vesc_1bXX(vehicle_data):
        vehicle_data['battery_voltage'] = struct.unpack('>H', message.data[4:6])[0]

    def process_dbms_1fXX(vehicle_data):
        vehicle_data['high_cell_voltage'], vehicle_data['low_cell_voltage'] = struct.unpack('>ii', message.data)

    def process_dbms_1eXX(vehicle_data):
        vehicle_data['battery_voltage_BMS'], vehicle_data['battery_current_BMS'] = struct.unpack('>ii', message.data)

    # TODO: make CAN ids based on device ID config file
    # TODO: more robust indexing scheme

    message_ids = [0x0901, 0x1001, 0x1b01, 0x1e0a, 0x1f0a]

    read_functions = {'0x901':process_vesc_09XX,
                      '0x1001':process_vesc_10XX,
                      '0x1b01':process_vesc_1bXX,
                      '0x1e0a':process_dbms_1eXX,
                      '0x1f0a':process_dbms_1fXX}

    message = bus.recv(timeout=0.050)
    if message is not None:
        print('+', end='')
        if message.arbitration_id in message_ids:
            # dispatch message to functions
            read_functions[str(hex(message.arbitration_id))](vehicle_data)
    else:
        print('.', end='')

def check_if_time_to_print():
    global last_display
    if time.monotonic() - last_display > display_update_seconds:
        print_to_console()
        last_display = time.monotonic()

bus = can.interface.Bus(bustype='slcan',
                        channel='/dev/tty.usbmodem14101',
                        bitrate=500000)

display_update_seconds = 1.0
last_display = time.monotonic()

# display class just checks if time to print and prints
# class CONSOLE():

while 1:
    update_parameters()
    check_if_time_to_print()