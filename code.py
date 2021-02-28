# desktop implementation of listener for VESC/ENNOID

import struct
import time
import can
from can.bus import BusState


'''
VESC packets

00000901   DLC:  8    00 00 00 00 00 00 00 00
00000e01   DLC:  8    00 00 00 00 00 00 00 00
00000f01   DLC:  8    00 00 00 00 00 00 00 00
00001001   DLC:  8    01 21 fc 82 00 00 23 c9
00001b01   DLC:  8    00 00 00 00 01 ea 00 00

EBMS packets

MAIN_IV
00001e0a   DLC:  8    00 4a 87 4c ff ff f9 e8
CELL_VOLTAGE
00001f0a   DLC:  8    00 06 3c 22 00 06 3c cc
THROTTLE_CH_DISCH_BOOL
0000200a   DLC:  8    12 d4 04 af 63 64 64 25
'''

vehicle_data = {'battery_voltage':0,
                'battery_current':0,
                'battery_voltage_BMS':0,
                'battery_current_BMS':0,
                'high_cell_voltage':0,
                'low_cell_voltage':0,
                'high_battery_temp':0,
                'high_BMS_temp':0,
                'motor_rpm':0,
                'motor_temperature':0,
                'motor_current':0,
                'controller_temperature':0}

# if hasattr(board, 'BOOST_ENABLE'):
#     boost_enable = digitalio.DigitalInOut(board.BOOST_ENABLE)
#     boost_enable.switch_to_output(True)

# can = canio.CAN(rx=board.CAN_RX, tx=board.CAN_TX, baudrate=500_000, auto_restart=True)
#listener = can.listen(matches=[canio.Match(0x002, mask=0xFFF, extended=True)], timeout=.5)
# listener = can.listen(timeout=.5)

# old_bus_state = None
# old_count = -1

print("Hello World!")

# TODO: this is hardwired for CAN bus ids
# TODO: make CAN ids based on device ID config file
# array of message ids to parse

# dict of pointers to read functions
# maybe list is more efficient

def print_to_console():

    print()
    print("H", vehicle_data['high_cell_voltage']/1E5, end=' | ')
    print("L", vehicle_data['low_cell_voltage']/1E5, end=' | ')
    print('Vb', vehicle_data['battery_voltage_BMS']/1E5, end=' | ')
    print('Vc', vehicle_data['battery_voltage']/1E1, end=' | ')
    print('I', vehicle_data['battery_current_BMS']/1E5, end=' | ')
    print('Tc', vehicle_data['controller_temperature']/1E1, end=' | ')
    print('Tm', vehicle_data['motor_temperature']/1E1, end=' | ')
    print(time.monotonic())

display_update_seconds = 1.0
last_display = time.monotonic()

def update_parameters():

    def process_vesc_10XX(vehicle_data):
        vehicle_data['controller_temperature'] = struct.unpack('>H', message.data[0:2])[0]
        vehicle_data['motor_temperature'] = struct.unpack('>H', message.data[2:4])[0]

    def process_vesc_1bXX(vehicle_data):
        vehicle_data['battery_voltage'] = struct.unpack('>H', message.data[4:6])[0]

    def process_dbms_1fXX(vehicle_data):
        vehicle_data['high_cell_voltage'], vehicle_data['low_cell_voltage'] = struct.unpack('>ii', message.data)

    def process_dbms_1eXX(vehicle_data):
        vehicle_data['battery_voltage_BMS'], vehicle_data['battery_current_BMS'] = struct.unpack('>ii', message.data)

    message_ids = [0x1001, 0x1b01, 0x1e0a, 0x1f0a]

    read_functions = {'0x1001':process_vesc_10XX,
                      '0x1b01':process_vesc_1bXX,
                      '0x1e0a':process_dbms_1eXX,
                      '0x1f0a':process_dbms_1fXX}

    message = bus.recv(1)
    if message is not None:
        print('.', end='')
        if message.arbitration_id in message_ids:
            # dispatch message to functions
            read_functions[str(hex(message.arbitration_id))](vehicle_data)

def check_if_time_to_print():
    global last_display
    if time.monotonic() - last_display > display_update_seconds:
        print_to_console()
        last_display = time.monotonic()

bus = can.interface.Bus(bustype='slcan',
                        channel='/dev/tty.usbmodem14101',
                        bitrate=500000)

while 1:
    update_parameters()
    check_if_time_to_print()