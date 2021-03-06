# desktop implementation of listener for VESC/ENNOID using Canable

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
                'controller_temperature':0,
                'dummy':0}

time_stamps = {'event_loop_current':0,
               'event_loop_previous':0,
               'event_loop_elapsed':0}

derived_data = {'internal_resistance':0.090,
                'speed':0,
                'distance':0,
                'battery_voltage_prev':0,
                'battery_current_prev':0,
                'charge':0,
                'energy':0,
                'trip_efficiency':0,
                'instantaneous_efficiency':0,
                'voltage_difference_mohm':0}

vehicle_parameters = {'wheel_circumference':1.89}

class DERIVED:
    def __init__(self):
        self.last_sample = time.monotonic()
        self.sampling_interval = 0.2

    def update(self):
        if time.monotonic() - self.last_sample > self.sampling_interval:
            self.last_sample = time.monotonic()
            self.compute_derived()
            print('o', end='')

    def compute_derived(self):
        derived_data['speed'] = vehicle_data['motor_rpm'] * vehicle_parameters['wheel_circumference'] / 23 / 60.0
        derived_data['distance'] += derived_data['speed']/1000.0

class CONSOLE:

    def __init__(self):
        self.display_update_seconds = 1.0
        self.last_display = time.monotonic()

    def update(self):
        if time.monotonic() - self.last_display > self.display_update_seconds:
            self.print_to_console()
            self.last_display = time.monotonic()

    def print_to_console(self):
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
        print('Tbat', vehicle_data['high_battery_temp']/1E2, end=' | ')
        print('TBMS', vehicle_data['high_BMS_temp']/1E2, end=' | ')
        print('Sp', derived_data['speed'], end=' | ')
        print('D', derived_data['distance'], end=' | ')
        print(time.monotonic())

class CANBUS:

    def __init__(self):

        self.bus = can.interface.Bus(bustype='slcan',
                                channel='/dev/tty.usbmodem14101',
                                bitrate=500000)

        self.packet_variables = {0x0901: [('motor_rpm', '>l', 0, 4),
                                          ('total_current', '>H', 4, 2)],
                                 0x1001: [('controller_temperature', '>H', 0, 2),
                                          ('motor_temperature', '>h', 2, 2)],
                                 0x1b01: [('battery_voltage', '>H', 4, 2)],
                                 0x1e0a: [('battery_voltage_BMS', '>i', 0, 4),
                                          ('battery_current_BMS', '>i', 4, 4)],
                                 0x1f0a: [('high_cell_voltage', '>i', 0, 4),
                                          ('low_cell_voltage', '>i', 4, 4)],
                                 0x210a: [('high_battery_temp', '>H', 2, 2),
                                          ('high_BMS_temp', '>H', 6, 2)],
                                         }

    def update(self):
        message = self.bus.recv(timeout=0.050)
        if message is not None:
            print('+', end='')
            # iterate over variables and store for expected messages
            if message.arbitration_id in self.packet_variables.keys():
                for pv in self.packet_variables[message.arbitration_id]:
                    vehicle_data[pv[0]] = struct.unpack(pv[1], message.data[pv[2]:pv[2]+pv[3]])[0]
        else:
            print('.', end='')

console = CONSOLE()
canbus = CANBUS()
derived = DERIVED()

print("ENNOID/VESC CAN reader")
while 1:
    canbus.update()
    console.update()
    derived.update()