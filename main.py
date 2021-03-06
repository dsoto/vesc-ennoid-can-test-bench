# desktop implementation of listener for VESC/ENNOID CAN packets

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
                'instantaneous_efficiency':0}

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
        self.display_update_seconds = 0.2
        self.last_display = time.monotonic()
        # (key, data dictionary, display abbreviation, precision)
        self.display = [
                        ('speed',     derived_data, 'S',   1),
                        ('motor_rpm', vehicle_data, 'rpm', 0),
                        ('high_cell_voltage',     vehicle_data, 'Vc', 3),
                        ('low_cell_voltage',     vehicle_data, 'Vc', 3),
                        ('battery_voltage',     vehicle_data, 'Vc', 1),
                        ('battery_voltage_BMS', vehicle_data, 'Vb', 1),
                        ('battery_current',     vehicle_data, 'Ic', 1),
                        ('battery_current_BMS', vehicle_data, 'Ib', 1),
                        ('total_current',       vehicle_data, 'Im', 1),
                        ('controller_temperature', vehicle_data, 'Tc', 1),
                        ('motor_temperature', vehicle_data, 'Tm', 1),
                        ('high_battery_temp', vehicle_data, 'Tbat', 1),
                        ('high_BMS_temp', vehicle_data, 'Tbms', 1),
                        ('distance', derived_data, 'D', 1),
                        ]
    def update(self):
        if time.monotonic() - self.last_display > self.display_update_seconds:
            self.print_to_console()
            self.last_display = time.monotonic()


    def print_to_console(self):
        print()
        for l in self.display:
            # output abbreviation and formatted float based on specification
            print(f'{l[2]} {l[1][l[0]]:.{l[3]}f}', end=' | ')

        print(f'{time.monotonic():.3f}')

class CANBUS:

    def __init__(self):

        self.bus = can.interface.Bus(bustype='slcan',
                                channel='/dev/tty.usbmodem14101',
                                bitrate=500000)

        # define CAN messages to interpret
        self.packet_variables = {0x0901: [('motor_rpm',     '>l', 0, 4, 1E0),
                                          ('total_current', '>H', 4, 2, 1E1)],
                                 0x1001: [('controller_temperature', '>H', 0, 2, 1E1),
                                          ('motor_temperature',      '>h', 2, 2, 1E1)],
                                 0x1b01: [('battery_voltage',     '>H', 4, 2, 1E1)],
                                 0x1e0a: [('battery_voltage_BMS', '>i', 0, 4, 1E5),
                                          ('battery_current_BMS', '>i', 4, 4, 1E5)],
                                 0x1f0a: [('high_cell_voltage', '>i', 0, 4, 1E5),
                                          ('low_cell_voltage',  '>i', 4, 4, 1E5)],
                                 0x210a: [('high_battery_temp', '>H', 2, 2, 1E2),
                                          ('high_BMS_temp',     '>H', 6, 2, 1E2)]}

    def update(self):
        message = self.bus.recv(timeout=0.050)
        if message is not None:
            print('+', end='')
            # iterate over variables and store for expected messages
            if message.arbitration_id in self.packet_variables.keys():
                for pv in self.packet_variables[message.arbitration_id]:
                    vehicle_data[pv[0]] = struct.unpack(pv[1], message.data[pv[2]:pv[2]+pv[3]])[0]/pv[4]
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