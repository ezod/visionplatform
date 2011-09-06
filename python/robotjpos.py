"""\
Basic robot pose query example program.

The serial port exchange for querying the robot's joint position is as follows:

SEND: 1;1;JPOSF.

RECV: QoKJ1;10.90;J2;70.76;J3;64.16;J4;-160.00;J5;40.06;J6;-183.40;;****,****;\
      20;30.28;00000000

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import serial
from optparse import OptionParser

if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option('-s', '--serialport', dest='serialport', action='store',
        default='COM1', help='serial port for robot')
    opts, args = parser.parse_args()
    port = serial.Serial(port=opts.serialport, baudrate=19200)
    while True:
        port.write('1;1;JPOSF.\r')
        rawpos = port.read(128).split(';;')[0][3:].split(';')
        pos = [float(rawpos[2 * i + 1]) for i in range(6)]
        print(pos)
