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

import sys
import serial
from optparse import OptionParser


def query_position(port):
    """\
    Query the robot position (given an open serial connection to the robot
    controller).

    @param port: Open serial connection to the robot controller.
    @type port: L{serial.Serial}
    @return 
    """
    port.write('1;1;JPOSF.\r')
    rawpos = ''
    while True:
        rawpos += port.read(128)
        try:
            if len(rawpos) > 3 and not rawpos.startswith('QoK'):
                raise ValueError('non-QoK response from robot')
            splitpos = rawpos.split(';;')[0][3:].split(';')
            return [float(splitpos[2 * i + 1])  for i in range(6)]
        except IndexError:
            continue


if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option('-s', '--serialport', dest='serialport', action='store',
        default='COM1', help='serial port for robot')
    opts, args = parser.parse_args()
    port = serial.Serial(port=opts.serialport, baudrate=19200, timeout=0.3)
    if len(args):
        f = open(args[0], 'w')
    else:
        f = sys.stdout
    p = 1
    try:
        while True:
            try:
                pos = (p,) + tuple(query_position(port))
            except ValueError:
                continue
            f.write('J%i=(%+.2f,%+.2f,%+.2f,%+.2f,%+.2f,%+.2f)\n' % pos)
            p += 1
    finally:
        port.close()
        f.close()
