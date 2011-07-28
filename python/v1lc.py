"""\
V1LC lens controller module.

@author: Jose Luis Alarcon Herrera
@organization: University of Windsor
@contact: alarconj@uwindsor.ca
@license: GPL-3
"""

import serial


class LensController(object):
    """
    V1LC lens controller class.
    """
    def __init__(self, port):
        """
        Constructor.

        @param port: Serial port of the V1LC.
        @type port: C{str}
        """
        self.port = serial.Serial(port=port, baudrate=9600, 
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, 
            stopbits=serial.STOPBITS_ONE)
        self.port.write('T255\n')

    def query_status(self):
        """
        Query the status of the zoom, focus, and iris of the lens.
       
        @return: Status of the lens controller.
        @rtype: C{tuple} of C{int}
        """
        self.port.flushInput()
        self.port.write('?\n')
        state = []
        line = self.port.readline()
        for i in range(3):
            line = self.port.readline()
            value = line.partition(':')[2]
            state.append(value.partition('\r')[0])
        Z = int(state[0])
        F = int(state[1])
        I = int(state[2])
        return Z, F, I
    
    @property
    def Z(self):
        """\
        The Zoom position of the lens.

        @rtype: C{int}
        """
        z, f, i = self.query_status()
        return z
        
    @property
    def F(self):
        """\
        The Focus value of the lens.

        @rtype: C{int}
        """
        z, f, i = self.query_status()
        return f
        
    @property
    def I(self):
        """\
        The state of the Iris of the lens.

        @rtype: C{int}
        """
        
        z, f, i = self.query_status()
        return i
    
    def zoom(self, value):
        """
        Zoom the lens to the specified level.
        
        @param value: The zoom value.
        @type value: C{int}
        """
        if value >= 44 and value <=180:
            self.port.write('Z%03d\n' % value)
        else:
            print 'The range must be between 44 and 180'

    def focus(self, value):
        """
        Focus the lens to the specified level.
        
        @param value: The focus value.
        @type value: C{int}
        """
        if value >= 44 and value <=180:
            self.port.write('F%03d\n' % value)
        else:
            print 'The range must be between 44 and 180'

    def iris(self, state='open'):
        """
        Iris the lens to either open or close state.
        
        @param state: The iris state.
        @type state: C{string}
        """
        if state == 'close':
            self.port.write('I0\n')
        elif state == 'open':
            self.port.write('I100\n')
        else:
            print 'The only options are \'close\' and \'open\'.'
