"""
Mini SSC II serial servo controller module.

@author: Jose Alarcon Herrera
@organization: University of Windsor
@contact: alarconj@uwindsor.ca
@license: GPL-3
"""

import serial


class InputError(Exception):
    """
    Exception raised for errors in the input.
    """
    def __init__(self, message):
        """
        Constructor.
        
        @param message: input expression in which the error occurred
        @type message: C{str}
        """
        self.message = message
    
    def __str__(self):
        return repr(self.message)

class Servo(object):
    """
    Servo object class for the servo controller class.
    """
    def __init__(self, servo_id, pos):
        """
        Constructor.
        
        @param servo_id: The identification number of the servo.
        @type servo_id: C{int}
        @param pos: The initial position of the servo.
        @type pos: C{int}
        """
        
        self.marker = 255
        if servo_id >= 0 and servo_id <= 7:
            self.servo_id = servo_id
        else:
            raise InputError('The identification range is 0 - 7')
        self.pos = pos
    
    @property
    def id(self):
        """
        The servo identification number.

        @rtype: C{int}
        """
        return self.servo_id
    
    @property
    def wh(self):
        """
        The last position sent to the servo.

        @rtype: C{int}
        """
        return self.pos
    
    def update_pos(self, pos):
        """
        Update the position variable of the servo.
        
        @param pos: The position.
        @type pos: C{int}
        """
        
        if pos >= 0 and pos <= 250:
            self.pos = pos
        else:
            raise InputError('The position range is between 0 and \
                90 degrees.')
    
class ServoController(object):
    """
    Mini SSC II servo controller class.
    """
    def __init__(self, port, num=1):
        """
        Constructor.

        @param port: Serial port of the Mini SSC II.
        @type port: C{str}
        @param num: The number of servos to be used.
        @type num: C{int}
        """
        
        try:
            self.servos = []
            for i in range(num):
                servo = Servo(i, 127)
                self.servos.append(servo)
            
            self.port = serial.Serial(port=port, baudrate=9600, \
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, \
                stopbits=serial.STOPBITS_ONE)
        except InputError, (instance):
            print instance.message
    
    def where(self, servo):
        """
        The last position sent to the servo expressed in degrees.
        
        @param servo: The servo.
        @type servo: C{int}
        @return: The last position sent to the servo.
        @rtype: C{float}
        """
        return self.servos[servo].wh * 0.36
    
    def move(self, servo, pos_deg):
        """
        Move the servo to the desired position in degrees.
        
        @param servo: The servo.
        @type servo: C{int}
        @param pos_deg: The position in degrees.
        @type pos_deg: C{float}
        """
        
        pos = int(round(pos_deg / 0.36))
        try:
            self.servos[servo].update_pos(pos)
            self.port.write(chr(255) + chr(self.servos[servo].id) + \
                chr(self.servos[servo].wh))
        except InputError, (instance):
            print instance.message
    
    def home(self, servos):
        """
        Move the servo or servos to the initial position.
        
        @param servos: The servo or servos to be moved.
        @type servos: C{int, list, str}
        """
        
        if servos == 'all':
            for servo in range(len(self.servos)):
                self.move(self.servos[servo].id, 45.72)
        elif isinstance(servos, list):
            for servo in servos:
                self.move(self.servos[servo].id, 45.72)
        elif isinstance(servos, int):
            self.move(self.servos[servos].id, 45.72)
    