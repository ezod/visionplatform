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
    def __init__(self, message, Errors):
        """
        Constructor.
        
        @param message: input expression in which the error occurred
        @type message: C{str}
        @param Errors: explanation of the error
        @type Errors: C{dict}
        """
        Exception.__init__(self, message)
        self.Errors = Errors

class Servo(object):
    """
    Servo object class for the servo controller class.
    """
    def __init__(self, id, pos):
        """
        Constructor.
        
        @param id: The identification number of the servo.
        @type id: C{int}
        @param pos: The initial position of the servo.
        @type pos: C{int}
        """
        
        self.marker = 255
        if id >= 0 and id <= 7:
            self.servo_id = id
        else:
            raise InputError(
                'The identification range is 0 - 7', 
                {1001:'The identification number is out of range'})
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
        Update the position variable of the servo class.
        
        @param pos: The position.
        @type pos: C{int}
        """
        
        if pos >= 0 and pos <= 255:
            self.pos = pos
        else:
            raise InputError(
                'The position range is 0 - 255', 
                {1002:'The position is out of range.'})
    
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
            
            self.port = serial.Serial(port=port, baudrate=9600, 
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, 
                stopbits=serial.STOPBITS_ONE)
        except InputError:
            print InputError.message
    
    def where(self, servo):
        """
        The last position sent to the servo.
        
        @param servo: The servo.
        @type servo: C{int}
        @return: The last position sent to the servo.
        @rtype: C{int}
        """
        
        return self.servos[servo].wh
    
    def move(self, servo, pos):
        """
        Move the servo to the desired position.
        
        @param servo: The servo.
        @type servo: C{int}
        @param pos: The position.
        @type pos: C{int}
        """
        
        try:
            self.servos[servo].update_pos(pos)
            self.port.write(chr(255) + chr(self.servos[servo].id) + 
                chr(self.servos[servo].wh))
        except InputError:
            print InputError.message
    