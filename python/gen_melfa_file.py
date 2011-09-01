"""\
MELFA Basic program generation module.

@author: Jose Alarcon Herrera
@organization: University of Windsor
@contact: alarconj@uwindsor.ca
@license: GPL-3
"""

from optparse import OptionParser


def gen_melfa_file(filename, joints):
    """
    This function generates a file containing a program written in 
    MELFA-BASIC IV, the program is used to move the robot though
    a set of positions configured in joint angles.
    
    @param filename: The name of the output program
    @type filename: C{str}
    @param joints: The number of joints in the robot program.
    @type joints: C{int}
    """
    f = open(filename, 'w')
    f.write('10 OPEN "COM1:" AS #1\n')
    f.write('15 M1= 1\n')
    line = 20
    for i in range(1, joints + 1):
        f.write('%d MOV J%d\n' % (line, i))
        line += 5
        f.write('%d GOSUB %d\n' % (line, 25 + (10 * joints)))
        line += 5
    f.write('%d END\n' % line)
    f.write('%d PRINT #1, M1\n' % (line + 5))
    f.write('%d INPUT #1, M2\n' % (line + 10))
    f.write('%d M1 = M1 + 1\n' % (line + 15))
    f.write('%d RETURN' % (line + 20))
    f.close()

if __name__ == '__main__':
    parser = OptionParser()
    opts, args = parser.parse_args()
    try:
        filename = args[0]
        joints = int(args[1])
    except IndexError:
        filename = 'program.prg'
        joints = 1
    gen_melfa_file(filename, joints)