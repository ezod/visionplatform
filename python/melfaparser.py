"""\
HALCON parameter parser module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

def parse_positions(filename):
    """\
    Parse positions.

    @param filename: The calibration output file to load from.
    @type filename: C{str}
    @return: A dict containing the positions by identifier.
    @rtype: C{dict}
    """
    positions = {}
    with open(filename, 'r') as prgfile:
        for line in prgfile:
            if line.startswith('J'):
                line = line.rstrip().split('=')
                positions[line[0]] = \
                    [float(s) for s in line[1].strip('()').split(',')]
            # TODO: elif line.startswith('P')
    return positions
