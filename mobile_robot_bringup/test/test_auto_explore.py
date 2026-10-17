import math
import random

import auto_explore as a

N = 720
INC = 2 * math.pi / N
AMIN = -math.pi


def scan(front=None):
    r = [8.0] * N
    if front is not None:
        for i in range(N):
            if abs(AMIN + i * INC) < math.radians(40):
                r[i] = front
    return r


def test_drives_forward_when_free():
    ex = a.Explorer(random.Random(1))
    lin, ang = ex.step(scan(), AMIN, INC, 0.0)
    assert lin > 0 and ang == 0


def test_stops_then_turns_then_continues():
    ex = a.Explorer(random.Random(1))
    assert ex.step(scan(front=0.4), AMIN, INC, 0.0) == (0.0, 0.0) and ex.state == 'STOP'
    ex.step(scan(front=0.4), AMIN, INC, 0.7)
    assert ex.state == 'TURN'
    lin, ang = ex.step(scan(front=0.4), AMIN, INC, 0.8)
    assert lin == 0 and abs(ang) > 0
    lin, ang = ex.step(scan(), AMIN, INC, 4.0)
    assert ex.state == 'FORWARD' and lin > 0


def test_detects_being_stuck_and_backs_up():
    ex = a.Explorer(random.Random(2))
    t = 0.0
    ex.step(scan(), AMIN, INC, t, pos=(1.0, 1.0))
    t = 4.5
    lin, ang = ex.step(scan(), AMIN, INC, t, pos=(1.0, 1.0))        # told to drive, did not move
    assert ex.state == 'BACKUP' and (lin, ang) == (0.0, 0.0)
    lin, ang = ex.step(scan(), AMIN, INC, t + 0.1, pos=(1.0, 1.0))
    assert lin < 0                                                   # reversing
    ex.step(scan(), AMIN, INC, t + 2.0, pos=(1.0, 1.0))
    assert ex.state == 'TURN'


def test_does_not_back_into_a_wall():
    ex = a.Explorer(random.Random(2))
    ex.state, ex.until = 'BACKUP', 100.0
    r = scan()
    for i in range(N):
        if abs(abs(AMIN + i * INC) - math.pi) < math.radians(20):
            r[i] = 0.2
    lin, _ = ex.step(r, AMIN, INC, 1.0)
    assert lin == 0.0 and ex.state == 'TURN'
