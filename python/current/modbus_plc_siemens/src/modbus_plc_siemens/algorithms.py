from modbus_plc_siemens.r_api import *


def pickup_block(way):
    # move to necessary column (column №3)
    r_set(3, 1, 6)
    # move to necessary cell (line №4)
    r_set(5, 1, 23)
    # pickup the block
    r_set(8, 1, 25)
    r_set(5, 1, 24)
    r_set(7, 1, 26)
    # deliver the block
    if way == 1:
        r_set(2, 1, 99)
    elif way == 2:
        r_set(3, 1, 100)
    elif way == 3:
        r_set(3, 1, 101)
    elif way == 4:
        r_set(3, 1, 102)

    r_set(7, 1, 27)
    r_set(6, 1, 17)
    r_set(8, 1, 26)

    # return to starting position
    r_post(r_set, (2, 1, 4))


# ---------------------------------------

def put_block():
    # pickup the block
    r_set(16, 1, 96)
    r_set(13, 1, 89)
    r_set(15, 1, 97)
    # move to necessary column (column №9)
    r_set(10, 1, 83)
    # move to necessary cell (line №4)
    r_set(13, 1, 95)
    # put the block
    r_set(15, 1, 98)
    r_set(14, 1, 94)
    r_set(16, 1, 97)
    # return to starting position
    r_set(14, 1, 88)
    r_set(11, 1, 86)


# ---------------------------------------

def run_a():
    pickup_block(1)
    # ----------------------------
    r_post(r_set, (19, 1, 33))
    r_set(21, 1, 33)

    # handler work
    r_set(22, 1, 34)
    r_set(25, 1, 36)
    r_set(26, 1, time=1)
    r_set(24, 1, 37)
    r_set(23, 1, 35)

    if r_get(39) == 1:
        r_set(31, 1, 39)

    r_post(r_set, (21, 1, 38))
    r_set(33, 1, 38)

    r_set(30, 1, 40)
    # ----------------------------

    r_post(r_set, (33, 1, 41))
    r_set(35, 1, 41)

    if r_get(50) == 1:
        r_set(50, 1, 50)

    r_post(r_set, (35, 1, 48))
    r_set(52, 1, 48)

    r_post(r_set, (52, 1, 51))
    r_set(54, 1, 51)

    if r_get(60) == 1:
        r_set(68, 1, 60)

    r_post(r_set, (54, 1, 58))
    r_set(71, 1, 58)

    r_post(r_set, (71, 1, 61))
    r_set(73, 1, 61)

    if r_get(70) == 1:
        r_set(87, 1, 70)

    r_post(r_set, (73, 1, 68))
    r_set(90, 1, 68)

    # ----------------------------
    r_set(88, 1, 69)

    r_post(r_set, (90, 1, 72))
    r_set(93, 1, 72)

    r_post(r_set, (87, 1, 70))
    r_sleep(1)  # color detection

    r_post(r_set, (93, 1, 73))
    r_set(95, 1, 73)
    # ----------------------------
    put_block()


# ---------------------------------------

def run_b():
    pickup_block(2)
    # ----------------------------
    r_post(r_set, (38, 1, 43))
    r_set(40, 1, 43)

    # handler work
    r_set(41, 1, 44)
    r_set(43, 1, 46)
    r_set(45, 1, time=1)
    r_set(44, 1, 47)
    r_set(42, 1, 45)

    if r_get(49) == 1:
        r_set(49, 1, 49)

    r_post(r_set, (40, 1, 48))
    r_set(52, 1, 48)

    r_set(50, 1, 50)
    # ----------------------------

    r_post(r_set, (52, 1, 51))
    r_set(54, 1, 51)

    if r_get(60) == 1:
        r_set(68, 1, 60)

    r_post(r_set, (54, 1, 58))
    r_set(71, 1, 58)

    r_post(r_set, (71, 1, 61))
    r_set(73, 1, 61)

    if r_get(70) == 1:
        r_set(87, 1, 70)

    r_post(r_set, (73, 1, 68))
    r_set(90, 1, 68)

    # ----------------------------
    r_set(88, 1, 69)

    r_post(r_set, (90, 1, 72))
    r_set(93, 1, 72)

    r_post(r_set, (87, 1, 70))
    r_sleep(1)  # color detection

    r_post(r_set, (93, 1, 73))
    r_set(95, 1, 73)
    # ----------------------------
    put_block()


# ---------------------------------------

def run_c():
    pickup_block(3)
    # ----------------------------
    r_post(r_set, (57, 1, 53))
    r_set(59, 1, 53)

    # handler work
    r_set(60, 1, 64)
    r_set(62, 1, 56)
    r_set(64, 1, time=1)
    r_set(63, 1, 57)
    r_set(61, 1, 65)

    if r_get(59) == 1:
        r_set(69, 1, 59)

    r_post(r_set, (59, 1, 58))
    r_set(71, 1, 58)

    r_set(68, 1, 60)
    # ----------------------------

    r_post(r_set, (71, 1, 61))
    r_set(73, 1, 61)

    if r_get(70) == 1:
        r_set(87, 1, 70)

    r_post(r_set, (73, 1, 68))
    r_set(90, 1, 68)

    # ----------------------------
    r_set(88, 1, 69)

    r_post(r_set, (90, 1, 72))
    r_set(93, 1, 72)

    r_post(r_set, (87, 1, 70))
    r_sleep(1)  # color detection

    r_post(r_set, (93, 1, 73))
    r_set(95, 1, 73)
    # ----------------------------
    put_block()


# ---------------------------------------

def run_d():
    pickup_block(4)
    # ----------------------------
    r_post(r_set, (76, 1, 63))
    r_set(78, 1, 63)

    # handler work
    r_set(79, 1, 64)
    r_set(81, 1, 66)
    r_set(83, 1, time=1)
    r_set(96, 1, time=1)
    r_set(82, 1, 67)
    r_set(80, 1, 65)

    if r_get(69) == 1:
        r_set(88, 1, 69)

    r_post(r_set, (78, 1, 68))
    r_set(90, 1, 68)

    r_post(r_set, (90, 1, 72))
    r_set(93, 1, 72)

    r_set(30, 1, 70)
    r_sleep(1)  # color detection

    r_post(r_set, (93, 1, 73))
    r_set(95, 1, 73)
    # ----------------------------
    put_block()
