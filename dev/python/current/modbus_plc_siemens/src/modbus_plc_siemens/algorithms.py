# --------------------------------------------------------------------------------

def pickup_block(self, coord, way):
    """
        Pick up and deliver the block to the conveyour
        :param coord: block' coordinates in the warehouse
        :type coord: list in range(0,1)
        :param way: way of moving the block
        :type way: int in range(1,5)
    """

    column = coord[0]
    line = coord[1]

    # move to necessary column
    # (column+3) - column' register
    if column != 1:
        self.set(3, column+3)

    # move to necessary cell
    # (line+7)*2+1) - line' register
    if line != 1:
        self.set(5, (line+7)*2+1)

    # pickup the block
    # ((line+8)*2) - line' register
    self.set(8, 25)
    self.set(5, (line+8)*2)
    self.set(7, 26)

    # deliver the block
    # (way*3-(4-way)//2) - navigation
    if column < (way*3-(4-way)//2):
        self.set(3, way+98)
    else:
        self.set(2, way+98)

    self.set(7, 27)
    self.set(6, 17)
    self.set(8, 26)

    # return to starting position
    self.post.set(2, 4)


# --------------------------------------------------------------------------------

def put_block(self, coord):
    """
        Deliver and put the block to the warehouse
        :param coord: cell' coordinates in the warehouse
        :type coord: list in range(0,1)
    """

    column = coord[0]
    line = coord[1]

    # pickup the block / move to necessary line
    # (line+43)*2+1) - line' register
    self.set(16, 96)
    self.set(13, (line+43)*2+1)
    self.set(15, 97)
    # move to necessary cell
    # (75+(12-column)) - column' register
    if column != 1:
        self.set(10, 75+(12-column))

    # put the block
    # (line+43)*2) - line' register
    self.set(15, 98)
    self.set(14, (line+43)*2)
    self.set(16, 97)
    # return to starting position
    if line != 1:
        self.set(14, 88)
    if column != 1:
        self.set(11, 86)


# --------------------------------------------------------------------------------

colors = (
    'black color',
    'yellow block',
    'blue block',
    'green block',
    'purple block',
    'unknown color'
)


def straight_move(self):
    """
        Move the block along the last conveyor' line
        Define color of the block
    """

    self.post.set(90, 72)
    self.set(93, 72)

    self.post.set(87, 70)
    self.sleep(1)

    # color recognition block
    print('- Color recognition block: ' + colors[self.get(0)])

    self.post.set(93, 73)

    # timer for right pick-up position
    self.set(95, 73, time=0.25)


# --------------------------------------------------------------------------------

def act0(self, loader, direction, point):
    """
        Helping method 0 - loader' moving between the conveyors
    """
    
    if direction not in range(2):
        print('- Error: incorrect direction')
        return
    if point not in ['a', 'b', 'c', 'd']:
        print('- Error: incorrect point')
        return
    # ------------------------------------
    if loader == 0:
        if point == 'a':
            self.set(3-direction, 99)
        elif point == 'b':
            self.set(3-direction, 100)
        elif point == 'c':
            self.set(3-direction, 101)
        elif point == 'd':
            self.set(3-direction, 102)
    elif loader == 1:
        if point == 'a':
            self.set(direction+10, 99)
        elif point == 'b':
            self.set(direction+10, 100)
        elif point == 'c':
            self.set(direction+10, 101)
        elif point == 'd':
            self.set(direction+10, 102)
    else:
        print('- Error: incorrect loader number')

def act1(self, loader, direction, point):
    """
        Helping method 1 - loader' moving along the warehouse
    """
    
    if direction not in range(2):
        print('- Error: incorrect direction')
        return
    if point not in range(1, 13):
        print('- Error: incorrect point')
        return
    # ------------------------------------
    if loader == 0:
        self.set(3-direction, point+3)
    elif loader == 1:
        self.set(direction+10, 75+(12-point))
    else:
        print('- Error: incorrect loader number')

def act2(self, loader, direction, point):
    """
        Helping method 2 - loader' moving up and down
    """
    
    if direction not in range(2):
        print('- Error: incorrect direction')
        return
    if point not in range(1, 9):
        print('- Error: incorrect point')
        return
    # ------------------------------------
    if loader == 0:
        self.set(direction+5, point+16)
    elif loader == 1:
        self.set(direction+13, point+87)
    else:
        print('- Error: incorrect loader number')

def act3(self, loader, action):
    """
        Helping method 3 - cariage' moving
    """
    
    if action not in range(1, 5):
        print('- Error: incorrect action')
        return
    # ------------------------------------
    if loader == 0:
        if action == 1:
            self.set(7, 26)
        elif action == 2:
            self.set(7, 27)
        elif action == 3:
            self.set(8, 26)
        elif action == 4:
            self.set(8, 25)
    elif loader == 1:
        if action == 1:
            self.set(16, 97)
        elif action == 2:
            self.set(16, 96)
        elif action == 3:
            self.set(15, 97)
        elif action == 4:
            self.set(15, 98)
    else:
        print('- Error: incorrect loader number')


# --------------------------------------------------------------------------------

def handlers_lights(self):
    """
        Control the handlers' lights independently
        - Downtime = Green color
        - Moving = Yellow color
        - Work = Red color
    """
    
    trig = [[True]*3]*4
    while True:

        # lights of the handler 1
        if self.get(33):
            # handler' moving (yellow)
            if self.get(36):
                if trig[0][0]:
                    self.set(27)
                    self.set(29, value=0)
                    trig[0] = [False, True, True]
            # handler' work (red)
            elif trig[0][1]:
                self.set(29)
                self.set(28, value=0)
                self.set(27, value=0)
                trig[0] = [True, False, True]
        # handler' downtime (green)
        elif trig[0][2]:
            self.set(28)
            self.set(29, value=0)
            trig[0] = [True, True, False]
        # ------------------------------------
        # lights of the handler 2
        if self.get(43):
            # handler' moving (yellow)
            if self.get(46):
                if trig[1][0]:
                    self.set(46)
                    self.set(48, value=0)
                    trig[1] = [False, True, True]
            # handler' work (red)
            elif trig[1][1]:
                self.set(48)
                self.set(47, value=0)
                self.set(46, value=0)
                trig[1] = [True, False, True]
        # handler' downtime (green)
        elif trig[1][2]:
            self.set(47)
            self.set(48, value=0)
            trig[1] = [True, True, False]
        # ------------------------------------
        # lights of the handler 3
        if self.get(53):
            # handler' moving (yellow)
            if self.get(56):
                if trig[2][0]:
                    self.set(65)
                    self.set(67, value=0)
                    trig[2] = [False, True, True]
            # handler' work (red)
            elif trig[2][1]:
                self.set(67)
                self.set(66, value=0)
                self.set(65, value=0)
                trig[2] = [True, False, True]
        # handler' downtime (green)
        elif trig[2][2]:
            self.set(66)
            self.set(67, value=0)
            trig[2] = [True, True, False]
        # ------------------------------------
        # lights of the handler 4
        if self.get(63):
            # handler' moving (yellow)
            if self.get(66):
                if trig[3][0]:
                    self.set(85)
                    self.set(86, value=0)
                    trig[3] = [False, True, True]
            # handler' work (red)
            elif trig[3][1]:
                self.set(86)
                self.set(85, value=0)
                self.set(84, value=0)
                trig[3] = [True, False, True]
        # handler' downtime (green)
        elif trig[3][2]:
            self.set(85)
            self.set(86, value=0)
            trig[3] = [True, True, False]

        self.sleep(0.001)


# --------------------------------------------------------------------------------

def run_a(self, pickup_coord, put_coord):
    """
        Pick up, deliver and put the block along the way A (handler 1)
        :param pickup_coord: block' coordinates in the warehouse
        :type pickup_coord: list in range(0,2)
        :param put_coord: cell' coordinates in the warehouse
        :type put_coord: list in range(0,2)
    """

    if (pickup_coord[0] not in range(1, 13)) or (pickup_coord[1] not in range(1, 5)):
        print('- Error: incorrect pickup-coordinates')
        return
    if (put_coord[0] not in range(1, 13)) or (put_coord[1] not in range(1, 5)):
        print('- Error: incorrect put-coordinates')
        return

    pickup_block(self, pickup_coord, 1)
    # ---------------------------------
    self.post.set(19, 33)
    self.set(21, 33)

    # handler work
    self.set(22, 34)
    self.set(25, 36)
    self.set(26, time=3)
    self.set(24, 37)
    self.set(23, 35)

    if self.get(39) == 0:
        self.set(31, 39)

    self.post.set(21, 38)
    self.set(33, 38)

    self.set(30, 40)
    # ---------------------------------

    # general line movement
    self.post.set(33, 41)
    self.set(35, 41)

    self.post.set(31, 39)

    if self.get(50) == 0:
        self.set(50, 50)

    self.post.set(35, 48)
    self.set(52, 48)

    self.post.set(52, 51)
    self.set(54, 51)

    if self.get(60) == 0:
        self.set(68, 60)

    self.post.set(54, 58)
    self.set(71, 58)

    self.post.set(71, 61)
    self.set(73, 61)

    if self.get(70) == 0:
        self.set(87, 70)

    self.post.set(73, 68)
    self.set(90, 68)

    # ---------------------------------
    self.set(88, 69)

    straight_move(self)
    put_block(self, put_coord)


# --------------------------------------------------------------------------------

def run_b(self, pickup_coord, put_coord):
    """
        Pick up, deliver and put the block along the way B (handler 2)
        :param pickup_coord: block' coordinates in the warehouse
        :type pickup_coord: list in range(0,2)
        :param put_coord: cell' coordinates in the warehouse
        :type put_coord: list in range(0,2)
    """

    if (pickup_coord[0] not in range(1, 13)) or (pickup_coord[1] not in range(1, 5)):
        print('- Error: incorrect pickup-coordinates')
        return
    if (put_coord[0] not in range(1, 13)) or (put_coord[1] not in range(1, 5)):
        print('- Error: incorrect put-coordinates')
        return

    pickup_block(self, pickup_coord, 2)
    # ---------------------------------
    self.post.set(38, 43)
    self.set(40, 43)

    # handler work
    self.set(41, 44)
    self.set(43, 46)
    self.set(45, time=3)
    self.set(44, 47)
    self.set(42, 45)

    if self.get(49) == 0:
        self.set(49, 49)

    self.post.set(40, 48)
    self.set(52, 48)

    self.set(50, 50)
    # ---------------------------------

    # general line movement
    self.post.set(52, 51)
    self.set(54, 51)

    if self.get(60) == 0:
        self.set(68, 60)

    self.post.set(54, 58)
    self.set(71, 58)

    self.post.set(71, 61)
    self.set(73, 61)

    if self.get(70) == 0:
        self.set(87, 70)

    self.post.set(73, 68)
    self.set(90, 68)

    # ---------------------------------
    self.set(88, 69)

    straight_move(self)
    put_block(self, put_coord)


# --------------------------------------------------------------------------------

def run_c(self, pickup_coord, put_coord):
    """
        Pick up, deliver and put the block along the way C (handler 3)
        :param pickup_coord: block' coordinates in the warehouse
        :type pickup_coord: list in range(0,2)
        :param put_coord: cell' coordinates in the warehouse
        :type put_coord: list in range(0,2)
    """

    if (pickup_coord[0] not in range(1, 13)) or (pickup_coord[1] not in range(1, 5)):
        print('- Error: incorrect pickup-coordinates')
        return
    if (put_coord[0] not in range(1, 13)) or (put_coord[1] not in range(1, 5)):
        print('- Error: incorrect put-coordinates')
        return

    pickup_block(self, pickup_coord, 3)
    # ---------------------------------
    self.post.set(57, 53)
    self.set(59, 53)

    # handler work
    self.set(60, 54)
    self.set(62, 56)
    self.set(64, time=3)
    self.set(63, 57)
    self.set(61, 55)

    if self.get(59) == 0:
        self.set(69, 59)

    self.post.set(59, 58)
    self.set(71, 58)

    self.set(68, 60)
    # ---------------------------------

    # general line movement
    self.post.set(71, 61)
    self.set(73, 61)

    if self.get(70) == 0:
        self.set(87, 70)

    self.post.set(73, 68)
    self.set(90, 68)

    # ---------------------------------
    self.set(88, 69)

    straight_move(self)
    put_block(self, put_coord)


# --------------------------------------------------------------------------------

def run_d(self, pickup_coord, put_coord):
    """
        Pick up, deliver and put the block along the way D (handler 4)
        :param pickup_coord: block' coordinates in the warehouse
        :type pickup_coord: list in range(0,2)
        :param put_coord: cell' coordinates in the warehouse
        :type put_coord: list in range(0,2)
    """
    
    if (pickup_coord[0] not in range(1, 13)) or (pickup_coord[1] not in range(1, 5)):
        print('- Error: incorrect pickup-coordinates')
        return
    if (put_coord[0] not in range(1, 13)) or (put_coord[1] not in range(1, 5)):
        print('- Error: incorrect put-coordinates')
        return

    pickup_block(self, pickup_coord, 4)
    # ---------------------------------
    self.post.set(76, 63)
    self.set(78, 63)
    
    # handler work
    self.set(79, 64)
    self.set(81, 66)
    self.set(96, time=1)
    self.set(83, time=3)
    self.set(82, 67)
    self.set(80, 65)

    if self.get(69) == 0:
        self.set(88, 69)

    self.post.set(78, 68)
    self.set(90, 68)

    straight_move(self)
    put_block(self, put_coord)
