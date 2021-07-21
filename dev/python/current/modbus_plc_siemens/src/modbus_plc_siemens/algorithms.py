# --------------------------------------------------------------------------------

def pickup_block(self, coord, way):
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
    column = coord[0]
    line = coord[1]
    
    # pickup the block / move to necessary line
    # (line+43)*2+1) - line' register
    self.set(16, 96)
    self.set(13, (line+43)*2+1)
    self.set(15, 97)
    # move to necessary cell
    # (column+74) - column' register
    if column != 12:
        self.set(10, column+74)
    
    # put the block
    # (line+43)*2) - line' register
    self.set(15, 98)
    self.set(14, (line+43)*2)
    self.set(16, 97)
    # return to starting position
    if line != 1:
        self.set(14, 88)
    self.set(11, 86)


# --------------------------------------------------------------------------------

colors = (
    'Black color',
    'Yellow block',
    'Blue block',
    'Green block',
    'Purple block',
    'Unknown color'
)
def straight_move(self):
    self.post.set(90, 72)
    self.set(93, 72)

    self.post.set(87, 70)
    self.sleep(1)
    
    # color detection
    print(colors[self.get(0)])

    self.post.set(93, 73)

    # timer for right pick-up position
    self.set(95, 73, time=0.25)


# --------------------------------------------------------------------------------

def run_a(self, pickup_coord, put_coord):
    if (pickup_coord[0] not in range(1,13)) or (pickup_coord[1] not in range(1,5)):
        print('Incorrect pickup-coordinates')
        return
    if (put_coord[0] not in range(1,13)) or (put_coord[1] not in range(1,5)):
        print('Incorrect put-coordinates')
        return
    
    pickup_block(self, pickup_coord, 1)
    # ---------------------------------
    self.post.set(19, 33)
    self.set(21, 33)

    # handler work
    self.set(22, 34)
    self.set(25, 36)
    self.set(26, time=1)
    self.set(24, 37)
    self.set(23, 35)

    if self.get(39) == 1:
        self.set(31, 39)

    self.post.set(21, 38)
    self.set(33, 38)

    self.set(30, 40)
    # ---------------------------------
    
    # general line movement
    self.post.set(33, 41)
    self.set(35, 41)

    if self.get(50) == 1:
        self.set(50, 50)

    self.post.set(35, 48)
    self.set(52, 48)

    self.post.set(52, 51)
    self.set(54, 51)

    if self.get(60) == 1:
        self.set(68, 60)

    self.post.set(54, 58)
    self.set(71, 58)

    self.post.set(71, 61)
    self.set(73, 61)

    if self.get(70) == 1:
        self.set(87, 70)

    self.post.set(73, 68)
    self.set(90, 68)

    # ---------------------------------
    self.set(88, 69)

    straight_move(self)
    put_block(self, put_coord)


# --------------------------------------------------------------------------------

def run_b(self, pickup_coord, put_coord):
    if (pickup_coord[0] not in range(1,13)) or (pickup_coord[1] not in range(1,5)):
        print('Incorrect pickup-coordinates')
        return
    if (put_coord[0] not in range(1,13)) or (put_coord[1] not in range(1,5)):
        print('Incorrect put-coordinates')
        return
    
    pickup_block(self, pickup_coord, 2)
    # ---------------------------------
    self.post.set(38, 43)
    self.set(40, 43)

    # handler work
    self.set(41, 44)
    self.set(43, 46)
    self.set(45, time=1)
    self.set(44, 47)
    self.set(42, 45)

    if self.get(49) == 1:
        self.set(49, 49)

    self.post.set(40, 48)
    self.set(52, 48)

    self.set(50, 50)
    # ---------------------------------

    # general line movement
    self.post.set(52, 51)
    self.set(54, 51)

    if self.get(60) == 1:
        self.set(68, 60)

    self.post.set(54, 58)
    self.set(71, 58)

    self.post.set(71, 61)
    self.set(73, 61)

    if self.get(70) == 1:
        self.set(87, 70)

    self.post.set(73, 68)
    self.set(90, 68)

    # ---------------------------------
    self.set(88, 69)

    straight_move(self)
    put_block(self, put_coord)


# --------------------------------------------------------------------------------

def run_c(self, pickup_coord, put_coord):
    if (pickup_coord[0] not in range(1,13)) or (pickup_coord[1] not in range(1,5)):
        print('Incorrect pickup-coordinates')
        return
    if (put_coord[0] not in range(1,13)) or (put_coord[1] not in range(1,5)):
        print('Incorrect put-coordinates')
        return
    
    pickup_block(self, pickup_coord, 3)
    # ---------------------------------
    self.post.set(57, 53)
    self.set(59, 53)

    # handler work
    self.set(60, 54)
    self.set(62, 56)
    self.set(64, time=1)
    self.set(63, 57)
    self.set(61, 55)

    if self.get(59) == 1:
        self.set(69, 59)

    self.post.set(59, 58)
    self.set(71, 58)

    self.set(68, 60)
    # ---------------------------------

    # general line movement
    self.post.set(71, 61)
    self.set(73, 61)

    if self.get(70) == 1:
        self.set(87, 70)

    self.post.set(73, 68)
    self.set(90, 68)

    # ---------------------------------
    self.set(88, 69)

    straight_move(self)
    put_block(self, put_coord)


# --------------------------------------------------------------------------------

def run_d(self, pickup_coord, put_coord):
    if (pickup_coord[0] not in range(1,13)) or (pickup_coord[1] not in range(1,5)):
        print('Incorrect pickup-coordinates')
        return
    if (put_coord[0] not in range(1,13)) or (put_coord[1] not in range(1,5)):
        print('Incorrect put-coordinates')
        return
    
    pickup_block(self, pickup_coord, 4)
    # ---------------------------------
    self.post.set(76, 63)
    self.set(78, 63)

    # handler work
    self.set(79, 64)
    self.set(81, 66)
    self.set(96, time=1)
    self.set(83, time=1)
    self.set(82, 67)
    self.set(80, 65)

    if self.get(69) == 1:
        self.set(88, 69)

    self.post.set(78, 68)
    self.set(90, 68)

    straight_move(self)
    put_block(self, put_coord)
