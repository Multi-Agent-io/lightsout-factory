# --------------------------------------------------------------------------------

ways = {'a': 1, 'b': 2, 'c': 3, 'd': 4}

loader_0_tasks = []
loader_1_tasks = []

conveyors = dict.fromkeys(('1.1', '1.2', '1.4',
                           '2.1', '2.2', '2.4',
                           '3.1', '3.2', '3.4',
                           '4.1', '4.2', '4.4', '4.5'), False)

queues = {'2.3': [], '3.3': [], '4.3': []}

triggers = dict.fromkeys(('R/S', 'loader 0', 'loader 1', 'lights'), False)


##################################################################################

######################################
#          SERVICE METHODS           #
######################################

def show_warehouse(self):
    """
        Show all warehouse' cells in the table
        :param self: RApi object
    """

    table = self.execute('SELECT * FROM warehouse_arrival AS arr ORDER BY arr.column', True)

    print('\n' + '-'*49 + '\n|' + ' '*11 + '| line_1 | line_2 | line_3 | line_4 |\n' + '-'*49)

    for row in table:
        print(f'| column_{row[0]:<2} |{row[1]:^8}|{row[2]:^8}|{row[3]:^8}|{row[4]:^8}|\n' + '-'*49)
    print()


# --------------------------------------------------------------------------------

def clear_warehouse(self):
    """
        Clear all warehouse' cells in the database
        :param self: RApi object
    """

    self.execute('UPDATE warehouse_arrival SET line_1 = 0, line_2 = 0, line_3 = 0, line_4 = 0')
    print('- Database is cleared!')


# --------------------------------------------------------------------------------

def add_tasks(self, *tasks):
    """
        Add new tasks to the queue
        :param self: RApi object
        :param tasks: new tasks for the factory
        :type tasks: tuple of tuples
    """

    for task in tasks:
        if (not isinstance(task, tuple)) or (len(task) != 2) or (not isinstance(task[0], tuple)) or (len(task[0]) != 2):
            print('- Error: incorrect tasks structure!')
            return
        elif (task[0][0] not in range(1, 13)) or (task[0][1] not in range(1, 5)) or \
                (task[1] not in ('a', 'b', 'c', 'd')):
            print('- Error: incorrect coordinates or ways!')
            return

    loader_0_tasks.extend(tasks)
    print('- New tasks is added to the queue')
    # algorithm for making optimized queue...
    # conditions: distance, delay, busyness, etc


##################################################################################

######################################
#          HELPING METHODS           #
######################################

def act0(self, direction, point):
    """
        Helping method 0 - loader' moving between the conveyors
    """

    if triggers['R/S']:
        print('- Warning: factory is running!')
        return
    if direction not in range(2):
        print('- Error: incorrect direction')
        return
    if point not in ['a', 'b', 'c', 'd']:
        print('- Error: incorrect point')
        return
    # ------------------------------------
    if point == 'a':
        self.set(3-direction, 99)
    elif point == 'b':
        self.set(3-direction, 100)
    elif point == 'c':
        self.set(3-direction, 101)
    elif point == 'd':
        self.set(3-direction, 102)


def act1(self, loader, direction, point):
    """
        Helping method 1 - loader' moving along the warehouse
    """

    if triggers['R/S']:
        print('- Warning: factory is running!')
        return
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

    if triggers['R/S']:
        print('- Warning: factory is running!')
        return
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
        Helping method 3 - carriage' moving
    """

    if triggers['R/S']:
        print('- Warning: factory is running!')
        return
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


##################################################################################

######################################
#         RUN / STOP METHODS         #
######################################

def run_factory(self):
    """
        Run the factory
        :param self: RApi object
    """

    if not triggers['R/S']:
        triggers['loader 0'] = True
        triggers['loader 1'] = True
        triggers['lights'] = True

        self.post.run_loader_0()
        self.post.run_loader_1()
        self.post.run_lights()

        triggers['R/S'] = True
        print('- Factory is running!')
    else:
        print('- Warning: factory is already running!')


# --------------------------------------------------------------------------------

def stop_factory(self):
    """
        Stop the factory
        :param self: RApi object
    """

    if triggers['R/S']:
        if not loader_0_tasks and not loader_1_tasks and not any(conveyors.values()):
            triggers['loader 0'] = False
            triggers['loader 1'] = False

            # return loader 0 to starting position
            self.set(2, 4)
            triggers['lights'] = False

            triggers['R/S'] = False
            print('- Factory is stopped!')
        else:
            print('- Warning: not all tasks are completed!')
    else:
        print('- Warning: factory has already stopped!')


##################################################################################

######################################
#        MAIN RUNNING METHODS        #
######################################

def run_loader_0(self):
    """
        Pick up and deliver blocks to the conveyor
        :param self: RApi object
    """

    start = 0

    while triggers['loader 0']:
        if loader_0_tasks:

            column = loader_0_tasks[0][0][0]
            line = loader_0_tasks[0][0][1]
            way = ways[loader_0_tasks[0][1]]

            # move to necessary column
            # (column+3) - column' register
            # (way*3-(4-way)//2) - navigation
            if not start:
                if column != 1:
                    self.set(3, column+3)
            else:
                if column < (start*3-(4-start)//2):
                    self.set(2, column+3)
                else:
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

            start = way
            conveyors[str(way)+'.1'] = True
            # run the line in parallel
            exec('self.post.run_line_'+loader_0_tasks[0][1]+'()')

            loader_0_tasks.pop(0)

        self.sleep(0.001)


# --------------------------------------------------------------------------------

def run_loader_1(self):
    """
        Deliver and put blocks to the warehouse
        :param self: RApi object
    """

    while triggers['loader 1']:
        if loader_1_tasks:

            color = loader_1_tasks[0]
            column = None
            line = None

            # check necessary cells
            table = self.execute('SELECT * FROM warehouse_arrival AS arr ORDER BY arr.column', True)
            for row in table:
                if row[0] not in (color, 4+color, 8+color):
                    continue
                for cell in range(4):
                    if not row[cell+1]:
                        column = row[0]
                        line = cell+1
                        break
                else:
                    continue
                break

            while self.get(73):
                self.sleep(0.001)

            # pickup the block / move to necessary line
            # (line+43)*2+1) - line' register
            self.set(16, 96)
            self.set(13, (line+43)*2+1)
            self.set(15, 97)
            conveyors['4.5'] = False

            # move to necessary cell
            # (75+(12-column)) - column' register
            if column != 1:
                self.set(10, 75+(12-column))

            # put the block
            # (line+43)*2) - line' register
            self.set(15, 98)
            self.set(14, (line+43)*2)
            self.set(16, 97)

            # block actual cell
            self.execute(
                f'UPDATE warehouse_arrival AS arr SET line_{line} = 1 WHERE arr.column = {column}')

            # return to starting position
            if line != 1:
                self.set(14, 88)
            if column != 1:
                self.set(11, 86)

            loader_1_tasks.pop(0)

        self.sleep(0.001)


# --------------------------------------------------------------------------------

def run_lights(self):
    """
        Control the handlers' lights independently:
        - Downtime = Green color
        - Moving = Yellow color
        - Work = Red color
        :param self: RApi object
    """

    trig = [[True]*3]*4
    while triggers['lights']:

        # lights of the handler 1
        if not self.get(35):
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
        if not self.get(45):
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
        if not self.get(55):
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
        if not self.get(65):
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


##################################################################################

######################################
#         CONVEYORS METHODS          #
######################################

def define_color(self):
    """
        Move the block along the last conveyor' line
        Define color of the block
        :param self: RApi object
    """

    # move from 4.3 to 4.4
    self.post.set(90, 72)
    self.set(93, 72)
    # rotate 4.3 to 0
    self.post.set(87, 70)

    # color recognition block
    self.sleep(1)
    loader_1_tasks.append(self.get(0))

    while conveyors['4.5']:
        self.sleep(0.001)
    conveyors['4.5'] = True

    # move from 4.4 to 4.5
    self.post.set(93, 73)
    self.set(95, 73, time=0.25)
    conveyors['4.4'] = False


# --------------------------------------------------------------------------------

def run_line_a(self):
    """
        Deliver the block along the way A (handler 1)
        :param self: RApi object
    """

    while conveyors['1.2']:
        self.sleep(0.001)
    conveyors['1.2'] = True

    # move from 1.1 to 1.2
    self.post.set(19, 33)
    self.set(21, 33)
    conveyors['1.1'] = False

    # handler work
    self.set(22, 34)
    self.set(24, 36)
    self.set(26, time=3)
    self.set(25, 37)
    self.set(23, 35)

    while conveyors['1.4']:
        self.sleep(0.001)
    conveyors['1.4'] = True

    # rotate 1.3 to 0
    if self.get(39) == 0:
        self.set(31, 39)
    # move from 1.2 to 1.3
    self.post.set(21, 38)
    self.set(33, 38)
    conveyors['1.2'] = False
    # rotate 1.3 to 1
    self.set(30, 40)

    # move from 1.3 to 1.4
    self.post.set(33, 41)
    self.set(35, 41)
    queues['2.3'].append(1)
    # rotate 1.3 to 0
    self.post.set(31, 39)
    
    while (conveyors['2.4']) or (queues['2.3'][0] != 1):
        self.sleep(0.001)
    conveyors['2.4'] = True
    queues['2.3'].pop(0)

    # rotatet 2.3 to 1
    if self.get(50) == 0:
        self.set(50, 50)
    # move from 1.4 to 2.3
    self.post.set(35, 48)
    self.set(52, 48)
    conveyors['1.4'] = False

    # move from 2.3 to 2.4
    self.post.set(52, 51)
    self.set(54, 51)
    queues['3.3'].append(1)

    while (conveyors['3.4']) or (queues['3.3'][0] != 1):
        self.sleep(0.001)
    conveyors['3.4'] = True
    queues['3.3'].pop(0)

    # rotate 3.3 to 1
    if self.get(60) == 0:
        self.set(68, 60)
    # move from 2.4 to 3.3
    self.post.set(54, 58)
    self.set(71, 58)
    conveyors['2.4'] = False

    # move from 3.3 to 3.4
    self.post.set(71, 61)
    self.set(73, 61)
    queues['4.3'].append(1)

    while (conveyors['4.4']) or (queues['4.3'][0] != 1):
        self.sleep(0.001)
    conveyors['4.4'] = True
    queues['4.3'].pop(0)

    # rotate 4.3 to 0
    if self.get(70) == 0:
        self.set(87, 70)
    # move from 3.4 to 4.3
    self.post.set(73, 68)
    self.set(90, 68)
    conveyors['3.4'] = False
    # rotate 4.3 to 1
    self.set(88, 69)

    # move to warehouse 1
    define_color(self)


# --------------------------------------------------------------------------------

def run_line_b(self):
    """
        Deliver the block along the way B (handler 2)
        :param self: RApi object
    """

    while conveyors['2.2']:
        self.sleep(0.001)
    conveyors['2.2'] = True

    # move from 2.1 to 2.2
    self.post.set(38, 43)
    self.set(40, 43)
    conveyors['2.1'] = False

    # handler work
    self.set(41, 44)
    self.set(43, 46)
    self.set(45, time=3)
    self.set(44, 47)
    self.set(42, 45)

    queues['2.3'].append(0)

    while (conveyors['2.4']) or (queues['2.3'][0] != 0):
        self.sleep(0.001)
    conveyors['2.4'] = True
    queues['2.3'].pop(0)

    # rotate 2.3 to 0
    if self.get(49) == 0:
        self.set(49, 49)
    # move from 2.2 to 2.3
    self.post.set(40, 48)
    self.set(52, 48)
    # rotate 2.3 to 1
    self.set(50, 50)
    conveyors['1.4'] = False

    # move from 2.3 to 2.4
    self.post.set(52, 51)
    self.set(54, 51)
    queues['3.3'].append(1)

    while (conveyors['3.4']) or (queues['3.3'][0] != 1):
        self.sleep(0.001)
    conveyors['3.4'] = True
    queues['3.3'].pop(0)

    # rotate 3.3 to 1
    if self.get(60) == 0:
        self.set(68, 60)
    # move from 2.4 to 3.3
    self.post.set(54, 58)
    self.set(71, 58)
    conveyors['2.4'] = False

    # move from 3.3 to 3.4
    self.post.set(71, 61)
    self.set(73, 61)
    queues['4.3'].append(1)

    while (conveyors['4.4']) or (queues['4.3'][0] != 1):
        self.sleep(0.001)
    conveyors['4.4'] = True
    queues['4.3'].pop(0)

    # rotate 4.3 to 0
    if self.get(70) == 0:
        self.set(87, 70)
    # move from 3.4 to 4.3
    self.post.set(73, 68)
    self.set(90, 68)
    conveyors['3.4'] = False
    # rotate 4.3 to 1
    self.set(88, 69)

    # move to warehouse 1
    define_color(self)


# --------------------------------------------------------------------------------

def run_line_c(self):
    """
        Deliver the block along the way C (handler 3)
        :param self: RApi object
    """

    while conveyors['3.2']:
        self.sleep(0.001)
    conveyors['3.2'] = True

    # move from 3.1 to 3.2
    self.post.set(57, 53)
    self.set(59, 53)
    conveyors['3.1'] = False

    # handler work
    self.set(60, 54)
    self.set(62, 56)
    self.set(64, time=3)
    self.set(63, 57)
    self.set(61, 55)

    queues['3.3'].append(0)

    while (conveyors['3.4']) or (queues['3.3'][0] != 1):
        self.sleep(0.001)
    conveyors['3.4'] = True
    queues['3.3'].pop(0)

    # rotate 3.3 to 0
    if self.get(59) == 0:
        self.set(69, 59)
    # move from 3.2 to 3.3
    self.post.set(59, 58)
    self.set(71, 58)
    # rotate 3.3 to 1
    self.set(68, 60)
    conveyors['2.4'] = False

    # move from 3.3 to 3.4
    self.post.set(71, 61)
    self.set(73, 61)
    queues['4.3'].append(1)

    while (conveyors['4.4']) or (queues['4.3'][0] != 1):
        self.sleep(0.001)
    conveyors['4.4'] = True
    queues['4.3'].pop(0)

    # rotate 4.3 to 0
    if self.get(70) == 0:
        self.set(87, 70)
    # move from 3.4 to 4.3
    self.post.set(73, 68)
    self.set(90, 68)
    conveyors['3.4'] = False
    # rotate 4.3 to 1
    self.set(88, 69)

    # move to warehouse 1
    define_color(self)


# --------------------------------------------------------------------------------

def run_line_d(self):
    """
        Deliver the block along the way D (handler 4)
        :param self: RApi object
    """

    while conveyors['4.2']:
        self.sleep(0.001)
    conveyors['4.2'] = True

    # move from 4.1 to 4.2
    self.post.set(76, 63)
    self.set(78, 63)
    conveyors['4.1'] = False

    # handler work
    self.set(79, 64)
    self.set(81, 66)
    self.set(96, time=1)
    self.set(83, time=3)
    self.set(82, 67)
    self.set(80, 65)

    queues['4.3'].append(0)

    while (conveyors['4.4']) or (queues['4.3'][0] != 1):
        self.sleep(0.001)
    conveyors['4.4'] = True
    queues['4.3'].pop(0)

    # rotate 4.3 to 1
    if self.get(69) == 0:
        self.set(88, 69)
    # move from 4.2 to 4.3
    self.post.set(78, 68)
    self.set(90, 68)
    conveyors['4.2'] = False

    # move to warehouse 1
    define_color(self)


# --------------------------------------------------------------------------------
