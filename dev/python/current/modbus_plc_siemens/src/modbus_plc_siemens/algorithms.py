from modbus_plc_siemens.r_api import *

def pickup_block(column, line, way):
    # move to necessary column
	# (column+3) - column' register
    if column != 1:
        R.set(port=3, sensor=column+3)

    # move to necessary cell
	# (line+7)*2+1) - line' register
    if line != 1:
        R.set(port=5, sensor=(line+7)*2+1)
    
    # pickup the block
	# ((line+8)*2) - line' register
    R.set(port=8, sensor=25)
    R.set(port=5, sensor=(line+8)*2)
    R.set(port=7, sensor=26)
    
    # deliver the block
	# (way*3-(4-way)//2) - navigation
	if column < (way*3-(4-way)//2):
		R.set(port=3, sensor=way+98)
	else:
		R.set(port=2, sensor=way+98)
    
	R.set(port=7, sensor=27)
    R.set(port=6, sensor=17)
    R.set(port=8, sensor=26)

    # return to starting position
    R.post.set(port=2, sensor=4)


# ---------------------------------------

def put_block():
    # pickup the block
    R.set(port=16, sensor=96)
    R.set(port=13, sensor=89)
    R.set(port=15, sensor=97)
    # move to necessary column (column №9)
    R.set(port=10, sensor=83)
    # move to necessary cell (line №4)
    R.set(port=13, sensor=95)
    # put the block
    R.set(port=15, sensor=98)
    R.set(port=14, sensor=94)
    R.set(port=16, sensor=97)
    # return to starting position
    R.set(port=14, sensor=88)
    R.set(port=11, sensor=86)


# ---------------------------------------

colors = {
	0: '\nBlack color',
	1: '\nYellow block',
	2: '\nBlue block',
	3: '\nGreen block',
	4: '\nPurple block',
	5: '\nUnknown color'
}
def straight_move():
	R.post.set(port=90, sensor=72)
    R.set(port=93, sensor=72)

    R.post.set(port=87, sensor=70)
    R.sleep(1)
	
	# color detection
	print(colors[R.get(port=0)])

    R.post.set(port=93, sensor=73)

    # timer for right pick-up position
    R.set(port=95, sensor=73, time=0.25)


# ---------------------------------------

def run_a(column, line):
    if (column<1 or column>12) or (line<1 or line>4):
		print('\nIncorrect column or line')
		return
	
	pickup_block(column, line, 1)
    # ----------------------------
    R.post.set(port=19, sensor=33)
    R.set(port=21, sensor=33)

    # handler work
    R.set(port=22, sensor=34)
    R.set(port=25, sensor=36)
    R.set(port=26, time=1)
    R.set(port=24, sensor=37)
    R.set(port=23, sensor=35)

    if R.get(port=39) == 1:
        R.set(port=31, sensor=39)

    R.post.set(port=21, sensor=38)
    R.set(port=33, sensor=38)

    R.set(port=30, sensor=40)
    # ----------------------------

    R.post.set(port=33, sensor=41)
    R.set(port=35, sensor=41)

    if R.get(port=50) == 1:
        R.set(port=50, sensor=50)

    R.post.set(port=35, sensor=48)
    R.set(port=52, sensor=48)

    R.post.set(port=52, sensor=51)
    R.set(port=54, sensor=51)

    if R.get(port=60) == 1:
        R.set(port=68, sensor=60)

    R.post.set(port=54, sensor=58)
    R.set(port=71, sensor=58)

    R.post.set(port=71, sensor=61)
    R.set(port=73, sensor=61)

    if R.get(port=70) == 1:
        R.set(port=87, sensor=70)

    R.post.set(port=73, sensor=68)
    R.set(port=90, sensor=68)

    # ----------------------------
    R.set(port=88, sensor=69)

    straight_move()
    put_block()


# ---------------------------------------

def run_b(column, line):
    if (column<1 or column>12) or (line<1 or line>4):
		print('\nIncorrect column or line')
		return
	
	pickup_block(column, line, 2)
    # ----------------------------
    R.post.set(port=38, sensor=43)
    R.set(port=40, sensor=43)

    # handler work
    R.set(port=41, sensor=44)
    R.set(port=43, sensor=46)
    R.set(port=45, time=1)
    R.set(port=44, sensor=47)
    R.set(port=42, sensor=45)

    if R.get(port=49) == 1:
        R.set(port=49, sensor=49)

    R.post.set(port=40, sensor=48)
    R.set(port=52, sensor=48)

    R.set(port=50, sensor=50)
    # ----------------------------

    R.post.set(port=52, sensor=51)
    R.set(port=54, sensor=51)

    if R.get(port=60) == 1:
        R.set(port=68, sensor=60)

    R.post.set(port=54, sensor=58)
    R.set(port=71, sensor=58)

    R.post.set(port=71, sensor=61)
    R.set(port=73, sensor=61)

    if R.get(port=70) == 1:
        R.set(port=87, sensor=70)

    R.post.set(port=73, sensor=68)
    R.set(port=90, sensor=68)

    # ----------------------------
    R.set(port=88, sensor=69)

    straight_move()
    put_block()


# ---------------------------------------

def run_c(column, line):
    if (column<1 or column>12) or (line<1 or line>4):
		print('\nIncorrect column or line')
		return
	
	pickup_block(column, line, 3)
    # ----------------------------
    R.post.set(port=57, sensor=53)
    R.set(port=59, sensor=53)

    # handler work
    R.set(port=60, sensor=54)
    R.set(port=62, sensor=56)
    R.set(port=64, time=1)
    R.set(port=63, sensor=57)
    R.set(port=61, sensor=55)

    if R.get(port=59) == 1:
        R.set(port=69, sensor=59)

    R.post.set(port=59, sensor=58)
    R.set(port=71, sensor=58)

    R.set(port=68, sensor=60)
    # ----------------------------

    R.post.set(port=71, sensor=61)
    R.set(port=73, sensor=61)

    if R.get(port=70) == 1:
        R.set(port=87, sensor=70)

    R.post.set(port=73, sensor=68)
    R.set(port=90, sensor=68)

    # ----------------------------
    R.set(port=88, sensor=69)

    straight_move()
    put_block()


# ---------------------------------------

def run_d(column, line):
	if (column<1 or column>12) or (line<1 or line>4):
		print('\nIncorrect column or line')
		return
    
	pickup_block(column, line, 4)
    # ----------------------------
    R.post.set(port=76, sensor=63)
    R.set(port=78, sensor=63)

    # handler work
    R.set(port=79, sensor=64)
    R.set(port=81, sensor=66)
    R.set(port=96, time=1)
    R.set(port=83, time=1)
    R.set(port=82, sensor=67)
    R.set(port=80, sensor=65)

    if R.get(port=69) == 1:
        R.set(port=88, sensor=69)

    R.post.set(port=78, sensor=68)
    R.set(port=90, sensor=68)

    straight_move()
    put_block()
