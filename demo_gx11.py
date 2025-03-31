from libgex.libgx11 import Hand
import time
 
hand = Hand(port='/dev/ttyACM0') # COM* for Windows, ttyACM* or ttyUSB* for Linux
hand.connect(goal_pwm=600) # goal_pwm changes the speed, max 855

hand.home() # home the hand

time.sleep(1)

for i in range(10):
    # finger1 vs finger2, thumb vs index
    finger1_xyz = [0.04, 0.08, 0.12] 
    finger2_xyz = [0.04, 0.086, 0.12]

    # use inverse kinmetics
    hand.setj_finger1(hand.ik_finger1(finger1_xyz))
    hand.setj_finger2(hand.ik_finger2(finger2_xyz))

    time.sleep(2)
    hand.home()
    time.sleep(2)

    # finger1 vs finger3, thumb vs middle
    finger1_xyz = [0.01, 0.08, 0.12]
    finger3_xyz = [0.01, 0.086, 0.12]

    hand.setj_finger1(hand.ik_finger1(finger1_xyz))
    hand.setj_finger3(hand.ik_finger3(finger3_xyz))

    time.sleep(2)
    hand.home()
    time.sleep(2)





