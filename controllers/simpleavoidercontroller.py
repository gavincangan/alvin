from controller import Controller
from common import Twist, M_TO_PIXELS

class SimpleAvoiderController(Controller):

    def react(self, robot, sensor_suite, visualize=False):

        range_scan = sensor_suite.range_scan

        twist = Twist()
        twist.linear = 4

        # Find closest distance < max range
        closestI = None
        closestRange = float('inf')
        for i in range(len(range_scan.ranges)):
            r = range_scan.ranges[i]
            if r < closestRange and r < range_scan.RANGE_MAX:
                closestI = i
                closestRange = r

        if closestI != None:
            if range_scan.angles[closestI] != 0:
                twist.angular = -1 / range_scan.angles[closestI]
                
        return twist
        
