from common import Twist, M_TO_PIXELS

class SimpleAvoider:

    def process_scan(self, robot, scan, visualize=False):
        twist = Twist()
        twist.linear = 4

        # Find closest distance < max range
        closestI = None
        closestRange = float('inf')
        for i in range(len(scan.ranges)):
            r = scan.ranges[i]
            if r < closestRange and r < scan.MAX_VALUE:
                closestI = i
                closestRange = r

        if closestI != None:
            if scan.angles[closestI] != 0:
                twist.angular = -1 / scan.angles[closestI]
                
        return twist
        
