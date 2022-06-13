import cv2
import rospy


class LineTracker():

    def __init__(self, start_x, start_y, name, delta_x=30):
        self.start_x = start_x
        self.start_y = start_y
        self.y = start_y
        self.x = self.start_x
        self.previous_x = self.start_x
        self.name = name
        self.state = 'LOST'
        self.delta_x = 35
        self.time_start_tracking = rospy.Time.now() - rospy.Duration(100)
        self.time_tracking = rospy.Duration(0)

    def track(self, image_stacked, image_gui, draw=True):
        """Find the new line x given the previous position
        """

        height, width, _ = image_gui.shape
        if self.state == 'LOST':  # Nothing to track
            print('Tracker ' + self.name + ' is lost. Nothing to track...')
            if draw:
                self.draw(image_gui)
            return

        self.state = 'LOST'

        minimum_number_white_pixels = 5
        for dx in range(0, self.delta_x):  # search for white pixels left and right

            # Search left
            x = int(self.x - dx)
            if x >= 0 and x < width:
                # print('Tracker ' + self.name + ' searching for white pixels at x=' + str(x))
                if image_stacked[x] > minimum_number_white_pixels:
                    self.state = 'TRACKING'
                    self.previous_x = self.x
                    self.x = x
                    print('Tracker ' + self.name + ' tracking found x=' + str(x))
                    break

            # Search right
            x = self.x + dx
            if x >= 0 and x < width:
                # print('Tracker ' + self.name + ' searching for white pixels at x=' + str(x))
                if image_stacked[x] > minimum_number_white_pixels:
                    self.state = 'TRACKING'
                    self.previous_x = self.x
                    self.x = x
                    print('Tracker ' + self.name + ' tracking found x=' + str(x))
                    break

        if self.state == 'TRACKING':
            self.time_tracking = rospy.Time.now() - self.time_start_tracking
        else:
            self.time_tracking = rospy.Duration(0)

        print(self.name + ' tracking for ' + str(round(self.time_tracking.to_sec(), 2)) + ' secs')

        if draw:
            self.draw(image_gui)

    def restart(self, x):
        print('Restarting tracker ' + self.name + 'at x=' + str(x))
        self.x = x
        self.previous_x = x
        self.state = 'TRACKING'
        self.time_start_tracking = rospy.Time.now()

    def draw(self, image_gui):

        point_text = (int(self.x), self.y - 15)
        point_up = (int(self.x), self.y + 5)
        point_down = (int(self.x), self.y - 5)
        point_a = (int(self.previous_x - self.delta_x), self.y - 10)
        point_b = (int(self.previous_x + self.delta_x), self.y - 10)

        if self.state == 'LOST':
            color = (128, 128, 128)
            text = self.name + ' (Lost)'
        else:
            color = (0, 255, 0)
            text = self.name + ' (Tracking ' + str(round(self.time_tracking.to_sec(), 1)) + ' secs) '

        cv2.line(image_gui, point_a, point_b, color, 4)
        cv2.line(image_gui, point_up, point_down, color, 4)
        cv2.putText(image_gui, text, point_text, cv2.FONT_HERSHEY_PLAIN, 1, color, 1, cv2.LINE_AA)
