import cv2
import numpy as np
class constants:
    
    basevelocity = 5.0
    center = 0
    kp = 0.05
    ki = 0.01
    kd = 0.0

    integral = 0
    previous_error = 0


class utils(constants) :
    
    def image_conversion(self) :
        self.image = list(self.image)
        self.image = np.array(self.image, np.uint8)
        self.image = self.image.reshape(self.resolution[0], self.resolution[1], 3)
        self.grayImage = cv2.cvtColor(self.image, cv2.COLOR_RGB2GRAY)
        _, self.binary_image = cv2.threshold(self.grayImage, 20, 225, cv2.THRESH_BINARY)
        return _, self.binary_image