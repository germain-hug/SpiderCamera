import cv2

"""
-------------------------
 Bounding Box Selection 
-------------------------
"""

class click_and_crop:

    def __init__(self, img, name):
        self.refPt = []
        self.img = img   # First frame of stream
        self.name = name # Window name (for display)

    def callback(self, event, x, y, flags, params):

        if event == cv2.EVENT_LBUTTONDOWN:
            self.refPt = [(x, y)]

        elif event == cv2.EVENT_LBUTTONUP:
            self.refPt.append((x, y))
            disp = self.img.copy()
            cv2.rectangle(disp, self.refPt[0], self.refPt[1], (0, 255, 0), 2)
            cv2.imshow(self.name, disp)
            cv2.waitKey(1)

        elif len(self.refPt) == 1:
            disp = self.img.copy()
            cv2.rectangle(disp, (self.refPt[0][0], self.refPt[0][1]), (x, y), (0, 255, 0), 2)
            cv2.imshow(self.name, disp)
            cv2.waitKey(1)

    def refresh(self):
        if(len(self.refPt)==0):
            cv2.imshow(self.name, self.img.copy())
        elif len(self.refPt) == 2:
            disp = self.img.copy()
            cv2.rectangle(disp, self.refPt[0], self.refPt[1], (0, 255, 0), 2)
            cv2.imshow(self.name, disp)
            cv2.waitKey(1)
