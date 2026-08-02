import cv2
import urllib.request
import numpy as np

req = urllib.request.urlopen('http://192.168.1.117:6688/snapshot/PROFILE_000')
np_arr = np.asarray(bytearray(req.read()), dtype=np.uint8)
frame = cv2.imdecode(np_arr, -1) # 'Load it as it is'

#np_arr = np.frombuffer(req, dtype=np.uint8)
#frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

cv2.imshow('lalala', frame)
if cv2.waitKey() & 0xff == 27: quit()