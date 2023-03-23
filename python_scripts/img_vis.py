import cv2
import matplotlib.pyplot as plt

img_path = r"/home/kai20/exercise/rdcm/envs/random_example_res.png"
img = plt.imread(img_path)
fig = plt.figure('show picture')
plt.imshow(img)
plt.show()