import cv2
from PIL import Image
import imageio
from pandas import cut


# from: https://stackoverflow.com/questions/53364769/get-frames-per-second-of-a-gif-in-python
def get_avg_fps(PIL_Image_object):
    """ Returns the average framerate of a PIL Image object """
    PIL_Image_object.seek(0)
    frames = duration = 0
    while True:
        try:
            frames += 1
            duration += PIL_Image_object.info['duration']
            PIL_Image_object.seek(PIL_Image_object.tell() + 1)
        except EOFError:
            return frames / duration * 1000
    return None

# I should probably take in as input
cutter_filename = "MARS_Shooting_&_Moving.MOV" 

maxFrames = 1000

im = Image.open(cutter_filename)

out_filename = "cut"+cutter_filename
fps_in = get_avg_fps(im)
print("fps:",fps_in)

try:
    frames = []
    cv2Vid = cv2.VideoCapture(cutter_filename)
    for i in range(maxFrames):
        ret,frame = cv2Vid.read()
        frame = cv2.cvtColor(frame,cv2.COLOR_RGB2BGR)
        frames.append(frame)

    imageio.mimsave(out_filename,frames,fps=fps_in)

except EOFError:
    print("Not enough frames/can't read")