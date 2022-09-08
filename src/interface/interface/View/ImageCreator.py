from PIL import Image
import os


class ImageCreator:
    def __init__(self, width, height):

        root_path = os.environ['ROS_ARARA_ROOT']

        self.image_file = root_path+"src/interface/Assets/arara.png"
        self.img_org = Image.open(self.image_file)

        # best down-sizing filter
        self.img_anti = self.img_org.resize((width, height), Image.ANTIALIAS)
        # split image filename into name and extension
        name, ext = os.path.splitext(self.image_file)
        # create a new file name for saving the result
        new_image_file = root_path+"src/interface/Assets/Cache/arara_"+str(width)+"_"+str(height)+ext
        self.img_anti.save(new_image_file)