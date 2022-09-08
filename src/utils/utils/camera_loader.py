from interface.Controller.CameraSelectController import CameraSelectController
import subprocess as sb
import sys

class CameraLoader:

    def get_index(self):
        return self.camera_dev

    def __init__(self, _camera_id):
        # Product ID
        self.camera_id = _camera_id
        # List of possible devices
        self.camera_list = {}

        search_index = self.camera_check()

        if search_index == -1:

            self.controller = CameraSelectController(self.camera_id, self.camera_list, self.camera_check)
            return_type = self.controller.return_type

            if return_type == 0:
                self.camera_dev = -1, -1
            elif return_type == 1:
                self.camera_dev = 1, self.controller.device
            else:
                self.camera_id['name'] = self.camera_list[self.controller.device]['name']
                self.camera_id['vendor'] = self.camera_list[self.controller.device]['vendor']
                self.camera_id['product'] = self.camera_list[self.controller.device]['product']
                self.camera_dev = 2, int(self.controller.device)
        else:
            self.camera_dev = 2, search_index

    def camera_check(self):
        self.camera_list.clear()
        # List capture devices options
        encoding = sys.getdefaultencoding()
        decode = lambda s : s.decode(encoding).split('\n')
        
        devices = decode(sb.check_output("ls", cwd="/sys/class/video4linux/"))
        
        # For each device in the dev dir
        for device_dir in devices:
            if len(device_dir):

                try:
                    name = sb.check_output("cat /sys/class/video4linux/"+device_dir+"/name",shell=True)
                    name = decode(name)[0]
                except (FileNotFoundError,ValueError, sb.CalledProcessError) as e:
                    name = "No_Name"
                    print(e)

                try:                
                    input_folder = sb.check_output("ls", cwd="/sys/class/video4linux/"+device_dir+"/device/input/")
                    input_folder = decode(input_folder)[0]
                except (FileNotFoundError,ValueError, sb.CalledProcessError) as e:
                    input_folder = "No_Folder"
                    print(e)

                try:
                    product = sb.check_output("cat /sys/class/video4linux/"+device_dir+"/device/input/"+input_folder+"/id/product", shell=True)
                    product = decode(product)[0]
                except (FileNotFoundError,ValueError, sb.CalledProcessError) as e:
                    product = "No_Procuct"
                    print(e)

                try:
                    vendor = sb.check_output("cat /sys/class/video4linux/"+device_dir+"/device/input/"+input_folder+"/id/vendor", shell=True)
                    vendor = decode(vendor)[0]
                except (FileNotFoundError,ValueError, sb.CalledProcessError) as e:
                    vendor = "No_Vendor"
                    print(e)

                device_number = device_dir.strip("video")
                self.camera_list[device_number] = {"name": name, "product": product, "vendor" : vendor}
                if product == self.camera_id["product"]:
                    return int(device_number)

        return -1