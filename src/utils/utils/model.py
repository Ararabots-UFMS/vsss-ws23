from utils.yaml_handler import YamlHandler
class Model():
    """The model class for loading and saving yml files"""

    def __init__(self):
        self.yaml_handler = YamlHandler()
        self.robot_params = self.yaml_handler.read("parameters/robots.yml", escape=True)
        self.robot_bluetooth = self.yaml_handler.read("parameters/bluetooth.yml", escape=True)
        self.robot_bodies = self.yaml_handler.read("parameters/bodies.yml", escape=True)
        self.robot_roles = self.yaml_handler.read("parameters/roles.yml", escape=True)
        self.game_opt = self.yaml_handler.read("parameters/game.yml", escape=True)
        self.debug_params = self.yaml_handler.read("parameters/debug.yml",escape=True)

    def save_params(self):
        self.yaml_handler.write(self.robot_params, "parameters/robots.yml")
        self.yaml_handler.write(self.game_opt, "parameters/game.yml")
        self.yaml_handler.write(self.robot_bluetooth, "parameters/bluetooth.yml")
        self.yaml_handler.write(self.debug_params, "parameters/debug.yml")
        # TODO: save bodies and pid