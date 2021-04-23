import json 

class Configuration(object):
    def __init__(self, config_file):
        self.config = ConfigLoader.load(config_file)
        #creates a class which instance attributes are based on the config dictionary
        for k, v in self.config.items():
            setattr(self, k, v)

class ConfigLoader(object):

    @staticmethod
    def load(filename):
        with open(filename, 'r') as fp:
            data = json.load(fp)
            return data
    
    @staticmethod
    def dump(filename, data):
        with open(filename, 'w') as fp:
            json.dump(data, fp)