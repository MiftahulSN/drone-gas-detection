import os
from ardupilot_log_reader import Ardupilot

class ArdupilotLogger(Ardupilot):

    def __init__(self, filepath: str, messages:  list[str]):
        self.filepath = filepath
        self.messages = messages
        self.parser = self.parse_log()

    def parse_log(self):
        return Ardupilot.parse(
            self.filepath,
            types=self.messages,
        )

    def get_messages(self):
        return self.parser.dfs

    def get_dataframe(self, field: str):
        return self.get_messages()[field]



if __name__ == '__main__':
    print(os.getcwd())
    filepath = f"{os.getcwd()}\Main\Drone_Log_Handler/Log_File/Example/nval_1.bin"
    
    log = ArdupilotLogger(
        filepath=filepath,
        messages=['NVAL'])
    
    data = log.get_dataframe('NVAL')
    ppm_data = data[data['Name'] == 'PPM']
    ppm_values = ppm_data['Value']

    print(ppm_values)