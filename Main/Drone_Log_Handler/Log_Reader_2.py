import os
import pandas as pd
from pymavlog import MavLog

class PixhawkLogger(MavLog):

    def __init__(self, filepath: str, message: str):
        super().__init__(filepath)
        self.message = message
        self.parse()

    def get_message(self):
        return self.get(self.message)

    def get_field(self, field: str):
        return self.get_message()[field]


if __name__ == '__main__':
    path = f"{os.getcwd()}/Drone_Log_Handler/Log_File/Example/nval_1.bin"

    log = PixhawkLogger(filepath=path, message='NVAL')

    ppm = log.get_field('Value')

    ppm_df = pd.DataFrame(ppm, columns=['PPM'])

    print(ppm_df)




# CACHE CODE

# class PixhawkLogger(MavLog):

#     def __init__(self, filepath: str):
#         super().__init__(filepath)
#         self.parse()

#     def get_messages(self, messages: str, fields: Union[str, list[str]]):
#         try:
#             msg = self.get(messages)
#             if isinstance(fields, list):
#                 msg_fields = {field: msg.fields[field] for field in fields}
#             else:
#                 msg_fields = msg.fields[fields]
#             return msg_fields
#         except Exception as e:
#             print(f"Error: {e}")
#             return None

# if __name__ == '__main__':
#     path = f"{os.getcwd()}/Drone_Log_Handler/Log_File/log3.bin"

#     log  = PixhawkLogger(path)
#     statustext = log.get_messages(messages="MSG", fields=["Message"])

#     filtered_msg = [msg for msg in statustext['Message']]
    
#     key = "SRC=1/200:data-value"
#     for message in filtered_msg:
#         match = re.search(fr'{key}: (\d+)', message)
#         if match:
#             value = match.group(1)
#             print(value)