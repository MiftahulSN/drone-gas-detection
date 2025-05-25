import os
import pandas as pd
from pymavlog import MavLog
from datetime import datetime

class PixhawkLogger(MavLog):

    def __init__(self, filepath: str, message: str = None):
        super().__init__(filepath)
        self.message = message
        self.parse()

    def get_message(self):
        try:
            return self.get(self.message)
        except:
            print("WARNING: there's no message!")
            return None

    def get_field(self, field: str):
        return self.get_message()[field]
    
    @staticmethod
    def get_time(TimeUS):
        timestamp = TimeUS / 1_000_000
        return pd.to_datetime(timestamp, unit='s')

    def get_gps(self):
        self.message = 'GPS'
        gps = pd.DataFrame({ 
            'time': self.get_field('TimeUS'),
            'lat' : self.get_field('Lat'),
            'lon' : self.get_field('Lng'),
            'alt' : self.get_field('Alt')})
        return gps

    def get_ppm(self):
        self.message = 'NVAL'
        ppm_df = pd.DataFrame({
            'timeUS_raw': self.get_field('TimeUS'),
            'time_s': self.get_field('TimeUS') / 1_000_000,
            'timeMS': self.get_field('TimeBootMS'),
            'ppm': self.get_field('Value'),
            'name': self.get_field('Name').astype(str)
        })
        ppm_mq2 = ppm_df[ppm_df['name'] == 'PPM_MQ2'].copy()
        ppm_mq2.rename(columns={'ppm': 'ppm_mq2'}, inplace=True)
        ppm_mq5 = ppm_df[ppm_df['name'] == 'PPM_MQ5'].copy()
        ppm_mq5.rename(columns={'ppm': 'ppm_mq5'}, inplace=True)
        output = pd.merge_asof(ppm_mq2.sort_values('timeMS'), ppm_mq5.sort_values('timeMS'),
                                on='timeMS', direction='nearest', tolerance=100)
        return output


if __name__ == '__main__':
    path = f"{os.getcwd()}/Main/Drone_Log_Handler/Log_File/Example/test_1.bin"

    log = PixhawkLogger(filepath=path, message='NVAL')

    gps = log.get_gps()
    print(gps)

    ppm = log.get_ppm()
    print(ppm)