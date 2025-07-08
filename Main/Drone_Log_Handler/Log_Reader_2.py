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

    def get_fields_nval(self, debug: bool=False):
        self.message = 'NVAL'
        name = self.get_field('Name')
        names = pd.Series(name).dropna().unique().tolist()

        if debug:
            print(names)
        return names
    
    @staticmethod
    def get_timestamp(TimeUS):
        timestamp = TimeUS / 1_000_000
        return pd.to_datetime(timestamp, unit='s')

    def get_statustext(self, debug: bool=False):
        self.message = 'MSG'
        messages = self.get_field('Message') 

        if debug:
            for msg in messages:
                print(msg)
        return messages
        

    def get_gps(self, debug: bool=False):
        self.message = 'GPS'
        time = self.get_field('TimeUS')

        gps = pd.DataFrame({ 
            'time': self.get_timestamp(time),
            'lat' : self.get_field('Lat'),
            'lon' : self.get_field('Lng'),
            'alt' : self.get_field('Alt')})

        if debug:
            print(gps)
        return gps

    def get_ppm(self, debug: bool=False):
        self.message = 'NVAL'
        time = self.get_field('TimeUS')

        ppm_df = pd.DataFrame({
            'time_raw' : self.get_field('TimeUS'),
            'time_boot': self.get_field('TimeBootMS'),
            'timestamp': self.get_timestamp(time),
            'ppm' : self.get_field('Value'),
            'name': self.get_field('Name').astype(str)
        })
        
        ppm_mq2 = ppm_df[ppm_df['name'] == 'PPM_MQ2'].copy()
        ppm_mq2.rename(columns={'ppm': 'ppm_mq2'}, inplace=True)
        ppm_mq5 = ppm_df[ppm_df['name'] == 'PPM_MQ5'].copy()
        ppm_mq5.rename(columns={'ppm': 'ppm_mq5'}, inplace=True)
        output = pd.merge_asof(ppm_mq2.sort_values('time_boot'), ppm_mq5.sort_values('time_boot'),
                                on='time_boot', direction='nearest', tolerance=100)

        if debug:
            print(output)

        return output

    def get_ppm_seq(self, debug: bool=False):
        self.message = 'NVAL'
        names = self.get_field('Name')
        values = self.get_field('Value')

        df = pd.DataFrame({'Name': names, 'Value': values})
        df = df.dropna(subset=['Name', 'Value'])
        df['LocalIndex'] = df.groupby('Name').cumcount()
        df_pivot = df.pivot(index='LocalIndex', columns='Name', values='Value')
        df_pivot = df_pivot.reset_index(drop=True)

        if debug:
            print(df_pivot)
        return df_pivot


if __name__ == '__main__':
    path = f"{os.getcwd()}/Main/Drone_Log_Handler/Log_File/Example/seq_2.bin"

    log = PixhawkLogger(filepath=path, message='NVAL')

    # gps = log.get_gps(debug=True)

    # ppm = log.get_ppm(debug=True)

    # msg = log.get_statustext(debug=True)

    log.get_ppm_seq(debug=True)