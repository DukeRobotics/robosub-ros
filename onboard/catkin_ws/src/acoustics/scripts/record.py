from saleae import automation
import os
import os.path

class SaleaeCapture:
    def __init__(self):
        self.manager = automation.Manager.connect(port=10430)
        self.device_config = automation.LogicDeviceConfiguration(
            enabled_analog_channels=[0, 1, 2, 3],
            analog_sample_rate=625_000,
        )
    def capture(self, seconds: int, directory: str):
        
        capture_configuration = automation.CaptureConfiguration(
            capture_mode=automation.TimedCaptureMode(duration_seconds=seconds)
        )
        
        with self.manager.start_capture(
            device_id='ECAF7B22509D66EE',
            device_configuration=self.device_config,
            capture_configuration=capture_configuration) as capture:
            
            capture.wait()
            
            output_dir = os.path.join(os.getcwd(), directory)
            os.makedirs(output_dir)

            
            capture.export_raw_data_binary(directory=output_dir, analog_channels=[0, 1, 2, 3])
        
    def close(self):
        self.manager.close()

if __name__ == '__main__':
    saleae = SaleaeCapture()
    print('Recording data for 5 seconds')
    saleae.capture(5, 'data')
    print('Recording complete')
    saleae.close()
    
