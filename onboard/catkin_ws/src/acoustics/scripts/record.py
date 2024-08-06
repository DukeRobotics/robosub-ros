import saleae
import datetime

s = saleae.Saleae(args='-disablepopups socket')
s.set_active_channels([], [0, 1, 2, 3, 4, 5, 6, 7])
s.set_sample_rate(s.get_all_sample_rates()[3]) #6.25 MS/s
print('Saleae connected')

def record(seconds: int, filename: str='data') -> bool:
    try:
        s.set_capture_seconds(seconds)
        s.capture_start_and_wait_until_finished()
        # Export as binary
        s.export_data2(f'{filename}.bin')
        print(f'Data recorded to {filename}.bin')
        return True
    except Exception as e:
        print(f'Failed to record data: {e}')
        return False
    
if __name__ == '__main__':
    print('Recording data for 5 seconds')
    record(5, filename=f'data_{datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}')
    print('Recording complete')
