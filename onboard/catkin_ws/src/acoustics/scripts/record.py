import saleae
import signal

class TimeoutException(Exception):
    pass

def timeout_handler(signum, frame):
    raise TimeoutException

# Set the timeout duration (in seconds)
timeout_duration = 5

# Register the timeout handler
signal.signal(signal.SIGALRM, timeout_handler)
signal.alarm(timeout_duration)

try:
    s = saleae.Saleae(args='-disablepopups socket')
    s.set_active_channels([], [0, 1, 2, 3, 4, 5, 6, 7])
    s.set_sample_rate(s.get_all_sample_rates()[3]) #6.25 MS/s
    print('Saleae connected')
    # Disable the alarm after successful connection
    signal.alarm(0)
except TimeoutException:
    print('Failed to connect to Saleae: Connection timed out')
except Exception as e:
    print(f'Failed to connect to Saleae: {e}')
    # Disable the alarm in case of other exceptions
    signal.alarm(0)


def record(seconds: int, filename: str='data') -> bool:
    try:
        s.set_capture_seconds(seconds)
        s.capture_start_and_wait_until_finished()
        # Export as binary
        s.export_data2(f'{filename}.bin', analog_channels=[0, 1, 2, 3], format='binary')
        print(f'Data recorded to {filename}.bin')
        return True
    except Exception as e:
        print(f'Failed to record data: {e}')
        return False
    
if __name__ == '__main__':
    print('Recording data for 5 seconds')
    record(5)
