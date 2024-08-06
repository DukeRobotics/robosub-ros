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
    print('Saleae connected')
    # Disable the alarm after successful connection
    signal.alarm(0)
except TimeoutException:
    print('Failed to connect to Saleae: Connection timed out')
except Exception as e:
    print(f'Failed to connect to Saleae: {e}')
    # Disable the alarm in case of other exceptions
    signal.alarm(0)
    