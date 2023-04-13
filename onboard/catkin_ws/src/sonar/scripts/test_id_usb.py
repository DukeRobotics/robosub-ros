import subprocess

# Run the lsusb command and capture the output
usb_output = subprocess.check_output(['ls', '-l', '/dev/serial/by-id/'])

# Split the output into lines
usb_lines = usb_output.decode().split('\n')

sonar_string = "usb-FTDI_FT230X_Basic_UART_DK0C1WF7-if00-port0"

# Iterate over each line and get the device path using udevadm
for line in usb_lines[1:]:
    if line:
        fields = line.split(' ')
        FTDI_Number = fields[8]
        USB_port = fields[10][-7:]
        #print('Number {:s} Device {:s}'.format(FTDI_Number, USB_port))
        if FTDI_Number == sonar_string:
            print(USB_port)



