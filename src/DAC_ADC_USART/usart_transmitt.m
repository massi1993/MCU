% Define serial port with 115200 baud rate 
rate = 115200;
deviceSTM32 = serialport('COM44',rate);

% Write char on SerialPort. Can be 'A', 'B', 'C', 'D'
write(deviceSTM32, 'B',"uint8");
while(deviceSTM32.NumBytesAvailable==0)
end
data=char(read(deviceSTM32,deviceSTM32.NumBytesAvailable,"uint8"));
pause(1);

% Close and delete the serial port
delete(deviceSTM32);

