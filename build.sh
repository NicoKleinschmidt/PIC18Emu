cd program
mkdir build
xc8-cc -mcpu=18F66K80 -o build/flash.bin Test\ Program.X/main.c
gobjcopy -O binary -I ihex build/flash.hex ../program.bin
