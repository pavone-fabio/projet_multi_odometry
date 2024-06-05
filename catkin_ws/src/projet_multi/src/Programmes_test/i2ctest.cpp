#include <iostream>
#include <errno.h>
#include <wiringPiI2C.h>
#include <wiringPi.h>

using namespace std;

int main(void)
{
   int fd, result;

   if (wiringPiSetup() == -1){
    std::cout << "Error wiringPi failed" << std::endl;
    return 1;
   }

   // Initialize the interface by giving it an external device ID.
   // The MCP4725 defaults to address 0x60.   
   //
   // It returns a standard file descriptor.
   // 
   fd = wiringPiI2CSetup(0x6f);

   cout << "Init result: "<< fd << endl;
   
   pinMode(3, INPUT);

   while (true){

    int buttonState = wiringPiI2CReadReg8(fd,0x03);
    if (buttonState & 0x02){
        std::cout << "Boutton clicked ! " << std::endl;
        wiringPiI2CWriteReg8(fd, 0x03, buttonState & ~0x02);
    }
    if (buttonState & 0x04){
        std::cout << "Boutton pushed ! " << std::endl;
        wiringPiI2CWriteReg8(fd, 0x03, buttonState & ~0x04);
    }
    
    delay(100);

   }
   return 0;
}