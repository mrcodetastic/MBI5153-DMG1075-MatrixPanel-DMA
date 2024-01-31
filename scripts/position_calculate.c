// Example program
#include <iostream>
#include <string>

//define csv_mode 1

int main()
{
    std::cout << "Note: (y, x) output starts at (0, 0) so (3, 3) IRL equals row 4, x pixel 4 from left." << std::endl;    
    // Light up all of R1
    int counter = 0;
    for (int row = 0; row < 20; row++) {
      for (int chan = 0; chan < 16; chan++) {
        for (int ic = 0; ic < 5; ic++) { // number of chained ICs

            // data latch on the last bit, when sending the last byte set latch=1
            int latch = 0;
            if (ic == 4) { latch = 1; }  // latch on last channel / ic

            int bit_offset = 16;
            while (bit_offset > 0)  // shift out MSB first per the documentation.
            {
              bit_offset--;  // start from 15
              
              #ifdef csv_mode
              
                if (bit_offset == 15) {
                    std::cout << row <<"\t" << chan << "\t" << ic << "\t" << counter << "\r\n";
                }
              
              #else
              
                  if (ic == 0 && row == 5 && chan == 5)
                  {       
                     std::cout << "(5, 5) is at position: " << counter << "! (bit_offset is " << bit_offset << ")\n";
                  }    
                  
              
                  if (ic == 1 && row == 3 && chan == 0)
                  {       
                     std::cout << "(3, 16) is at position: " << counter << "! (bit_offset is " << bit_offset << ")\n";
                  }              
                  
                  if (ic == 1 && row == 3 && chan == 1)
                  {     
                     std::cout << "(3, 17) is at position: " << counter << "! (bit_offset is " << bit_offset << ")\n";
                  }              
                  
                          
               
                  
                  if (latch == 1 && bit_offset == 0) {
                    // std::cout << "Latch is at position: " << counter << "!\n";
                  } 
                               
              #endif 
            
               counter++;             
            }

        }
      }
    }
    
    

}