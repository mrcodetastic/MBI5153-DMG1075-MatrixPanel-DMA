// This works ok with ghosting on, 
//        .clock_speed_hz = SPI_MASTER_FREQ_8M/2,   

// Example program
#include <iostream>
#include <string>
#include <bitset>
#include <vector>
#include <cstdint>

/*

spi_bus_config_t host_conf;
  // ensure GND is connected on the logic analyser!!
  // Don't use MOSI as it has a differnt value
  host_conf.data2_io_num = GPIO_NUM_1; // gclk
  host_conf.data3_io_num = GPIO_NUM_5; // a
  host_conf.data4_io_num = GPIO_NUM_4; // b
  host_conf.data5_io_num = GPIO_NUM_42; // c
  host_conf.data6_io_num = GPIO_NUM_7; // d 
  host_conf.data7_io_num = GPIO_NUM_41; // e
  host_conf.data0_io_num = -1; // MOSI / Not used
  host_conf.data1_io_num = -1; // MISO / Not Used
  host_conf.sclk_io_num  = -1; // clock not used  (therefore don't have SPICOMMON_BUSFLAG_SCLK below )
  host_conf.max_transfer_sz = 32768; //32768;
  host_conf.flags = SPICOMMON_BUSFLAG_OCTAL | SPICOMMON_BUSFLAG_GPIO_PINS | SPICOMMON_BUSFLAG_MASTER;
  host_conf.intr_flags = 0;
  host_conf.isr_cpu_id = INTR_CPU_ID_AUTO;

*/
  
// bit positions to purpose
#define BIT_GCLK (1 << 2)
#define BIT_A (1 << 3)
#define BIT_B (1 << 4)
#define BIT_C (1 << 5)
#define BIT_D (1 << 6)
#define BIT_E (1 << 7)
  
// The gray scale data needs the GCLK to save the data into SRAM. 
// The frequency of GCLK must be higher than 20% of DCLK to get the correct data.

int spi_offset = 2; // because we start from bit lsb pos 3 ( 1 << 2) for octal output
int main()
{
    // Create a vector containing integers
    std::vector<uint8_t> v;
	
	// 513 clock manually for no multipler clock multipler OFF
	//mbi_GCLK_clock(257); // clock manually for no multipler clock multipler ON
	int GCLKs = 513; 	// when clock multipler is OFF
	//int GCLKs 	= 257; 	// when clock multipler is ON
	int ROWS 	= 20;
	uint8_t parallel_output_val = 0;
	
	std::bitset<32> bits;
	
	for (int row = 0; row < ROWS; row++)
	{ 
		for ( int gclk = 0; gclk < GCLKs; gclk++) 
		{		
		
			bits.set(1+spi_offset, (row & 0b00000001));
			bits.set(2+spi_offset, (row & 0b00000010));
			bits.set(3+spi_offset, (row & 0b00000100));
			bits.set(4+spi_offset, (row & 0b00001000));
			bits.set(5+spi_offset, (row & 0b00010000));
			
			// (gclk < 16) -> gets rid of ghosting BEFORE the row change for whatever reason
			// (gclk > (GCLKs-6)) -> gets rid of ghosting AFTER the row change
			if ( (gclk < 16) || (gclk > (GCLKs-9))) {
				
				uint32_t val_h = (0x00 | BIT_GCLK | BIT_A | BIT_B | BIT_C | BIT_D | BIT_E);
				uint32_t val_l = (0x00 | BIT_A | BIT_B | BIT_C | BIT_D | BIT_E);
				
				v.push_back(val_h);	// high - out of range row
				v.push_back(val_l);	// low	- out of range row			
				
			} else {				
			
				bits.set(0+spi_offset,true); // clk HIGH			
				v.push_back((uint8_t) bits.to_ulong());						
				
				bits.set(0+spi_offset,false); // clk LOW
				v.push_back((uint8_t) bits.to_ulong());
				
				
			}
			
		}
	}



    int vecSize = v.size(); // returns length of vector    
    
    std::cout << "// total byte size: " << vecSize << "\n";
    std::cout << "//--------------------------------------------\n";
	std::cout << "\n";
	std::cout << "const uint8_t dma_data_nodt[] = {\n";
      
    // Print values
	int counter = 0;
    for (auto & element : v) 
    {
        std::cout << "0b" << std::bitset<8>(element);
		
		if ( ++counter < v.size()) {
				std::cout << "," << std::endl;
		}
    }
	std::cout << "};\n" << std::endl;
	
	
    return 0;


}
