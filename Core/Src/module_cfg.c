/*
 * module_cfg.c
 *
 *  Created on: Jan 6, 2024
 *      Author: lucas
 */


#include "module_cfg.h"

module_cfg configs = {0};
module_cfg read_cfg = {0};

float bufferTensaoVA[64] = { 1,          1.09801714, 1.19509032, 1.29028468, 1.38268343, 1.47139674,
							  1.55557023, 1.63439328, 1.70710678, 1.77301045, 1.83146961, 1.88192126,
							  1.92387953, 1.95694034, 1.98078528, 1.99518473, 2         , 1.99518473,
							  1.98078528, 1.95694034, 1.92387953, 1.88192126, 1.83146961, 1.77301045,
							  1.70710678, 1.63439328, 1.55557023, 1.47139674, 1.38268343, 1.29028468,
							  1.19509032, 1.09801714, 1         , 0.90198286, 0.80490968, 0.70971532,
							  0.61731657, 0.52860326, 0.44442977, 0.36560672, 0.29289322, 0.22698955,
							  0.16853039, 0.11807874, 0.07612047, 0.04305966, 0.01921472, 0.00481527,
							  0         , 0.00481527, 0.01921472, 0.04305966, 0.07612047, 0.11807874,
							  0.16853039, 0.22698955, 0.29289322, 0.36560672, 0.44442977, 0.52860326,
							  0.61731657, 0.70971532, 0.80490968, 0.90198286};

uint16_t bufferTensaoVAD[64]= {1240, 1362, 1482, 1601, 1715, 1825, 1930, 2028, 2118, 2200, 2272, 2335, 2387, 2428, 2457,
		                    2475, 2481, 2475, 2457, 2428, 2387, 2335, 2272, 2200, 2118, 2028, 1930, 1825, 1715, 1601,
							1482, 1362, 1240, 1119, 998, 880, 766, 655, 551, 453, 363, 281, 209, 146, 94, 53, 23, 5,
							0, 5, 23, 53, 94, 146, 209, 281, 363, 453, 551, 655, 766, 880, 998, 1119};

float bufferTensaoVB[64] = {1.86602540, 1.81284668, 1.75183981, 1.68359230,
						          1.60876143, 1.52806785, 1.44228869, 1.35225005,
						          1.25881905, 1.16289547, 1.06540313, 0.967280917,
						          0.869473808, 0.772923737, 0.678560535, 0.58729297,
						          0.5, 0.417522303, 0.340654185, 0.270135927,
						          0.20664666, 0.150797818, 0.103127258, 0.0640940732,
						          0.0340741737, 0.0133566679, 0.00214107676, 0.000535412524,
						          0.00855513863, 0.0261230207, 0.0530698705, 0.0891361751,
						          0.133974596, 0.187153315, 0.248160193, 0.316407698,
						          0.391238571, 0.471932149, 0.55771131, 0.647749952,
						          0.741180955, 0.837104527, 0.934596871, 1.03271908,
						          1.13052619, 1.22707626, 1.32143947, 1.41270703,
						          1.5, 1.58247770, 1.65934582, 1.72986407,
						          1.79335334, 1.84920218, 1.89687274, 1.93590593,
						          1.96592583, 1.98664333, 1.99785892, 1.99946459,
						          1.99144486, 1.97387698, 1.94693013, 1.91086382};
uint16_t bufferTensaoVBD[64] = {2315, 2249, 2173, 2089, 1996, 1896, 1789, 1678, 1562, 1443, 1322, 1200, 1078, 959, 842,
		                        728, 620, 518, 422, 335, 256, 187, 127, 79, 42, 16, 2, 0, 10, 32, 65, 110, 166, 232, 307,
								392, 485, 585, 692, 803, 919, 1038, 1159, 1281, 1402, 1522, 1639, 1753, 1861, 1963, 2059,
								2146, 2225, 2294, 2353, 2402, 2439, 2465, 2479, 2481, 2471, 2449, 2415, 2371};

float bufferTensaoVC[64] = {0.133974596, 0.0891361751, 0.0530698705, 0.0261230207,
						          0.00855513863, 0.000535412524, 0.00214107676, 0.0133566679,
						          0.0340741737, 0.0640940732, 0.103127258, 0.150797818,
						          0.20664666, 0.270135927, 0.340654185, 0.417522303,
						          0.5, 0.58729297, 0.678560535, 0.772923737,
						          0.869473808, 0.967280917, 1.06540313, 1.16289547,
						          1.25881905, 1.35225005, 1.44228869, 1.52806785,
						          1.60876143, 1.6835923, 1.75183981, 1.81284668,
						          1.8660254, 1.91086382, 1.94693013, 1.97387698,
						          1.99144486, 1.99946459, 1.99785892, 1.98664333,
						          1.96592583, 1.93590593, 1.89687274, 1.84920218,
						          1.79335334, 1.72986407, 1.65934582, 1.5824777,
						          1.5, 1.41270703, 1.32143947, 1.22707626,
						          1.13052619, 1.03271908, 0.934596871, 0.837104527,
						          0.741180955, 0.647749952, 0.55771131, 0.471932149,
						          0.391238571, 0.316407698, 0.248160193, 0.187153315};
uint16_t bufferTensaoVCD[64] = {166, 110, 65, 32, 10, 0, 2, 16, 42, 79, 127, 187, 256, 335, 422, 518, 620, 728, 842, 959,
		                        1078, 1200, 1322, 1443, 1562, 1678, 1789, 1896, 1996, 2089, 2173, 2249, 2315, 2371, 2415,
								2449, 2471, 2481, 2479, 2465, 2439, 2402, 2353, 2294, 2225, 2146, 2059, 1963, 1861, 1753,
								1639, 1522, 1402, 1281, 1159, 1038, 919, 803, 692, 585, 485, 392, 307, 232};

//uint32_t data[10] = {1111,2222,3333,4444,5555,6666,7777,8888,9999,1010};
//uint32_t Rx_Data[10] = {0};
//char string [64] = {0};
//Flash_Write_Data(0x08060000 , (uint32_t *)data, 10);
//Flash_Read_Data (0x08060000 , Rx_Data,          10);
//Convert_To_Str(Rx_Data, string);

void module_cfg_init(void)
{
	uint32_t savedConfig[TOTAL_CFG_BYTES] = {0};
	Flash_Read_Data (CFG_SECTOR_ADDRESS , savedConfig, TOTAL_CFG_BYTES);
	if ( (savedConfig[HEADER_0] == 2)   &&
		 (savedConfig[HEADER_1] == 200) &&
		 (savedConfig[HEADER_2] == 230) &&
		 (savedConfig[HEADER_3] == 165)    ) // aplly saved config
	{
		configs.sensors[0].enable = (bool) savedConfig[SENSOR_0_ENABLE];
		configs.sensors[1].enable = (bool) savedConfig[SENSOR_1_ENABLE];
		configs.sensors[2].enable = (bool) savedConfig[SENSOR_2_ENABLE];
		configs.sensors[3].enable = (bool) savedConfig[SENSOR_3_ENABLE];
		configs.sensors[4].enable = (bool) savedConfig[SENSOR_4_ENABLE];
		configs.sensors[5].enable = (bool) savedConfig[SENSOR_5_ENABLE];
		configs.sensors[6].enable = (bool) savedConfig[SENSOR_6_ENABLE];
		configs.sensors[7].enable = (bool) savedConfig[SENSOR_7_ENABLE];

	}
	else // default
	{
		configs.boardID = 1;
		configs.enable = true;

		for (int i = 0; i < SENSOR_NUMBERS; i++)
		{
			configs.sensors[i].enable = true;
		}

		for (int i = 0; i < INPUT_NUMBERS; i++)
		{
			configs.inputs[0].enable = true;
			configs.inputs[0].inverted = false;
			configs.inputs[0].debouncingTime = 5;
		}

		for (int i = 0; i < OUTPUT_NUMBERS; i++)
		{
			configs.outputs[2].enable = true;
		}
	}
}

int apply_config(module_cfg newConfig)
{
	int result = 0;
	//copy de cfg received to the cfg in ram
	memcpy(&configs, &newConfig, sizeof(configs));

	//save the new data in flash
	uint32_t saveBuffer[64]  = {0};

	uint32_t headerBuff[4] = {2,200,230,165};
	uint32_t cfgBuff[8] = {0};
	for (int i = 0; i < 8; i++)
	{
		cfgBuff[i] = configs.sensors[i].enable;
	}
	memcpy(saveBuffer, headerBuff, sizeof(headerBuff) );
	memcpy(saveBuffer + ( sizeof(headerBuff) / sizeof(uint32_t) ), cfgBuff, sizeof(cfgBuff) );

	Flash_Write_Data(CFG_SECTOR_ADDRESS , (uint32_t *)saveBuffer, TOTAL_CFG_BYTES); // save the cfg
	uint32_t savedConfig[TOTAL_CFG_BYTES] = {0};
	Flash_Read_Data (CFG_SECTOR_ADDRESS , savedConfig, TOTAL_CFG_BYTES);
	if (!memcmp(saveBuffer, savedConfig,TOTAL_CFG_BYTES))
	{
		result = 1;
	}
	return result;
}
