#include "stm32f4xx.h"
#include "launch_effect.h"
#include "effect.h"
#include "draw.h"
#include "gameoflife.h"
#include "3d.h"
#include "draw_3d.h"

const char string_arr [][12] = {"STEFFS CUBE","VOLUME","MUSIC","RELAX","FEEL"};
const unsigned char string_nbr = 5;
void launch_effect (int effect)
{
	int var, i,x;
    unsigned char ii;

	fill(0x00);

	switch (effect)
	{
		case 0x0:
			effect_rain(1000);
			break;
		
			
		case 1:
			sendvoxels_rand_z(200,17,130);
			break;
				
		case 2:
			for (var = 0; var < 5; ++var)
			{
			effect_random_filler(5,1);
			effect_random_filler(5,0);
			effect_random_filler(5,1);
			effect_random_filler(5,0);
			}
			break;
				
		case 3:
			effect_z_updown(500,77);
			break;
				
		case 4:
			effect_wormsqueeze (2, AXIS_Z, -1, 1000, 77);
			break;
				
		case 5:
			effect_blinky2();
			break;
				
		case 6: 
            for (ii=0;ii<80;ii++)
			{
				effect_box_shrink_grow (1, ii%4, ii & 0x04, 35);
			}

			effect_box_woopwoop(12,0);
			effect_box_woopwoop(12,1);
			effect_box_woopwoop(12,0);
			effect_box_woopwoop(12,1);
			break;
			
		case 7:
			for (var = 0; var < 10; ++var) {
			effect_planboing (AXIS_Z, 50);
			effect_planboing (AXIS_X, 50);
			effect_planboing (AXIS_Y, 50);
			effect_planboing (AXIS_Z, 50);
			effect_planboing (AXIS_X, 50);
			effect_planboing (AXIS_Y, 50);
			fill(0x00);
			}
			break;
		
		case 8:
			for (var = 0; var < 10; ++var){
				fill(0x00);
				effect_telcstairs(0,61,0xff);
				effect_telcstairs(0,61,0x00);
				effect_telcstairs(1,61,0xff);
				effect_telcstairs(1,61,0xff);
			}
			break;
			
		case 9:
			for (var = 0; var < 10;++var) {
				effect_axis_updown_randsuspend(AXIS_Z, 42,400,0);
				effect_axis_updown_randsuspend(AXIS_Z, 42,400,1);
				effect_axis_updown_randsuspend(AXIS_Z, 42,400,0);
				effect_axis_updown_randsuspend(AXIS_Z, 42,400,1);
				effect_axis_updown_randsuspend(AXIS_X, 42,400,0);
				effect_axis_updown_randsuspend(AXIS_X, 42,400,1);
				effect_axis_updown_randsuspend(AXIS_Y, 42,400,0);
				effect_axis_updown_randsuspend(AXIS_Y, 42,400,1);
			}
			break;
			
		case 10:
			for (var = 0; var < 10; ++var) {
				effect_loadbar(80);
			}
			break;
			
		case 11:
			effect_wormsqueeze (1, AXIS_Z, 1, 1000, 80);
			break;
			
			
		case 12:
			effect_stringfly2(string_arr[rand()%string_nbr]);
			break;
			
		case 13:
			fill(0x00);
            // Create a random starting point for the Game of Life effect.
			for (i = 0; i < 200;i++)
			{
				setvoxel(rand()%4,rand()%4,rand()%4);
			}
	
			gol_play(200, 80);
			break;
			
		case 14:
			for (var = 0; var < 10; ++var) {
				effect_boxside_randsend_parallel (AXIS_Z, 0 , 20,1);
				delay_ms(150);
				effect_boxside_randsend_parallel (AXIS_Z, 1 , 20,1);
				delay_ms(150);

				effect_boxside_randsend_parallel (AXIS_Z, 0 , 20,2);
				delay_ms(150);
				effect_boxside_randsend_parallel (AXIS_Z, 1 , 20,2);
				delay_ms(150);

				effect_boxside_randsend_parallel (AXIS_Y, 0 , 20,1);
				delay_ms(150);
				effect_boxside_randsend_parallel (AXIS_Y, 1 , 20,1);
				delay_ms(150);
			}
			break;
			
		case 15:
			boingboing(2500, 45, 0x01, (rand()%3) +1);
			break;
			
		case 16:
			effect_smileyspin(20,80,0);
			break;
			
		case 17:
			effect_pathspiral(1000,80);
			break;
			
		case 18:
			effect_path_bitmap(55,2,30);
			break;
			
		case 19:
			effect_smileyspin(10,80,1);
			break;
			
		case 20:
			effect_path_text(80,"MUSIC");
			break;
	
		case 21:
			effect_rand_patharound(150,40);
			break;
			
		case 22:
			effect_wormsqueeze (1, AXIS_Z, -1, 1000, 80);
			break;
			
		case 23:
			effect_smileyspin(10,80,2);
			break;
			
		case 24:
			effect_random_sparkle();
			break;
			
		case 25:
			effect_wormsqueeze (1, AXIS_Z, -1, 1000, 80);
			break;
		
		case 26:
			boingboing(2500, 46, 0x01, 0x03);
			break;
		case 28:
			fireworks(50,50,80);
			break;
		case 29:
			sidewaves(10000,5);
			break;
		case 30:
			ripples(10000,5);
			//ripples(2000,10);
			break;
		case 31:
			linespin(10000,3);
			break;
		case 32:
			sinelines(10000,3);
			break;
		case 33:
			spheremove(10000,5);
			break;
		case 34:
			random_pixels(50,10000);
			break;
		// In case the effect number is out of range:
		default:
			//effect_stringfly2("FAIL");
			break;
		
		

	}
}

