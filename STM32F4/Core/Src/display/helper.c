/*
 * helper.c
 *
 *  Created on: Jul 29, 2025
 *      Author: jakob
 */
#include "display/helper.h"

/*
 * To write to external display with only one parameter, the string.
 * Also wraps the text around the edges if string size is bigger than the width of the monitor
 */

void writeDisplay(char* str)
{
	ssd1306_Init();
	SSD1306_Font_t font = Font_11x18; //Set font size
	uint8_t y = 0;
	int8_t str_length = strlen(str);
	uint8_t display_length = 128;
	uint8_t display_height = 64;
	uint8_t max_char = (display_length-2)/font.width; //Calculate maximum number of characters that fit in one line
	uint8_t max_lines = display_height/font.height;
	uint8_t line = 0;
	ssd1306_Fill(Black);

	while(str_length > 0 && line < max_lines)
	{
		ssd1306_SetCursor(2, y); //Set cursor to start of the line

		char* sub_str = malloc(max_char + 1); //Variable needed to store the string of the current line
		for(int i = 0; i < max_char; i++)
		{
			sub_str[i] = str[i + max_char * line];
		}
		sub_str[max_char] = '\0';	//At the end of the string, add a null terminator
		ssd1306_WriteString(sub_str, font, White); //Write the current line to the buffer
		str_length = str_length - max_char;
		y += font.height;
		line++;
		free(sub_str);
	}
	ssd1306_UpdateScreen();
}



