#define LCD_CLEAR 0
#define LCD_ROW1 1
#define LCD_ROW2 2
#define LCD_SEL 3

void dispInit(void);
void dispStr(char cmd, char *Str);
char selectButton(void);