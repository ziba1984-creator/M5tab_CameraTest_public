#include <string.h>
#include "ui_draw.h"

/* Five-bit rows, top to bottom. */
static const uint8_t *glyph(char ch)
{
    static const uint8_t blank[7] = {0};
    static const uint8_t digits[10][7] = {
        {14,17,19,21,25,17,14}, {4,12,4,4,4,4,14}, {14,17,1,2,4,8,31}, {31,2,4,2,1,17,14},
        {2,6,10,18,31,2,2}, {31,16,30,1,1,17,14}, {6,8,16,30,17,17,14}, {31,1,2,4,8,8,8},
        {14,17,17,14,17,17,14}, {14,17,17,15,1,2,12},
    };
    static const uint8_t letters[26][7] = {
        {14,17,17,31,17,17,17}, {30,17,17,30,17,17,30}, {14,17,16,16,16,17,14}, {28,18,17,17,17,18,28},
        {31,16,16,30,16,16,31}, {31,16,16,30,16,16,16}, {14,17,16,23,17,17,15}, {17,17,17,31,17,17,17},
        {14,4,4,4,4,4,14},      {7,2,2,2,2,18,12},      {17,18,20,24,20,18,17}, {16,16,16,16,16,16,31},
        {17,27,21,21,17,17,17}, {17,25,21,19,17,17,17}, {14,17,17,17,17,17,14}, {30,17,17,30,16,16,16},
        {14,17,17,17,21,18,13}, {30,17,17,30,20,18,17}, {15,16,16,14,1,1,30},   {31,4,4,4,4,4,4},
        {17,17,17,17,17,17,14}, {17,17,17,17,17,10,4},  {17,17,17,21,21,21,10}, {17,17,10,4,10,17,17},
        {17,17,10,4,4,4,4},     {31,1,2,4,8,16,31},
    };
    static const struct { char ch; uint8_t rows[7]; } symbols[] = {
        { '+', {0,4,4,31,4,4,0} },   { '-', {0,0,0,31,0,0,0} },   { ':', {0,4,4,0,4,4,0} },
        { '.', {0,0,0,0,0,12,12} },  { '/', {1,1,2,4,8,16,16} },  { '<', {2,4,8,16,8,4,2} },
        { '>', {8,4,2,1,2,4,8} },    { '%', {24,25,2,4,8,19,3} }, { '(', {2,4,8,8,8,4,2} },
        { ')', {8,4,2,2,2,4,8} },    { ',', {0,0,0,0,12,4,8} },
    };
    if (ch >= '0' && ch <= '9') return digits[ch - '0'];
    if (ch >= 'A' && ch <= 'Z') return letters[ch - 'A'];
    for (size_t i = 0; i < sizeof(symbols) / sizeof(symbols[0]); i++) {
        if (symbols[i].ch == ch) return symbols[i].rows;
    }
    return blank;
}

void ui_rect(canvas_t *c, int x, int y, int w, int h, uint16_t color)
{
    if (x < 0 || y < 0 || w <= 0 || h <= 0 || x + w > c->w || y + h > c->h) return;
    for (int yy = y; yy < y + h; yy++) {
        for (int xx = x; xx < x + w; xx++) c->px[yy * c->w + xx] = color;
    }
}

void ui_text(canvas_t *c, int x, int y, const char *text, uint16_t color, int scale)
{
    for (; *text; text++, x += 6 * scale) {
        const uint8_t *rows = glyph(*text);
        for (int gy = 0; gy < 7; gy++) {
            for (int gx = 0; gx < 5; gx++) {
                if (rows[gy] & (1 << (4 - gx))) {
                    ui_rect(c, x + gx * scale, y + gy * scale, scale, scale, color);
                }
            }
        }
    }
}

int ui_text_width(const char *text, int scale)
{
    int n = (int)strlen(text);
    return n ? (n * 6 - 1) * scale : 0;
}

void ui_text_centered(canvas_t *c, int x0, int x1, int y, const char *text, uint16_t color, int scale)
{
    ui_text(c, x0 + (x1 - x0 - ui_text_width(text, scale)) / 2, y, text, color, scale);
}

void ui_button(canvas_t *c, int x, int y, int w, int h, const char *line1, const char *line2,
               uint16_t bg, uint16_t fg, int scale)
{
    ui_rect(c, x, y, w, h, UI_COLOR_DIVIDER);
    ui_rect(c, x + 2, y + 2, w - 4, h - 4, bg);
    const int th = 7 * scale;
    if (line2 && line2[0]) {
        int gap = th / 2;
        int top = y + (h - 2 * th - gap) / 2;
        ui_text_centered(c, x, x + w, top, line1, fg, scale);
        ui_text_centered(c, x, x + w, top + th + gap, line2, UI_COLOR_VALUE, scale);
    } else {
        ui_text_centered(c, x, x + w, y + (h - th) / 2, line1, fg, scale);
    }
}
