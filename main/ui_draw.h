#pragma once

#include <stdint.h>

/* Minimal drawing on RGB565 images for the UI (bar, tabs, pages). Text uses a
 * 5x7 font: capital letters, digits and + - : . / < > % ( ) , and space. */

typedef struct {
    uint16_t *px;
    int w, h;
} canvas_t;

#define UI_COLOR_LABEL      0xffff
#define UI_COLOR_VALUE      0xffe0  /* yellow */
#define UI_COLOR_DIM        0x8410
#define UI_COLOR_DIVIDER    0x18e3
#define UI_COLOR_PAGE_BG    0x0841
#define UI_COLOR_OPT        0x2104  /* button */
#define UI_COLOR_OPT_ON     0x03a6  /* selected / on (green) */
#define UI_COLOR_PRESSED    0x5acb

void ui_rect(canvas_t *c, int x, int y, int w, int h, uint16_t color);
/* `scale` 1 = 6x7 pixels per character cell. */
void ui_text(canvas_t *c, int x, int y, const char *text, uint16_t color, int scale);
int ui_text_width(const char *text, int scale);
void ui_text_centered(canvas_t *c, int x0, int x1, int y, const char *text, uint16_t color, int scale);
/* Filled box with a 2-pixel border and a centred label (one or two lines). */
void ui_button(canvas_t *c, int x, int y, int w, int h, const char *line1, const char *line2,
               uint16_t bg, uint16_t fg, int scale);
