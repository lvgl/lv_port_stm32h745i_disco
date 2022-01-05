/*
 * lcd.c
 *
 *  Created on: 23 Dec 2021
 *      Author: Ahmet Alperen Bulut / github.com/ahmetalperenbulut
 */


/*********************
 *      INCLUDES
 *********************/
#include "stm32h745i_discovery.h"
#include "stm32h745i_discovery_lcd.h"
#include "lvgl/lvgl.h"
#include <stdlib.h>
/*********************
 *      DEFINES
 *********************/
#define LCD_INSTANCE				(0)
#define LVGL_BUFFER_ADDR_AT_SDRAM	(0xD007F810)
#define LVGL_BUFFER_2_ADDR_AT_SDRAM (0xD00FF020)

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

static void disp_flush(lv_disp_drv_t *drv, const lv_area_t *area,
		lv_color_t *color_p);
static void disp_clean_dcache(lv_disp_drv_t *drv);
static uint8_t CopyImageToLcdFrameBuffer(void *pSrc, void *pDst, uint32_t xSize,
		uint32_t ySize);

/**********************
 *  STATIC VARIABLES
 **********************/

static lv_disp_t *display = NULL;
static lv_disp_drv_t disp_drv;
static lv_disp_draw_buf_t disp_buf;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * Initialize LCD
 */

void LCD_init()
{
	/* There is only one display on STM32 */
	if (display != NULL)
		abort();

	/* Initialize the LCD */
	BSP_LCD_Init(LCD_INSTANCE, LCD_ORIENTATION_LANDSCAPE);

	BSP_LCD_SetBrightness(LCD_INSTANCE, 100);
	BSP_LCD_DisplayOn(LCD_INSTANCE);

	lv_disp_draw_buf_init(&disp_buf, (void*)LVGL_BUFFER_ADDR_AT_SDRAM,
			(void*)LVGL_BUFFER_2_ADDR_AT_SDRAM,
			Lcd_Ctx[LCD_INSTANCE].XSize * Lcd_Ctx[LCD_INSTANCE].YSize); /*Initialize the display buffer*/

	lv_disp_drv_init(&disp_drv);

	/*Set up the functions to access to your display*/

	/*Set the resolution of the display*/
	disp_drv.hor_res = Lcd_Ctx[LCD_INSTANCE].XSize;
	disp_drv.ver_res = Lcd_Ctx[LCD_INSTANCE].YSize;

	/*Used to copy the buffer's content to the display*/
	disp_drv.flush_cb = disp_flush;
	disp_drv.clean_dcache_cb = disp_clean_dcache;

	/*Set a display buffer*/
	disp_drv.draw_buf = &disp_buf;

	/*Finally register the driver*/
	display = lv_disp_drv_register(&disp_drv);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/* Flush the content of the internal buffer the specific area on the display
 * You can use DMA or any hardware acceleration to do this operation in the background but
 * 'lv_disp_flush_ready()' has to be called when finished*/
static void disp_flush(lv_disp_drv_t *drv, const lv_area_t *area,
		lv_color_t *color_p)
{
	/*Return if the area is out the screen*/
	if (area->x2 < 0)
		return;
	if (area->y2 < 0)
		return;
	if (area->x1 > Lcd_Ctx[LCD_INSTANCE].XSize - 1)
		return;
	if (area->y1 > Lcd_Ctx[LCD_INSTANCE].YSize - 1)
		return;
	//BSP_LED_Toggle(LED2);
	SCB_CleanInvalidateDCache();
	SCB_InvalidateICache();

	uint32_t address =
			hlcd_ltdc.LayerCfg[Lcd_Ctx[LCD_INSTANCE].ActiveLayer].FBStartAdress
					+ (((Lcd_Ctx[LCD_INSTANCE].XSize * area->y1) + area->x1)
							* Lcd_Ctx[LCD_INSTANCE].BppFactor);

	CopyImageToLcdFrameBuffer((void*) color_p, (void*) address,
			lv_area_get_width(area), lv_area_get_height(area));

	lv_disp_flush_ready(&disp_drv);
	return;
}

static void disp_clean_dcache(lv_disp_drv_t *drv)
{
	SCB_CleanInvalidateDCache();
}

/**
 * @brief  Copy to LCD frame buffer area centered in WVGA resolution.
 * The area of copy is of size in ARGB8888.
 * @param  pSrc: Pointer to source buffer : source image buffer start here
 * @param  pDst: Pointer to destination buffer LCD frame buffer center area start here
 * @param  xSize: Buffer width
 * @param  ySize: Buffer height
 * @retval LCD Status : BSP_ERROR_NONE or BSP_ERROR_BUS_DMA_FAILURE
 */
static uint8_t CopyImageToLcdFrameBuffer(void *pSrc, void *pDst, uint32_t xSize,
		uint32_t ySize)
{
	HAL_StatusTypeDef hal_status = HAL_OK;
	uint8_t lcd_status;

	/* Configure the DMA2D Mode, Color Mode and output offset */
	hlcd_dma2d.Init.Mode = DMA2D_M2M_PFC;
	hlcd_dma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888; /* Output color out of PFC */
	hlcd_dma2d.Init.AlphaInverted = DMA2D_REGULAR_ALPHA; /* No Output Alpha Inversion*/
	hlcd_dma2d.Init.RedBlueSwap = DMA2D_RB_REGULAR; /* No Output Red & Blue swap */

	/* Output offset in pixels == nb of pixels to be added at end of line to come to the  */
	/* first pixel of the next line : on the output side of the DMA2D computation         */
	hlcd_dma2d.Init.OutputOffset = LCD_DEFAULT_WIDTH - xSize;

	hlcd_dma2d.Instance = DMA2D;

	/* DMA2D Initialization */
	if (HAL_DMA2D_Init(&hlcd_dma2d) == HAL_OK)
	{
		if (HAL_DMA2D_Start(&hlcd_dma2d, (uint32_t) pSrc, (uint32_t) pDst,
				xSize, ySize) == HAL_OK)
		{
			/* Polling For DMA transfer */
			hal_status = HAL_DMA2D_PollForTransfer(&hlcd_dma2d, 20);
			if (hal_status == HAL_OK)
			{
				/* return good status on exit */
				lcd_status = BSP_ERROR_NONE;
			}
			else
			{
				lcd_status = BSP_ERROR_BUS_DMA_FAILURE;
			}
		}
	}

	return (lcd_status);
}
