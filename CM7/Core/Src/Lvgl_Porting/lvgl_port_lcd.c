/*
 * lvgl_port_lcd.c
 *
 *  Updated for LVGL v9.x
 *  Original: Ahmet Alperen Bulut / github.com/ahmetalperenbulut
 */


/*********************
 *      INCLUDES
 *********************/
#include "stm32h745i_discovery.h"
#include "stm32h745i_discovery_lcd.h"
#include "lvgl/lvgl.h"
#include <stdlib.h>
#include <string.h>

/*********************
 *      DEFINES
 *********************/
#define LCD_INSTANCE                (0)
#define LVGL_BUFFER_ADDR_AT_SDRAM   (0xD007F810)
#define LVGL_BUFFER_2_ADDR_AT_SDRAM (0xD00FF020)

/* Cache line size for STM32H7 */
#define CACHE_LINE_SIZE             (32)

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

static void disp_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map);
static uint8_t CopyImageToLcdFrameBuffer(void *pSrc, void *pDst, uint32_t xSize, uint32_t ySize);
static void clean_dcache_by_addr(void *addr, uint32_t size);

/**********************
 *  STATIC VARIABLES
 **********************/

static lv_display_t *display = NULL;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * @brief Initialize LCD for LVGL v9
 */
void LCD_init(void)
{
    /* There is only one display on STM32 */
    if (display != NULL)
        abort();

    /* Initialize the LCD hardware */
    BSP_LCD_Init(LCD_INSTANCE, LCD_ORIENTATION_LANDSCAPE);
    BSP_LCD_SetBrightness(LCD_INSTANCE, 100);
    BSP_LCD_DisplayOn(LCD_INSTANCE);

    /* Create display object */
    display = lv_display_create(Lcd_Ctx[LCD_INSTANCE].XSize, Lcd_Ctx[LCD_INSTANCE].YSize);
    if (display == NULL)
        abort();

    /* Set buffers in SDRAM with proper cache alignment
     * Using double buffering to avoid tearing */
    lv_display_set_buffers(display,
                          (void*)LVGL_BUFFER_ADDR_AT_SDRAM,
                          (void*)LVGL_BUFFER_2_ADDR_AT_SDRAM,
                          Lcd_Ctx[LCD_INSTANCE].XSize * Lcd_Ctx[LCD_INSTANCE].YSize * sizeof(lv_color_t),
                          LV_DISPLAY_RENDER_MODE_PARTIAL);

    /* Set the flush callback */
    lv_display_set_flush_cb(display, disp_flush_cb);

    /* Set color format to ARGB8888 to match hardware */
    lv_display_set_color_format(display, LV_COLOR_FORMAT_ARGB8888);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/**
 * @brief Flush callback for LVGL v9
 * @note This function handles cache coherency issues specific to STM32H745
 */
static void disp_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    /* Validate area bounds */
    if (area->x2 < 0 || area->y2 < 0 ||
        area->x1 > (int32_t)(Lcd_Ctx[LCD_INSTANCE].XSize - 1) ||
        area->y1 > (int32_t)(Lcd_Ctx[LCD_INSTANCE].YSize - 1))
    {
        lv_display_flush_ready(disp);
        return;
    }

    /* Calculate area size for cache operations */
    uint32_t width = lv_area_get_width(area);
    uint32_t height = lv_area_get_height(area);
    uint32_t buffer_size = width * height * sizeof(lv_color_t);

    /* Clean D-Cache for the source buffer to ensure DMA2D reads correct data
     * This is critical to prevent smearing artifacts on STM32H745 */
    clean_dcache_by_addr((void*)px_map, buffer_size);

    /* Calculate destination address in framebuffer */
    uint32_t fb_address = hlcd_ltdc.LayerCfg[Lcd_Ctx[LCD_INSTANCE].ActiveLayer].FBStartAdress
                         + (((Lcd_Ctx[LCD_INSTANCE].XSize * area->y1) + area->x1)
                         * Lcd_Ctx[LCD_INSTANCE].BppFactor);

    /* Copy buffer to framebuffer using DMA2D */
    CopyImageToLcdFrameBuffer((void*)px_map, (void*)fb_address, width, height);

    /* Clean D-Cache for destination framebuffer region
     * This ensures the LTDC peripheral sees the updated data */
    uint32_t fb_line_size = Lcd_Ctx[LCD_INSTANCE].XSize * Lcd_Ctx[LCD_INSTANCE].BppFactor;
    for (uint32_t y = 0; y < height; y++)
    {
        void *line_addr = (void*)(fb_address + y * fb_line_size);
        clean_dcache_by_addr(line_addr, width * Lcd_Ctx[LCD_INSTANCE].BppFactor);
    }

    /* Notify LVGL that flushing is complete */
    lv_display_flush_ready(disp);
}

/**
 * @brief Clean D-Cache for specific address range
 * @note This is more efficient than full cache clean and prevents smearing
 */
static void clean_dcache_by_addr(void *addr, uint32_t size)
{
    /* Align address to cache line boundary */
    uint32_t aligned_addr = (uint32_t)addr & ~(CACHE_LINE_SIZE - 1);
    uint32_t aligned_size = size + ((uint32_t)addr - aligned_addr);

    /* Round up size to cache line boundary */
    aligned_size = (aligned_size + CACHE_LINE_SIZE - 1) & ~(CACHE_LINE_SIZE - 1);

    /* Clean D-Cache for the specified region */
    SCB_CleanDCache_by_Addr((uint32_t*)aligned_addr, aligned_size);
}

/**
 * @brief Copy to LCD frame buffer using DMA2D
 * @param pSrc: Pointer to source buffer
 * @param pDst: Pointer to destination buffer (LCD framebuffer)
 * @param xSize: Buffer width in pixels
 * @param ySize: Buffer height in pixels
 * @retval LCD Status: BSP_ERROR_NONE or BSP_ERROR_BUS_DMA_FAILURE
 */
static uint8_t CopyImageToLcdFrameBuffer(void *pSrc, void *pDst, uint32_t xSize, uint32_t ySize)
{
    HAL_StatusTypeDef hal_status = HAL_OK;
    uint8_t lcd_status;

    /* Configure DMA2D for memory-to-memory with pixel format conversion */
    hlcd_dma2d.Init.Mode = DMA2D_M2M_PFC;
    hlcd_dma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
    hlcd_dma2d.Init.AlphaInverted = DMA2D_REGULAR_ALPHA;
    hlcd_dma2d.Init.RedBlueSwap = DMA2D_RB_REGULAR;

    /* Set output offset to handle non-full-width transfers */
    hlcd_dma2d.Init.OutputOffset = LCD_DEFAULT_WIDTH - xSize;

    hlcd_dma2d.Instance = DMA2D;

    /* Initialize and start DMA2D transfer */
    if (HAL_DMA2D_Init(&hlcd_dma2d) == HAL_OK)
    {
        if (HAL_DMA2D_Start(&hlcd_dma2d, (uint32_t)pSrc, (uint32_t)pDst, xSize, ySize) == HAL_OK)
        {
            /* Wait for DMA2D transfer to complete */
            hal_status = HAL_DMA2D_PollForTransfer(&hlcd_dma2d, 20);
            lcd_status = (hal_status == HAL_OK) ? BSP_ERROR_NONE : BSP_ERROR_BUS_DMA_FAILURE;
        }
        else
        {
            lcd_status = BSP_ERROR_BUS_DMA_FAILURE;
        }
    }
    else
    {
        lcd_status = BSP_ERROR_BUS_DMA_FAILURE;
    }

    return lcd_status;
}
