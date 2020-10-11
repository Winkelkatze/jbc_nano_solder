#include <stdint.h>

typedef struct {
	const uint8_t FontWidth;    /*!< Font width in pixels */
	const uint8_t FontHeight;   /*!< Font height in pixels */
	const uint8_t FontWidthBits;
	const uint8_t FontHeightBits;
	const uint8_t OffsetTblBits;
	const void *data; /*!< Pointer to data font data array */
	const void *offset_tbl;
} CompressedFont;

extern CompressedFont CompFont_11x18;
extern CompressedFont CompFont_7x10;
extern CompressedFont CompFont_16x26_NUMC;
