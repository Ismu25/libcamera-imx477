// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Sony IMX477 cameras.
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd
 *
 * Based on Sony imx219 camera driver
 * Copyright (C) 2019-2020 Raspberry Pi (Trading) Ltd
 */
#include <asm/unaligned.h> // Biblioteca para acceder a la memoria sin alineación adecuada.
#include <linux/clk.h> // Biblioteca para el acceso y control del Common Clock Framework.
#include <linux/delay.h> // Biblioteca para el uso de funciones de espera y retardos.
#include <linux/gpio/consumer.h> // Biblioteca para la gestión de la entrada/salida general-purpose (GPIO).
#include <linux/i2c.h> // Biblioteca para la interfaz y el uso del bus I2C para la comunicación serial.
#include <linux/module.h> // Biblioteca para la creación y gestión de módulos del kernel (código cargable para extender funcionalidad).
#include <linux/of_device.h> // Biblioteca para manejar descriptores de dispositivos basados en Device Tree.
#include <linux/pm_runtime.h> // Biblioteca para la gestión de energía en dispositivos de entrada/salida (I/O).
#include <linux/regulator/consumer.h> // Biblioteca para el manejo de reguladores de energía en SoCs.

// V4L2 : API para la captura de video para Linux
#include <media/v4l2-ctrls.h> // Biblioteca para el manejo de controles V4L2.
#include <media/v4l2-device.h> // Biblioteca para la gestión de dispositivos V4L2.
#include <media/v4l2-event.h> // Biblioteca para el manejo de eventos V4L2.
#include <media/v4l2-fwnode.h> // Biblioteca para manejar nodos de firmware V4L2.
#include <media/v4l2-mediabus.h> // Biblioteca para la gestión de la media bus API en V4L2.

/* El uso de module_param permite el paso de argumentos al modulo 
   parecido argc/argv 
*/ 

static int dpc_enable = 1; // Activación de la corrección de píxeles defectuosos (DPC).
module_param(dpc_enable, int, 0644); // Define el parámetro del módulo y establece sus permisos.
MODULE_PARM_DESC(dpc_enable, "Enable on-sensor DPC"); // Establece la descripción del parámetro 'dpc_enable'.

static int trigger_mode; // Activación del modo de sincronización vertical (vsync).
module_param(trigger_mode, int, 0644); // Establece la descripción del parámetro 'trigger_mode'.
// TODO: CHECK trigger_mode
MODULE_PARM_DESC(trigger_mode, "Set vsync trigger mode: 1=source, 2=sink"); // Establece la descripcion
// 1 = source: La sincronización vertical (vsync) es generada por la cámara.
// 2 = sink: La sincronización vertical (vsync) es generada por el software.

/* Tamaños de los registros en bytes */
#define IMX477_REG_VALUE_08BIT   1 // Longitud en bytes de un registro de 8 bits
#define IMX477_REG_VALUE_16BIT   2 // Longitud en bytes de un registro de 16 bits

/* Chip IDs */ 
#define IMX477_REG_CHIP_ID       0x0016  // Dirección del registro que guarda el ID del sensor
#define IMX477_CHIP_ID           0x0477  // ID del sensor IMX477
#define IMX378_CHIP_ID           0x0378  // ID del sensor IMX378

/* Modos de funcionamiento */
#define IMX477_REG_MODE_SELECT   0x0100  // Dirección del registro que guarda el modo de funcionamiento
#define IMX477_MODE_STANDBY      0x00    // Modo Standby
#define IMX477_MODE_STREAMING    0x01    // Modo Streaming

#define IMX477_REG_ORIENTATION   0x0101  // Dirección del registro que guarda la orientación del sensor

#define IMX477_XCLK_FREQ         24000000  // Frecuencia del reloj externo al sensor (24 MHz)

#define IMX477_DEFAULT_LINK_FREQ 450000000 // Velocidad de comunicación con el sistema (450 MHz)

/* Pixel rate is fixed at 840MHz for all the modes
   El ratio de los píxeles es constante a 840 MHz en todos los modos disponibles */
#define IMX477_PIXEL_RATE       840000000 // Ratio de transmisión y recepción de los datos de los píxeles

/* V_TIMING internal : Parámetros de temporización vertical */ 
#define IMX477_REG_FRAME_LENGTH 0x0340 // Registro que indica la longitud del frame
#define IMX477_FRAME_LENGTH_MAX 0xffdc // Longitud máxima de un frame

/* H_TIMING internal : Parámetros de temporización horizontal */ 
#define IMX477_REG_LINE_LENGTH  0x0342 // Registro que indica la longitud de una línea
#define IMX477_LINE_LENGTH_MAX  0xfff0 // Longitud máxima de una línea

/* Long exposure multiplier: */ // Variables usadas para manejar la exposición larga
#define IMX477_LONG_EXP_SHIFT_MAX  7       // Multiplicador máximo para la exposición larga
#define IMX477_LONG_EXP_SHIFT_REG  0x3100  // Registro para la indicación de la exposición larga

/* Exposure control: Control de la exposición */
#define IMX477_REG_EXPOSURE          0x0202      // Registro para el control de exposición
#define IMX477_EXPOSURE_OFFSET       22          // Ajuste de la exposición del sensor
#define IMX477_EXPOSURE_MIN          4           // Valor mínimo para la exposición
#define IMX477_EXPOSURE_STEP         1           // Paso de ajuste de la exposición
#define IMX477_EXPOSURE_DEFAULT      0x640       // Valor predeterminado de la exposición
// Valor máximo de exposición calculado teniendo en cuenta el offset
#define IMX477_EXPOSURE_MAX          (IMX477_FRAME_LENGTH_MAX - IMX477_EXPOSURE_OFFSET)
                               
/* Analog gain control: Control de ganancia analógica */
#define IMX477_REG_ANALOG_GAIN       0x0204  // Registro para el control de ganancia analógica
#define IMX477_ANA_GAIN_MIN          0       // Ganancia analógica mínima
#define IMX477_ANA_GAIN_MAX          978     // Ganancia analógica máxima
#define IMX477_ANA_GAIN_STEP         1       // Paso de ajuste de ganancia analógica
#define IMX477_ANA_GAIN_DEFAULT      0x0     // Valor por defecto de la ganancia analógica

/* Digital gain control: Control de ganancia digital */
#define IMX477_REG_DIGITAL_GAIN     0x020e  // Registro para el control de ganancia digital
#define IMX477_DGTL_GAIN_MIN        0x0100  // Ganancia digital mínima
#define IMX477_DGTL_GAIN_MAX        0xffff  // Ganancia digital máxima
#define IMX477_DGTL_GAIN_DEFAULT    0x0100  // Valor por defecto de la ganancia digital
#define IMX477_DGTL_GAIN_STEP       1       // Paso de ajuste de ganancia digital

/* Test Pattern Control: Control de patrón de prueba */
#define IMX477_REG_TEST_PATTERN         0x0600  // Registro para el control del patrón de prueba
#define IMX477_TEST_PATTERN_DISABLE     0       // Patrón de prueba desactivado
#define IMX477_TEST_PATTERN_SOLID_COLOR 1       // Patrón de prueba de color sólido
#define IMX477_TEST_PATTERN_COLOR_BARS  2       // Patrón de prueba de barras de color
#define IMX477_TEST_PATTERN_GREY_COLOR  3       // Patrón de prueba de escala de grises
#define IMX477_TEST_PATTERN_PN9         4       // Patrón de prueba PN9

/* Test pattern colour components: Componentes de color del patrón de prueba */
#define IMX477_REG_TEST_PATTERN_R       0x0602  // Registro del componente de color rojo del patrón de prueba
#define IMX477_REG_TEST_PATTERN_GR      0x0604  // Registro del componente de color verde (rojo) del patrón de prueba
#define IMX477_REG_TEST_PATTERN_B       0x0606  // Registro del componente de color azul del patrón de prueba
#define IMX477_REG_TEST_PATTERN_GB      0x0608  // Registro del componente de color verde (azul) del patrón de prueba
#define IMX477_TEST_PATTERN_COLOUR_MIN  0       // Valor mínimo para los componentes de color
#define IMX477_TEST_PATTERN_COLOUR_MAX  0x0fff  // Valor máximo para los componentes de color
#define IMX477_TEST_PATTERN_COLOUR_STEP 1       // Paso de ajuste para los componentes de color
#define IMX477_TEST_PATTERN_R_DEFAULT   IMX477_TEST_PATTERN_COLOUR_MAX  // Valor por defecto para el componente rojo
#define IMX477_TEST_PATTERN_GR_DEFAULT  0       // Valor por defecto para el componente verde (rojo)
#define IMX477_TEST_PATTERN_B_DEFAULT   0       // Valor por defecto para el componente azul
#define IMX477_TEST_PATTERN_GB_DEFAULT  0       // Valor por defecto para el componente verde (azul)

/* Trigger mode: Modo de activación */
#define IMX477_REG_MC_MODE      0x3f0b    // Registro para el modo de control de operación
#define IMX477_REG_MS_SEL       0x3041    // Registro para la selección de modo Master/Slave
#define IMX477_REG_XVS_IO_CTRL  0x3040    // Registro para el control de sincronización vertical externa
#define IMX477_REG_EXTOUT_EN    0x4b81    // Registro para la configuración de señales de salida

/* Embedded metadata stream structure: Estructura de flujo de metadatos */
#define IMX477_EMBEDDED_LINE_WIDTH   16384   // Longitud de la línea de metadatos incrustados
#define IMX477_NUM_EMBEDDED_LINES    1       // Número de líneas para los metadatos incrustados

/* Tipos de pads */
enum pad_types {
    IMAGE_PAD,      // Pad de imagen
    METADATA_PAD,   // Pad de metadatos
    NUM_PADS        // Número de pads
};

/* IMX477 native and active pixel array size: Tamaño del array de píxeles nativos y activos */
#define IMX477_NATIVE_WIDTH         4072U   // Anchura nativa del sensor
#define IMX477_NATIVE_HEIGHT        3176U   // Altura nativa del sensor
#define IMX477_PIXEL_ARRAY_LEFT     8U      // El array comienza 8 píxeles a la izquierda
#define IMX477_PIXEL_ARRAY_TOP      16U     // El array comienza a 16 píxeles desde arriba
#define IMX477_PIXEL_ARRAY_WIDTH    4056U   // Anchura del array de píxeles
#define IMX477_PIXEL_ARRAY_HEIGHT   3040U   // Altura del array de píxeles

/* Estructura de un Registro con su valor y dirección */
struct imx477_reg { 
    u16 address;    // Dirección del registro
    u8 val;         // Valor del registro
};

/* Estructura de una lista de registros */
struct imx477_reg_list {
    unsigned int num_of_regs;       // Número de registros en la lista
    const struct imx477_reg *regs;  // Puntero a un array de registros
};

/* Mode: resolution and related config & values 
   Modo de funcionamiento de la cámara con configuraciones y sus valores */
struct imx477_mode {
    /* Frame width */
    unsigned int width;                         // Anchura del frame

    /* Frame height */
    unsigned int height;                        // Altura del frame

    /* H-timing in pixels */
    unsigned int line_length_pix;               // Longitud de la línea en píxeles

    /* Analog crop rectangle */
    struct v4l2_rect crop;                      // Rectángulo de recorte analógico

    /* Highest possible framerate */
    struct v4l2_fract timeperframe_min;         // Tiempo mínimo por frame

    /* Default framerate */
    struct v4l2_fract timeperframe_default;     // Tiempo por frame por defecto

    /* Default register values */
    struct imx477_reg_list reg_list;            // Lista de valores de registro por defecto
};

/* Frecuencias de link compatibles (solo por defecto) */
static const s64 imx477_link_freq_menu[] = {
    IMX477_DEFAULT_LINK_FREQ, // Frecuencia de enlace por defecto
};

/* Lista de Registros en modo comun */
static const struct imx477_reg mode_common_regs[] = {
	{0x0136, 0x18},
	{0x0137, 0x00},
	{0x0138, 0x01},
	{0xe000, 0x00},
	{0xe07a, 0x01},
	{0x0808, 0x02},
	{0x4ae9, 0x18},
	{0x4aea, 0x08},
	{0xf61c, 0x04},
	{0xf61e, 0x04},
	{0x4ae9, 0x21},
	{0x4aea, 0x80},
	{0x38a8, 0x1f},
	{0x38a9, 0xff},
	{0x38aa, 0x1f},
	{0x38ab, 0xff},
	{0x55d4, 0x00},
	{0x55d5, 0x00},
	{0x55d6, 0x07},
	{0x55d7, 0xff},
	{0x55e8, 0x07},
	{0x55e9, 0xff},
	{0x55ea, 0x00},
	{0x55eb, 0x00},
	{0x574c, 0x07},
	{0x574d, 0xff},
	{0x574e, 0x00},
	{0x574f, 0x00},
	{0x5754, 0x00},
	{0x5755, 0x00},
	{0x5756, 0x07},
	{0x5757, 0xff},
	{0x5973, 0x04},
	{0x5974, 0x01},
	{0x5d13, 0xc3},
	{0x5d14, 0x58},
	{0x5d15, 0xa3},
	{0x5d16, 0x1d},
	{0x5d17, 0x65},
	{0x5d18, 0x8c},
	{0x5d1a, 0x06},
	{0x5d1b, 0xa9},
	{0x5d1c, 0x45},
	{0x5d1d, 0x3a},
	{0x5d1e, 0xab},
	{0x5d1f, 0x15},
	{0x5d21, 0x0e},
	{0x5d22, 0x52},
	{0x5d23, 0xaa},
	{0x5d24, 0x7d},
	{0x5d25, 0x57},
	{0x5d26, 0xa8},
	{0x5d37, 0x5a},
	{0x5d38, 0x5a},
	{0x5d77, 0x7f},
	{0x7b75, 0x0e},
	{0x7b76, 0x0b},
	{0x7b77, 0x08},
	{0x7b78, 0x0a},
	{0x7b79, 0x47},
	{0x7b7c, 0x00},
	{0x7b7d, 0x00},
	{0x8d1f, 0x00},
	{0x8d27, 0x00},
	{0x9004, 0x03},
	{0x9200, 0x50},
	{0x9201, 0x6c},
	{0x9202, 0x71},
	{0x9203, 0x00},
	{0x9204, 0x71},
	{0x9205, 0x01},
	{0x9371, 0x6a},
	{0x9373, 0x6a},
	{0x9375, 0x64},
	{0x991a, 0x00},
	{0x996b, 0x8c},
	{0x996c, 0x64},
	{0x996d, 0x50},
	{0x9a4c, 0x0d},
	{0x9a4d, 0x0d},
	{0xa001, 0x0a},
	{0xa003, 0x0a},
	{0xa005, 0x0a},
	{0xa006, 0x01},
	{0xa007, 0xc0},
	{0xa009, 0xc0},
	{0x3d8a, 0x01},
	{0x4421, 0x04},
	{0x7b3b, 0x01},
	{0x7b4c, 0x00},
	{0x9905, 0x00},
	{0x9907, 0x00},
	{0x9909, 0x00},
	{0x990b, 0x00},
	{0x9944, 0x3c},
	{0x9947, 0x3c},
	{0x994a, 0x8c},
	{0x994b, 0x50},
	{0x994c, 0x1b},
	{0x994d, 0x8c},
	{0x994e, 0x50},
	{0x994f, 0x1b},
	{0x9950, 0x8c},
	{0x9951, 0x1b},
	{0x9952, 0x0a},
	{0x9953, 0x8c},
	{0x9954, 0x1b},
	{0x9955, 0x0a},
	{0x9a13, 0x04},
	{0x9a14, 0x04},
	{0x9a19, 0x00},
	{0x9a1c, 0x04},
	{0x9a1d, 0x04},
	{0x9a26, 0x05},
	{0x9a27, 0x05},
	{0x9a2c, 0x01},
	{0x9a2d, 0x03},
	{0x9a2f, 0x05},
	{0x9a30, 0x05},
	{0x9a41, 0x00},
	{0x9a46, 0x00},
	{0x9a47, 0x00},
	{0x9c17, 0x35},
	{0x9c1d, 0x31},
	{0x9c29, 0x50},
	{0x9c3b, 0x2f},
	{0x9c41, 0x6b},
	{0x9c47, 0x2d},
	{0x9c4d, 0x40},
	{0x9c6b, 0x00},
	{0x9c71, 0xc8},
	{0x9c73, 0x32},
	{0x9c75, 0x04},
	{0x9c7d, 0x2d},
	{0x9c83, 0x40},
	{0x9c94, 0x3f},
	{0x9c95, 0x3f},
	{0x9c96, 0x3f},
	{0x9c97, 0x00},
	{0x9c98, 0x00},
	{0x9c99, 0x00},
	{0x9c9a, 0x3f},
	{0x9c9b, 0x3f},
	{0x9c9c, 0x3f},
	{0x9ca0, 0x0f},
	{0x9ca1, 0x0f},
	{0x9ca2, 0x0f},
	{0x9ca3, 0x00},
	{0x9ca4, 0x00},
	{0x9ca5, 0x00},
	{0x9ca6, 0x1e},
	{0x9ca7, 0x1e},
	{0x9ca8, 0x1e},
	{0x9ca9, 0x00},
	{0x9caa, 0x00},
	{0x9cab, 0x00},
	{0x9cac, 0x09},
	{0x9cad, 0x09},
	{0x9cae, 0x09},
	{0x9cbd, 0x50},
	{0x9cbf, 0x50},
	{0x9cc1, 0x50},
	{0x9cc3, 0x40},
	{0x9cc5, 0x40},
	{0x9cc7, 0x40},
	{0x9cc9, 0x0a},
	{0x9ccb, 0x0a},
	{0x9ccd, 0x0a},
	{0x9d17, 0x35},
	{0x9d1d, 0x31},
	{0x9d29, 0x50},
	{0x9d3b, 0x2f},
	{0x9d41, 0x6b},
	{0x9d47, 0x42},
	{0x9d4d, 0x5a},
	{0x9d6b, 0x00},
	{0x9d71, 0xc8},
	{0x9d73, 0x32},
	{0x9d75, 0x04},
	{0x9d7d, 0x42},
	{0x9d83, 0x5a},
	{0x9d94, 0x3f},
	{0x9d95, 0x3f},
	{0x9d96, 0x3f},
	{0x9d97, 0x00},
	{0x9d98, 0x00},
	{0x9d99, 0x00},
	{0x9d9a, 0x3f},
	{0x9d9b, 0x3f},
	{0x9d9c, 0x3f},
	{0x9d9d, 0x1f},
	{0x9d9e, 0x1f},
	{0x9d9f, 0x1f},
	{0x9da0, 0x0f},
	{0x9da1, 0x0f},
	{0x9da2, 0x0f},
	{0x9da3, 0x00},
	{0x9da4, 0x00},
	{0x9da5, 0x00},
	{0x9da6, 0x1e},
	{0x9da7, 0x1e},
	{0x9da8, 0x1e},
	{0x9da9, 0x00},
	{0x9daa, 0x00},
	{0x9dab, 0x00},
	{0x9dac, 0x09},
	{0x9dad, 0x09},
	{0x9dae, 0x09},
	{0x9dc9, 0x0a},
	{0x9dcb, 0x0a},
	{0x9dcd, 0x0a},
	{0x9e17, 0x35},
	{0x9e1d, 0x31},
	{0x9e29, 0x50},
	{0x9e3b, 0x2f},
	{0x9e41, 0x6b},
	{0x9e47, 0x2d},
	{0x9e4d, 0x40},
	{0x9e6b, 0x00},
	{0x9e71, 0xc8},
	{0x9e73, 0x32},
	{0x9e75, 0x04},
	{0x9e94, 0x0f},
	{0x9e95, 0x0f},
	{0x9e96, 0x0f},
	{0x9e97, 0x00},
	{0x9e98, 0x00},
	{0x9e99, 0x00},
	{0x9ea0, 0x0f},
	{0x9ea1, 0x0f},
	{0x9ea2, 0x0f},
	{0x9ea3, 0x00},
	{0x9ea4, 0x00},
	{0x9ea5, 0x00},
	{0x9ea6, 0x3f},
	{0x9ea7, 0x3f},
	{0x9ea8, 0x3f},
	{0x9ea9, 0x00},
	{0x9eaa, 0x00},
	{0x9eab, 0x00},
	{0x9eac, 0x09},
	{0x9ead, 0x09},
	{0x9eae, 0x09},
	{0x9ec9, 0x0a},
	{0x9ecb, 0x0a},
	{0x9ecd, 0x0a},
	{0x9f17, 0x35},
	{0x9f1d, 0x31},
	{0x9f29, 0x50},
	{0x9f3b, 0x2f},
	{0x9f41, 0x6b},
	{0x9f47, 0x42},
	{0x9f4d, 0x5a},
	{0x9f6b, 0x00},
	{0x9f71, 0xc8},
	{0x9f73, 0x32},
	{0x9f75, 0x04},
	{0x9f94, 0x0f},
	{0x9f95, 0x0f},
	{0x9f96, 0x0f},
	{0x9f97, 0x00},
	{0x9f98, 0x00},
	{0x9f99, 0x00},
	{0x9f9a, 0x2f},
	{0x9f9b, 0x2f},
	{0x9f9c, 0x2f},
	{0x9f9d, 0x00},
	{0x9f9e, 0x00},
	{0x9f9f, 0x00},
	{0x9fa0, 0x0f},
	{0x9fa1, 0x0f},
	{0x9fa2, 0x0f},
	{0x9fa3, 0x00},
	{0x9fa4, 0x00},
	{0x9fa5, 0x00},
	{0x9fa6, 0x1e},
	{0x9fa7, 0x1e},
	{0x9fa8, 0x1e},
	{0x9fa9, 0x00},
	{0x9faa, 0x00},
	{0x9fab, 0x00},
	{0x9fac, 0x09},
	{0x9fad, 0x09},
	{0x9fae, 0x09},
	{0x9fc9, 0x0a},
	{0x9fcb, 0x0a},
	{0x9fcd, 0x0a},
	{0xa14b, 0xff},
	{0xa151, 0x0c},
	{0xa153, 0x50},
	{0xa155, 0x02},
	{0xa157, 0x00},
	{0xa1ad, 0xff},
	{0xa1b3, 0x0c},
	{0xa1b5, 0x50},
	{0xa1b9, 0x00},
	{0xa24b, 0xff},
	{0xa257, 0x00},
	{0xa2ad, 0xff},
	{0xa2b9, 0x00},
	{0xb21f, 0x04},
	{0xb35c, 0x00},
	{0xb35e, 0x08},
	{0x0112, 0x0c},
	{0x0113, 0x0c},
	{0x0114, 0x01},
	{0x0350, 0x00},
	{0xbcf1, 0x02},
	{0x3ff9, 0x01},
};

/* 12 mpix 10fps :  Lista de Registros en modo 12 Mp 10 fps */
static const struct imx477_reg mode_4056x3040_regs[] = {
	{0x0342, 0x5d},
	{0x0343, 0xc0},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x0f},
	{0x0349, 0xd7},
	{0x034a, 0x0b},
	{0x034b, 0xdf},
	{0x00e3, 0x00},
	{0x00e4, 0x00},
	{0x00fc, 0x0a},
	{0x00fd, 0x0a},
	{0x00fe, 0x0a},
	{0x00ff, 0x0a},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x00},
	{0x0901, 0x11},
	{0x0902, 0x02},
	{0x3140, 0x02},
	{0x3c00, 0x00},
	{0x3c01, 0x03},
	{0x3c02, 0xa2},
	{0x3f0d, 0x01},
	{0x5748, 0x07},
	{0x5749, 0xff},
	{0x574a, 0x00},
	{0x574b, 0x00},
	{0x7b75, 0x0a},
	{0x7b76, 0x0c},
	{0x7b77, 0x07},
	{0x7b78, 0x06},
	{0x7b79, 0x3c},
	{0x7b53, 0x01},
	{0x9369, 0x5a},
	{0x936b, 0x55},
	{0x936d, 0x28},
	{0x9304, 0x00},
	{0x9305, 0x00},
	{0x9e9a, 0x2f},
	{0x9e9b, 0x2f},
	{0x9e9c, 0x2f},
	{0x9e9d, 0x00},
	{0x9e9e, 0x00},
	{0x9e9f, 0x00},
	{0xa2a9, 0x60},
	{0xa2b7, 0x00},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x0f},
	{0x040d, 0xd8},
	{0x040e, 0x0b},
	{0x040f, 0xe0},
	{0x034c, 0x0f},
	{0x034d, 0xd8},
	{0x034e, 0x0b},
	{0x034f, 0xe0},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x01},
	{0x0307, 0x5e},
	{0x0309, 0x0c},
	{0x030b, 0x02},
	{0x030d, 0x02},
	{0x030e, 0x00},
	{0x030f, 0x96},
	{0x0310, 0x01},
	{0x0820, 0x07},
	{0x0821, 0x08},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x080a, 0x00},
	{0x080b, 0x7f},
	{0x080c, 0x00},
	{0x080d, 0x4f},
	{0x080e, 0x00},
	{0x080f, 0x77},
	{0x0810, 0x00},
	{0x0811, 0x5f},
	{0x0812, 0x00},
	{0x0813, 0x57},
	{0x0814, 0x00},
	{0x0815, 0x4f},
	{0x0816, 0x01},
	{0x0817, 0x27},
	{0x0818, 0x00},
	{0x0819, 0x3f},
	{0xe04c, 0x00},
	{0xe04d, 0x7f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3e20, 0x01},
	{0x3e37, 0x00},
	{0x3f50, 0x00},
	{0x3f56, 0x02},
	{0x3f57, 0xae},
};

/* 2x2 binned. 40fps : Lista de Registros en modo 5Mp 40fps con union de pixeles de 2x2 */ 
static const struct imx477_reg mode_2028x1520_regs[] = {
	{0x0342, 0x31},
	{0x0343, 0xc4},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x00},
	{0x0347, 0x00},
	{0x0348, 0x0f},
	{0x0349, 0xd7},
	{0x034a, 0x0b},
	{0x034b, 0xdf},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x22},
	{0x0902, 0x02},
	{0x3140, 0x02},
	{0x3c00, 0x00},
	{0x3c01, 0x03},
	{0x3c02, 0xa2},
	{0x3f0d, 0x01},
	{0x5748, 0x07},
	{0x5749, 0xff},
	{0x574a, 0x00},
	{0x574b, 0x00},
	{0x7b53, 0x01},
	{0x9369, 0x73},
	{0x936b, 0x64},
	{0x936d, 0x5f},
	{0x9304, 0x00},
	{0x9305, 0x00},
	{0x9e9a, 0x2f},
	{0x9e9b, 0x2f},
	{0x9e9c, 0x2f},
	{0x9e9d, 0x00},
	{0x9e9e, 0x00},
	{0x9e9f, 0x00},
	{0xa2a9, 0x60},
	{0xa2b7, 0x00},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x20},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x0f},
	{0x040d, 0xd8},
	{0x040e, 0x0b},
	{0x040f, 0xe0},
	{0x034c, 0x07},
	{0x034d, 0xec},
	{0x034e, 0x05},
	{0x034f, 0xf0},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x01},
	{0x0307, 0x5e},
	{0x0309, 0x0c},
	{0x030b, 0x02},
	{0x030d, 0x02},
	{0x030e, 0x00},
	{0x030f, 0x96},
	{0x0310, 0x01},
	{0x0820, 0x07},
	{0x0821, 0x08},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x080a, 0x00},
	{0x080b, 0x7f},
	{0x080c, 0x00},
	{0x080d, 0x4f},
	{0x080e, 0x00},
	{0x080f, 0x77},
	{0x0810, 0x00},
	{0x0811, 0x5f},
	{0x0812, 0x00},
	{0x0813, 0x57},
	{0x0814, 0x00},
	{0x0815, 0x4f},
	{0x0816, 0x01},
	{0x0817, 0x27},
	{0x0818, 0x00},
	{0x0819, 0x3f},
	{0xe04c, 0x00},
	{0xe04d, 0x7f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3e20, 0x01},
	{0x3e37, 0x00},
	{0x3f50, 0x00},
	{0x3f56, 0x01},
	{0x3f57, 0x6c},
};

/* 1080p cropped mode : Lista de Registros en modo 1080 recortado*/ 
static const struct imx477_reg mode_2028x1080_regs[] = {
	{0x0342, 0x31},
	{0x0343, 0xc4},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x01},
	{0x0347, 0xb8},
	{0x0348, 0x0f},
	{0x0349, 0xd7},
	{0x034a, 0x0a},
	{0x034b, 0x27},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x22},
	{0x0902, 0x02},
	{0x3140, 0x02},
	{0x3c00, 0x00},
	{0x3c01, 0x03},
	{0x3c02, 0xa2},
	{0x3f0d, 0x01},
	{0x5748, 0x07},
	{0x5749, 0xff},
	{0x574a, 0x00},
	{0x574b, 0x00},
	{0x7b53, 0x01},
	{0x9369, 0x73},
	{0x936b, 0x64},
	{0x936d, 0x5f},
	{0x9304, 0x00},
	{0x9305, 0x00},
	{0x9e9a, 0x2f},
	{0x9e9b, 0x2f},
	{0x9e9c, 0x2f},
	{0x9e9d, 0x00},
	{0x9e9e, 0x00},
	{0x9e9f, 0x00},
	{0xa2a9, 0x60},
	{0xa2b7, 0x00},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x20},
	{0x0408, 0x00},
	{0x0409, 0x00},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x0f},
	{0x040d, 0xd8},
	{0x040e, 0x04},
	{0x040f, 0x38},
	{0x034c, 0x07},
	{0x034d, 0xec},
	{0x034e, 0x04},
	{0x034f, 0x38},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x04},
	{0x0306, 0x01},
	{0x0307, 0x5e},
	{0x0309, 0x0c},
	{0x030b, 0x02},
	{0x030d, 0x02},
	{0x030e, 0x00},
	{0x030f, 0x96},
	{0x0310, 0x01},
	{0x0820, 0x07},
	{0x0821, 0x08},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x080a, 0x00},
	{0x080b, 0x7f},
	{0x080c, 0x00},
	{0x080d, 0x4f},
	{0x080e, 0x00},
	{0x080f, 0x77},
	{0x0810, 0x00},
	{0x0811, 0x5f},
	{0x0812, 0x00},
	{0x0813, 0x57},
	{0x0814, 0x00},
	{0x0815, 0x4f},
	{0x0816, 0x01},
	{0x0817, 0x27},
	{0x0818, 0x00},
	{0x0819, 0x3f},
	{0xe04c, 0x00},
	{0xe04d, 0x7f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3e20, 0x01},
	{0x3e37, 0x00},
	{0x3f50, 0x00},
	{0x3f56, 0x01},
	{0x3f57, 0x6c},
};

/* 4x4 binned. 120fps : Lista de Registros en modo 120 fps con unión de pixeles de 4x4p */
static const struct imx477_reg mode_1332x990_regs[] = {
	{0x420b, 0x01},
	{0x990c, 0x00},
	{0x990d, 0x08},
	{0x9956, 0x8c},
	{0x9957, 0x64},
	{0x9958, 0x50},
	{0x9a48, 0x06},
	{0x9a49, 0x06},
	{0x9a4a, 0x06},
	{0x9a4b, 0x06},
	{0x9a4c, 0x06},
	{0x9a4d, 0x06},
	{0x0112, 0x0a},
	{0x0113, 0x0a},
	{0x0114, 0x01},
	{0x0342, 0x1a},
	{0x0343, 0x08},
	{0x0340, 0x04},
	{0x0341, 0x1a},
	{0x0344, 0x00},
	{0x0345, 0x00},
	{0x0346, 0x02},
	{0x0347, 0x10},
	{0x0348, 0x0f},
	{0x0349, 0xd7},
	{0x034a, 0x09},
	{0x034b, 0xcf},
	{0x00e3, 0x00},
	{0x00e4, 0x00},
	{0x00fc, 0x0a},
	{0x00fd, 0x0a},
	{0x00fe, 0x0a},
	{0x00ff, 0x0a},
	{0xe013, 0x00},
	{0x0220, 0x00},
	{0x0221, 0x11},
	{0x0381, 0x01},
	{0x0383, 0x01},
	{0x0385, 0x01},
	{0x0387, 0x01},
	{0x0900, 0x01},
	{0x0901, 0x22},
	{0x0902, 0x02},
	{0x3140, 0x02},
	{0x3c00, 0x00},
	{0x3c01, 0x01},
	{0x3c02, 0x9c},
	{0x3f0d, 0x00},
	{0x5748, 0x00},
	{0x5749, 0x00},
	{0x574a, 0x00},
	{0x574b, 0xa4},
	{0x7b75, 0x0e},
	{0x7b76, 0x09},
	{0x7b77, 0x08},
	{0x7b78, 0x06},
	{0x7b79, 0x34},
	{0x7b53, 0x00},
	{0x9369, 0x73},
	{0x936b, 0x64},
	{0x936d, 0x5f},
	{0x9304, 0x03},
	{0x9305, 0x80},
	{0x9e9a, 0x2f},
	{0x9e9b, 0x2f},
	{0x9e9c, 0x2f},
	{0x9e9d, 0x00},
	{0x9e9e, 0x00},
	{0x9e9f, 0x00},
	{0xa2a9, 0x27},
	{0xa2b7, 0x03},
	{0x0401, 0x00},
	{0x0404, 0x00},
	{0x0405, 0x10},
	{0x0408, 0x01},
	{0x0409, 0x5c},
	{0x040a, 0x00},
	{0x040b, 0x00},
	{0x040c, 0x05},
	{0x040d, 0x34},
	{0x040e, 0x03},
	{0x040f, 0xde},
	{0x034c, 0x05},
	{0x034d, 0x34},
	{0x034e, 0x03},
	{0x034f, 0xde},
	{0x0301, 0x05},
	{0x0303, 0x02},
	{0x0305, 0x02},
	{0x0306, 0x00},
	{0x0307, 0xaf},
	{0x0309, 0x0a},
	{0x030b, 0x02},
	{0x030d, 0x02},
	{0x030e, 0x00},
	{0x030f, 0x96},
	{0x0310, 0x01},
	{0x0820, 0x07},
	{0x0821, 0x08},
	{0x0822, 0x00},
	{0x0823, 0x00},
	{0x080a, 0x00},
	{0x080b, 0x7f},
	{0x080c, 0x00},
	{0x080d, 0x4f},
	{0x080e, 0x00},
	{0x080f, 0x77},
	{0x0810, 0x00},
	{0x0811, 0x5f},
	{0x0812, 0x00},
	{0x0813, 0x57},
	{0x0814, 0x00},
	{0x0815, 0x4f},
	{0x0816, 0x01},
	{0x0817, 0x27},
	{0x0818, 0x00},
	{0x0819, 0x3f},
	{0xe04c, 0x00},
	{0xe04d, 0x5f},
	{0xe04e, 0x00},
	{0xe04f, 0x1f},
	{0x3e20, 0x01},
	{0x3e37, 0x00},
	{0x3f50, 0x00},
	{0x3f56, 0x00},
	{0x3f57, 0xbf},
};

/* Configuraciones de modos con tamaño de píxel de 12 bits */ 
static const struct imx477_mode supported_modes_12bit[] = {
	/* Modo 12MPix 10fps */ {
		.width = 4056,                             // Anchura de la imagen
		.height = 3040,                            // Altura de la imagen
		.line_length_pix = 0x5dc0,                 // Longitud de línea en píxeles
		.crop = {                                  // Recorte del sensor (ninguno)
			.left = IMX477_PIXEL_ARRAY_LEFT,    		// Recorte desde la izquierda
			.top = IMX477_PIXEL_ARRAY_TOP,      		// Recorte desde arriba
			.width = 4056,                      		// Anchura del recorte
			.height = 3040,                     		// Altura del recorte
		},
		.timeperframe_min = {                       // Tiempo mínimo por fotograma
			.numerator = 100,
			.denominator = 1000
		},
		.timeperframe_default = {                   // Tiempo por defecto por fotograma
			.numerator = 100,
			.denominator = 1000
		},
		.reg_list = {                               // Lista de registros
			.num_of_regs = ARRAY_SIZE(mode_4056x3040_regs), 	// Número de registros
			.regs = mode_4056x3040_regs,            			// Puntero a los registros
		},
	},
	/* Modo 2x2 binning 40fps */ {
		
		.width = 2028,                             // Anchura de la imagen
		.height = 1520,                            // Altura de la imagen
		.line_length_pix = 0x31c4,                 // Longitud de línea en píxeles
		.crop = {                                  // Recorte del sensor (ninguno)
			.left = IMX477_PIXEL_ARRAY_LEFT,    // Recorte desde la izquierda
			.top = IMX477_PIXEL_ARRAY_TOP,      // Recorte desde arriba
			.width = 4056,                      // Anchura del recorte
			.height = 3040,                     // Altura del recorte
		},
		.timeperframe_min = {                       // Tiempo mínimo por fotograma
			.numerator = 100,
			.denominator = 4000
		},
		.timeperframe_default = {                   // Tiempo por defecto por fotograma
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {                               // Lista de registros
			.num_of_regs = ARRAY_SIZE(mode_2028x1520_regs), // Número de registros
			.regs = mode_2028x1520_regs,            // Puntero a los registros
		},
	},
	/* Modo 1080p 50fps recortado */ {
		.width = 2028,                             // Anchura de la imagen
		.height = 1080,                            // Altura de la imagen
		.line_length_pix = 0x31c4,                 // Longitud de línea en píxeles
		.crop = {                                  // Recorte del sensor 
			.left = IMX477_PIXEL_ARRAY_LEFT,    // Recorte desde la izquierda
			.top = IMX477_PIXEL_ARRAY_TOP + 440,// Recorte desde arriba con un desplazamiento de 440
			.width = 4056,                      // Anchura del recorte
			.height = 2160,                     // Altura del recorte
		},
		.timeperframe_min = {                       // Tiempo mínimo por fotograma
			.numerator = 100,
			.denominator = 5000
		},
		.timeperframe_default = {                   // Tiempo por defecto por fotograma
			.numerator = 100,
			.denominator = 3000
		},
		.reg_list = {                               // Lista de registros
			.num_of_regs = ARRAY_SIZE(mode_2028x1080_regs), // Número de registros
			.regs = mode_2028x1080_regs,            // Puntero a los registros
		},
	}
};

/* Configuraciones de modos con tamaño de píxel de 10 bits */
static const struct imx477_mode supported_modes_10bit[] = {
	/* Modo 120fps, 2x2 binning y recortado */ {
		
		.width = 1332,                             // Anchura de la imagen
		.height = 990,                             // Altura de la imagen
		.line_length_pix = 6664,                   // Longitud de línea en píxeles
		.crop = {                                  // Recorte del sensor
			/*
			 * FIXME: the analog crop rectangle is actually
			 * programmed with a horizontal displacement of 0
			 * pixels, not 4. It gets shrunk after going through
			 * the scaler. Move this information to the compose
			 * rectangle once the driver is expanded to represent
			 * its processing blocks with multiple subdevs.
			 */
			.left = IMX477_PIXEL_ARRAY_LEFT + 696, // Desplazamiento horizontal del recorte
			.top = IMX477_PIXEL_ARRAY_TOP + 528,   // Desplazamiento vertical del recorte
			.width = 2664,                         // Anchura del recorte
			.height = 1980,                        // Altura del recorte
		},
		.timeperframe_min = {                       // Tiempo mínimo por fotograma
			.numerator = 100,
			.denominator = 12000
		},
		.timeperframe_default = {                   // Tiempo por defecto por fotograma
			.numerator = 100,
			.denominator = 12000
		},
		.reg_list = {                               // Lista de registros
			.num_of_regs = ARRAY_SIZE(mode_1332x990_regs), // Número de registros
			.regs = mode_1332x990_regs,            // Puntero a los registros
		}
	}
};

/*
 * Lista de los modos de transferencia compatibles
 * The supported formats.
 * This table MUST contain 4 entries per format, to cover the various flip
 * combinations in the order
 * - no flip
 * - h flip
 * - v flip
 * - h&v flips
 */
static const u32 codes[] = { // Bayer formats
	/* Formatos de transferencia de la información de los pixeles
	https://www.kernel.org/doc/html/v4.9/media/uapi/v4l/subdev-formats.html */
	/* 12-bit modes. */
	MEDIA_BUS_FMT_SRGGB12_1X12, // Rojo, Verde, Verde, Azul RG - GB
	MEDIA_BUS_FMT_SGRBG12_1X12,	// Verde, Rojo, Azul, Verde GR - BG
	MEDIA_BUS_FMT_SGBRG12_1X12, // Verde, Azul, Rojo, Verde GB - RG
	MEDIA_BUS_FMT_SBGGR12_1X12, // Azul, Verde, Verde, Rojo BG - GR
	/* 10-bit modes. */
	MEDIA_BUS_FMT_SRGGB10_1X10, // Rojo, Verde, Verde, Azul RG - GB
	MEDIA_BUS_FMT_SGRBG10_1X10, // Verde, Rojo, Azul, Verde GR - BG
	MEDIA_BUS_FMT_SGBRG10_1X10, // Verde, Azul, Rojo, Verde GB - RG
	MEDIA_BUS_FMT_SBGGR10_1X10, // Azul, Verde, Verde, Rojo BG - GR
};

/* Tipos de patrones de prueba ("Menú") */
static const char * const imx477_test_pattern_menu[] = {
	"Disabled",         // Deshabilitado
	"Color Bars",       // Barras de color
	"Solid Color",      // Color sólido
	"Grey Color Bars",  // Barras de color gris
	"PN9"               // Patrón PN9 (pseudoaleatorio)
};

/* Tipos de patrones de prueba */
static const int imx477_test_pattern_val[] = {
	IMX477_TEST_PATTERN_DISABLE,     // Deshabilitado
	IMX477_TEST_PATTERN_COLOR_BARS,  // Barras de color
	IMX477_TEST_PATTERN_SOLID_COLOR, // Color sólido
	IMX477_TEST_PATTERN_GREY_COLOR,  // Escala de grises
	IMX477_TEST_PATTERN_PN9          // Patrón PN9 (pseudoaleatorio)
};

/* Fuentes del regulador */
static const char * const imx477_supply_name[] = {
	/* Supplies can be enabled in any order: pueden ser habilitadas en cualquier orden  */
	"VANA",  /* Analog (2.8V) supply : Fuente analógica (2.8V) */
	"VDIG",  /* Digital Core (1.05V) supply : Fuente núcleo digital (1.05V) */
	"VDDL",  /* IF(Intemidiate Frecuency) (1.8V) supply : Fuente de frecuencia intermedia (1.8V) */
}; // TODO: Repasar

/* Número de diferentes reguladores (3) */
#define IMX477_NUM_SUPPLIES ARRAY_SIZE(imx477_supply_name)

/*
 * Initialisation delay between XCLR low->high and the moment when the sensor
 * can start capture (i.e. can leave software standby), given by T7 in the
 * datasheet is 8ms.  This does include I2C setup time as well.
 *
 * Note, that delay between XCLR low->high and reading the CCI ID register (T6
 * in the datasheet) is much smaller - 600us.
 */

#define IMX477_XCLR_MIN_DELAY_US	8000 // Retardo mínimo en microsegundos antes de iniciar la inicialización
#define IMX477_XCLR_DELAY_RANGE_US	1000 // Rango del retardo para ajustarlo a niveles aceptables

// Estructura de datos usada para establecer chips compatibles y sus registros extra
struct imx477_compatible_data {
	unsigned int chip_id;
	struct imx477_reg_list extra_regs;
};

/* Estructura principal del sensor IMX477 */
struct imx477 {
	struct v4l2_subdev sd; 				// Interfaz para acceso al subdispositivo V4L2
	struct media_pad pad[NUM_PADS]; 	// Entidades para modelar los pads del hardware

	unsigned int fmt_code; 				// Modo de funcionamiento del bus de media y transmisión de pixels

	struct clk *xclk; 					// Controlador del reloj
	u32 xclk_freq;						// Frecuencia del reloj

	struct gpio_desc *reset_gpio; 		// Descriptor GPIO para reset
	struct regulator_bulk_data supplies[IMX477_NUM_SUPPLIES]; // Reguladores de suministro de energía

	struct v4l2_ctrl_handler ctrl_handler; // Manejador de controles V4L2
	/* V4L2 Controls */
	struct v4l2_ctrl *pixel_rate; 		// Tasa de píxeles
	struct v4l2_ctrl *link_freq;		// Frecuencia de enlace
	struct v4l2_ctrl *exposure; 		// Exposición
	struct v4l2_ctrl *vflip; 			// Volteo vertical
	struct v4l2_ctrl *hflip; 			// Volteo horizontal
	struct v4l2_ctrl *vblank; 			// Vertical blank : Tiempo de inactividad desde el paso de un frame al siguiente (ultima a primera linea)
	struct v4l2_ctrl *hblank; 			// Horizontal blank : Tiempo de inactividad desde la lectura de una linea hasta la lectura de la siguiente

	/* Current mode : Modo de funcionamiento actual de la camara */
	const struct imx477_mode *mode; 	

	/* Trigger mode : Modo de disparo (VSYNC) */
	int trigger_mode_of;  // Modo de operación del disparador de VSYNC (0, 1, 2)

	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex; 

	/* Streaming on/off : Indica si se esta en modo streaming o standby*/
	bool streaming; 

	/* Rewrite common registers on stream on? : Reescribir registros comunes al activar modo stream */
	bool common_regs_written;

	/* Current long exposure factor in use. Set through V4L2_CID_VBLANK ;  Factor de exposición larga actual en uso */
	unsigned int long_exp_shift; 

	/* Any extra information related to different compatible sensors : Información extra de registros compatibles */
	const struct imx477_compatible_data *compatible_data; //Informacion de sensores adicionales compatibles
};

// Transforma la structura _sd a una estructura imx477
static inline struct imx477 *to_imx477(struct v4l2_subdev *_sd) {
	return container_of(_sd, struct imx477, sd);
}

/**
 * @brief Obtiene la tabla de modos de operación según el formato de bits especificado.
 *
 * Esta función asigna la tabla de modos de operación y su número correspondiente
 * dependiendo del código de formato de bits proporcionado (10 o 12 bits).
 *
 * @param code Código de formato de bits de media.
 * @param mode_list Puntero a una variable donde se almacenará la tabla de modos de operación.
 * @param num_modes Puntero a una variable donde se almacenará el número de modos disponibles.
 */
static inline void get_mode_table(unsigned int code,
				  const struct imx477_mode **mode_list,
				  unsigned int *num_modes) {
	switch (code) {
	/* 12-bit */
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		*mode_list = supported_modes_12bit;				// Tabla de modos de 12 bits
		*num_modes = ARRAY_SIZE(supported_modes_12bit);	// Número de modos de 12 bits
		break;
	/* 10-bit */
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
		*mode_list = supported_modes_10bit;   			// Tabla de modos de 10 bits
		*num_modes = ARRAY_SIZE(supported_modes_10bit);	// Número de modos de 10 bits
		break;
	default: // Asigna una lista de modos nula por defecto
		*mode_list = NULL;
		*num_modes = 0;
	}
}

/* Read registers up to 2 at a time */
/**
 * @brief Lee un registro desde el sensor IMX477 a través de I2C.
 *
 * Esta función envía una solicitud de lectura al dispositivo IMX477 a través del bus I2C
 * para leer un registro específico y devuelve el valor leído en formato big-endian.
 *
 * @param imx477 Estructura que contiene los datos y configuraciones del sensor IMX477.
 * @param reg Dirección del registro que se desea leer.
 * @param len Longitud de datos a leer (en bytes, máximo 4 bytes).
 * @param val Puntero donde se almacenará el valor leído desde el registro.
 * @return 0 si la lectura fue exitosa, o un código de error negativo en caso de fallo.
 */
static int imx477_read_reg(struct imx477 *imx477, u16 reg, u32 len, u32 *val) {
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);  // Obtiene el cliente I2C desde la estructura V4L2
	struct i2c_msg msgs[2]; // Estructura para los mensajes I2C (escritura y lectura)
	u8 addr_buf[2] = { reg >> 8, reg & 0xff };  // Buffer para la dirección del registro dividida en alta y baja
	u8 data_buf[4] = { 0, }; // Buffer para almacenar los datos leídos
	int ret; // Variable para almacenar el valor de retorno

	if (len > 4) // Verifica que la longitud no sea mayor a 4 bytes
		return -EINVAL;


	// Configura el mensaje para escribir la dirección del registro
	/* Write register address */
	msgs[0].addr = client->addr;	// Dirección del dispositivo I2C
	msgs[0].flags = 0;				// Operación de escritura 
	msgs[0].len = ARRAY_SIZE(addr_buf);	// Longitud del mensaje (2 bytes para la dirección)
	msgs[0].buf = addr_buf;			// Buffer con la dirección del registro a escribir

	// Configura el mensaje para leer los datos desde el registro
	/* Read data from register */
	msgs[1].addr = client->addr;	// Dirección del dispositivo I2C
	msgs[1].flags = I2C_M_RD;		// Operación de lectura
	msgs[1].len = len;				// Longitud del buffer de lectura de datos		
	msgs[1].buf = &data_buf[4 - len];  // Buffer para almacenar los datos leídos

	// Realiza la transferencia I2C y espera la respuesta del dispositivo
	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	// Si la devolucion no es del tamaño del buffer de los mensajes enviados, devolvemos error de IO
	if (ret != ARRAY_SIZE(msgs)) // Verifica si la transferencia fue exitosa
		return -EIO; // Error de IO
	// Convierte los datos leídos (big-endian 4 bytes) y los guarda
	*val = get_unaligned_be32(data_buf);
	
	return 0;
}

/* Write registers up to 2 at a time */
/**
 * @brief Escribe un valor en un registro del sensor IMX477 a través de I2C.
 *
 * Esta función envía un valor al registro especificado del dispositivo IMX477
 * a través del bus I2C. Permite escribir hasta 4 bytes de datos.
 *
 * @param imx477 Estructura que contiene los datos y configuraciones del sensor IMX477.
 * @param reg Dirección del registro al que se desea escribir.
 * @param len Longitud de datos a escribir (en bytes, máximo 4 bytes).
 * @param val Valor a escribir en el registro.
 * @return 0 si la escritura fue exitosa, o un código de error negativo en caso de fallo.
 */
static int imx477_write_reg(struct imx477 *imx477, u16 reg, u32 len, u32 val) {
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd); // Obtiene el cliente I2C desde la estructura V4L2
	u8 buf[6]; // Buffer para almacenar la dirección del registro y los datos a escribir

	if (len > 4) // Verifica que la longitud no sea mayor a 4 bytes
		return -EINVAL; // Error valor invalido

	// Coloca la dirección del registro en el buffer en formato big-endian (16 bits)
	put_unaligned_be16(reg, buf);
	// Coloca el valor a escribir en el buffer, ajustando según la longitud especificada (32 bits)
	put_unaligned_be32(val << (8 * (4 - len)), buf + 2);

	// Envía el buffer al dispositivo I2C y verifica si la longitud del mensaje enviado es correcta
	if (i2c_master_send(client, buf, len + 2) != len + 2) 
		return -EIO; // Error Input Output
	
	return 0;
}

/* Anotacion de funcionamiento :
	- Lectura : Para una lectura el dispositivo realiza dos operaciones como envio de mensaje, 
	  primero enviar el registro en modo escritura, marcando así el registro que se quiere leer,
	  y luego envía otro mensaje que guarda el resultado en el buffer pasado, este luego es colocado
	  en la variable con el valor
	- Escritura : Para la escritura se envia un buffer con solo la direccion del registro
	y el dato que se quiere escribir,  el dispositivo cambia el valor, y la operacion devuelve la
	longitud de los datos como valor de comprobacion.

*/

/* Write a list of registers */
/**
 * @brief Escribe una lista de registros en el sensor IMX477 a través de I2C.
 *
 * Esta función recorre una lista de registros y escribe cada uno de ellos.
 *
 * @param imx477 Estructura que contiene los datos y configuraciones del sensor IMX477.
 * @param regs Puntero a una lista de estructuras `imx477_reg` que contiene
 *             los registros y sus valores correspondientes.
 * @param len Longitud de la lista de registros (número de elementos en el array).
 * @return 0 si todas las escrituras fueron exitosas, o un código de error negativo en caso de fallo.
 */
static int imx477_write_regs(struct imx477 *imx477,
			     const struct imx477_reg *regs, u32 len) {
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd); // Obtiene el cliente I2C desde la estructura V4L2
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		// Escribe el valor en el registro especificado; la longitud es siempre de un único byte
		ret = imx477_write_reg(imx477, regs[i].address, 1, regs[i].val);
		// Si la escritura falla (retorna un valor distinto de 0)
		if (ret) {
			// Imprime un mensaje de error, con una tasa limitada para evitar saturar el log
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    regs[i].address, ret);
			return ret; // Retorna el código de error y sale de la función
		}
	}

	return 0; // Retorna 0 para indicar que todas las escrituras fueron exitosas
}

/* Get bayer order based on flip setting. */

/**
 * @brief Obtiene el código de formato Bayer basado en la configuración de flip.
 *
 * Esta función determina el orden Bayer correcto según las configuraciones de
 * rotación vertical (vflip) y horizontal (hflip) del sensor IMX477.
 *
 * @param imx477 Estructura que contiene los datos y configuraciones del sensor IMX477.
 * @param code Código de formato original a buscar en la lista de códigos soportados.
 * @return El código de formato Bayer modificado según las configuraciones de flip.
 */
static u32 imx477_get_format_code(struct imx477 *imx477, u32 code) {
	unsigned int i;
	// Asegura que se tiene el mutex
	lockdep_assert_held(&imx477->mutex);

	// Busca el código pasado en la lista de códigos soportados
	for (i = 0; i < ARRAY_SIZE(codes); i++)
		if (codes[i] == code)
			break;
	
	// Si no se encuentra el código, se usa el primer código de la lista
	if (i >= ARRAY_SIZE(codes))
		i = 0;

	// Modifica el índice para reflejar las configuraciones de vflip y hflip
	// Borra los dos últimos bits del índice y ajusta según los valores de vflip y hflip (bit0 a 1 hflip, bit1 a 1 vflip)
	i = (i & ~3) | (imx477->vflip->val ? 2 : 0) |
	    (imx477->hflip->val ? 1 : 0);

	// Devuelve el código de formato correspondiente al índice modificado
	return codes[i];
}


/**
 * @brief Establece el formato por defecto de funcionamiento de la cámara IMX477.
 *
 * Esta función configura el modo y el formato del bus de la cámara IMX477 a su 
 * configuración por defecto.
 *
 * @param imx477 Estructura que contiene los datos y configuraciones del sensor IMX477.
 */
static void imx477_set_default_format(struct imx477 *imx477) {
	/* Establece el modo por defecto a la resolución máxima */
	imx477->mode = &supported_modes_12bit[0];  		// Modo de funcionamiento por defecto
	imx477->fmt_code = MEDIA_BUS_FMT_SRGGB12_1X12;	// Modo de funcionamiento del bus correspondiente
}


/**
 * @brief Abre la cámara y la prepara para su correcto funcionamiento.
 *
 * @param sd Interfaz para controlar el dispositivo.
 * @param fh Manejador de archivos para el subdispositivo.
 * @return 0 si la operación fue exitosa, un código de error en caso contrario.
 */
static int imx477_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh) {
	struct imx477 *imx477 = to_imx477(sd); // Obtiene la estructura específica de la cámara
	struct v4l2_mbus_framefmt *try_fmt_img =
		v4l2_subdev_get_try_format(sd, fh->state, IMAGE_PAD); // Obtiene el formato del pad de imagen
	struct v4l2_mbus_framefmt *try_fmt_meta =
		v4l2_subdev_get_try_format(sd, fh->state, METADATA_PAD);  // Obtiene el formato del pad de metadatos
	struct v4l2_rect *try_crop;

	mutex_lock(&imx477->mutex); // Obtiene el lock

	/* Initialize try_fmt for the image pad */
	/* Inicializa try_fmt para el pad de imagen */
	try_fmt_img->width = supported_modes_12bit[0].width;	// Anchura
	try_fmt_img->height = supported_modes_12bit[0].height;	// Altura
	try_fmt_img->code = imx477_get_format_code(imx477,
						   MEDIA_BUS_FMT_SRGGB12_1X12);		// Establece el codigo de formato
	try_fmt_img->field = V4L2_FIELD_NONE; // Imágenes en formato progresivo

	/* Initialize try_fmt for the embedded metadata pad */
	/* Inicializa try_fmt para el pad de metadatos */
	try_fmt_meta->width = IMX477_EMBEDDED_LINE_WIDTH;	// Tamaño de linea
	try_fmt_meta->height = IMX477_NUM_EMBEDDED_LINES; 	// Número de lineas
	try_fmt_meta->code = MEDIA_BUS_FMT_SENSOR_DATA;		// Datos RAW del sensor
	try_fmt_meta->field = V4L2_FIELD_NONE; // Imágenes en formato progresivo

	/* Initialize try_crop */
	/* Inicializa try_crop */
	try_crop = v4l2_subdev_get_try_crop(sd, fh->state, IMAGE_PAD);	// Obtiene el crop de la imagen
	// Establecemos los valores del crop
	try_crop->left = IMX477_PIXEL_ARRAY_LEFT;			
	try_crop->top = IMX477_PIXEL_ARRAY_TOP;
	try_crop->width = IMX477_PIXEL_ARRAY_WIDTH;
	try_crop->height = IMX477_PIXEL_ARRAY_HEIGHT;

	mutex_unlock(&imx477->mutex); // Libera el lock

	return 0;
}

/**
 * @brief Ajusta los rangos de exposición de la cámara.
 *
 * Esta función ajusta los valores de exposición de acuerdo a los límites de VBLANK.
 *
 * @param imx477 Estructura que contiene los datos y configuraciones del sensor IMX477.
 */
static void imx477_adjust_exposure_range(struct imx477 *imx477) {
	int exposure_max, exposure_def;

	/* Honour the VBLANK limits when setting exposure. */
	/* Respeta los límites máximos de VBLANK al ajustar la exposición. */
	exposure_max = imx477->mode->height + imx477->vblank->val -
		       IMX477_EXPOSURE_OFFSET;
	
	// Se define la exposición como el mínimo del valor de la exposición y el máximo
	exposure_def = min(exposure_max, imx477->exposure->val);
	// Modifica la exposición que se le pasa, según el mínimo, máximo, el step, y la exposición por defecto
	__v4l2_ctrl_modify_range(imx477->exposure, imx477->exposure->minimum,
				 exposure_max, imx477->exposure->step,
				 exposure_def);
}

/**
 * @brief Establece la longitud de un frame.
 *
 * Ajusta la longitud del frame y el factor de exposición larga si es necesario.
 *
 * @param imx477 Estructura que contiene los datos y configuraciones del sensor IMX477.
 * @param val Nuevo valor de la longitud del frame.
 * @return 0 si la operación fue exitosa, un código de error en caso contrario.
 */
static int imx477_set_frame_length(struct imx477 *imx477, unsigned int val) {
	int ret = 0;

	imx477->long_exp_shift = 0; // Establecemos la variación de la exposición larga a cero

	// Mientras el valor sea mayor que el máximo permitido
	while (val > IMX477_FRAME_LENGTH_MAX) {
		imx477->long_exp_shift++; // Aumentamos la exposición en uno
		val >>= 1; // Divide el número entre dos
	}

	// Guardamos el valor de la longitud del frame
	ret = imx477_write_reg(imx477, IMX477_REG_FRAME_LENGTH,
			       IMX477_REG_VALUE_16BIT, val);
	if (ret)
		return ret;
	// Se registra el valor de la variación en la exposición larga
	return imx477_write_reg(imx477, IMX477_LONG_EXP_SHIFT_REG,
				IMX477_REG_VALUE_08BIT, imx477->long_exp_shift);
}

/**
 * @brief Establece un control V4L2 para la cámara IMX477.
 *
 * Se encarga de aplicar el valor del control V4L2 correspondiente al sensor IMX477.
 *
 * @param ctrl Puntero al control V4L2 que se va a establecer.
 * @return 0 si la operación fue exitosa, un código de error en caso contrario.
 */
static int imx477_set_ctrl(struct v4l2_ctrl *ctrl) {
	// Obtiene la estructura imx477 a partir del control handler
	struct imx477 *imx477 =
		container_of(ctrl->handler, struct imx477, ctrl_handler);
	// Obtiene el cliente I2C asociado al subdispositivo
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	int ret = 0;

	/*
	 * The VBLANK (Vertical blancking) control may change the limits of usable exposure, so check
	 * and adjust if necessary. 
	 */ 
	if (ctrl->id == V4L2_CID_VBLANK)
		imx477_adjust_exposure_range(imx477); // Ajustamos los rangos de la exposición

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	 // Vuelve si el dispositivo no esta en uso en este momento
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

    // Según el ID pasado por el control, se escribe un registro en la cámara
	switch (ctrl->id) {
    // Ganancia analógica que afecta a todos los componentes de color en la matriz de píxeles
	case V4L2_CID_ANALOGUE_GAIN:
		ret = imx477_write_reg(imx477, IMX477_REG_ANALOG_GAIN,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
    // Establecer la exposición
	case V4L2_CID_EXPOSURE:
		ret = imx477_write_reg(imx477, IMX477_REG_EXPOSURE,
				       IMX477_REG_VALUE_16BIT, ctrl->val >>
							imx477->long_exp_shift);
		break;
    // Ganancia digital que multiplica todos los colores
	case V4L2_CID_DIGITAL_GAIN:
		ret = imx477_write_reg(imx477, IMX477_REG_DIGITAL_GAIN,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
    // Establecer el patrón de imagen de prueba
	case V4L2_CID_TEST_PATTERN:
		ret = imx477_write_reg(imx477, IMX477_REG_TEST_PATTERN,
				       IMX477_REG_VALUE_16BIT,
				       imx477_test_pattern_val[ctrl->val]);
		break;
    // Establecer patrón de prueba rojo
	case V4L2_CID_TEST_PATTERN_RED:
		ret = imx477_write_reg(imx477, IMX477_REG_TEST_PATTERN_R,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
    // Establecer patrón de prueba verde (rojo)
	case V4L2_CID_TEST_PATTERN_GREENR:
		ret = imx477_write_reg(imx477, IMX477_REG_TEST_PATTERN_GR,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
    // Establecer patrón de prueba azul
	case V4L2_CID_TEST_PATTERN_BLUE:
		ret = imx477_write_reg(imx477, IMX477_REG_TEST_PATTERN_B,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
    // Establecer patrón de prueba verde (azul)
	case V4L2_CID_TEST_PATTERN_GREENB:
		ret = imx477_write_reg(imx477, IMX477_REG_TEST_PATTERN_GB,
				       IMX477_REG_VALUE_16BIT, ctrl->val);
		break;
    // Establecer la inversión horizontal o vertical
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		ret = imx477_write_reg(imx477, IMX477_REG_ORIENTATION, 1,
				       imx477->hflip->val |
				       imx477->vflip->val << 1);
		break;
    // Establecer la longitud de un frame (en líneas)
	case V4L2_CID_VBLANK:
		ret = imx477_set_frame_length(imx477,
					      imx477->mode->height + ctrl->val);
		break;
    // Establecer la longitud de una línea de un frame
	case V4L2_CID_HBLANK:
		ret = imx477_write_reg(imx477, IMX477_REG_LINE_LENGTH, 2,
				       imx477->mode->width + ctrl->val);
		break;
	default:
        // Registrar un mensaje en el log indicando que el ID de control no está manejado
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL; // Devolver error de valor inválido
		break;
	}
    // Decrementar el contador de uso del dispositivo
	pm_runtime_put(&client->dev);

	return ret;
}

// Definición de operaciones de control para el driver IMX477
static const struct v4l2_ctrl_ops imx477_ctrl_ops = {
	.s_ctrl = imx477_set_ctrl, // Función para aplicar el valor del control de V4L2
};

/**
 * Obtiene el código del formato del media bus correspondiente al número de pad indicado.
 *
 * @param sd Puntero al subdispositivo V4L2.
 * @param sd_state Estado del subdispositivo V4L2 (no utilizado en esta función).
 * @param code Estructura que contiene el número de pad y el índice del formato a enumerar.
 * @return 0 si la operación fue exitosa, un código de error en caso contrario.
 */
static int imx477_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code) {
	// Transforma la estructura genérica del subdispositivo a la estructura específica de imx477
	struct imx477 *imx477 = to_imx477(sd);

	// Verificar que el número de pads sea válido
	if (code->pad >= NUM_PADS)
		return -EINVAL;
	
	
	if (code->pad == IMAGE_PAD) {
		// Verificar que el índice no sobrepase el tamaño del array de códigos
		if (code->index >= (ARRAY_SIZE(codes) / 4))
			return -EINVAL;
		// Obtener el código de formato usando el índice proporcionado
		code->code = imx477_get_format_code(imx477,
						    codes[code->index * 4]);
	} else {
		if (code->index > 0)
			return -EINVAL;
		// Si el tipo de pad no es de imagen, es de Datos de sensor RAW
		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
	}

	return 0;
}


/**
 * Enumera el tamaño de los frames según el pad y el formato de media bus especificados.
 *
 * @param sd Puntero al subdispositivo V4L2.
 * @param sd_state Estado del subdispositivo V4L2 (no utilizado en esta función).
 * @param fse Estructura que contiene información sobre el pad, código de formato y tamaño de frame.
 * @return 0 si la operación fue exitosa, un código de error en caso contrario.
 */
static int imx477_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse) {
	// Transformar la estructura genérica del subdispositivo a la estructura específica de imx477
	struct imx477 *imx477 = to_imx477(sd);

	// Verificar que el número de pad sea válido
	if (fse->pad >= NUM_PADS)
		return -EINVAL;
	
	if (fse->pad == IMAGE_PAD) { // Para pads de imagen
		const struct imx477_mode *mode_list;
		unsigned int num_modes;
		// Obtener la tabla de modos y el número de modos según el código de formato
		get_mode_table(fse->code, &mode_list, &num_modes);

		// Verificar que el índice no sobrepase el número de modos disponibles
		if (fse->index >= num_modes)
			return -EINVAL;
		
		// Verificar que el código de formato sea correcto para el modo solicitado
		if (fse->code != imx477_get_format_code(imx477, fse->code))
			return -EINVAL;

		// Establecer los tamaños mínimo y máximo de ancho y alto según el modo de la lista
		fse->min_width = mode_list[fse->index].width;
		fse->max_width = fse->min_width;
		fse->min_height = mode_list[fse->index].height;
		fse->max_height = fse->min_height;
	} else {
		// Verificar que el código de formato sea MEDIA_BUS_FMT_SENSOR_DATA y que el índice sea válido
		if (fse->code != MEDIA_BUS_FMT_SENSOR_DATA || fse->index > 0)
			return -EINVAL;
		// Establecer el tamaño mínimo y máximo de ancho y alto para datos  del sensor
		fse->min_width = IMX477_EMBEDDED_LINE_WIDTH;
		fse->max_width = fse->min_width;
		fse->min_height = IMX477_NUM_EMBEDDED_LINES;
		fse->max_height = fse->min_height;
	}

	return 0;
}

/**
 * Resetea el espacio de color del formato de marco de media bus especificado.
 *
 * @param fmt Puntero al formato de marco de media bus a resetear.
 */
static void imx477_reset_colorspace(struct v4l2_mbus_framefmt *fmt) {
	// Establecer el espacio de color como RAW por defecto, mínimo procesamiento
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	// Establecer la codificación YCbCr por defecto basada en el espacio de color
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	// Establecer la cuantificación por defecto basada en el espacio de color y codificación YCbCr
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	// Establecer la función de transferencia por defecto basada en el espacio de color
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

/**
 * Actualiza el formato del pad de imagen según los parámetros indicados en la estructura mode.
 *
 * @param imx477 Puntero a la estructura de datos del sensor IMX477.
 * @param mode Puntero a la estructura que contiene el modo de operación deseado.
 * @param fmt Puntero al formato de subdispositivo V4L2 a actualizar.
 */
static void imx477_update_image_pad_format(struct imx477 *imx477,
					   const struct imx477_mode *mode,
					   struct v4l2_subdev_format *fmt) {
	// Actualizar el tamaño del formato con el ancho y alto del modo proporcionado
	fmt->format.width = mode->width;   
	fmt->format.height = mode->height;
	// Establecer el tipo de campo como escaneo progresivo
	fmt->format.field = V4L2_FIELD_NONE;
	// Resetear el espacio de color del formato
	imx477_reset_colorspace(&fmt->format);
}

/**
 * Actualiza el formato de los ajustes del dispositivo al formato original del pad de metadatos.
 *
 * @param fmt Puntero al formato de subdispositivo V4L2 a actualizar.
 */static void imx477_update_metadata_pad_format(struct v4l2_subdev_format *fmt) {
	// Establecer el ancho y alto del formato como el tamaño de línea embebido y número de líneas embebidas
	fmt->format.width = IMX477_EMBEDDED_LINE_WIDTH;
	fmt->format.height = IMX477_NUM_EMBEDDED_LINES;
	// Establecer el código de formato como MEDIA_BUS_FMT_SENSOR_DATA
	fmt->format.code = MEDIA_BUS_FMT_SENSOR_DATA;
	// Establecer el tipo de campo como escaneo progresivo
	fmt->format.field = V4L2_FIELD_NONE;
}

/**
 * Obtiene el formato actual del pad especificado.
 *
 * @param sd Puntero al subdispositivo V4L2.
 * @param sd_state Estado del subdispositivo V4L2 (no utilizado en esta función).
 * @param fmt Puntero al formato de subdispositivo V4L2 donde se almacenará el formato obtenido.
 * @return 0 si la operación fue exitosa, un código de error en caso contrario.
 */
static int imx477_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt) {
	// Transformar la estructura genérica del subdispositivo a la estructura específica de imx477
	struct imx477 *imx477 = to_imx477(sd);
	
	// Verificar que el número de pad sea válido
	if (fmt->pad >= NUM_PADS)
		return -EINVAL;
	
	// Obtener el lock para acceso exclusivo
	mutex_lock(&imx477->mutex);

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		// Obtener el formato TRY del subdispositivo
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(&imx477->sd, sd_state,
						   fmt->pad);
		/* update the code which could change due to vflip or hflip: */
		// Actualizar el código que podría cambiar debido a vflip o hflip:
		try_fmt->code = fmt->pad == IMAGE_PAD ?
				imx477_get_format_code(imx477, try_fmt->code) :
				MEDIA_BUS_FMT_SENSOR_DATA;
		// Copiar el formato TRY actualizado a la estructura de formato solicitada
		fmt->format = *try_fmt;
	} else {
		// Obtener el formato actual del pad de imagen o de metadatos
		if (fmt->pad == IMAGE_PAD) {
			// Actualizar el formato del pad de imagen
			imx477_update_image_pad_format(imx477, imx477->mode,
						       fmt);
			// Obtener el código de formato actualizado para el pad de imagen
			fmt->format.code =
			       imx477_get_format_code(imx477, imx477->fmt_code);
		} else {
			// Actualizar el formato del pad de metadatos
			imx477_update_metadata_pad_format(fmt);
		}
	}

	// Liberamos el lock
	mutex_unlock(&imx477->mutex);
	return 0;
}

/**
 * Calcula el tamaño del frame según el modo y el tiempo por frame especificados.
 *
 * @param mode Puntero al modo de operación del sensor IMX477.
 * @param timeperframe Puntero a la estructura que especifica el tiempo por frame (fracción de tiempo).
 * @return Tamaño del frame calculado en unidades específicas.
 */
static unsigned int imx477_get_frame_length(const struct imx477_mode *mode,
				     const struct v4l2_fract *timeperframe) {
	u64 frame_length;
	// Calcular la longitud del frame como el producto del numerador del tiempo por frame y la tasa de píxeles IMX477
	frame_length = (u64)timeperframe->numerator * IMX477_PIXEL_RATE;
	// Dividir la longitud del frame por el denominador del tiempo por frame y la longitud en píxeles por línea del modo
	do_div(frame_length,
	       (u64)timeperframe->denominator * mode->line_length_pix);

	// Verificar si la longitud del frame excede el máximo permitido; si es así, generar una advertencia
	if (WARN_ON(frame_length > IMX477_FRAME_LENGTH_MAX))
		frame_length = IMX477_FRAME_LENGTH_MAX;
	
	// Retornar el máximo entre la longitud calculada del frame y la altura máxima especificada en el modo
	return max_t(unsigned int, frame_length, mode->height);
}

/**
 * Define los límites de framing (encuadre) para el sensor IMX477.
 *
 * @param imx477 Puntero a la estructura de datos del sensor IMX477.
 */
static void imx477_set_framing_limits(struct imx477 *imx477) {
	unsigned int frm_length_min, frm_length_default, hblank_min;
	const struct imx477_mode *mode = imx477->mode;

	// Calcular el tamaño mínimo y por defecto del frame para el modo actual
	frm_length_min = imx477_get_frame_length(mode, &mode->timeperframe_min);
	frm_length_default =
		     imx477_get_frame_length(mode, &mode->timeperframe_default);

	/* Default to no long exposure multiplier. */
	// Restablecer el multiplicador de exposición larga a cero
	imx477->long_exp_shift = 0;

	/* Update limits and set FPS to default */
	// Actualizar los límites de VBlank y establecer FPS a valor por defecto
	__v4l2_ctrl_modify_range(imx477->vblank, frm_length_min - mode->height,
				 ((1 << IMX477_LONG_EXP_SHIFT_MAX) *
					IMX477_FRAME_LENGTH_MAX) - mode->height,
				 1, frm_length_default - mode->height);

	/* Setting this will adjust the exposure limits as well. */
	__v4l2_ctrl_s_ctrl(imx477->vblank, frm_length_default - mode->height);
	
	// Calcular el tamaño mínimo de HBlank
	hblank_min = mode->line_length_pix - mode->width;
	__v4l2_ctrl_modify_range(imx477->hblank, hblank_min,
				 IMX477_LINE_LENGTH_MAX, 1, hblank_min);
	__v4l2_ctrl_s_ctrl(imx477->hblank, hblank_min);
}

/**
 * Establece el formato del PAD especificado en la estructura `fmt`.
 *
 * @param sd Puntero al subdispositivo V4L2.
 * @param sd_state Puntero al estado del subdispositivo V4L2.
 * @param fmt Puntero a la estructura que contiene el formato a establecer.
 * @return 0 si la operación fue exitosa, un código de error en caso contrario.
 */
static int imx477_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt) {
	struct v4l2_mbus_framefmt *framefmt;
	const struct imx477_mode *mode;
	struct imx477 *imx477 = to_imx477(sd);

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;
	
	// Obtenemos el lock para evitar condiciones de carrera
	mutex_lock(&imx477->mutex);

	// Para el pad de imagen
	if (fmt->pad == IMAGE_PAD) {
		const struct imx477_mode *mode_list;
		unsigned int num_modes;

		/* Bayer order varies with flips */
		// Obtener el código de formato correcto
		fmt->format.code = imx477_get_format_code(imx477,
							  fmt->format.code);
		// Obtener la tabla de modos para el código de formato
		get_mode_table(fmt->format.code, &mode_list, &num_modes);

		// Encontrar el modo más cercano según el ancho y alto especificados
		mode = v4l2_find_nearest_size(mode_list,
					      num_modes,
					      width, height,
					      fmt->format.width,
					      fmt->format.height);
		// Actualizar el formato del PAD de imagen
		imx477_update_image_pad_format(imx477, mode, fmt);
		// Manejar el formato TRY
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else if (imx477->mode != mode) {
			// Si el modo ha cambiado, actualizar el modo y el código de formato
			imx477->mode = mode;
			imx477->fmt_code = fmt->format.code;
			// Actualizar los límites de framing
			imx477_set_framing_limits(imx477);
		}
	} else {
		// Para el pad de metadatos
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else {
			/* Only one embedded data mode is supported */
			// Actualizar el formato del pad de metadatos
			imx477_update_metadata_pad_format(fmt);
		}
	}
	// Liberar el lock
	mutex_unlock(&imx477->mutex);

	return 0;
}

/**
 * Obtiene el rectángulo de recorte (crop) del PAD especificado del sensor IMX477.
 *
 * @param imx477 Puntero a la estructura de datos del sensor IMX477.
 * @param sd_state Puntero al estado del subdispositivo V4L2.
 * @param pad Número de PAD del cual se desea obtener el recorte.
 * @param which Tipo de formato V4L2 para determinar si es intento (TRY) o activo (ACTIVE).
 * @return Puntero al rectángulo de recorte correspondiente al PAD y tipo de formato especificados.
 */
static const struct v4l2_rect * __imx477_get_pad_crop(struct imx477 *imx477,
		      struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which) {
	switch (which) {
	
	case V4L2_SUBDEV_FORMAT_TRY:
		// Devolver el rectángulo de recorte de TRY
		return v4l2_subdev_get_try_crop(&imx477->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		// Devolver el rectángulo de recorte activo si está disponible
		return &imx477->mode->crop;
	}

	return NULL;
}

/**
 * Obtiene la selección especificada para el subdispositivo V4L2.
 *
 * @param sd Puntero al subdispositivo V4L2.
 * @param sd_state Puntero al estado del subdispositivo V4L2.
 * @param sel Puntero a la estructura de selección V4L2 que se desea obtener.
 * @return 0 si la operación fue exitosa, -EINVAL si ocurre un error.
 */
static int imx477_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel) {
	switch (sel->target) {
	
	// Si se busca el crop
	case V4L2_SEL_TGT_CROP: {
		struct imx477 *imx477 = to_imx477(sd);
		// Obtenemos el lock
		mutex_lock(&imx477->mutex);
		// Obtenemos el rectángulo de recorte del PAD especificado
		sel->r = *__imx477_get_pad_crop(imx477, sd_state, sel->pad,
						sel->which);
		// Liberamos el lock
		mutex_unlock(&imx477->mutex);

		return 0;
	}
	
	// Define el rectángulo para el tamaño nativo del sensor IMX477
	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = IMX477_NATIVE_WIDTH;
		sel->r.height = IMX477_NATIVE_HEIGHT;

		return 0;
	
	// Default y el tamaño del sensor
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		// Define el rectángulo para el recorte por defecto o los límites del sensor IMX477
		sel->r.left = IMX477_PIXEL_ARRAY_LEFT;
		sel->r.top = IMX477_PIXEL_ARRAY_TOP;
		sel->r.width = IMX477_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX477_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL; // Devuelve error si el tipo de selección no es válido 
}

/**
 * Inicia el streaming de datos desde el sensor IMX477.
 *
 * @param imx477 Puntero a la estructura que representa el sensor IMX477.
 * @return 0 si la operación fue exitosa, de lo contrario devuelve un código de error.
 */
static int imx477_start_streaming(struct imx477 *imx477) {
	// Obtenemos el cliente I2C asociado al subdispositivo V4L2
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	const struct imx477_reg_list *reg_list;
	const struct imx477_reg_list *extra_regs;
	int ret, tm;
	
	// Si los registros comunes no han sido escritos, los configuramos
	if (!imx477->common_regs_written) {
		// Escribimos todos los registros comunes de configuración
		ret = imx477_write_regs(imx477, mode_common_regs,
					ARRAY_SIZE(mode_common_regs));
		// Si la escritura fue exitosa, configuramos los registros adicionales
		if (!ret) {
			extra_regs = &imx477->compatible_data->extra_regs;
			ret = imx477_write_regs(imx477,	extra_regs->regs,
						extra_regs->num_of_regs);
		}
		// Si hubo un error, registramos el error y devolvemos el resultado
		if (ret) {
			dev_err(&client->dev, "%s failed to set common settings\n",
				__func__);
			return ret;
		}
		// Marcamos que los registros comunes han sido escritos
		imx477->common_regs_written = true;
	}

	/* Apply default values of current mode */
	/* Aplicamos los valores por defecto del modo actual */
	reg_list = &imx477->mode->reg_list;
	ret = imx477_write_regs(imx477, reg_list->regs, reg_list->num_of_regs);
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
		return ret;
	}

	/* Set on-sensor DPC. */
	// Configuramos el DPC (correccion de pixeles con defectos) en el sensor
	imx477_write_reg(imx477, 0x0b05, IMX477_REG_VALUE_08BIT, !!dpc_enable);
	imx477_write_reg(imx477, 0x0b06, IMX477_REG_VALUE_08BIT, !!dpc_enable);

	/* Apply customized values from user */
	// Preparamos el manejador del dispositivo
	ret =  __v4l2_ctrl_handler_setup(imx477->sd.ctrl_handler);
	if (ret)
		return ret;

	/* Set vsync trigger mode: 0=standalone, 1=source, 2=sink */
	// Si el modo de funcionamiento no es standalone lo guardamos, si no indicamos el establecido en trigger_mode
	tm = (imx477->trigger_mode_of >= 0) ? imx477->trigger_mode_of : trigger_mode;
	// Escribe los registros con distintos modos de funcionamiento
	imx477_write_reg(imx477, IMX477_REG_MC_MODE,
			 IMX477_REG_VALUE_08BIT, (tm > 0) ? 1 : 0);
	imx477_write_reg(imx477, IMX477_REG_MS_SEL,
			 IMX477_REG_VALUE_08BIT, (tm <= 1) ? 1 : 0);
	imx477_write_reg(imx477, IMX477_REG_XVS_IO_CTRL,
			 IMX477_REG_VALUE_08BIT, (tm == 1) ? 1 : 0);
	imx477_write_reg(imx477, IMX477_REG_EXTOUT_EN,
			 IMX477_REG_VALUE_08BIT, (tm == 1) ? 1 : 0);

	/* set stream on register */
	// Indicamos el modo de funcionamiento como streaming
	return imx477_write_reg(imx477, IMX477_REG_MODE_SELECT,
				IMX477_REG_VALUE_08BIT, IMX477_MODE_STREAMING);
}

/**
 * Detiene el streaming de datos desde el sensor IMX477.
 *
 * @param imx477 Puntero a la estructura que representa el sensor IMX477.
 */
static void imx477_stop_streaming(struct imx477 *imx477) {
	// Obtenemos el cliente I2C asociado al subdispositivo V4L2
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	int ret;

	/* set stream off register */
	// Configuramos el registro para detener el streaming 
	ret = imx477_write_reg(imx477, IMX477_REG_MODE_SELECT,
			       IMX477_REG_VALUE_08BIT, IMX477_MODE_STANDBY);
	if (ret)
		dev_err(&client->dev, "%s failed to set stream\n", __func__);

	/* Stop driving XVS out (there is still a weak pull-up) */
	/* Detenemos la salida de XVS (aunque queda un pull-up débil) */ 
	imx477_write_reg(imx477, IMX477_REG_EXTOUT_EN,
			 IMX477_REG_VALUE_08BIT, 0);
}

/**
 * Inicia o detiene el streaming de datos desde el sensor IMX477.
 *
 * @param sd Puntero al subdispositivo V4L2.
 * @param enable Flag que indica si se debe iniciar (true) o detener (false) el streaming.
 * @return 0 si se realiza correctamente, o un código de error negativo en caso de fallo.
 */
static int imx477_set_stream(struct v4l2_subdev *sd, int enable) {
	// Obtenemos la estructura específica del sensor IMX477 desde el subdispositivo V4L2
	struct imx477 *imx477 = to_imx477(sd);
	// Obtenemos el cliente I2C asociado al subdispositivo V4L2
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	// Obtenemos el lock para asegurar operaciones atómicas
	mutex_lock(&imx477->mutex);

	// Si el estado de streaming es el mismo que el solicitado, no hacemos nada y retornamos
	if (imx477->streaming == enable) {
		mutex_unlock(&imx477->mutex);
		return 0;
	}

	if (enable) {
		// Iniciamos el runtime power management para el dispositivo
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) { // Si hay un error al iniciar el runtime power management
			// Intentamos poner el dispositivo en estado de espera (idle)
			pm_runtime_put_noidle(&client->dev);
			goto err_unlock;
		}

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		// Llamamos a la función para que prepare los registros, etc y comienze el streaming
		ret = imx477_start_streaming(imx477);
		if (ret)
			goto err_rpm_put;
	} else {
		// Detenemos el streaming de datos
		imx477_stop_streaming(imx477);
		// Liberamos el runtime power management
		pm_runtime_put(&client->dev);
	}

	// Actualizamos el estado de streaming en la estructura del sensor
	imx477->streaming = enable; 

	/* vflip and hflip cannot change during streaming */
	// Bloqueamos los controles de flip vertical y horizontal durante el streaming para evitar cambios
	__v4l2_ctrl_grab(imx477->vflip, enable);
	__v4l2_ctrl_grab(imx477->hflip, enable);
	
	// Liberamos el lock
	mutex_unlock(&imx477->mutex);

	return ret;

err_rpm_put:
	// Liberamos el runtime power management
	pm_runtime_put(&client->dev);
err_unlock:
	// Liberamos el lock
	mutex_unlock(&imx477->mutex);

	return ret;
}

/* Power/clock management functions */
/**
 * Enciende la alimentación y el reloj para el sensor IMX477,
 * preparándolo para la operación normal.
 *
 * @param dev Puntero al dispositivo asociado.
 * @return 0 si se realiza correctamente, o un código de error negativo en caso de fallo.
 */
static int imx477_power_on(struct device *dev) {
	// Obtenemos el cliente I2C a partir del dispositivo
	struct i2c_client *client = to_i2c_client(dev);
	// Obtenemos el subdispositivo V4L2 asociado al cliente I2C
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	// Convertimos el subdispositivo V4L2 en la estructura específica del sensor IMX477
	struct imx477 *imx477 = to_imx477(sd);
	int ret;

	// Habilitamos los reguladores de alimentación necesarios
	ret = regulator_bulk_enable(IMX477_NUM_SUPPLIES,
				    imx477->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}
	// Preparamos y habilitamos el reloj externo (XCLK)
	ret = clk_prepare_enable(imx477->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off; // En caso de error, deshabilitamos los reguladores
	}
	// Configuramos el GPIO de reset para permitir el modo suspendido
	gpiod_set_value_cansleep(imx477->reset_gpio, 1);
	// Esperamos un rango de tiempo para que el dispositivo registre la señal
	usleep_range(IMX477_XCLR_MIN_DELAY_US,
		     IMX477_XCLR_MIN_DELAY_US + IMX477_XCLR_DELAY_RANGE_US);

	return 0;

reg_off:
	// En caso de fallo al habilitar el reloj, deshabilitamos los reguladores previamente habilitados
	regulator_bulk_disable(IMX477_NUM_SUPPLIES, imx477->supplies);
	return ret;
}

/**
 * Apaga el dispositivo IMX477, deshabilitando la alimentación,
 * el reloj y estableciendo el modo de reset a apagado.
 *
 * @param dev Puntero al dispositivo asociado.
 * @return 0 si se realiza correctamente, o un código de error negativo en caso de fallo.
 */
static int imx477_power_off(struct device *dev) {
	// Obtenemos el cliente I2C a partir del dispositivo
	struct i2c_client *client = to_i2c_client(dev);
	// Obtenemos el subdispositivo V4L2 asociado al cliente I2C
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	// Convertimos el subdispositivo V4L2 en la estructura específica del sensor IMX477
	struct imx477 *imx477 = to_imx477(sd);

	// Establecemos el GPIO de reset a apagado
	gpiod_set_value_cansleep(imx477->reset_gpio, 0);
	// Deshabilitamos los reguladores de alimentación
	regulator_bulk_disable(IMX477_NUM_SUPPLIES, imx477->supplies);
	// Deshabilitamos y liberamos el reloj externo (XCLK)
	clk_disable_unprepare(imx477->xclk);

	/* Force reprogramming of the common registers when powered up again. */
	// Marcamos que los registros comunes deben ser reprogramados al encender el dispositivo nuevamente
	imx477->common_regs_written = false;

	return 0;
}

/**
 * Pone el dispositivo IMX477 en modo suspendido.
 *
 * @param dev Puntero al dispositivo asociado.
 * @return 0 si se realiza correctamente, o un código de error negativo en caso de fallo.
 */
static int __maybe_unused imx477_suspend(struct device *dev) {
	// Obtenemos el cliente I2C a partir del dispositivo
	struct i2c_client *client = to_i2c_client(dev);
	// Obtenemos el subdispositivo V4L2 asociado al cliente I2C
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	// Convertimos el subdispositivo V4L2 en la estructura específica del sensor IMX477
	struct imx477 *imx477 = to_imx477(sd);
	// Si el dispositivo está en modo streaming, lo detenemos antes de suspender
	if (imx477->streaming)
		imx477_stop_streaming(imx477);

	return 0;
}

/**
 * Reanuda el funcionamiento de la cámara IMX477 después de una suspensión.
 *
 * @param dev Puntero al dispositivo asociado.
 * @return 0 si se reanuda correctamente, o un código de error negativo en caso de fallo.
 */
static int __maybe_unused imx477_resume(struct device *dev) {
	// Obtenemos el cliente I2C a partir del dispositivo
	struct i2c_client *client = to_i2c_client(dev);
	// Obtenemos el subdispositivo V4L2 asociado al cliente I2C
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	// Convertimos el subdispositivo V4L2 en la estructura específica del sensor IMX477
	struct imx477 *imx477 = to_imx477(sd);
	int ret;
	// Si el dispositivo estaba en modo de streaming antes de la suspensión, lo reanudamos
	if (imx477->streaming) {
		ret = imx477_start_streaming(imx477);
		if (ret)
			goto error;
	}

	return 0;

error:
	// Si ocurre un error al reanudar, detenemos el streaming y marcamos el estado de streaming como apagado
	imx477_stop_streaming(imx477);
	imx477->streaming = 0;
	return ret;
}

/**
 * Obtiene los reguladores necesarios para el sensor IMX477.
 *
 * @param imx477 Puntero a la estructura que representa el sensor IMX477.
 * @return 0 si se obtienen los reguladores correctamente, o un código de error negativo en caso de fallo.
 */
static int imx477_get_regulators(struct imx477 *imx477) {
	// Obtenemos el cliente I2C asociado al subdispositivo V4L2
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	unsigned int i;
	// Configuramos los nombres de los reguladores en la estructura de suministros (supplies)
	for (i = 0; i < IMX477_NUM_SUPPLIES; i++)
		imx477->supplies[i].supply = imx477_supply_name[i];
	
	// Obtenemos los reguladores utilizando la función devm_regulator_bulk_get
	return devm_regulator_bulk_get(&client->dev,
				       IMX477_NUM_SUPPLIES,
				       imx477->supplies);
}

/* Verify chip ID */
/**
 * @brief Verifica la identidad del chip IMX477 comparando su ID esperado.
 *
 * Lee el registro de identificación del chip y compara su valor con el ID esperado.
 *
 * @param imx477 Puntero a la estructura del sensor IMX477.
 * @param expected_id ID esperado del chip IMX477.
 * @return 0 si la identificación fue exitosa y coincide con el ID esperado, un código de error en caso contrario.
 */
static int imx477_identify_module(struct imx477 *imx477, u32 expected_id) {
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	int ret;
	u32 val;
	// Lee el registro de identificación del chip
	ret = imx477_read_reg(imx477, IMX477_REG_CHIP_ID,
			      IMX477_REG_VALUE_16BIT, &val);
	if (ret) { 	// Error al leer el ID del chip
		dev_err(&client->dev, "failed to read chip id %x, with error %d\n",
			expected_id, ret);
		return ret;
	}

	// Compara el ID leído con el ID esperado
	if (val != expected_id) { 		// ID del chip no coincide con el esperado
		dev_err(&client->dev, "chip id mismatch: %x!=%x\n",
			expected_id, val);
		return -EIO; // Error IO
	}
	
	// Informa que se ha encontrado el dispositivo con el ID correcto
	dev_info(&client->dev, "Device found is imx%x\n", val);

	return 0;
}

/**
 * @brief Define las operaciones principales del núcleo para el subdispositivo V4L2.
 *
 * Estas operaciones incluyen la subscripción y desubscripción de eventos. Se útilizan las
 * operaciones estandar de V4L2
 */
static const struct v4l2_subdev_core_ops imx477_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

/**
 * @brief Define las operaciones de video para el subdispositivo V4L2.
 *
 * Estas operaciones incluyen el control de inicio y finalización del streaming.
 */
static const struct v4l2_subdev_video_ops imx477_video_ops = {
	.s_stream = imx477_set_stream,
};

/**
 * @brief Define las operaciones de los PAD para el subdispositivo V4L2.
 *
 * Estas operaciones incluyen la enumeración del código de media,
 * obtener y establecer el formato, obtener selección, y enumeración
 * del tamaño de frame.
 */
static const struct v4l2_subdev_pad_ops imx477_pad_ops = {
	.enum_mbus_code = imx477_enum_mbus_code,
	.get_fmt = imx477_get_pad_format,
	.set_fmt = imx477_set_pad_format,
	.get_selection = imx477_get_selection,
	.enum_frame_size = imx477_enum_frame_size,
};

/**
 * @brief Guarda las estructuras con las funciones para su acceso al subdispositivo V4L2.
 *
 * Esta estructura almacena las operaciones principales del subdispositivo,
 * incluyendo las operaciones básicas del núcleo, operaciones de video y
 * operaciones del PAD.
 */
static const struct v4l2_subdev_ops imx477_subdev_ops = {
	.core = &imx477_core_ops,
	.video = &imx477_video_ops,
	.pad = &imx477_pad_ops,
};

/**
 * @brief Define la función de encendido del sensor para su acceso interno al subdispositivo V4L2.
 *
 * Esta estructura define la función de encendido del sensor que será utilizada
 * para el acceso interno al subdispositivo V4L2.
 */
static const struct v4l2_subdev_internal_ops imx477_internal_ops = {
	.open = imx477_open,
};

/* Initialize control handlers */
/**
 * @brief Inicializa los controladores de los ajustes de la cámara.
 *
 * Esta función inicializa los controladores de ajustes de la cámara, estableciendo
 * los parámetros de control como ganancia, exposición, flip, patrones de prueba, etc.
 *
 * @param imx477 Puntero a la estructura del sensor IMX477.
 * @return Retorna 0 si la inicialización es exitosa, de lo contrario retorna un código de error.
 */
static int imx477_init_controls(struct imx477 *imx477) {
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct i2c_client *client = v4l2_get_subdevdata(&imx477->sd);
	struct v4l2_fwnode_device_properties props;
	unsigned int i;
	int ret;

	// Inicializa el controlador de controladores con capacidad para 16 controles
	ctrl_hdlr = &imx477->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
	if (ret)
		return ret;

	// Inicializa el mutex para la estructura IMX477
	mutex_init(&imx477->mutex);
	ctrl_hdlr->lock = &imx477->mutex;

	/* By default, PIXEL_RATE is read only */
	// Añade los controladores al manejador
	imx477->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					       V4L2_CID_PIXEL_RATE, // ID de control del ratio del pixel
					       IMX477_PIXEL_RATE, // Mínimo
					       IMX477_PIXEL_RATE, 1, // Máximo, step
					       IMX477_PIXEL_RATE); // Default
					
	if (imx477->pixel_rate)
		imx477->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY; // Bloqueamos el pixel_rate a read_only

	/* LINK_FREQ is also read only */
	imx477->link_freq =
		v4l2_ctrl_new_int_menu(ctrl_hdlr, &imx477_ctrl_ops,
				       V4L2_CID_LINK_FREQ, // ID de control de la frecuencia del link
				       ARRAY_SIZE(imx477_link_freq_menu) - 1, 0, // Maximo, Skip_mask
				       imx477_link_freq_menu); // Definición
	if (imx477->link_freq)
		imx477->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY; // Bloqueamos el link_freq como read only

	/*
	 * Create the controls here, but mode specific limits are setup
	 * in the imx477_set_framing_limits() call below.
	 */
	 // Prepara el tiempo de separación de frames
	imx477->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					   V4L2_CID_VBLANK, 0, 0xffff, 1, 0);
	// Prepara el tiempo de separación de lineas
	imx477->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					   V4L2_CID_HBLANK, 0, 0xffff, 1, 0);

	// Prapara los valores de la exposición
	imx477->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX477_EXPOSURE_MIN,
					     IMX477_EXPOSURE_MAX,
					     IMX477_EXPOSURE_STEP,
					     IMX477_EXPOSURE_DEFAULT);
	
	// Prepara los valores de la ganancia analogica
	v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX477_ANA_GAIN_MIN, IMX477_ANA_GAIN_MAX,
			  IMX477_ANA_GAIN_STEP, IMX477_ANA_GAIN_DEFAULT);

	// Prepara los valores de la ganancia digital
	v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX477_DGTL_GAIN_MIN, IMX477_DGTL_GAIN_MAX,
			  IMX477_DGTL_GAIN_STEP, IMX477_DGTL_GAIN_DEFAULT);

	// Establece la posibilidad de flip vertical
	imx477->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (imx477->hflip)
		imx477->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT; // Se puede modificar
	
	// Establece la posibilidad de flip horizontal
	imx477->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (imx477->vflip)
		imx477->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT; // Se puede modificar
	
	// Establece los controles, indicando los tipos de pruebas
	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx477_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx477_test_pattern_menu) - 1,
				     0, 0, imx477_test_pattern_menu);

	// Preparamos cada uno de los tipos de patrones de prueba
	for (i = 0; i < 4; i++) {
		/*
		 * The assumption is that
		 * V4L2_CID_TEST_PATTERN_GREENR == V4L2_CID_TEST_PATTERN_RED + 1
		 * V4L2_CID_TEST_PATTERN_BLUE   == V4L2_CID_TEST_PATTERN_RED + 2
		 * V4L2_CID_TEST_PATTERN_GREENB == V4L2_CID_TEST_PATTERN_RED + 3
		 */
		v4l2_ctrl_new_std(ctrl_hdlr, &imx477_ctrl_ops,
				  V4L2_CID_TEST_PATTERN_RED + i,
				  IMX477_TEST_PATTERN_COLOUR_MIN,
				  IMX477_TEST_PATTERN_COLOUR_MAX,
				  IMX477_TEST_PATTERN_COLOUR_STEP,
				  IMX477_TEST_PATTERN_COLOUR_MAX);
		/* The "Solid color" pattern is white by default */
	}
	
	// En el caso de que haya habido algun error durante la preparacion
	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}
	// Preparamos y validamos las propiedades del dispositivo del nodo V4L2 y rellena la estructura dev
	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;
	// Registra los manejadores del dispositivo
	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx477_ctrl_ops,
					      &props);
	if (ret)
		goto error;
	
	// Prepara el manejador
	imx477->sd.ctrl_handler = ctrl_hdlr;
	// Obtenemos el mutex
	mutex_lock(&imx477->mutex);

	/* Setup exposure and frame/line length limits. */
	// Modificamos los limites de VBlank y HBlank que solo habiamos creado antes
	imx477_set_framing_limits(imx477);
	// liberamos el Mutex
	mutex_unlock(&imx477->mutex);

	return 0;

error:
	// En caso de error liberamos el manejador y el mutex
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&imx477->mutex);

	return ret;
}

/**
 * @brief Libera el manejador de controles y el mutex asociado.
 *
 * Esta función libera el manejador y el mutex asociado a la estructura
 * IMX477.
 *
 * @param imx477 Puntero a la estructura del sensor IMX477.
 */
static void imx477_free_controls(struct imx477 *imx477) {
	v4l2_ctrl_handler_free(imx477->sd.ctrl_handler);
	mutex_destroy(&imx477->mutex);
}

/**
 * @brief Comprueba la configuración del hardware para el sensor IMX477.
 *
 * Esta función verifica la configuración del hardware del sensor IMX477,
 * incluyendo el número de líneas de datos MIPI CSI2 y la frecuencia de enlace
 * establecida en el dispositivo tree.
 *
 * @param dev Puntero al dispositivo asociado.
 * @return 0 si la configuración es válida, de lo contrario un código de error negativo.
 */
static int imx477_check_hwcfg(struct device *dev) {
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY // Compact Camera Port 2
	};
	int ret = -EINVAL;
	// Obtiene el punto de conexión del sensor desde el nodo en el árbol de dispositivos
	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}
	// Parsea y asigna los datos del endpoint (asignación de memoria)
	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
		dev_err(dev, "could not parse endpoint\n");
		goto error_out;
	}

	/* Check the number of MIPI CSI2 data lanes */
	// Comprueba el número de líneas de datos MIPI CSI2
	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 2) {
		dev_err(dev, "only 2 data lanes are currently supported\n");
		goto error_out;
	}

	/* Check the link frequency set in device tree */ 
	// Comprueba la frecuencia de enlace establecida en el device tree
	if (!ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	// Verifica que solo hay una frecuencia de enlace y que coincide con la frecuencia predeterminada
	if (ep_cfg.nr_of_link_frequencies != 1 ||
	    ep_cfg.link_frequencies[0] != IMX477_DEFAULT_LINK_FREQ) {
		dev_err(dev, "Link frequency not supported: %lld\n",
			ep_cfg.link_frequencies[0]);
		goto error_out;
	}

	ret = 0;

error_out:
	// Libera la estructura de configuración del endpoint
	v4l2_fwnode_endpoint_free(&ep_cfg);
	// Decrementa la referencia al endpoint obtenido
	fwnode_handle_put(endpoint);

	return ret;
}

/**
 * @brief Establece el identificador del chip IMX477 y el número de registros extras a 0.
 */
static const struct imx477_compatible_data imx477_compatible = {
	.chip_id = IMX477_CHIP_ID, // Identificador del chip IMX477
	.extra_regs = {
		.num_of_regs = 0, // Número de registros extras es 0
		.regs = NULL
	}
};

/**
 * @brief Indica los registros extra para el funcionamiento del IMX378 como compatibles.
 */
static const struct imx477_reg imx378_regs[] = {
	{0x3e35, 0x01},
	{0x4421, 0x08},
	{0x3ff9, 0x00},
};

/**
 * @brief Establece la estructura del IMX378 con su ID y los registros adicionales.
 */
static const struct imx477_compatible_data imx378_compatible = {
	.chip_id = IMX378_CHIP_ID,	  // Identificador del chip IMX378
	.extra_regs = {
		.num_of_regs = ARRAY_SIZE(imx378_regs), // Número de registros adicionales
		.regs = imx378_regs						// Registros adicionales específicos del IMX378
	}
};

/**
 * @brief Indica la lista de dispositivos compatibles con el driver.
 */
static const struct of_device_id imx477_dt_ids[] = {
	{ .compatible = "sony,imx477", .data = &imx477_compatible },
	{ .compatible = "sony,imx378", .data = &imx378_compatible },
	{ /* sentinel */ }
};

/**
 * Función usada para inicializar el dispositivo, preparar los recursos y registrar el dispositivo.
 *
 * @param client Puntero al cliente I2C asociado al dispositivo.
 * @return int Retorna 0 si la inicialización fue exitosa, o un código de error negativo en caso de fallo.
 */
static int imx477_probe(struct i2c_client *client) {
	struct device *dev = &client->dev;
	struct imx477 *imx477;
	const struct of_device_id *match;
	int ret;
	u32 tm_of;

	// Adquirimos memoria que será liberada si se quita el dispositivo, se inicializa a cero.
	imx477 = devm_kzalloc(&client->dev, sizeof(*imx477), GFP_KERNEL);// Se aloca en la zona del kernel
	if (!imx477)
		return -ENOMEM; // Error de memoria

	// Inicializamos el dispositivo como un subdispositivo V4L2 I2C.
	v4l2_i2c_subdev_init(&imx477->sd, client, &imx477_subdev_ops);
	// Comprobamos que coincida con uno de los dos IDs compatibles: IMX477 o IMX378.
	match = of_match_device(imx477_dt_ids, dev);
	if (!match)
		return -ENODEV; // Dispositivo no encontrado en el árbol de dispositivos.

	// Guarda los registros compatibles (IMX477 ninguno).
	imx477->compatible_data =
		(const struct imx477_compatible_data *)match->data;

	/* Check the hardware configuration in device tree */
	// Comprobamos la configuración del hardware en el árbol de dispositivos.
	if (imx477_check_hwcfg(dev))
		return -EINVAL; // Valor no válido.

	/* Default the trigger mode from OF to -1, which means invalid */
	ret = of_property_read_u32(dev->of_node, "trigger-mode", &tm_of); // Lee la propiedad "trigger-mode".
	imx477->trigger_mode_of = (ret == 0) ? tm_of : -1; // Guarda el modo de disparo obtenido del árbol.

	/* Get system clock (xclk) */
	// Obtenemos el reloj del sistema (xclk).
	imx477->xclk = devm_clk_get(dev, NULL); // Se liberará con el dispositivo
	if (IS_ERR(imx477->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(imx477->xclk);
	}

	// Obtiene la frecuencia del reloj y comprueba que es la esperada.
	imx477->xclk_freq = clk_get_rate(imx477->xclk);
	if (imx477->xclk_freq != IMX477_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			imx477->xclk_freq);
		return -EINVAL;
	}

	// Obtiene los reguladores necesarios para el dispositivo.
	ret = imx477_get_regulators(imx477);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	/* Request optional enable pin */
	// Solicita el pin de habilitación opcional (reset).
	imx477->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);

	/*
	 * The sensor must be powered for imx477_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	// Enciende el dispositivo.
	ret = imx477_power_on(dev);
	if (ret)
		return ret;
	// Comprueba que el módulo es el que debería ser.
	ret = imx477_identify_module(imx477, imx477->compatible_data->chip_id);
	if (ret)
		goto error_power_off;

	/* Initialize default format */
	imx477_set_default_format(imx477); // Inicializa el formato por defecto.

	/* Enable runtime PM and turn off the device */
	// Habilita el PM en tiempo de ejecución y apaga el dispositivo.
	pm_runtime_set_active(dev); // Marca el dispositivo como activo.
	pm_runtime_enable(dev);  // Habilita el PM.
	pm_runtime_idle(dev); // Pone el dispositivo en modo suspendido.

	/* This needs the pm runtime to be registered. */
	ret = imx477_init_controls(imx477); // Inicializa los controles del dispositivo.
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	imx477->sd.internal_ops = &imx477_internal_ops;  		// Guarda las operaciones internas.
	imx477->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | 		// Tiene un dispositivo asociado
			    V4L2_SUBDEV_FL_HAS_EVENTS;					// Indica compatibilidad con V4L2.
	imx477->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR; 	// Identifica el tipo de entidad como un sensor de cámara.

	/* Initialize source pads */
	imx477->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	imx477->pad[METADATA_PAD].flags = MEDIA_PAD_FL_SOURCE;

	// Inicializa los pads de la entidad de medios.
	ret = media_entity_pads_init(&imx477->sd.entity, NUM_PADS, imx477->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	// Registra el sensor de manera asíncrona en V4L2.
	ret = v4l2_async_register_subdev_sensor(&imx477->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	return 0;

error_media_entity:
	media_entity_cleanup(&imx477->sd.entity); 

error_handler_free:
	imx477_free_controls(imx477);

error_power_off:
	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	imx477_power_off(&client->dev);

	return ret;
}

/**
 * @brief Función para remover el dispositivo IMX477.
 *
 * @param client Puntero al cliente I2C asociado al dispositivo.
 */
static void imx477_remove(struct i2c_client *client) {
	// Obtenemos el subdispositivo V4L2
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx477 *imx477 = to_imx477(sd);  // Convertimos a estructura IMX477

	v4l2_async_unregister_subdev(sd); 		// Deregistramos el subdispositivo de V4L2
	media_entity_cleanup(&sd->entity); 		// Limpiamos la entidad de medios
	imx477_free_controls(imx477); 			// Liberamos los controles y el mutex
 
	pm_runtime_disable(&client->dev); 				// Deshabilitamos el PM
	if (!pm_runtime_status_suspended(&client->dev)) // Si no está suspendido
		imx477_power_off(&client->dev); 			// Apagamos el dispositivo
	pm_runtime_set_suspended(&client->dev); 		// Marcamos el PM como suspendido
}

/**
 * @brief Crea la tabla de dispositivos del módulo para los identificadores de dispositivo de
 *        dispositivo de dispositivo que soporta el IMX477.
 */
MODULE_DEVICE_TABLE(of, imx477_dt_ids);

/**
 * @brief Define las operaciones de gestión de energía del dispositivo IMX477.
 */
static const struct dev_pm_ops imx477_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx477_suspend, imx477_resume) // Operaciones para suspender y resumir
	SET_RUNTIME_PM_OPS(imx477_power_off, imx477_power_on, NULL) // Operaciones para encender y apagar
};

/**
 * @brief Define el controlador I2C para el sensor IMX477.
 */
static struct i2c_driver imx477_i2c_driver = {
	.driver = {
		.name = "imx477", // Nombre del controlador
		.of_match_table	= imx477_dt_ids, // Tabla de identificación de dispositivos compatibles
		.pm = &imx477_pm_ops, // Operaciones de gestión de energía
	},
	.probe = imx477_probe, // Función de inicialización del dispositivo
	.remove = imx477_remove, // Función de eliminación del dispositivo
};

/**
 * @brief Registra el módulo como un controlador I2C.
 */
module_i2c_driver(imx477_i2c_driver);

MODULE_AUTHOR("Naushir Patuck <naush@raspberrypi.com>");
MODULE_DESCRIPTION("Sony IMX477 sensor driver");
MODULE_LICENSE("GPL v2");
