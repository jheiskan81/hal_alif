/*
 * Copyright (C) 2024 Alif Semiconductor.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _ALIF_HAL_OSPI_H_
#define _ALIF_HAL_OSPI_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
	volatile uint32_t  OSPI_CTRLR0;		/* (@ 0x00) Control Reg 0 */
	volatile uint32_t  OSPI_CTRLR1;		/* (@ 0x04) Control Reg 1 */
	volatile uint32_t  OSPI_ENR;		/* (@ 0x08) Enable Reg */
	volatile uint32_t  RESERVED;
	volatile uint32_t  OSPI_SER;		/* (@ 0x10) Slave Enable Reg */
	volatile uint32_t  OSPI_BAUDR;		/* (@ 0x14) Baud Rate Select Reg */
	volatile uint32_t  OSPI_TXFTLR;		/* (@ 0x18) Transmit FIFO Threshold Level Reg */
	volatile uint32_t  OSPI_RXFTLR;		/* (@ 0x1C) Receive FIFO Threshold Level Reg */
	volatile uint32_t  OSPI_TXFLR;		/* (@ 0x20) Transmit FIFO Level Reg */
	volatile uint32_t  OSPI_RXFLR;		/* (@ 0x24) Receive FIFO Level Reg */
	volatile uint32_t  OSPI_SR;		/* (@ 0x28) Status Reg */
	volatile uint32_t  OSPI_IMR;		/* (@ 0x2C) Interrupt Mask Reg */
	volatile uint32_t  OSPI_ISR;		/* (@ 0x30) Interrupt Status Reg */
	volatile uint32_t  OSPI_RISR;		/* (@ 0x34) Raw Interrupt Status Reg */
	volatile uint32_t  OSPI_TXEICR;		/* (@ 0x38) Tx FIFO Err Intr Clear Reg */
	volatile uint32_t  OSPI_RXOICR;		/* (@ 0x3C) Rx FIFO Overflow Interrupt Clear Reg */
	volatile uint32_t  OSPI_RXUICR;		/* (@ 0x40) Rx FIFO Underflow Interrupt Clear Reg */
	volatile uint32_t  RESERVED1;
	volatile uint32_t  OSPI_ICR;		/* (@ 0x048) OSPI Interrupt Clear Reg */
	volatile uint32_t  OSPI_DMACR;		/* (@ 0x04C) OSPI DMA Control Reg */
	volatile uint32_t  OSPI_DMATDLR;	/* (@ 0x050) OSPI DMA Transmit Data Level Reg */
	volatile uint32_t  OSPI_DMARDLR;	/* (@ 0x054) OSPI DMA Receive Data Level Reg */
	volatile uint32_t  OSPI_IDR;		/* (@ 0x058) Reserved */
	volatile uint32_t  OSPI_VERSION_ID;	/* (@ 0x05C) Reserved */
	volatile uint32_t  OSPI_DR0;		/* (@ 0x060) Data Reg (n) */
	volatile uint32_t  OSPI_DR1;		/* (@ 0x064) Data Reg (n) */
	volatile uint32_t  OSPI_DR2;		/* (@ 0x068) Data Reg (n) */
	volatile uint32_t  OSPI_DR3;		/* (@ 0x06C) Data Reg (n) */
	volatile uint32_t  OSPI_DR4;		/* (@ 0x070) Data Reg (n) */
	volatile uint32_t  OSPI_DR5;		/* (@ 0x074) Data Reg (n) */
	volatile uint32_t  OSPI_DR6;		/* (@ 0x078) Data Reg (n) */
	volatile uint32_t  OSPI_DR7;		/* (@ 0x07C) Data Reg (n) */
	volatile uint32_t  OSPI_DR8;		/* (@ 0x080) Data Reg (n) */
	volatile uint32_t  OSPI_DR9;		/* (@ 0x084) Data Reg (n) */
	volatile uint32_t  OSPI_DR10;		/* (@ 0x088) Data Reg (n) */
	volatile uint32_t  OSPI_DR11;		/* (@ 0x08C) Data Reg (n) */
	volatile uint32_t  OSPI_DR12;		/* (@ 0x090) Data Reg (n) */
	volatile uint32_t  OSPI_DR13;		/* (@ 0x094) Data Reg (n) */
	volatile uint32_t  OSPI_DR14;		/* (@ 0x098) Data Reg (n) */
	volatile uint32_t  OSPI_DR15;		/* (@ 0x09C) Data Reg (n) */
	volatile uint32_t  OSPI_DR16;		/* (@ 0x0A0) Data Reg (n) */
	volatile uint32_t  OSPI_DR17;		/* (@ 0x0A4) Data Reg (n) */
	volatile uint32_t  OSPI_DR18;		/* (@ 0x0A8) Data Reg (n) */
	volatile uint32_t  OSPI_DR19;		/* (@ 0x0AC) Data Reg (n) */
	volatile uint32_t  OSPI_DR20;		/* (@ 0x0B0) Data Reg (n) */
	volatile uint32_t  OSPI_DR21;		/* (@ 0x0B4) Data Reg (n) */
	volatile uint32_t  OSPI_DR22;		/* (@ 0x0B8) Data Reg (n) */
	volatile uint32_t  OSPI_DR23;		/* (@ 0x0BC) Data Reg (n) */
	volatile uint32_t  OSPI_DR24;		/* (@ 0x0C0) Data Reg (n) */
	volatile uint32_t  OSPI_DR25;		/* (@ 0x0C4) Data Reg (n) */
	volatile uint32_t  OSPI_DR26;		/* (@ 0x0C8) Data Reg (n) */
	volatile uint32_t  OSPI_DR27;		/* (@ 0x0CC) Data Reg (n) */
	volatile uint32_t  OSPI_DR28;		/* (@ 0x0D0) Data Reg (n) */
	volatile uint32_t  OSPI_DR29;		/* (@ 0x0D4) Data Reg (n) */
	volatile uint32_t  OSPI_DR30;		/* (@ 0x0D8) Data Reg (n) */
	volatile uint32_t  OSPI_DR31;		/* (@ 0x0DC) Data Reg (n) */
	volatile uint32_t  OSPI_DR32;		/* (@ 0x0E0) Data Reg (n) */
	volatile uint32_t  OSPI_DR33;		/* (@ 0x0E4) Data Reg (n) */
	volatile uint32_t  OSPI_DR34;		/* (@ 0x0E8) Data Reg (n) */
	volatile uint32_t  OSPI_DR35;		/* (@ 0x0EC) Data Reg (n) */
	volatile uint32_t  OSPI_RX_SAMPLE_DELAY;	/* (@ 0x0F0) RX Sample Delay Reg */
	volatile uint32_t  OSPI_SPI_CTRLR0;	/* (@ 0x0F4) SPI Control Reg */
	volatile uint32_t  OSPI_DDR_DRIVE_EDGE;	/* (@ 0x0F8) Transmit Drive Edge Reg */
	volatile uint32_t  OSPI_XIP_MODE_BITS;	/* (@ 0x0FC) XIP Mode Bits Reg */
	volatile uint32_t  OSPI_XIP_INCR_INST;	/* (@ 0x100) XIP INCR Transfer Opcode Reg */
	volatile uint32_t  OSPI_XIP_WRAP_INST;	/* (@ 0x104) XIP WRAP Transfer Opcode Reg */
	volatile uint32_t  OSPI_XIP_CTRL;	/* (@ 0x108) XIP Control Reg */
	volatile uint32_t  OSPI_XIP_SER;	/* (@ 0x10C) XIP Slave Enable Reg */
	volatile uint32_t  RESERVED2;
	volatile uint32_t  OSPI_XIP_CNT_TIME_OUT;        /* (@ 0x114) For Continuous Tx */
	volatile const uint32_t RESERVED3[10];
	volatile uint32_t  OSPI_XIP_WRITE_INCR_INST;     /* (@ 0x140) INCR Transfer Opcode */
	volatile uint32_t  OSPI_XIP_WRITE_WRAP_INST;     /* (@ 0x144) WRAP Transfer Opcode */
	volatile uint32_t  OSPI_XIP_WRITE_CTRL;          /* (@ 0x148) XIP Write Control */
} OSPI_Type;

#define OSPI_TX_FIFO_DEPTH                               256U
#define OSPI_RX_FIFO_DEPTH                               256U

#define OSPI_ENABLE                                      1
#define OSPI_DISABLE                                     0

/* SPI Control Register 0 (CTRLR0) bit Definition, Macros, Offsets and Masks
 * these include DFS, FRF, SCPH, SCPOL, TMOD, etc
 */
/* Data Frame DFS bit[4:0]*/
#define SPI_CTRLR0_DFS                                  0U
#define SPI_CTRLR0_DFS_8bit                             0x07U
#define SPI_CTRLR0_DFS_16bit                            0x0FU
#define SPI_CTRLR0_DFS_32bit                            0x1FU
#define SPI_CTRLR0_DFS_MASK                             (0x1FU << SPI_CTRLR0_DFS)

/* Frame Format FRF bit[7:6] */
#define SPI_CTRLR0_FRF                  6U
#define SPI_CTRLR0_FRF_MASK             (0x3UL << SPI_CTRLR0_FRF)
#define SPI_CTRLR0_FRF_MOTOROLA         0x00U     /* Motorola SPI Frame Format */
#define SPI_CTRLR0_FRF_TI               0x40U     /* Texas Instruments SSP Frame Format */
#define SPI_CTRLR0_FRF_MICROWIRE        0x80U     /* National Semiconductors Microwire Format */

/* Serial Clock Polarity SCPOL | Serial Clock Phase SCPH bit[9:8] */
#define SPI_CTRLR0_SC                           8U
#define SPI_CTRLR0_SC_MASK                      (0x3UL << SPI_CTRLR0_SC)
#define SPI_CTRLR0_SCPH_HIGH                    0x100U     /* 0x1 SPI SCPH high */
#define SPI_CTRLR0_SCPH_LOW                     0x000U     /* 0x0 SPI SCPH low */
#define SPI_CTRLR0_SCPOL_HIGH                   0x200U     /* 0x2 SPI SCPOL high */
#define SPI_CTRLR0_SCPOL_LOW                    0x000U     /* 0x0 SPI SCPOL low */

/* Transfer Mode TMOD bit[11:10] */
#define SPI_CTRLR0_TMOD                         10U
#define SPI_CTRLR0_TMOD_MASK                    (0x3UL << SPI_CTRLR0_TMOD)
#define SPI_CTRLR0_TMOD_TRANSFER                0x000U     /* 0x0 SPI transfer mode */
#define SPI_CTRLR0_TMOD_SEND_ONLY               0x400U     /* 0x1 SPI send only mode */
#define SPI_CTRLR0_TMOD_RECEIVE_ONLY            0x800U     /* 0x2 SPI receive only mode */
#define SPI_CTRLR0_TMOD_EEPROM_READ_ONLY        0xC00U     /* 0x3 SPI EEPROM read only mode */

/* Slave Select toggle Enable bit[14]
 * While operating in SPI mode with clock phase (SCPH) set to
 * 0, this register controls the behavior of the slave select line
 *  between data frames.
 */
/* Slave Select toggle Enable SSTE bit[14] */
#define SPI_CTRLR0_SSTE                 14U
#define SPI_CTRLR0_SSTE_MASK            (1U << SPI_CTRLR0_SSTE)
#define SPI_CTRLR0_SSTE_ENABLE          0x4000U     /* 0x1 SPI slave select toggle enable */
#define SPI_CTRLR0_SSTE_DISABLE         0x0000U     /* 0x0 SPI slave select toggle disable */

/* Control Frame Size for the Microwire frame format CFS bit[19:16]*/
#define SPI_CTRLR0_CFS                  16U
#define SPI_CTRLR0_CFS_MASK             (0xFU << SPI_CTRLR0_CFS)

/* SPI Frame Format SPI_FRF bit[23:22]*/
#define SPI_CTRLR0_SPI_FRF              22U
#define SPI_CTRLR0_SPI_FRF_MASK         (0x3U << SPI_CTRLR0_SPI_FRF)
#define SPI_CTRLR0_SPI_FRF_STANDRAD     0x000000U    /* 0x0 Standard SPI Format */
#define SPI_CTRLR0_SPI_FRF_DUAL         0x400000U    /* 0x1 Dual SPI Format */
#define SPI_CTRLR0_SPI_FRF_QUAD         0x800000U    /* 0X2 Quad SPI Format */
#define SPI_CTRLR0_SPI_FRF_OCTAL        0xC00000U    /* 0X2 Octal SPI Format */

/* SPI Hyperbus Frame format enable SPI_HYPERBUS_EN bit[24] */
#define SPI_CTRLR0_SPI_HYPERBUS_EN              24
#define SPI_CTRLR0_SPI_HYPERBUS_EN_SSTE_MASK    (1 << SPI_CTRLR0_SPI_HYPERBUS_EN)
#define SPI_CTRLR0_SPI_HYPERBUS_ENABLE          0x4000   /* SPI Hyperbus Frame format enable */
#define SPI_CTRLR0_SPI_HYPERBUS_DISABLE         0x0000   /* sPI Hyperbus Frame format disable */

/* SPI is working in Master or Slave SSI_IS_MST bit[31] */
#define SPI_CTRLR0_SSI_IS_MST                           31U
#define SPI_CTRLR0_SSI_IS_MST_MASK                      (0x1UL << SPI_CTRLR0_SSI_IS_MST)
#define SPI_CTRLR0_SSI_IS_MST_MASTER                    0x80000000U  /* 0x1 SPI master */
#define SPI_CTRLR0_SSI_IS_MST_SLAVE                     0x00000000U  /* 0x0 SPI slave  */

/* Quad SPI MODE Macros */
#define SPI_CTRLR0_SPI_QUAD_ENABLE                     (0x2UL << SPI_CTRLR0_SPI_FRF)
#define SPI_CTRLR0_SPI_QUAD_TX_MODE                     SPI_TMOD_SEND_ONLY
#define SPI_CTRLR0_SPI_QUAD_RX_MODE                     SPI_TMOD_RECEIVE_ONLY

/* Octal SPI MODE Macros */
#define SPI_CTRLR0_SPI_OCTAL_ENABLE                     (0x3UL << SPI_CTRLR0_SPI_FRF)
#define SPI_CTRLR0_SPI_OCTAL_TX_RX_MODE                 SPI_TMOD_TRANSFER
#define SPI_CTRLR0_SPI_OCTAL_TX_MODE                    SPI_TMOD_SEND_ONLY
#define SPI_CTRLR0_SPI_OCTAL_RX_MODE                    SPI_TMOD_RECEIVE_ONLY

#define SPI_IMR_TX_FIFO_EMPTY_INTERRUPT_MASK            0x00000001U  /* Tx fifo empty intr */
#define SPI_IMR_TX_FIFO_OVER_FLOW_INTERRUPT_MASK        0x00000002U  /* Tx fifo overflow intr */
#define SPI_IMR_RX_FIFO_UNDER_FLOW_INTERRUPT_MASK       0x00000004U  /* Rx fifo underflow intr */
#define SPI_IMR_RX_FIFO_OVER_FLOW_INTERRUPT_MASK        0x00000008U  /* Rx fifo Overflow intr */
#define SPI_IMR_RX_FIFO_FULL_INTERRUPT_MASK             0x00000010U  /* Rx fifo full intr */
#define SPI_IMR_MULTI_MASTER_CONTENTION_INTERRUPT_MASK  0x00000020U  /* Multi-Master contention */

#define SPI_TXFTLR_TFT_SHIFT                            0U
#define SPI_TXFTLR_TFT_MASK                             (0xFFFFU << SPI_TXFTLR_TFT_SHIFT)
#define SPI_TXFTLR_TXFTHR_SHIFT                         16U
#define SPI_TXFTLR_TXFTHR_MASK                          (0xFFFFU << SPI_TXFTLR_TXFTHR_SHIFT)

/* Bit fields in SPI_CTRLR0 */
#define SPI_CTRLR0_CLK_STRETCH_EN_OFFSET        30U
#define SPI_CTRLR0_XIP_PREFETCH_EN_OFFSET       29U
#define SPI_CTRLR0_XIP_MBL_OFFSET               26U
#define SPI_CTRLR0_SPI_RXDS_SIG_EN_OFFSET       25U
#define SPI_CTRLR0_SPI_DM_EN_OFFSET             24U
#define SPI_CTRLR0_XIP_CONT_EN_OFFSET           21U
#define SPI_CTRLR0_XIP_INST_EN_OFFSET           20U
#define SPI_CTRLR0_XIP_DFS_HC_OFFSET            19U
#define SPI_CTRLR0_SPI_RXDS_EN_OFFSET           18U
#define SPI_CTRLR0_INST_DDR_EN_OFFSET           17U
#define SPI_CTRLR0_SPI_DDR_EN_OFFSET            16U
#define SPI_CTRLR0_WAIT_CYCLES_OFFSET           11U
#define SPI_CTRLR0_INST_L_OFFSET                8U
#define SPI_CTRLR0_XIP_MD_EN_OFFSET             7U
#define SPI_CTRLR0_ADDR_L_OFFSET                2U
#define SPI_CTRLR0_TRANS_TYPE_OFFSET            0U

#define SPI_CTRLR0_TRANS_TYPE_MASK              3U
#define SPI_TRANS_TYPE_STANDARD                 0U
#define SPI_TRANS_TYPE_FRF_DEFINED              2U  /* CTRLR0.SPI_FRF :Standard/Dual/Quad/Octal */

#define SPI_CTRLR0_SPI_RXDS_ENABLE              1U
#define SPI_CTRLR0_SPI_RXDS_DISABLE             0U

#define SPI_CTRLR0_INST_L_0bit                  0x0U
#define SPI_CTRLR0_INST_L_4bit                  0x1U
#define SPI_CTRLR0_INST_L_8bit                  0x2U
#define SPI_CTRLR0_INST_L_16bit                 0x3U

#define SPI_DMACR_TDMAE                         2U
#define SPI_DMACR_RDMAE                         1U

#define XIP_CTRL_RXDS_VL_EN_OFFSET              30U
#define XIP_CTRL_XIP_PREFETCH_EN_OFFSET         29U
#define XIP_CTRL_XIP_MBL_OFFSET                 26U
#define XIP_CTRL_RXDS_SIG_EN_OFFSET             25U
#define XIP_CTRL_XIP_HYPERBUS_EN_OFFSET         24U
#define XIP_CTRL_CONT_XFER_EN_OFFSET            23U
#define XIP_CTRL_INST_EN_OFFSET                 22U
#define XIP_CTRL_RXDS_EN_OFFSET                 21U
#define XIP_CTRL_INST_DDR_EN_OFFSET             20U
#define XIP_CTRL_DDR_EN_OFFSET                  19U
#define XIP_CTRL_DFS_HC_OFFSET                  18U
#define XIP_CTRL_WAIT_CYCLES_OFFSET             13U
#define XIP_CTRL_MD_BITS_EN_OFFSET              12U
#define XIP_CTRL_INST_L_OFFSET                  9U
#define XIP_CTRL_ADDR_L_OFFSET                  4U
#define XIP_CTRL_TRANS_TYPE_OFFSET              2U
#define XIP_CTRL_FRF_OFFSET                     0U

#define XIP_WRITE_CTRL_XIPWR_DFS_HC_OFFSET      21U
#define XIP_WRITE_CTRL_XIPWR_WAIT_CYCLES        16U
#define XIP_WRITE_CTRL_XIPWR_DM_EN_OFFSET       14U
#define XIP_WRITE_CTRL_XIPWR_RXDS_SIG_EN_OFFSET 13U
#define XIP_WRITE_CTRL_XIPWR_HYPERBUS_EN_OFFSET 12U
#define XIP_WRITE_CTRL_WR_INST_DDR_EN_OFFSET    11U
#define XIP_WRITE_CTRL_WR_SPI_DDR_EN_OFFSET     10U
#define XIP_WRITE_CTRL_WR_INST_L_OFFSET         8U
#define XIP_WRITE_CTRL_WR_ADDR_L_OFFSET         4U
#define XIP_WRITE_CTRL_WR_TRANS_TYPE_OFFSET     2U
#define XIP_WRITE_CTRL_WR_FRF_OFFSET            0U

#define SPI_SR_TX_FIFO_EMPTY                    0x4U
#define SPI_SR_BUSY                             0x1U

#define SPI_TX_FIFO_EMPTY_EVENT                 0x01U  /* Tx fifo empty interrupt mask*/
#define SPI_TX_FIFO_OVER_FLOW_EVENT             0x02U  /* Tx fifo overflow interrupt mask*/
#define SPI_RX_FIFO_UNDER_FLOW_EVENT            0x04U  /* Rx fifo underflow interrupt mask*/
#define SPI_RX_FIFO_OVER_FLOW_EVENT             0x08U  /* Rx fifo Overflow interrupt mask*/
#define SPI_RX_FIFO_FULL_EVENT                  0x10U  /* Rx fifo full interrupt mask*/
#define SPI_MULTI_MASTER_CONTENTION_EVENT       0x20U  /* Multi-Master contention intr mask*/

/**
 * enum SPI_FRAME_FORMAT.
 * SPI frame formats.
 */
typedef enum _SPI_FRAME_FORMAT {
	SPI_FRAME_FORMAT_STANDARD,          /* Standard SPI frame format */
	SPI_FRAME_FORMAT_DUAL,              /* Dual SPI frame format */
	SPI_FRAME_FORMAT_QUAD,              /* Quad SPI frame format */
	SPI_FRAME_FORMAT_OCTAL              /* Octal SPI frame format */
} SPI_FRAME_FORMAT;

/**
 * enum SPI_MODE.
 * SPI modes.
 */
typedef enum _SPI_MODE {
	SPI_MODE_0,                             /* SPI Mode - 0 : CPHA = 0, CPOL = 0 */
	SPI_MODE_1,                             /* SPI Mode - 1 : CPHA = 1, CPOL = 0 */
	SPI_MODE_2,                             /* SPI Mode - 2 : CPHA = 0, CPOL = 1 */
	SPI_MODE_3                              /* SPI Mode - 3 : CPHA = 1, CPOL = 1 */
} SPI_MODE;

/**
 * enum SPI_TMOD.
 * SPI transfer modes.
 */
typedef enum _SPI_TMOD {
	SPI_TMOD_TX_AND_RX     = 0x00,         /* Transmit and Receive */
	SPI_TMOD_TX            = 0x01,         /* Transmit only  */
	SPI_TMOD_RX            = 0x02,         /* Receive only */
	SPI_TMOD_EEPROM_READ   = 0x03          /* EEPROM read  */
} SPI_TMOD;

/**
 * enum SPI_SS_STATE.
 * SPI Slave Select States.
 */
typedef enum _SPI_SS_STATE {
	SPI_SS_STATE_DISABLE,                   /* Slave select disabled */
	SPI_SS_STATE_ENABLE,                    /* Slave select Enabled */
} SPI_SS_STATE;

/**
 * enum SPI_TRANSFER_STATUS.
 * Status of an ongoing SPI transfer.
 */
typedef enum _SPI_TRANSFER_STATUS {
	SPI_TRANSFER_STATUS_NONE,               /* Transfer status none */
	SPI_TRANSFER_STATUS_COMPLETE,           /* Transfer status complete */
	SPI_TRANSFER_STATUS_OVERFLOW,           /* Transfer status Tx/Rx overflow */
	SPI_TRANSFER_STATUS_MASTER_CONTENTION,  /* Transfer status master contention */
	SPI_TRANSFER_STATUS_RX_UNDERFLOW,       /* Transfer status Rx underflow */
} SPI_TRANSFER_STATUS;

/**
 * struct ospi_transfer_t.
 * Information about an ongoing OSPI transfer.
 */
typedef struct _ospi_transfer_t {
	volatile uint32_t               tx_current_cnt;     /* Current Tx Transfer count */
	volatile uint32_t               rx_current_cnt;     /* Current Rx Transfer count */
	uint32_t                        tx_total_cnt;       /* Total count to transmit */
	uint32_t                        rx_total_cnt;       /* Total count to receive */
	const uint32_t                  *tx_buff;           /* Pointer to TX buffer */
	void                            *rx_buff;           /* Pointer to Rx buffer */
	uint32_t                        tx_default_val;     /* Default value to Transfer */
	uint32_t                        spi_frf;            /* Standard/Dual/Quad/Octal */
	uint32_t                        addr_len;           /* Address length for the transfer */
	uint32_t                        dummy_cycle;        /* Dummy cycles for the transfer */
	uint32_t                        ddr;                /* DDR / SDR mode for the transfer */
	bool                            tx_default_enable;  /* Enable Tx default value transfer */
	SPI_TMOD                        mode;               /* SPI transfer mode */
	volatile SPI_TRANSFER_STATUS    status;             /* transfer status */
} ospi_transfer_t;

/**
 * \fn          static inline void ospi_disable(OSPI_Type *spi)
 * \brief       Disable the OSPI instance
 * \param[in]   ospi     Pointer to the OSPI register map
 * \return      none
 */
static inline void ospi_disable(OSPI_Type *ospi)
{
	ospi->OSPI_ENR = OSPI_DISABLE;
}

/**
 * \fn          static inline void ospi_enable(OSPI_Type *spi)
 * \brief       Enable the OSPI instance
 * \param[in]   ospi     Pointer to the OSPI register map
 * \return      none
 */
static inline void ospi_enable(OSPI_Type *ospi)
{
	ospi->OSPI_ENR = OSPI_ENABLE;
}

/***
 * \fn          static inline void ospi_mode_master(OSPI_Type *ospi)
 * \brief       Enable master mode in the OSPI instance.
 * \param[in]   ospi     Pointer to the OSPI register map
 * \return      none
 */
static inline void ospi_mode_master(OSPI_Type *ospi)
{
	ospi_disable(ospi);
	ospi->OSPI_CTRLR0 |= SPI_CTRLR0_SSI_IS_MST_MASTER;
	ospi_enable(ospi);
}

/***
 * \fn          static inline void ospi_mode_slave(OSPI_Type *ospi)
 * \brief       Enable slave mode in the OSPI instance.
 * \param[in]   ospi     Pointer to the OSPI register map
 * \return      none
 */
static inline void ospi_mode_slave(OSPI_Type *ospi)
{
	ospi_disable(ospi);
	ospi->OSPI_CTRLR0 &= ~SPI_CTRLR0_SSI_IS_MST_MASTER;
	ospi_enable(ospi);
}

/***
 * \fn          static inline void ospi_set_bus_speed(OSPI_Type *ospi, uint32_t speed, uint32_t clk)
 * \brief       Set the bus speed for the OSPI instance.
 * \param[in]   ospi    Pointer to the OSPI register map
 * \param[in]   speed   The bus speed to be set
 * \param[in]   clk     OSPI input clk
 * \return      none
 */
static inline void ospi_set_bus_speed(OSPI_Type *ospi, uint32_t speed, uint32_t clk)
{
	ospi_disable(ospi);
	ospi->OSPI_BAUDR = (clk / speed);
	ospi_enable(ospi);
}

/***
 * \fn          static inline uint32_t ospi_get_bus_speed(OSPI_Type *ospi, uint32_t clk)
 * \brief       Get the current bus speed of the OSPI instance.
 * \param[in]   ospi    Pointer to the OSPI register map
 * \param[in]   clk     OSPI input clk
 * \return      Current bus speed
 */
static inline uint32_t ospi_get_bus_speed(OSPI_Type *ospi, uint32_t clk)
{
	return clk / ospi->OSPI_BAUDR;
}

/***
 * \fn          static inline void ospi_mask_interrupts(OSPI_Type *spi)
 * \brief       Mask all the interrupts for the OSPI instance
 * \param[in]   ospi       Pointer to the SPI register map
 * \return      none
 */
static inline void ospi_mask_interrupts(OSPI_Type *ospi)
{
	ospi->OSPI_IMR = 0;
}

/***
 * \fn          static inline uint32_t ospi_get_dma_addr(OSPI_Type *ospi)
 * \brief       Get the Data(FIFO) register address for the OSPI instance
 * \param[in]   ospi     Pointer to the OSPI register map
 * \return      TXDMA register address.
 */
static inline volatile uint32_t *ospi_get_dma_addr(OSPI_Type *ospi)
{
	return &ospi->OSPI_DR0;
}

/***
 * \fn          void ospi_set_tx_dma_data_level(OSPI_Type *ospi, uint8_t level)
 * \brief       Set Tx DMA trigger level for the OSPI instance.
 * \param[in]   ospi       Pointer to the OSPI register map
 * \param[in]   trig_level Tx DMA trigger level
 * \return      none
 */
static inline void ospi_set_tx_dma_data_level(OSPI_Type *ospi, uint8_t level)
{
	ospi->OSPI_DMATDLR = level;
}

/***
 * \fn          void ospi_set_rx_dma_data_level(OSPI_Type *ospi, uint8_t level)
 * \brief       Set Tx DMA trigger level for the OSPI instance.
 * \param[in]   ospi       Pointer to the OSPI register map
 * \param[in]   trig_level Tx DMA trigger level
 * \return      none
 */
static inline void ospi_set_rx_dma_data_level(OSPI_Type *ospi, uint8_t level)
{
	ospi->OSPI_DMARDLR = level;
}

/**
 * \fn          void ospi_enable_tx_dma(OSPI_Type *ospi)
 * \brief       Enable Tx DMA for the OSPI instance.
 * \param[in]   ospi       Pointer to the OSPI register map
 * \return      none
 */
static inline void ospi_enable_tx_dma(OSPI_Type *ospi)
{
	ospi->OSPI_DMACR |= SPI_DMACR_TDMAE;
}

/**
 * \fn          void ospi_enable_rx_dma(OSPI_Type *ospi)
 * \brief       Enable Tx DMA for the OSPI instance.
 * \param[in]   ospi       Pointer to the OSPI register map
 * \return      none
 */
static inline void ospi_enable_rx_dma(OSPI_Type *ospi)
{
	ospi->OSPI_DMACR |= SPI_DMACR_RDMAE;
}

/**
 * \fn          void ospi_disable_tx_dma(OSPI_Type *ospi)
 * \brief       Disable Tx DMA for the OSPI instance.
 * \param[in]   ospi       Pointer to the OSPI register map
 * \return      none
 */
static inline void ospi_disable_tx_dma(OSPI_Type *ospi)
{
	ospi->OSPI_DMACR &= ~SPI_DMACR_TDMAE;
}

/**
 * \fn          void ospi_disable_rx_dma(OSPI_Type *ospi)
 * \brief       Disable Tx DMA for the OSPI instance.
 * \param[in]   ospi       Pointer to the OSPI register map
 * \return      none
 */
static inline void ospi_disable_rx_dma(OSPI_Type *ospi)
{
	ospi->OSPI_DMACR &= ~SPI_DMACR_RDMAE;
}

/**
 * \fn          bool ospi_busy(OSPI_Type *ospi)
 * \brief       Get the busy status of the OSPI instance.
 * \param[in]   ospi       Pointer to the OSPI register map
 * \return      True/False based on the busy status
 */
static inline bool ospi_busy(OSPI_Type *ospi)
{
	return (ospi->OSPI_SR & (SPI_SR_BUSY | SPI_SR_TX_FIFO_EMPTY)) != SPI_SR_TX_FIFO_EMPTY;
}

/**
 * \fn          uint32_t ospi_get_dfs(OSPI_Type *ospi)
 * \brief       Get the data frame size for the OSPI instance.
 * \param[in]   ospi     Pointer to the OSPI register map
 * \return      current dfs configuration
 */
static inline uint32_t ospi_get_dfs(OSPI_Type *ospi)
{
	return (ospi->OSPI_CTRLR0 & SPI_CTRLR0_DFS_MASK) + 1;
}

/**
 * \fn          void ospi_set_mode(OSPI_Type *ospi, SPI_MODE mode)
 * \brief       Set the OSPI mode for the OSPI instance.
 * \param[in]   ospi     Pointer to the OSPI register map
 * \param[in]   mode    The mode to be set.
 * \return      none
 */
void ospi_set_mode(OSPI_Type *ospi, SPI_MODE mode);

/**
 * \fn          void ospi_set_dfs(OSPI_Type *ospi, uint8_t dfs)
 * \brief       Set the data frame size for the OSPI instance.
 * \param[in]   ospi    Pointer to the SPI register map
 * \param[in]   dfs     The data frame size
 * \return      none
 */
void ospi_set_dfs(OSPI_Type *ospi, uint8_t dfs);

/**
 * \fn          void ospi_set_tmode(OSPI_Type *ospi, SPI_TMOD tmod)
 * \brief       Set the transfer mode for the OSPI instance.
 * \param[in]   ospi    Pointer to the OSPI register map
 * \param[in]   tmod    Transfer mode
 * \return      none
 */
void ospi_set_tmod(OSPI_Type *ospi, SPI_TMOD tmod);

/**
 * \fn          void ospi_set_tx_threshold(OSPI_Type *ospi, uint8_t threshold)
 * \brief       Set Transmit FIFO interrupt threshold for the OSPI instance
 * \param[in]   ospi       Pointer to the OSPI register map
 * \param[in]   threshold  Transmit FIFO threshold
 * \return      none
 */
void ospi_set_tx_threshold(OSPI_Type *ospi, uint8_t threshold);

/**
 * \fn          void ospi_set_rx_threshold(OSPI_Type *ospi, uint8_t threshold)
 * \brief       Set Receive FIFO interrupt threshold for the OSPI instance
 * \param[in]   ospi       Pointer to the OSPI register map
 * \param[in]   threshold  Receive FIFO threshold
 * \return      none
 */
static inline void ospi_set_rx_threshold(OSPI_Type *ospi, uint8_t threshold)
{
	ospi->OSPI_RXFTLR = threshold;
}

/**
 * \fn          void ospi_set_tx_fifo_start_level(OSPI_Type *spi, uint16_t level)
 * \brief       Set Transmit FIFO start level
 * \param[in]   ospi   Pointer to the OSPI register map
 * \param[in]   level  Transmit FIFO start level
 * \return      none
 */
void ospi_set_tx_fifo_start_level(OSPI_Type *ospi, uint16_t level);

/**
 * \fn          void ospi_set_rx_sample_delay(OSPI_Type *ospi, uint8_t rx_sample_delay)
 * \brief       Set Receive sample delay for the OSPI instance
 * \param[in]   ospi       Pointer to the OSPI register map
 * \param[in]   threshold  Receive FIFO threshold
 * \return      none
 */
void ospi_set_rx_sample_delay(OSPI_Type *ospi, uint8_t rx_sample_delay);

/**
 * \fn          void ospi_set_ddr_drive_edge(OSPI_Type *ospi, uint8_t ddr_drive_edge)
 * \brief       Set DDR drive edge for the OSPI instance
 * \param[in]   ospi       Pointer to the OSPI register map
 * \param[in]   threshold  Receive FIFO threshold
 * \return      none
 */
void ospi_set_ddr_drive_edge(OSPI_Type *ospi, uint8_t ddr_drive_edge);

/**
 * \fn          void ospi_control_ss(OSPI_Type *ospi, uint8_t slave, SPI_SS_STATE state)
 * \brief       Control the slave select line
 * \param[in]   ospi   Pointer to the OSPI register map
 * \param[in]   slave  The slave to be selected
 * \param[in]   state  The state of the slave select line
 * \return      none
 */
void ospi_control_ss(OSPI_Type *ospi, uint8_t slave, SPI_SS_STATE state);

/**
 * \fn          void ospi_control_xip_ss(OSPI_Type *ospi, uint8_t slave, SPI_SS_STATE state)
 * \brief       Control the XIP slave select line
 * \param[in]   ospi   Pointer to the OSPI register map
 * \param[in]   slave  The slave to be selected
 * \param[in]   state  The state of the slave select line
 * \return      none
 */
void ospi_control_xip_ss(OSPI_Type *ospi, uint8_t slave, SPI_SS_STATE state);

/**
 * \fn          void ospi_send(OSPI_Type *spi, ospi_transfer_t *transfer)
 * \brief       Prepare the OSPI instance for transmission
 * \param[in]   ospi       Pointer to the OSPI register map
 * \param[in]   transfer   Transfer parameters
 * \return      none
 */
void ospi_send(OSPI_Type *ospi, ospi_transfer_t *transfer);

/**
 * \fn          void ospi_receive(OSPI_Type *ospi, ospi_transfer_t *transfer)
 * \brief       Prepare the OSPI instance for reception
 * \param[in]   ospi       Pointer to the OSPI register map
 * \param[in]   transfer   Transfer parameters
 * \return      none
 */
void ospi_receive(OSPI_Type *ospi, ospi_transfer_t *transfer);

/**
 * \fn          void ospi_transfer(OSPI_Type *spi, ospi_transfer_t *transfer)
 * \brief       Prepare the OSPI instance for transfer
 * \param[in]   ospi       Pointer to the OSPI register map
 * \param[in]   transfer   Transfer parameters
 * \return      none
 */
void ospi_transfer(OSPI_Type *ospi, ospi_transfer_t *transfer);

/**
 * \fn          void ospi_dma_send(OSPI_Type *spi, ospi_transfer_t *transfer)
 * \brief       Prepare the OSPI instance for transmission with DMA support
 * \param[in]   ospi       Pointer to the OSPI register map
 * \param[in]   transfer   Transfer parameters
 * \return      none
 */
void ospi_dma_send(OSPI_Type *ospi, ospi_transfer_t *transfer);

/**
 * \fn          void ospi_dma_transfer(OSPI_Type *spi, ospi_transfer_t *transfer)
 * \brief       Prepare the OSPI instance for transfer with DMA support
 * \param[in]   ospi       Pointer to the OSPI register map
 * \param[in]   transfer   Transfer parameters
 * \return      none
 */
void ospi_dma_transfer(OSPI_Type *ospi, ospi_transfer_t *transfer);

/**
 * \fn          void ospi_hyperbus_xip_init(OSPI_Type *ospi, uint8_t wait_cycles)
 * \brief       Initialize hyperbus XIP configuration for the OSPI instance
 * \param[in]   ospi        Pointer to the OSPI register map
 * \param[in]   wait_cycles Wait cycles needed by the hyperbus device
 * \return      none
 */
void ospi_hyperbus_xip_init(OSPI_Type *ospi, uint8_t wait_cycles);

/**
 * \fn          void ospi_irq_handler(OSPI_Type *ospi, ospi_transfer_t *transfer)
 * \brief       Handle interrupts for the OSPI instance.
 * \param[in]   ospi      Pointer to the OSPI register map
 * \param[in]   transfer  The transfer structure for the SPI instance
 * \return      none
 */
void ospi_irq_handler(OSPI_Type *ospi, ospi_transfer_t *transfer);

#ifdef __cplusplus
}
#endif

#endif /* _ALIF_HAL_OSPI_H_ */
