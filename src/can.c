/*

The MIT License (MIT)

Copyright (c) 2016 Hubert Denkmair

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include "can.h"
#include "config.h"

#ifdef STM32G4
static FDCAN_HandleTypeDef can_handle;
#endif

static uint8_t can_enabled = 0;

void can_init(can_data_t *hcan)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();

	#ifdef STM32G4
		__HAL_RCC_FDCAN_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
	    __HAL_RCC_GPIOC_CLK_ENABLE();

    	GPIO_InitTypeDef GPIO_InitStruct;

        GPIO_InitStruct.Pin = GPIO_PIN_0;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, 0); // CAN Standby - turn standby off (hw pull hi)


        GPIO_InitStruct.Pin = GPIO_PIN_13;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1); // CAN IO power



	    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	    // Set some reasonable defaults
		hcan->instance   = FDCAN1;
		hcan->brp        = 6;
		hcan->phase_seg1 = 13;
		hcan->phase_seg2 = 2;
		hcan->sjw        = 1;


	#else
		__HAL_RCC_CAN1_CLK_ENABLE();


		GPIO_InitTypeDef itd;
		itd.Pin = GPIO_PIN_8|GPIO_PIN_9;
		itd.Mode = GPIO_MODE_AF_PP;
		itd.Pull = GPIO_NOPULL;
		itd.Speed = GPIO_SPEED_FREQ_HIGH;
		itd.Alternate = GPIO_AF4_CAN;
		HAL_GPIO_Init(GPIOB, &itd);

		hcan->instance   = CAN;
		hcan->brp        = 6;
		hcan->phase_seg1 = 13;
		hcan->phase_seg2 = 2;
		hcan->sjw        = 1;
	#endif


}



bool can_set_bittiming(can_data_t *hcan, uint32_t brp, uint32_t phase_seg1, uint32_t phase_seg2, uint32_t sjw)
{
	if ( (brp>=CAN_BRP_MIN) && (brp<=CAN_BRP_MAX)
	  && (phase_seg1>=CAN_TSEG1_MIN) && (phase_seg1<=CAN_TSEG1_MAX)
	  && (phase_seg2>=CAN_TSEG2_MIN) && (phase_seg2<=CAN_TSEG2_MAX)
	  && (sjw>0) && (sjw<=CAN_SJW_MAX)
	) {
		hcan->brp = brp & 0x3FF; // FIXME: Mask for 1024. Set to 512 for BxCAN? Do we need to mask at all??
		hcan->phase_seg1 = phase_seg1;
		hcan->phase_seg2 = phase_seg2;
		hcan->sjw = sjw;
		return true;
	} else {
		return false;
	}
}

void can_enable(can_data_t *hcan, bool loop_back, bool listen_only, bool one_shot)
{
	can_enabled = 1;
	TARGET_CAN_TYPEDEF *can = hcan->instance;

#ifdef STM32G4

	// Stop the CAN interface
//    HAL_FDCAN_Stop(&can_handle);

	can_handle.Instance = FDCAN1;


    can_handle.Init.ClockDivider = FDCAN_CLOCK_DIV1; // 144Mhz
    can_handle.Init.FrameFormat = FDCAN_FRAME_CLASSIC;


	// Default to normal mode
    can_handle.Init.Mode = FDCAN_MODE_NORMAL;

//	// Loopback
//	if(loop_back)
//		can_handle.Init.Mode == FDCAN_MODE_INTERNAL_LOOPBACK;
//
//	// Listen only
//	if(listen_only)
//		can_handle.Init.Mode = FDCAN_MODE_BUS_MONITORING; // bus monitoring

	// Auto retransmit
	can_handle.Init.AutoRetransmission = !one_shot; // TODO: check polarity

    can_handle.Init.TransmitPause = DISABLE; // emz
    can_handle.Init.ProtocolException = DISABLE; // emz


    // Pull values from RAM structure
	can_handle.Init.NominalPrescaler = hcan->brp;
	can_handle.Init.NominalTimeSeg1 = hcan->phase_seg1;
	can_handle.Init.NominalTimeSeg2 = hcan->phase_seg2;
	can_handle.Init.NominalSyncJumpWidth = hcan->sjw;

    // FD only??
    can_handle.Init.DataPrescaler = 1;
    can_handle.Init.DataSyncJumpWidth = 1;
    can_handle.Init.DataTimeSeg1 = 1;
    can_handle.Init.DataTimeSeg2 = 1;

    can_handle.Init.StdFiltersNbr = 0;
    can_handle.Init.ExtFiltersNbr = 0;
    can_handle.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;


    volatile HAL_StatusTypeDef res1 = HAL_FDCAN_Init(&can_handle);
    //HAL_CAN_ConfigFilter(&can_handle, &filter);
    volatile HAL_StatusTypeDef res2 = HAL_FDCAN_Start(&can_handle);

    volatile uint32_t bkpt = 1235;
#else

	uint32_t mcr = CAN_MCR_INRQ
				 | CAN_MCR_ABOM
				 | CAN_MCR_TXFP
				 | (one_shot ? CAN_MCR_NART : 0);

	uint32_t btr = ((uint32_t)(hcan->sjw-1)) << 24
				 | ((uint32_t)(hcan->phase_seg1-1)) << 16
				 | ((uint32_t)(hcan->phase_seg2-1)) << 20
				 | (hcan->brp - 1)
				 | (loop_back ? CAN_MODE_LOOPBACK : 0)
				 | (listen_only ? CAN_MODE_SILENT : 0);


	// Reset CAN peripheral
	can->MCR |= CAN_MCR_RESET;
	while((can->MCR & CAN_MCR_RESET) != 0); // reset bit is set to zero after reset
	while((can->MSR & CAN_MSR_SLAK) == 0);  // should be in sleep mode after reset

	can->MCR |= CAN_MCR_INRQ ;
	while((can->MSR & CAN_MSR_INAK) == 0);

	can->MCR = mcr;
	can->BTR = btr;

	can->MCR &= ~CAN_MCR_INRQ;
	while((can->MSR & CAN_MSR_INAK) != 0);

	uint32_t filter_bit = 0x00000001;
	can->FMR |= CAN_FMR_FINIT;
	can->FMR &= ~CAN_FMR_CAN2SB;
	can->FA1R &= ~filter_bit;        // disable filter
	can->FS1R |= filter_bit;         // set to single 32-bit filter mode
	can->FM1R &= ~filter_bit;        // set filter mask mode for filter 0
	can->sFilterRegister[0].FR1 = 0; // filter ID = 0
	can->sFilterRegister[0].FR2 = 0; // filter Mask = 0
	can->FFA1R &= ~filter_bit;       // assign filter 0 to FIFO 0
	can->FA1R |= filter_bit;         // enable filter
	can->FMR &= ~CAN_FMR_FINIT;


#endif


#ifdef nCANSTBY_Pin
	HAL_GPIO_WritePin(nCANSTBY_Port, nCANSTBY_Pin, GPIO_PIN_SET);
#endif
}

void can_disable(can_data_t *hcan)
{
	can_enabled = 0;

	TARGET_CAN_TYPEDEF *can = hcan->instance;

#ifdef nCANSTBY_Pin
	HAL_GPIO_WritePin(nCANSTBY_Port, nCANSTBY_Pin, GPIO_PIN_RESET);
#endif

#ifdef STM32G4
	// Stop the CAN interface
    HAL_FDCAN_Stop(&can_handle);
#else
	can->MCR |= CAN_MCR_INRQ ; // send can controller into initialization mode
#endif
}

bool can_is_enabled(can_data_t *hcan)
{
	TARGET_CAN_TYPEDEF *can = hcan->instance;

#ifdef STM32G4
	return can_handle.State > HAL_FDCAN_STATE_RESET;
#else
	return (can->MCR & CAN_MCR_INRQ) == 0;
#endif
}

bool can_is_rx_pending(can_data_t *hcan)
{
	if(can_enabled == 0)
		return false;

	TARGET_CAN_TYPEDEF *can = hcan->instance;

#ifdef STM32G4
	return HAL_FDCAN_GetRxFifoFillLevel(&can_handle, FDCAN_RX_FIFO0) > 0;
#else
	return ((can->RF0R & CAN_RF0R_FMP0) != 0);
#endif
}

bool can_receive(can_data_t *hcan, struct gs_host_frame *rx_frame)
{
	TARGET_CAN_TYPEDEF *can = hcan->instance;

#ifdef STM32G4

	if (can_is_rx_pending(hcan))
	{
		uint32_t status;
	    FDCAN_RxHeaderTypeDef header;
	    uint8_t data[8];
	    status = HAL_FDCAN_GetRxMessage(&can_handle, FDCAN_RX_FIFO0, &header, data);


		if(header.IdType == FDCAN_EXTENDED_ID)
		{
			rx_frame->can_id = CAN_EFF_FLAG | (header.Identifier & 0x1FFFFFFF);
		}
		else
		{
			rx_frame->can_id = (header.Identifier & 0x7FF);
		}

		if(header.RxFrameType == FDCAN_REMOTE_FRAME)
			rx_frame->can_id |= CAN_RTR_FLAG;


		// FIXME: This is not real datalength, it is a code
		rx_frame->can_dlc = (header.DataLength >> 16);

		rx_frame->data[0] = data[0];
		rx_frame->data[1] = data[1];
		rx_frame->data[2] = data[2];
		rx_frame->data[3] = data[3];
		rx_frame->data[4] = data[4];
		rx_frame->data[5] = data[5];
		rx_frame->data[6] = data[6];
		rx_frame->data[7] = data[7];
		return true;
	} else {
		return false;
	}

#else
	if (can_is_rx_pending(hcan)) {
		CAN_FIFOMailBox_TypeDef *fifo = &can->sFIFOMailBox[0];

		if (fifo->RIR &  CAN_RI0R_IDE) {
			rx_frame->can_id = CAN_EFF_FLAG | ((fifo->RIR >> 3) & 0x1FFFFFFF);
		} else {
			rx_frame->can_id = (fifo->RIR >> 21) & 0x7FF;
		}

		if (fifo->RIR & CAN_RI0R_RTR)  {
			rx_frame->can_id |= CAN_RTR_FLAG;
		}

		rx_frame->can_dlc = fifo->RDTR & CAN_RDT0R_DLC;

		rx_frame->data[0] = (fifo->RDLR >>  0) & 0xFF;
		rx_frame->data[1] = (fifo->RDLR >>  8) & 0xFF;
		rx_frame->data[2] = (fifo->RDLR >> 16) & 0xFF;
		rx_frame->data[3] = (fifo->RDLR >> 24) & 0xFF;
		rx_frame->data[4] = (fifo->RDHR >>  0) & 0xFF;
		rx_frame->data[5] = (fifo->RDHR >>  8) & 0xFF;
		rx_frame->data[6] = (fifo->RDHR >> 16) & 0xFF;
		rx_frame->data[7] = (fifo->RDHR >> 24) & 0xFF;

		can->RF0R |= CAN_RF0R_RFOM0; // release FIFO

		return true;
	} else {
		return false;
	}
#endif
}

#ifndef STM32G4
static CAN_TxMailBox_TypeDef *can_find_free_mailbox(can_data_t *hcan)
{
       CAN_TypeDef *can = hcan->instance;

       uint32_t tsr = can->TSR;
       if ( tsr & CAN_TSR_TME0 ) {
               return &can->sTxMailBox[0];
       } else if ( tsr & CAN_TSR_TME1 ) {
               return &can->sTxMailBox[1];
       } else if ( tsr & CAN_TSR_TME2 ) {
               return &can->sTxMailBox[2];
       } else {
               return 0;
       }
}
#endif

bool can_send(can_data_t *hcan, struct gs_host_frame *frame)
{
	if(!can_enabled)
		return;

#ifdef STM32G4

    uint32_t status;

	FDCAN_TxHeaderTypeDef frame_header =
	{
		.TxFrameType = FDCAN_DATA_FRAME,
		.FDFormat = FDCAN_CLASSIC_CAN, // default to classic frame
		.IdType = FDCAN_STANDARD_ID, // default to standard ID
		.BitRateSwitch = FDCAN_BRS_OFF, // no bitrate switch
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE, // error active
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS, // don't record tx events
        .MessageMarker = 0, // ?
	};
	uint8_t data[8] = {0};

	if(frame->can_id & CAN_EFF_FLAG)
		frame_header.IdType = FDCAN_EXTENDED_ID;

	if(frame->can_id & CAN_RTR_FLAG)
		frame_header.TxFrameType = FDCAN_REMOTE_FRAME;

	 // Convert to HAL code
    frame_header.DataLength = frame->can_dlc << 16;

    // Copy data
    for(uint8_t i=0; i<frame->can_dlc; i++)
    {
    	data[i] = frame->data[i];
    }

    // Transmit can frame
    status = HAL_FDCAN_AddMessageToTxFifoQ(&can_handle, &frame_header, data);

    if(status == HAL_OK)
    	return true;
    else
    	return false;

#else
	CAN_TxMailBox_TypeDef *mb = can_find_free_mailbox(hcan);
	if (mb != 0) {

		/* first, clear transmission request */
		mb->TIR &= CAN_TI0R_TXRQ;

		if (frame->can_id & CAN_EFF_FLAG) { // extended id
			mb->TIR = CAN_ID_EXT | (frame->can_id & 0x1FFFFFFF) << 3;
		} else {
			mb->TIR = (frame->can_id & 0x7FF) << 21;
		}

		if (frame->can_id & CAN_RTR_FLAG) {
			mb->TIR |= CAN_RTR_REMOTE;
		}

		mb->TDTR &= 0xFFFFFFF0;
		mb->TDTR |= frame->can_dlc & 0x0F;

		mb->TDLR =
			  ( frame->data[3] << 24 )
			| ( frame->data[2] << 16 )
			| ( frame->data[1] <<  8 )
			| ( frame->data[0] <<  0 );

		mb->TDHR =
			  ( frame->data[7] << 24 )
			| ( frame->data[6] << 16 )
			| ( frame->data[5] <<  8 )
			| ( frame->data[4] <<  0 );

		/* request transmission */
		mb->TIR |= CAN_TI0R_TXRQ;

		return true;
	} else {
		return false;
	}
#endif
}

uint32_t can_get_error_status(can_data_t *hcan)
{
	// FIXME: Add support for error status on G4

	TARGET_CAN_TYPEDEF *can = hcan->instance;
//	return can->ESR;
}

static bool status_is_active(uint32_t err)
{
	// FIXME: Add support for error status on G4

//	return !(err & (CAN_ESR_BOFF | CAN_ESR_EPVF));
}

bool can_parse_error_status(uint32_t err, uint32_t last_err, can_data_t *hcan, struct gs_host_frame *frame)
{

#ifdef STM32G4

// FIXME: Add support for error status on G4
return false;

#else
	/* We build up the detailed error information at the same time as we decide
	 * whether there's anything worth sending. This variable tracks that final
	 * result. */
	bool should_send = false;

	frame->echo_id = 0xFFFFFFFF;
	frame->can_id  = CAN_ERR_FLAG | CAN_ERR_CRTL;
	frame->can_dlc = CAN_ERR_DLC;
	frame->data[0] = CAN_ERR_LOSTARB_UNSPEC;
	frame->data[1] = CAN_ERR_CRTL_UNSPEC;
	frame->data[2] = CAN_ERR_PROT_UNSPEC;
	frame->data[3] = CAN_ERR_PROT_LOC_UNSPEC;
	frame->data[4] = CAN_ERR_TRX_UNSPEC;
	frame->data[5] = 0;
	frame->data[6] = 0;
	frame->data[7] = 0;

	if (err & CAN_ESR_BOFF) {
		frame->can_id |= CAN_ERR_BUSOFF;
		if (!(last_err & CAN_ESR_BOFF)) {
			/* We transitioned to bus-off. */
			should_send = true;
		}
	} else if (last_err & CAN_ESR_BOFF) {
		/* We transitioned out of bus-off. */
		should_send = true;
	}

	/* We transitioned from passive/bus-off to active, so report the edge. */
	if (!status_is_active(last_err) && status_is_active(err)) {
		frame->data[1] |= CAN_ERR_CRTL_ACTIVE;
		should_send = true;
	}

	uint8_t tx_error_cnt = (err>>16) & 0xFF;
	uint8_t rx_error_cnt = (err>>24) & 0xFF;
	/* The Linux sja1000 driver puts these counters here. Seems like as good a
	 * place as any. */
	frame->data[6] = tx_error_cnt;
	frame->data[7] = rx_error_cnt;

	uint8_t last_tx_error_cnt = (last_err>>16) & 0xFF;
	uint8_t last_rx_error_cnt = (last_err>>24) & 0xFF;
	/* If either error counter transitioned to/from 0. */
	if ((tx_error_cnt == 0) != (last_tx_error_cnt == 0)) {
		should_send = true;
	}
	if ((rx_error_cnt == 0) != (last_rx_error_cnt == 0)) {
		should_send = true;
	}
	/* If either error counter increased by 15. */
	if (((int)last_tx_error_cnt + CAN_ERRCOUNT_THRESHOLD) < tx_error_cnt) {
		should_send = true;
	}
	if (((int)last_rx_error_cnt + CAN_ERRCOUNT_THRESHOLD) < rx_error_cnt) {
		should_send = true;
	}

	if (err & CAN_ESR_EPVF) {
		frame->data[1] |= CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE;
		if (!(last_err & CAN_ESR_EPVF)) {
			should_send = true;
		}
	} else if (err & CAN_ESR_EWGF) {
		frame->data[1] |= CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING;
		if (!(last_err & CAN_ESR_EWGF)) {
			should_send = true;
		}
	} else if (last_err & (CAN_ESR_EPVF | CAN_ESR_EWGF)) {
		should_send = true;
	}

	uint8_t lec = (err>>4) & 0x07;
	switch (lec) {
		case 0x01: /* stuff error */
			frame->can_id |= CAN_ERR_PROT;
			frame->data[2] |= CAN_ERR_PROT_STUFF;
			should_send = true;
			break;
		case 0x02: /* form error */
			frame->can_id |= CAN_ERR_PROT;
			frame->data[2] |= CAN_ERR_PROT_FORM;
			should_send = true;
			break;
		case 0x03: /* ack error */
			frame->can_id |= CAN_ERR_ACK;
			should_send = true;
			break;
		case 0x04: /* bit recessive error */
			frame->can_id |= CAN_ERR_PROT;
			frame->data[2] |= CAN_ERR_PROT_BIT1;
			should_send = true;
			break;
		case 0x05: /* bit dominant error */
			frame->can_id |= CAN_ERR_PROT;
			frame->data[2] |= CAN_ERR_PROT_BIT0;
			should_send = true;
			break;
		case 0x06: /* CRC error */
			frame->can_id |= CAN_ERR_PROT;
			frame->data[3] |= CAN_ERR_PROT_LOC_CRC_SEQ;
			should_send = true;
			break;
		default: /* 0=no error, 7=no change */
			break;
	}

	CAN_TypeDef *can = hcan->instance;
	/* Write 7 to LEC so we know if it gets set to the same thing again */
	can->ESR = 7<<4;

	return should_send;
#endif
}
