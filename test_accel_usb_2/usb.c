			/*
    if (TM_USB_VCP_GetStatus() == TM_USB_VCP_CONNECTED) {
            /* Turn on GREEN led */
					TM_DISCO_LedOn(LED_GREEN);
            /* If something arrived at VCP */
    if (TM_USB_VCP_Getc(&c) == TM_USB_VCP_DATA_OK) {
                /* Return data back */
             
							//sprintf(bufor,"%f",liczba);
							if(c==0x31){TM_DISCO_LedOn(LED_RED);
								
								for( i=0;i<4;i++) {TM_USB_VCP_Putc(liczba_q0[i]);}
								for( i=0;i<4;i++) {TM_USB_VCP_Putc(liczba_q1[i]);}					
								for( i=0;i<4;i++) {TM_USB_VCP_Putc(liczba_q2[i]);}								
								for( i=0;i<4;i++) {TM_USB_VCP_Putc(liczba_q3[i]);}								
							}
            }
        } else {
       */
            TM_DISCO_LedOff(LED_GREEN);
        }
			TM_DISCO_LedOff(LED_RED); 