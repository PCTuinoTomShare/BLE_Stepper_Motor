
// Stepper motor control via Bluetooth LE, test source code. 
// by PCT / tom hsieh.
// Board : Arduino M0 PRO ( MCU : ATSAMD21G18A, ARM Cortex M0+ ). 
// IDE : Arduino 1.7.10 ( Arduino.ORG ).

#define ANDUINO_UART 1 // Use Arduino UART interface.
//#define EDBG_CDC 1   // USE on board Atmel EDBG CDC port ( for test only ).

// Connect to Microchip RN4020 bluetooth LE module, preseting :
// - buadrate 9600pbs.
// - Peripheral.
// - Support MLDP.
#define MLDP     13  // RN4020 MLDP control pin.
#define HW_WAKE  12  // RN4020 hardware wake control pin.
#define SW_WAKE  11  // RN4020 software wake control pin.
// Connect to Microchip MTS62C19A ( pin compatible L6219 ) stepper motor driver IC.
#define PHASE1  2    // MTS62C19A ( L6219 ) stepper motor phase 1 pin.
#define PHASE2  3    // MTS62C19A ( L6219 ) stepper motor phase 2 pin.
#define I01 4        // MTS62C19A ( L6219 ) I01 pin.
#define I11 5        // MTS62C19A ( L6219 ) I11 pin.
#define I02 6        // MTS62C19A ( L6219 ) I02 pin.
#define I12 7        // MTS62C19A ( L6219 ) I12 pin. 
// Test only.
#define TEST 8       // Test output pin. 

// Motor speed pulse width table.
const unsigned short pulse_motor_table[8]={
    2000, // 2ms,       500pps.  // Motor do not rotation, because installed gearbox. 
    1400, // 1.4ms,     700pps.  // Motor do not rotation, because installed gearbox.
    1100, // 1.1ms,     900pps.  // Motor do not rotation, because installed gearbox.
    900,  // 0.9ms,     1100pps. // Motor do not rotation, because installed gearbox.
    769,  // 0.769ms,   1300pps. // Motor do not rotation, because installed gearbox.
    666,  // 0.666ms,   1500pps  // Motor do not rotation, because installed gearbox.
    4000, // 4ms,       250pps.
    10000 // 10ms,      100pps.
};

// Motor status flag.
// - bit #0, running busy.
unsigned char motor_flag;          
unsigned char motor_phase_index;   // Motor phase index.
unsigned char motor_speed_index;   // Motor speed index.
unsigned char uart_rec_data[16];   // UART received data.
unsigned char uart_rec_cnt;        // UART received data counter.
unsigned char uart_rec_to;         // UART received timeout.
unsigned char temp2;               // Temporary #2. 
unsigned char mldp_keep_cnt;	   // MLDP signal hold counter.	
unsigned char motor_dir;		   // Motor rotation direction.	
unsigned char motor_dir_dest;      // Motor rotation direction destination.

unsigned short motor_step_cnt;     // Motor step counter.
unsigned short motor_step_cnt_dest;// Motor step count value distanation.
unsigned short temp1;              // Temporary #1.

unsigned long us_cur;              // uS timer counter current value.
unsigned long us_past_motor;       // uS past count, for motor speed. 
unsigned long us_pre_motor;        // uS timer counter previous value, for motor speed.
unsigned long motor_speed;         // Motor speed.
unsigned long ms_cur;              // MS timer counter current value.
unsigned long ms_past_10ms;        // 10ms past count.
unsigned long ms_pre_10ms;         // 10ms timer counter previous value.

void setup() {
  // put your setup code here, to run once:
  // IO port initialize.
  pinMode( MLDP, OUTPUT );
  pinMode( HW_WAKE, OUTPUT );
  pinMode( SW_WAKE, OUTPUT );  
  pinMode( PHASE1, OUTPUT );
  pinMode( PHASE2, OUTPUT );
  pinMode( I01, OUTPUT );
  pinMode( I02, OUTPUT );
  pinMode( I11, OUTPUT );
  pinMode( I12, OUTPUT );
  //pinMode( TEST, OUTPUT );
  // Serial port.
  #ifdef ANDUINO_UART 
    Serial5.begin( 9600 ); // Arduino UART interface.  
  #elif EDBG_CDC  
    Serial.begin( 9600 );  // Atmel EDBG CDC port. 
  #endif 
  // RN4020 control.
  digitalWrite( MLDP, LOW );        // Command mode.
  digitalWrite( HW_WAKE, HIGH );    // Hardware wake.
  digitalWrite( SW_WAKE, HIGH );    // Software wake.
  //digitalWrite( TEST, LOW );  
  // Clear motor phase index.
  motor_phase_index = 0;
  // Motor direction.
  motor_dir = 0;
  // Initialize motor speed value.
  motor_speed_index = 7;
  motor_speed = pulse_motor_table[7];
  // Flag reset.
  motor_flag = 0;
  // Motor off.
  Motor_Off();  
}

void loop() {
  // put your main code here, to run repeatedly:
  // Get current millisecon timer count value.
  ms_cur = millis();
  // Calculate interval time past.
  // 10ms past.
  ms_past_10ms = ms_cur;
  ms_past_10ms -= ms_pre_10ms;
  // 10ms past check.
  if( ms_past_10ms > 9 ){   
    // About 10ms past. 
    // Keep current counter value as previous. 
     ms_pre_10ms = ms_cur;
     // Signal keep.
     if( mldp_keep_cnt ){
        --mldp_keep_cnt;
     }
     else{
       // MLDP output low for reset.
       digitalWrite( MLDP, LOW );       
     }
     // Timeout check.
     if( uart_rec_cnt ){
        // Increase timeout counter.
        ++uart_rec_to;        
        if( uart_rec_to == 10 ){
            // Clear count.
            uart_rec_cnt = 0;
            uart_rec_to = 0;
            // MLDP output high for reset.
            digitalWrite( MLDP, HIGH );
            // Set keep counter.
            mldp_keep_cnt = 2;
        } 
     }
  }
  // Get current microsecond timer count value.
  us_cur = micros();
  // Motor speed count.
  us_past_motor = us_cur;
  us_past_motor -= us_pre_motor;
  if( us_past_motor > motor_speed ){
    // Stepper motor task.
    Motor_Run();
    // Keep current counter value as previous.
    us_pre_motor = us_cur;   
  } 
  // UART received.
  #ifdef ANDUINO_UART
  if( Serial5.available() ){ // From Arduino UART interface.
  #elif EDBG_CDC
  if( Serial.available() ){  // From Atmel EDBG CDC port.
  #endif  
    // Clear timeout counter.
    uart_rec_to = 0;
    // Keep received data. 
    #ifdef ANDUINO_UART
	uart_rec_data[ uart_rec_cnt ] = Serial5.read(); // From Arduino UART interface.
    #elif EDBG_CDC
	uart_rec_data[ uart_rec_cnt ] = Serial.read(); // From Atmel EDBG CDC port.
    #endif
    // For next data.
    ++uart_rec_cnt;    
    if( uart_rec_cnt == 15 ){      
      // Received data check.
      UART_Rec_Check();      
    }    
  }
}

// Motor off.
void Motor_Off( void ){
  digitalWrite( I01, HIGH );
  digitalWrite( I02, HIGH );
  digitalWrite( I11, HIGH );
  digitalWrite( I12, HIGH );
}

// Motor on.
void Motor_On( void ){
  digitalWrite( I01, LOW );
  digitalWrite( I02, LOW );
  digitalWrite( I11, LOW );
  digitalWrite( I12, LOW );  
}

// Motor phase 1.
void Motor_Phase1( void ){
  digitalWrite( PHASE1, HIGH );
  digitalWrite( PHASE2, LOW );  
}

// Motor phase 2.
void Motor_Phase2( void ){
  digitalWrite( PHASE1, HIGH );
  digitalWrite( PHASE2, HIGH );  
}

// Motor phase 3.
void Motor_Phase3( void ){
  digitalWrite( PHASE1, LOW );
  digitalWrite( PHASE2, HIGH );  
}

// Motor phase 3.
void Motor_Phase4( void ){
  digitalWrite( PHASE1, LOW );
  digitalWrite( PHASE2, LOW );  
}

// Motor run.
void Motor_Run( void ){
 
 if( motor_step_cnt == 0 ){
   // Motor off.
   Motor_Off();
   // Update step counter. 
   motor_step_cnt = motor_step_cnt_dest;
   motor_step_cnt_dest = 0;
   // Update rotation direction.
   motor_dir = motor_dir_dest;
   // Update speed.
   motor_speed = pulse_motor_table[ motor_speed_index ];
   // Flag bit clear, motor off.
   motor_flag = 0;   
   return;
 }
 // Flag bit set, motor on.
 motor_flag |= 0x01;     
 // Motor on.
 Motor_On();
 // motor direction.
 // CW
 if( motor_dir ){ 
   ++motor_phase_index; 
   if( motor_phase_index == 4 ){
      motor_phase_index = 0;
   } 
 }
 // CCW
 else{
   if( motor_phase_index == 0 ){
      motor_phase_index = 4;
   }
   --motor_phase_index;  
 }
 // Phase control out.
 switch( motor_phase_index ){
   case 0:
       Motor_Phase1();
       break;
   case 1:
       Motor_Phase2();
       break;
   case 2:
       Motor_Phase3();
       break;
   case 3:
       Motor_Phase4();
       break;
 }
 // Descrease step counter.
 --motor_step_cnt;
}
  
// UART received check.
void UART_Rec_Check( void ){
  // Received :
  // uart_rec_data[ 0 ~ 5 ] = " MDLP\r\n", useless. 
  // uart_rec_data[6], start byte #1, must be '<'.
  // uart_rec_data[7], start byte #2, must be '='.
  // uart_rec_data[8], rotation direction, '0' for CCW otherwise CW. 
  // uart_rec_data[9], rotation speed, '0' for 100pps otherwise 250pps.
  // uart_rec_data[10], step count value 10^3, range '0' ~'9'.
  // uart_rec_data[11], step count value 10^2, range '0' ~'9'.
  // uart_rec_data[12], step count value 10^1, range '0' ~'9'.
  // uart_rec_data[13], step count value 10^0, range '0' ~'9'.
  // uart_rec_data[14], end byte, must be '>'.
  // Return:
  // byte #1, '0' motor idle, '1' motor busy.
  // byte #2, '\r',
  // byte #3, '\n'.
  
  // Clear counter.
  uart_rec_cnt = 0;
  // Check start byte #1 - '<'.
  if( uart_rec_data[6] != 0x3c ){
    // MLDP output high for reset.
    digitalWrite( MLDP, HIGH );
    // Set keep counter.
    mldp_keep_cnt = 2;
    return;
  }	
  // Check start byte #2 - '='.
  if( uart_rec_data[7] != 0x3d ){
    // MLDP output high for reset.
    digitalWrite( MLDP, HIGH );
    // Set keep counter.
    mldp_keep_cnt = 2;
    return;	
  }	
  // Check end byte - '>'.
  if( uart_rec_data[14] != 0x3e ){
    // MLDP output high for reset.
    digitalWrite( MLDP, HIGH );
    // Set keep counter.
    mldp_keep_cnt = 2;
    return;
  }
  
  //digitalWrite( TEST, HIGH );

  // Rotation direction.
  temp2 = uart_rec_data[8];
  temp2 &= 0x0f;
  if( temp2 > 1 ){
    temp2 = 1;
  }
  motor_dir_dest = temp2;  
  // Speed
  temp2 = uart_rec_data[9];
  temp2 &= 0x0f;
  if( temp2 == 0 ){
    temp2 = 7;
  }
  else{
    temp2 = 6;
  }
  
  motor_speed_index = temp2;
  // Motor step count.
  // 10^3.
  temp1 = uart_rec_data[10];
  temp1 &= 0x000f;
  if( temp1 > 9 ){
    temp1 = 0;
  }  
  temp1 *= 1000;
  motor_step_cnt_dest = temp1;
  // 10^2.
  temp2 = uart_rec_data[11];
  temp2 &= 0x0f;
  if( temp2 > 9 ){
    temp2 = 0;
  }
  temp2 *= 100;
  motor_step_cnt_dest += temp2;
  // 10^1.
  temp2 = uart_rec_data[12];
  temp2 &= 0x0f;
  if( temp2 > 9 ){
    temp2 = 0;
  }
  temp2 *= 10;
  motor_step_cnt_dest += temp2;
  // 10^0.
  temp2 = uart_rec_data[13];
  temp2 &= 0x0f;
  if( temp2 > 9 ){
    temp2 = 0;
  }
  motor_step_cnt_dest += temp2;
  // Return motor status.
  temp2 = motor_flag;
  temp2 += 0x30;     
  #ifdef ANDUINO_UART	
    Serial5.write( temp2 ); // To Arduino UART port.
    Serial5.write( 0x0d );
    Serial5.write( 0x0a );	
  #elif EDBG_CDC
    Serial.write( temp2 ); // To Atmel EDBG CDC port.
    Serial.write( 0x0d );
    Serial.write( 0x0a );	
  #endif
  delay(10);
  // MLDP output high for reset.
  digitalWrite( MLDP, HIGH );
  // Set keep counter.
  mldp_keep_cnt = 2;
}

