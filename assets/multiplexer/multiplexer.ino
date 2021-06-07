#define ERR_VAL 10

volatile unsigned long prev_tim[2] = {0, 0};
volatile unsigned long pul_tim[2] = {1500, 1500};

bool ok;

String buf = "";
int arr[2] = {1500, 1500};

int V[2] = {1500, 1500};

void setup() 
{
  
  //Configuring pins 9 & 10 as output pins.
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  
  //Setting pins 9 & 10 to fast PWM mode with time period 10ms.
  TCCR1A = 0;
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);  
  TCCR1B = 0;
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); 
  ICR1 = 9999;
  
  //Begin serial communication.
  Serial.begin(57600);
  Serial.setTimeout(100);
  
  //Configuring pins 2, 3 & 4 as input pins.
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  
  //Rising edge of INT0 & INT1 generates interrupts.
  EICRA |= (1 << ISC00) | (1 << ISC01) | (1 << ISC10) | (1 << ISC11);
  
  //Enable interrupts for INT0 & INT1.
  EIMSK |= (1 << INT0) | (1 << INT1);

  //Enable global interrupts
  sei();
}

void loop() 
{

  ok = (pulseIn(4, HIGH, 20000) > 1800); //Reading if pulse of CH3 > 1800
  if(ok)
  {
    while(Serial.available() > 0) 
    {
      char temp = Serial.read();
      
      if(isDigit(temp))
      {
        buf += temp;
      }
      else if(temp == 'D')
      {
        arr[0] = buf.toInt();
        buf = "";
      }
      else if(temp == 'T')
      {
        arr[1] = buf.toInt();
        buf = "";
      }
      
      if(temp == '\n') buf = "";
    }
    OCR1A = correct(arr[0]);
    OCR1B = correct(arr[1]);
  }
  else
  {
    if(abs(V[0] - pul_tim[0]) > ERR_VAL)
    {
      V[0] = pul_tim[0];
    }
    if(abs(V[1] - pul_tim[1]) > ERR_VAL)
    {
      V[1] = pul_tim[1];
    }
    OCR1A = correct(V[0]);
    OCR1B = correct(V[1]);
  }
}

int correct(int Val)
{
  if(ok)
  {
    if(Val < 1000 || Val > 2000) Val = 1500;
  }
  else
  {
    if(Val < 1000) Val = 1000;
    else if(Val > 2000) Val = 2000;
  }
  return Val;
}

//Interrupt service routine
ISR(INT0_vect)
{

  EIMSK &= ~(1 << INT0);  //Disable INT0
  
  if((EICRA & (1 << ISC00)) == (1 << ISC00))
  {
    prev_tim[0] = micros();
  }
  else
  {
    pul_tim[0] = micros() - prev_tim[0];
  }
  
  EICRA ^= (1 << ISC00);  //Toggle triggering.
  EIMSK |= (1 << INT0);   //Enable INT0
}

ISR(INT1_vect)
{
  
  EIMSK &= ~(1 << INT1);  //Disable INT1
  
  if((EICRA & (1 << ISC10)) == (1 << ISC10))
  {
    prev_tim[1] = micros();
  }
  else
  {
    pul_tim[1] = micros() - prev_tim[1];
  }
  
  EICRA ^= (1 << ISC10);  //Toggle triggering.
  EIMSK |= (1 << INT1);   //Enable INT1
}
