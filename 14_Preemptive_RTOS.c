// RTOS Framework - Fall 2017
// J Losh

// Student Name: Disha Shah
// TO DO: Add your name on this line.  Do not include your ID number.

// Submit only two .c files in an e-mail to me (not in a compressed file):
// 14_rtos_coop.c   Single-file with cooperative version of your project
// 14_rtos_prempt.c Single-file with premptive version of your project
// (xx is a unique number that will be issued in class)
// Please do not include .intvecs section in your code submissions
// Please do not change any function names in this code or thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------
// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz
// Hardware configuration:
// 5 Pushbuttons and 5 LEDs, UART

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include <string.h>
#include <stdlib.h>
// REQUIRED: correct these bitbanding references for the off-board LEDs - DONE
#define BLUE_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED PF2
#define RED_LED     (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4))) // off-board red LED PA5
#define ORANGE_LED  (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4))) // off-board orange LED PE4
#define YELLOW_LED  (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 4*4))) // off-board yellow LED PB4
#define GREEN_LED   (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 1*4))) // off-board green LED PB1

//5 PB
#define PB1 (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4)))  //PC4
#define PB2 (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 5*4)))  //PC5
#define PB3 (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 6*4)))  //PC6
#define PB4 (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4)))  //PC7
#define PB5 (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4)))  //PA3

#define pressed 0
#define unpressed 1
uint8_t value;
char name[20];
int MAX_CHAR=80;
char str[81];
char field_offset[20];
char field_type[10];
int field_count=0;
char alphabet[]={65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,97,98,99,100,101,102,103,104,105,106,107,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122};
int numeric[]={48,49,50,51,52,53,54,55,56,57};
char delimiter[]={32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,58,59,60,61,62,63,91,92,93,94,95,96,123,124,125,126};
uint8_t i=0;
uint8_t SVC_no;
struct semaphore *pSemaphore;
int rtosScheduler();
// function pointer
typedef void (*_fn)();
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks
uint32_t task=0;
uint8_t priority;
uint32_t counter=0;
uint8_t total_ticks;
uint8_t x;
_fn fn;

extern void ResetISR(void);

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------
// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
struct semaphore
{
  uint16_t count;
  uint16_t queueSize;
  char name;
  uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
} semaphores[MAX_SEMAPHORES],*keyPressed, *keyReleased, *flashReq, *resource;
uint8_t semaphoreCount = 0;

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_BLOCKED    3 // has run, but now blocked by semaphore
#define STATE_DELAYED    4 // has run, but now awaiting timer

#define MAX_TASKS 10       // maximum number of valid tasks

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
     void *pid;                     // used to uniquely identify thread
     void *sp;                      // location of stack pointer for thread
     uint8_t priority;              // 0=highest, 15=lowest
     uint8_t currentPriority;       // used for priority inheritance
     uint8_t skipPriority;
     uint8_t cpucount;
     uint32_t ticks;                // ticks until sleep complete
     char name[16];                 // name of task used in ps command
     void *semaphore;           // pointer to the semaphore that is blocking the thread
} tcb[MAX_TASKS];

uint32_t stack[MAX_TASKS][256];
uint32_t Take_sp_to_tcb(){
}
uint32_t SP_to_R0(uint32_t i)
{
    __asm("      MOV SP,R0");

}
uint32_t Pid_to_PC(uint32_t z)
{
    __asm("      MOV PC,R0");
}
uint32_t push_r1(uint32_t r1)
{
    __asm("      MOV R1,R0");
}
uint32_t push_r0(uint32_t r0)
{
    __asm("      MOV R1,R0");
}
uint32_t push_r2(uint32_t r2)
{
    __asm("      MOV R2,R0");
}
__asm("      MOV R4,R1");
__asm("      MOV R5,R0");
__asm("      MOV R6,R2");

uint32_t Take_from_R0() {
    __asm("      MOV R0,R5");
}
uint32_t r1_to_r0(){
    __asm("      MOV R0,R4");
}
uint32_t r2_to_r0(){
    __asm("      MOV R0,R6");
}
void waitMicrosecond(uint32_t us)
{
  // Approx clocks per us
  __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
  __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
  __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
  __asm("             NOP");                  // 5
  __asm("             B    WMS_LOOP1");       // 5*3
  __asm("WMS_DONE1:   SUB  R0, #1");          // 1
  __asm("             CBZ  R0, WMS_DONE0");   // 1
  __asm("             B    WMS_LOOP0");       // 1*3
  __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------
void rtosInit()
{
    uint8_t i;
     // no tasks running
     taskCount = 0;
     // clear out tcb records
     for (i = 0; i < MAX_TASKS; i++)
     {
       tcb[i].state = STATE_INVALID;
       tcb[i].pid = 0;
     }
     // REQUIRED: initialize systick for 1ms system timer
     NVIC_ST_CTRL_R  = 0X07;
     NVIC_ST_RELOAD_R = 0X9C3F; //39999
}

void rtosStart()
{
  // Add code to initialize the SP with tcb[task_current].sp;
 // REQUIRED: add code to call the first task to be run
    taskCurrent = rtosScheduler();
    fn = (_fn)tcb[taskCurrent].pid;
    (*fn)();
}

bool createThread(_fn fn, char name[], int priority)
{
    __asm(" SVC #0x06");
}

void Timer1ISR()
{
     counter++;
     if(counter==0x2625A00)
     {
         counter=0;
     }
    TIMER1_ICR_R |= TIMER_ICR_TATOCINT;
}

// REQUIRED: modify this function to destroy a thread
void destroyThread(_fn fn)
{
    __asm(" SVC #0x05");
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
    uint8_t i=0;
        for(i=0; i<9; i++)
        {
            tcb[i].priority=priority;
        }
}

struct semaphore* createSemaphore(uint8_t count)
{
  struct semaphore *pSemaphore = 0;
  if (semaphoreCount < MAX_SEMAPHORES)
  {
    pSemaphore = &semaphores[semaphoreCount++];
    pSemaphore->count = count;
  }
  return pSemaphore;
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
void yield()
{
    __asm(" SVC #0x01");
    // push registers, call scheduler, pop registers, return to new function
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
void sleep(uint32_t ticks)
{
    __asm(" SVC #0x02");
  // push registers, set state to delayed, store timeout, call scheduler, pop registers,
  // return to new function (separate unrun or ready processing)

}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(struct semaphore *pSemaphore)
{
   __asm(" SVC #0x03");
}
// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post (struct semaphore *pSemaphore)
{
    __asm(" SVC #0x04");
}

// REQUIRED: Implement prioritization to 16 levels
int rtosScheduler()
{
      tcb[task].ticks += counter;
      total_ticks += counter;
      counter++;
      bool ok;
      static uint8_t task = 0xFF;
      ok = false;
      while (!ok)
      {
        task++;
        if (task >= MAX_TASKS)
          task = 0;

        if((tcb[task].state == STATE_READY) && (tcb[task].skipPriority == 0))
               {
                ok = true;
                tcb[task].skipPriority = tcb[task].priority;
               }
               else if ((tcb[task].state == STATE_READY) && (tcb[task].skipPriority > 0))
               {
                     tcb[task].skipPriority--;
               }
               else if(tcb[task].state == STATE_UNRUN)
               {
                   ok=true;
               }
             }
              return task;
}
// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch

void systickIsr()
{
    for (i = 0; i <= MAX_TASKS; i++)
        {
           if(tcb[i].state==STATE_DELAYED)
           {
           tcb[i].ticks--;
           if(tcb[i].ticks==0)
           {
               tcb[i].state=STATE_READY;
           }
           }
        }
     NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
    __asm("             MOV R0, SP");
    __asm("             LDR R0, [R0, #56]");
    __asm("             LDRB R0, [R0, #-2]");
    SVC_no = Take_from_R0();
    switch (SVC_no)
    {
    case 1://yield

            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            break;

    case 2://sleep
        tcb[taskCurrent].ticks = Take_from_R0();
        if (tcb[taskCurrent].ticks > 0)
        {
           tcb[taskCurrent].state = STATE_DELAYED;
        }
           NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
           break;

    case 3: //wait
        (pSemaphore) = Take_from_R0();
        if((*pSemaphore).count > 0)
        {
          (*pSemaphore).count--;
          return;
        }
        else
           {
             for (i = 0; i < MAX_TASKS; ++i)
             {
              if(((*pSemaphore).processQueue != 0) && ((*pSemaphore).queueSize < 6))
              {
              if (tcb[i].priority > tcb[taskCurrent].priority)
              {
               tcb[i].currentPriority = tcb[taskCurrent].priority;
               tcb[i].skipPriority = tcb[taskCurrent].priority;
               (*pSemaphore).processQueue[(*pSemaphore).queueSize] = taskCurrent;
               (*pSemaphore).queueSize++;
               tcb[taskCurrent].state = STATE_BLOCKED;
               (*pSemaphore).queueSize = (*pSemaphore).queueSize % 10;
             }
           }
        }
               NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
        }
               break;

    case 4: //post
        (pSemaphore) = Take_from_R0();
        (*pSemaphore).count++;
        if((*pSemaphore).processQueue[0]>0)
        {
         tcb[(*pSemaphore).processQueue[0]].state = STATE_READY;
         (*pSemaphore).queueSize--;
         for (task = 0; task < 8; task++)
         {
         (*pSemaphore).processQueue[task] = (*pSemaphore).processQueue[task + 1];
         }
         }
          NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
          break;

    case 5: //destroy process

        fn=Take_from_R0();
        while (i < MAX_TASKS)
        {
        if (tcb[i].pid == fn)
        {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
        taskCount--;
        if(tcb[i].pid==tcb[(*pSemaphore).processQueue[i]].pid)
        tcb[(*pSemaphore).processQueue[i]].pid = 0;
        (*pSemaphore).queueSize--;
        (*pSemaphore).processQueue[i] = (*pSemaphore).processQueue[i + 1];
        }
        i++;
        }
        break;

    case 6: //create thread
       {
          bool ok = false;
          bool found = false;
          char* name = r1_to_r0();
          _fn fn = Take_from_R0();
          uint32_t priority = r2_to_r0();

         if (taskCount < MAX_TASKS)
         {
         // make sure fn not already in list (prevent reentrancy)
           while (!found && (i < MAX_TASKS))
           {
             found = (tcb[i++].pid==fn);
           }
             if (!found)
           {
             // find first available tcb record
             i = 0;
             while (tcb[i].state != STATE_INVALID) {i++;}
             tcb[i].state = STATE_UNRUN;
             tcb[i].pid = fn;
             for (x = 0;  x <= strlen(name); x++)
             {
               tcb[i].name[x] = name[x];
             }
               tcb[i].sp = &stack[i][255];
               tcb[i].priority = priority;
               tcb[i].currentPriority = priority;
               // increment task count
               taskCount++;
               ok = true;
            }
          }
           // REQUIRED: allow tasks switches again
              return ok;
              break;
       }
          default:
          break;
    }
}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    YELLOW_LED=1;
    waitMicrosecond(1000);
    YELLOW_LED=0;
    __asm("        PUSH {r4-r11}");
    __asm("        MOV R0,SP");
    tcb[taskCurrent].sp = (void*)Take_sp_to_tcb();
    if(tcb[taskCurrent].ticks==0)
       {
           tcb[taskCurrent].state=STATE_READY;
       }
    taskCurrent = rtosScheduler();
    uint32_t i=(uint32_t)tcb[taskCurrent].sp;
    SP_to_R0(i);
    __asm(" PUSH{R0}");
    __asm(" PUSH{R0}");
    if(tcb[taskCurrent].state == STATE_READY)
    {
     __asm("       POP {r4-r11}");
    }
    else
    {
     __asm("      MOV32 R0,#0x01000000");
     __asm("      PUSH {R0}");               //1
     uint32_t r1=(uint32_t)tcb[taskCurrent].pid;
     push_r1(r1);
     __asm("      PUSH {r1}");               //2
     uint32_t r2=(uint32_t)tcb[taskCurrent].pid;
     push_r2(r2);
     __asm("      PUSH {r2}");               //3
     __asm("      MOV r12,#0x04");
     __asm("      PUSH {r12}");              //4
     __asm("      MOV r3,#0x05");
     __asm("      PUSH {r3}");               //5
     __asm("      MOV r2,#0x06");
     __asm("      PUSH {r2}");               //6
     __asm("      MOV r1,#0x07");
     __asm("      PUSH {r1}");               //7
     __asm("      MOV r0,#0x08");
     __asm("      PUSH {r0}");               //8
     __asm("      MVN r1,#~0xFFFFFFF9");
     __asm("      PUSH {r1}");               //9
     __asm("      MOV r3,#0x10");
     __asm("      PUSH {r3}");               //10
     __asm("      MOV r2,#0x11");
     __asm("      PUSH {r2}");               //11
     __asm("      MOV r3,#0x12");
     __asm("      PUSH {r3}");               //12
    }
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs, 5 pushbuttons, and uart
        // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
        SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);
        // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
        SYSCTL_GPIOHBCTL_R = 0;
        // Enable GPIO port peripherals
        SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA|SYSCTL_RCGC2_GPIOB|SYSCTL_RCGC2_GPIOC|SYSCTL_RCGC2_GPIOD|SYSCTL_RCGC2_GPIOE|SYSCTL_RCGC2_GPIOF;

        // Configure LED GREEN PB1
        GPIO_PORTB_DIR_R |= 0x02;  // bits 2
        GPIO_PORTB_DR2R_R |= 0x02; // set drive strength to 2mA (not needed since default configuration -- for clarity)
        GPIO_PORTB_DEN_R |= 0x02;  // enable

        // Configure LED YELLOW PB4
        GPIO_PORTB_DIR_R |= 0x10;  // bits 6
        GPIO_PORTB_DR2R_R |= 0x10; // set drive strength to 2mA (not needed since default configuration -- for clarity)
        GPIO_PORTB_DEN_R |= 0x10;

        // Configure LED ORANGE PE4
        GPIO_PORTE_DIR_R |= 0x10;  // bits 5
        GPIO_PORTE_DR2R_R |= 0x10; // set drive strength to 2mA (not needed since default configuration -- for clarity)
        GPIO_PORTE_DEN_R |= 0x10;  // enable

        // Configure LED RED PA5
        GPIO_PORTA_DIR_R |= 0x20;  // bits 7
        GPIO_PORTA_DR2R_R |= 0x20; // set drive strength to 2mA (not needed since default configuration -- for clarity)
        GPIO_PORTA_DEN_R |= 0x20;

        // Configure LED BLUE PF2
        GPIO_PORTF_DIR_R |= 0x04;  // bits 6
        GPIO_PORTF_DR2R_R |= 0x04; // set drive strength to 2mA (not needed since default configuration -- for clarity)
        GPIO_PORTF_DEN_R |= 0x04;  // enable

        // Configure PB1 - PC4
           GPIO_PORTC_DEN_R |= 0x10;  // enable
           GPIO_PORTC_PUR_R |= 0x10;  // enable

           //PB2 to PC5
           GPIO_PORTC_DEN_R |= 0x20;
           GPIO_PORTC_PUR_R |= 0x20;

          // Configure PB3 to PC6
           GPIO_PORTC_DEN_R |= 0x40;  // enable
           GPIO_PORTC_PUR_R |= 0x40;  // pull-up

           // Configure PB4 to PC7
           GPIO_PORTC_DEN_R |= 0x80;  // enable
           GPIO_PORTC_PUR_R |= 0x80;  // pull-up

           // Configure PB5 to PA3
           GPIO_PORTA_DEN_R |= 0x08;  // enable
           GPIO_PORTA_PUR_R |= 0x08;  // pull-up

        // Configure UART0 pins PA0-Rx PA1-Tx
        SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
        GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
        GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
        GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

        // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
        UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
        UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
        UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
        UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
        UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
        UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

        // Configure Timer 1 as the time base
        SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
        TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
        TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
        TIMER1_TAILR_R = 0x2625A00;                      // set load value to 40e6 for 1 Hz interrupt rate
        TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
        NVIC_EN0_R |= 1 << (INT_TIMER1A - 16);           // turn-on interrupt 37 (TIMER1A)
        TIMER1_ICR_R |= TIMER_ICR_TATOCINT;
}


// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
// REQUIRED: add code to return a value from 0-31 indicating which of 5 PBs are pressed
uint8_t readPbs()
{
  if(PB1 == pressed)
  {
      value = 1;
  }
  if(PB2 == pressed)
  {
      value = 2;
  }
  if(PB3 == pressed)
  {
      value = 4;
  }
  if(PB4 == pressed)
  {
      value = 8;
  }
  if(PB5 == pressed)
  {
      value = 16;
  }
   return value;
}
// -----------------------------------------------------------------------------
// shell input commands
// -----------------------------------------------------------------------------

void putcUart0(char c) {
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}

void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
        putcUart0(str[i]);
}

char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE)
    {
    yield();
    }
    return UART0_DR_R & 0xFF;
}

void inputString()
           {
           int count=0;
           while(1)    // Endless loop
           {
             char c = getcUart0();
             if (c == 8)  //backspace
             {
               if(count>0)
               {
                 count--;
               }
             }
              else if(c==13)  //CR
              {
                str[count]=0;
                break;
              }

             else if (c>=32) //Space
             {
               str[count++]=c;
               if(count==MAX_CHAR)
               {
                 str[count]=0;
                 break;
               }
            }
          }
       }

   // Convert user entered string to lowercase (Case insensitive)
     void convert_to_lowercase()
       {
          int k;
          for(k = 0; k<strlen(str); k++)
          {
            str[k] = tolower (str[k]);
          }
            putsUart0(str);
        }

   // Converting delimiters to null characters
      void Conv_Delimeters_to_Null()
       {
          int count,r;
          count=strlen(str);
          for(i = 0; str[i] != '\0'; i++)
          {
            if ((str[i]>=32 && str[i] <= 47) || (str[i] >= 58 && str[i] <= 64)|| (str[i] >= 91 && str[i] <= 96)||(str[i] >= 123 && str[i] <= 126)) //Check for delimiters
            {
              str[i] = 0; // make it null
            }
          }
       for(r=0; r<count; r++)
         {
          if((str[r]=='\0')    &&    (str[r+1]>=65 && str[r+1]<=122))  //Compare if alphabet is there after null
          {
            field_offset[field_count]=(r+1);
            field_type[field_count]='A';   //Alphabet
            field_count++;
          }
          if((str[r]=='\0')    &&    (str[r+1]>=48 && str[r+1]<=57))  //Compare if number is there after null
          {
            field_offset[field_count]=(r+1);
            field_type[field_count]='N';    //Number
            field_count++;
          }
        }
      }

// ------------------------------------------------------------------------------
//  Task functions
//-------------------------------------------------------------------------------
// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
  while(true)
  {
      ORANGE_LED = 1;
      waitMicrosecond(1000);
      ORANGE_LED = 0;
      yield();
  }
}

void flash4Hz()
{
  while(true)
  {
    GREEN_LED ^= 1;
    sleep(125);
  }
}

void oneshot()
{
  while(true)
  {
    wait(flashReq);
    YELLOW_LED = 1;
    sleep(1000);
    YELLOW_LED = 0;
  }
}

void partOfLengthyFn()
{
  // represent some lengthy operation
  waitMicrosecond(1000);
  // give another process a chance to run
  yield();
}

void lengthyFn()
{
  uint16_t i;
  while(true)
  {
    wait(resource);
    for (i = 0; i < 4; i++)
    {
      partOfLengthyFn();
    }
    RED_LED ^= 1;
    post(resource);
  }
}

void readKeys()
{
  uint8_t buttons;
  while(true)
  {
    wait(keyReleased);
    buttons = 0;
    while (buttons == 0)
    {
      buttons = readPbs();
      yield();
    }
    post(keyPressed);
    if ((buttons & 1) != 0)
    {
      YELLOW_LED ^= 1;
      RED_LED = 1;
    }
    if ((buttons & 2) != 0)
    {
      post(flashReq);
      RED_LED = 0;
    }
    if ((buttons & 4) != 0)
    {
      createThread(flash4Hz, "Flash4hz", 0);
    }
    if ((buttons & 8) != 0)
    {
      destroyThread(flash4Hz);
    }
    if ((buttons & 16) != 0)
    {
      setThreadPriority(lengthyFn, 4);
    }
    yield();
  }
}

void debounce()
{
  uint8_t count;
  while(true)
  {
    wait(keyPressed);
    count = 10;
    while (count != 0)
    {
      sleep(10);
      if (readPbs() == 0)
        count--;
      else
        count = 10;
    }
    post(keyReleased);
  }
}

void uncooperative()
{
  while(true)
  {
    while (readPbs() == 8)
    {

    }
    yield();
  }
}

void important()
{
    while(true)
    {
      wait(resource);
      BLUE_LED = 1;
      sleep(1000);
      BLUE_LED = 0;
      post(resource);
    }
}

void shell()
{
  unsigned char pointer[150];
  while (true)
  {
   // REQUIRED: add processing for the shell commands through the UART here
      putsUart0("\n\rEnter string:");
      inputString();
      Conv_Delimeters_to_Null();
      convert_to_lowercase();

      //ps command
          NVIC_ST_CTRL_R = 0;
          uint8_t i = 0;
          if(strcmp((str+field_offset[0]),"ps")==0 )  //compare string on offset0
          {
            putsUart0("| PID\t\t| NAME\t\t\t| STATE\t\t| %CPU\t\t |\n\r");
            char* state;
            while (i <= 9)
            {
             if (tcb[i].state == 0)
              {
               state = "STATE_INVALID";
              }
             else if (tcb[i].state == 1)
              {
               state = "STATE_UNRUN";
              }
             else if (tcb[i].state == 2)
              {
               state = "STATE_READY";
              }
              else if (tcb[i].state == 3)
              {
                state = "STATE_BLOCKED";
              }
              else if (tcb[i].state == 4)
              {
                state = "STATE_DELAYED";
              }
            //Calculate CPU
             float percent = (tcb[i].ticks * 100.00) / (total_ticks);
             sprintf(pointer, "| %-10d | %-15s | %-10s | %-5f |\r\n", tcb[i].pid, tcb[i].name, state, percent);
             putsUart0(pointer);
             i++;
            }
         }
         //Command "ipcs"
            if (strcmp((str+field_offset[0]), "ipcs") == 0)
            {
            putsUart0("| SEMAPHORE   | COUNT   |  WAITING TASK \r\n");
            sprintf(pointer, "| flashReq | %-5d |\r\n", (*flashReq).count, "| %-7d |\r\n",(*flashReq).processQueue[0],"\n");
            putsUart0(pointer);
            sprintf(pointer, "| keyReleased | %-5d |\r\n", (*keyReleased).count, "| %-5d |\r\n",(*keyReleased).processQueue[0],"\n");
            putsUart0(pointer);
            sprintf(pointer, "| keyPressed  | %-5d |\r\n", (*keyPressed).count, "| %-5d |\r\n",(*keyPressed).processQueue[0],"\n");
            putsUart0(pointer);
            sprintf(pointer, "| resource  | %-5d |\r\n", (*resource).count, "| %-7d |\r\n", (*resource).processQueue[0],"\n");
            putsUart0(pointer);
            }

           //Command "kill"
            for(s=0;s<field_count;s++)
            {
              if(strcmp((str+field_offset[s]),"kill")==0)  //compare string on offset0
               {
                if(strcmp((str+field_offset[s+1]),"idle")==0) //compare string on offset1
                {
                  putsUart0("\r\n Idle can never be Destroyed \r\n");
                }

              else if (strcmp((str+field_offset[s+1]),"lengthyFn") == 0)
                {
                  tcb[1].state=STATE_INVALID;
                  tcb[1].pid = 0;
                  for(i=0;i<7;i++)
                  {
                    if(tcb[1].pid == tcb[(*pSemaphore).processQueue[i]].pid)
                    tcb[(*pSemaphore).processQueue[i]].pid = 0;
                  }
                    putsUart0("\r\n Thread 1 Destroyed \r\n");
                }
              else if (strcmp((str+field_offset[s+1]),"flash4hz") == 0)
                 {
                  tcb[2].state=STATE_INVALID;
                  tcb[2].pid = 0;
                  for(i=0;i<7;i++)
                   {
                     if(tcb[2].pid == tcb[(*pSemaphore).processQueue[i]].pid)
                     tcb[(*pSemaphore).processQueue[i]].pid = 0;
                   }
                  putsUart0("\r\n Thread 2 Destroyed \r\n");
                 }
              else if (strcmp((str+field_offset[s+1]),"oneShot") == 0)
                 {
                   tcb[3].state=STATE_INVALID;
                   tcb[3].pid = 0;
                   for(i=0;i<7;i++)
                    {
                     if(tcb[3].pid == tcb[(*pSemaphore).processQueue[i]].pid)
                     tcb[(*pSemaphore).processQueue[i]].pid = 0;
                    }
                   putsUart0("\r\n Thread 3 Destroyed \r\n");
                 }
              else if (strcmp((str+field_offset[s+1]),"readKeys") == 0)
                 {
                   tcb[4].state=STATE_INVALID;
                   tcb[4].pid = 0;
                   for(i=0;i<7;i++)
                   {
                     if(tcb[4].pid == tcb[(*pSemaphore).processQueue[i]].pid)
                     tcb[(*pSemaphore).processQueue[i]].pid = 0;
                    }
                   putsUart0("\r\n Thread 4 Destroyed \r\n");
                 }
              else if (strcmp((str+field_offset[s+1]),"Debounce") == 0)
                {
                   tcb[5].state=STATE_INVALID;
                   tcb[5].pid = 0;
                   for(i=0;i<7;i++)
                   {
                    if(tcb[5].pid == tcb[(*pSemaphore).processQueue[i]].pid)
                    tcb[(*pSemaphore).processQueue[i]].pid = 0;
                   }
                  putsUart0("\r\n Thread 5 Destroyed \r\n");
                }
              else if (strcmp((str+field_offset[s+1]),"Important") == 0)
                {
                  tcb[6].state=STATE_INVALID;
                  tcb[6].pid = 0;
                  for(i=0;i<7;i++)
                   {
                    if(tcb[6].pid == tcb[(*pSemaphore).processQueue[i]].pid)
                    tcb[(*pSemaphore).processQueue[i]].pid = 0;
                   }
                    putsUart0("\r\n Thread 6 Destroyed \r\n");
                }
              else if (strcmp((str+field_offset[s+1]),"Uncoop") == 0)
                {
                  tcb[7].state=STATE_INVALID;
                  tcb[7].pid = 0;
                  for(i=0;i<7;i++)
                   {
                    if(tcb[7].pid == tcb[(*pSemaphore).processQueue[i]].pid)
                    tcb[(*pSemaphore).processQueue[i]].pid = 0;
                   }
                  putsUart0("\r\n Thread 7 Destroyed \r\n");
                }
              else if (strcmp((str+field_offset[s+1]),"Shell") == 0)
                {
                  putsUart0("\r\n Shell can never be Destroyed \r\n");
                }
              }
            }

           //Command "reboot"
             if (strcmp((str+field_offset[0]),"reboot") == 0)
              {
                ResetISR();
              }

           //Command "pidof <process name>"
             for(s=0;s<field_count;s++)
               {
                 if (strcmp((str+field_offset[s]), "pidof") == 0)
                  {
                    if (strcmp((str+field_offset[s+1]), "idle") == 0)
                    {
                      sprintf(pointer,"PID of idle is %d \r\n" ,tcb[0].pid);
                    }
                    else if (strcmp((str+field_offset[s+1]), "lengthyfn") == 0)
                    {
                      sprintf(pointer,"PID of lengthyfn is %d \r\n" ,tcb[1].pid);
                    }
                    else if (strcmp((str+field_offset[s+1]), "flash4hz") == 0)
                    {
                      sprintf(pointer,"PID of flash4hz is %d \r\n" ,tcb[2].pid);
                    }
                     else if (strcmp((str+field_offset[s+1]), "oneShot")== 0)
                    {
                      sprintf(pointer,"PID of oneshot is %d \r\n" ,tcb[3].pid);
                    }
                     else if (strcmp((str+field_offset[s+1]), "readkeys") == 0)
                    {
                      sprintf(pointer,"PID of readKeys is %d \r\n" ,tcb[4].pid);
                    }
                    else if (strcmp((str+field_offset[s+1]), "debounce") == 0)
                    {
                      sprintf(pointer,"PID of debounce is %d \r\n" ,tcb[5].pid);
                    }
                    else if (strcmp((str+field_offset[s+1]), "important") == 0)
                    {
                      sprintf(pointer,"PID of important is %d \r\n" ,tcb[6].pid);
                    }
                   else if (strcmp((str+field_offset[s+1]), "uncooperative") == 0)
                    {
                      sprintf(pointer,"PID of uncooperative is %d \r\n" ,tcb[7].pid);
                    }
                    else if (strcmp((str+field_offset[s+1]), "shell") == 0)
                    {
                      sprintf(pointer,"PID of shell is %d \r\n" ,tcb[8].pid);
                    }
                }
            }
                     putsUart0(pointer);


     //process name
                   if(strcmp((str+field_offset[0]), "idle") == 0)
                   {
                       createThread(idle, "Idle", 15);
                   }
                   if(strcmp((str+field_offset[0]), "flash4hz") == 0)
                   {
                       createThread(flash4Hz, "Flash4hz", 4);
                   }
                   if(strcmp((str+field_offset[0]), "lengthyfn") == 0)
                   {
                       createThread(lengthyFn, "LengthyFn", 12);
                   }
                   if(strcmp((str+field_offset[0]), "oneshot") == 0)
                   {
                       createThread(oneshot, "Oneshot", 4);
                   }
                   if(strcmp((str+field_offset[0]), "readkeys") == 0)
                   {
                       createThread(readKeys, "ReadKeys", 8);
                   }
                   if(strcmp((str+field_offset[0]), "debounce") == 0)
                   {
                       createThread(debounce, "Debounce", 8);
                   }
                   if(strcmp((str+field_offset[0]), "important") == 0)
                   {
                       createThread(important, "Important", 0);
                   }
                   if(strcmp((str+field_offset[0]), "uncooperative") == 0)
                   {
                       createThread(uncooperative, "Uncoop", 10);
                   }
                   if(strcmp((str+field_offset[0]), "shell") == 0)
                   {
                       createThread(shell, "Shell", 8);
                   }
           }
    }

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
  bool ok;

  // Initialize hardware
  initHw();
  rtosInit();
         // Power-up flash
  RED_LED = 1;
  waitMicrosecond(250);
  RED_LED = 0;
  waitMicrosecond(250);

  // Initialize semaphores
  keyPressed = createSemaphore(1);
  keyReleased = createSemaphore(0);
  flashReq = createSemaphore(5);
  resource = createSemaphore(1);

  putsUart0("RTOS STARTED...!!!");
  // Add required idle process
  ok =  createThread(idle, "Idle", 15);
  //ok =  createThread(idle2, "Idle2", 15);
  //ok =  createThread(idle1, "Idle1", 15);

  // Add other processes
  ok &= createThread(lengthyFn, "LengthyFn", 12);
  ok &= createThread(flash4Hz, "Flash4hz", 4);
  ok &= createThread(oneshot, "OneShot", 4);
  ok &= createThread(readKeys, "ReadKeys", 8);
  ok &= createThread(debounce, "Debounce", 8);
  ok &= createThread(important, "Important", 0);
  ok &= createThread(uncooperative, "Uncoop", 10);
  ok &= createThread(shell, "Shell", 8);


  // Start up RTOS
  if (ok)
    rtosStart(); // never returns
  else
    RED_LED = 1;

return 0;
  // don't delete this unreachable code
  // if a function is only called once in your code, it will be
  // accessed with two goto instructions instead of call-return,
  // so any stack-based code will not function correctly
  yield(0);
  sleep(0);
  wait(0);
  post(0);
}
