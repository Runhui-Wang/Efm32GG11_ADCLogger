/***************************************************************************//**
 * @file
 * @brief stream adc data through uart and write to sd card. hw and ag
 * last edited: 7/6/2023
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include "os.h"
#include "ff.h"
#include "sdio.h"
#include "diskio.h"
#include "stdbool.h"
#include "mod_fatfs_chan.h"
#include "em_leuart.h"
#include "em_adc.h"
#include "sl_cli.h"
#include "sl_cli_instances.h"
#include "sl_cli_arguments.h"
#include "sl_cli_handles.h"
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "sl_sleeptimer.h"
#include "app_iostream_usart.h"
#include "sl_iostream.h"
#include "sl_iostream_init_instances.h"
#include "sl_iostream_handles.h"
#include<inttypes.h>
int output_channel;

static uint32_t rxDataReady = 0;  // Flag indicating receiver does not have data
static volatile char rxBuffer[20]; // Software receive buffer
static char txBuffer[20]; // Software transmit buffer
char read_buffer[30];
int read_request_bytes = 30;
unsigned int read_bytes_recieved = 0;
unsigned int file_num = 0;
static sl_sleeptimer_timer_handle_t periodic_timer;
#define TIMEOUT_MS 2
#define adcFreq         16000000
uint32_t sample;
uint32_t millivolts;
void
ADC_tas ();
uint64_t tick_cnt = 0;
uint64_t timestamp;
sl_status_t stat;
int retval = 0;
int total_bytes_written;
int write_request_bytes;
int write_bytes_written;
RTOS_ERR RT_ERR;
OS_TCB ADC_TaskTCB;
OS_TASK_PTR ADC_task;
#define   ADC_TASK_STK_SIZE 256u
#define RX_BUFFER_SIZE 20
CPU_STK ADC_TaskStk[ADC_TASK_STK_SIZE];
;
FIL superfile;
uint64_t time_offset = 0;
uint32_t data_entries = 0;
uint64_t new_time;
uint64_t new_tick_cnt;
uint64_t descrep;
int reach_flag = 0;

/*******************************************************************************
 *******************************   DEFINES   ***********************************
 ******************************************************************************/

#ifndef BUFSIZE
#define BUFSIZE    80
#endif

#ifndef TERMINAL_TASK_STACK_SIZE
#define TERMINAL_TASK_STACK_SIZE      256
#endif

#ifndef TERMINAL_TASK_PRIO
#define TERMINAL_TASK_PRIO            20
#endif
char file_name[] = "ADC_Data_0000.txt";
void
set_time (sl_cli_command_arg_t *arguments);

struct adc_data
{
  uint64_t t_stamp;
  uint32_t voltage;
};

struct adc_data *cirbuff;
int front = 0;
int back = 160;
/*******************************************************************************
 ***************************  LOCAL VARIABLES   ********************************
 ******************************************************************************/

static OS_TCB tcb;
static CPU_STK stack[TERMINAL_TASK_STACK_SIZE];

/* Input buffer */
static char buffer[BUFSIZE];
static const sl_cli_command_info_t cmd__echoint =
SL_CLI_COMMAND(set_time,
    "Set the time",
    "?",
      { SL_CLI_ARG_INT32, SL_CLI_ARG_END,});

static sl_cli_command_entry_t a_table[] =
  {
    { "set_time", &cmd__echoint, false },

    { NULL, NULL, false }, };

static sl_cli_command_group_t a_group =
  {
    { NULL },
  false, a_table };
sl_cli_command_group_t *command_group = &a_group;

/*******************************************************************************
 *********************   LOCAL FUNCTION PROTOTYPES   ***************************
 ******************************************************************************/
void
app_iostream_terminal_task (void *arg);

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

/***************************************************************************//**
 * Initialize example.
 ******************************************************************************/
void
initialize_GPIO ()
{

}
void
file_setup (void)
{

  char file_name[] = "ADC_Data_0000.txt";
  int num_write = 0;
  unsigned int nwritten = 0;
  char header_txt[25] = "I don't know the format.\n";
  int write_request_bytes = 0;
  int read_request_bytes = 24;
  int total_bytes_written = 0;
  unsigned int write_bytes_written = 0;
  unsigned int read_bytes_recieved = 0;

  //create testfile on sd card and write a test string
  retval = f_open (&superfile, file_name,
  FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
  if (retval != FR_OK)
    {
      printf ("Failed to open %s, error %u\n", file_name, retval);
    }

  retval = f_close (&superfile);
  if (retval != FR_OK)
    {
      printf ("Failed to close %s, error %u\n", file_name, retval);
    }

  //closing and reopening is taxing- but we'd like to write to the file without overwriting
  //each data entry. opening in FA_OPEN_APPEND lets us write multiple lines to the file
  retval = f_open (&superfile, file_name, FA_OPEN_APPEND | FA_WRITE | FA_READ);
  if (retval != FR_OK)
    {
      printf ("Failed to open %s, error %u\n", file_name, retval);
    }

  return;
}
void
app_iostream_usart_init (void)
{
  RTOS_ERR err;

  /* Prevent buffering of output/input.*/
#if !defined(__CROSSWORKS_ARM) && defined(__GNUC__)
  setvbuf (stdout, NULL, _IONBF, 0); /*Set unbuffered mode for stdout (newlib)*/
  setvbuf (stdin, NULL, _IONBF, 0); /*Set unbuffered mode for stdin (newlib)*/
#endif
  fs_bsp_init (); //SD Card Set-up
  file_setup (); // First file initialization

  // Enable ADC0 clock
  CMU_ClockEnable (cmuClock_ADC0, true);
  CMU_ClockSelectSet (cmuClock_LFE, cmuSelect_LFRCO);
  CMU_ClockEnable (cmuClock_RTCC, true);
  // Declare init structs
  ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
  ADC_InitSingle_TypeDef initSingle = ADC_INITSINGLE_DEFAULT;

  // Modify init structs and initialize
  init.prescale = ADC_PrescaleCalc (adcFreq, 0); // Init to max ADC clock for Series 1

  initSingle.diff = false;        // single ended
  initSingle.reference = adcRef2V5;    // internal 2.5V reference
  initSingle.resolution = adcRes12Bit;  // 12-bit resolution
  initSingle.acqTime = adcAcqTime4; // set acquisition time to meet minimum requirement

  // Expansion Header Pin #6
  initSingle.posSel = adcPosSelAPORT4XCH11;
  init.timebase = ADC_TimebaseCalc (0);

  //Initialize ADC
  ADC_Init (ADC0, &init);
  ADC_InitSingle (ADC0, &initSingle);

  //Add task to system
  OSTaskCreate (&tcb, "iostream terminal task", app_iostream_terminal_task,
  DEF_NULL,
                TERMINAL_TASK_PRIO, &stack[0], (TERMINAL_TASK_STACK_SIZE / 10u),
                TERMINAL_TASK_STACK_SIZE,
                0u, 0u,
                DEF_NULL,
                (OS_OPT_TASK_STK_CLR ), &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}

/***************************************************************************//**
 * Terminal task.
 ******************************************************************************/
static void
on_periodic_timeout (sl_sleeptimer_timer_handle_t *handle, void *data)
{
  if (front == 160)
    {
      front = 0;
    }
  (void) &handle;
  (void) &data;
  reach_flag = 1;
  ADC_Start (ADC0, adcStartSingle);
  while (!(ADC0->STATUS & _ADC_STATUS_SINGLEDV_MASK))
    ;
  sample = ADC_DataSingleGet (ADC0);
  // Calculate input voltage in mV
  millivolts = (sample * 2500) / 4096;
  tick_cnt = sl_sleeptimer_get_tick_count64 ();
  stat = sl_sleeptimer_tick64_to_ms (tick_cnt, &timestamp);
  uint64_t new_t = timestamp + time_offset * 1000;
  (cirbuff + front)->t_stamp = new_t;
  (cirbuff + front)->voltage = millivolts;
  front++;


}
void
app_iostream_terminal_task (void *arg)
{

  RTOS_ERR err;
  //Set default iostream to be expansion header 12,14
  sl_iostream_set_default (sl_iostream_exp_handle);
  //Start the periodic timer of 200 ms. Trigger timeout callback function.
  sl_sleeptimer_start_periodic_timer_ms (
      &periodic_timer,
      TIMEOUT_MS,
      on_periodic_timeout, NULL, 0,
      0);
  char* output =  malloc(512*sizeof output);

  while (true)
    {
      uint64_t new_t;
      uint32_t millivolt;
      if (back == 160)
        {
          //check whether reached end of cirbuff
          //return to beginning if reached
          back = 0;
        }

      while (true)
        {
          if(back==front){
              OSTimeDly( 1,
                         OS_OPT_TIME_DLY,
                         &err);
          }
          else{
              //if there is no new data.
              /**OSTimeDly( 1,
               OS_OPT_TIME_DLY,
               &err);**/
              new_t = (cirbuff + back)->t_stamp;
              millivolt = (cirbuff + back)->voltage;
              //divide the 64 bit int to two to correctly print
              uint32_t first_t, second_t;
              first_t = (uint32_t) (new_t >> 32);
              second_t = (uint32_t) new_t;
              snprintf (output + ((back %16) * 32), 34, "Time: %08x%08x mV: %04d\n", first_t,
                        second_t, millivolt);
              back++;
          }

          if(back!=0&&back%16==0){
              break;
          }

        }

      //read from circular buffer

      //divide the 64 bit int to two to correctly print
      uint32_t first_t, second_t;
      first_t = (uint32_t) (new_t >> 32);
      second_t = (uint32_t) new_t;
      //write to the string to be used for SD card write
      //print to iostream
     printf ("Time: %08x%08x mV: %04d\n", first_t, second_t, millivolts);
      if (data_entries+160 >= 32000)
        {
          //if a file has 30000 data entries, close that and initialize a new one

          f_close (&superfile);
          file_num++;

          snprintf (file_name, 18, "ADC_Data_%04d.txt", file_num);
          retval = f_open (&superfile, file_name,
          FA_CREATE_ALWAYS | FA_WRITE | FA_READ);
          data_entries = 0;
          retval = f_open (&superfile, file_name,
          FA_OPEN_APPEND | FA_WRITE | FA_READ);

        }
      else
        {
          data_entries+=16;
          //if there is still space, open the file to append at the  end


        }
      if (retval != FR_OK)
        {
          printf ("Failed to open %s, error %u\n", file_name, retval);
        }
      //write the string to file

      write_request_bytes = 512;

          retval = f_write (&superfile, output, write_request_bytes,
                            &write_bytes_written);



      if (retval != FR_OK)
        {
          printf ("Failed to write %s, error %u\n", file_name, retval);
        }

    }

}
void
cli_init ()
{

  bool status;
  sl_iostream_set_default (sl_iostream_exp_handle);
  GPIO_PinModeSet (gpioPortE, 9, gpioModeInput, 0);    // RX
  //init the cirbuff, able to hold 20 struct adc_data
  cirbuff = malloc (160 * sizeof *cirbuff);
  status = sl_cli_command_add_command_group (sl_cli_inst_handle, command_group);
  EFM_ASSERT(status);
  printf ("\r\n Started ADC, enter command\r\n\r\n");
}
void
set_time (sl_cli_command_arg_t *arguments)
{
  uint64_t argument_value;
  RTOS_ERR err;
  // Read all the arguments provided as integers and print them back

  argument_value = (uint64_t) sl_cli_get_argument_int32(arguments, 0);

  time_offset = argument_value;

  app_iostream_usart_init ();

}
