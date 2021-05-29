//
// File: ert_main.cpp
//
// Code generated for Simulink model 'RflysimIndoorController_r2018b_N12_V9'.
//
// Model version                  : 1.88
// Simulink Coder version         : 9.0 (R2018b) 24-May-2018
// C/C++ source code generated on : Fri Dec 25 16:18:22 2020
//
// Target selection: ert.tlc
// Embedded hardware selection: Generic->Unspecified (assume 32-bit Generic)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include <stdio.h>
#include <stdlib.h>
#include "RflysimIndoorController_r2018b_N12_V9.h"
#include "RflysimIndoorController_r2018b_N12_V9_private.h"
#include "rtwtypes.h"
#include "limits.h"
#include "rt_nonfinite.h"
#include "linuxinitialize.h"
#define UNUSED(x)                      x = x

// Function prototype declaration
void exitFcn(int sig);
void *terminateTask(void *arg);
void *baseRateTask(void *arg);
void *subrateTask(void *arg);
volatile boolean_T stopRequested = false;
volatile boolean_T runModel = true;
sem_t stopSem;
sem_t baserateTaskSem;
pthread_t schedulerThread;
pthread_t baseRateThread;
unsigned long threadJoinStatus[8];
int terminatingmodel = 0;
void *baseRateTask(void *arg)
{
  runModel = (rtmGetErrorStatus(RflysimIndoorController_r201_M) == (NULL));
  while (runModel) {
    sem_wait(&baserateTaskSem);
    RflysimIndoorController_r2018b_N12_V9_step();

    // Get model outputs here
    stopRequested = !((rtmGetErrorStatus(RflysimIndoorController_r201_M) ==
                       (NULL)));
    runModel = !stopRequested;
  }

  runModel = 0;
  terminateTask(arg);
  pthread_exit((void *)0);
  return NULL;
}

void exitFcn(int sig)
{
  UNUSED(sig);
  rtmSetErrorStatus(RflysimIndoorController_r201_M, "stopping the model");
}

void *terminateTask(void *arg)
{
  UNUSED(arg);
  terminatingmodel = 1;

  {
    runModel = 0;
  }

  // Disable rt_OneStep() here

  // Terminate model
  RflysimIndoorController_r2018b_N12_V9_terminate();
  sem_post(&stopSem);
  return NULL;
}

int main(int argc, char **argv)
{
  UNUSED(argc);
  UNUSED(argv);
  void slros_node_init(int argc, char** argv);
  slros_node_init(argc, argv);
  rtmSetErrorStatus(RflysimIndoorController_r201_M, 0);

  // Initialize model
  RflysimIndoorController_r2018b_N12_V9_initialize();

  // Call RTOS Initialization function
  myRTOSInit(0.05, 0);

  // Wait for stop semaphore
  sem_wait(&stopSem);

#if (MW_NUMBER_TIMER_DRIVEN_TASKS > 0)

  {
    int i;
    for (i=0; i < MW_NUMBER_TIMER_DRIVEN_TASKS; i++) {
      CHECK_STATUS(sem_destroy(&timerTaskSem[i]), 0, "sem_destroy");
    }
  }

#endif

  return 0;
}

//
// File trailer for generated code.
//
// [EOF]
//
