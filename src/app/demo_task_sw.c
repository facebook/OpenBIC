#include <stdio.h>
#include "cmsis_os.h"

osThreadId_t tid_taskA;
osThreadAttr_t tattr_taskA;
osThreadId_t tid_taskB;
osThreadAttr_t tattr_taskB;

static void vTestTaskB(void *argv)
{
	while (1) {
		printf("taskB\n");
		osDelay(1000);
	}
}

static void vTestTaskA(void *argv)
{
	while (1) {
		printf("taskA\n");
		osDelay(1000);
	}
}

void demo_task_sw_init(void)
{
	tattr_taskA.name = "demo_A";
	tattr_taskA.priority = osPriorityBelowNormal;
	tattr_taskB.name = "demo_B";
	tattr_taskB.priority = osPriorityBelowNormal;

	tid_taskA = osThreadNew(vTestTaskA, NULL, &tattr_taskA);
	tid_taskB = osThreadNew(vTestTaskB, NULL, &tattr_taskB);
}