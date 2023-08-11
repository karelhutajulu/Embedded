// // RTOS TASK 1
#include "mbed.h"
#include "rtos.h"

uint32_t counter = 0;
Mutex mutex;

void task1() {
  while (true) {
    mutex.lock();
    printf("Counter value: %lu\n", counter);
    mutex.unlock();
    Thread::wait(1000);
  }
}

void task2() {
  while (true) {
    mutex.lock();
    counter++;
    mutex.unlock();
    Thread::wait(5000);
  }
}

int main() {
  Thread thread1(task1);
  Thread thread2(task2);

  thread1.join();
  thread2.join();
}

// RTOS TASK 2
// #include "mbed.h"
// #include "rtos.h"

// Queue<uint32_t, 5> queue;
// Mutex mutex;

// void task1() {
//   while (true) {
//     osEvent evt = queue.get();
//     if (evt.status == osEventMessage) {
//       uint32_t counter = *(uint32_t *)evt.value.p;
//       printf("Counter value: %lu\n", counter);
//     }
//   }
// }

// void task2() {
//   uint32_t counter = 0;
//   while (true) {
//     queue.put(&counter);
//     counter++;
//     Thread::wait(5000);
//   }
// }

// void timer_callback() { mutex.lock(); }

// int main() {
//   Thread thread1(task1);
//   Thread thread2(task2);

//   RtosTimer timer(timer_callback, osTimerOnce);
//   timer.start(15000);

//   thread1.join();
//   thread2.join();
// }
