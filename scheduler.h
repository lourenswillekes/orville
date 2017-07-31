#ifndef _SCHEDULER_H_
#define _SCHEDULER_H_

extern unsigned currThread;
extern void yield(void);
extern int getCurrThread(void);

extern unsigned uartlock;
extern void lock_init(unsigned *lock);
extern unsigned lock_acquire(unsigned *lock);
extern void lock_release(unsigned *lock);

#endif // _SCHEDULER_H_
