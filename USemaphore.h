/*
 * =====================================================================================
 *
 *       Filename:  USemaphore.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2016年09月08日 12时10分50秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:   (), 
 *   Organization:  
 *
 * =====================================================================================
 */



#ifndef USEMAPHORE_H_APLE0CQ2
#define USEMAPHORE_H_APLE0CQ2

#include <errno.h>
#include <pthread.h>
#include <sys/time.h>

class USemaphore{
public:
    USemaphore(int initValue = 0){
        _available = initValue;
        pthread_mutex_init(&_waitMutex, NULL);
        pthread_cond_init(&_cond, NULL);
    }
    virtual ~USemaphore(){
        pthread_cond_destroy(&_cond);
        pthread_mutex_destroy(&_waitMutex);
    }
    bool acquire(int n = 1, int ms = 0){
        int rt = 0;
        pthread_mutex_lock(&_waitMutex);
        while(n > _available && rt == 0){
            if(ms > 0){
                struct timespec timeToWait;
                struct timeval now;
                gettimeofday(&now, NULL);
                timeToWait.tv_sec = now.tv_sec + ms / 1000;
                timeToWait.tv_nsec = (now.tv_usec + 1000UL * (ms % 1000)) * 1000UL;
                rt = pthread_cond_timedwait(&_cond, &_waitMutex, &timeToWait);
            }
            else
                rt = pthread_cond_wait(&_cond, &_waitMutex);
        }
        if(rt == 0){
            _available -= n;
        }
        pthread_mutex_unlock(&_waitMutex);
        return rt == 0;
    }
    int acquireTry(int n){
        pthread_mutex_lock(&_waitMutex);
        if(n > _available){
            pthread_mutex_unlock(&_waitMutex);
            return false;
        }
        _available -= n;
        pthread_mutex_unlock(&_waitMutex);
        return true;
    }
    void release(int n = 1){
        pthread_mutex_lock(&_waitMutex);
        _available += n;
        pthread_cond_broadcast(&_cond);
        pthread_mutex_unlock(&_waitMutex);
    }
    int value(){
        int value = 0;
        pthread_mutex_lock(&_waitMutex);
        value = _available;
        pthread_mutex_unlock(&_waitMutex);
        return value;
    }
private:
    void operator=(const USemaphore &S){}
    USemaphore(const USemaphore &S):_available(0){}
    pthread_mutex_t _waitMutex;
    pthread_cond_t _cond;
    int _available;
};

#endif /* end of include guard: USEMAPHORE_H_APLE0CQ2 */
