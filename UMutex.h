/*
 * =====================================================================================
 *
 *       Filename:  UMutex.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2016年09月08日 11时16分15秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:   (), 
 *   Organization:  
 *
 * =====================================================================================
 */
#ifndef UMUTEX_H_E9PFNIXM
#define UMUTEX_H_E9PFNIXM

#include <errno.h>
#include <pthread.h>
class UMutex{
public:
    UMutex(){
        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
        pthread_mutex_init(&M, &attr);
        pthread_mutexattr_destroy(&attr);
    }
    virtual ~UMutex(){
        pthread_mutex_unlock(&M);
        pthread_mutex_destroy(&M);
    }
    int lock() const{
        return pthread_mutex_lock(&M);
    }
    int lockTry() const{
        return pthread_mutex_trylock(&M);
    }
    int unlock() const{
        return pthread_mutex_unlock(&M);
    }
private:
    mutable pthread_mutex_t M;
    void operator=(UMutex &M){}
    UMutex(const UMutex&M){}
};

class UScopeMutex{
public:
    UScopeMutex(const UMutex& mutex):
        _mutex(mutex){
            _mutex.lock();
        }
    UScopeMutex(UMutex* mutex):
        _mutex(*mutex){
            _mutex.lock();
        }
    //UScopeMutex(UMutex& mutex):
    //_mutex(mutex){
    //_mutex.lock();
    //}
    ~UScopeMutex(){
        _mutex.unlock();
    }
private:
    const UMutex& _mutex;
};

#endif /* end of include guard: UMUTEX_H_E9PFNIXM */
