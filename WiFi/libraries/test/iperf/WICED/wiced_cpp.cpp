/*
 * $ Copyright Broadcom Corporation $
 */
/**
 * Most low-end ARM-based MCUs cannot tolerate 50KB code overhead. To eliminate
 * that code you need to define your own, non-throwing versions of global new
 * and delete.
 *
 * @remarks Adopted from http://www.state-machine.com/arm/Building_bare-metal_ARM_with_GNU.pdf
 * @see http://www.state-machine.com/arm/Building_bare-metal_ARM_with_GNU.pdf
 */

#if defined(WICED) && !defined(RTOS_EMBOS)

#include <stdlib.h>
/**
 * The standard version of the operator new throws std::bad_alloc exception.
 * This version explicitly throws no exceptions. This minimal implementation
 * uses the standard malloc().
 */
void *operator new( size_t size )
{
    return malloc( size );
}


/**
 * This minimal implementation uses the standard free().
 */
#if !(defined (__GNUC__) && (__GNUC__ >= 6))
void operator delete( void *p )
{
    free( p );
}
#else
void operator delete( void *p, size_t size )
{
    free( p );
}
#endif /* !(defined (__GNUC__) && (__GNUC__ >= 6)) */

/**
 * The function __aeabi_atexit() handles the static destructors. In a
 * bare-metal system this function can be empty because application has no
 * operating system to return to, and consequently the static destructors are
 * never called.
 */
extern "C" int __aeabi_atexit( void *object, void (*destructor) (void *), void *dso_handle )
{
    return 0;
}
#endif /* defined(WICED) && !defined(RTOS_EMBOS) */
