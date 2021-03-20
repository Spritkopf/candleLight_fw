//
// This file is part of the µOS++ III distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <sys/types.h>
#include <errno.h>

// ----------------------------------------------------------------------------

caddr_t
_sbrk(int incr);

// ----------------------------------------------------------------------------

// The definitions used here should be kept in sync with the
// stack definitions in the linker script.
register char * stack_ptr asm("sp");

caddr_t _sbrk(int incr)
{
	extern char end asm("end");
	static char *heap_end;
	char *prev_heap_end;

	if (heap_end == 0)
		heap_end = &end;

	prev_heap_end = heap_end;
	if (heap_end + incr > stack_ptr)
	{
//		write(1, "Heap and stack collision\n", 25);
//		abort();
		errno = ENOMEM;
		return (caddr_t) -1;
	}

	heap_end += incr;

	return (caddr_t) prev_heap_end;
}


// ----------------------------------------------------------------------------

