#pragma once
/*
*  debugging.h 
*  use to set up debugging code that can be removed before compile time
*/


#define DEBUG
//#define DEBUG1
 
#ifdef DEBUG
#define D(x) x
#ifdef DEBUG1
#define D1(x) x
#else
#define D1(x)
#endif
#else
#define D(x)
#define D1(x)
#endif