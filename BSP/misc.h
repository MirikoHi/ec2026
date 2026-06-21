#ifndef _MISC_H_
#define _MISC_H_

#define limit_max_min(value,max,min)		\
{																				\
		if(value>max)												\
		{																		\
			value=max;												\
		}																		\
		if(value<min)												\
		{																		\
			value=min;												\
		}																		\
}

#define Abs(value)											\
{																				\
		if(value<0)													\
		{																		\
			value=-value;											\
		}																		\
}
#define abs_out(x) ((x > 0) ? x : -(x))
typedef enum {
	DISABLE=0,
	ENABLE
}State;

#define PI 3.1415926f

#endif