#ifndef __WALKER_YARD_CONFIG_H__
#define __WALKER_YARD_CONFIG_H__

#cmakedefine WALKER_YARD_USE_SSE2
#cmakedefine WALKER_YARD_USE_SSE3
#cmakedefine WALKER_YARD_USE_SSE4

#if defined(WALKER_YARD_USE_SSE2) || defined(WALKER_YARD_USE_SSE3) || defined (WALKER_YARD_USE_SSE4)
#define WALKER_YARD_USE_SSE
#endif

#endif // __WALKER_YARD_CONFIG_H__
