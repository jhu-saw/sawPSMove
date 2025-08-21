// sawPSMove/components/include/sawPSMove/sawPSMoveExport.h
#pragma once
#if defined(_WIN32)
  #if defined(sawPSMove_EXPORTS)
    #define SAW_PSMOVE_EXPORT __declspec(dllexport)
  #else
    #define SAW_PSMOVE_EXPORT __declspec(dllimport)
  #endif
#else
  #define SAW_PSMOVE_EXPORT
#endif
