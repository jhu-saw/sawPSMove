// sawPSMove/components/include/sawPSMove/sawPSMoveQtExport.h
#pragma once
#if defined(_WIN32)
  #if defined(sawPSMoveQt_EXPORTS)
    #define SAW_PSMOVE_QT_EXPORT __declspec(dllexport)
  #else
    #define SAW_PSMOVE_QT_EXPORT __declspec(dllimport)
  #endif
#else
  #define SAW_PSMOVE_QT_EXPORT
#endif
