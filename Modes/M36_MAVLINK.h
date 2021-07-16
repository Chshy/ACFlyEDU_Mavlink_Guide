#pragma once
#include "Modes.h"

extern const Mode M36_MAVLINK;
bool Get_Guided_Mode_Enabled(void);
bool Get_POS_Control_Enabled(void);
void Set_POS_Control_Enabled(bool value);