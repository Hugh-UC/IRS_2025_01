PROGRAM timerbelt
  VAR
    Button AT %QX0.0 : BOOL;
    Motor AT %QX0.1 : BOOL;
    Pulse AT %QX0.2 : BOOL;
    ResetButton AT %QX0.3 : BOOL;
    count_value AT %MW1 : INT;
  END_VAR
  VAR
    Preset : INT := 100;
  END_VAR
  VAR
    WeightSensor AT %MW0 : INT;
    BigBoxCount AT %MW2 : INT;
    MediumBoxCount AT %MW3 : INT;
    SmallBoxCount AT %MW4 : INT;
  END_VAR
  VAR
    LowerLimit : INT := 5;
    UpperLimit : INT := 10;
    ZeroLimit : INT := 0;
    BigCounter : CTU;
    MediumCounter : CTU;
    SmallCounter : CTU;
    isSmallBox : BOOL := False;
    isMediumBox : BOOL := False;
    isBigBox : BOOL := False;
    BigBoxTimer : TON;
    MediumBoxTimer : TON;
    SmallBoxTimer : TON;
    StopMotorTimer : BOOL := False;
    _TMP_GT6_OUT : BOOL;
    _TMP_AND12_OUT : BOOL;
    R_TRIG1 : R_TRIG;
    _TMP_GT7_OUT : BOOL;
    _TMP_LE9_OUT : BOOL;
    _TMP_AND17_OUT : BOOL;
    _TMP_AND13_OUT : BOOL;
    R_TRIG2 : R_TRIG;
    _TMP_LE8_OUT : BOOL;
    _TMP_GT49_OUT : BOOL;
    _TMP_AND50_OUT : BOOL;
    _TMP_AND14_OUT : BOOL;
    R_TRIG3 : R_TRIG;
    _TMP_ADD28_OUT : INT;
    _TMP_OR46_OUT : BOOL;
  END_VAR

  Motor := NOT(StopMotorTimer) AND Button;
  _TMP_GT6_OUT := GT(WeightSensor, UpperLimit);
  _TMP_AND12_OUT := AND(_TMP_GT6_OUT, Pulse);
  R_TRIG1(CLK := _TMP_AND12_OUT);
  BigCounter(CU := R_TRIG1.Q, R := ResetButton, PV := Preset);
  BigBoxCount := BigCounter.CV;
  _TMP_GT7_OUT := GT(WeightSensor, LowerLimit);
  _TMP_LE9_OUT := LE(WeightSensor, UpperLimit);
  _TMP_AND17_OUT := AND(_TMP_GT7_OUT, _TMP_LE9_OUT);
  _TMP_AND13_OUT := AND(_TMP_AND17_OUT, Pulse);
  R_TRIG2(CLK := _TMP_AND13_OUT);
  MediumCounter(CU := R_TRIG2.Q, R := ResetButton, PV := Preset);
  MediumBoxCount := MediumCounter.CV;
  _TMP_LE8_OUT := LE(WeightSensor, LowerLimit);
  _TMP_GT49_OUT := GT(WeightSensor, ZeroLimit);
  _TMP_AND50_OUT := AND(_TMP_LE8_OUT, _TMP_GT49_OUT);
  _TMP_AND14_OUT := AND(_TMP_AND50_OUT, Pulse);
  R_TRIG3(CLK := _TMP_AND14_OUT);
  SmallCounter(CU := R_TRIG3.Q, R := ResetButton, PV := Preset);
  SmallBoxCount := SmallCounter.CV;
  _TMP_ADD28_OUT := ADD(BigBoxCount, MediumBoxCount, SmallBoxCount);
  count_value := _TMP_ADD28_OUT;
  isBigBox := _TMP_GT6_OUT;
  isMediumBox := _TMP_AND17_OUT;
  isSmallBox := _TMP_AND50_OUT;
  BigBoxTimer(IN := isBigBox, PT := T#2000ms);
  MediumBoxTimer(IN := isMediumBox, PT := T#3500ms);
  SmallBoxTimer(IN := isSmallBox, PT := T#5000ms);
  _TMP_OR46_OUT := OR(BigBoxTimer.Q, MediumBoxTimer.Q, SmallBoxTimer.Q);
  StopMotorTimer := _TMP_OR46_OUT;
END_PROGRAM


CONFIGURATION Config0

  RESOURCE Res0 ON PLC
    TASK task0(INTERVAL := T#20ms,PRIORITY := 0);
    PROGRAM instance0 WITH task0 : timerbelt;
  END_RESOURCE
END_CONFIGURATION

(*DBG:char md5[] = "6174b1bade5d9927c12b2ab29136f85b";*)
(*DBG:/**)
(*DBG: * This file is part of OpenPLC Runtime*)
(*DBG: **)
(*DBG: * Copyright (C) 2023 Autonomy, GP Orcullo*)
(*DBG: * Based on the work by GP Orcullo on Beremiz for uC*)
(*DBG: **)
(*DBG: * This program is free software; you can redistribute it and/or*)
(*DBG: * modify it under the terms of the GNU General Public License*)
(*DBG: * as published by the Free Software Foundation; either version 2*)
(*DBG: * of the License, or (at your option) any later version.*)
(*DBG: **)
(*DBG: * This program is distributed in the hope that it will be useful,*)
(*DBG: * but WITHOUT ANY WARRANTY; without even the implied warranty of*)
(*DBG: * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the*)
(*DBG: * GNU General Public License for more details.*)
(*DBG: **)
(*DBG: * You should have received a copy of the GNU General Public License*)
(*DBG: * along with this program; If not, see <http://www.gnu.org/licenses/>.*)
(*DBG: **)
(*DBG: */*)
(*DBG:*)
(*DBG:#include <stdbool.h>*)
(*DBG:*)
(*DBG:#include "iec_types_all.h"*)
(*DBG:#include "POUS.h"*)
(*DBG:*)
(*DBG:#define SAME_ENDIANNESS      0*)
(*DBG:#define REVERSE_ENDIANNESS   1*)
(*DBG:*)
(*DBG:uint8_t endianness;*)
(*DBG:*)
(*DBG:*)
(*DBG:extern TIMERBELT RES0__INSTANCE0;*)
(*DBG:*)
(*DBG:static const struct {*)
(*DBG:    void *ptr;*)
(*DBG:    __IEC_types_enum type;*)
(*DBG:} debug_vars[] = {*)
(*DBG:    {&(RES0__INSTANCE0.BUTTON), BOOL_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MOTOR), BOOL_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.PULSE), BOOL_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.RESETBUTTON), BOOL_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.COUNT_VALUE), INT_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.PRESET), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.WEIGHTSENSOR), INT_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGBOXCOUNT), INT_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MEDIUMBOXCOUNT), INT_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMALLBOXCOUNT), INT_O_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.LOWERLIMIT), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.UPPERLIMIT), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.ZEROLIMIT), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGCOUNTER.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGCOUNTER.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGCOUNTER.CU), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGCOUNTER.R), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGCOUNTER.PV), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGCOUNTER.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGCOUNTER.CV), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGCOUNTER.CU_T.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGCOUNTER.CU_T.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGCOUNTER.CU_T.CLK), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGCOUNTER.CU_T.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGCOUNTER.CU_T.M), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MEDIUMCOUNTER.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MEDIUMCOUNTER.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MEDIUMCOUNTER.CU), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MEDIUMCOUNTER.R), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MEDIUMCOUNTER.PV), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MEDIUMCOUNTER.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MEDIUMCOUNTER.CV), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MEDIUMCOUNTER.CU_T.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MEDIUMCOUNTER.CU_T.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MEDIUMCOUNTER.CU_T.CLK), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MEDIUMCOUNTER.CU_T.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MEDIUMCOUNTER.CU_T.M), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMALLCOUNTER.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMALLCOUNTER.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMALLCOUNTER.CU), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMALLCOUNTER.R), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMALLCOUNTER.PV), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMALLCOUNTER.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMALLCOUNTER.CV), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMALLCOUNTER.CU_T.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMALLCOUNTER.CU_T.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMALLCOUNTER.CU_T.CLK), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMALLCOUNTER.CU_T.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMALLCOUNTER.CU_T.M), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.ISSMALLBOX), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.ISMEDIUMBOX), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.ISBIGBOX), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGBOXTIMER.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGBOXTIMER.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGBOXTIMER.IN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGBOXTIMER.PT), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGBOXTIMER.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGBOXTIMER.ET), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGBOXTIMER.STATE), SINT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGBOXTIMER.PREV_IN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGBOXTIMER.CURRENT_TIME), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.BIGBOXTIMER.START_TIME), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MEDIUMBOXTIMER.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MEDIUMBOXTIMER.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MEDIUMBOXTIMER.IN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MEDIUMBOXTIMER.PT), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MEDIUMBOXTIMER.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MEDIUMBOXTIMER.ET), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MEDIUMBOXTIMER.STATE), SINT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MEDIUMBOXTIMER.PREV_IN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MEDIUMBOXTIMER.CURRENT_TIME), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.MEDIUMBOXTIMER.START_TIME), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMALLBOXTIMER.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMALLBOXTIMER.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMALLBOXTIMER.IN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMALLBOXTIMER.PT), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMALLBOXTIMER.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMALLBOXTIMER.ET), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMALLBOXTIMER.STATE), SINT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMALLBOXTIMER.PREV_IN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMALLBOXTIMER.CURRENT_TIME), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.SMALLBOXTIMER.START_TIME), TIME_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.STOPMOTORTIMER), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_GT6_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_AND12_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG1.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG1.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG1.CLK), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG1.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG1.M), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_GT7_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_LE9_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_AND17_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_AND13_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG2.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG2.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG2.CLK), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG2.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG2.M), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_LE8_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_GT49_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_AND50_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_AND14_OUT), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG3.EN), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG3.ENO), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG3.CLK), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG3.Q), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0.R_TRIG3.M), BOOL_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_ADD28_OUT), INT_ENUM},*)
(*DBG:    {&(RES0__INSTANCE0._TMP_OR46_OUT), BOOL_ENUM},*)
(*DBG:};*)
(*DBG:*)
(*DBG:#define VAR_COUNT               110*)
(*DBG:*)
(*DBG:uint16_t get_var_count(void)*)
(*DBG:{*)
(*DBG:    return VAR_COUNT;*)
(*DBG:}*)
(*DBG:*)
(*DBG:size_t get_var_size(size_t idx)*)
(*DBG:{*)
(*DBG:    if (idx >= VAR_COUNT)*)
(*DBG:    {*)
(*DBG:        return 0;*)
(*DBG:    }*)
(*DBG:    switch (debug_vars[idx].type) {*)
(*DBG:    case SINT_ENUM:*)
(*DBG:        return sizeof(SINT);*)
(*DBG:    case TIME_ENUM:*)
(*DBG:        return sizeof(TIME);*)
(*DBG:    case INT_ENUM:*)
(*DBG:    case INT_O_ENUM:*)
(*DBG:        return sizeof(INT);*)
(*DBG:    case BOOL_ENUM:*)
(*DBG:    case BOOL_O_ENUM:*)
(*DBG:        return sizeof(BOOL);*)
(*DBG:    default:*)
(*DBG:        return 0;*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)
(*DBG:void *get_var_addr(size_t idx)*)
(*DBG:{*)
(*DBG:    void *ptr = debug_vars[idx].ptr;*)
(*DBG:*)
(*DBG:    switch (debug_vars[idx].type) {*)
(*DBG:    case SINT_ENUM:*)
(*DBG:        return (void *)&((__IEC_SINT_t *) ptr)->value;*)
(*DBG:    case TIME_ENUM:*)
(*DBG:        return (void *)&((__IEC_TIME_t *) ptr)->value;*)
(*DBG:    case INT_ENUM:*)
(*DBG:        return (void *)&((__IEC_INT_t *) ptr)->value;*)
(*DBG:    case INT_O_ENUM:*)
(*DBG:        return (void *)((((__IEC_INT_p *) ptr)->flags & __IEC_FORCE_FLAG) *)
(*DBG:                        ? &(((__IEC_INT_p *) ptr)->fvalue) *)
(*DBG:                        : ((__IEC_INT_p *) ptr)->value);*)
(*DBG:    case BOOL_ENUM:*)
(*DBG:        return (void *)&((__IEC_BOOL_t *) ptr)->value;*)
(*DBG:    case BOOL_O_ENUM:*)
(*DBG:        return (void *)((((__IEC_BOOL_p *) ptr)->flags & __IEC_FORCE_FLAG) *)
(*DBG:                        ? &(((__IEC_BOOL_p *) ptr)->fvalue) *)
(*DBG:                        : ((__IEC_BOOL_p *) ptr)->value);*)
(*DBG:    default:*)
(*DBG:        return 0;*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)
(*DBG:void force_var(size_t idx, bool forced, void *val)*)
(*DBG:{*)
(*DBG:    void *ptr = debug_vars[idx].ptr;*)
(*DBG:*)
(*DBG:    if (forced) {*)
(*DBG:        size_t var_size = get_var_size(idx);*)
(*DBG:        switch (debug_vars[idx].type) {*)
(*DBG:        case SINT_ENUM: {*)
(*DBG:            memcpy(&((__IEC_SINT_t *) ptr)->value, val, var_size);*)
(*DBG:            //((__IEC_SINT_t *) ptr)->value = *((SINT *) val);*)
(*DBG:            ((__IEC_SINT_t *) ptr)->flags |= __IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:        case TIME_ENUM: {*)
(*DBG:            memcpy(&((__IEC_TIME_t *) ptr)->value, val, var_size);*)
(*DBG:            //((__IEC_TIME_t *) ptr)->value = *((TIME *) val);*)
(*DBG:            ((__IEC_TIME_t *) ptr)->flags |= __IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:        case INT_ENUM: {*)
(*DBG:            memcpy(&((__IEC_INT_t *) ptr)->value, val, var_size);*)
(*DBG:            //((__IEC_INT_t *) ptr)->value = *((INT *) val);*)
(*DBG:            ((__IEC_INT_t *) ptr)->flags |= __IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:        case INT_O_ENUM: {*)
(*DBG:            memcpy((((__IEC_INT_p *) ptr)->value), val, var_size);*)
(*DBG:            //*(((__IEC_INT_p *) ptr)->value) = *((INT *) val);*)
(*DBG:            ((__IEC_INT_p *) ptr)->flags |= __IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:        case BOOL_ENUM: {*)
(*DBG:            memcpy(&((__IEC_BOOL_t *) ptr)->value, val, var_size);*)
(*DBG:            //((__IEC_BOOL_t *) ptr)->value = *((BOOL *) val);*)
(*DBG:            ((__IEC_BOOL_t *) ptr)->flags |= __IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:        case BOOL_O_ENUM: {*)
(*DBG:            memcpy((((__IEC_BOOL_p *) ptr)->value), val, var_size);*)
(*DBG:            //*(((__IEC_BOOL_p *) ptr)->value) = *((BOOL *) val);*)
(*DBG:            ((__IEC_BOOL_p *) ptr)->flags |= __IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:        default:*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:    } else {*)
(*DBG:        switch (debug_vars[idx].type) {*)
(*DBG:        case SINT_ENUM:*)
(*DBG:            ((__IEC_SINT_t *) ptr)->flags &= ~__IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        case TIME_ENUM:*)
(*DBG:            ((__IEC_TIME_t *) ptr)->flags &= ~__IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        case INT_ENUM:*)
(*DBG:            ((__IEC_INT_t *) ptr)->flags &= ~__IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        case INT_O_ENUM:*)
(*DBG:            ((__IEC_INT_p *) ptr)->flags &= ~__IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        case BOOL_ENUM:*)
(*DBG:            ((__IEC_BOOL_t *) ptr)->flags &= ~__IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        case BOOL_O_ENUM:*)
(*DBG:            ((__IEC_BOOL_p *) ptr)->flags &= ~__IEC_FORCE_FLAG;*)
(*DBG:            break;*)
(*DBG:        default:*)
(*DBG:            break;*)
(*DBG:        }*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)
(*DBG:void swap_bytes(void *ptr, size_t size) *)
(*DBG:{*)
(*DBG:    uint8_t *bytePtr = (uint8_t *)ptr;*)
(*DBG:    size_t i;*)
(*DBG:    for (i = 0; i < size / 2; ++i) *)
(*DBG:    {*)
(*DBG:        uint8_t temp = bytePtr[i];*)
(*DBG:        bytePtr[i] = bytePtr[size - 1 - i];*)
(*DBG:        bytePtr[size - 1 - i] = temp;*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)
(*DBG:void trace_reset(void)*)
(*DBG:{*)
(*DBG:    for (size_t i=0; i < VAR_COUNT; i++) *)
(*DBG:    {*)
(*DBG:        force_var(i, false, 0);*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)
(*DBG:void set_trace(size_t idx, bool forced, void *val)*)
(*DBG:{*)
(*DBG:    if (idx >= 0 && idx < VAR_COUNT) *)
(*DBG:    {*)
(*DBG:        if (endianness == REVERSE_ENDIANNESS)*)
(*DBG:        {*)
(*DBG:            // Aaaaarghhhh... Stupid AVR is Big Endian.*)
(*DBG:            swap_bytes(val, get_var_size(idx));*)
(*DBG:        }*)
(*DBG:*)
(*DBG:        force_var(idx, forced, val);*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)
(*DBG:void set_endianness(uint8_t value)*)
(*DBG:{*)
(*DBG:    if (value == SAME_ENDIANNESS || value == REVERSE_ENDIANNESS)*)
(*DBG:    {*)
(*DBG:        endianness = value;*)
(*DBG:    }*)
(*DBG:}*)
(*DBG:*)