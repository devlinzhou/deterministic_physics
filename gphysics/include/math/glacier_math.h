/* 
 * Copyright (C) 2022 zhou xuan, Email: zhouxuan6676@gmail.com
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at * 
 * http://www.apache.org/licenses/LICENSE-2.0 * 
 * Unless required by applicable law or agreed to in writing, software 
 * distributed under the License is distributed on an "AS IS" BASIS, 
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
 * See the License for the specific language governing permissions and 
 * limitations under the License. 
 */
#pragma once

#include "glacier_float.h"


#define  Deterministic_Float

#ifdef Deterministic_Float
typedef GFloat f32;
#else
typedef float f32;
#endif 


class GMath 
{
public:

    GMath( void )
	{

	}

public:

#ifdef  Deterministic_Float

    static constexpr f32 Zero()     { return GFloat::Zero(); }
    static constexpr f32 One()      { return GFloat::One(); }
    static constexpr f32 Two()      { return GFloat::Two(); }
    static const f32 Three()    { return GFloat(3,0,2); }
    static constexpr f32 Half()     { return GFloat::Half(); }
    static const f32 Epsilon()  { return GFloat(0,1,1000000); }
    static constexpr f32 Pi()       { return GFloat::Pi(); }
    static constexpr f32 Pi_Half()  { return GFloat::Pi_Half(); }
    static constexpr f32 Pi_Two()   { return GFloat::Pi_Two(); }
    static constexpr f32 Pi_Quarter(){return GFloat::Pi_Quarter(); }

    static inline f32 Abs( const f32 value )    { return GFloat::Abs(value); }
    static inline f32 Sin( const f32 value )    { return GFloat::Sin(value); }
    static inline f32 Cos( const f32 value )    { return GFloat::Cos(value); }
    static inline f32 ASin(const f32 value)     { return GFloat::ASin(value); }
    static inline f32 ACos(const f32 value)     { return GFloat::ACos(value); }
    static inline f32 Tan(const f32 value)      { return GFloat::Tan(value); }
    static inline f32 ATan(const f32 value)     { return GFloat::ATan(value); }
    static inline f32 Exp(const f32 value)      { return GFloat::Exp(value); }
    static inline f32 Log(const f32 value)      { return GFloat::Log(value); }
    static inline f32 Log2(const f32 value)     { return GFloat::Log2(value); }
    static inline f32 Log10(const f32 value)    { return GFloat::Log10(value); }
    static inline f32 Pow2(const f32 value)     { return GFloat::Pow2(value); }
    static inline f32 InvSqrt(const f32 value)  { return GFloat::InvSqrt(value); }
    static inline f32 Sqrt(const f32 value)     { return GFloat::Sqrt(value); }
    static inline f32 ATan2(const f32 y, const f32 x)                   { return GFloat::ATan2(y,x); }
    static inline f32 Pow(const f32 base, const f32 exponent)           { return GFloat::Pow(base, exponent); }
    static inline void SinCos(const f32 value, f32& OutSin, f32& OutCos){ GFloat::SinCos(value, OutSin, OutCos);}
    
#else

    static constexpr f32 Zero()         { return 0; }
    static constexpr f32 One()          { return 1.f;}
    static constexpr f32 Two()          { return 2.f;}
    static constexpr f32 Three()        { return 3.f;}
    static constexpr f32 Half()         { return 0.5f; }
    static constexpr f32 Epsilon()      { return 0.000001f; }
    static constexpr f32 Pi()           { return 3.14159265359f; }
    static constexpr f32 Pi_Half()      { return Pi() * 0.5f; }
    static constexpr f32 Pi_Two()       { return Pi() * 2.f; }
    static constexpr f32 Pi_Quarter()   { return Pi() * 0.25f; }


    static inline f32   Abs(const f32 value)                    { return abs(value); }
    static inline f32   Sin(const f32 value)                    { return sinf(value); }
    static inline f32   Cos(const f32 value)                    { return cosf(value); }  
    static inline f32   ASin(const f32 value)                   { return asinf(value); }
    static inline f32   ACos(const f32 value)                   { return acosf(value); }
    static inline f32   Tan(const f32 value)                    { return tanf(value); }
    static inline f32   ATan(const f32 value)                   { return atanf(value); }
    static inline f32   ATan2(const f32 y, const f32 x)         { return atan2f(y, x); }
    static inline f32   Exp(const f32 value)                    { return expf(value); }
    static inline f32   Log(const f32 value)                    { return logf(value); }
    static inline f32   Log2(const f32 value)                   { return log2f(value); }
    static inline f32   Log10(const f32 value)                  { return log10(value); }
    static inline f32   Pow2(const f32 value)                   { return powf(2.f, value); }
    static inline f32   Pow(const f32 base, const f32 exponent) { return powf(base, exponent); }
    static inline f32   Sqrt(const f32 value)                   { return sqrtf(value); }
    static inline f32   InvSqrt(const f32 value)                { return 1.f / sqrtf(value); }
    static inline void  SinCos(const f32 value, f32& OutSin, f32& OutCos) { OutSin = sinf(value); OutCos = cosf(value); }
#endif 

     static inline f32 Clamp(const f32 Value, const f32 fMin, const f32 fMax)
     {
        return Value > fMax ? fMax : (Value < fMin ? fMin : Value );
     }
     static inline f32 Min(const f32 fa, const f32 fb) { return fa < fb ? fa : fb; }
     static inline f32 Max(const f32 fa, const f32 fb) { return fa > fb ? fa : fb; }
     static inline f32 Min3(const f32 fa, const f32 fb, const f32 fc ) { return Min(fa,Min(fb,fc)); }
     static inline f32 Max3(const f32 fa, const f32 fb, const f32 fc ) { return Max(fa,Max(fb,fc)); }
    

};

