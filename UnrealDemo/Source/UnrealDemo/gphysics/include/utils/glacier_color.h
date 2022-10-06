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

#include <stdint.h>

// ARGB color
class GColor
{
public:

    GColor( const GColor& )  = default;

    GColor( uint32_t value ): RawValue(value)
    {
    }

    static GColor White()   { return GColor(0xFFFFFFFF); }
    static GColor Black()   { return GColor(0xFF000000); }
    static GColor Red()     { return GColor(0xFFFF0000); }
    static GColor Green()   { return GColor(0xFF00FF00); }
    static GColor Blue()    { return GColor(0xFF0000FF); }
    static GColor Yellow()  { return GColor(0xFFFFFF00); }



    uint32_t RawValue;
};