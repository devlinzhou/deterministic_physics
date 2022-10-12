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

#include "glacier_time.h"
#include <thread>
#include <map>

#if UseProfiler_RDTSCP

static double CountCpuGhz() {

    Myclock::time_point tStart = Myclock::now();;
    uint64_t uStart = GTimer::get_CPUCycles();

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    uint64_t uEnd = GTimer::get_CPUCycles();
    Myclock::time_point tEnd = Myclock::now();

    double time = double(std::chrono::duration_cast<Myres>(tEnd - tStart).count() * 1e-9);

    double CpuGhz = double(uEnd - uStart) / (time * 1000000000);
    return CpuGhz;

}

double GTimer::InvCPUGHZ = 0.000001f / CountCpuGhz();
#endif

static std::map<std::string, float> ms_TotalTime;

GTimeProfiler::GTimeProfiler(const char* Str )
{
    m_Str = Str;
    m_Time.Start();
}
GTimeProfiler::~GTimeProfiler()
{
    
}

void GTimeProfiler::EndCuptrue()
{
    char name[128];
    float fTime = (float)m_Time.GetDeltaTimeMS() * 1000.f;
    sprintf(name, "Time:% 8.3fms %s\n", fTime, m_Str.c_str());

    std::map<std::string, float>::iterator iterm = ms_TotalTime.find(m_Str);
    if (iterm != ms_TotalTime.end())
    {
        iterm->second += fTime;
    }
    else
    {
        ms_TotalTime[m_Str] = fTime;
    }


    if (m_bOutToScreen)
    {
        //g_GlobalSystem.FontToScreen( name );
    }
    else
    {
        // OutputDebugStringA( name );
    }

    m_bCapture = true;
}

void GTimeProfiler::ClearTime()
{
    std::map<std::string, float>::iterator iterm = ms_TotalTime.begin();
    for (; iterm != ms_TotalTime.end(); ++iterm)
    {
        iterm->second = 0;
    }
}

void GTimeProfiler::DebugOut()
{
    char name[128];
    std::map<std::string, float>::iterator iterm = ms_TotalTime.begin();
    for (; iterm != ms_TotalTime.end(); ++iterm)
    {
        sprintf(name, "Time:% 8.3fms %s\n", iterm->second, iterm->first.c_str());
       // OutputDebugStringA(name);
    }

    //OutputDebugStringA("=========================================\n");
}
