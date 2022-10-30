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

#include "glacier_vector.h"
#include <chrono>
#include <string>


#if defined(__GNUC__) && defined(__x86_64__) && !defined(__aarch64__)
#include <cpuid.h>
#endif

#if defined(_MSC_VER) || (defined(__GNUC__))
#define UseProfiler_RDTSCP 1
#endif


#define  GProfilerFun GTimeProfiler GProfiler(__FUNCTION__);

typedef std::chrono::high_resolution_clock Myclock;
typedef std::chrono::nanoseconds Myres;

class  GTimer
{
public:
    GTimer()
#if UseProfiler_RDTSCP
        : start_(0), end_(0)
#else
        : t1(Myres::zero()), t2(Myres::zero())
#endif
    {
        Start();
    }

    ~GTimer()
    {}

    static inline uint64_t get_CPUCycles()
    {
#ifdef _MSC_VER
        return __rdtsc();
#elif __GNUC__    

#if defined(__x86_64__)
        unsigned int lo, hi;
        __asm__ __volatile__("rdtsc" : "=a" (lo), "=d" (hi));
        return ((uint64_t)hi << 32) | lo;
#elif defined(__aarch64__)

        uint64_t virtual_timer_value;
        asm volatile("mrs %0, cntvct_el0" : "=r"(virtual_timer_value));
        return virtual_timer_value;

#else
        return 0;
#endif

#else
        return 0;
#endif
    }

    void Start()
    {
#if UseProfiler_RDTSCP
        start_ = get_CPUCycles();
#else
        t1 = Myclock::now();
#endif    
    }

    void End()
    {
#if UseProfiler_RDTSCP
        end_ = get_CPUCycles();
#else
        t2 = Myclock::now();
#endif
    }


    float GetDeltaTimeMS_NoEnd()
    {
#if UseProfiler_RDTSCP
        return float(double(end_ - start_) * InvCPUGHZ);
#else
        return float(std::chrono::duration_cast<Myres>(t2 - t1).count() * 1e-6);
#endif
    }

    float GetDeltaTimeMS()
    {
        End();
        return GetDeltaTimeMS_NoEnd();
    }

    static double GetCpuFrequency_Compute()
    {
#if UseProfiler_RDTSCP
        return 1 / InvCPUGHZ;
#else
        return 0;
#endif
    }

    static int GetCpuFrequency_CpuInfo()
    {
        int cpuInfo[4] = { 0, 0, 0, 0 };
#ifdef _MSC_VER
        __cpuid(cpuInfo, 0);
        if (cpuInfo[0] >= 0x16) {
            __cpuid(cpuInfo, 0x16);
            return cpuInfo[0];
        }
#elif __GNUC__

#if defined(__x86_64__) && !defined(__aarch64__)

        __cpuid(0, cpuInfo[0], cpuInfo[1], cpuInfo[2], cpuInfo[3]);

        if (cpuInfo[0] >= 0x16) {
            __cpuid(0x16, cpuInfo[0], cpuInfo[1], cpuInfo[2], cpuInfo[3]);
            return cpuInfo[0];
        }
#elif defined(__ARM_ARCH)

        uint64_t freq;
        asm volatile("mrs %0, cntfrq_el0" : "=r" (freq));
        return (int)(freq / 1000000);

#endif

#else

#endif

        return 0;
    }

private:

#if UseProfiler_RDTSCP

    static double InvCPUGHZ;
    volatile uint64_t start_;
    volatile uint64_t end_;
#else
    Myclock::time_point t1;
    Myclock::time_point t2;
#endif

};

class GTimeProfiler
{
public:
    GTimeProfiler(const char* Str);
    ~GTimeProfiler();

    void EndCuptrue();
    static void DebugOut();
    static void ClearTime();

    std::string     m_Str;
    bool            m_bCapture;
    GTimer          m_Time;
    bool            m_bOutToScreen;
};