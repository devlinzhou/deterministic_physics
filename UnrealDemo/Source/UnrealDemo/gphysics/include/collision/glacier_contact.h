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

#include "glacier_transform_qt.h"
#include "glacier_collision_object.h"
#include <vector>
#include <map>



class GManifoldPoint
{
public:

    GVector3    m_PosOnSurfaceB_World;
    GVector3    m_NormalOnB;
    f32         m_depth;
};



class GCollisionContact
{
public:

    GCollisionContact() = default;
    GCollisionContact(const GCollisionContact&) = default;


    void AddContactPoint( const GVector3& PosOnSurfaceB_World, const GVector3& NormalOnB, f32 depth );

    void Clear()
    {
        PairId = 0;
        m_nPointCount = 0;
    }

    void ClearPoint()
    {
        m_nPointCount = 0;
    }

    uint64_t PairId;


    static constexpr int32_t MaxPoint = 4;

    int32_t         m_nPointCount;
    GManifoldPoint  m_Point[MaxPoint];


    GVector3 VTest;
};


class GContactManerger
{
public:

    GContactManerger() 
    {
    
    }

    void Add( const GCollisionContact& TContact )
    {
        std::map<uint64_t, uint32_t>::iterator iter = m_Finder.find( TContact.PairId);
        if( iter != m_Finder.end() )
        {
            m_Contacts[iter->second] = TContact;// todo update
        }
        else
        {
            uint32_t    FreedId = -1;
            for( int i = 0; i < m_Contacts.size(); ++i) // todo opimization
            {
                if( m_Finder.find( m_Contacts[i].PairId ) == m_Finder.end() )
                {
                    FreedId = i;
                    break;
                }
            }

            if( FreedId != -1 )
            {
                m_Finder[TContact.PairId] = FreedId;
                m_Contacts[FreedId] = TContact;
            }
            else
            {
                m_Contacts.push_back(TContact);
                m_Finder[TContact.PairId] = m_Contacts.size() - 1;
            }
        }
    }

    void Delete( uint64_t PairId )
    {
        std::map<uint64_t, uint32_t>::iterator iter = m_Finder.find(PairId);
        if (iter != m_Finder.end())
        {
            m_Contacts[iter->second].Clear();
            m_Finder.erase(iter);
        }
    }

    std::vector<GCollisionContact>  m_Contacts;
    std::map<uint64_t, uint32_t>    m_Finder; // todo optimization
         
};