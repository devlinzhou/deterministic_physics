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

class GPlane
{
public:

    enum GPlaneSide
    {
        NO_SIDE = 0,
        POSITIVE,
        NEGATIVE,
        BOTH
    };

public:

    GPlane( ) = default;
    GPlane( const GPlane& P ) = default;

    inline GPlane(const GVector3& V3Normal, f32 fDis )
    {
        m_Normal = V3Normal;
        m_fDis   = fDis;
    }

    inline GPlane( const GVector3& V3Normal, const GVector3& V3Point )
    {
        Redefine( V3Normal, V3Point );
    }

    inline GPlane( const GVector3& V0, const GVector3& V1, const GVector3& V2 )
    {
        Redefine( V0, V1, V2 );
    }

public:
    
    bool operator == ( const GPlane& P ) const
    {
        return ( P.m_fDis == m_fDis && P.m_Normal == m_Normal );
    }

    bool operator != ( const GPlane& P ) const
    {
        return ( P.m_fDis != m_fDis || P.m_Normal != m_Normal );
    }

    GPlane operator + (const GPlane& P) const
    {
        return GPlane( m_Normal + P.m_Normal, m_fDis + P.m_fDis);
    }

    GPlane operator - (const GPlane& P) const
    {
        return GPlane(m_Normal - P.m_Normal, m_fDis - P.m_fDis);
    }

    void Normalize()
    {
        f32 fLength = GVector3::DotProduct(m_Normal, m_Normal) + m_fDis * m_fDis;
        if (fLength < GMath::Epsilon())
        {
            GPlane( GVector3::UnitX(), GMath::Zero());
        }
        else
        {
            f32 fInvLength = GMath::InvSqrt(fLength);
            m_Normal *= fInvLength;
            m_fDis *= fInvLength;
        }
    }

public:

    inline GPlane Flip( void ) const
    {
        return GPlane( -m_Normal, -m_fDis );
    }

    inline f32 GetDistance(const GVector3& V3Point) const
    {
        return GVector3::DotProduct(m_Normal, V3Point) + m_fDis;
    }

    inline GPlaneSide GetSide( const GVector3& V3Point, f32 Radius = GMath::Epsilon() ) const
    {
        f32 fDistance = GetDistance( V3Point );

        if( fDistance < -Radius )
        {
            return GPlane::NEGATIVE;
        }
        else if( fDistance > Radius )
        {
            return GPlane::POSITIVE;
        }
        else
        {
            return GPlane::NO_SIDE;
        }
    }

    inline GPlaneSide GetSide( const GVector3& VCenter, const GVector3& VHalfSize ) const
    {
        f32 fDist = GetDistance( VCenter );

        f32 fMaxAbsDist = m_Normal.AbsDotProduct( VHalfSize );

        if( fDist < -fMaxAbsDist )
        {
            return GPlane::NEGATIVE;
        }

        if( fDist > +fMaxAbsDist )
        {
            return GPlane::POSITIVE;
        }

        return GPlane::BOTH;
    }
   
    inline void Redefine( const GVector3& V0, const GVector3& V1, const GVector3& V2 )
    {
        GVector3 VEdge1 = V1 - V0;
        GVector3 VEdge2 = V2 - V0;
        m_Normal    = GVector3::CrossProduct( VEdge1, VEdge2 ).GetNormalize();
        m_fDis      =-GVector3::DotProduct(m_Normal, V0 );
    }

    inline void Redefine( const GVector3& VNormal, const GVector3& V3Point )
    {
        m_Normal    = VNormal;
        m_fDis      = -GVector3::DotProduct(m_Normal, V3Point );
    }

    inline void Normalise( void )
    {
        f32 fLength = m_Normal.SizeSquare( );

        if( fLength > GMath::Epsilon() )
        {
            f32 fInvLength = GMath::InvSqrt( fLength);
            m_Normal = m_Normal * fInvLength;
            m_fDis   = m_fDis * fInvLength;
        }
        else
        {
            m_Normal = GVector3::Zero();
            m_fDis   = GMath::Zero();
        }
    }

    inline GPlane GetNormalise( ) const
    {
        GPlane TP = *this;
        TP.Normalise( );
        return TP;
    }

    inline GVector3 ProjectVector(const GVector3& V) const
    {
        return V - m_Normal * GVector3::DotProduct(m_Normal, V);
    }
  
    inline GVector3 ReflectPoint( const GVector3& V ) const
    {
        return V - m_Normal * ( GMath::Two() * GetDistance( V ) );
    }

    inline GVector3 ReflectVector( const GVector3& V ) const
    {
        return V - m_Normal * ( GMath::Two() * GVector3::DotProduct(m_Normal, V) );
    }

public:

    GVector3    m_Normal;
    f32         m_fDis;
};

