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
#include "glacier_quaternion.h"
#include "glacier_aabb.h"

class GTransform_QT
{
public:

    GTransform_QT() = default;

    GTransform_QT(const GTransform_QT&) = default;

    inline GTransform_QT( const GQuaternion& Quat, const GVector3& Trans ) :
        m_Rot(Quat ), m_Pos(Trans)
    {

    }

    inline GTransform_QT( const GQuaternion& Quat ) :
        m_Rot(Quat ), m_Pos( GVector3::Identity() )
    {

    }

    inline GTransform_QT( const GVector3& Trans ) :
        m_Rot( GQuaternion::Identity() ), m_Pos(Trans)
    {

    }

    inline GTransform_QT operator*( const GTransform_QT& Q ) const
    {
        return GTransform_QT( m_Rot * Q.m_Rot, Q.m_Rot.RotateVector(m_Pos) + Q.m_Pos );
    }

    inline GTransform_QT operator*( const GQuaternion& Q ) const
    {
        return GTransform_QT( m_Rot * Q,  Q.RotateVector(m_Pos) );
    }

    friend inline GTransform_QT operator*( const GQuaternion& Q, const GTransform_QT& QT )
    {
        return GTransform_QT( Q * QT.m_Rot, QT.m_Pos );
    }

    GVector3 TransformNormal( const GVector3& V ) const
    {
        return m_Rot.RotateVector( V );
    }

    GVector3 TransformPosition(const GVector3& V) const
    {
        return m_Rot.RotateVector(V) + m_Pos;
    }

    GAABB TransformAABB( const GAABB& InBox ) const
    {
        GAABB TBox( TransformPosition( InBox.GetCorner(0)) );
        for (uint32_t uLoop = 1; uLoop < 8; ++uLoop)
        {
            TBox.Merge( TransformPosition( InBox.GetCorner(uLoop)));
        }
        return TBox;
    }

public:

    inline void Inverse( )
    {
        m_Rot.Inverse( );
        m_Pos = m_Rot.RotateVector( -m_Pos);
    }

    inline GTransform_QT GetInverse( ) const
    {
        GQuaternion TQ = m_Rot.GetInverse( );
        return GTransform_QT( TQ, TQ.RotateVector( -m_Pos) );
    }

    inline GTransform_QT GetInverse_fast() const
    {
        GQuaternion TQ = m_Rot.GetUnitInverse();
        return GTransform_QT(TQ, TQ.RotateVector(-m_Pos));
    }

    bool IsIdentity( ) const
    {
        return m_Rot == GQuaternion::Identity( ) && m_Pos == GVector3::Zero( );
    }

    static inline GTransform_QT Identity( )
    {
        return GTransform_QT( GQuaternion::Identity(), GVector3::Zero( ) );
    }

public:

    GQuaternion     m_Rot;
    GVector3		m_Pos;
};

