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


#include "glacier_math.h"
#include "glacier_vector.h"

/*
y 
    |
    |              5--------6
    |             /|       /|
    |            / |      / |
    |     z     4--------7  |
    |    /      |  1-----|--2
    |   /       | /      | /
    |  /        |/       |/
    | /         0--------3 
    |/
    0------------------------>x 
    */

enum GGlacierBoxFaceType
{
    GLACIER_BOX_FACE_NEGETIVE_Y    =    0,
    GLACIER_BOX_FACE_NEGETIVE_Z    =    1,
    GLACIER_BOX_FACE_NEGETIVE_X    =    2,
    GLACIER_BOX_FACE_POSITIVE_Z    =    3,
    GLACIER_BOX_FACE_POSITIVE_X    =    4,
    GLACIER_BOX_FACE_POSITIVE_Y    =    5,
    GLACIER_BOX_FACE_MAX           =    6,
};


class GAABB
{
public:

     GAABB( ) = default;
     GAABB( const GAABB&) = default;

     inline GAABB(const GVector3& V3)
     {
         m_VMin = V3;
         m_VMax = V3;
     }

    inline GAABB( const GVector3& VMin, const GVector3& VMax ) 
    {
        m_VMin = VMin;
        m_VMax = VMax;
    }

public:

    inline bool operator == ( const GAABB& Box ) const
    {
        return m_VMin == Box.m_VMin && m_VMax == Box.m_VMax;
    }

    inline bool operator != ( const GAABB& B ) const
    {
        return !( *this == B );
    }

    inline const GVector3& GetMin( void ) const { return m_VMin; }
    inline GVector3& GetMin( void ) { return m_VMin; }
    inline const GVector3& GetMax( void ) const { return m_VMax; }
    inline GVector3& GetMax( void ) { return m_VMax; }
    inline void SetMin( const GVector3& V ) { m_VMin = V; }
    inline void SetMax( const GVector3& V ) { m_VMax = V; }
    inline void SetBox( const GVector3& VMin, const GVector3& VMax )
    {
        m_VMin    = VMin;
        m_VMax    = VMax;
    }

    inline GVector3 GetCorner( uint32_t uIndex ) const
    {
        GVector3 TV;
        TV.x    =    uIndex & 1 ? m_VMax.x : m_VMin.x;
        TV.y    =    uIndex & 2 ? m_VMax.y : m_VMin.y;
        TV.z    =    uIndex & 4 ? m_VMax.z : m_VMin.z;

        return TV;
    }

    inline GVector3 GetLoopCorner( int32_t nIndex ) const
    {
        switch( nIndex )
        {
        case 0: return GVector3( m_VMin.x, m_VMin.y, m_VMin.z );
        case 1: return GVector3( m_VMin.x, m_VMin.y, m_VMax.z );
        case 2: return GVector3( m_VMax.x, m_VMin.y, m_VMax.z );
        case 3: return GVector3( m_VMax.x, m_VMin.y, m_VMin.z );
        case 4: return GVector3( m_VMin.x, m_VMax.y, m_VMin.z );
        case 5: return GVector3( m_VMin.x, m_VMax.y, m_VMax.z );
        case 6: return GVector3( m_VMax.x, m_VMax.y, m_VMax.z );
        case 7: return GVector3( m_VMax.x, m_VMax.y, m_VMin.z );
        }

        return GVector3::Zero( );    
    }

    inline void GetLoopCorner( GVector3* pVert ) const
    {
        pVert[0] = GVector3( m_VMin.x, m_VMin.y, m_VMin.z );
        pVert[1] = GVector3( m_VMin.x, m_VMin.y, m_VMax.z );
        pVert[2] = GVector3( m_VMax.x, m_VMin.y, m_VMax.z );
        pVert[3] = GVector3( m_VMax.x, m_VMin.y, m_VMin.z );
        pVert[4] = GVector3( m_VMin.x, m_VMax.y, m_VMin.z );
        pVert[5] = GVector3( m_VMin.x, m_VMax.y, m_VMax.z );
        pVert[6] = GVector3( m_VMax.x, m_VMax.y, m_VMax.z );
        pVert[7] = GVector3( m_VMax.x, m_VMax.y, m_VMin.z );
    }

    inline GVector3 GetFaceCenter( GGlacierBoxFaceType FaceType ) const
    {
        return GetHalfSize( ) * GetFaceNormal( FaceType ) + GetCenter( );
    }

    static GVector3 GetFaceNormal( GGlacierBoxFaceType FaceType );

    static inline bool IsFaceAdjacent( GGlacierBoxFaceType FaceType0, GGlacierBoxFaceType FaceType1 )
    {
        if( FaceType0 > FaceType1 )
        {
            auto T = FaceType0;
            FaceType0 = FaceType1;
            FaceType1 = FaceType0;
        }

        if( FaceType0 == GLACIER_BOX_FACE_NEGETIVE_X || FaceType1 ==  GLACIER_BOX_FACE_POSITIVE_X ) return false;
        if( FaceType0 == GLACIER_BOX_FACE_NEGETIVE_Y || FaceType1 ==  GLACIER_BOX_FACE_POSITIVE_Y ) return false;
        if( FaceType0 == GLACIER_BOX_FACE_NEGETIVE_Z || FaceType1 ==  GLACIER_BOX_FACE_POSITIVE_Z ) return false;
            
        return true;
    }

    static GGlacierBoxFaceType GetAdjacentFace( GGlacierBoxFaceType FaceType, int32_t Index );

    static GGlacierBoxFaceType GetFaceFromNormal( const GVector3& VDir );

    GVector3 GetAdjacentPoint( GGlacierBoxFaceType FaceType0, GGlacierBoxFaceType FaceType1 ) const;

    inline void Merge( const GAABB& Box )
    {
        m_VMax.MakeCeil( Box.m_VMax );
        m_VMin.MakeFloor( Box.m_VMin );
    }

    inline void Merge( const GVector3& V3 )
    {
        m_VMax.MakeCeil( V3 );
        m_VMin.MakeFloor( V3 );
    }

    inline void Move( const GVector3& V3 )
    {
        m_VMin += V3;
        m_VMax += V3;
    }

   // GBox TransByMatrix( const GMatrix4& M4 ) const;

   // GBox TransByAffineMatrix( const GMatrix4& M ) const;

    inline bool Intersects( const GAABB& OtherBox ) const
    {
        if( m_VMax.x < OtherBox.m_VMin.x || m_VMin.x > OtherBox.m_VMax.x ||
            m_VMax.y < OtherBox.m_VMin.y || m_VMin.y > OtherBox.m_VMax.y || 
            m_VMax.z < OtherBox.m_VMin.z || m_VMin.z > OtherBox.m_VMax.z )
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    inline GAABB GetIntersects( const GAABB& OtherBox ) const
    {
        GVector3 VMin = m_VMin;
        GVector3 VMax = m_VMax;

        VMin.MakeCeil( OtherBox.GetMin( ) );
        VMax.MakeFloor( OtherBox.GetMax( ) );

        if( VMin.x < VMax.x &&
            VMin.y < VMax.y &&
            VMin.z < VMax.z )
        {
            return GAABB( VMin, VMax );
        }

        return GAABB( GVector3::Zero(), GVector3::Zero());
    }

    inline f32 Volume( void ) const
    {
        GVector3 V3Temp = m_VMax - m_VMin;
        return V3Temp.x * V3Temp.y * V3Temp.z;
    }

    inline void Expand( const GVector3& V )
    {
        m_VMin -= V;
        m_VMax += V;
    }

    inline void Expand( f32 fDelta )
    {
        GVector3 TV( fDelta );
        Expand( TV );
    }

    inline void Scale( const GVector3& V3Scale )
    {
        m_VMin *= V3Scale;
        m_VMax *= V3Scale;
    }

    inline bool Intersects( const GVector3& SphereCenter, const f32 fRadius ) const
    {
        const GVector3&        Vmin    = GetMin( );
        const GVector3&        Vmax    = GetMax( );

        if( Vmin.x - SphereCenter.x > fRadius ) return false;
        if( SphereCenter.x - Vmax.x > fRadius ) return false;
        if( Vmin.y - SphereCenter.y > fRadius ) return false;
        if( SphereCenter.y - Vmax.y > fRadius ) return false;
        if( Vmin.z - SphereCenter.z > fRadius ) return false;
        if( SphereCenter.z - Vmax.z > fRadius ) return false;

        return true;
    }

    //bool Intersects( const GPlane& Plane ) const;

    inline bool Intersects( const GVector3& V3 ) const
    {
        return ( 
            V3.x >= m_VMin.x && V3.x <= m_VMax.x && 
            V3.y >= m_VMin.y && V3.y <= m_VMax.y && 
            V3.z >= m_VMin.z && V3.z <= m_VMax.z );
    }

    inline GVector3 GetCenter( void ) const     { return ( m_VMax + m_VMin ) * GMath::Half(); }
    inline GVector3 GetSize( void ) const       { return m_VMax - m_VMin; }
    inline GVector3 GetHalfSize( void ) const   { return GetSize( ) * GMath::Half(); }

    bool Contains( const GAABB& OtherBox ) const
    {
        return  m_VMin.x            <= OtherBox.m_VMin.x    &&
                m_VMin.y            <= OtherBox.m_VMin.y    &&
                m_VMin.z            <= OtherBox.m_VMin.z    &&
                OtherBox.m_VMax.x    <= m_VMax.x            &&
                OtherBox.m_VMax.y    <= m_VMax.y            &&
                OtherBox.m_VMax.z    <= m_VMax.z;
    }

    inline void SetZero( void )
    {
        m_VMax = GVector3::Zero();
        m_VMin = GVector3::Zero();
    }

//     GSphere GetSphere( void ) const
//     {
//         f32            fRadius    = GetHalfSize( ).GetLength( );
//         GVector3    VCenter    = GetCenter( );
//         return GSphere( VCenter, fRadius );
//     }

  //  GPosNormal GetClosestPoint( const GVector3& VPos ) const;

//     static GVector3 GetMaxBB( )
//     {
//         return GVector3( GLACIER_MAX_BB, GLACIER_MAX_BB, GLACIER_MAX_BB );
//     }
// 
//     static GVector3 GetMinBB( )
//     {
//         return GVector3( GLACIER_MIN_BB, GLACIER_MIN_BB, GLACIER_MIN_BB );
//     }


public:

    GVector3            m_VMin;
    GVector3            m_VMax;
};
