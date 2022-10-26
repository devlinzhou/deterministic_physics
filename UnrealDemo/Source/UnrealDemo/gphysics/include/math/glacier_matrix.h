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
#include "glacier_transform_qt.h"

class GMatrix4;
class GMatrix3
{
public:

    inline GMatrix3( ){ };

    inline GMatrix3( const GMatrix3& M3 ) = default;

    GMatrix3( const GMatrix4& M4 );

    GMatrix3(
        f32 f00, f32 f01, f32 f02,
        f32 f10, f32 f11, f32 f12,
        f32 f20, f32 f21, f32 f22 )
    {
        m[0][0] = f00; m[0][1] = f01;  m[0][2] = f02;
        m[1][0] = f10; m[1][1] = f11;  m[1][2] = f12;
        m[2][0] = f20; m[2][1] = f21;  m[2][2] = f22;
    }

    inline GMatrix3( const GQuaternion& Q )
    {
        f32 fTx = GMath::Two() * Q.x;
        f32 fTy = GMath::Two() * Q.y;
        f32 fTz = GMath::Two() * Q.z;

        f32 fTwx = fTx * Q.w;
        f32 fTwy = fTy * Q.w;
        f32 fTwz = fTz * Q.w;
        f32 fTxx = fTx * Q.x;
        f32 fTxy = fTy * Q.x;
        f32 fTxz = fTz * Q.x;
        f32 fTyy = fTy * Q.y;
        f32 fTyz = fTz * Q.y;
        f32 fTzz = fTz * Q.z;

        m[0][0] = GMath::One() - (fTyy + fTzz);
        m[0][1] = fTxy + fTwz;
        m[0][2] = fTxz - fTwy;

        m[1][0] = fTxy - fTwz;
        m[1][1] = GMath::One() - (fTxx + fTzz);
        m[1][2] = fTyz + fTwx;

        m[2][0] = fTxz + fTwy;
        m[2][1] = fTyz - fTwx;
        m[2][2] = GMath::One() - (fTxx + fTyy);
    }

public:

    inline f32* operator [] ( uint32_t nRow ) const
    {
        return ( f32* ) m[nRow];
    }

    bool operator == ( const GMatrix3& M ) const
    {
        return(
            m[0][0] == M.m[0][0] && m[0][1] == M.m[0][1] && m[0][2] == M.m[0][2] &&
            m[1][0] == M.m[1][0] && m[1][1] == M.m[1][1] && m[1][2] == M.m[1][2] &&
            m[2][0] == M.m[2][0] && m[2][1] == M.m[2][1] && m[2][2] == M.m[2][2] );
    }

    inline bool operator != ( const GMatrix3& M3 ) const
    {
        return ! ( *this == M3 );
    }

    inline GMatrix3 operator + ( const GMatrix3& M3 ) const
    {
        return GMatrix3 (
            m[0][0] + M3.m[0][0],
            m[0][1] + M3.m[0][1],
            m[0][2] + M3.m[0][2],
            m[1][0] + M3.m[1][0],
            m[1][1] + M3.m[1][1],
            m[1][2] + M3.m[1][2],
            m[2][0] + M3.m[2][0],
            m[2][1] + M3.m[2][1],
            m[2][2] + M3.m[2][2]);
    }

    inline GMatrix3 operator - ( const GMatrix3& M3 ) const
    {
        return GMatrix3(
            m[0][0] - M3.m[0][0],
            m[0][1] - M3.m[0][1],
            m[0][2] - M3.m[0][2],
            m[1][0] - M3.m[1][0],
            m[1][1] - M3.m[1][1],
            m[1][2] - M3.m[1][2],
            m[2][0] - M3.m[2][0],
            m[2][1] - M3.m[2][1],
            m[2][2] - M3.m[2][2]);
    }

    inline GMatrix3 operator * ( const GMatrix3& M3 ) const
    {
        GMatrix3 Reuslt;

        Reuslt.m[0][0] = m[0][0] * M3.m[0][0] + m[0][1] * M3.m[1][0] + m[0][2] * M3.m[2][0];
        Reuslt.m[0][1] = m[0][0] * M3.m[0][1] + m[0][1] * M3.m[1][1] + m[0][2] * M3.m[2][1];
        Reuslt.m[0][2] = m[0][0] * M3.m[0][2] + m[0][1] * M3.m[1][2] + m[0][2] * M3.m[2][2];

        Reuslt.m[1][0] = m[1][0] * M3.m[0][0] + m[1][1] * M3.m[1][0] + m[1][2] * M3.m[2][0];
        Reuslt.m[1][1] = m[1][0] * M3.m[0][1] + m[1][1] * M3.m[1][1] + m[1][2] * M3.m[2][1];
        Reuslt.m[1][2] = m[1][0] * M3.m[0][2] + m[1][1] * M3.m[1][2] + m[1][2] * M3.m[2][2];

        Reuslt.m[2][0] = m[2][0] * M3.m[0][0] + m[2][1] * M3.m[1][0] + m[2][2] * M3.m[2][0];
        Reuslt.m[2][1] = m[2][0] * M3.m[0][1] + m[2][1] * M3.m[1][1] + m[2][2] * M3.m[2][1];
        Reuslt.m[2][2] = m[2][0] * M3.m[0][2] + m[2][1] * M3.m[1][2] + m[2][2] * M3.m[2][2];

        return Reuslt;
    }

    inline GMatrix3 operator - ( ) const
    {
        return GMatrix3(
            -m[0][0], -m[0][1], -m[0][2],
            -m[1][0], -m[1][1], -m[1][2], 
            -m[2][0], -m[2][1], -m[2][2]);
    }

    inline GMatrix3 operator * ( f32 fScalar ) const
    {
        return GMatrix3(
            fScalar * m[0][0],
            fScalar * m[0][1],
            fScalar * m[0][2],
            fScalar * m[1][0],
            fScalar * m[1][1],
            fScalar * m[1][2],
            fScalar * m[2][0],
            fScalar * m[2][1],
            fScalar * m[2][2]);
    }

    friend inline GMatrix3 operator * (f32 fScalar, const GMatrix3& M3)
    {
        return M3 * fScalar;
    }

    inline GVector3 TransformVector( const GVector3& V )
    {
        return GVector3(
            V.x * m[0][0] + V.y * m[1][0] + V.z * m[2][0],
            V.x * m[0][1] + V.y * m[1][1] + V.z * m[2][1],
            V.x * m[0][2] + V.y * m[1][2] + V.z * m[2][2] );
    }

public:

    static inline GMatrix3 Zero( ) 
    {
        return GMatrix3(
            GMath::Zero(), GMath::Zero(), GMath::Zero(),
            GMath::Zero(), GMath::Zero(), GMath::Zero(),
            GMath::Zero(), GMath::Zero(), GMath::Zero() );
    }

    static inline GMatrix3 Identity( )
    {
        return GMatrix3(
            GMath::One(),   GMath::Zero(),  GMath::Zero(),
            GMath::Zero(),  GMath::One(),   GMath::Zero(),
            GMath::Zero(),  GMath::Zero(),  GMath::One());
    }

    inline GVector3 GetRow(uint32_t nRow)
    {
        return  *(GVector3*)(m[nRow]);
    }

    inline const GVector3& GetRow(uint32_t nRow) const
    {
        return  *(GVector3*)(m[nRow]);
    }

    inline GVector3 GetColumn( uint32_t nCol ) const
    {
        return GVector3( m[0][nCol], m[1][nCol], m[2][nCol] );
    }

    inline void SetColumn( uint32_t nCol, const GVector3& V3 )
    {
        m[0][nCol] = V3.x;
        m[1][nCol] = V3.y;
        m[2][nCol] = V3.z;
    }

    inline void FromAxesColumn( const GVector3& V3X, const GVector3& V3Y, const GVector3& V3Z )
    {
        SetColumn( 0, V3X );
        SetColumn( 1, V3Y );
        SetColumn( 2, V3Z );
    }

    inline GMatrix3 Transpose( void ) const
    {
        GMatrix3 M3temp;

        M3temp[0][0] = m[0][0];
        M3temp[0][1] = m[1][0];
        M3temp[0][2] = m[2][0];

        M3temp[1][0] = m[0][1];
        M3temp[1][1] = m[1][1];
        M3temp[1][2] = m[2][1];

        M3temp[2][0] = m[0][2];
        M3temp[2][1] = m[1][2];
        M3temp[2][2] = m[2][2];

        return M3temp;
    }

    bool Inverse( GMatrix3& M3Out, f32 fTolerance = GMath::Epsilon() ) const;

    GMatrix3 GetInverse( f32 fTolerance = GMath::Epsilon() ) const
    {
        GMatrix3 M3;
        if (Inverse(M3, fTolerance))
        {
            return M3;
        }
        else
        {
            return GMatrix3::Identity();
        }
    }

    f32 Determinant( ) const
    {
        f32 fCofactor00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
        f32 fCofactor10 = m[1][2] * m[2][0] - m[1][0] * m[2][2];
        f32 fCofactor20 = m[1][0] * m[2][1] - m[1][1] * m[2][0];

        return m[0][0] * fCofactor00 + m[0][1] * fCofactor10 + m[0][2] * fCofactor20;
    }

    void FromAxisAngle( const GVector3& VAxis, const f32& fRadians)
    {
        GQuaternion TRot( VAxis, fRadians );
        *this = GMatrix3( TRot ); 
    }

public:

    f32 m[3][3];
};


class GMatrix4
{
public:
    
    inline GMatrix4( ) = default;

    inline GMatrix4( const GMatrix4& M4 ) = default;

    inline GMatrix4(
        f32 m00, f32 m01, f32 m02, f32 m03,
        f32 m10, f32 m11, f32 m12, f32 m13,
        f32 m20, f32 m21, f32 m22, f32 m23,
        f32 m30, f32 m31, f32 m32, f32 m33 )
    {
        m[0][0] = m00;    m[0][1] = m01;    m[0][2] = m02;    m[0][3] = m03;
        m[1][0] = m10;    m[1][1] = m11;    m[1][2] = m12;    m[1][3] = m13;
        m[2][0] = m20;    m[2][1] = m21;    m[2][2] = m22;    m[2][3] = m23;
        m[3][0] = m30;    m[3][1] = m31;    m[3][2] = m32;    m[3][3] = m33;
    }

    inline GMatrix4( const GMatrix3& M3 )
    {
        m[0][0] = M3.m[0][0];    m[0][1] = M3.m[0][1];    m[0][2] = M3.m[0][2];    m[0][3] = GMath::Zero();
        m[1][0] = M3.m[1][0];    m[1][1] = M3.m[1][1];    m[1][2] = M3.m[1][2];    m[1][3] = GMath::Zero();
        m[2][0] = M3.m[2][0];    m[2][1] = M3.m[2][1];    m[2][2] = M3.m[2][2];    m[2][3] = GMath::Zero();
        m[3][0] = GMath::Zero(); m[3][1] = GMath::Zero(); m[3][2] = GMath::Zero(); m[3][3] = GMath::One();
    }

public:

    inline f32* operator [] ( uint32_t nRow )
    {
        return m[nRow];
    }

    inline const f32* const operator [] ( uint32_t nRow ) const
    {
        return m[nRow];
    }

    inline GMatrix4 operator * ( f32 fScalar ) const
    {
        return GMatrix4(
            fScalar * m[0][0], fScalar * m[0][1], fScalar * m[0][2], fScalar * m[0][3],
            fScalar * m[1][0], fScalar * m[1][1], fScalar * m[1][2], fScalar * m[1][3],
            fScalar * m[2][0], fScalar * m[2][1], fScalar * m[2][2], fScalar * m[2][3],
            fScalar * m[3][0], fScalar * m[3][1], fScalar * m[3][2], fScalar * m[3][3] );
    }

    friend inline GMatrix4 operator * (f32 fScalar, const GMatrix4& M4)
    {
        return M4 * fScalar;
    }
     
    inline GMatrix4 operator * ( const GMatrix4& M2 ) const
    {
        GMatrix4 m4temp;
        m4temp.m[0][0] = m[0][0] * M2.m[0][0] + m[0][1] * M2.m[1][0] + m[0][2] * M2.m[2][0] + m[0][3] * M2.m[3][0];
        m4temp.m[0][1] = m[0][0] * M2.m[0][1] + m[0][1] * M2.m[1][1] + m[0][2] * M2.m[2][1] + m[0][3] * M2.m[3][1];
        m4temp.m[0][2] = m[0][0] * M2.m[0][2] + m[0][1] * M2.m[1][2] + m[0][2] * M2.m[2][2] + m[0][3] * M2.m[3][2];
        m4temp.m[0][3] = m[0][0] * M2.m[0][3] + m[0][1] * M2.m[1][3] + m[0][2] * M2.m[2][3] + m[0][3] * M2.m[3][3];

        m4temp.m[1][0] = m[1][0] * M2.m[0][0] + m[1][1] * M2.m[1][0] + m[1][2] * M2.m[2][0] + m[1][3] * M2.m[3][0];
        m4temp.m[1][1] = m[1][0] * M2.m[0][1] + m[1][1] * M2.m[1][1] + m[1][2] * M2.m[2][1] + m[1][3] * M2.m[3][1];
        m4temp.m[1][2] = m[1][0] * M2.m[0][2] + m[1][1] * M2.m[1][2] + m[1][2] * M2.m[2][2] + m[1][3] * M2.m[3][2];
        m4temp.m[1][3] = m[1][0] * M2.m[0][3] + m[1][1] * M2.m[1][3] + m[1][2] * M2.m[2][3] + m[1][3] * M2.m[3][3];

        m4temp.m[2][0] = m[2][0] * M2.m[0][0] + m[2][1] * M2.m[1][0] + m[2][2] * M2.m[2][0] + m[2][3] * M2.m[3][0];
        m4temp.m[2][1] = m[2][0] * M2.m[0][1] + m[2][1] * M2.m[1][1] + m[2][2] * M2.m[2][1] + m[2][3] * M2.m[3][1];
        m4temp.m[2][2] = m[2][0] * M2.m[0][2] + m[2][1] * M2.m[1][2] + m[2][2] * M2.m[2][2] + m[2][3] * M2.m[3][2];
        m4temp.m[2][3] = m[2][0] * M2.m[0][3] + m[2][1] * M2.m[1][3] + m[2][2] * M2.m[2][3] + m[2][3] * M2.m[3][3];

        m4temp.m[3][0] = m[3][0] * M2.m[0][0] + m[3][1] * M2.m[1][0] + m[3][2] * M2.m[2][0] + m[3][3] * M2.m[3][0];
        m4temp.m[3][1] = m[3][0] * M2.m[0][1] + m[3][1] * M2.m[1][1] + m[3][2] * M2.m[2][1] + m[3][3] * M2.m[3][1];
        m4temp.m[3][2] = m[3][0] * M2.m[0][2] + m[3][1] * M2.m[1][2] + m[3][2] * M2.m[2][2] + m[3][3] * M2.m[3][2];
        m4temp.m[3][3] = m[3][0] * M2.m[0][3] + m[3][1] * M2.m[1][3] + m[3][2] * M2.m[2][3] + m[3][3] * M2.m[3][3];

        return m4temp;
    }

    inline GMatrix4& operator *= ( const GMatrix4& M4 )
    {
        GMatrix4 MT = *this * M4;
        *this = MT;
        return *this;
    }

    inline GMatrix4 operator + ( const GMatrix4& M ) const
    {
        return GMatrix4(
            m[0][0] + M.m[0][0], m[0][1] + M.m[0][1], m[0][2] + M.m[0][2], m[0][3] + M.m[0][3],
            m[1][0] + M.m[1][0], m[1][1] + M.m[1][1], m[1][2] + M.m[1][2], m[1][3] + M.m[1][3],
            m[2][0] + M.m[2][0], m[2][1] + M.m[2][1], m[2][2] + M.m[2][2], m[2][3] + M.m[2][3],
            m[3][0] + M.m[3][0], m[3][1] + M.m[3][1], m[3][2] + M.m[3][2], m[3][3] + M.m[3][3]);
    }

    inline GMatrix4 operator - ( const GMatrix4& M ) const
    {
        return GMatrix4(
            m[0][0] - M.m[0][0], m[0][1] - M.m[0][1], m[0][2] - M.m[0][2], m[0][3] - M.m[0][3],
            m[1][0] - M.m[1][0], m[1][1] - M.m[1][1], m[1][2] - M.m[1][2], m[1][3] - M.m[1][3],
            m[2][0] - M.m[2][0], m[2][1] - M.m[2][1], m[2][2] - M.m[2][2], m[2][3] - M.m[2][3],
            m[3][0] - M.m[3][0], m[3][1] - M.m[3][1], m[3][2] - M.m[3][2], m[3][3] - M.m[3][3]);
    }

    inline bool operator == ( const GMatrix4& M ) const
    {
        if( m[0][0] != M.m[0][0] || m[0][1] != M.m[0][1] || m[0][2] != M.m[0][2] || m[0][3] != M.m[0][3] ||
            m[1][0] != M.m[1][0] || m[1][1] != M.m[1][1] || m[1][2] != M.m[1][2] || m[1][3] != M.m[1][3] ||
            m[2][0] != M.m[2][0] || m[2][1] != M.m[2][1] || m[2][2] != M.m[2][2] || m[2][3] != M.m[2][3] ||
            m[3][0] != M.m[3][0] || m[3][1] != M.m[3][1] || m[3][2] != M.m[3][2] || m[3][3] != M.m[3][3] )
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    inline bool operator != ( const GMatrix4& M ) const
    {
        return !( *this == M );
    }

    inline GMatrix4& operator = ( const GMatrix3& M3 )
    {
        m[0][0] = M3.m[0][0];       m[0][1] = M3.m[0][1];       m[0][2] = M3.m[0][2];       m[0][3] = GMath::Zero();
        m[1][0] = M3.m[1][0];       m[1][1] = M3.m[1][1];       m[1][2] = M3.m[1][2];       m[1][3] = GMath::Zero();
        m[2][0] = M3.m[2][0];       m[2][1] = M3.m[2][1];       m[2][2] = M3.m[2][2];       m[2][3] = GMath::Zero();
        m[3][0] = GMath::Zero();    m[3][1] = GMath::Zero();    m[3][2] = GMath::Zero();    m[3][3] = GMath::One();

        return *this;
    }

    inline GMatrix4 Transpose( void ) const
    {
        return GMatrix4 (
            m[0][0], m[1][0], m[2][0], m[3][0],
            m[0][1], m[1][1], m[2][1], m[3][1],
            m[0][2], m[1][2], m[2][2], m[3][2],
            m[0][3], m[1][3], m[2][3], m[3][3] );
    }

    inline void SetTrans( const GVector3& V3 )
    {
        m[3][0] = V3.x; m[3][1] = V3.y; m[3][2] = V3.z;
    }

    inline GVector3 GetTrans( void ) const
    {
        return GVector3( m[3][0], m[3][1], m[3][2] );
    }

    inline GVector3 GetScale( void ) const
    {
        return GVector3( m[0][0], m[1][1], m[2][2] );
    }

    inline void SetScale( const GVector3& V )
    {
        m[0][0] = V.x; m[1][1] = V.y; m[2][2] = V.z;
    }

    inline void SetScale( f32 fX, f32 fY, f32 fZ )
    {
        m[0][0] = fX; m[1][1] = fY; m[2][2] = fZ;
    }

    GQuaternion ToQuat( void ) const;

    GMatrix4 Adjoint( void ) const;

    f32 Determinant( void ) const;

    f32 Inverse( );

    inline GMatrix4 GetInverse( void ) const
    {
        GMatrix4 TM4 = *this;
        TM4.Inverse( );
        return TM4;
    }

    void MakeTransformMatrix(const GVector3& V3Scale, const GQuaternion& Rot, const GVector3& VTranslation );

    GMatrix4 TransposeAdjoint( void ) const;

    void RemoveScaling( void );

    static inline GMatrix4 Zero(void)
    {
        return GMatrix4(
            GMath::Zero(), GMath::Zero(), GMath::Zero(), GMath::Zero(),
            GMath::Zero(), GMath::Zero(), GMath::Zero(), GMath::Zero(),
            GMath::Zero(), GMath::Zero(), GMath::Zero(), GMath::Zero(),
            GMath::Zero(), GMath::Zero(), GMath::Zero(), GMath::Zero());
    }

    static inline GMatrix4 Identity( void )
    {
        return GMatrix4(
            GMath::One(),   GMath::Zero(),  GMath::Zero(),  GMath::Zero(),
            GMath::Zero(),  GMath::One(),   GMath::Zero(),  GMath::Zero(),
            GMath::Zero(),  GMath::Zero(),  GMath::One(),   GMath::Zero(),
            GMath::Zero(),  GMath::Zero(),  GMath::Zero(),  GMath::One() );
    }

    inline GVector3 TransformPosition( const GVector3& V3 ) const
    {
        return GVector3(
            m[0][0] * V3.x + m[1][0] * V3.y + m[2][0] * V3.z + m[3][0],
            m[0][1] * V3.x + m[1][1] * V3.y + m[2][1] * V3.z + m[3][1],
            m[0][2] * V3.x + m[1][2] * V3.y + m[2][2] * V3.z + m[3][2]);
    }

    inline GVector3 TransformNormal( const GVector3& V3 ) const
    {
        return GVector3(
            m[0][0] * V3.x + m[1][0] * V3.y + m[2][0] * V3.z,
            m[0][1] * V3.x + m[1][1] * V3.y + m[2][1] * V3.z,
            m[0][2] * V3.x + m[1][2] * V3.y + m[2][2] * V3.z);
    }

public:


    f32 m[4][4];
};


