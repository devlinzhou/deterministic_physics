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


class GQuaternion
{
public:
    f32    x;
    f32    y;
    f32    z;
    f32    w;

public:
    GQuaternion() = default;
    GQuaternion( const GQuaternion& ) = default;
    inline GQuaternion( f32 fX, f32 fY, f32 fZ,f32 fW ): x(fX), y(fY),z(fZ),w(fW) { }
    inline GQuaternion( const f32* pf ): x(pf[0]), y(pf[1]),z(pf[2]),w(pf[3]) { }

    inline GQuaternion( const GVector3& VAxis, f32 fRot )
    {
        this->FromAxisAngle( VAxis, fRot );
    }
public:

    inline f32 operator [] ( const uint32_t i ) const{return *( &x + i );}

    inline f32& operator [] ( const uint32_t i ){ return *( &x + i ); }

    inline GQuaternion& operator = ( const GQuaternion& Q )
    {    
        x = Q.x; y = Q.y; z = Q.z; w = Q.w;
        return *this;
    }

    inline GQuaternion operator + ( const GQuaternion& Q ) const
    {
        return GQuaternion( x + Q.x, y + Q.y, z + Q.z, w + Q.w );
    }

    inline GQuaternion operator - ( const GQuaternion& Q ) const
    {
        return GQuaternion( x - Q.x, y - Q.y, z - Q.z, w - Q.w );
    }

    inline GQuaternion operator * ( const GQuaternion& QB ) const
    {
        GQuaternion Temp = GQuaternion(    
            QB.w * x + QB.x * w + QB.y * z - QB.z * y,
            QB.w * y - QB.x * z + QB.y * w + QB.z * x,
            QB.w * z + QB.x * y - QB.y * x + QB.z * w,
            QB.w * w - QB.x * x - QB.y * y - QB.z * z);

        return Temp;
    }

    inline GQuaternion& operator *= ( const GQuaternion& Q )
    {
        *this = *this * Q;
        return *this;
    }

    inline GQuaternion operator * ( f32 fScalar ) const
    {
        return GQuaternion( fScalar * x, fScalar * y, fScalar * z, fScalar * w );
    }

    inline GQuaternion& operator *= ( f32 fScalar )
    {
        *this = *this * fScalar;
        return *this;
    }

    friend inline GQuaternion operator * (f32 fScalar, const GQuaternion& Q)
    {
        return GQuaternion(fScalar * Q.x, fScalar * Q.y, fScalar * Q.z, fScalar * Q.w);
    }

    inline GQuaternion operator - ( ) const
    {
        return GQuaternion( -x, -y, -z, -w );
    }

    inline bool operator == ( const GQuaternion& Q ) const
    {
        return ( Q.x == x ) && ( Q.y == y ) && ( Q.z == z ) && ( Q.w == w );
    }

    inline bool operator != ( const GQuaternion& Q ) const
    {
        return !operator==( Q );
    }

public:

    inline f32* Ptr( ) {  return &x; }

    inline const f32* Ptr( ) const
    {
        return &x;
    }

    void FromAxisAngle(const GVector3& VAxis, const f32 R )
    {
        f32 fSin;
        f32 fCos;

        GMath::SinCos(R * GMath::Half(), fSin, fCos );
        w = fCos;
        x = fSin * VAxis.x;
        y = fSin * VAxis.y;
        z = fSin * VAxis.z;
    }

    void FromAxisAngle(const GVector3& VAxis, const f32 fSina, const f32 fCosa )
    {
        w = fCosa;
        x = fSina * VAxis.x;
        y = fSina * VAxis.y;
        z = fSina * VAxis.z;
    }

    void ToAxisAngle( GVector3& V3Out, f32& ROut ) const
    {
        f32 fSqrLength = x * x + y * y + z * z;
        if( fSqrLength > GMath::Zero() )
        {
            ROut = GMath::Two() * GMath::ACos( w );
            f32 fInvLength = GMath::InvSqrt( fSqrLength );
            V3Out.x = x * fInvLength;
            V3Out.y = y * fInvLength;
            V3Out.z = z * fInvLength;
        }
        else
        {
            ROut = GMath::Zero();
            V3Out.x = GMath::One();
            V3Out.y = GMath::Zero();
            V3Out.z = GMath::Zero();
        }
    }

    f32 Dot( const GQuaternion& Q ) const
    {
        return x * Q.x + y * Q.y + z * Q.z + w * Q.w;
    }

    f32 DotDirect( const GQuaternion& Q ) const
    {
        return z * Q.z + x * Q.x + y * Q.y;
    }

    f32 Size( ) const
    {
        return GMath::Sqrt( w * w + x * x + y * y + z * z );
    }

    f32 SizeSquare( ) const
    {
        return w * w + x * x + y * y + z * z;
    }

    void Normalize( )
    {
        f32 fLength = SizeSquare();
        if (fLength < GMath::Epsilon() )
        {
            Zero();
        }
        else
        {
            f32 fInvLength = GMath::InvSqrt(fLength);
            x = x * fInvLength;
            y = y * fInvLength;
            z = z * fInvLength;
            w = w * fInvLength;
        }
    }

    inline GQuaternion GetNormalize( ) const
    {
        GQuaternion Qtemp( *this );
        Qtemp.Normalize( );
        return Qtemp;
    }

    inline void Inverse( )
    {
        f32 fNorm = SizeSquare( );
        if( fNorm > GMath::Epsilon() )
        {
            f32 fInvNorm = GMath::One() / fNorm;            
            x    =   -x * fInvNorm;
            y    =   -y * fInvNorm;
            z    =   -z * fInvNorm;
            w    =    w * fInvNorm;
        }
        else
        {
            Zero( );
        }
    }

    inline GQuaternion GetInverse( ) const
    {
        GQuaternion TQ = *this;
        TQ.Inverse( );
        return TQ;
    }

    inline void UnitInverse( )
    {
        x    =    -x;
        y    =    -y;
        z    =    -z;
        //w    =    w;
    }

    inline GQuaternion GetUnitInverse( ) const
    {
        return GQuaternion( -x, -y, -z, w );
    }

    GQuaternion Exp( ) const
    {
        f32 RAngle = GMath::Sqrt( x * x + y * y + z * z );

        GQuaternion QuatTemp = Identity();

        QuatTemp.w = GMath::Cos( RAngle );
        f32 fSin = GMath::Sin( RAngle );

        if( GMath::Abs( fSin ) >= GMath::Epsilon() )
        {
            f32 fCoeff = fSin / RAngle;
            QuatTemp.x = fCoeff * x;
            QuatTemp.y = fCoeff * y;
            QuatTemp.z = fCoeff * z;
        }
        else
        {
            QuatTemp.x = x;
            QuatTemp.y = y;
            QuatTemp.z = z;
        }
        return QuatTemp;
    }

    GQuaternion Log( ) const
    {
        GQuaternion QuatTemp = Identity();
        QuatTemp.w = GMath::Zero();

        f32 fw2 = w * w;
        if( fw2 < GMath::One() )
        {
            f32 RAngle = GMath::ACos( w );

            f32 fSin = GMath::Sqrt( GMath::One() - fw2 );

            if( GMath::Abs( fSin ) >= GMath::Epsilon() )
            {
                f32 fCoeff = RAngle / fSin;
                QuatTemp.x = fCoeff * x;
                QuatTemp.y = fCoeff * y;
                QuatTemp.z = fCoeff * z;
                return QuatTemp;
            }
        }

        QuatTemp.x = x;
        QuatTemp.y = y;
        QuatTemp.z = z;
        return QuatTemp;
    }

    inline GVector3 RotateVector( const GVector3& V ) const
    {
        GVector3 Vqvec(x, y, z);
        GVector3 uv = GVector3::CrossProduct(Vqvec, V);
        GVector3 uuv = GVector3::CrossProduct(Vqvec, uv);

        uv *= GMath::Two() * w;
        uuv *= GMath::Two();

        return V + uv + uuv;
    }


    inline GVector3 UnRotateVector(const GVector3& V) const
    {
        GVector3 Vqvec(-x, -y, -z);
        GVector3 uv = GVector3::CrossProduct(Vqvec, V);
        GVector3 uuv = GVector3::CrossProduct(Vqvec, uv);

        uv *= GMath::Two() * w;
        uuv *= GMath::Two();

        return V + uv + uuv;
    }

    static inline GQuaternion Identity() { return GQuaternion(GMath::Zero(), GMath::Zero(), GMath::Zero(), GMath::One());}
    static inline GQuaternion Zero()     { return GQuaternion(GMath::Zero(), GMath::Zero(), GMath::Zero(), GMath::Zero()); }

    inline bool IsIdentity( )  const{return *this == Identity();}
    inline bool IsZero( )      const{return *this == Zero();}


     void Test();
};


