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
#include "glacier_physics_world.h"
#include "glacier_vector.h"
#include "glacier_plane.h"
#include "glacier_transform_qt.h"
#include "glacier_collision_shape.h"
#include "glacier_physics_utils.h"
#include "glacier_debug_draw.h"
#include "glacier_collision_object.h"
#include "glacier_rigid_body.h"
#include "glacier_time.h"

GGridPosition s_BroadPhaseNeighbourt[13] =
{
    GGridPosition(1, -1, -1),
    GGridPosition(1,  0, -1),
    GGridPosition(1,  1, -1),
    GGridPosition(1, -1,  0),
    GGridPosition(1,  0,  0),
    GGridPosition(1,  1,  0),
    GGridPosition(1, -1,  1),
    GGridPosition(1,  0,  1),
    GGridPosition(1,  1,  1),
    GGridPosition(0,  1, -1),
    GGridPosition(0,  1,  0),
    GGridPosition(0,  1,  1),
    GGridPosition(0,  0,  1)
};

void GGridCell::DebugDraw(IGlacierDraw* pDraw, uint32_t mask) const
{
    if( mask & GPDraw_CeilBox)
        pDraw->DrawBox( GTransform_QT::Identity(), GetCenter(), GetHalfSize(), GColor::Gray() );

    for (int32_t i = 0; i < (int32_t)m_Objects.size(); ++i)
    {
        const GCObject* pObject = m_Objects[i];

        GPhysicsWorld::DebugDrawObject( pDraw, pObject, mask );
    }
}

bool GGridCell::AddCollisionObject(GCObject* pObject)
{
    if (std::find(m_Objects.begin(), m_Objects.end(), pObject) != m_Objects.end())
    {
        return false;
    }

    m_Objects.push_back(pObject);
    
    return true;
}

bool GGridCell::RemoveObject(GCObject* pObject)
{
    std::vector<GCObject*>::iterator iter = std::find(m_Objects.begin(), m_Objects.end(), pObject);
    if (iter != m_Objects.end())
    {
        m_Objects.erase(iter);

        return true;
    }

    return false;
}


GVector3 GBroadPhasePair::GetWorldRelativeB(const GVector3& VPos) const
{
    GVector3 V1 = GVector3::Zero();
    if (pObjectA->GetCOType() == CO_Rigid_Body && ((GRigidBody*)pObjectA)->m_bDynamic)
    {
        GRigidBody* pRA = (GRigidBody*)pObjectA;
        V1 = pRA->GetWorldPosVelocity(VPos);
    }

    GVector3 V2 = GVector3::Zero();
    if (pObjectB->GetCOType() == CO_Rigid_Body && ((GRigidBody*)pObjectB)->m_bDynamic)
    {
        GRigidBody* pRB = (GRigidBody*)pObjectB;
        V2 = pRB->GetWorldPosVelocity(VPos);
    }

    return V1 - V2;
}


f32 GBroadPhasePair::GetContactPairEnergy() const
{
    f32 fEnergyA = GMath::Zero();
    if (pObjectA->GetCOType() == CO_Rigid_Body)
    {
        GRigidBody* pRA = (GRigidBody*)pObjectA;
        fEnergyA = pRA->GetEnergy();
    }

    f32 fEnergyB = GMath::Zero();
    if (pObjectB->GetCOType() == CO_Rigid_Body)
    {
        GRigidBody* pRB = (GRigidBody*)pObjectB;
        fEnergyB = pRB->GetEnergy();
    }

    return fEnergyA + fEnergyB;
}


f32 GBroadPhasePair::GetContactPairMomentum_world() const
{
    f32 fMomentumA = GMath::Zero();
    if (pObjectA->GetCOType() == CO_Rigid_Body)
    {
        GRigidBody* pRA = (GRigidBody*)pObjectA;
        fMomentumA = pRA->GetMomentum();
    }

    f32 fMomentumB = GMath::Zero();
    if (pObjectB->GetCOType() == CO_Rigid_Body)
    {
        GRigidBody* pRB = (GRigidBody*)pObjectB;
        fMomentumB = pRB->GetMomentum();
    }

    return fMomentumA + fMomentumB;
}

void GBroadPhasePair::SeparatePair( GRigidBody* pRA, GRigidBody* pRB, bool bSwap )
{
    int32_t nPointCount = PairContact.GetPointCount();
    if(nPointCount <= 0)
        return;

    GVector3 VSep = GVector3::Zero();

    if( nPointCount == 1 )
    {
        GManifoldPoint TPoint = PairContact.m_Point[0];

        GVector3 VNormal = bSwap ? -TPoint.m_Normal : TPoint.m_Normal;

        VSep = VNormal * TPoint.m_depth;
    }
    else
    {
        GVector3 VNormal = GVector3::Zero();

        for( int32_t i = 0; i < nPointCount; ++i )
        {
            GManifoldPoint TPoint = PairContact.m_Point[i];

            VNormal += bSwap ? -TPoint.m_Normal : TPoint.m_Normal;
        }
    
        VNormal.Normalize();

        f32 fDis = GMath::Zero();
        for (int32_t i = 0; i < nPointCount; ++i)
        {
            GManifoldPoint TPoint = PairContact.m_Point[i];

            GVector3 VTNormal = bSwap ? -TPoint.m_Normal : TPoint.m_Normal;

            f32 fCos = GVector3::DotProduct( VNormal, VTNormal );

            f32 fT2 = GMath::Max(fCos, GMath::Makef32(0,1,10));

            fT2 = TPoint.m_depth / fT2;

            if( fT2 < fDis ) 
            {
                fDis = fT2;
            }
        }

        VSep = VNormal * fDis;
    }

    if (pRA != nullptr && pRB != nullptr)
    {
        f32 fAlphaA = pRB->m_Mass / (pRA->m_Mass + pRB->m_Mass);
        f32 fAlphaB = GMath::One() - fAlphaA;

        pRA->m_Transform.m_Pos -= VSep * fAlphaA;
        pRB->m_Transform.m_Pos += VSep * fAlphaB;
    }
    else if (pRA != nullptr)
    {
        pRA->m_Transform.m_Pos -= VSep;
    }
    else if (pRB != nullptr)
    {
        pRB->m_Transform.m_Pos += VSep;
    }
}

bool GPhysicsWorld::AddCollisionObject(GCObject* pObject)
{
    if (std::find(m_Objects.begin(), m_Objects.end(), pObject) != m_Objects.end())
    {
        return false;
    }

    m_Objects.push_back(pObject);
    m_ObjectMap[pObject->GetId()] = pObject;

    GVector3 VPos = pObject->m_Transform.m_Pos;

    GGridPosition CellPos;

    CellPos.x = GMath::FloorToInt(VPos.x / m_nCellWide);
    CellPos.y = GMath::FloorToInt(VPos.y / m_nCellWide);
    CellPos.z = GMath::FloorToInt(VPos.z / m_nCellHeight);

    GGridCell* pCell = nullptr;

    std::map<GGridPosition, GGridCell*>::iterator iter = m_Grids.find(CellPos);

    if (iter != m_Grids.end())
    {
        pCell = iter->second;
    }
    else
    {
        pCell = new GGridCell(CellPos, m_nCellWide, m_nCellHeight);
        m_Grids[CellPos] = pCell;
    }

    if (pCell != nullptr)
    {
        pCell->AddCollisionObject(pObject);

        pObject->m_pGridCell = pCell;
    }

    return true;
}

bool GPhysicsWorld::DeleteCollisionObject( GCObject* pObject )
{
    auto iter = std::find(m_Objects.begin(), m_Objects.end(), pObject);

    if ( iter == m_Objects.end())
    {
        return false;
    }

    m_Objects.erase( iter);
    m_ObjectMap.erase( pObject->GetId() );

    if( pObject->m_pGridCell != nullptr )
    {
        pObject->m_pGridCell->RemoveObject( pObject );
    }

    return true;
}

const GCObject* GPhysicsWorld::FindCollisionObject(uint32_t id) const
{
    auto iter = m_ObjectMap.find( id );
    if( iter != m_ObjectMap.end() )
    {
        return iter->second;
    }


    for (int32_t i = 0; i < (int32_t)m_StaticLargeObj.size(); ++i)
    {
        GCObject* pObject = m_StaticLargeObj[i];

        if( pObject->GetId() == id )
        {
            return pObject;
        }
    }


    return nullptr;
}

bool GPhysicsWorld::AddStaticLargeObj(GCObject* pObject )
{
    if (std::find(m_StaticLargeObj.begin(), m_StaticLargeObj.end(), pObject) != m_StaticLargeObj.end())
    {
        return false;
    }

    m_StaticLargeObj.push_back(pObject);

    return true;
}

bool GPhysicsWorld::DeleteStaticLargeObj( GCObject* pObject )
{
    auto iter = std::find(m_StaticLargeObj.begin(), m_StaticLargeObj.end(), pObject);

    if (iter == m_StaticLargeObj.end())
    {
        return false;
    }

    m_StaticLargeObj.erase(iter);

    return true;
}


bool GPhysicsWorld::UpdateCollisionObject(GCObject* pObject)
{
    GVector3 VPos = pObject->m_Transform.m_Pos;

    GGridPosition newCellPos = GetGridPos(VPos);

    if( pObject->m_pGridCell == nullptr || ( pObject->m_pGridCell != nullptr && pObject->m_pGridCell->m_pos != newCellPos ) )
    {
        GGridCell* pNewCell = nullptr;

        std::map<GGridPosition, GGridCell*>::iterator iter = m_Grids.find(newCellPos);

        if (iter != m_Grids.end())
        {
            pNewCell = iter->second;
        }
        else
        {
            pNewCell = new GGridCell(newCellPos, m_nCellWide, m_nCellHeight);
            m_Grids[newCellPos] = pNewCell;
        }

        pNewCell->AddCollisionObject( pObject );

        if (pObject->m_pGridCell != nullptr)
        {
            pObject->m_pGridCell->RemoveObject(pObject);
        }

        pObject->m_pGridCell = pNewCell;
    }

    return true;
}

void GPhysicsWorld::PreTick()
{
    GPRORILER_FUN
}


void GPhysicsWorld::Tick(f32 DetltaTime)
{
    GPRORILER_FUN
    
    PreTick();

    Simulate(DetltaTime);

    CollisionBroadPhase();

    CollisionNarrowPhase();

    SolveContactConstraint();

    UpdateSceneGrid();
 
    PostTick();
}

void GPhysicsWorld::PostTick()
{
    GTimeProfiler::DebugOut();
}

void GPhysicsWorld::Simulate( f32 DetltaTime )
{
    for (int32_t i = 0; i < (int32_t)m_Objects.size(); ++i)
    {
        GCObject* pObject = m_Objects[i];
        if (pObject->m_CollisionType == ECollisionObjectType::CO_Rigid_Body)
        {
            GRigidBody* pDynamicRigid = (GRigidBody*)pObject;

            if( pDynamicRigid->m_bDynamic )
            {
                pDynamicRigid->Tick_PreTransform(DetltaTime);
                pDynamicRigid->UpdateAABB();
            }
        }
    }


}

void GPhysicsWorld::DebugDrawObject( IGlacierDraw* pDraw, const GCObject* pObj, uint32_t mask )
{
    const GCObject* pObject = pObj;
    if (mask & GPDraw_Shape)
    {
        GColor TColor = GColor::Yellow(); 
        if( pObj->m_ContactArray.size() != 0 )
        {
            TColor = GColor::Red(); 
        }
        GPhyscsUtils::DrawShape(pObject->m_Transform, pObject->m_Shape, pDraw, TColor);
    }

    if (mask & GPDraw_LocalBox)
    {
        GAABB TAABB = pObject->GetLocalAABB();
        pDraw->DrawBox(pObject->m_Transform, TAABB.GetCenter(), TAABB.GetHalfSize(), GColor::White());
    }

    if (mask & GPDraw_WorldBox)
    {
        GAABB TAABB = pObject->GetAABB();
        pDraw->DrawBox(GTransform_QT::Identity(), TAABB.GetCenter(), TAABB.GetHalfSize(), GColor::Gray());
    }

    if((mask & GPDraw_Momentum) && pObject->IsDynamic() )
    {
        GRigidBody* pRigid = (GRigidBody*)pObject;

        GVector3 VOffset = GVector3::UnitZ() * GMath::Two();

        GTransform_QT Trans = pObject->m_Transform;
        Trans.m_Pos += VOffset;

        pDraw->DrawArrow(pObject->m_Transform.m_Pos, pRigid->GetLinearMomentum(),  GColor::Gray());
        pDraw->DrawArrow(pObject->m_Transform.m_Pos, pRigid->GetAngularMomentum(), GColor::Red());

    }

}

void GPhysicsWorld::DebugDraw(IGlacierDraw* pDraw, uint32_t mask ) const
{
    GPRORILER_FUN

    for( std::map<GGridPosition, GGridCell*>::const_iterator iter = m_Grids.begin(); iter != m_Grids.end(); ++iter )
    {
        iter->second->DebugDraw(pDraw, mask);
    }

    if (mask & GPDraw_Contact)
    {
        for (uint32_t i = 0; i < (uint32_t)m_BroadPhasePairs.size(); ++i)
        {
            const GBroadPhasePair& TestPair = m_BroadPhasePairs[i];
            GPhyscsUtils::DrawContact( TestPair.PairContact, pDraw, GColor::White(), this );
        }
    }

    for( int32_t i = 0; i < (int32_t)m_StaticLargeObj.size(); ++i )
    {
        DebugDrawObject( pDraw, m_StaticLargeObj[i], mask );
    }
}

void GPhysicsWorld::CollisionBroadPhase( )
{
    GPRORILER_FUN

    ClearContactPair();

    for (std::map<GGridPosition, GGridCell*>::const_iterator iterA = m_Grids.begin(); iterA != m_Grids.end(); ++iterA)
    {
        // ceil 
        for (int32_t i = 0; i < (int32_t)iterA->second->m_Objects.size(); ++i)
        {
            GCObject* pObjectA = iterA->second->m_Objects[i];

            const GAABB& BoxA = pObjectA->GetAABB();
            for (int32_t j = 0; j < i; ++j)
            {
                GCObject* pObjectB = iterA->second->m_Objects[j];
                const GAABB& BoxB = pObjectB->GetAABB();
                if (BoxA.Intersects(BoxB))
                {
                    AddContactPair(pObjectA, pObjectB);
                }
            }

            for ( int32_t nLargeObj = 0; nLargeObj < (int32_t)m_StaticLargeObj.size(); ++nLargeObj )
            {
                GCObject* pLargeObj = m_StaticLargeObj[nLargeObj];
                const GAABB& BoxB = pLargeObj->GetAABB();
                if (BoxA.Intersects(BoxB))
                {
                    AddContactPair(pObjectA, pLargeObj);
                }
            }
        }
        // other ceil

        for (int32_t i = 0; i < 13; ++i)
        {
            GGridPosition TPos = s_BroadPhaseNeighbourt[i] + iterA->first;

            std::map<GGridPosition, GGridCell*>::const_iterator iterB = m_Grids.find(TPos);

            if (iterB == m_Grids.end())
                continue;

            for (int32_t LoopA = 0; LoopA < (int32_t)iterA->second->m_Objects.size(); ++LoopA)
            {
                GCObject*       pObjectA    = iterA->second->m_Objects[LoopA];
                const GAABB&    BoxA        = pObjectA->GetAABB();


                for (int32_t LoopB = 0; LoopB < (int32_t)iterB->second->m_Objects.size(); ++LoopB)
                {
                    GCObject* pObjectB = iterB->second->m_Objects[LoopB];
                    const GAABB& BoxB = pObjectB->GetAABB();
                    if (BoxA.Intersects(BoxB))
                    {
                        AddContactPair(pObjectA, pObjectB);
                    }
                }
            }
        }
    }
}

void GPhysicsWorld::CollisionNarrowPhase( )
{
    GPRORILER_FUN
    for( int32_t i = 0; i < (int32_t)m_Objects.size(); ++i )
    {
        m_Objects[i]->ClearContactPair();
    }

    for (int32_t nLargeObj = 0; nLargeObj < (int32_t)m_StaticLargeObj.size(); ++nLargeObj)
    {
        m_StaticLargeObj[nLargeObj]->ClearContactPair();
    }
        
    for( int32_t i = 0; i < (int32_t)m_BroadPhasePairs.size(); ++i )
    {
        GBroadPhasePair& TestPair = m_BroadPhasePairs[i];

        int32_t ShapeTypeA = (int32_t)TestPair.pObjectA->m_Shape.ShapType;
        int32_t ShapeTypeB = (int32_t)TestPair.pObjectB->m_Shape.ShapType;

        GCollisionAlgorithm* pAlgorithm = m_CollisionManager.GetAlgrithm( ShapeTypeA, ShapeTypeB );

        if( pAlgorithm != nullptr )
        {
            TestPair.PairContact.ClearPoint( );
            pAlgorithm->ProcessCollision( TestPair.pObjectA, TestPair.pObjectB, &TestPair.PairContact );

            if( TestPair.PairContact.GetPointCount() > 0 )
            {
                TestPair.pObjectA->AddContactPair(TestPair.PairId, i);
                TestPair.pObjectB->AddContactPair(TestPair.PairId, i);
            }
        }
    }
}

GVector3 GetWorldRelative( const GVector3& VPos, GCObject* pObjectA, GCObject* pObjectB )
{
    GVector3 V1 = GVector3::Zero();
    if (pObjectA->GetCOType() == CO_Rigid_Body)
    {
        GRigidBody* pRA = (GRigidBody*)pObjectA;
        V1 = pRA->GetWorldPosVelocity( VPos );
    }

    GVector3 V2 = GVector3::Zero();
    if (pObjectB->GetCOType() == CO_Rigid_Body)
    {
        GRigidBody* pRB = (GRigidBody*)pObjectB;
        V2 = pRB->GetWorldPosVelocity(VPos);
    }

    return V1 - V2;

}

void GPhysicsWorld::SolveContactConstraint( GBroadPhasePair& pPair )
{
    int32_t nPointCount = pPair.PairContact.GetPointCount();
    if( nPointCount <= 0)
        return;

    GRigidBody* pRA = (pPair.pObjectA->IsDynamic() && pPair.pObjectA->GetCOType() == CO_Rigid_Body) ? (GRigidBody*)pPair.pObjectA : nullptr;
    GRigidBody* pRB = (pPair.pObjectB->IsDynamic() && pPair.pObjectB->GetCOType() == CO_Rigid_Body) ? (GRigidBody*)pPair.pObjectB : nullptr;

    if(pRA != nullptr || pRB != nullptr )
    {
        bool bSwap = pPair.PairContact.PointOnSurface == pPair.pObjectA->GetId() ? false : true;

        f32 fMomentum = GMath::Zero();

        for (int32_t i = 0; i < nPointCount; ++i)
        {
            GManifoldPoint& TPoint = pPair.PairContact.m_Point[i];

            GVector3 VNormal = bSwap ? -TPoint.m_Normal : TPoint.m_Normal;

            f32 EquivalentMassA = GMath::Zero();
            f32 EquivalentMassB = GMath::Zero();

            if (pRA != nullptr)
                EquivalentMassA = pRA->GetEquivalentMass(TPoint.m_PosWorld, VNormal);

            if (pRB != nullptr)
                EquivalentMassB = pRB->GetEquivalentMass(TPoint.m_PosWorld, VNormal);

            f32 TotalMass = EquivalentMassA + EquivalentMassB;

            f32 VWorldVelocity = GVector3::DotProduct( pPair.GetWorldRelativeB(TPoint.m_PosWorld), VNormal);

            f32 TMomentum = TotalMass * GMath::Abs(VWorldVelocity) ;
           
            if( fMomentum < TMomentum )
            {
                fMomentum = TMomentum;
            }
        }

        f32 factorA = fMomentum * GMath::Makef32(0, 1, 10);
        f32 factorB = -factorA;

        f32 TPairEnergy = pPair.GetContactPairEnergy() * m_CollisionEnergyLost;

        f32 PairEnergy = TPairEnergy / f32(nPointCount);
   
        for( int32_t nLoop = 0; nLoop < 20; nLoop ++ )
        {
            bool bSeparate = true;

            for (int32_t i = 0; i < nPointCount; ++i)
            {
                GManifoldPoint& TPoint = pPair.PairContact.m_Point[i];

                GVector3 VNormal = bSwap ? -TPoint.m_Normal : TPoint.m_Normal;

                GVector3 VWorldVelocity0 = pPair.GetWorldRelativeB( TPoint.m_PosWorld);

              //  if (GVector3::DotProduct(VWorldVelocity0, VNormal) > GMath::Zero())
               // {
                  //  continue;
               // }

                GPlane TPlane( VNormal, GMath::Zero());

                GVector3 VFriNormal = TPlane.ProjectVector( VWorldVelocity0 );
                f32 fSq = VFriNormal.SizeSquare();
                if( fSq < (m_FrictionVelocity * m_FrictionVelocity) )
                {
                    VFriNormal = GVector3::Zero();
                }
                else
                {
                    VFriNormal.Normalize();
                }

                if( pRA != nullptr )
                    pRA->AddImpulse_World(TPoint.m_PosWorld, ( VNormal - VFriNormal * m_Friction ) * factorA );

                if( pRB != nullptr )
                    pRB->AddImpulse_World(TPoint.m_PosWorld, ( VNormal - VFriNormal * m_Friction ) * factorB);

                GVector3 VWorldVelocity1 = pPair.GetWorldRelativeB( TPoint.m_PosWorld);

                if( GVector3::DotProduct( VWorldVelocity1, VNormal ) <= GMath::Zero() )
                {
                    TPoint.m_bCurrentSeparate = false;
                    bSeparate = false;
                }
                else
                {
                    TPoint.m_bCurrentSeparate = true;
                }
            }

            f32 fCurrentEnergy = pPair.GetContactPairEnergy();
            if (fCurrentEnergy > PairEnergy)
            {
                pPair.SeparatePair(pRA, pRB, bSwap);
                pPair.Solved = true;
                break;
            }

        }
    }

}


void GPhysicsWorld::SolveContactConstraint( )
{
    GPRORILER_FUN

    for (int32_t i = 0; i < (int32_t)m_Objects.size(); ++i)
    {
        GCObject* pObject = m_Objects[i];
        if (pObject->GetCOType() == CO_Rigid_Body && ((GRigidBody*)pObject)->m_bDynamic)
        {
            pObject->m_ContactStaticDepth = 1000;
        }
        else
        {
            pObject->m_ContactStaticDepth = 0;
        }
    }


    for( int n = 0; n < 100; n++)
    {
        for (uint32_t i = 0; i < (uint32_t)m_BroadPhasePairs.size(); ++i)
        {
            m_BroadPhasePairs[i].CalculateContactStaticDepth();
        }
    }

    std::multimap<uint32_t, GBroadPhasePair*> SortedMap;


    for ( uint32_t i = 0; i < (uint32_t)m_BroadPhasePairs.size(); ++i)
    {
        GBroadPhasePair* TPair = &m_BroadPhasePairs[i];
        SortedMap.insert( std::pair<uint32_t, GBroadPhasePair*>( TPair->m_ContactStaticDepth, TPair ));
    }


    for ( auto iter = SortedMap.begin(); iter != SortedMap.end(); ++iter)
    {
        SolveContactConstraint( *(iter->second) );
    }





}

void GPhysicsWorld::UpdateSceneGrid( )
{
    GPRORILER_FUN

    for (int32_t i = 0; i < (int32_t)m_Objects.size(); ++i)
    {
        GCObject* pObject = m_Objects[i];
        if( pObject->m_bNeedUpdate )
        {
            pObject->UpdateAABB();

            UpdateCollisionObject(pObject);  
            pObject->m_bNeedUpdate = false;
        }
    }

    for (std::map<GGridPosition, GGridCell*>::const_iterator iter = m_Grids.begin(); iter != m_Grids.end(); ++iter)
    {
        if( iter->second->m_Objects.size() == 0 )
        {
            delete iter->second;
            m_Grids.erase( iter++ );
        }
    }

    for (int32_t i = 0; i < (int32_t)m_StaticLargeObj.size(); ++i)
    {
        GCObject* pObject = m_StaticLargeObj[i];
        if (pObject->m_bNeedUpdate)
        {
            pObject->UpdateAABB();
        }
    }
}

void GPhysicsWorld::ClearContactPair()
{
    m_BroadPhasePairs.clear();
    m_BroadPhaseMap.clear();
}
void GPhysicsWorld::AddContactPair(GCObject* p1, GCObject* p2)
{
    GBroadPhasePair  TPair(p1, p2);

    m_BroadPhasePairs.push_back(TPair);

    m_BroadPhaseMap[TPair.PairId] = TPair;
}
