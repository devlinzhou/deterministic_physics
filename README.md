# deterministic_physics
deterministic physics engine for lock-step game development

## Technologies
* [deterministic_float](https://github.com/devlinzhou/deterministic_float)

## Features
- [ ] Shape
    - [x] Plane
    - [x] Sphere
    - [x] Box
    - [x] Convex
    - [ ] HeightField & WaterShape
    - [ ] TriangleMesh
- [ ] Collision Test Algorithm
    - [x] BoxBox
    - [x] BoxSphere
    - [x] BoxPlane
    - [x] SphereSphere
    - [ ] BoxTriangle
    - [ ] SphereTriangle
- [x] Contact Algorithm
- [ ] SolveConstraint
    - [ ]  
- [ ] Dynamic
    - [x] Rigid Body
    - [ ] Soft Body
    - [ ] Water Body
- [ ] Articulate body 
- [ ] Scene Manager
    - [x] Loose Grid

## How to Start
 * Unreal demo(4.27) : UnrealDemo/UnrealDemo.uproject 
   * UnrealDemo/Content/ThirdPersonCPP/Maps/GRigidTest_Box.umap : 6x6x6 cubes falling test
   * UnrealDemo/Content/ThirdPersonCPP/Maps/GPhsics.umap : physics simulation
   * UnrealDemo/Content/ThirdPersonCPP/Maps/TestCollisionGPhysics.umap : Test collision algorithm

