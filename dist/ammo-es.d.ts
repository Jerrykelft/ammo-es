export declare const enum PHY {
    Float = 0,
    Double = 1,
    Integer = 2,
    Short = 3,
    FixedPoint88 = 4,
    Uchar = 5
}
export declare const enum CollisionFilterGroups {
    DefaultFilter = 1,
    StaticFilter = 2,
    KinematicFilter = 4,
    DebrisFilter = 8,
    SensorTrigger = 16,
    CharacterFilter = 32,
    AllFilter = -1
}
export declare const enum CollisionFlags {
    StaticObject = 1,
    KinematicObject = 2,
    NoContactResponse = 4,
    CustomMaterialCallback = 8,
    CharacterObject = 16,
    DisableVisualizeObject = 32,
    DisableSpuCollisionProcessing = 64,
    HasContactStiffnessDamping = 128,
    HasCustomDebugRenderingColor = 256,
    HasFrictionAnchor = 512,
    HasCollisionSoundTrigger = 1024
}
export declare const enum CollisionObjectTypes {
    CollisionObject = 1,
    RigidBody = 2,
    GhostObject = 4,
    SoftBody = 8,
    HfFluid = 16,
    UserType = 32,
    FeatherstoneLink = 64
}
export declare const enum AnisotropicFrictionFlags {
    AnisotropicFrictionDisabled = 0,
    AnisotropicFriction = 1,
    AnisotropicRollingFriction = 2
}
export declare const enum ActivationState {
    ActiveTag = 1,
    IslandSleeping = 2,
    WantsDeactivation = 3,
    DisableDeactivation = 4,
    DisableSimulation = 5
}
export declare const enum Constraint {
    ERP = 1,
    StopERP = 2,
    CFM = 3,
    StopCFM = 4
}
export declare const enum ConstantGimpact {
    CompoundShape = 0,
    TrimeshShapePart = 1,
    TrimeshShape = 2
}
export type AmmoExports = [
    'webidl_free',
    'webidl_malloc',
    'malloc',
    'bind_btCollisionShape_setLocalScaling_1',
    'bind_btCollisionShape_getLocalScaling_0',
    'bind_btCollisionShape_calculateLocalInertia_2',
    'bind_btCollisionShape_setMargin_1',
    'bind_btCollisionShape_getMargin_0',
    'bind_btCollisionShape___destroy___0',
    'bind_btCollisionWorld_getDispatcher_0',
    'bind_btCollisionWorld_rayTest_3',
    'bind_btCollisionWorld_getPairCache_0',
    'bind_btCollisionWorld_getDispatchInfo_0',
    'bind_btCollisionWorld_addCollisionObject_1',
    'bind_btCollisionWorld_addCollisionObject_2',
    'bind_btCollisionWorld_addCollisionObject_3',
    'bind_btCollisionWorld_removeCollisionObject_1',
    'bind_btCollisionWorld_getBroadphase_0',
    'bind_btCollisionWorld_convexSweepTest_5',
    'bind_btCollisionWorld_contactPairTest_3',
    'bind_btCollisionWorld_contactTest_2',
    'bind_btCollisionWorld_updateSingleAabb_1',
    'bind_btCollisionWorld___destroy___0',
    'bind_btCollisionObject_setAnisotropicFriction_2',
    'bind_btCollisionObject_getCollisionShape_0',
    'bind_btCollisionObject_setContactProcessingThreshold_1',
    'bind_btCollisionObject_setActivationState_1',
    'bind_btCollisionObject_forceActivationState_1',
    'bind_btCollisionObject_activate_0',
    'bind_btCollisionObject_activate_1',
    'bind_btCollisionObject_isActive_0',
    'bind_btCollisionObject_isKinematicObject_0',
    'bind_btCollisionObject_isStaticObject_0',
    'bind_btCollisionObject_isStaticOrKinematicObject_0',
    'bind_btCollisionObject_getRestitution_0',
    'bind_btCollisionObject_getFriction_0',
    'bind_btCollisionObject_getRollingFriction_0',
    'bind_btCollisionObject_setRestitution_1',
    'bind_btCollisionObject_setFriction_1',
    'bind_btCollisionObject_setRollingFriction_1',
    'bind_btCollisionObject_getWorldTransform_0',
    'bind_btCollisionObject_getCollisionFlags_0',
    'bind_btCollisionObject_setCollisionFlags_1',
    'bind_btCollisionObject_setWorldTransform_1',
    'bind_btCollisionObject_setCollisionShape_1',
    'bind_btCollisionObject_setCcdMotionThreshold_1',
    'bind_btCollisionObject_setCcdSweptSphereRadius_1',
    'bind_btCollisionObject_getUserIndex_0',
    'bind_btCollisionObject_setUserIndex_1',
    'bind_btCollisionObject_getUserPointer_0',
    'bind_btCollisionObject_setUserPointer_1',
    'bind_btCollisionObject_getBroadphaseHandle_0',
    'bind_btCollisionObject___destroy___0',
    'bind_btConcaveShape_setLocalScaling_1',
    'bind_btConcaveShape_getLocalScaling_0',
    'bind_btConcaveShape_calculateLocalInertia_2',
    'bind_btConcaveShape___destroy___0',
    'bind_btCollisionAlgorithm___destroy___0',
    'bind_btTypedConstraint_enableFeedback_1',
    'bind_btTypedConstraint_getBreakingImpulseThreshold_0',
    'bind_btTypedConstraint_setBreakingImpulseThreshold_1',
    'bind_btTypedConstraint_getParam_2',
    'bind_btTypedConstraint_setParam_3',
    'bind_btTypedConstraint___destroy___0',
    'bind_btDynamicsWorld_addAction_1',
    'bind_btDynamicsWorld_removeAction_1',
    'bind_btDynamicsWorld_getSolverInfo_0',
    'bind_btDynamicsWorld_setInternalTickCallback_1',
    'bind_btDynamicsWorld_setInternalTickCallback_2',
    'bind_btDynamicsWorld_setInternalTickCallback_3',
    'bind_btDynamicsWorld_getDispatcher_0',
    'bind_btDynamicsWorld_rayTest_3',
    'bind_btDynamicsWorld_getPairCache_0',
    'bind_btDynamicsWorld_getDispatchInfo_0',
    'bind_btDynamicsWorld_addCollisionObject_1',
    'bind_btDynamicsWorld_addCollisionObject_2',
    'bind_btDynamicsWorld_addCollisionObject_3',
    'bind_btDynamicsWorld_removeCollisionObject_1',
    'bind_btDynamicsWorld_getBroadphase_0',
    'bind_btDynamicsWorld_convexSweepTest_5',
    'bind_btDynamicsWorld_contactPairTest_3',
    'bind_btDynamicsWorld_contactTest_2',
    'bind_btDynamicsWorld_updateSingleAabb_1',
    'bind_btDynamicsWorld___destroy___0',
    'bind_btVector3_btVector3_0',
    'bind_btVector3_btVector3_3',
    'bind_btVector3_length_0',
    'bind_btVector3_x_0',
    'bind_btVector3_y_0',
    'bind_btVector3_z_0',
    'bind_btVector3_setX_1',
    'bind_btVector3_setY_1',
    'bind_btVector3_setZ_1',
    'bind_btVector3_setValue_3',
    'bind_btVector3_normalize_0',
    'bind_btVector3_rotate_2',
    'bind_btVector3_dot_1',
    'bind_btVector3_op_mul_1',
    'bind_btVector3_op_add_1',
    'bind_btVector3_op_sub_1',
    'bind_btVector3___destroy___0',
    'bind_btQuadWord_x_0',
    'bind_btQuadWord_y_0',
    'bind_btQuadWord_z_0',
    'bind_btQuadWord_w_0',
    'bind_btQuadWord_setX_1',
    'bind_btQuadWord_setY_1',
    'bind_btQuadWord_setZ_1',
    'bind_btQuadWord_setW_1',
    'bind_btQuadWord___destroy___0',
    'bind_btMotionState_getWorldTransform_1',
    'bind_btMotionState_setWorldTransform_1',
    'bind_btMotionState___destroy___0',
    'bind_RayResultCallback_hasHit_0',
    'bind_RayResultCallback_get_m_collisionFilterGroup_0',
    'bind_RayResultCallback_set_m_collisionFilterGroup_1',
    'bind_RayResultCallback_get_m_collisionFilterMask_0',
    'bind_RayResultCallback_set_m_collisionFilterMask_1',
    'bind_RayResultCallback_get_m_closestHitFraction_0',
    'bind_RayResultCallback_set_m_closestHitFraction_1',
    'bind_RayResultCallback_get_m_collisionObject_0',
    'bind_RayResultCallback_set_m_collisionObject_1',
    'bind_RayResultCallback_get_m_flags_0',
    'bind_RayResultCallback_set_m_flags_1',
    'bind_RayResultCallback___destroy___0',
    'bind_ContactResultCallback_addSingleResult_7',
    'bind_ContactResultCallback___destroy___0',
    'bind_ConvexResultCallback_hasHit_0',
    'bind_ConvexResultCallback_get_m_collisionFilterGroup_0',
    'bind_ConvexResultCallback_set_m_collisionFilterGroup_1',
    'bind_ConvexResultCallback_get_m_collisionFilterMask_0',
    'bind_ConvexResultCallback_set_m_collisionFilterMask_1',
    'bind_ConvexResultCallback_get_m_closestHitFraction_0',
    'bind_ConvexResultCallback_set_m_closestHitFraction_1',
    'bind_ConvexResultCallback___destroy___0',
    'bind_btConvexShape_setLocalScaling_1',
    'bind_btConvexShape_getLocalScaling_0',
    'bind_btConvexShape_calculateLocalInertia_2',
    'bind_btConvexShape_setMargin_1',
    'bind_btConvexShape_getMargin_0',
    'bind_btConvexShape___destroy___0',
    'bind_btCapsuleShape_btCapsuleShape_2',
    'bind_btCapsuleShape_setMargin_1',
    'bind_btCapsuleShape_getMargin_0',
    'bind_btCapsuleShape_getUpAxis_0',
    'bind_btCapsuleShape_getRadius_0',
    'bind_btCapsuleShape_getHalfHeight_0',
    'bind_btCapsuleShape_setLocalScaling_1',
    'bind_btCapsuleShape_getLocalScaling_0',
    'bind_btCapsuleShape_calculateLocalInertia_2',
    'bind_btCapsuleShape___destroy___0',
    'bind_btCylinderShape_btCylinderShape_1',
    'bind_btCylinderShape_setMargin_1',
    'bind_btCylinderShape_getMargin_0',
    'bind_btCylinderShape_setLocalScaling_1',
    'bind_btCylinderShape_getLocalScaling_0',
    'bind_btCylinderShape_calculateLocalInertia_2',
    'bind_btCylinderShape___destroy___0',
    'bind_btConeShape_btConeShape_2',
    'bind_btConeShape_setLocalScaling_1',
    'bind_btConeShape_getLocalScaling_0',
    'bind_btConeShape_calculateLocalInertia_2',
    'bind_btConeShape___destroy___0',
    'bind_btStridingMeshInterface_setScaling_1',
    'bind_btStridingMeshInterface___destroy___0',
    'bind_btTriangleMeshShape_setLocalScaling_1',
    'bind_btTriangleMeshShape_getLocalScaling_0',
    'bind_btTriangleMeshShape_calculateLocalInertia_2',
    'bind_btTriangleMeshShape___destroy___0',
    'bind_btPrimitiveManagerBase_is_trimesh_0',
    'bind_btPrimitiveManagerBase_get_primitive_count_0',
    'bind_btPrimitiveManagerBase_get_primitive_box_2',
    'bind_btPrimitiveManagerBase_get_primitive_triangle_2',
    'bind_btPrimitiveManagerBase___destroy___0',
    'bind_btGImpactShapeInterface_updateBound_0',
    'bind_btGImpactShapeInterface_postUpdate_0',
    'bind_btGImpactShapeInterface_getShapeType_0',
    'bind_btGImpactShapeInterface_getName_0',
    'bind_btGImpactShapeInterface_getGImpactShapeType_0',
    'bind_btGImpactShapeInterface_getPrimitiveManager_0',
    'bind_btGImpactShapeInterface_getNumChildShapes_0',
    'bind_btGImpactShapeInterface_childrenHasTransform_0',
    'bind_btGImpactShapeInterface_needsRetrieveTriangles_0',
    'bind_btGImpactShapeInterface_needsRetrieveTetrahedrons_0',
    'bind_btGImpactShapeInterface_getBulletTriangle_2',
    'bind_btGImpactShapeInterface_getBulletTetrahedron_2',
    'bind_btGImpactShapeInterface_getChildShape_1',
    'bind_btGImpactShapeInterface_getChildTransform_1',
    'bind_btGImpactShapeInterface_setChildTransform_2',
    'bind_btGImpactShapeInterface_setLocalScaling_1',
    'bind_btGImpactShapeInterface_getLocalScaling_0',
    'bind_btGImpactShapeInterface_calculateLocalInertia_2',
    'bind_btGImpactShapeInterface___destroy___0',
    'bind_btActivatingCollisionAlgorithm___destroy___0',
    'bind_btDefaultCollisionConfiguration_btDefaultCollisionConfiguration_0',
    'bind_btDefaultCollisionConfiguration_btDefaultCollisionConfiguration_1',
    'bind_btDefaultCollisionConfiguration___destroy___0',
    'bind_btDispatcher_getNumManifolds_0',
    'bind_btDispatcher_getManifoldByIndexInternal_1',
    'bind_btDispatcher___destroy___0',
    'bind_btGeneric6DofConstraint_btGeneric6DofConstraint_3',
    'bind_btGeneric6DofConstraint_btGeneric6DofConstraint_5',
    'bind_btGeneric6DofConstraint_setLinearLowerLimit_1',
    'bind_btGeneric6DofConstraint_setLinearUpperLimit_1',
    'bind_btGeneric6DofConstraint_setAngularLowerLimit_1',
    'bind_btGeneric6DofConstraint_setAngularUpperLimit_1',
    'bind_btGeneric6DofConstraint_getFrameOffsetA_0',
    'bind_btGeneric6DofConstraint_enableFeedback_1',
    'bind_btGeneric6DofConstraint_getBreakingImpulseThreshold_0',
    'bind_btGeneric6DofConstraint_setBreakingImpulseThreshold_1',
    'bind_btGeneric6DofConstraint_getParam_2',
    'bind_btGeneric6DofConstraint_setParam_3',
    'bind_btGeneric6DofConstraint___destroy___0',
    'bind_btDiscreteDynamicsWorld_btDiscreteDynamicsWorld_4',
    'bind_btDiscreteDynamicsWorld_setGravity_1',
    'bind_btDiscreteDynamicsWorld_getGravity_0',
    'bind_btDiscreteDynamicsWorld_addRigidBody_1',
    'bind_btDiscreteDynamicsWorld_addRigidBody_3',
    'bind_btDiscreteDynamicsWorld_removeRigidBody_1',
    'bind_btDiscreteDynamicsWorld_addConstraint_1',
    'bind_btDiscreteDynamicsWorld_addConstraint_2',
    'bind_btDiscreteDynamicsWorld_removeConstraint_1',
    'bind_btDiscreteDynamicsWorld_stepSimulation_1',
    'bind_btDiscreteDynamicsWorld_stepSimulation_2',
    'bind_btDiscreteDynamicsWorld_stepSimulation_3',
    'bind_btDiscreteDynamicsWorld_setContactAddedCallback_1',
    'bind_btDiscreteDynamicsWorld_setContactProcessedCallback_1',
    'bind_btDiscreteDynamicsWorld_setContactDestroyedCallback_1',
    'bind_btDiscreteDynamicsWorld_getDispatcher_0',
    'bind_btDiscreteDynamicsWorld_rayTest_3',
    'bind_btDiscreteDynamicsWorld_getPairCache_0',
    'bind_btDiscreteDynamicsWorld_getDispatchInfo_0',
    'bind_btDiscreteDynamicsWorld_addCollisionObject_1',
    'bind_btDiscreteDynamicsWorld_addCollisionObject_2',
    'bind_btDiscreteDynamicsWorld_addCollisionObject_3',
    'bind_btDiscreteDynamicsWorld_removeCollisionObject_1',
    'bind_btDiscreteDynamicsWorld_getBroadphase_0',
    'bind_btDiscreteDynamicsWorld_convexSweepTest_5',
    'bind_btDiscreteDynamicsWorld_contactPairTest_3',
    'bind_btDiscreteDynamicsWorld_contactTest_2',
    'bind_btDiscreteDynamicsWorld_updateSingleAabb_1',
    'bind_btDiscreteDynamicsWorld_addAction_1',
    'bind_btDiscreteDynamicsWorld_removeAction_1',
    'bind_btDiscreteDynamicsWorld_getSolverInfo_0',
    'bind_btDiscreteDynamicsWorld_setInternalTickCallback_1',
    'bind_btDiscreteDynamicsWorld_setInternalTickCallback_2',
    'bind_btDiscreteDynamicsWorld_setInternalTickCallback_3',
    'bind_btDiscreteDynamicsWorld___destroy___0',
    'bind_btVehicleRaycaster_castRay_3',
    'bind_btVehicleRaycaster___destroy___0',
    'bind_btActionInterface_updateAction_2',
    'bind_btActionInterface___destroy___0',
    'bind_btGhostObject_btGhostObject_0',
    'bind_btGhostObject_getNumOverlappingObjects_0',
    'bind_btGhostObject_getOverlappingObject_1',
    'bind_btGhostObject_setAnisotropicFriction_2',
    'bind_btGhostObject_getCollisionShape_0',
    'bind_btGhostObject_setContactProcessingThreshold_1',
    'bind_btGhostObject_setActivationState_1',
    'bind_btGhostObject_forceActivationState_1',
    'bind_btGhostObject_activate_0',
    'bind_btGhostObject_activate_1',
    'bind_btGhostObject_isActive_0',
    'bind_btGhostObject_isKinematicObject_0',
    'bind_btGhostObject_isStaticObject_0',
    'bind_btGhostObject_isStaticOrKinematicObject_0',
    'bind_btGhostObject_getRestitution_0',
    'bind_btGhostObject_getFriction_0',
    'bind_btGhostObject_getRollingFriction_0',
    'bind_btGhostObject_setRestitution_1',
    'bind_btGhostObject_setFriction_1',
    'bind_btGhostObject_setRollingFriction_1',
    'bind_btGhostObject_getWorldTransform_0',
    'bind_btGhostObject_getCollisionFlags_0',
    'bind_btGhostObject_setCollisionFlags_1',
    'bind_btGhostObject_setWorldTransform_1',
    'bind_btGhostObject_setCollisionShape_1',
    'bind_btGhostObject_setCcdMotionThreshold_1',
    'bind_btGhostObject_setCcdSweptSphereRadius_1',
    'bind_btGhostObject_getUserIndex_0',
    'bind_btGhostObject_setUserIndex_1',
    'bind_btGhostObject_getUserPointer_0',
    'bind_btGhostObject_setUserPointer_1',
    'bind_btGhostObject_getBroadphaseHandle_0',
    'bind_btGhostObject___destroy___0',
    'bind_btSoftBodySolver___destroy___0',
    'bind_VoidPtr___destroy___0',
    'bind_btVector4_btVector4_0',
    'bind_btVector4_btVector4_4',
    'bind_btVector4_w_0',
    'bind_btVector4_setValue_4',
    'bind_btVector4_length_0',
    'bind_btVector4_x_0',
    'bind_btVector4_y_0',
    'bind_btVector4_z_0',
    'bind_btVector4_setX_1',
    'bind_btVector4_setY_1',
    'bind_btVector4_setZ_1',
    'bind_btVector4_normalize_0',
    'bind_btVector4_rotate_2',
    'bind_btVector4_dot_1',
    'bind_btVector4_op_mul_1',
    'bind_btVector4_op_add_1',
    'bind_btVector4_op_sub_1',
    'bind_btVector4___destroy___0',
    'bind_btQuaternion_btQuaternion_4',
    'bind_btQuaternion_setValue_4',
    'bind_btQuaternion_setEulerZYX_3',
    'bind_btQuaternion_setRotation_2',
    'bind_btQuaternion_normalize_0',
    'bind_btQuaternion_length2_0',
    'bind_btQuaternion_length_0',
    'bind_btQuaternion_dot_1',
    'bind_btQuaternion_normalized_0',
    'bind_btQuaternion_getAxis_0',
    'bind_btQuaternion_inverse_0',
    'bind_btQuaternion_getAngle_0',
    'bind_btQuaternion_getAngleShortestPath_0',
    'bind_btQuaternion_angle_1',
    'bind_btQuaternion_angleShortestPath_1',
    'bind_btQuaternion_op_add_1',
    'bind_btQuaternion_op_sub_1',
    'bind_btQuaternion_op_mul_1',
    'bind_btQuaternion_op_mulq_1',
    'bind_btQuaternion_op_div_1',
    'bind_btQuaternion_x_0',
    'bind_btQuaternion_y_0',
    'bind_btQuaternion_z_0',
    'bind_btQuaternion_w_0',
    'bind_btQuaternion_setX_1',
    'bind_btQuaternion_setY_1',
    'bind_btQuaternion_setZ_1',
    'bind_btQuaternion_setW_1',
    'bind_btQuaternion___destroy___0',
    'bind_btMatrix3x3_setEulerZYX_3',
    'bind_btMatrix3x3_getRotation_1',
    'bind_btMatrix3x3_getRow_1',
    'bind_btMatrix3x3___destroy___0',
    'bind_btTransform_btTransform_0',
    'bind_btTransform_btTransform_2',
    'bind_btTransform_setIdentity_0',
    'bind_btTransform_setOrigin_1',
    'bind_btTransform_setRotation_1',
    'bind_btTransform_getOrigin_0',
    'bind_btTransform_getRotation_0',
    'bind_btTransform_getBasis_0',
    'bind_btTransform_setFromOpenGLMatrix_1',
    'bind_btTransform_inverse_0',
    'bind_btTransform_op_mul_1',
    'bind_btTransform___destroy___0',
    'bind_MotionState_MotionState_0',
    'bind_MotionState_getWorldTransform_1',
    'bind_MotionState_setWorldTransform_1',
    'bind_MotionState___destroy___0',
    'bind_btDefaultMotionState_btDefaultMotionState_0',
    'bind_btDefaultMotionState_btDefaultMotionState_1',
    'bind_btDefaultMotionState_btDefaultMotionState_2',
    'bind_btDefaultMotionState_getWorldTransform_1',
    'bind_btDefaultMotionState_setWorldTransform_1',
    'bind_btDefaultMotionState_get_m_graphicsWorldTrans_0',
    'bind_btDefaultMotionState_set_m_graphicsWorldTrans_1',
    'bind_btDefaultMotionState___destroy___0',
    'bind_btCollisionObjectWrapper_getWorldTransform_0',
    'bind_btCollisionObjectWrapper_getCollisionObject_0',
    'bind_btCollisionObjectWrapper_getCollisionShape_0',
    'bind_ClosestRayResultCallback_ClosestRayResultCallback_2',
    'bind_ClosestRayResultCallback_hasHit_0',
    'bind_ClosestRayResultCallback_get_m_rayFromWorld_0',
    'bind_ClosestRayResultCallback_set_m_rayFromWorld_1',
    'bind_ClosestRayResultCallback_get_m_rayToWorld_0',
    'bind_ClosestRayResultCallback_set_m_rayToWorld_1',
    'bind_ClosestRayResultCallback_get_m_hitNormalWorld_0',
    'bind_ClosestRayResultCallback_set_m_hitNormalWorld_1',
    'bind_ClosestRayResultCallback_get_m_hitPointWorld_0',
    'bind_ClosestRayResultCallback_set_m_hitPointWorld_1',
    'bind_ClosestRayResultCallback_get_m_collisionFilterGroup_0',
    'bind_ClosestRayResultCallback_set_m_collisionFilterGroup_1',
    'bind_ClosestRayResultCallback_get_m_collisionFilterMask_0',
    'bind_ClosestRayResultCallback_set_m_collisionFilterMask_1',
    'bind_ClosestRayResultCallback_get_m_closestHitFraction_0',
    'bind_ClosestRayResultCallback_set_m_closestHitFraction_1',
    'bind_ClosestRayResultCallback_get_m_collisionObject_0',
    'bind_ClosestRayResultCallback_set_m_collisionObject_1',
    'bind_ClosestRayResultCallback_get_m_flags_0',
    'bind_ClosestRayResultCallback_set_m_flags_1',
    'bind_ClosestRayResultCallback___destroy___0',
    'bind_btConstCollisionObjectArray_size_0',
    'bind_btConstCollisionObjectArray_at_1',
    'bind_btConstCollisionObjectArray___destroy___0',
    'bind_btScalarArray_size_0',
    'bind_btScalarArray_at_1',
    'bind_btScalarArray___destroy___0',
    'bind_AllHitsRayResultCallback_AllHitsRayResultCallback_2',
    'bind_AllHitsRayResultCallback_hasHit_0',
    'bind_AllHitsRayResultCallback_get_m_collisionObjects_0',
    'bind_AllHitsRayResultCallback_set_m_collisionObjects_1',
    'bind_AllHitsRayResultCallback_get_m_rayFromWorld_0',
    'bind_AllHitsRayResultCallback_set_m_rayFromWorld_1',
    'bind_AllHitsRayResultCallback_get_m_rayToWorld_0',
    'bind_AllHitsRayResultCallback_set_m_rayToWorld_1',
    'bind_AllHitsRayResultCallback_get_m_hitNormalWorld_0',
    'bind_AllHitsRayResultCallback_set_m_hitNormalWorld_1',
    'bind_AllHitsRayResultCallback_get_m_hitPointWorld_0',
    'bind_AllHitsRayResultCallback_set_m_hitPointWorld_1',
    'bind_AllHitsRayResultCallback_get_m_hitFractions_0',
    'bind_AllHitsRayResultCallback_set_m_hitFractions_1',
    'bind_AllHitsRayResultCallback_get_m_collisionFilterGroup_0',
    'bind_AllHitsRayResultCallback_set_m_collisionFilterGroup_1',
    'bind_AllHitsRayResultCallback_get_m_collisionFilterMask_0',
    'bind_AllHitsRayResultCallback_set_m_collisionFilterMask_1',
    'bind_AllHitsRayResultCallback_get_m_closestHitFraction_0',
    'bind_AllHitsRayResultCallback_set_m_closestHitFraction_1',
    'bind_AllHitsRayResultCallback_get_m_collisionObject_0',
    'bind_AllHitsRayResultCallback_set_m_collisionObject_1',
    'bind_AllHitsRayResultCallback_get_m_flags_0',
    'bind_AllHitsRayResultCallback_set_m_flags_1',
    'bind_AllHitsRayResultCallback___destroy___0',
    'bind_btManifoldPoint_getPositionWorldOnA_0',
    'bind_btManifoldPoint_getPositionWorldOnB_0',
    'bind_btManifoldPoint_getAppliedImpulse_0',
    'bind_btManifoldPoint_getDistance_0',
    'bind_btManifoldPoint_get_m_localPointA_0',
    'bind_btManifoldPoint_set_m_localPointA_1',
    'bind_btManifoldPoint_get_m_localPointB_0',
    'bind_btManifoldPoint_set_m_localPointB_1',
    'bind_btManifoldPoint_get_m_positionWorldOnB_0',
    'bind_btManifoldPoint_set_m_positionWorldOnB_1',
    'bind_btManifoldPoint_get_m_positionWorldOnA_0',
    'bind_btManifoldPoint_set_m_positionWorldOnA_1',
    'bind_btManifoldPoint_get_m_normalWorldOnB_0',
    'bind_btManifoldPoint_set_m_normalWorldOnB_1',
    'bind_btManifoldPoint_get_m_userPersistentData_0',
    'bind_btManifoldPoint_set_m_userPersistentData_1',
    'bind_btManifoldPoint___destroy___0',
    'bind_ConcreteContactResultCallback_ConcreteContactResultCallback_0',
    'bind_ConcreteContactResultCallback_addSingleResult_7',
    'bind_ConcreteContactResultCallback___destroy___0',
    'bind_LocalShapeInfo_get_m_shapePart_0',
    'bind_LocalShapeInfo_set_m_shapePart_1',
    'bind_LocalShapeInfo_get_m_triangleIndex_0',
    'bind_LocalShapeInfo_set_m_triangleIndex_1',
    'bind_LocalShapeInfo___destroy___0',
    'bind_LocalConvexResult_LocalConvexResult_5',
    'bind_LocalConvexResult_get_m_hitCollisionObject_0',
    'bind_LocalConvexResult_set_m_hitCollisionObject_1',
    'bind_LocalConvexResult_get_m_localShapeInfo_0',
    'bind_LocalConvexResult_set_m_localShapeInfo_1',
    'bind_LocalConvexResult_get_m_hitNormalLocal_0',
    'bind_LocalConvexResult_set_m_hitNormalLocal_1',
    'bind_LocalConvexResult_get_m_hitPointLocal_0',
    'bind_LocalConvexResult_set_m_hitPointLocal_1',
    'bind_LocalConvexResult_get_m_hitFraction_0',
    'bind_LocalConvexResult_set_m_hitFraction_1',
    'bind_LocalConvexResult___destroy___0',
    'bind_ClosestConvexResultCallback_ClosestConvexResultCallback_2',
    'bind_ClosestConvexResultCallback_hasHit_0',
    'bind_ClosestConvexResultCallback_get_m_hitCollisionObject_0',
    'bind_ClosestConvexResultCallback_set_m_hitCollisionObject_1',
    'bind_ClosestConvexResultCallback_get_m_convexFromWorld_0',
    'bind_ClosestConvexResultCallback_set_m_convexFromWorld_1',
    'bind_ClosestConvexResultCallback_get_m_convexToWorld_0',
    'bind_ClosestConvexResultCallback_set_m_convexToWorld_1',
    'bind_ClosestConvexResultCallback_get_m_hitNormalWorld_0',
    'bind_ClosestConvexResultCallback_set_m_hitNormalWorld_1',
    'bind_ClosestConvexResultCallback_get_m_hitPointWorld_0',
    'bind_ClosestConvexResultCallback_set_m_hitPointWorld_1',
    'bind_ClosestConvexResultCallback_get_m_collisionFilterGroup_0',
    'bind_ClosestConvexResultCallback_set_m_collisionFilterGroup_1',
    'bind_ClosestConvexResultCallback_get_m_collisionFilterMask_0',
    'bind_ClosestConvexResultCallback_set_m_collisionFilterMask_1',
    'bind_ClosestConvexResultCallback_get_m_closestHitFraction_0',
    'bind_ClosestConvexResultCallback_set_m_closestHitFraction_1',
    'bind_ClosestConvexResultCallback___destroy___0',
    'bind_btConvexTriangleMeshShape_btConvexTriangleMeshShape_1',
    'bind_btConvexTriangleMeshShape_btConvexTriangleMeshShape_2',
    'bind_btConvexTriangleMeshShape_setLocalScaling_1',
    'bind_btConvexTriangleMeshShape_getLocalScaling_0',
    'bind_btConvexTriangleMeshShape_calculateLocalInertia_2',
    'bind_btConvexTriangleMeshShape_setMargin_1',
    'bind_btConvexTriangleMeshShape_getMargin_0',
    'bind_btConvexTriangleMeshShape___destroy___0',
    'bind_btBoxShape_btBoxShape_1',
    'bind_btBoxShape_setMargin_1',
    'bind_btBoxShape_getMargin_0',
    'bind_btBoxShape_setLocalScaling_1',
    'bind_btBoxShape_getLocalScaling_0',
    'bind_btBoxShape_calculateLocalInertia_2',
    'bind_btBoxShape___destroy___0',
    'bind_btCapsuleShapeX_btCapsuleShapeX_2',
    'bind_btCapsuleShapeX_setMargin_1',
    'bind_btCapsuleShapeX_getMargin_0',
    'bind_btCapsuleShapeX_getUpAxis_0',
    'bind_btCapsuleShapeX_getRadius_0',
    'bind_btCapsuleShapeX_getHalfHeight_0',
    'bind_btCapsuleShapeX_setLocalScaling_1',
    'bind_btCapsuleShapeX_getLocalScaling_0',
    'bind_btCapsuleShapeX_calculateLocalInertia_2',
    'bind_btCapsuleShapeX___destroy___0',
    'bind_btCapsuleShapeZ_btCapsuleShapeZ_2',
    'bind_btCapsuleShapeZ_setMargin_1',
    'bind_btCapsuleShapeZ_getMargin_0',
    'bind_btCapsuleShapeZ_getUpAxis_0',
    'bind_btCapsuleShapeZ_getRadius_0',
    'bind_btCapsuleShapeZ_getHalfHeight_0',
    'bind_btCapsuleShapeZ_setLocalScaling_1',
    'bind_btCapsuleShapeZ_getLocalScaling_0',
    'bind_btCapsuleShapeZ_calculateLocalInertia_2',
    'bind_btCapsuleShapeZ___destroy___0',
    'bind_btCylinderShapeX_btCylinderShapeX_1',
    'bind_btCylinderShapeX_setMargin_1',
    'bind_btCylinderShapeX_getMargin_0',
    'bind_btCylinderShapeX_setLocalScaling_1',
    'bind_btCylinderShapeX_getLocalScaling_0',
    'bind_btCylinderShapeX_calculateLocalInertia_2',
    'bind_btCylinderShapeX___destroy___0',
    'bind_btCylinderShapeZ_btCylinderShapeZ_1',
    'bind_btCylinderShapeZ_setMargin_1',
    'bind_btCylinderShapeZ_getMargin_0',
    'bind_btCylinderShapeZ_setLocalScaling_1',
    'bind_btCylinderShapeZ_getLocalScaling_0',
    'bind_btCylinderShapeZ_calculateLocalInertia_2',
    'bind_btCylinderShapeZ___destroy___0',
    'bind_btSphereShape_btSphereShape_1',
    'bind_btSphereShape_setMargin_1',
    'bind_btSphereShape_getMargin_0',
    'bind_btSphereShape_setLocalScaling_1',
    'bind_btSphereShape_getLocalScaling_0',
    'bind_btSphereShape_calculateLocalInertia_2',
    'bind_btSphereShape___destroy___0',
    'bind_btMultiSphereShape_btMultiSphereShape_3',
    'bind_btMultiSphereShape_setLocalScaling_1',
    'bind_btMultiSphereShape_getLocalScaling_0',
    'bind_btMultiSphereShape_calculateLocalInertia_2',
    'bind_btMultiSphereShape___destroy___0',
    'bind_btConeShapeX_btConeShapeX_2',
    'bind_btConeShapeX_setLocalScaling_1',
    'bind_btConeShapeX_getLocalScaling_0',
    'bind_btConeShapeX_calculateLocalInertia_2',
    'bind_btConeShapeX___destroy___0',
    'bind_btConeShapeZ_btConeShapeZ_2',
    'bind_btConeShapeZ_setLocalScaling_1',
    'bind_btConeShapeZ_getLocalScaling_0',
    'bind_btConeShapeZ_calculateLocalInertia_2',
    'bind_btConeShapeZ___destroy___0',
    'bind_btIntArray_size_0',
    'bind_btIntArray_at_1',
    'bind_btIntArray___destroy___0',
    'bind_btFace_get_m_indices_0',
    'bind_btFace_set_m_indices_1',
    'bind_btFace_get_m_plane_1',
    'bind_btFace_set_m_plane_2',
    'bind_btFace___destroy___0',
    'bind_btVector3Array_size_0',
    'bind_btVector3Array_at_1',
    'bind_btVector3Array___destroy___0',
    'bind_btFaceArray_size_0',
    'bind_btFaceArray_at_1',
    'bind_btFaceArray___destroy___0',
    'bind_btConvexPolyhedron_get_m_vertices_0',
    'bind_btConvexPolyhedron_set_m_vertices_1',
    'bind_btConvexPolyhedron_get_m_faces_0',
    'bind_btConvexPolyhedron_set_m_faces_1',
    'bind_btConvexPolyhedron___destroy___0',
    'bind_btConvexHullShape_btConvexHullShape_0',
    'bind_btConvexHullShape_btConvexHullShape_1',
    'bind_btConvexHullShape_btConvexHullShape_2',
    'bind_btConvexHullShape_addPoint_1',
    'bind_btConvexHullShape_addPoint_2',
    'bind_btConvexHullShape_setMargin_1',
    'bind_btConvexHullShape_getMargin_0',
    'bind_btConvexHullShape_getNumVertices_0',
    'bind_btConvexHullShape_initializePolyhedralFeatures_1',
    'bind_btConvexHullShape_recalcLocalAabb_0',
    'bind_btConvexHullShape_getConvexPolyhedron_0',
    'bind_btConvexHullShape_setLocalScaling_1',
    'bind_btConvexHullShape_getLocalScaling_0',
    'bind_btConvexHullShape_calculateLocalInertia_2',
    'bind_btConvexHullShape___destroy___0',
    'bind_btShapeHull_btShapeHull_1',
    'bind_btShapeHull_buildHull_1',
    'bind_btShapeHull_numVertices_0',
    'bind_btShapeHull_getVertexPointer_0',
    'bind_btShapeHull___destroy___0',
    'bind_btCompoundShape_btCompoundShape_0',
    'bind_btCompoundShape_btCompoundShape_1',
    'bind_btCompoundShape_addChildShape_2',
    'bind_btCompoundShape_removeChildShape_1',
    'bind_btCompoundShape_removeChildShapeByIndex_1',
    'bind_btCompoundShape_getNumChildShapes_0',
    'bind_btCompoundShape_getChildShape_1',
    'bind_btCompoundShape_updateChildTransform_2',
    'bind_btCompoundShape_updateChildTransform_3',
    'bind_btCompoundShape_setMargin_1',
    'bind_btCompoundShape_getMargin_0',
    'bind_btCompoundShape_setLocalScaling_1',
    'bind_btCompoundShape_getLocalScaling_0',
    'bind_btCompoundShape_calculateLocalInertia_2',
    'bind_btCompoundShape___destroy___0',
    'bind_btIndexedMesh_get_m_numTriangles_0',
    'bind_btIndexedMesh_set_m_numTriangles_1',
    'bind_btIndexedMesh___destroy___0',
    'bind_btIndexedMeshArray_size_0',
    'bind_btIndexedMeshArray_at_1',
    'bind_btIndexedMeshArray___destroy___0',
    'bind_btTriangleMesh_btTriangleMesh_0',
    'bind_btTriangleMesh_btTriangleMesh_1',
    'bind_btTriangleMesh_btTriangleMesh_2',
    'bind_btTriangleMesh_addTriangle_3',
    'bind_btTriangleMesh_addTriangle_4',
    'bind_btTriangleMesh_findOrAddVertex_2',
    'bind_btTriangleMesh_addIndex_1',
    'bind_btTriangleMesh_getIndexedMeshArray_0',
    'bind_btTriangleMesh_setScaling_1',
    'bind_btTriangleMesh___destroy___0',
    'bind_btEmptyShape_btEmptyShape_0',
    'bind_btEmptyShape_setLocalScaling_1',
    'bind_btEmptyShape_getLocalScaling_0',
    'bind_btEmptyShape_calculateLocalInertia_2',
    'bind_btEmptyShape___destroy___0',
    'bind_btStaticPlaneShape_btStaticPlaneShape_2',
    'bind_btStaticPlaneShape_setLocalScaling_1',
    'bind_btStaticPlaneShape_getLocalScaling_0',
    'bind_btStaticPlaneShape_calculateLocalInertia_2',
    'bind_btStaticPlaneShape___destroy___0',
    'bind_btBvhTriangleMeshShape_btBvhTriangleMeshShape_2',
    'bind_btBvhTriangleMeshShape_btBvhTriangleMeshShape_3',
    'bind_btBvhTriangleMeshShape_setLocalScaling_1',
    'bind_btBvhTriangleMeshShape_getLocalScaling_0',
    'bind_btBvhTriangleMeshShape_calculateLocalInertia_2',
    'bind_btBvhTriangleMeshShape___destroy___0',
    'bind_btHeightfieldTerrainShape_btHeightfieldTerrainShape_9',
    'bind_btHeightfieldTerrainShape_setMargin_1',
    'bind_btHeightfieldTerrainShape_getMargin_0',
    'bind_btHeightfieldTerrainShape_setLocalScaling_1',
    'bind_btHeightfieldTerrainShape_getLocalScaling_0',
    'bind_btHeightfieldTerrainShape_calculateLocalInertia_2',
    'bind_btHeightfieldTerrainShape___destroy___0',
    'bind_btAABB_btAABB_4',
    'bind_btAABB_invalidate_0',
    'bind_btAABB_increment_margin_1',
    'bind_btAABB_copy_with_margin_2',
    'bind_btAABB___destroy___0',
    'bind_btPrimitiveTriangle_btPrimitiveTriangle_0',
    'bind_btPrimitiveTriangle___destroy___0',
    'bind_btTriangleShapeEx_btTriangleShapeEx_3',
    'bind_btTriangleShapeEx_getAabb_3',
    'bind_btTriangleShapeEx_applyTransform_1',
    'bind_btTriangleShapeEx_buildTriPlane_1',
    'bind_btTriangleShapeEx___destroy___0',
    'bind_btTetrahedronShapeEx_btTetrahedronShapeEx_0',
    'bind_btTetrahedronShapeEx_setVertices_4',
    'bind_btTetrahedronShapeEx___destroy___0',
    'bind_CompoundPrimitiveManager_get_primitive_count_0',
    'bind_CompoundPrimitiveManager_get_primitive_box_2',
    'bind_CompoundPrimitiveManager_get_primitive_triangle_2',
    'bind_CompoundPrimitiveManager_is_trimesh_0',
    'bind_CompoundPrimitiveManager_get_m_compoundShape_0',
    'bind_CompoundPrimitiveManager_set_m_compoundShape_1',
    'bind_CompoundPrimitiveManager___destroy___0',
    'bind_btGImpactCompoundShape_btGImpactCompoundShape_0',
    'bind_btGImpactCompoundShape_btGImpactCompoundShape_1',
    'bind_btGImpactCompoundShape_childrenHasTransform_0',
    'bind_btGImpactCompoundShape_getPrimitiveManager_0',
    'bind_btGImpactCompoundShape_getCompoundPrimitiveManager_0',
    'bind_btGImpactCompoundShape_getNumChildShapes_0',
    'bind_btGImpactCompoundShape_addChildShape_2',
    'bind_btGImpactCompoundShape_getChildShape_1',
    'bind_btGImpactCompoundShape_getChildAabb_4',
    'bind_btGImpactCompoundShape_getChildTransform_1',
    'bind_btGImpactCompoundShape_setChildTransform_2',
    'bind_btGImpactCompoundShape_calculateLocalInertia_2',
    'bind_btGImpactCompoundShape_getName_0',
    'bind_btGImpactCompoundShape_getGImpactShapeType_0',
    'bind_btGImpactCompoundShape_setLocalScaling_1',
    'bind_btGImpactCompoundShape_getLocalScaling_0',
    'bind_btGImpactCompoundShape_updateBound_0',
    'bind_btGImpactCompoundShape_postUpdate_0',
    'bind_btGImpactCompoundShape_getShapeType_0',
    'bind_btGImpactCompoundShape_needsRetrieveTriangles_0',
    'bind_btGImpactCompoundShape_needsRetrieveTetrahedrons_0',
    'bind_btGImpactCompoundShape_getBulletTriangle_2',
    'bind_btGImpactCompoundShape_getBulletTetrahedron_2',
    'bind_btGImpactCompoundShape___destroy___0',
    'bind_TrimeshPrimitiveManager_TrimeshPrimitiveManager_0',
    'bind_TrimeshPrimitiveManager_TrimeshPrimitiveManager_1',
    'bind_TrimeshPrimitiveManager_lock_0',
    'bind_TrimeshPrimitiveManager_unlock_0',
    'bind_TrimeshPrimitiveManager_is_trimesh_0',
    'bind_TrimeshPrimitiveManager_get_vertex_count_0',
    'bind_TrimeshPrimitiveManager_get_indices_4',
    'bind_TrimeshPrimitiveManager_get_vertex_2',
    'bind_TrimeshPrimitiveManager_get_bullet_triangle_2',
    'bind_TrimeshPrimitiveManager_get_m_margin_0',
    'bind_TrimeshPrimitiveManager_set_m_margin_1',
    'bind_TrimeshPrimitiveManager_get_m_meshInterface_0',
    'bind_TrimeshPrimitiveManager_set_m_meshInterface_1',
    'bind_TrimeshPrimitiveManager_get_m_part_0',
    'bind_TrimeshPrimitiveManager_set_m_part_1',
    'bind_TrimeshPrimitiveManager_get_m_lock_count_0',
    'bind_TrimeshPrimitiveManager_set_m_lock_count_1',
    'bind_TrimeshPrimitiveManager_get_numverts_0',
    'bind_TrimeshPrimitiveManager_set_numverts_1',
    'bind_TrimeshPrimitiveManager_get_type_0',
    'bind_TrimeshPrimitiveManager_set_type_1',
    'bind_TrimeshPrimitiveManager_get_stride_0',
    'bind_TrimeshPrimitiveManager_set_stride_1',
    'bind_TrimeshPrimitiveManager_get_indexstride_0',
    'bind_TrimeshPrimitiveManager_set_indexstride_1',
    'bind_TrimeshPrimitiveManager_get_numfaces_0',
    'bind_TrimeshPrimitiveManager_set_numfaces_1',
    'bind_TrimeshPrimitiveManager_get_indicestype_0',
    'bind_TrimeshPrimitiveManager_set_indicestype_1',
    'bind_TrimeshPrimitiveManager___destroy___0',
    'bind_btGImpactMeshShapePart_btGImpactMeshShapePart_2',
    'bind_btGImpactMeshShapePart_getTrimeshPrimitiveManager_0',
    'bind_btGImpactMeshShapePart_getVertexCount_0',
    'bind_btGImpactMeshShapePart_getVertex_2',
    'bind_btGImpactMeshShapePart_getPart_0',
    'bind_btGImpactMeshShapePart_setLocalScaling_1',
    'bind_btGImpactMeshShapePart_getLocalScaling_0',
    'bind_btGImpactMeshShapePart_updateBound_0',
    'bind_btGImpactMeshShapePart_postUpdate_0',
    'bind_btGImpactMeshShapePart_getShapeType_0',
    'bind_btGImpactMeshShapePart_needsRetrieveTriangles_0',
    'bind_btGImpactMeshShapePart_needsRetrieveTetrahedrons_0',
    'bind_btGImpactMeshShapePart_getBulletTriangle_2',
    'bind_btGImpactMeshShapePart_getBulletTetrahedron_2',
    'bind_btGImpactMeshShapePart___destroy___0',
    'bind_btGImpactMeshShape_btGImpactMeshShape_1',
    'bind_btGImpactMeshShape_getMeshInterface_0',
    'bind_btGImpactMeshShape_getMeshPartCount_0',
    'bind_btGImpactMeshShape_getMeshPart_1',
    'bind_btGImpactMeshShape_calculateSerializeBufferSize_0',
    'bind_btGImpactMeshShape_setLocalScaling_1',
    'bind_btGImpactMeshShape_getLocalScaling_0',
    'bind_btGImpactMeshShape_updateBound_0',
    'bind_btGImpactMeshShape_postUpdate_0',
    'bind_btGImpactMeshShape_getShapeType_0',
    'bind_btGImpactMeshShape_needsRetrieveTriangles_0',
    'bind_btGImpactMeshShape_needsRetrieveTetrahedrons_0',
    'bind_btGImpactMeshShape_getBulletTriangle_2',
    'bind_btGImpactMeshShape_getBulletTetrahedron_2',
    'bind_btGImpactMeshShape___destroy___0',
    'bind_btCollisionAlgorithmConstructionInfo_btCollisionAlgorithmConstructionInfo_0',
    'bind_btCollisionAlgorithmConstructionInfo_btCollisionAlgorithmConstructionInfo_2',
    'bind_btCollisionAlgorithmConstructionInfo_get_m_dispatcher1_0',
    'bind_btCollisionAlgorithmConstructionInfo_set_m_dispatcher1_1',
    'bind_btCollisionAlgorithmConstructionInfo_get_m_manifold_0',
    'bind_btCollisionAlgorithmConstructionInfo_set_m_manifold_1',
    'bind_btCollisionAlgorithmConstructionInfo___destroy___0',
    'bind_btGImpactCollisionAlgorithm_btGImpactCollisionAlgorithm_3',
    'bind_btGImpactCollisionAlgorithm_registerAlgorithm_1',
    'bind_btGImpactCollisionAlgorithm___destroy___0',
    'bind_btDefaultCollisionConstructionInfo_btDefaultCollisionConstructionInfo_0',
    'bind_btDefaultCollisionConstructionInfo___destroy___0',
    'bind_btPersistentManifold_btPersistentManifold_0',
    'bind_btPersistentManifold_getBody0_0',
    'bind_btPersistentManifold_getBody1_0',
    'bind_btPersistentManifold_getNumContacts_0',
    'bind_btPersistentManifold_getContactPoint_1',
    'bind_btPersistentManifold___destroy___0',
    'bind_btCollisionDispatcher_btCollisionDispatcher_1',
    'bind_btCollisionDispatcher_getNumManifolds_0',
    'bind_btCollisionDispatcher_getManifoldByIndexInternal_1',
    'bind_btCollisionDispatcher___destroy___0',
    'bind_btOverlappingPairCallback___destroy___0',
    'bind_btOverlappingPairCache_setInternalGhostPairCallback_1',
    'bind_btOverlappingPairCache_getNumOverlappingPairs_0',
    'bind_btOverlappingPairCache___destroy___0',
    'bind_btAxisSweep3_btAxisSweep3_2',
    'bind_btAxisSweep3_btAxisSweep3_3',
    'bind_btAxisSweep3_btAxisSweep3_4',
    'bind_btAxisSweep3_btAxisSweep3_5',
    'bind_btAxisSweep3___destroy___0',
    'bind_btBroadphaseInterface_getOverlappingPairCache_0',
    'bind_btBroadphaseInterface___destroy___0',
    'bind_btCollisionConfiguration___destroy___0',
    'bind_btDbvtBroadphase_btDbvtBroadphase_0',
    'bind_btDbvtBroadphase___destroy___0',
    'bind_btBroadphaseProxy_get_m_collisionFilterGroup_0',
    'bind_btBroadphaseProxy_set_m_collisionFilterGroup_1',
    'bind_btBroadphaseProxy_get_m_collisionFilterMask_0',
    'bind_btBroadphaseProxy_set_m_collisionFilterMask_1',
    'bind_btBroadphaseProxy___destroy___0',
    'bind_btRigidBodyConstructionInfo_btRigidBodyConstructionInfo_3',
    'bind_btRigidBodyConstructionInfo_btRigidBodyConstructionInfo_4',
    'bind_btRigidBodyConstructionInfo_get_m_linearDamping_0',
    'bind_btRigidBodyConstructionInfo_set_m_linearDamping_1',
    'bind_btRigidBodyConstructionInfo_get_m_angularDamping_0',
    'bind_btRigidBodyConstructionInfo_set_m_angularDamping_1',
    'bind_btRigidBodyConstructionInfo_get_m_friction_0',
    'bind_btRigidBodyConstructionInfo_set_m_friction_1',
    'bind_btRigidBodyConstructionInfo_get_m_rollingFriction_0',
    'bind_btRigidBodyConstructionInfo_set_m_rollingFriction_1',
    'bind_btRigidBodyConstructionInfo_get_m_restitution_0',
    'bind_btRigidBodyConstructionInfo_set_m_restitution_1',
    'bind_btRigidBodyConstructionInfo_get_m_linearSleepingThreshold_0',
    'bind_btRigidBodyConstructionInfo_set_m_linearSleepingThreshold_1',
    'bind_btRigidBodyConstructionInfo_get_m_angularSleepingThreshold_0',
    'bind_btRigidBodyConstructionInfo_set_m_angularSleepingThreshold_1',
    'bind_btRigidBodyConstructionInfo_get_m_additionalDamping_0',
    'bind_btRigidBodyConstructionInfo_set_m_additionalDamping_1',
    'bind_btRigidBodyConstructionInfo_get_m_additionalDampingFactor_0',
    'bind_btRigidBodyConstructionInfo_set_m_additionalDampingFactor_1',
    'bind_btRigidBodyConstructionInfo_get_m_additionalLinearDampingThresholdSqr_0',
    'bind_btRigidBodyConstructionInfo_set_m_additionalLinearDampingThresholdSqr_1',
    'bind_btRigidBodyConstructionInfo_get_m_additionalAngularDampingThresholdSqr_0',
    'bind_btRigidBodyConstructionInfo_set_m_additionalAngularDampingThresholdSqr_1',
    'bind_btRigidBodyConstructionInfo_get_m_additionalAngularDampingFactor_0',
    'bind_btRigidBodyConstructionInfo_set_m_additionalAngularDampingFactor_1',
    'bind_btRigidBodyConstructionInfo___destroy___0',
    'bind_btRigidBody_btRigidBody_1',
    'bind_btRigidBody_getCenterOfMassTransform_0',
    'bind_btRigidBody_setCenterOfMassTransform_1',
    'bind_btRigidBody_setSleepingThresholds_2',
    'bind_btRigidBody_getLinearDamping_0',
    'bind_btRigidBody_getAngularDamping_0',
    'bind_btRigidBody_setDamping_2',
    'bind_btRigidBody_setMassProps_2',
    'bind_btRigidBody_getLinearFactor_0',
    'bind_btRigidBody_setLinearFactor_1',
    'bind_btRigidBody_applyTorque_1',
    'bind_btRigidBody_applyLocalTorque_1',
    'bind_btRigidBody_applyForce_2',
    'bind_btRigidBody_applyCentralForce_1',
    'bind_btRigidBody_applyCentralLocalForce_1',
    'bind_btRigidBody_applyTorqueImpulse_1',
    'bind_btRigidBody_applyImpulse_2',
    'bind_btRigidBody_applyCentralImpulse_1',
    'bind_btRigidBody_updateInertiaTensor_0',
    'bind_btRigidBody_getLinearVelocity_0',
    'bind_btRigidBody_getAngularVelocity_0',
    'bind_btRigidBody_setLinearVelocity_1',
    'bind_btRigidBody_setAngularVelocity_1',
    'bind_btRigidBody_getMotionState_0',
    'bind_btRigidBody_setMotionState_1',
    'bind_btRigidBody_getAngularFactor_0',
    'bind_btRigidBody_setAngularFactor_1',
    'bind_btRigidBody_upcast_1',
    'bind_btRigidBody_getAabb_2',
    'bind_btRigidBody_applyGravity_0',
    'bind_btRigidBody_getGravity_0',
    'bind_btRigidBody_setGravity_1',
    'bind_btRigidBody_getBroadphaseProxy_0',
    'bind_btRigidBody_clearForces_0',
    'bind_btRigidBody_setFlags_1',
    'bind_btRigidBody_getFlags_0',
    'bind_btRigidBody_setAnisotropicFriction_2',
    'bind_btRigidBody_getCollisionShape_0',
    'bind_btRigidBody_setContactProcessingThreshold_1',
    'bind_btRigidBody_setActivationState_1',
    'bind_btRigidBody_forceActivationState_1',
    'bind_btRigidBody_activate_0',
    'bind_btRigidBody_activate_1',
    'bind_btRigidBody_isActive_0',
    'bind_btRigidBody_isKinematicObject_0',
    'bind_btRigidBody_isStaticObject_0',
    'bind_btRigidBody_isStaticOrKinematicObject_0',
    'bind_btRigidBody_getRestitution_0',
    'bind_btRigidBody_getFriction_0',
    'bind_btRigidBody_getRollingFriction_0',
    'bind_btRigidBody_setRestitution_1',
    'bind_btRigidBody_setFriction_1',
    'bind_btRigidBody_setRollingFriction_1',
    'bind_btRigidBody_getWorldTransform_0',
    'bind_btRigidBody_getCollisionFlags_0',
    'bind_btRigidBody_setCollisionFlags_1',
    'bind_btRigidBody_setWorldTransform_1',
    'bind_btRigidBody_setCollisionShape_1',
    'bind_btRigidBody_setCcdMotionThreshold_1',
    'bind_btRigidBody_setCcdSweptSphereRadius_1',
    'bind_btRigidBody_getUserIndex_0',
    'bind_btRigidBody_setUserIndex_1',
    'bind_btRigidBody_getUserPointer_0',
    'bind_btRigidBody_setUserPointer_1',
    'bind_btRigidBody_getBroadphaseHandle_0',
    'bind_btRigidBody___destroy___0',
    'bind_btConstraintSetting_btConstraintSetting_0',
    'bind_btConstraintSetting_get_m_tau_0',
    'bind_btConstraintSetting_set_m_tau_1',
    'bind_btConstraintSetting_get_m_damping_0',
    'bind_btConstraintSetting_set_m_damping_1',
    'bind_btConstraintSetting_get_m_impulseClamp_0',
    'bind_btConstraintSetting_set_m_impulseClamp_1',
    'bind_btConstraintSetting___destroy___0',
    'bind_btPoint2PointConstraint_btPoint2PointConstraint_2',
    'bind_btPoint2PointConstraint_btPoint2PointConstraint_4',
    'bind_btPoint2PointConstraint_setPivotA_1',
    'bind_btPoint2PointConstraint_setPivotB_1',
    'bind_btPoint2PointConstraint_getPivotInA_0',
    'bind_btPoint2PointConstraint_getPivotInB_0',
    'bind_btPoint2PointConstraint_enableFeedback_1',
    'bind_btPoint2PointConstraint_getBreakingImpulseThreshold_0',
    'bind_btPoint2PointConstraint_setBreakingImpulseThreshold_1',
    'bind_btPoint2PointConstraint_getParam_2',
    'bind_btPoint2PointConstraint_setParam_3',
    'bind_btPoint2PointConstraint_get_m_setting_0',
    'bind_btPoint2PointConstraint_set_m_setting_1',
    'bind_btPoint2PointConstraint___destroy___0',
    'bind_btGeneric6DofSpringConstraint_btGeneric6DofSpringConstraint_3',
    'bind_btGeneric6DofSpringConstraint_btGeneric6DofSpringConstraint_5',
    'bind_btGeneric6DofSpringConstraint_enableSpring_2',
    'bind_btGeneric6DofSpringConstraint_setStiffness_2',
    'bind_btGeneric6DofSpringConstraint_setDamping_2',
    'bind_btGeneric6DofSpringConstraint_setEquilibriumPoint_0',
    'bind_btGeneric6DofSpringConstraint_setEquilibriumPoint_1',
    'bind_btGeneric6DofSpringConstraint_setEquilibriumPoint_2',
    'bind_btGeneric6DofSpringConstraint_setLinearLowerLimit_1',
    'bind_btGeneric6DofSpringConstraint_setLinearUpperLimit_1',
    'bind_btGeneric6DofSpringConstraint_setAngularLowerLimit_1',
    'bind_btGeneric6DofSpringConstraint_setAngularUpperLimit_1',
    'bind_btGeneric6DofSpringConstraint_getFrameOffsetA_0',
    'bind_btGeneric6DofSpringConstraint_enableFeedback_1',
    'bind_btGeneric6DofSpringConstraint_getBreakingImpulseThreshold_0',
    'bind_btGeneric6DofSpringConstraint_setBreakingImpulseThreshold_1',
    'bind_btGeneric6DofSpringConstraint_getParam_2',
    'bind_btGeneric6DofSpringConstraint_setParam_3',
    'bind_btGeneric6DofSpringConstraint___destroy___0',
    'bind_btSequentialImpulseConstraintSolver_btSequentialImpulseConstraintSolver_0',
    'bind_btSequentialImpulseConstraintSolver___destroy___0',
    'bind_btConeTwistConstraint_btConeTwistConstraint_2',
    'bind_btConeTwistConstraint_btConeTwistConstraint_4',
    'bind_btConeTwistConstraint_setLimit_2',
    'bind_btConeTwistConstraint_setAngularOnly_1',
    'bind_btConeTwistConstraint_setDamping_1',
    'bind_btConeTwistConstraint_enableMotor_1',
    'bind_btConeTwistConstraint_setMaxMotorImpulse_1',
    'bind_btConeTwistConstraint_setMaxMotorImpulseNormalized_1',
    'bind_btConeTwistConstraint_setMotorTarget_1',
    'bind_btConeTwistConstraint_setMotorTargetInConstraintSpace_1',
    'bind_btConeTwistConstraint_enableFeedback_1',
    'bind_btConeTwistConstraint_getBreakingImpulseThreshold_0',
    'bind_btConeTwistConstraint_setBreakingImpulseThreshold_1',
    'bind_btConeTwistConstraint_getParam_2',
    'bind_btConeTwistConstraint_setParam_3',
    'bind_btConeTwistConstraint___destroy___0',
    'bind_btHingeConstraint_btHingeConstraint_2',
    'bind_btHingeConstraint_btHingeConstraint_3',
    'bind_btHingeConstraint_btHingeConstraint_4',
    'bind_btHingeConstraint_btHingeConstraint_5',
    'bind_btHingeConstraint_btHingeConstraint_6',
    'bind_btHingeConstraint_btHingeConstraint_7',
    'bind_btHingeConstraint_getHingeAngle_0',
    'bind_btHingeConstraint_setLimit_4',
    'bind_btHingeConstraint_setLimit_5',
    'bind_btHingeConstraint_enableAngularMotor_3',
    'bind_btHingeConstraint_setAngularOnly_1',
    'bind_btHingeConstraint_enableMotor_1',
    'bind_btHingeConstraint_setMaxMotorImpulse_1',
    'bind_btHingeConstraint_setMotorTarget_2',
    'bind_btHingeConstraint_enableFeedback_1',
    'bind_btHingeConstraint_getBreakingImpulseThreshold_0',
    'bind_btHingeConstraint_setBreakingImpulseThreshold_1',
    'bind_btHingeConstraint_getParam_2',
    'bind_btHingeConstraint_setParam_3',
    'bind_btHingeConstraint___destroy___0',
    'bind_btSliderConstraint_btSliderConstraint_3',
    'bind_btSliderConstraint_btSliderConstraint_5',
    'bind_btSliderConstraint_getLinearPos_0',
    'bind_btSliderConstraint_getAngularPos_0',
    'bind_btSliderConstraint_setLowerLinLimit_1',
    'bind_btSliderConstraint_setUpperLinLimit_1',
    'bind_btSliderConstraint_setLowerAngLimit_1',
    'bind_btSliderConstraint_setUpperAngLimit_1',
    'bind_btSliderConstraint_setPoweredLinMotor_1',
    'bind_btSliderConstraint_setMaxLinMotorForce_1',
    'bind_btSliderConstraint_setTargetLinMotorVelocity_1',
    'bind_btSliderConstraint_enableFeedback_1',
    'bind_btSliderConstraint_getBreakingImpulseThreshold_0',
    'bind_btSliderConstraint_setBreakingImpulseThreshold_1',
    'bind_btSliderConstraint_getParam_2',
    'bind_btSliderConstraint_setParam_3',
    'bind_btSliderConstraint___destroy___0',
    'bind_btFixedConstraint_btFixedConstraint_4',
    'bind_btFixedConstraint_enableFeedback_1',
    'bind_btFixedConstraint_getBreakingImpulseThreshold_0',
    'bind_btFixedConstraint_setBreakingImpulseThreshold_1',
    'bind_btFixedConstraint_getParam_2',
    'bind_btFixedConstraint_setParam_3',
    'bind_btFixedConstraint___destroy___0',
    'bind_btConstraintSolver___destroy___0',
    'bind_btDispatcherInfo_get_m_timeStep_0',
    'bind_btDispatcherInfo_set_m_timeStep_1',
    'bind_btDispatcherInfo_get_m_stepCount_0',
    'bind_btDispatcherInfo_set_m_stepCount_1',
    'bind_btDispatcherInfo_get_m_dispatchFunc_0',
    'bind_btDispatcherInfo_set_m_dispatchFunc_1',
    'bind_btDispatcherInfo_get_m_timeOfImpact_0',
    'bind_btDispatcherInfo_set_m_timeOfImpact_1',
    'bind_btDispatcherInfo_get_m_useContinuous_0',
    'bind_btDispatcherInfo_set_m_useContinuous_1',
    'bind_btDispatcherInfo_get_m_enableSatConvex_0',
    'bind_btDispatcherInfo_set_m_enableSatConvex_1',
    'bind_btDispatcherInfo_get_m_enableSPU_0',
    'bind_btDispatcherInfo_set_m_enableSPU_1',
    'bind_btDispatcherInfo_get_m_useEpa_0',
    'bind_btDispatcherInfo_set_m_useEpa_1',
    'bind_btDispatcherInfo_get_m_allowedCcdPenetration_0',
    'bind_btDispatcherInfo_set_m_allowedCcdPenetration_1',
    'bind_btDispatcherInfo_get_m_useConvexConservativeDistanceUtil_0',
    'bind_btDispatcherInfo_set_m_useConvexConservativeDistanceUtil_1',
    'bind_btDispatcherInfo_get_m_convexConservativeDistanceThreshold_0',
    'bind_btDispatcherInfo_set_m_convexConservativeDistanceThreshold_1',
    'bind_btDispatcherInfo___destroy___0',
    'bind_btContactSolverInfo_get_m_splitImpulse_0',
    'bind_btContactSolverInfo_set_m_splitImpulse_1',
    'bind_btContactSolverInfo_get_m_splitImpulsePenetrationThreshold_0',
    'bind_btContactSolverInfo_set_m_splitImpulsePenetrationThreshold_1',
    'bind_btContactSolverInfo_get_m_numIterations_0',
    'bind_btContactSolverInfo_set_m_numIterations_1',
    'bind_btContactSolverInfo___destroy___0',
    'bind_btVehicleTuning_btVehicleTuning_0',
    'bind_btVehicleTuning_get_m_suspensionStiffness_0',
    'bind_btVehicleTuning_set_m_suspensionStiffness_1',
    'bind_btVehicleTuning_get_m_suspensionCompression_0',
    'bind_btVehicleTuning_set_m_suspensionCompression_1',
    'bind_btVehicleTuning_get_m_suspensionDamping_0',
    'bind_btVehicleTuning_set_m_suspensionDamping_1',
    'bind_btVehicleTuning_get_m_maxSuspensionTravelCm_0',
    'bind_btVehicleTuning_set_m_maxSuspensionTravelCm_1',
    'bind_btVehicleTuning_get_m_frictionSlip_0',
    'bind_btVehicleTuning_set_m_frictionSlip_1',
    'bind_btVehicleTuning_get_m_maxSuspensionForce_0',
    'bind_btVehicleTuning_set_m_maxSuspensionForce_1',
    'bind_btVehicleRaycasterResult_get_m_hitPointInWorld_0',
    'bind_btVehicleRaycasterResult_set_m_hitPointInWorld_1',
    'bind_btVehicleRaycasterResult_get_m_hitNormalInWorld_0',
    'bind_btVehicleRaycasterResult_set_m_hitNormalInWorld_1',
    'bind_btVehicleRaycasterResult_get_m_distFraction_0',
    'bind_btVehicleRaycasterResult_set_m_distFraction_1',
    'bind_btVehicleRaycasterResult___destroy___0',
    'bind_btDefaultVehicleRaycaster_btDefaultVehicleRaycaster_1',
    'bind_btDefaultVehicleRaycaster_castRay_3',
    'bind_btDefaultVehicleRaycaster___destroy___0',
    'bind_RaycastInfo_get_m_contactNormalWS_0',
    'bind_RaycastInfo_set_m_contactNormalWS_1',
    'bind_RaycastInfo_get_m_contactPointWS_0',
    'bind_RaycastInfo_set_m_contactPointWS_1',
    'bind_RaycastInfo_get_m_suspensionLength_0',
    'bind_RaycastInfo_set_m_suspensionLength_1',
    'bind_RaycastInfo_get_m_hardPointWS_0',
    'bind_RaycastInfo_set_m_hardPointWS_1',
    'bind_RaycastInfo_get_m_wheelDirectionWS_0',
    'bind_RaycastInfo_set_m_wheelDirectionWS_1',
    'bind_RaycastInfo_get_m_wheelAxleWS_0',
    'bind_RaycastInfo_set_m_wheelAxleWS_1',
    'bind_RaycastInfo_get_m_isInContact_0',
    'bind_RaycastInfo_set_m_isInContact_1',
    'bind_RaycastInfo_get_m_groundObject_0',
    'bind_RaycastInfo_set_m_groundObject_1',
    'bind_RaycastInfo___destroy___0',
    'bind_btWheelInfoConstructionInfo_get_m_chassisConnectionCS_0',
    'bind_btWheelInfoConstructionInfo_set_m_chassisConnectionCS_1',
    'bind_btWheelInfoConstructionInfo_get_m_wheelDirectionCS_0',
    'bind_btWheelInfoConstructionInfo_set_m_wheelDirectionCS_1',
    'bind_btWheelInfoConstructionInfo_get_m_wheelAxleCS_0',
    'bind_btWheelInfoConstructionInfo_set_m_wheelAxleCS_1',
    'bind_btWheelInfoConstructionInfo_get_m_suspensionRestLength_0',
    'bind_btWheelInfoConstructionInfo_set_m_suspensionRestLength_1',
    'bind_btWheelInfoConstructionInfo_get_m_maxSuspensionTravelCm_0',
    'bind_btWheelInfoConstructionInfo_set_m_maxSuspensionTravelCm_1',
    'bind_btWheelInfoConstructionInfo_get_m_wheelRadius_0',
    'bind_btWheelInfoConstructionInfo_set_m_wheelRadius_1',
    'bind_btWheelInfoConstructionInfo_get_m_suspensionStiffness_0',
    'bind_btWheelInfoConstructionInfo_set_m_suspensionStiffness_1',
    'bind_btWheelInfoConstructionInfo_get_m_wheelsDampingCompression_0',
    'bind_btWheelInfoConstructionInfo_set_m_wheelsDampingCompression_1',
    'bind_btWheelInfoConstructionInfo_get_m_wheelsDampingRelaxation_0',
    'bind_btWheelInfoConstructionInfo_set_m_wheelsDampingRelaxation_1',
    'bind_btWheelInfoConstructionInfo_get_m_frictionSlip_0',
    'bind_btWheelInfoConstructionInfo_set_m_frictionSlip_1',
    'bind_btWheelInfoConstructionInfo_get_m_maxSuspensionForce_0',
    'bind_btWheelInfoConstructionInfo_set_m_maxSuspensionForce_1',
    'bind_btWheelInfoConstructionInfo_get_m_bIsFrontWheel_0',
    'bind_btWheelInfoConstructionInfo_set_m_bIsFrontWheel_1',
    'bind_btWheelInfoConstructionInfo___destroy___0',
    'bind_btWheelInfo_btWheelInfo_1',
    'bind_btWheelInfo_getSuspensionRestLength_0',
    'bind_btWheelInfo_updateWheel_2',
    'bind_btWheelInfo_get_m_suspensionStiffness_0',
    'bind_btWheelInfo_set_m_suspensionStiffness_1',
    'bind_btWheelInfo_get_m_frictionSlip_0',
    'bind_btWheelInfo_set_m_frictionSlip_1',
    'bind_btWheelInfo_get_m_engineForce_0',
    'bind_btWheelInfo_set_m_engineForce_1',
    'bind_btWheelInfo_get_m_rollInfluence_0',
    'bind_btWheelInfo_set_m_rollInfluence_1',
    'bind_btWheelInfo_get_m_suspensionRestLength1_0',
    'bind_btWheelInfo_set_m_suspensionRestLength1_1',
    'bind_btWheelInfo_get_m_wheelsRadius_0',
    'bind_btWheelInfo_set_m_wheelsRadius_1',
    'bind_btWheelInfo_get_m_wheelsDampingCompression_0',
    'bind_btWheelInfo_set_m_wheelsDampingCompression_1',
    'bind_btWheelInfo_get_m_wheelsDampingRelaxation_0',
    'bind_btWheelInfo_set_m_wheelsDampingRelaxation_1',
    'bind_btWheelInfo_get_m_steering_0',
    'bind_btWheelInfo_set_m_steering_1',
    'bind_btWheelInfo_get_m_maxSuspensionForce_0',
    'bind_btWheelInfo_set_m_maxSuspensionForce_1',
    'bind_btWheelInfo_get_m_maxSuspensionTravelCm_0',
    'bind_btWheelInfo_set_m_maxSuspensionTravelCm_1',
    'bind_btWheelInfo_get_m_wheelsSuspensionForce_0',
    'bind_btWheelInfo_set_m_wheelsSuspensionForce_1',
    'bind_btWheelInfo_get_m_bIsFrontWheel_0',
    'bind_btWheelInfo_set_m_bIsFrontWheel_1',
    'bind_btWheelInfo_get_m_raycastInfo_0',
    'bind_btWheelInfo_set_m_raycastInfo_1',
    'bind_btWheelInfo_get_m_chassisConnectionPointCS_0',
    'bind_btWheelInfo_set_m_chassisConnectionPointCS_1',
    'bind_btWheelInfo_get_m_worldTransform_0',
    'bind_btWheelInfo_set_m_worldTransform_1',
    'bind_btWheelInfo_get_m_wheelDirectionCS_0',
    'bind_btWheelInfo_set_m_wheelDirectionCS_1',
    'bind_btWheelInfo_get_m_wheelAxleCS_0',
    'bind_btWheelInfo_set_m_wheelAxleCS_1',
    'bind_btWheelInfo_get_m_rotation_0',
    'bind_btWheelInfo_set_m_rotation_1',
    'bind_btWheelInfo_get_m_deltaRotation_0',
    'bind_btWheelInfo_set_m_deltaRotation_1',
    'bind_btWheelInfo_get_m_brake_0',
    'bind_btWheelInfo_set_m_brake_1',
    'bind_btWheelInfo_get_m_clippedInvContactDotSuspension_0',
    'bind_btWheelInfo_set_m_clippedInvContactDotSuspension_1',
    'bind_btWheelInfo_get_m_suspensionRelativeVelocity_0',
    'bind_btWheelInfo_set_m_suspensionRelativeVelocity_1',
    'bind_btWheelInfo_get_m_skidInfo_0',
    'bind_btWheelInfo_set_m_skidInfo_1',
    'bind_btWheelInfo___destroy___0',
    'bind_btKinematicCharacterController_btKinematicCharacterController_3',
    'bind_btKinematicCharacterController_btKinematicCharacterController_4',
    'bind_btKinematicCharacterController_setUpAxis_1',
    'bind_btKinematicCharacterController_setWalkDirection_1',
    'bind_btKinematicCharacterController_setVelocityForTimeInterval_2',
    'bind_btKinematicCharacterController_warp_1',
    'bind_btKinematicCharacterController_preStep_1',
    'bind_btKinematicCharacterController_playerStep_2',
    'bind_btKinematicCharacterController_setFallSpeed_1',
    'bind_btKinematicCharacterController_setJumpSpeed_1',
    'bind_btKinematicCharacterController_setMaxJumpHeight_1',
    'bind_btKinematicCharacterController_canJump_0',
    'bind_btKinematicCharacterController_jump_0',
    'bind_btKinematicCharacterController_setGravity_1',
    'bind_btKinematicCharacterController_getGravity_0',
    'bind_btKinematicCharacterController_setMaxSlope_1',
    'bind_btKinematicCharacterController_getMaxSlope_0',
    'bind_btKinematicCharacterController_getGhostObject_0',
    'bind_btKinematicCharacterController_setUseGhostSweepTest_1',
    'bind_btKinematicCharacterController_onGround_0',
    'bind_btKinematicCharacterController_setUpInterpolate_1',
    'bind_btKinematicCharacterController_updateAction_2',
    'bind_btKinematicCharacterController___destroy___0',
    'bind_btRaycastVehicle_btRaycastVehicle_3',
    'bind_btRaycastVehicle_applyEngineForce_2',
    'bind_btRaycastVehicle_setSteeringValue_2',
    'bind_btRaycastVehicle_getWheelTransformWS_1',
    'bind_btRaycastVehicle_updateWheelTransform_2',
    'bind_btRaycastVehicle_addWheel_7',
    'bind_btRaycastVehicle_getNumWheels_0',
    'bind_btRaycastVehicle_getRigidBody_0',
    'bind_btRaycastVehicle_getWheelInfo_1',
    'bind_btRaycastVehicle_setBrake_2',
    'bind_btRaycastVehicle_setCoordinateSystem_3',
    'bind_btRaycastVehicle_getCurrentSpeedKmHour_0',
    'bind_btRaycastVehicle_getChassisWorldTransform_0',
    'bind_btRaycastVehicle_rayCast_1',
    'bind_btRaycastVehicle_updateVehicle_1',
    'bind_btRaycastVehicle_resetSuspension_0',
    'bind_btRaycastVehicle_getSteeringValue_1',
    'bind_btRaycastVehicle_updateWheelTransformsWS_1',
    'bind_btRaycastVehicle_updateWheelTransformsWS_2',
    'bind_btRaycastVehicle_setPitchControl_1',
    'bind_btRaycastVehicle_updateSuspension_1',
    'bind_btRaycastVehicle_updateFriction_1',
    'bind_btRaycastVehicle_getRightAxis_0',
    'bind_btRaycastVehicle_getUpAxis_0',
    'bind_btRaycastVehicle_getForwardAxis_0',
    'bind_btRaycastVehicle_getForwardVector_0',
    'bind_btRaycastVehicle_getUserConstraintType_0',
    'bind_btRaycastVehicle_setUserConstraintType_1',
    'bind_btRaycastVehicle_setUserConstraintId_1',
    'bind_btRaycastVehicle_getUserConstraintId_0',
    'bind_btRaycastVehicle_updateAction_2',
    'bind_btRaycastVehicle___destroy___0',
    'bind_btPairCachingGhostObject_btPairCachingGhostObject_0',
    'bind_btPairCachingGhostObject_setAnisotropicFriction_2',
    'bind_btPairCachingGhostObject_getCollisionShape_0',
    'bind_btPairCachingGhostObject_setContactProcessingThreshold_1',
    'bind_btPairCachingGhostObject_setActivationState_1',
    'bind_btPairCachingGhostObject_forceActivationState_1',
    'bind_btPairCachingGhostObject_activate_0',
    'bind_btPairCachingGhostObject_activate_1',
    'bind_btPairCachingGhostObject_isActive_0',
    'bind_btPairCachingGhostObject_isKinematicObject_0',
    'bind_btPairCachingGhostObject_isStaticObject_0',
    'bind_btPairCachingGhostObject_isStaticOrKinematicObject_0',
    'bind_btPairCachingGhostObject_getRestitution_0',
    'bind_btPairCachingGhostObject_getFriction_0',
    'bind_btPairCachingGhostObject_getRollingFriction_0',
    'bind_btPairCachingGhostObject_setRestitution_1',
    'bind_btPairCachingGhostObject_setFriction_1',
    'bind_btPairCachingGhostObject_setRollingFriction_1',
    'bind_btPairCachingGhostObject_getWorldTransform_0',
    'bind_btPairCachingGhostObject_getCollisionFlags_0',
    'bind_btPairCachingGhostObject_setCollisionFlags_1',
    'bind_btPairCachingGhostObject_setWorldTransform_1',
    'bind_btPairCachingGhostObject_setCollisionShape_1',
    'bind_btPairCachingGhostObject_setCcdMotionThreshold_1',
    'bind_btPairCachingGhostObject_setCcdSweptSphereRadius_1',
    'bind_btPairCachingGhostObject_getUserIndex_0',
    'bind_btPairCachingGhostObject_setUserIndex_1',
    'bind_btPairCachingGhostObject_getUserPointer_0',
    'bind_btPairCachingGhostObject_setUserPointer_1',
    'bind_btPairCachingGhostObject_getBroadphaseHandle_0',
    'bind_btPairCachingGhostObject_getNumOverlappingObjects_0',
    'bind_btPairCachingGhostObject_getOverlappingObject_1',
    'bind_btPairCachingGhostObject___destroy___0',
    'bind_btGhostPairCallback_btGhostPairCallback_0',
    'bind_btGhostPairCallback___destroy___0',
    'bind_btSoftBodyWorldInfo_btSoftBodyWorldInfo_0',
    'bind_btSoftBodyWorldInfo_get_air_density_0',
    'bind_btSoftBodyWorldInfo_set_air_density_1',
    'bind_btSoftBodyWorldInfo_get_water_density_0',
    'bind_btSoftBodyWorldInfo_set_water_density_1',
    'bind_btSoftBodyWorldInfo_get_water_offset_0',
    'bind_btSoftBodyWorldInfo_set_water_offset_1',
    'bind_btSoftBodyWorldInfo_get_m_maxDisplacement_0',
    'bind_btSoftBodyWorldInfo_set_m_maxDisplacement_1',
    'bind_btSoftBodyWorldInfo_get_water_normal_0',
    'bind_btSoftBodyWorldInfo_set_water_normal_1',
    'bind_btSoftBodyWorldInfo_get_m_broadphase_0',
    'bind_btSoftBodyWorldInfo_set_m_broadphase_1',
    'bind_btSoftBodyWorldInfo_get_m_dispatcher_0',
    'bind_btSoftBodyWorldInfo_set_m_dispatcher_1',
    'bind_btSoftBodyWorldInfo_get_m_gravity_0',
    'bind_btSoftBodyWorldInfo_set_m_gravity_1',
    'bind_btSoftBodyWorldInfo___destroy___0',
    'bind_Face_get_m_n_1',
    'bind_Face_set_m_n_2',
    'bind_Face_get_m_normal_0',
    'bind_Face_set_m_normal_1',
    'bind_Face_get_m_ra_0',
    'bind_Face_set_m_ra_1',
    'bind_Face___destroy___0',
    'bind_tFaceArray_size_0',
    'bind_tFaceArray_at_1',
    'bind_tFaceArray___destroy___0',
    'bind_Node_get_m_x_0',
    'bind_Node_set_m_x_1',
    'bind_Node_get_m_q_0',
    'bind_Node_set_m_q_1',
    'bind_Node_get_m_v_0',
    'bind_Node_set_m_v_1',
    'bind_Node_get_m_f_0',
    'bind_Node_set_m_f_1',
    'bind_Node_get_m_n_0',
    'bind_Node_set_m_n_1',
    'bind_Node_get_m_im_0',
    'bind_Node_set_m_im_1',
    'bind_Node_get_m_area_0',
    'bind_Node_set_m_area_1',
    'bind_Node___destroy___0',
    'bind_tNodeArray_size_0',
    'bind_tNodeArray_at_1',
    'bind_tNodeArray___destroy___0',
    'bind_Material_get_m_kLST_0',
    'bind_Material_set_m_kLST_1',
    'bind_Material_get_m_kAST_0',
    'bind_Material_set_m_kAST_1',
    'bind_Material_get_m_kVST_0',
    'bind_Material_set_m_kVST_1',
    'bind_Material_get_m_flags_0',
    'bind_Material_set_m_flags_1',
    'bind_Material___destroy___0',
    'bind_tMaterialArray_size_0',
    'bind_tMaterialArray_at_1',
    'bind_tMaterialArray___destroy___0',
    'bind_Anchor_get_m_node_0',
    'bind_Anchor_set_m_node_1',
    'bind_Anchor_get_m_local_0',
    'bind_Anchor_set_m_local_1',
    'bind_Anchor_get_m_body_0',
    'bind_Anchor_set_m_body_1',
    'bind_Anchor_get_m_influence_0',
    'bind_Anchor_set_m_influence_1',
    'bind_Anchor_get_m_c0_0',
    'bind_Anchor_set_m_c0_1',
    'bind_Anchor_get_m_c1_0',
    'bind_Anchor_set_m_c1_1',
    'bind_Anchor_get_m_c2_0',
    'bind_Anchor_set_m_c2_1',
    'bind_Anchor___destroy___0',
    'bind_tAnchorArray_size_0',
    'bind_tAnchorArray_at_1',
    'bind_tAnchorArray_clear_0',
    'bind_tAnchorArray_push_back_1',
    'bind_tAnchorArray_pop_back_0',
    'bind_tAnchorArray___destroy___0',
    'bind_Config_get_kVCF_0',
    'bind_Config_set_kVCF_1',
    'bind_Config_get_kDP_0',
    'bind_Config_set_kDP_1',
    'bind_Config_get_kDG_0',
    'bind_Config_set_kDG_1',
    'bind_Config_get_kLF_0',
    'bind_Config_set_kLF_1',
    'bind_Config_get_kPR_0',
    'bind_Config_set_kPR_1',
    'bind_Config_get_kVC_0',
    'bind_Config_set_kVC_1',
    'bind_Config_get_kDF_0',
    'bind_Config_set_kDF_1',
    'bind_Config_get_kMT_0',
    'bind_Config_set_kMT_1',
    'bind_Config_get_kCHR_0',
    'bind_Config_set_kCHR_1',
    'bind_Config_get_kKHR_0',
    'bind_Config_set_kKHR_1',
    'bind_Config_get_kSHR_0',
    'bind_Config_set_kSHR_1',
    'bind_Config_get_kAHR_0',
    'bind_Config_set_kAHR_1',
    'bind_Config_get_kSRHR_CL_0',
    'bind_Config_set_kSRHR_CL_1',
    'bind_Config_get_kSKHR_CL_0',
    'bind_Config_set_kSKHR_CL_1',
    'bind_Config_get_kSSHR_CL_0',
    'bind_Config_set_kSSHR_CL_1',
    'bind_Config_get_kSR_SPLT_CL_0',
    'bind_Config_set_kSR_SPLT_CL_1',
    'bind_Config_get_kSK_SPLT_CL_0',
    'bind_Config_set_kSK_SPLT_CL_1',
    'bind_Config_get_kSS_SPLT_CL_0',
    'bind_Config_set_kSS_SPLT_CL_1',
    'bind_Config_get_maxvolume_0',
    'bind_Config_set_maxvolume_1',
    'bind_Config_get_timescale_0',
    'bind_Config_set_timescale_1',
    'bind_Config_get_viterations_0',
    'bind_Config_set_viterations_1',
    'bind_Config_get_piterations_0',
    'bind_Config_set_piterations_1',
    'bind_Config_get_diterations_0',
    'bind_Config_set_diterations_1',
    'bind_Config_get_citerations_0',
    'bind_Config_set_citerations_1',
    'bind_Config_get_collisions_0',
    'bind_Config_set_collisions_1',
    'bind_Config___destroy___0',
    'bind_btSoftBody_btSoftBody_4',
    'bind_btSoftBody_checkLink_2',
    'bind_btSoftBody_checkFace_3',
    'bind_btSoftBody_appendMaterial_0',
    'bind_btSoftBody_appendNode_2',
    'bind_btSoftBody_appendLink_4',
    'bind_btSoftBody_appendFace_4',
    'bind_btSoftBody_appendTetra_5',
    'bind_btSoftBody_appendAnchor_4',
    'bind_btSoftBody_addForce_1',
    'bind_btSoftBody_addForce_2',
    'bind_btSoftBody_addAeroForceToNode_2',
    'bind_btSoftBody_getTotalMass_0',
    'bind_btSoftBody_setTotalMass_2',
    'bind_btSoftBody_setMass_2',
    'bind_btSoftBody_transform_1',
    'bind_btSoftBody_translate_1',
    'bind_btSoftBody_rotate_1',
    'bind_btSoftBody_scale_1',
    'bind_btSoftBody_generateClusters_1',
    'bind_btSoftBody_generateClusters_2',
    'bind_btSoftBody_generateBendingConstraints_2',
    'bind_btSoftBody_upcast_1',
    'bind_btSoftBody_getRestLengthScale_0',
    'bind_btSoftBody_setRestLengthScale_1',
    'bind_btSoftBody_setAnisotropicFriction_2',
    'bind_btSoftBody_getCollisionShape_0',
    'bind_btSoftBody_setContactProcessingThreshold_1',
    'bind_btSoftBody_setActivationState_1',
    'bind_btSoftBody_forceActivationState_1',
    'bind_btSoftBody_activate_0',
    'bind_btSoftBody_activate_1',
    'bind_btSoftBody_isActive_0',
    'bind_btSoftBody_isKinematicObject_0',
    'bind_btSoftBody_isStaticObject_0',
    'bind_btSoftBody_isStaticOrKinematicObject_0',
    'bind_btSoftBody_getRestitution_0',
    'bind_btSoftBody_getFriction_0',
    'bind_btSoftBody_getRollingFriction_0',
    'bind_btSoftBody_setRestitution_1',
    'bind_btSoftBody_setFriction_1',
    'bind_btSoftBody_setRollingFriction_1',
    'bind_btSoftBody_getWorldTransform_0',
    'bind_btSoftBody_getCollisionFlags_0',
    'bind_btSoftBody_setCollisionFlags_1',
    'bind_btSoftBody_setWorldTransform_1',
    'bind_btSoftBody_setCollisionShape_1',
    'bind_btSoftBody_setCcdMotionThreshold_1',
    'bind_btSoftBody_setCcdSweptSphereRadius_1',
    'bind_btSoftBody_getUserIndex_0',
    'bind_btSoftBody_setUserIndex_1',
    'bind_btSoftBody_getUserPointer_0',
    'bind_btSoftBody_setUserPointer_1',
    'bind_btSoftBody_getBroadphaseHandle_0',
    'bind_btSoftBody_get_m_cfg_0',
    'bind_btSoftBody_set_m_cfg_1',
    'bind_btSoftBody_get_m_nodes_0',
    'bind_btSoftBody_set_m_nodes_1',
    'bind_btSoftBody_get_m_faces_0',
    'bind_btSoftBody_set_m_faces_1',
    'bind_btSoftBody_get_m_materials_0',
    'bind_btSoftBody_set_m_materials_1',
    'bind_btSoftBody_get_m_anchors_0',
    'bind_btSoftBody_set_m_anchors_1',
    'bind_btSoftBody___destroy___0',
    'bind_btSoftBodyRigidBodyCollisionConfiguration_btSoftBodyRigidBodyCollisionConfiguration_0',
    'bind_btSoftBodyRigidBodyCollisionConfiguration_btSoftBodyRigidBodyCollisionConfiguration_1',
    'bind_btSoftBodyRigidBodyCollisionConfiguration___destroy___0',
    'bind_btDefaultSoftBodySolver_btDefaultSoftBodySolver_0',
    'bind_btDefaultSoftBodySolver___destroy___0',
    'bind_btSoftBodyArray_size_0',
    'bind_btSoftBodyArray_at_1',
    'bind_btSoftBodyArray___destroy___0',
    'bind_btSoftRigidDynamicsWorld_btSoftRigidDynamicsWorld_5',
    'bind_btSoftRigidDynamicsWorld_addSoftBody_3',
    'bind_btSoftRigidDynamicsWorld_removeSoftBody_1',
    'bind_btSoftRigidDynamicsWorld_removeCollisionObject_1',
    'bind_btSoftRigidDynamicsWorld_getWorldInfo_0',
    'bind_btSoftRigidDynamicsWorld_getSoftBodyArray_0',
    'bind_btSoftRigidDynamicsWorld_getDispatcher_0',
    'bind_btSoftRigidDynamicsWorld_rayTest_3',
    'bind_btSoftRigidDynamicsWorld_getPairCache_0',
    'bind_btSoftRigidDynamicsWorld_getDispatchInfo_0',
    'bind_btSoftRigidDynamicsWorld_addCollisionObject_1',
    'bind_btSoftRigidDynamicsWorld_addCollisionObject_2',
    'bind_btSoftRigidDynamicsWorld_addCollisionObject_3',
    'bind_btSoftRigidDynamicsWorld_getBroadphase_0',
    'bind_btSoftRigidDynamicsWorld_convexSweepTest_5',
    'bind_btSoftRigidDynamicsWorld_contactPairTest_3',
    'bind_btSoftRigidDynamicsWorld_contactTest_2',
    'bind_btSoftRigidDynamicsWorld_updateSingleAabb_1',
    'bind_btSoftRigidDynamicsWorld_setGravity_1',
    'bind_btSoftRigidDynamicsWorld_getGravity_0',
    'bind_btSoftRigidDynamicsWorld_addRigidBody_1',
    'bind_btSoftRigidDynamicsWorld_addRigidBody_3',
    'bind_btSoftRigidDynamicsWorld_removeRigidBody_1',
    'bind_btSoftRigidDynamicsWorld_addConstraint_1',
    'bind_btSoftRigidDynamicsWorld_addConstraint_2',
    'bind_btSoftRigidDynamicsWorld_removeConstraint_1',
    'bind_btSoftRigidDynamicsWorld_stepSimulation_1',
    'bind_btSoftRigidDynamicsWorld_stepSimulation_2',
    'bind_btSoftRigidDynamicsWorld_stepSimulation_3',
    'bind_btSoftRigidDynamicsWorld_setContactAddedCallback_1',
    'bind_btSoftRigidDynamicsWorld_setContactProcessedCallback_1',
    'bind_btSoftRigidDynamicsWorld_setContactDestroyedCallback_1',
    'bind_btSoftRigidDynamicsWorld_addAction_1',
    'bind_btSoftRigidDynamicsWorld_removeAction_1',
    'bind_btSoftRigidDynamicsWorld_getSolverInfo_0',
    'bind_btSoftRigidDynamicsWorld_setInternalTickCallback_1',
    'bind_btSoftRigidDynamicsWorld_setInternalTickCallback_2',
    'bind_btSoftRigidDynamicsWorld_setInternalTickCallback_3',
    'bind_btSoftRigidDynamicsWorld___destroy___0',
    'bind_btSoftBodyHelpers_btSoftBodyHelpers_0',
    'bind_btSoftBodyHelpers_CreateRope_5',
    'bind_btSoftBodyHelpers_CreatePatch_9',
    'bind_btSoftBodyHelpers_CreatePatchUV_10',
    'bind_btSoftBodyHelpers_CreateEllipsoid_4',
    'bind_btSoftBodyHelpers_CreateFromTriMesh_5',
    'bind_btSoftBodyHelpers_CreateFromConvexHull_4',
    'bind_btSoftBodyHelpers___destroy___0',
    'enum_PHY_ScalarType_PHY_FLOAT',
    'enum_PHY_ScalarType_PHY_DOUBLE',
    'enum_PHY_ScalarType_PHY_INTEGER',
    'enum_PHY_ScalarType_PHY_SHORT',
    'enum_PHY_ScalarType_PHY_FIXEDPOINT88',
    'enum_PHY_ScalarType_PHY_UCHAR',
    'enum_eGIMPACT_SHAPE_TYPE_CONST_GIMPACT_COMPOUND_SHAPE',
    'enum_eGIMPACT_SHAPE_TYPE_CONST_GIMPACT_TRIMESH_SHAPE_PART',
    'enum_eGIMPACT_SHAPE_TYPE_CONST_GIMPACT_TRIMESH_SHAPE',
    'enum_btConstraintParams_BT_CONSTRAINT_ERP',
    'enum_btConstraintParams_BT_CONSTRAINT_STOP_ERP',
    'enum_btConstraintParams_BT_CONSTRAINT_CFM',
    'enum_btConstraintParams_BT_CONSTRAINT_STOP_CFM',
    'init',
    'stack_restore',
    'stack_get_current',
    'cxa_increment_exception_refcount'
];
export interface AmmoWasmModule extends Record<AmmoExports[number], (...args: (number | boolean)[]) => number> {
    memory: WebAssembly.Memory;
}
interface AmmoCallbacks {
    [key: string]: {
        internalTickCallback?: (timeStep: number) => void;
        internalPreTickCallback?: (timeStep: number) => void;
    } | undefined;
}
export declare class EventEmitter {
    static readonly listeners: WeakMap<EventEmitter, {
        [type: string | symbol]: ((...args: any[]) => any)[] | undefined;
    }>;
    on(type: string | symbol, callback: (...args: any[]) => any): void;
    once(type: string | symbol, callback: (...args: any[]) => any): void;
    off(type: string | symbol, callback: (...args: any[]) => any): void;
    emit(type: string | symbol, args?: any[]): void;
}
declare const DESTROY_FUNC: unique symbol;
declare const INIT_MEMBERS: unique symbol;
export declare const asm: AmmoWasmModule;
export declare const callback: AmmoCallbacks;
export declare const CONTACT_ADDED_CALLBACK_SIGNATURE = "iiiiiiii";
export declare const CONTACT_DESTROYED_CALLBACK_SIGNATURE = "ii";
export declare const CONTACT_PROCESSED_CALLBACK_SIGNATURE = "iiii";
export declare const INTERNAL_TICK_CALLBACK_SIGNATURE = "vif";
export interface AmmoInitConfig {
    memory?: WebAssembly.Memory;
}
export declare function init(config?: AmmoInitConfig): void;
export declare function destroy(...objs: any[]): void;
export declare abstract class btObject extends EventEmitter {
    static from<T extends typeof btObject>(this: T, pointer: number, parent?: btObject): InstanceType<T>;
    static stackAlloc<T extends typeof btObject>(this: T, ...args: ConstructorParameters<T>): InstanceType<T>;
    static stackAlloc<T extends typeof btObject>(this: T, ...args: any[]): InstanceType<T>;
    static cache: {
        [key: string]: btObject | undefined;
    };
    pointer: number;
    destroyed: boolean;
    members: btObject[];
    constructor(...args: any[]);
    [DESTROY_FUNC](): void;
    [INIT_MEMBERS](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtCollisionShape)*/
export declare abstract class btCollisionShape extends btObject {
    setLocalScaling(scaling: Vector3): void;
    getLocalScaling(): btVector3;
    calculateLocalInertia(mass: number, inertia: btVector3): void;
    setMargin(margin: number): void;
    getMargin(): number;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtCollisionWorld)*/
export declare abstract class btCollisionWorld extends btObject {
    getDispatcher(): btDispatcher;
    rayTest(rayFromWorld: btVector3, rayToWorld: btVector3, resultCallback: btCollisionWorld.RayResultCallback): void;
    getPairCache(): btOverlappingPairCache;
    getDispatchInfo(): btDispatcherInfo;
    addCollisionObject(collisionObject: btCollisionObject, collisionFilterGroup?: CollisionFilterGroups, collisionFilterMask?: CollisionFilterGroups): void;
    removeCollisionObject(collisionObject: btCollisionObject): void;
    getBroadphase(): btBroadphaseInterface;
    convexSweepTest(castShape: btConvexShape, from: btTransform, to: btTransform, resultCallback: btCollisionWorld.ConvexResultCallback, allowedCcdPenetration: number): void;
    contactPairTest(colObjA: btCollisionObject, colObjB: btCollisionObject, resultCallback: btCollisionWorld.ContactResultCallback): void;
    contactTest(colObj: btCollisionObject, resultCallback: btCollisionWorld.ContactResultCallback): void;
    updateSingleAabb(colObj: btCollisionObject): void;
}
export declare namespace btCollisionWorld {
    /**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtCollisionWorld_1_1RayResultCallback)*/
    abstract class RayResultCallback extends btObject {
        hasHit(): boolean;
        get collisionFilterGroup(): number;
        set collisionFilterGroup(v: number);
        get collisionFilterMask(): number;
        set collisionFilterMask(v: number);
        get closestHitFraction(): number;
        set closestHitFraction(v: number);
        get collisionObject(): btCollisionObject;
        set collisionObject(obj: btCollisionObject);
        get flags(): number;
        set flags(v: number);
    }
    /**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtCollisionWorld_1_1ContactResultCallback)*/
    abstract class ContactResultCallback extends btObject {
        addSingleResult(cp: btManifoldPoint, colObj0Wrap: btCollisionObjectWrapper, partId0: number, index0: number, colObj1Wrap: btCollisionObjectWrapper, partId1: number, index1: number): number;
    }
    /**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtCollisionWorld_1_1ConvexResultCallback)*/
    abstract class ConvexResultCallback extends btObject {
        hasHit(): boolean;
        get collisionFilterGroup(): number;
        set collisionFilterGroup(v: number);
        get collisionFilterMask(): number;
        set collisionFilterMask(v: number);
        get closestHitFraction(): number;
        set closestHitFraction(v: number);
    }
    /**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtCollisionWorld_1_1ClosestRayResultCallback)*/
    class ClosestRayResultCallback extends btCollisionWorld.RayResultCallback {
        constructor(from: btVector3, to: btVector3);
        get rayFromWorld(): btVector3;
        set rayFromWorld(v: btVector3);
        get rayToWorld(): btVector3;
        set rayToWorld(v: btVector3);
        get hitNormalWorld(): btVector3;
        set hitNormalWorld(v: btVector3);
        get hitPointWorld(): btVector3;
        set hitPointWorld(v: btVector3);
        [DESTROY_FUNC](): void;
    }
    /**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtCollisionWorld_1_1AllHitsRayResultCallback)*/
    class AllHitsRayResultCallback extends btCollisionWorld.RayResultCallback {
        constructor(from: btVector3, to: btVector3);
        get collisionObjects(): btConstCollisionObjectArray;
        set collisionObjects(cco: btConstCollisionObjectArray);
        get rayFromWorld(): btVector3;
        set rayFromWorld(v: btVector3);
        get rayToWorld(): btVector3;
        set rayToWorld(v: btVector3);
        get hitNormalWorld(): btVector3Array;
        set hitNormalWorld(va: btVector3Array);
        get hitPointWorld(): btVector3Array;
        set hitPointWorld(va: btVector3Array);
        get hitFractions(): btScalarArray;
        set hitFractions(sa: btScalarArray);
        [DESTROY_FUNC](): void;
    }
    /**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtCollisionWorld_1_1LocalShapeInfo)*/
    abstract class LocalShapeInfo extends btObject {
        get shapePart(): number;
        set shapePart(n: number);
        get triangleIndex(): number;
        set triangleIndex(n: number);
    }
    /**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtCollisionWorld_1_1LocalConvexResult)*/
    class LocalConvexResult extends btObject {
        constructor(hitCollisionObject: btCollisionObject, localShapeInfo: btCollisionWorld.LocalShapeInfo, hitNormalLocal: btVector3, hitPointLocal: btVector3, hitFraction: number);
        get hitCollisionObject(): btCollisionObject;
        set hitCollisionObject(co: btCollisionObject);
        get localShapeInfo(): btCollisionWorld.LocalShapeInfo;
        set localShapeInfo(lsi: btCollisionWorld.LocalShapeInfo);
        get hitNormalLocal(): btVector3;
        set hitNormalLocal(v: btVector3);
        get hitPointLocal(): btVector3;
        set hitPointLocal(v: btVector3);
        get hitFraction(): number;
        set hitFraction(n: number);
        [DESTROY_FUNC](): void;
    }
    /**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtCollisionWorld_1_1ClosestConvexResultCallback)*/
    class ClosestConvexResultCallback extends btCollisionWorld.ConvexResultCallback {
        constructor(convexFromWorld: btVector3, convexToWorld: btVector3);
        get hitCollisionObject(): btCollisionObject;
        set hitCollisionObject(co: btCollisionObject);
        get convexFromWorld(): btVector3;
        set convexFromWorld(v: btVector3);
        get convexToWorld(): btVector3;
        set convexToWorld(v: btVector3);
        get hitNormalWorld(): btVector3;
        set hitNormalWorld(v: btVector3);
        get hitPointWorld(): btVector3;
        set hitPointWorld(v: btVector3);
        [DESTROY_FUNC](): void;
    }
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtCollisionObject)*/
export declare abstract class btCollisionObject extends btObject {
    setAnisotropicFriction(anisotropicFriction: btVector3, frictionMode: number): void;
    getCollisionShape(): btCollisionShape;
    setContactProcessingThreshold(contactProcessingThreshold: number): void;
    setActivationState(newState: ActivationState): void;
    forceActivationState(newState: ActivationState): void;
    activate(forceActivation?: boolean): void;
    isActive(): boolean;
    isKinematicObject(): boolean;
    isStaticObject(): boolean;
    isStaticOrKinematicObject(): boolean;
    getRestitution(): number;
    getFriction(): number;
    getRollingFriction(): number;
    setRestitution(rest: number): void;
    setFriction(frict: number): void;
    setRollingFriction(frict: number): void;
    getWorldTransform(): btTransform;
    /**return: @see {@link CollisionFlags}*/
    getCollisionFlags(): number;
    /**flags: @see {@link CollisionFlags}*/
    setCollisionFlags(flags: number): void;
    setWorldTransform(worldTrans: btTransform): void;
    setCollisionShape(collisionShape: btCollisionShape): void;
    setCcdMotionThreshold(ccdMotionThreshold: number): void;
    setCcdSweptSphereRadius(radius: number): void;
    getUserIndex(): number;
    setUserIndex(index: number): void;
    getUserPointer(): VoidPtr;
    setUserPointer(userPointer: VoidPtr): void;
    getBroadphaseHandle(): btBroadphaseProxy;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtConcaveShape)*/
export declare abstract class btConcaveShape extends btCollisionShape {
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtCollisionAlgorithm)*/
export declare abstract class btCollisionAlgorithm extends btObject {
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtTypedConstraint)*/
export declare abstract class btTypedConstraint extends btObject {
    enableFeedback(needsFeedback: boolean): void;
    getBreakingImpulseThreshold(): number;
    setBreakingImpulseThreshold(threshold: number): void;
    getParam(num: number, axis: number): number;
    setParam(num: number, value: number, axis: number): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtDynamicsWorld)*/
export declare abstract class btDynamicsWorld extends btCollisionWorld {
    addAction(action: btActionInterface): this;
    removeAction(action: btActionInterface): this;
    getSolverInfo(): btContactSolverInfo;
    setInternalTickCallback(cb?: (timeStep: number) => void, isPreTick?: boolean): this;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtVector3)*/
export declare class btVector3 extends btObject {
    static stackAlloc<T extends typeof btObject>(this: T, x?: number, y?: number, z?: number): InstanceType<T>;
    static stackAlloc<T extends typeof btObject>(this: T, v: Vector3): InstanceType<T>;
    view: Float32Array;
    constructor(x?: number, y?: number, z?: number);
    constructor(v: Vector3);
    [INIT_MEMBERS](): void;
    get x(): number;
    set x(x: number);
    get y(): number;
    set y(y: number);
    get z(): number;
    set z(z: number);
    set(x?: number, y?: number, z?: number): this;
    set(v: Vector3): this;
    length(): number;
    normalize(): this;
    rotate(wAxis: btVector3, angle: number): btVector3;
    dot(v: btVector3): number;
    mul(x: number): this;
    add(v: btVector3): this;
    sub(v: btVector3): this;
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtMotionState)*/
export declare abstract class btMotionState extends btObject {
    getWorldTransform(worldTrans: btTransform): void;
    setWorldTransform(worldTrans: btTransform): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtConvexShape)*/
export declare abstract class btConvexShape extends btCollisionShape {
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtCapsuleShape)*/
export declare class btCapsuleShape extends btCollisionShape {
    constructor(radius: number, height?: number);
    getUpAxis(): number;
    getRadius(): number;
    getHalfHeight(): number;
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtCylinderShape)*/
export declare class btCylinderShape extends btCollisionShape {
    constructor(halfExtents: Vector3);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtConeShape)*/
export declare class btConeShape extends btCollisionShape {
    constructor(radius: number, height?: number);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtStridingMeshInterface)*/
export declare abstract class btStridingMeshInterface extends btObject {
    setScaling(scaling: Vector3): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtTriangleMeshShape)*/
export declare abstract class btTriangleMeshShape extends btConcaveShape {
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtPrimitiveManagerBase)*/
export declare abstract class btPrimitiveManagerBase extends btObject {
    isTrimesh(): boolean;
    getPrimitiveCount(): number;
    getPrimitiveBox(index: number, primbox: btAABB): void;
    getPrimitiveTriangle(index: number, triangle: btPrimitiveTriangle): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtGImpactShapeInterface)*/
export declare abstract class btGImpactShapeInterface extends btConcaveShape {
    updateBound(): void;
    postUpdate(): void;
    getShapeType(): number;
    getName(): string;
    getGImpactShapeType(): number;
    getPrimitiveManager(): btPrimitiveManagerBase;
    getNumChildShapes(): number;
    childrenHasTransform(): boolean;
    needsRetrieveTriangles(): boolean;
    needsRetrieveTetrahedrons(): boolean;
    getBulletTriangle(index: number, triangle: btTriangleShapeEx): void;
    getBulletTetrahedron(index: number, tetrahedron: btTetrahedronShapeEx): void;
    getChildShape(index: number): btCollisionShape;
    getChildTransform(index: number): btTransform;
    setChildTransform(index: number, transform: btTransform): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtActivatingCollisionAlgorithm)*/
export declare abstract class btActivatingCollisionAlgorithm extends btCollisionAlgorithm {
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtDefaultCollisionConfiguration)*/
export declare class btDefaultCollisionConfiguration extends btObject {
    constructor(info?: btDefaultCollisionConstructionInfo);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtDispatcher)*/
export declare abstract class btDispatcher extends btObject {
    getNumManifolds(): number;
    getManifoldByIndexInternal(index: number): btPersistentManifold;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtGeneric6DofConstraint)*/
export declare class btGeneric6DofConstraint extends btTypedConstraint {
    static stackAlloc<T extends typeof btObject>(this: T, rbA: btRigidBody, rbB: btRigidBody, frameInA: btTransform, frameInB: btTransform, useLinearFrameReferenceFrameA: boolean): InstanceType<T>;
    static stackAlloc<T extends typeof btObject>(this: T, rbB: btRigidBody, frameInB: btTransform, useLinearFrameReferenceFrameB: boolean): InstanceType<T>;
    constructor(rbA: btRigidBody, rbB: btRigidBody, frameInA: btTransform, frameInB: btTransform, useLinearFrameReferenceFrameA: boolean);
    constructor(rbB: btRigidBody, frameInB: btTransform, useLinearFrameReferenceFrameB: boolean);
    setLinearLowerLimit(linearLower: Vector3): void;
    setLinearUpperLimit(linearUpper: Vector3): void;
    setAngularLowerLimit(angularLower: Vector3): void;
    setAngularUpperLimit(angularUpper: Vector3): void;
    getFrameOffsetA(): btTransform;
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtDiscreteDynamicsWorld)*/
export declare class btDiscreteDynamicsWorld extends btDynamicsWorld {
    constructor(dispatcher: btDispatcher, pairCache?: btBroadphaseInterface, constraintSolver?: btConstraintSolver, collisionConfiguration?: btCollisionConfiguration);
    getGravity(): btVector3;
    setGravity(x: number, y: number, z: number): this;
    setGravity(gravity: Vector3): this;
    addRigidBody(body: btRigidBody): this;
    addRigidBody(body: btRigidBody, group: number, mask: number): this;
    removeRigidBody(body: btRigidBody): this;
    addConstraint(constraint: btTypedConstraint, disableCollisionsBetweenLinkedBodies?: boolean): this;
    removeConstraint(constraint: btTypedConstraint): this;
    stepSimulation(timeStep: number, maxSubSteps?: number, fixedTimeStep?: number): number;
    setContactAddedCallback(funcpointer: number): this;
    setContactProcessedCallback(funcpointer: number): this;
    setContactDestroyedCallback(funcpointer: number): this;
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtVehicleRaycaster)*/
export declare abstract class btVehicleRaycaster extends btObject {
    castRay(from: btVector3, to: btVector3, result: btVehicleRaycaster.btVehicleRaycasterResult): void;
}
export declare namespace btVehicleRaycaster {
    /**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtVehicleRaycaster_1_1btVehicleRaycasterResult)*/
    abstract class btVehicleRaycasterResult extends btObject {
        get hitPointInWorld(): btVector3;
        set hitPointInWorld(v: btVector3);
        get hitNormalInWorld(): btVector3;
        set hitNormalInWorld(v: btVector3);
        get distFraction(): number;
        set distFraction(n: number);
    }
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtActionInterface)*/
export declare abstract class btActionInterface extends btObject {
    updateAction(collisionWorld: btCollisionWorld, deltaTimeStep: number): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtGhostObject)*/
export declare class btGhostObject extends btCollisionObject {
    constructor();
    getNumOverlappingObjects(): number;
    getOverlappingObject(index: number): btCollisionObject;
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtSoftBodySolver)*/
export declare abstract class btSoftBodySolver extends btObject {
}
/***/
export declare class VoidPtr extends btObject {
    constructor(ptr?: number);
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtVector4)*/
export declare class btVector4 extends btVector3 {
    static stackAlloc<T extends typeof btObject>(this: T, x?: number, y?: number, z?: number, w?: number): InstanceType<T>;
    static stackAlloc<T extends typeof btObject>(this: T, v: Vector4): InstanceType<T>;
    constructor(x?: number, y?: number, z?: number, w?: number);
    constructor(v: Vector4);
    [INIT_MEMBERS](): void;
    get w(): number;
    set w(w: number);
    set(x?: number, y?: number, z?: number, w?: number): this;
    set(v: Vector4): this;
    normalize(): this;
    length(): number;
    dot(v: btVector4): number;
    add(v: btVector4): this;
    sub(v: btVector4): this;
    mul(s: number): this;
    rotate(wAxis: btVector3, angle: number): btVector3;
    mulq(v: btVector4): this;
    div(s: number): this;
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtQuaternion)*/
export declare class btQuaternion extends btVector4 {
    static stackAlloc<T extends typeof btObject>(this: T, x?: number, y?: number, z?: number, w?: number): InstanceType<T>;
    static stackAlloc<T extends typeof btObject>(this: T, q: Quaternion): InstanceType<T>;
    constructor(x?: number, y?: number, z?: number, w?: number);
    constructor(q: Quaternion);
    add(v: btQuaternion): this;
    sub(v: btQuaternion): this;
    mul(s: number): this;
    mulq(v: btQuaternion): this;
    div(s: number): this;
    setEulerZYX(z: number, y: number, x: number): this;
    setRotation(axis: Vector3, angle: number): this;
    length2(): number;
    normalized(): btQuaternion;
    getAxis(): btVector3;
    inverse(): btQuaternion;
    getAngle(): number;
    getAngleShortestPath(): number;
    angle(q: btQuaternion): number;
    angleShortestPath(q: btQuaternion): number;
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtMatrix3x3)*/
export declare abstract class btMatrix3x3 extends btObject {
    setEulerZYX(z?: number, y?: number, x?: number): this;
    getRotation(q: btQuaternion): void;
    getRow(y: number): btVector3;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtTransform)*/
export declare class btTransform extends btObject {
    constructor(q?: Quaternion, v?: Vector3);
    set(q: Quaternion, v: Vector3): this;
    getBasis(): btMatrix3x3;
    getOrigin(): btVector3;
    setOrigin(x: number, y: number, z: number): this;
    setOrigin(origin: Vector3): this;
    getRotation(): btQuaternion;
    setRotation(x: number, y: number, z: number, w: number): this;
    setRotation(rotation: Quaternion): this;
    setIdentity(): this;
    setFromOpenGLMatrix(m: readonly number[]): void;
    inverse(): btTransform;
    mul(t: btTransform): btTransform;
    [DESTROY_FUNC](): void;
}
/***/
export declare class MotionState extends btMotionState {
    constructor();
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtDefaultMotionState)*/
export declare class btDefaultMotionState extends btMotionState {
    constructor(startTrans?: btTransform, centerOfMassOffset?: btTransform);
    get graphicsWorldTrans(): btTransform;
    set graphicsWorldTrans(graphicsWorldTrans: btTransform);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtCollisionObjectWrapper)*/
export declare abstract class btCollisionObjectWrapper extends btObject {
    getWorldTransform(): btTransform;
    getCollisionObject(): btCollisionObject;
    getCollisionShape(): btCollisionShape;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtManifoldPoint)*/
export declare abstract class btManifoldPoint extends btObject {
    getPositionWorldOnA(): btVector3;
    getPositionWorldOnB(): btVector3;
    getAppliedImpulse(): number;
    getDistance(): number;
    get localPointA(): btVector3;
    set localPointA(v: btVector3);
    get localPointB(): btVector3;
    set localPointB(v: btVector3);
    get positionWorldOnB(): btVector3;
    set positionWorldOnB(v: btVector3);
    get positionWorldOnA(): btVector3;
    set positionWorldOnA(v: btVector3);
    get normalWorldOnB(): btVector3;
    set normalWorldOnB(v: btVector3);
    get userPersistentData(): number;
    set userPersistentData(n: number);
}
/***/
export declare class ConcreteContactResultCallback extends btCollisionWorld.ContactResultCallback {
    constructor();
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtConvexTriangleMeshShape)*/
export declare class btConvexTriangleMeshShape extends btConvexShape {
    constructor(meshInterface: btStridingMeshInterface, calcAabb?: boolean);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtBoxShape)*/
export declare class btBoxShape extends btCollisionShape {
    constructor(boxHalfExtents: Vector3);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtCapsuleShapeX)*/
export declare class btCapsuleShapeX extends btCapsuleShape {
    constructor(radius: number, height: number);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtCapsuleShapeZ)*/
export declare class btCapsuleShapeZ extends btCapsuleShape {
    constructor(radius: number, height: number);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtCylinderShapeX)*/
export declare class btCylinderShapeX extends btCylinderShape {
    constructor(halfExtents: btVector3);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtCylinderShapeZ)*/
export declare class btCylinderShapeZ extends btCylinderShape {
    constructor(halfExtents: btVector3);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtSphereShape)*/
export declare class btSphereShape extends btCollisionShape {
    constructor(radius: number);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtMultiSphereShape)*/
export declare class btMultiSphereShape extends btCollisionShape {
    constructor(positions: btVector3, radii: readonly number[], numPoints: number);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtConeShapeX)*/
export declare class btConeShapeX extends btConeShape {
    constructor(radius: number, height: number);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtConeShapeZ)*/
export declare class btConeShapeZ extends btConeShape {
    constructor(radius: number, height: number);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtFace)*/
export declare abstract class btFace extends btObject {
    get indices(): btIntArray;
    set indices(ia: btIntArray);
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtConvexPolyhedron)*/
export declare abstract class btConvexPolyhedron extends btObject {
    get vertices(): btVector3Array;
    set vertices(va: btVector3Array);
    get faces(): btFaceArray;
    set faces(fa: btFaceArray);
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtConvexHullShape)*/
export declare class btConvexHullShape extends btCollisionShape {
    constructor(points?: readonly number[], numPoints?: number);
    addPoint(point: btVector3, recalculateLocalAABB?: boolean): void;
    getNumVertices(): number;
    initializePolyhedralFeatures(shiftVerticesByMargin: number): boolean;
    recalcLocalAabb(): void;
    getConvexPolyhedron(): btConvexPolyhedron;
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtShapeHull)*/
export declare class btShapeHull extends btObject {
    constructor(shape: btConvexShape);
    buildHull(margin: number): boolean;
    numVertices(): number;
    getVertexPointer(): btVector3;
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtCompoundShape)*/
export declare class btCompoundShape extends btCollisionShape {
    constructor(enableDynamicAabbTree?: boolean);
    addChildShape(localTransform: btTransform, shape: btCollisionShape): void;
    removeChildShape(shape: btCollisionShape): void;
    removeChildShapeByIndex(childShapeindex: number): void;
    getNumChildShapes(): number;
    getChildShape(index: number): btCollisionShape;
    updateChildTransform(childIndex: number, newChildTransform: btTransform, shouldRecalculateLocalAabb?: boolean): void;
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtIndexedMesh)*/
export declare abstract class btIndexedMesh extends btObject {
    get numTriangles(): number;
    set numTriangles(n: number);
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtTriangleMesh)*/
export declare class btTriangleMesh extends btStridingMeshInterface {
    constructor(use32bitIndices?: boolean, use4componentVertices?: boolean);
    addTriangle(vertex0: btVector3, vertex1: btVector3, vertex2: btVector3, removeDuplicateVertices?: boolean): void;
    findOrAddVertex(vertex: btVector3, removeDuplicateVertices: boolean): number;
    addIndex(index: number): void;
    getIndexedMeshArray(): btIndexedMeshArray;
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtEmptyShape)*/
export declare class btEmptyShape extends btConcaveShape {
    constructor();
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtStaticPlaneShape)*/
export declare class btStaticPlaneShape extends btConcaveShape {
    constructor(planeNormal: btVector3, planeConstant: number);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtBvhTriangleMeshShape)*/
export declare class btBvhTriangleMeshShape extends btTriangleMeshShape {
    constructor(meshInterface: btStridingMeshInterface, useQuantizedAabbCompression: boolean, buildBvh?: boolean);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtHeightfieldTerrainShape)*/
export declare class btHeightfieldTerrainShape extends btConcaveShape {
    constructor(heightStickWidth: number, heightStickLength: number, heightfieldData: VoidPtr, heightScale: number, minHeight: number, maxHeight: number, upAxis: number, hdt: PHY, flipQuadEdges: boolean);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtAABB)*/
export declare class btAABB extends btObject {
    constructor(v1: btVector3, v2: btVector3, v3: btVector3, margin: number);
    invalidate(): void;
    incrementMargin(margin: number): void;
    copyWithMargin(other: btAABB, margin: number): void;
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtPrimitiveTriangle)*/
export declare class btPrimitiveTriangle extends btObject {
    constructor();
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtTriangleShapeEx)*/
export declare class btTriangleShapeEx extends btObject {
    constructor(p0: btVector3, p1: btVector3, p2: btVector3);
    getAabb(t: btTransform, aabbMin: btVector3, aabbMax: btVector3): void;
    applyTransform(t: btTransform): void;
    buildTriPlane(plane: btVector4): void;
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtTetrahedronShapeEx)*/
export declare class btTetrahedronShapeEx extends btObject {
    constructor();
    setVertices(v0: btVector3, v1: btVector3, v2: btVector3, v3: btVector3): void;
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtGImpactCompoundShape)*/
export declare class btGImpactCompoundShape extends btGImpactShapeInterface {
    constructor(childrenHasTransform?: boolean);
    getCompoundPrimitiveManager(): btGImpactCompoundShape.CompoundPrimitiveManager;
    addChildShape(localTransform: btTransform, shape: btCollisionShape): void;
    getChildAabb(childIndex: number, t: btTransform, aabbMin: btVector3, aabbMax: btVector3): void;
    getChildTransform(index: number): btTransform;
    [DESTROY_FUNC](): void;
}
export declare namespace btGImpactCompoundShape {
    /**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtGImpactCompoundShape_1_1CompoundPrimitiveManager)*/
    abstract class CompoundPrimitiveManager extends btPrimitiveManagerBase {
        get compoundShape(): btGImpactCompoundShape;
        set compoundShape(gics: btGImpactCompoundShape);
    }
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtGImpactMeshShapePart)*/
export declare class btGImpactMeshShapePart extends btGImpactShapeInterface {
    constructor(meshInterface: btStridingMeshInterface, part: number);
    getTrimeshPrimitiveManager(): btGImpactMeshShapePart.TrimeshPrimitiveManager;
    getVertexCount(): number;
    getVertex(vertexIndex: number, vertex: btVector3): void;
    getPart(): number;
    [DESTROY_FUNC](): void;
}
export declare namespace btGImpactMeshShapePart {
    /**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtGImpactMeshShapePart_1_1TrimeshPrimitiveManager)*/
    class TrimeshPrimitiveManager extends btPrimitiveManagerBase {
        constructor(manager?: TrimeshPrimitiveManager);
        lock(): void;
        unlock(): void;
        getVertexCount(): number;
        getIndices(faceIndex: number, i0: number, i1: number, i2: number): void;
        getVertex(vertexIndex: number, vertex: btVector3): void;
        getBulletTriangle(primIndex: number, triangle: btTriangleShapeEx): void;
        get margin(): number;
        set margin(n: number);
        get meshInterface(): btStridingMeshInterface;
        set meshInterface(n: btStridingMeshInterface);
        get part(): number;
        set part(n: number);
        get lock_count(): number;
        set lock_count(n: number);
        get numverts(): number;
        set numverts(n: number);
        get type(): number;
        set type(n: number);
        get stride(): number;
        set stride(n: number);
        get indexstride(): number;
        set indexstride(n: number);
        get numfaces(): number;
        set numfaces(n: number);
        get indicestype(): number;
        set indicestype(n: number);
        [DESTROY_FUNC](): void;
    }
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtGImpactMeshShape)*/
export declare class btGImpactMeshShape extends btGImpactShapeInterface {
    constructor(meshInterface: btStridingMeshInterface);
    getMeshInterface(): btStridingMeshInterface;
    getMeshPartCount(): number;
    getMeshPart(index: number): btGImpactMeshShapePart;
    calculateSerializeBufferSize(): number;
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtCollisionAlgorithmConstructionInfo)*/
export declare class btCollisionAlgorithmConstructionInfo extends btObject {
    constructor(dispatcher: btDispatcher, temp: number);
    get dispatcher1(): btDispatcher;
    set dispatcher1(d: btDispatcher);
    get manifold(): btPersistentManifold;
    set manifold(pm: btPersistentManifold);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtGImpactCollisionAlgorithm)*/
export declare class btGImpactCollisionAlgorithm extends btActivatingCollisionAlgorithm {
    constructor(ci: btCollisionAlgorithmConstructionInfo, body0Wrap: btCollisionObjectWrapper, body1Wrap: btCollisionObjectWrapper);
    registerAlgorithm(dispatcher: btCollisionDispatcher): void;
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtDefaultCollisionConstructionInfo)*/
export declare class btDefaultCollisionConstructionInfo extends btObject {
    constructor();
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtPersistentManifold)*/
export declare class btPersistentManifold extends btObject {
    constructor();
    getBody0(): btCollisionObject;
    getBody1(): btCollisionObject;
    getNumContacts(): number;
    getContactPoint(index: number): btManifoldPoint;
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtCollisionDispatcher)*/
export declare class btCollisionDispatcher extends btDispatcher {
    constructor(conf: btDefaultCollisionConfiguration);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtOverlappingPairCallback)*/
export declare abstract class btOverlappingPairCallback extends btObject {
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtOverlappingPairCache)*/
export declare abstract class btOverlappingPairCache extends btObject {
    setInternalGhostPairCallback(ghostPairCallback: btOverlappingPairCallback): void;
    getNumOverlappingPairs(): number;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtAxisSweep3)*/
export declare class btAxisSweep3 extends btObject {
    constructor(worldAabbMin: btVector3, worldAabbMax: btVector3, maxHandles?: number, pairCache?: btOverlappingPairCache, disableRaycastAccelerator?: boolean);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtBroadphaseInterface)*/
export declare abstract class btBroadphaseInterface extends btObject {
    getOverlappingPairCache(): btOverlappingPairCache;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtCollisionConfiguration)*/
export declare abstract class btCollisionConfiguration extends btObject {
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtDbvtBroadphase)*/
export declare class btDbvtBroadphase extends btBroadphaseInterface {
    constructor();
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtBroadphaseProxy)*/
export declare abstract class btBroadphaseProxy extends btObject {
    get collisionFilterGroup(): number;
    set collisionFilterGroup(n: number);
    get collisionFilterMask(): number;
    set collisionFilterMask(n: number);
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtRigidBody)*/
export declare class btRigidBody extends btCollisionObject {
    constructor(constructionInfo: btRigidBody.ConstructionInfo);
    getGravity(): btVector3;
    setGravity(x: number, y: number, z: number): void;
    setGravity(acceleration: Vector3): void;
    getMotionState(): btMotionState;
    setMotionState(motionState: btMotionState): void;
    getCenterOfMassTransform(): btTransform;
    setCenterOfMassTransform(a: btTransform): void;
    getLinearFactor(): btVector3;
    setLinearFactor(x: number, y: number, z: number): void;
    setLinearFactor(linearFactor: Vector3): void;
    getLinearVelocity(): btVector3;
    setLinearVelocity(linVel: Vector3): void;
    getAngularVelocity(): btVector3;
    setAngularVelocity(x: number, y: number, z: number): void;
    setAngularVelocity(angVel: Vector3): void;
    getAngularFactor(): btVector3;
    setAngularFactor(x: number, y: number, z: number): void;
    setAngularFactor(angularFactor: Vector3): void;
    getFlags(): number;
    setFlags(flags: number): void;
    getLinearDamping(): number;
    getAngularDamping(): number;
    setDamping(linDamping: number, angDamping: number): void;
    setSleepingThresholds(linear: number, angular: number): void;
    setMassProps(mass: number, inertia: btVector3): void;
    applyTorque(torque: Vector3): void;
    applyLocalTorque(torque: Vector3): void;
    applyForce(force: Vector3, relPos: Vector3): void;
    applyCentralForce(force: Vector3): void;
    applyCentralLocalForce(force: Vector3): void;
    applyTorqueImpulse(torque: Vector3): void;
    applyImpulse(impulse: Vector3, relPos: Vector3): void;
    applyCentralImpulse(impulse: Vector3): void;
    applyGravity(): void;
    updateInertiaTensor(): void;
    upcast(colObj: btCollisionObject): btRigidBody;
    getAabb(aabbMin: btVector3, aabbMax: btVector3): void;
    getBroadphaseProxy(): btBroadphaseProxy;
    clearForces(): void;
    [DESTROY_FUNC](): void;
}
export declare namespace btRigidBody {
    /**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtRigidBody_1_1btRigidBodyConstructionInfo)*/
    class ConstructionInfo extends btObject {
        constructor(mass: number, motionState: btMotionState, collisionShape: btCollisionShape, localInertia?: Vector3);
        get linearDamping(): number;
        set linearDamping(n: number);
        get angularDamping(): number;
        set angularDamping(n: number);
        get friction(): number;
        set friction(n: number);
        get rollingFriction(): number;
        set rollingFriction(n: number);
        get restitution(): number;
        set restitution(n: number);
        get linearSleepingThreshold(): number;
        set linearSleepingThreshold(n: number);
        get angularSleepingThreshold(): number;
        set angularSleepingThreshold(n: number);
        get additionalDamping(): boolean;
        set additionalDamping(b: boolean);
        get additionalDampingFactor(): number;
        set additionalDampingFactor(n: number);
        get additionalLinearDampingThresholdSqr(): number;
        set additionalLinearDampingThresholdSqr(n: number);
        get additionalAngularDampingThresholdSqr(): number;
        set additionalAngularDampingThresholdSqr(n: number);
        get additionalAngularDampingFactor(): number;
        set additionalAngularDampingFactor(n: number);
        [DESTROY_FUNC](): void;
    }
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtConstraintSetting)*/
export declare class btConstraintSetting extends btObject {
    constructor();
    get tau(): number;
    set tau(n: number);
    get damping(): number;
    set damping(n: number);
    get impulseClamp(): number;
    set impulseClamp(n: number);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtPoint2PointConstraint)*/
export declare class btPoint2PointConstraint extends btTypedConstraint {
    static stackAlloc<T extends typeof btObject>(this: T, rbA: btRigidBody, rbB: btRigidBody, pivotInA: btVector3, pivotInB: btVector3): InstanceType<T>;
    static stackAlloc<T extends typeof btObject>(this: T, rbA: btRigidBody, pivotInA: btVector3): InstanceType<T>;
    constructor(rbA: btRigidBody, rbB: btRigidBody, pivotInA: btVector3, pivotInB: btVector3);
    constructor(rbA: btRigidBody, pivotInA: btVector3);
    setPivotA(pivotA: Vector3): void;
    setPivotB(pivotB: Vector3): void;
    getPivotInA(): btVector3;
    getPivotInB(): btVector3;
    get setting(): btConstraintSetting;
    set setting(cs: btConstraintSetting);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtGeneric6DofSpringConstraint)*/
export declare class btGeneric6DofSpringConstraint extends btGeneric6DofConstraint {
    constructor(rbA: btRigidBody, rbB: btRigidBody, frameInA: btTransform, frameInB: btTransform, useLinearFrameReferenceFrameA: boolean);
    constructor(rbB: btRigidBody, frameInB: btTransform, useLinearFrameReferenceFrameB: boolean);
    enableSpring(index: number, onOff: boolean): void;
    setStiffness(index: number, stiffness: number): void;
    setDamping(index: number, damping: number): void;
    setEquilibriumPoint(index: number, val: number): void;
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtConstraintSolver)*/
export declare abstract class btConstraintSolver extends btObject {
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtSequentialImpulseConstraintSolver)*/
export declare class btSequentialImpulseConstraintSolver extends btConstraintSolver {
    constructor();
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtConeTwistConstraint)*/
export declare class btConeTwistConstraint extends btTypedConstraint {
    static stackAlloc<T extends typeof btObject>(this: T, rbA: btRigidBody, rbB: btRigidBody, rbAFrame: btTransform, rbBFrame: btTransform): InstanceType<T>;
    static stackAlloc<T extends typeof btObject>(this: T, rbA: btRigidBody, rbAFrame: btTransform): InstanceType<T>;
    constructor(rbA: btRigidBody, rbB: btRigidBody, rbAFrame: btTransform, rbBFrame: btTransform);
    constructor(rbA: btRigidBody, rbAFrame: btTransform);
    setLimit(limitIndex: number, limitValue: number): void;
    setAngularOnly(angularOnly: boolean): void;
    setDamping(damping: number): void;
    enableMotor(b: boolean): void;
    setMaxMotorImpulse(maxMotorImpulse: number): void;
    setMaxMotorImpulseNormalized(maxMotorImpulse: number): void;
    setMotorTarget(q: btQuaternion): void;
    setMotorTargetInConstraintSpace(q: btQuaternion): void;
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtHingeConstraint)*/
export declare class btHingeConstraint extends btTypedConstraint {
    static stackAlloc<T extends typeof btObject>(this: T, rbA: btRigidBody, rbB: btRigidBody, pivotInA: btVector3, pivotInB: btVector3, axisInA: btVector3, axisInB: btVector3, useReferenceFrameA?: boolean): InstanceType<T>;
    static stackAlloc<T extends typeof btObject>(this: T, rbA: btRigidBody, rbB: btRigidBody, rbAFrame: btTransform, rbBFrame: btTransform, useReferenceFrameA?: boolean): InstanceType<T>;
    static stackAlloc<T extends typeof btObject>(this: T, rbA: btRigidBody, rbAFrame: btTransform, useReferenceFrameA?: boolean): InstanceType<T>;
    constructor(rbA: btRigidBody, rbB: btRigidBody, pivotInA: btVector3, pivotInB: btVector3, axisInA: btVector3, axisInB: btVector3, useReferenceFrameA?: boolean);
    constructor(rbA: btRigidBody, rbB: btRigidBody, rbAFrame: btTransform, rbBFrame: btTransform, useReferenceFrameA?: boolean);
    constructor(rbA: btRigidBody, rbAFrame: btTransform, useReferenceFrameA?: boolean);
    getHingeAngle(): number;
    /**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtHingeConstraint#a99cfd186cc4f41246d1cac1ce840eb0d)*/
    setLimit(low: number, high: number, softness?: number, biasFactor?: number, relaxationFactor?: number): void;
    enableAngularMotor(enableMotor: boolean, targetVelocity: number, maxMotorImpulse: number): void;
    setAngularOnly(angularOnly: boolean): void;
    enableMotor(enableMotor: boolean): void;
    setMaxMotorImpulse(maxMotorImpulse: number): void;
    setMotorTarget(targetAngle: number, dt: number): void;
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtSliderConstraint)*/
export declare class btSliderConstraint extends btTypedConstraint {
    static stackAlloc<T extends typeof btObject>(this: T, rbA: btRigidBody, rbB: btRigidBody, frameInA: btTransform, frameInB: btTransform, useLinearReferenceFrameA: boolean): InstanceType<T>;
    static stackAlloc<T extends typeof btObject>(this: T, rbB: btRigidBody, frameInB: btTransform, useLinearReferenceFrameA: boolean): InstanceType<T>;
    constructor(rbA: btRigidBody, rbB: btRigidBody, frameInA: btTransform, frameInB: btTransform, useLinearReferenceFrameA: boolean);
    constructor(rbB: btRigidBody, frameInB: btTransform, useLinearReferenceFrameA: boolean);
    getLinearPos(): number;
    getAngularPos(): number;
    setLowerLinLimit(lowerLimit: number): void;
    setUpperLinLimit(upperLimit: number): void;
    setLowerAngLimit(lowerLimit: number): void;
    setUpperAngLimit(upperLimit: number): void;
    setPoweredLinMotor(onOff: boolean): void;
    setMaxLinMotorForce(maxLinMotorForce: number): void;
    setTargetLinMotorVelocity(targetLinMotorVelocity: number): void;
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtFixedConstraint)*/
export declare class btFixedConstraint extends btTypedConstraint {
    constructor(rbA: btRigidBody, rbB: btRigidBody, frameInA: btTransform, frameInB: btTransform);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtDispatcherInfo)*/
export declare abstract class btDispatcherInfo extends btObject {
    get timeStep(): number;
    set timeStep(n: number);
    get stepCount(): number;
    set stepCount(n: number);
    get dispatchFunc(): number;
    set dispatchFunc(n: number);
    get timeOfImpact(): number;
    set timeOfImpact(n: number);
    get useContinuous(): boolean;
    set useContinuous(b: boolean);
    get enableSatConvex(): boolean;
    set enableSatConvex(b: boolean);
    get enableSPU(): boolean;
    set enableSPU(b: boolean);
    get useEpa(): boolean;
    set useEpa(b: boolean);
    get allowedCcdPenetration(): number;
    set allowedCcdPenetration(n: number);
    get useConvexConservativeDistanceUtil(): boolean;
    set useConvexConservativeDistanceUtil(b: boolean);
    get convexConservativeDistanceThreshold(): number;
    set convexConservativeDistanceThreshold(n: number);
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtContactSolverInfo)*/
export declare abstract class btContactSolverInfo extends btObject {
    get splitImpulse(): boolean;
    set splitImpulse(b: boolean);
    get splitImpulsePenetrationThreshold(): number;
    set splitImpulsePenetrationThreshold(n: number);
    get numIterations(): number;
    set numIterations(n: number);
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtDefaultVehicleRaycaster)*/
export declare class btDefaultVehicleRaycaster extends btVehicleRaycaster {
    constructor(world: btDynamicsWorld);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtWheelInfo)*/
export declare abstract class btWheelInfoConstructionInfo extends btObject {
    get chassisConnectionCS(): btVector3;
    set chassisConnectionCS(a: btVector3);
    get wheelDirectionCS(): btVector3;
    set wheelDirectionCS(v: btVector3);
    get wheelAxleCS(): btVector3;
    set wheelAxleCS(v: btVector3);
    get suspensionRestLength(): number;
    set suspensionRestLength(n: number);
    get maxSuspensionTravelCm(): number;
    set maxSuspensionTravelCm(n: number);
    get wheelRadius(): number;
    set wheelRadius(n: number);
    get suspensionStiffness(): number;
    set suspensionStiffness(n: number);
    get wheelsDampingCompression(): number;
    set wheelsDampingCompression(n: number);
    get wheelsDampingRelaxation(): number;
    set wheelsDampingRelaxation(n: number);
    get frictionSlip(): number;
    set frictionSlip(n: number);
    get maxSuspensionForce(): number;
    set maxSuspensionForce(n: number);
    get bIsFrontWheel(): boolean;
    set bIsFrontWheel(b: boolean);
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtWheelInfo)*/
export declare class btWheelInfo extends btObject {
    constructor(ci: btWheelInfoConstructionInfo);
    getSuspensionRestLength(): number;
    updateWheel(chassis: btRigidBody, raycastInfo: btWheelInfo.RaycastInfo): void;
    get suspensionStiffness(): number;
    set suspensionStiffness(n: number);
    get frictionSlip(): number;
    set frictionSlip(n: number);
    get engineForce(): number;
    set engineForce(n: number);
    get rollInfluence(): number;
    set rollInfluence(n: number);
    get suspensionRestLength1(): number;
    set suspensionRestLength1(n: number);
    get wheelsRadius(): number;
    set wheelsRadius(n: number);
    get wheelsDampingCompression(): number;
    set wheelsDampingCompression(n: number);
    get wheelsDampingRelaxation(): number;
    set wheelsDampingRelaxation(n: number);
    get steering(): number;
    set steering(n: number);
    get maxSuspensionForce(): number;
    set maxSuspensionForce(n: number);
    get maxSuspensionTravelCm(): number;
    set maxSuspensionTravelCm(n: number);
    get wheelsSuspensionForce(): number;
    set wheelsSuspensionForce(n: number);
    get bIsFrontWheel(): boolean;
    set bIsFrontWheel(b: boolean);
    get raycastInfo(): btWheelInfo.RaycastInfo;
    set raycastInfo(ri: btWheelInfo.RaycastInfo);
    get chassisConnectionPointCS(): btVector3;
    set chassisConnectionPointCS(v: btVector3);
    get worldTransform(): btTransform;
    set worldTransform(t: btTransform);
    get wheelDirectionCS(): btVector3;
    set wheelDirectionCS(v: btVector3);
    get wheelAxleCS(): btVector3;
    set wheelAxleCS(v: btVector3);
    get rotation(): number;
    set rotation(n: number);
    get deltaRotation(): number;
    set deltaRotation(n: number);
    get brake(): number;
    set brake(n: number);
    get clippedInvContactDotSuspension(): number;
    set clippedInvContactDotSuspension(n: number);
    get suspensionRelativeVelocity(): number;
    set suspensionRelativeVelocity(n: number);
    get skidInfo(): number;
    set skidInfo(n: number);
    [DESTROY_FUNC](): void;
}
export declare namespace btWheelInfo {
    /**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtWheelInfo_1_1RaycastInfo)*/
    abstract class RaycastInfo extends btObject {
        get contactNormalWS(): btVector3;
        set contactNormalWS(v: btVector3);
        get contactPointWS(): btVector3;
        set contactPointWS(v: btVector3);
        get suspensionLength(): number;
        set suspensionLength(n: number);
        get hardPointWS(): btVector3;
        set hardPointWS(v: btVector3);
        get wheelDirectionWS(): btVector3;
        set wheelDirectionWS(v: btVector3);
        get wheelAxleWS(): btVector3;
        set wheelAxleWS(v: btVector3);
        get isInContact(): boolean;
        set isInContact(b: boolean);
        get groundObject(): number;
        set groundObject(n: number);
    }
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtKinematicCharacterController)*/
export declare class btKinematicCharacterController extends btActionInterface {
    constructor(ghostObject: btPairCachingGhostObject, convexShape: btConvexShape, stepHeight: number, upAxis?: btVector3);
    getGravity(): number;
    setGravity(gravity: number): void;
    setUpAxis(axis: number): void;
    setWalkDirection(walkDirection: btVector3): void;
    setVelocityForTimeInterval(velocity: btVector3, timeInterval: number): void;
    warp(origin: btVector3): void;
    preStep(collisionWorld: btCollisionWorld): void;
    playerStep(collisionWorld: btCollisionWorld, dt: number): void;
    setFallSpeed(fallSpeed: number): void;
    setJumpSpeed(jumpSpeed: number): void;
    setMaxJumpHeight(maxJumpHeight: number): void;
    canJump(): boolean;
    jump(): void;
    setMaxSlope(slopeRadians: number): void;
    getMaxSlope(): number;
    getGhostObject(): btPairCachingGhostObject;
    setUseGhostSweepTest(useGhostObjectSweepTest: boolean): void;
    onGround(): boolean;
    setUpInterpolate(value: boolean): void;
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtRaycastVehicle)*/
export declare class btRaycastVehicle extends btActionInterface {
    constructor(tuning: btRaycastVehicle.btVehicleTuning, chassis: btRigidBody, raycaster: btVehicleRaycaster);
    applyEngineForce(force: number, wheel: number): void;
    setSteeringValue(steering: number, wheel: number): void;
    getWheelTransformWS(wheelIndex: number): btTransform;
    updateWheelTransform(wheelIndex: number, interpolatedTransform: boolean): void;
    addWheel(connectionPointCS0: btVector3, wheelDirectionCS0: btVector3, wheelAxleCS: btVector3, suspensionRestLength: number, wheelRadius: number, tuning: btRaycastVehicle.btVehicleTuning, isFrontWheel: boolean): btWheelInfo;
    getNumWheels(): number;
    getRigidBody(): btRigidBody;
    getWheelInfo(index: number): btWheelInfo;
    setBrake(brake: number, wheelIndex: number): void;
    setCoordinateSystem(rightIndex: number, upIndex: number, forwardIndex: number): void;
    getCurrentSpeedKmHour(): number;
    getChassisWorldTransform(): btTransform;
    rayCast(wheel: btWheelInfo): number;
    updateVehicle(step: number): void;
    resetSuspension(): void;
    getSteeringValue(wheel: number): number;
    updateWheelTransformsWS(wheel: btWheelInfo, interpolatedTransform?: boolean): void;
    setPitchControl(pitch: number): void;
    updateSuspension(deltaTime: number): void;
    updateFriction(timeStep: number): void;
    getRightAxis(): number;
    getUpAxis(): number;
    getForwardAxis(): number;
    getForwardVector(): btVector3;
    getUserConstraintType(): number;
    setUserConstraintType(userConstraintType: number): void;
    setUserConstraintId(uid: number): void;
    getUserConstraintId(): number;
    [DESTROY_FUNC](): void;
}
export declare namespace btRaycastVehicle {
    /**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtRaycastVehicle_1_1btVehicleTuning)*/
    class btVehicleTuning extends btObject {
        constructor();
        get suspensionStiffness(): number;
        set suspensionStiffness(n: number);
        get suspensionCompression(): number;
        set suspensionCompression(n: number);
        get suspensionDamping(): number;
        set suspensionDamping(n: number);
        get maxSuspensionTravelCm(): number;
        set maxSuspensionTravelCm(n: number);
        get frictionSlip(): number;
        set frictionSlip(n: number);
        get maxSuspensionForce(): number;
        set maxSuspensionForce(n: number);
    }
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtPairCachingGhostObject)*/
export declare class btPairCachingGhostObject extends btGhostObject {
    constructor();
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtGhostPairCallback)*/
export declare class btGhostPairCallback extends btObject {
    constructor();
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtSoftBodyWorldInfo)*/
export declare class btSoftBodyWorldInfo extends btObject {
    constructor();
    get airDensity(): number;
    set airDensity(n: number);
    get waterDensity(): number;
    set waterDensity(n: number);
    get waterOffset(): number;
    set waterOffset(n: number);
    get maxDisplacement(): number;
    set maxDisplacement(n: number);
    get waterNormal(): btVector3;
    set waterNormal(v: Vector3);
    get broadphase(): btBroadphaseInterface;
    set broadphase(bi: btBroadphaseInterface);
    get dispatcher(): btDispatcher;
    set dispatcher(d: btDispatcher);
    get gravity(): btVector3;
    set gravity(v: Vector3);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtSoftBody)*/
export declare class btSoftBody extends btCollisionObject {
    constructor(worldInfo: btSoftBodyWorldInfo, nodeCount: number, x: btVector3, m: readonly number[]);
    checkLink(node0: number, node1: number): boolean;
    checkFace(node0: number, node1: number, node2: number): boolean;
    appendMaterial(): btSoftBody.Material;
    appendNode(x: btVector3, m: number): void;
    appendLink(node0: number, node1: number, mat: btSoftBody.Material, bcheckexist: boolean): void;
    appendFace(node0: number, node1: number, node2: number, mat: btSoftBody.Material): void;
    appendTetra(node0: number, node1: number, node2: number, node3: number, mat: btSoftBody.Material): void;
    appendAnchor(node: number, body: btRigidBody, disableCollisionBetweenLinkedBodies: boolean, influence: number): void;
    addForce(force: btVector3, node?: number): void;
    addAeroForceToNode(windVelocity: btVector3, nodeIndex: number): void;
    getTotalMass(): number;
    setTotalMass(mass: number, fromfaces: boolean): void;
    setMass(node: number, mass: number): void;
    transform(trs: btTransform): void;
    translate(trs: btVector3): void;
    rotate(rot: btQuaternion): void;
    scale(scl: btVector3): void;
    generateClusters(k: number, maxiterations?: number): number;
    generateBendingConstraints(distance: number, mat: btSoftBody.Material): number;
    upcast(colObj: btCollisionObject): btSoftBody;
    getRestLengthScale(): number;
    setRestLengthScale(restLength: number): void;
    get cfg(): btSoftBody.Config;
    set cfg(a: btSoftBody.Config);
    get nodes(): tNodeArray;
    set nodes(a: tNodeArray);
    get faces(): tFaceArray;
    set faces(a: tFaceArray);
    get materials(): tMaterialArray;
    set materials(a: tMaterialArray);
    get anchors(): tAnchorArray;
    set anchors(a: tAnchorArray);
    [DESTROY_FUNC](): void;
}
export declare namespace btSoftBody {
    /**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtSoftBody_1_1Face)*/
    abstract class Face extends btObject {
        get normal(): btVector3;
        set normal(v: btVector3);
        get ra(): number;
        set ra(n: number);
    }
    /**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtSoftBody_1_1Note)*/
    abstract class Node extends btObject {
        get x(): btVector3;
        set x(v: btVector3);
        get q(): btVector3;
        set q(v: btVector3);
        get v(): btVector3;
        set v(v: btVector3);
        get f(): btVector3;
        set f(v: btVector3);
        get n(): btVector3;
        set n(v: btVector3);
        get im(): number;
        set im(n: number);
        get area(): number;
        set area(n: number);
    }
    /**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtSoftBody_1_1Material)*/
    abstract class Material extends btObject {
        get kLST(): number;
        set kLST(n: number);
        get kAST(): number;
        set kAST(n: number);
        get kVST(): number;
        set kVST(n: number);
        get flags(): number;
        set flags(n: number);
    }
    /**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtSoftBody_1_1Anchor)*/
    abstract class Anchor extends btObject {
        get node(): btSoftBody.Node;
        set node(nd: btSoftBody.Node);
        get local(): btVector3;
        set local(v: btVector3);
        get body(): btRigidBody;
        set body(rb: btRigidBody);
        get influence(): number;
        set influence(n: number);
        get c0(): btMatrix3x3;
        set c0(m: btMatrix3x3);
        get c1(): btVector3;
        set c1(v: btVector3);
        get c2(): number;
        set c2(n: number);
    }
    /**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtSoftBody_1_1Config)*/
    abstract class Config extends btObject {
        get kVCF(): number;
        set kVCF(n: number);
        get kDP(): number;
        set kDP(n: number);
        get kDG(): number;
        set kDG(n: number);
        get kLF(): number;
        set kLF(n: number);
        get kPR(): number;
        set kPR(n: number);
        get kVC(): number;
        set kVC(n: number);
        get kDF(): number;
        set kDF(n: number);
        get kMT(): number;
        set kMT(n: number);
        get kCHR(): number;
        set kCHR(n: number);
        get kKHR(): number;
        set kKHR(n: number);
        get kSHR(): number;
        set kSHR(n: number);
        get kAHR(): number;
        set kAHR(n: number);
        get kSRHR_CL(): number;
        set kSRHR_CL(n: number);
        get kSKHR_CL(): number;
        set kSKHR_CL(n: number);
        get kSSHR_CL(): number;
        set kSSHR_CL(n: number);
        get kSR_SPLT_CL(): number;
        set kSR_SPLT_CL(n: number);
        get kSK_SPLT_CL(): number;
        set kSK_SPLT_CL(n: number);
        get kSS_SPLT_CL(): number;
        set kSS_SPLT_CL(n: number);
        get maxvolume(): number;
        set maxvolume(n: number);
        get timescale(): number;
        set timescale(n: number);
        get viterations(): number;
        set viterations(n: number);
        get piterations(): number;
        set piterations(n: number);
        get diterations(): number;
        set diterations(n: number);
        get citerations(): number;
        set citerations(n: number);
        get collisions(): number;
        set collisions(n: number);
    }
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtSoftBodyRigidBodyCollisionConfiguration)*/
export declare class btSoftBodyRigidBodyCollisionConfiguration extends btDefaultCollisionConfiguration {
    constructor(info?: btDefaultCollisionConstructionInfo);
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtDefaultSoftBodySolver)*/
export declare class btDefaultSoftBodySolver extends btSoftBodySolver {
    constructor();
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/classbtSoftRigidDynamicsWorld)*/
export declare class btSoftRigidDynamicsWorld extends btDiscreteDynamicsWorld {
    constructor(dispatcher: btDispatcher, pairCache: btBroadphaseInterface, constraintSolver: btConstraintSolver, collisionConfiguration: btCollisionConfiguration, softBodySolver: btSoftBodySolver);
    addSoftBody(body: btSoftBody, collisionFilterGroup: CollisionFilterGroups, collisionFilterMask: CollisionFilterGroups): void;
    removeSoftBody(body: btSoftBody): void;
    getWorldInfo(): btSoftBodyWorldInfo;
    getSoftBodyArray(): btSoftBodyArray;
    [DESTROY_FUNC](): void;
}
/**[Bullet Documentation](https://pybullet.org/Bullet/BulletFull/structbtSoftBodyHelpers)*/
export declare class btSoftBodyHelpers extends btObject {
    constructor();
    createRope(worldInfo: btSoftBodyWorldInfo, from: btVector3, to: btVector3, res: number, fixeds: number): btSoftBody;
    createPatch(worldInfo: btSoftBodyWorldInfo, corner00: btVector3, corner10: btVector3, corner01: btVector3, corner11: btVector3, resx: number, resy: number, fixeds: number, gendiags: boolean): btSoftBody;
    createPatchUV(worldInfo: btSoftBodyWorldInfo, corner00: btVector3, corner10: btVector3, corner01: btVector3, corner11: btVector3, resx: number, resy: number, fixeds: number, gendiags: boolean, texCoords: readonly number[]): btSoftBody;
    createEllipsoid(worldInfo: btSoftBodyWorldInfo, center: btVector3, radius: btVector3, res: number): btSoftBody;
    createFromTriMesh(worldInfo: btSoftBodyWorldInfo, vertices: readonly number[], triangles: readonly number[], ntriangles: number, randomizeConstraints: boolean): btSoftBody;
    createFromConvexHull(worldInfo: btSoftBodyWorldInfo, vertices: btVector3, nvertices: number, randomizeConstraints: boolean): btSoftBody;
    [DESTROY_FUNC](): void;
}
/***/
export declare abstract class btConstCollisionObjectArray extends btObject {
    size(): number;
    at(index: number): btCollisionObject;
}
/***/
export declare abstract class btScalarArray extends btObject {
    size(): number;
    at(index: number): number;
}
/***/
export declare abstract class btSoftBodyArray extends btObject {
    size(): number;
    at(index: number): btSoftBody;
}
/***/
export declare abstract class tFaceArray extends btObject {
    size(): number;
    at(index: number): btSoftBody.Face;
}
/***/
export declare abstract class tNodeArray extends btObject {
    size(): number;
    at(index: number): btSoftBody.Node;
}
/***/
export declare abstract class tMaterialArray extends btObject {
    size(): number;
    at(index: number): btSoftBody.Material;
}
/***/
export declare abstract class btIntArray extends btObject {
    size(): number;
    at(index: number): number;
}
/***/
export declare abstract class btVector3Array extends btObject {
    size(): number;
    at(index: number): btVector3;
}
/***/
export declare abstract class btFaceArray extends btObject {
    size(): number;
    at(index: number): btFace;
}
/***/
export declare abstract class btIndexedMeshArray extends btObject {
    size(): number;
    at(index: number): btIndexedMesh;
}
/***/
export declare abstract class tAnchorArray extends btObject {
    push_back(a: btSoftBody.Anchor): void;
    pop_back(): void;
    size(): number;
    at(index: number): btSoftBody.Anchor;
    clear(): void;
}
export declare const moduleList: Set<typeof btObject>;
export interface WorldConfig {
    readonly gravity?: Vector3;
}
export declare class World extends btDiscreteDynamicsWorld {
    readonly collisionConfiguration: btDefaultCollisionConfiguration;
    readonly dispatcher: btCollisionDispatcher;
    readonly pairCache: btDbvtBroadphase;
    readonly constraintSolver: btSequentialImpulseConstraintSolver;
    constructor(config?: WorldConfig);
}
export interface SoftWorldConfig {
    readonly gravity?: Vector3;
    readonly softGravity?: Vector3;
    readonly airDensity?: number;
    readonly waterDensity?: number;
    readonly waterOffset?: number;
    readonly waterNormal?: Vector3;
    readonly maxDisplacement?: number;
    readonly useBroadphase?: boolean;
    readonly useDispatcher?: boolean;
}
export declare class SoftWorld extends btSoftRigidDynamicsWorld {
    readonly softBodyWorldInfo: btSoftBodyWorldInfo;
    readonly collisionConfiguration: btSoftBodyRigidBodyCollisionConfiguration;
    readonly dispatcher: btCollisionDispatcher;
    readonly pairCache: btDbvtBroadphase;
    readonly constraintSolver: btSequentialImpulseConstraintSolver;
    readonly softBodySolver: btDefaultSoftBodySolver;
    constructor(config?: SoftWorldConfig);
    setSoftGravity(x: number, y: number, z: number): this;
    setSoftGravity(gravity: Vector3): this;
}
export interface RigidBodyConfig {
    readonly mass?: number;
    readonly friction?: number;
    readonly restitution?: number;
    readonly rollingFriction?: number;
    readonly contactProcessingThreshold?: number;
    readonly activationState?: ActivationState;
    /**@see {@link CollisionFlags}*/
    readonly collisionFlags?: number;
    readonly rotation?: Quaternion;
    readonly position?: Vector3;
    readonly halfExtents?: Vector3;
    readonly linearFactor?: Vector3;
    readonly angularFactor?: Vector3;
    readonly gravity?: Vector3;
    readonly damping?: {
        lin: number;
        ang: number;
    };
}
export declare class RigidBody extends btRigidBody {
    private _mass;
    private _motionState;
    constructor(config?: RigidBodyConfig);
    get mass(): number;
    set mass(v: number);
    getState(): Transform;
    setState(transform: Transform): this;
    setPosition(position: Vector3): this;
    setRotation(rotation: Quaternion): this;
}
export declare class Vector3 {
    x: number;
    y: number;
    z: number;
    constructor(x?: number, y?: number, z?: number);
}
export interface Vector3 {
    [key: string | symbol]: any;
}
export declare class Vector4 extends Vector3 {
    w: number;
    constructor(x?: number, y?: number, z?: number, w?: number);
}
export declare class Quaternion extends Vector4 {
    constructor(x?: number, y?: number, z?: number, w?: number);
}
export declare class Transform {
    rotation: Quaternion;
    position: Vector3;
    constructor(rotation?: Quaternion, position?: Vector3);
}
export interface Transform {
    [key: string | symbol]: any;
}
export {};
