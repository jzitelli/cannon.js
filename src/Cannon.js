// Export classes
module.exports = {
    version :                       require('../package.json').version,

    ArrayCollisionMatrix :          require('./collision/ArrayCollisionMatrix'),
    Body :                          require('./objects/Body'),
    Box :                           require('./shapes/Box'),
    Broadphase :                    require('./collision/Broadphase'),
    Compound :                      require('./shapes/Compound'),
    Constraint :                    require('./constraints/Constraint'),
    ContactEquation :               require('./equations/ContactEquation'),
    ContactGenerator :              require('./world/ContactGenerator'),
    ContactMaterial :               require('./material/ContactMaterial'),
    ConvexPolyhedron :              require('./shapes/ConvexPolyhedron'),
    Cylinder :                      require('./shapes/Cylinder'),
    DistanceConstraint :            require('./constraints/DistanceConstraint'),
    Equation :                      require('./equations/Equation'),
    EventTarget :                   require('./utils/EventTarget'),
    FrictionEquation :              require('./equations/FrictionEquation'),
    GSSolver :                      require('./solver/GSSolver'),
    GridBroadphase :                require('./collision/GridBroadphase'),
    Heightfield :                   require('./shapes/Heightfield'),
    HingeConstraint :               require('./constraints/HingeConstraint'),
    Mat3 :                          require('./math/Mat3'),
    Material :                      require('./material/Material'),
    NaiveBroadphase :               require('./collision/NaiveBroadphase'),
    ObjectCollisionMatrix :         require('./collision/ObjectCollisionMatrix'),
    Pool :                          require('./utils/Pool'),
    Particle :                      require('./objects/Particle'),
    Plane :                         require('./shapes/Plane'),
    PointToPointConstraint :        require('./constraints/PointToPointConstraint'),
    Quaternion :                    require('./math/Quaternion'),
    Ray :                           require('./collision/Ray'),
    RigidBody :                     require('./objects/RigidBody'),
    RotationalEquation :            require('./equations/RotationalEquation'),
    RotationalMotorEquation :       require('./equations/RotationalMotorEquation'),
    SAPBroadphase :               require('./collision/SAPBroadphase'),
    SPHSystem :                     require('./objects/SPHSystem'),
    Shape :                         require('./shapes/Shape'),
    Solver :                        require('./solver/Solver'),
    Sphere :                        require('./shapes/Sphere'),
    SplitSolver :                   require('./solver/SplitSolver'),
    Spring :                        require('./objects/Spring'),
    Vec3 :                          require('./math/Vec3'),
    Vec3Pool :                      require('./utils/Vec3Pool'),
    World :                         require('./world/World'),
};
