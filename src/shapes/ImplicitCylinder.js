module.exports = ImplicitCylinder;

var Shape = require('./Shape');
var Vec3 = require('../math/Vec3');
var Quaternion = require('../math/Quaternion');

/**
 * Implicitly-defined cylinder:
 *
 *    x^2 + z^2 = R^2,
 *
 *    -0.5*H <= y <= 0.5*H
 *
 * Note that this does not match the orientation of the existing
 * (ConvexPolyhedron-based) Cylinder class, which defines the circular
 * cross-section within the x/y plane.
 *
 * @class ImplicitCylinder
 * @constructor
 * @author jzitelli / https://github.com/jzitelli
 * @param {Number} radius
 * @param {Number} height
 */
function ImplicitCylinder( radius, height ) {
    Shape.call(this);
    this.type = Shape.types.IMPLICITCYLINDER;
    this.radius = radius;
    this.height = height;
    if (this.radius <= 0){
        throw new Error('The cylinder radius must be positive.');
    }
    if (this.height <= 0) {
        throw new Error('The cylinder height must be positive.');
    }
    this.updateBoundingSphereRadius();
}

ImplicitCylinder.prototype = new Shape();
ImplicitCylinder.prototype.constructor = ImplicitCylinder;

ImplicitCylinder.prototype.volume = function() {
    return Math.PI * this.radius * this.radius * this.height;
};

ImplicitCylinder.prototype.calculateLocalInertia = function (mass,target) {
    // see http://scienceworld.wolfram.com/physics/MomentofInertiaCylinder.html
    target = target || new Vec3();
    target.x = (1/12) * (mass * this.height * this.height) + (1/4) * (mass * this.radius * this.radius);
    target.z = target.x;
    target.y = (1/2) * (mass * this.radius * this.radius);
    return target;
};

ImplicitCylinder.prototype.updateBoundingSphereRadius = function () {
    this.boundingSphereRadius = Math.sqrt(this.radius * this.radius + this.height * this.height / 4);
};

var euler = new Vec3();
var temp = new Vec3();
ImplicitCylinder.prototype.calculateWorldAABB = function (pos, quat, min, max) {
    // determine AA bounds for the ring x^2 + z^2 = R^2 after rotation:
    var R = this.radius;
    quat.toEuler(euler, 'YZX');
    var psi = euler.x;
    var phi = euler.z;
    var cphi = Math.cos(phi),
        sphi = Math.sin(phi);
    var cpsi = Math.cos(psi),
        spsi = Math.sin(psi);
    max.set(R * Math.abs(cphi),
            R * Math.sqrt(cpsi*cpsi * sphi*sphi + spsi*spsi),
            R * Math.sqrt(spsi*spsi * sphi*sphi + cpsi*cpsi));
    max.negate(min);
    // now consider translations which place the rings on the top/bottom ends of the cylinder:
    temp.set(0, 0.5*this.height, 0);
    quat.vmult(temp, temp);
    max.set(Math.max(temp.x + max.x, -temp.x + max.x),
            Math.max(temp.y + max.y, -temp.y + max.y),
            Math.max(temp.z + max.z, -temp.z + max.z));
    min.set(Math.min(temp.x + min.x, -temp.x + min.x),
            Math.min(temp.y + min.y, -temp.y + min.y),
            Math.min(temp.z + min.z, -temp.z + min.z));
    max.vadd(pos, max);
    min.vadd(pos, min);
};
