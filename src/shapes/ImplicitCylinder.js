module.exports = ImplicitCylinder;

var Shape = require('./Shape');
var Vec3 = require('../math/Vec3');
var Quaternion = require('../math/Quaternion');

/**
 * Implicitly-defined cylinder:
 *
 *    x^2 + y^2 = R^2,
 *
 *    -0.5*H <= z <= 0.5*H
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
    if(this.radius < 0){
        throw new Error('The cylinder radius cannot be negative.');
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
    target.y = target.x;
    target.z = (1/2) * (mass * this.radius * this.radius);
    return target;
};

ImplicitCylinder.prototype.updateBoundingSphereRadius = function () {
    this.boundingSphereRadius = Math.sqrt(this.radius * this.radius + this.height * this.height / 4);
};

var rotatedLengthsTemp = new Vec3();
ImplicitCylinder.prototype.calculateWorldAABB = function (pos,quat,min,max) {
    var r = this.radius;
    var h = this.height;
    rotatedLengthsTemp.set(r, r, h/2);
    quat.vmult(rotatedLengthsTemp, rotatedLengthsTemp);
    var a = rotatedLengthsTemp.x,
        b = rotatedLengthsTemp.y,
        c = rotatedLengthsTemp.z;
    min.x = pos.x - a;
    max.x = pos.x + a;
    min.y = pos.y - b;
    max.y = pos.y + b;
    min.z = pos.z - c;
    max.z = pos.z + c;
};
