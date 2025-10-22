// import {btTransform, destroy, init as ammoInit} from './ammo-es.js';
// main();
// async function main() {
//     await ammoInit();
//     const trs = new btTransform();
//     const {origin, rotation} = trs;
//     destroy(trs);
//     console.log(trs, origin, rotation);
// }
import * as Ammo from './ammo-es.js';
Ammo.init();
globalThis.Ammo = Ammo;
main();
async function main() {
    const w = new Ammo.World();
    // f(Ammo, w, 1);
    const b = new Ammo.RigidBody({
        mass: 1,
        gravity: new Ammo.Vector3(0, 1, 0)
    });
    w.addRigidBody(b);

    // globalThis.w = w;
    // console.log(Ammo.AnisotropicFrictionFlags);

    let transform = new Ammo.btTransform();
    // transform.setIdentity();
    // console.log(transform.getRotation());
    setInterval(() => {
        // w.stepSimulation(1 / 60);
        // b.getMotionState().getWorldTransform(transform); // 取得世界變換
        // const o = transform.getOrigin();
        // console.log(o.x, o.y, o.z); // btVector3
    }, 1000 / 60);

    // const v = new Ammo.btVector3();
    // Ammo.destroy(v);
    // console.log(v);
    // console.log(v.rotate(new Ammo.btVector3(1, 2, 3), 10), v);
    // console.log(v.x, v.y, v.z);

    // const w2 = new Ammo.World();
    // f(Ammo, w2, 1000);

    // console.time();
    // for (let i = 0; i < 100; i++) {
    //     w2.stepSimulation(1 / 60, 0);
    // }
    // console.timeEnd();
}

globalThis.bodies = [];
function f(AmmoLib, dynamicsWorld, n = 1) {
    // 建立剛體形狀（立方體）
    const boxShape = new AmmoLib.btBoxShape(new AmmoLib.btVector3(1, 1, 1));

    // 創建剛體
    for (let i = 0; i < n; i++) {
        const startTransform = new AmmoLib.btTransform();
        startTransform.setIdentity();

        // 隨機分佈位置
        startTransform.setOrigin(Math.random() * 20 - 10, Math.random() * 20 + 5, Math.random() * 20 - 10);

        const mass = 1;
        const isDynamic = mass !== 0;

        const localInertia = new AmmoLib.btVector3(0, 0, 0);
        if (isDynamic) boxShape.calculateLocalInertia(mass, localInertia);

        const motionState = new AmmoLib.btDefaultMotionState(startTransform);
        const rbInfo = new AmmoLib.btRigidBodyConstructionInfo(mass, motionState, boxShape, localInertia);
        const body = new AmmoLib.btRigidBody(rbInfo);
        body.setGravity(new AmmoLib.btVector3(0, -5, 0))
        dynamicsWorld.addRigidBody(body);
        bodies.push(body);
    }
}
