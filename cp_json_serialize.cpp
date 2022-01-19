

#include "cp_json_serialize.hpp"
#include <chipmunk/chipmunk_private.h>
#include <chipmunk/chipmunk_structs.h>
#include <chipmunk/chipmunk_types.h>
std::string ptrToStr(const void *ptr) {
  std::stringstream ss;
  ss << std::hex << ptr;
  return ss.str();
}

namespace nlohmann {

template <> struct adl_serializer<struct cpTransform> {
  static void to_json(json &j, const struct cpTransform &t) {
    j["a"] = t.a;
    j["b"] = t.b;
    j["c"] = t.c;
    j["d"] = t.d;
    j["tx"] = t.tx;
    j["ty"] = t.ty;
  }

  static void from_json(const json &j, cpTransform &t) {
    t.a = j["a"];
    t.b = j["b"];
    t.c = j["c"];
    t.d = j["d"];
    t.tx = j["tx"];
    t.ty = j["ty"];
  }
};

template <> struct adl_serializer<struct cpVect> {
  static void to_json(json &j, const struct cpVect &v) { j = {v.x, v.y}; }

  static void from_json(const json &j, cpVect &v) {
    v.x = j[0].get<cpFloat>();
    v.y = j[1].get<cpFloat>();
  }
};

template <> struct adl_serializer<struct cpShapeFilter> {
  static void to_json(json &j, const struct cpShapeFilter &f) {
    j = {f.group, f.categories, f.mask};
  }

  static void from_json(const json &j, cpShapeFilter &f) {
    f.group = j[0].get<cpGroup>();
    f.categories = j[1].get<cpBitmask>();
    f.mask = j[2].get<cpBitmask>();
  }
};

template <> struct adl_serializer<struct cpBB> {
  static void to_json(json &j, const struct cpBB &bb) {
    j["l"] = bb.l;
    j["b"] = bb.b;
    j["r"] = bb.r;
    j["t"] = bb.t;
  }

  static void from_json(const json &j, cpBB &bb) {
    bb.l = j["l"].get<cpFloat>();
    bb.b = j["b"].get<cpFloat>();
    bb.r = j["r"].get<cpFloat>();
    bb.t = j["t"].get<cpFloat>();
  }
};
template <> struct adl_serializer<struct cpShapeMassInfo> {
  static void to_json(json &j, const struct cpShapeMassInfo &mi) {
    j["m"] = mi.m;
    j["i"] = mi.i;
    j["cog"] = mi.cog;
    j["area"] = mi.area;
  }

  static void from_json(const json &j, cpShapeMassInfo &mi) {
    mi.m = j["m"].get<cpFloat>();
    mi.i = j["i"].get<cpFloat>();
    mi.cog = j["cog"].get<cpVect>();
    mi.area = j["area"].get<cpFloat>();
  }
};

template <> struct adl_serializer<struct cpShape *> {
  static cpBody *body;
  static cpSpace *space;

  static void to_json(json &j, const struct cpShape *shape) {
    switch (shape->klass->type) {
    case CP_CIRCLE_SHAPE: {

      j["type"] = "circle";
      cpCircleShape *circle = (cpCircleShape *)shape;
      j["c"] = circle->c;
      j["r"] = circle->r;
      break;
    }

    case CP_SEGMENT_SHAPE: {
      cpSegmentShape *seg = (cpSegmentShape *)shape;
      j["type"] = "segment";
      j["a"] = seg->a;
      j["b"] = seg->b;
      j["r"] = seg->r;
      break;
    }

    case CP_POLY_SHAPE: {
      j["type"] = "poly";
      cpPolyShape *poly = (cpPolyShape *)shape;
      std::size_t count = poly->count;
      std::vector<cpVect> verts(count);
      for (auto i = 0; i < count; i++) {
        verts[i] = poly->planes[i + count].v0;
      }
      j["verts"] = verts;
      j["r"] = poly->r;

      j["ctype"] = shape->type;
      j["filter"] = shape->filter;
      j["sensor"] = shape->sensor;
      j["e"] = shape->e;
      j["u"] = shape->u;
      j["surfaceV"] = shape->surfaceV;
      j["bb"] = shape->bb;
      j["userData"] = (uint64_t)shape->userData;

      j["raw"] = true;
      break;
    }
    }
  }

  static cpShape *from_json(const json &j) {
    cpShape *shape = nullptr;
    if (j == nullptr) {
      return shape;
    }
    if (j["type"] == "circle") {
      shape =
          cpCircleShapeNew(body, j["r"].get<cpFloat>(), j["c"].get<cpVect>());
    }
    if (j["type"] == "segment") {
      shape = cpSegmentShapeNew(body, j["a"].get<cpVect>(),
                                j["b"].get<cpVect>(), j["r"].get<cpFloat>());
    }
    if (j["type"] == "poly") {
      std::size_t count = j["verts"].size();
      std::vector<cpVect> verts(count);
      auto jv = j["verts"];
      for (auto i = 0; i < count; i++) {
        verts[i] = jv[i].get<cpVect>();
      }
      bool raw = false;
      if (j.count("raw") > 0)
        raw = j["raw"].get<bool>();
      cpShape *polyShape;
      if (raw) {
        polyShape =
            cpPolyShapeNewRaw(body, count, &verts[0], j["r"].get<cpFloat>());
      } else {
        polyShape = cpPolyShapeNew(body, count, &verts[0], cpTransformIdentity,
                                   j["r"].get<cpFloat>());
      }

      shape = cpSpaceAddShape(space, polyShape);
      shape->sensor = j["sensor"].get<cpBool>();
      shape->e = j["e"].get<cpFloat>();
      shape->u = j["u"].get<cpFloat>();
      shape->surfaceV = j["surfaceV"].get<cpVect>();
      shape->userData = (cpDataPointer)j["userData"].get<uint64_t>();
      shape->type = j["ctype"].get<cpCollisionType>();
      shape->filter = j["filter"].get<cpShapeFilter>();
      shape->bb = j["bb"].get<cpBB>();
      cpShapeSetCollisionType(shape, shape->type);
    }
    return shape;
  }
};

template <> struct adl_serializer<struct cpBody *> {
  static cpSpace *space;
  static std::map<std::string, cpConstraint *> constraintMap;
  static std::map<std::string, cpBody *> bodyMap;

  static void to_json(json &j, const struct cpBody *body) {
    if (!body) {
      j = nullptr;
    } else {
      j["i"] = body->i;
      j["m"] = body->m;

      // center of gravity
      j["cog"] = body->cog;

      // position, velocity, force
      j["p"] = body->p;
      j["v"] = body->v;
      j["f"] = body->f;

      // Angle, angular velocity, torque (radians)
      j["a"] = body->a;
      j["w"] = body->w;
      j["t"] = body->t;
      j["transform"] = body->transform;
      j["userData"] = (uint64_t)body->userData;
      // "pseudo-velocities" used for eliminating overlap.
      // Erin Catto has some papers that talk about what these are.
      j["v_bias"] = body->v_bias;
      j["w_bias"] = body->w_bias;

      std::vector<cpShape *> shapes;
      // get shapes by walking over the shapeList
      cpShape *it = body->shapeList;
      while (it) {
        shapes.push_back(it);
        it = it->next;
      }
      j["shapes"] = shapes;

      j["ptr"] = ptrToStr(body);
      cpBody *staticBody = cpSpaceGetStaticBody(space);

      std::vector<std::string> constraintPtrs;
      // get shapes by walking over the shapeList
      cpConstraint *itc = body->constraintList;
      while (itc) {
        auto ptr = ptrToStr(itc);
        constraintMap[ptr] = itc;
        constraintPtrs.push_back(ptr);
        if (itc->a == body)
          itc = itc->next_a;
        else
          itc = itc->next_b;
      }
      j["constraintPtrs"] = constraintPtrs;
    }
  }

  static cpBody *from_json(const json &j) {
    cpBody *body;
    if (j.is_null()) {
      body = nullptr;
    } else {
      body = cpSpaceAddBody(
          space, cpBodyNew(j["m"].get<cpFloat>(), j["i"].get<cpFloat>()));

      // place in bodyMap for constraint recovery
      auto ptr = j["ptr"].get<std::string>();
      bodyMap[ptr] = body;

      // center of gravity
      body->cog = j["cog"].get<cpVect>();

      // position, velocity, force
      body->p = j["p"].get<cpVect>();
      body->v = j["v"].get<cpVect>();
      body->f = j["f"].get<cpVect>();

      // Angle, angular velocity, torque (radians)
      body->a = j["a"].get<cpFloat>();
      body->w = j["w"].get<cpFloat>();
      body->t = j["t"].get<cpFloat>();
      body->transform = j["transform"].get<cpTransform>();
      body->userData = (cpDataPointer)j["userData"].get<uint64_t>();

      // "pseudo-velocities" used for eliminating overlap.
      // Erin Catto has some papers that talk about what these are.
      body->v_bias = j["v_bias"].get<cpVect>();
      body->w_bias = j["w_bias"].get<cpFloat>();
      cpJSSetShapeBody(body);
      cpJSSetShapeSpace(space);
      body->shapeList = j["shapes"][0].get<cpShape *>();
      cpShape *it = body->shapeList;
      for (auto i = 1; i < j["shapes"].size(); ++i) {
        cpShape *s = j["shapes"][i].get<cpShape *>();
        it->next = s;
        it = s;
      }
      cpJSSetShapeBody(nullptr);
      cpJSSetShapeSpace(nullptr);
    }
    return body;
  }
};

template <> struct adl_serializer<cpConstraint *> {
  static cpSpace *space;
  static std::map<std::string, cpBody *> *bodyMap;

  static void getConstraintType(json &j, const cpConstraint *ct) {
    if (cpConstraintIsGearJoint(ct)) {
      j["type"] = "GearJoint";
    }
    if (cpConstraintIsPivotJoint(ct)) {
      cpPivotJoint *pv = (cpPivotJoint *)ct;
      j["type"] = "PivotJoint";
      j["anchorA"] = pv->anchorA;
      j["anchorB"] = pv->anchorB;
    }
    if (cpConstraintIsPinJoint(ct)) {
      cpPinJoint *pv = (cpPinJoint *)ct;
      j["type"] = "PinJoint";
      j["anchorA"] = pv->anchorA;
      j["anchorB"] = pv->anchorB;
    }
    if (cpConstraintIsDampedRotarySpring(ct)) {
      cpDampedRotarySpring *drs = (cpDampedRotarySpring *)ct;
      j["restAngle"] = drs->restAngle;
      j["stiffness"] = drs->stiffness;
      j["damping"] = drs->damping;
      j["type"] = "DampedRotarySpring";
    }
    if (cpConstraintIsDampedSpring(ct)) {
      cpDampedSpring *ds = (cpDampedSpring *)ct;
      j["anchorA"] = ds->anchorA;
      j["anchorB"] = ds->anchorB;
      j["restLength"] = ds->restLength;
      j["stiffness"] = ds->stiffness;
      j["damping"] = ds->damping;

      j["type"] = "DampedSpring";
    }
    if (cpConstraintIsGrooveJoint(ct)) {
      j["type"] = "GrooveJoint";
    }
    if (cpConstraintIsRatchetJoint(ct)) {
      j["type"] = "RatchetJoint";
    }
    if (cpConstraintIsRotaryLimitJoint(ct)) {
      j["type"] = "RotaryLimitJoint";
    }
    if (cpConstraintIsSimpleMotor(ct)) {
      j["type"] = "SimpleMotor";
    }
    if (cpConstraintIsSlideJoint(ct)) {
      j["type"] = "SlideJoint";
    }
  }

  static void to_json(json &j, const cpConstraint *ct) {
    if (!ct) {
      j = nullptr;
      return;
    }
    cpBody *staticBody = cpSpaceGetStaticBody(space);

    getConstraintType(j, ct);
    j["ptr"] = ptrToStr(ct);
    j["maxForce"] = ct->maxForce;
    j["errorBias"] = ct->errorBias;
    j["maxBias"] = ct->maxBias;
    j["collideBodies"] = ct->collideBodies;
    j["userData"] = (uint64_t)ct->userData;
    j["a"] = (ct->a == staticBody) ? "static" : ptrToStr(ct->a);
    j["b"] = (ct->b == staticBody) ? "static" : ptrToStr(ct->b);
  }

  static cpConstraint *from_json(const json &j) {
    cpBody *staticBody = cpSpaceGetStaticBody(space);
    if (!bodyMap) {
      std::cerr << "bodyMap not provided , cannot deserialize constraints";
      return nullptr;
    }
    auto &bm = *bodyMap;
    cpBody *a, *b;
    cpConstraint *ct;
    auto as = j["a"].get<std::string>();
    auto bs = j["b"].get<std::string>();
    if (as == "static")
      a = staticBody;
    else
      a = bm[as];
    if (bs == "static")
      b = staticBody;
    else
      b = bm[bs];
    if (!a || !b)
      return nullptr;

    if ("GearJoint" == j["type"]) {

      cpFloat phase = j["phase"].get<cpFloat>();
      cpFloat ratio = j["ratio"].get<cpFloat>();
      ct = cpGearJointNew(a, b, phase, ratio);

    } else if ("PivotJoint" == j["type"]) {

      auto anchorA = j["anchorA"].get<cpVect>();
      auto anchorB = j["anchorB"].get<cpVect>();
      ct = cpPivotJointNew2(a, b, anchorA, anchorB);

    } else if ("PinJoint" == j["type"]) {

      auto anchorA = j["anchorA"].get<cpVect>();
      auto anchorB = j["anchorB"].get<cpVect>();
      ct = cpPinJointNew(a, b, anchorA, anchorB);

    } else if ("DampedRotarySpring" == j["type"]) {

      auto restAngle = j["restAngle"].get<cpFloat>();
      auto stiffness = j["stiffness"].get<cpFloat>();
      auto damping = j["damping"].get<cpFloat>();
      ct = cpDampedRotarySpringNew(a, b, restAngle, stiffness, damping);

    } else if ("DampedSpring" == j["type"]) {

      cpVect anchorA = j["anchorA"].get<cpVect>();
      cpVect anchorB = j["anchorB"].get<cpVect>();
      cpFloat restLength = j["restLength"].get<cpFloat>();
      cpFloat stiffness = j["stiffness"].get<cpFloat>();
      cpFloat damping = j["damping"].get<cpFloat>();

      ct = cpDampedSpringNew(a, b, anchorA, anchorB, restLength, stiffness,
                             damping);

    } else
      return nullptr;
    // TODO add other contraint types
    // if ("GrooveJoint" == j["type"]) {
    //   cpGrooveJointNew();
    // }
    // if ("RatchetJoint" == j["type"]) {
    //   cpRatchetJointNew();
    // }
    // if ("RotaryLimitJoint" == j["type"]) {
    //   cpRotaryLimitJointNew();
    // }
    // if ("SimpleMotor" == j["type"]) {
    //   cpSimpleMotorNew();
    // }
    // if ("SlideJoint" == j["type"]) {
    //   cpSlideJointNew();
    // }

    ct = cpSpaceAddConstraint(space, ct);

    ct->maxForce = j["maxForce"].get<cpFloat>();
    ct->errorBias = j["errorBias"].get<cpFloat>();
    ct->maxBias = j["maxBias"].get<cpFloat>();
    ct->collideBodies = j["collideBodies"].get<cpBool>();
    ct->userData = (cpDataPointer)j["userData"].get<uint64_t>();

    return ct;
  }
};

template <> struct adl_serializer<struct cpSpace *> {
  static void to_json(json &j, const struct cpSpace *space) {
    space = const_cast<cpSpace *>(space);

    std::vector<nlohmann::json> body_vec;
    std::vector<nlohmann::json> constraint_vec;
    nlohmann::adl_serializer<struct cpBody *>::space = (cpSpace *)space;

    {
      cpArray *bodies = space->dynamicBodies;
      for (int i = 0; i < bodies->num; i++) {
        body_vec.push_back((cpBody *)bodies->arr[i]);
      }

      cpArray *otherBodies = space->staticBodies;
      for (int i = 0; i < otherBodies->num; i++) {
        body_vec.push_back((cpBody *)otherBodies->arr[i]);
      }

      cpArray *components = space->sleepingComponents;
      for (int i = 0; i < components->num; i++) {
        cpBody *root = (cpBody *)components->arr[i];

        cpBody *body = root;
        while (body) {
          cpBody *next = body->sleeping.next;
          body_vec.push_back(body);
          body = next;
        }
      }

      nlohmann::adl_serializer<struct cpConstraint *>::space = (cpSpace *)space;
      cpArray *constraints = space->constraints;

      for (int i = 0; i < constraints->num; i++) {
        constraint_vec.push_back((cpConstraint *)constraints->arr[i]);
      }
    }
    j["bodies"] = body_vec;
    nlohmann::adl_serializer<struct cpBody *>::space = nullptr;

    j["constraints"] = constraint_vec;
    nlohmann::adl_serializer<struct cpConstraint *>::space = nullptr;

    j["iterations"] = space->iterations;

    j["gravity"] = space->gravity;
    j["damping"] = space->damping;
    j["idleSpeedThreshold"] = space->idleSpeedThreshold;
    j["sleepTimeThreshold"] = space->sleepTimeThreshold;
    j["collisionSlop"] = space->collisionSlop;
    j["collisionBias"] = space->collisionBias;
    j["collisionPersistence"] = space->collisionPersistence;
    j["userData"] = (uint64_t)space->userData;
  }

  static cpSpace *from_json(const json &j) {
    cpSpace *space = cpSpaceNew();
    space->iterations = j["iterations"].get<int>();
    space->gravity = j["gravity"].get<cpVect>();
    space->damping = j["damping"].get<cpFloat>();
    space->idleSpeedThreshold = j["idleSpeedThreshold"].get<cpFloat>();
    space->sleepTimeThreshold = j["sleepTimeThreshold"].get<cpFloat>();
    space->collisionSlop = j["collisionSlop"].get<cpFloat>();
    space->collisionBias = j["collisionBias"].get<cpFloat>();
    space->collisionPersistence = j["collisionPersistence"].get<cpTimestamp>();
    space->userData = (cpDataPointer)j["userData"].get<uint64_t>();

    nlohmann::adl_serializer<struct cpBody *>::space = space;
    for (auto const &body_val : j["bodies"]) {
      nlohmann::json body = body_val.get<cpBody *>();
    }
    nlohmann::adl_serializer<struct cpBody *>::space = nullptr;

    // constraints come after bodies
    nlohmann::adl_serializer<struct cpConstraint *>::space = space;
    nlohmann::adl_serializer<struct cpConstraint *>::bodyMap =
        &nlohmann::adl_serializer<struct cpBody *>::bodyMap;
    for (auto const &constraint_val : j["contraints"]) {
      nlohmann::json constraint = constraint_val.get<cpConstraint *>();
    }
    nlohmann::adl_serializer<struct cpConstraint *>::space = nullptr;
    nlohmann::adl_serializer<struct cpConstraint *>::bodyMap = nullptr;
  }
};

cpSpace *adl_serializer<struct cpBody *>::space = nullptr;
std::map<std::string, cpConstraint *>
    adl_serializer<struct cpBody *>::constraintMap = {};
std::map<std::string, cpBody *> adl_serializer<struct cpBody *>::bodyMap = {};

cpBody *adl_serializer<struct cpShape *>::body = nullptr;
cpSpace *adl_serializer<struct cpShape *>::space = nullptr;

cpSpace *adl_serializer<struct cpConstraint *>::space = nullptr;
std::map<std::string, cpBody *>
    *adl_serializer<struct cpConstraint *>::bodyMap = nullptr;

} // namespace nlohmann

static void cpJSSetShapeBody(cpBody *body) {
  nlohmann::adl_serializer<struct cpShape *>::body = body;
};

static void cpJSSetBodySpace(cpSpace *space) {
  nlohmann::adl_serializer<struct cpBody *>::space = space;
};

static void cpJSSetShapeSpace(cpSpace *space) {
  nlohmann::adl_serializer<struct cpShape *>::space = space;
};

static void cpJSClearBodyConstraintMap() {
  nlohmann::adl_serializer<struct cpBody *>::constraintMap.clear();
};

static nlohmann::json cpJSBodySerializeConstraints(cpSpace *space) {
  nlohmann::adl_serializer<struct cpConstraint *>::space = space;
  std::vector<nlohmann::json> res;
  for (auto const &[key, val] :
       nlohmann::adl_serializer<struct cpBody *>::constraintMap) {
    nlohmann::json ct = val;
    res.push_back(ct);
  }
  nlohmann::adl_serializer<struct cpConstraint *>::space = nullptr;
  return res;
};

static void cpJSBodyDeserializeConstraints(cpSpace *space,
                                           const nlohmann::json &contraints) {
  nlohmann::adl_serializer<struct cpConstraint *>::space = space;
  nlohmann::adl_serializer<struct cpConstraint *>::bodyMap =
      &nlohmann::adl_serializer<struct cpBody *>::bodyMap;

  for (auto const &val : contraints) {
    std::cout << val.dump() << std::endl;
    nlohmann::json x = val.get<cpConstraint *>();
  }
  nlohmann::adl_serializer<struct cpConstraint *>::space = nullptr;
  nlohmann::adl_serializer<struct cpConstraint *>::bodyMap = nullptr;
};
